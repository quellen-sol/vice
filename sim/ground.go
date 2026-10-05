// sim/ground.go
// Copyright(c) vice contributors, licensed under the GNU Public License, Version 3.
// SPDX: GPL-3.0-only

package sim

import (
	"log/slog"
	"slices"
	"strings"
	"sync"
	"time"

	av "github.com/mmp/vice/aviation"
	"github.com/mmp/vice/math"
	"github.com/mmp/vice/surface"
	"github.com/mmp/vice/util"
)

// This file has the sim's movement of aircraft on the airport surface, at
// airports where a human works the tower and whose surface is known (see
// package surface). Ground control is automated: departures start at a
// gate, taxi to their runway and hold short of it, where they call the
// tower; arrivals roll out after landing, turn off the runway and taxi to
// a gate. The sim moves aircraft on the ground itself, along the taxi
// network, rather than with the nav flight model; it hands departures to
// the flight model when they start their takeoff roll. How aircraft keep
// out of each other's way is in ground_traffic.go.

// GroundPhase is what an aircraft on the ground is doing.
type GroundPhase int

const (
	GroundParked         GroundPhase = iota // at a gate
	GroundTaxiOut                           // a departure taxiing to its runway
	GroundHoldingShort                      // a departure holding short of its runway
	GroundLiningUp                          // a departure taxiing onto its runway
	GroundLinedUp                           // a departure in position on its runway
	GroundRollout                           // an arrival slowing down after landing
	GroundTaxiIn                            // an arrival taxiing to its gate
	GroundHoldingToCross                    // holding short of a runway it needs to cross
)

func (p GroundPhase) moving() bool {
	return p == GroundTaxiOut || p == GroundLiningUp || p == GroundRollout || p == GroundTaxiIn
}

// holding reports whether an aircraft in the phase stays where it is until
// it's cleared to go.
func (p GroundPhase) holding() bool {
	return !p.moving()
}

// GroundEvent is something that happens when an aircraft reaches a point
// along its ground path.
type GroundEvent int

const (
	GroundEventNone          GroundEvent = iota
	GroundEventHoldShort                 // a departure reached its runway's hold-short line
	GroundEventLinedUp                   // a departure is in position on its runway
	GroundEventExitedRunway              // an arrival turned off the runway
	GroundEventClearOfRunway             // an arrival is clear of the runway it landed on
	GroundEventAtGate
	GroundEventCrossingHold // reached the hold-short line of a runway it has to cross
	GroundEventCrossed      // clear of a runway it crossed
)

// GroundPoint is a point along an aircraft's ground path.
type GroundPoint struct {
	P        math.Point2LL
	Node     surface.NodeID // the taxi network node at P; -1 if it isn't one
	MaxSpeed float32        // knots; zero if there is no limit there
	Stop     bool           // the aircraft stops here
	Event    GroundEvent
	Runway   string // for crossing events, the runway (end) crossed
}

// GroundState is the state of an aircraft that the sim is moving on the
// airport surface.
type GroundState struct {
	Phase GroundPhase
	// Path is the rest of the aircraft's path; it is heading for Path[0].
	Path []GroundPoint
	// LastNode is the taxi network node it last passed; -1 if none.
	LastNode surface.NodeID
	Speed    float32 // knots
	Heading  math.TrueHeading
	Gate     string
	Runway   string // the runway end it departs from or landed on
	// Departures: the heading it lines up on and whether it's cleared for
	// takeoff (in which case it rolls once lined up).
	LineupHeading  math.TrueHeading
	TakeoffCleared bool
	// Arrivals: how far it floats down the runway before braking, and when
	// it reached its gate.
	FloatDistance float32 // nm
	ParkedTime    Time
	// Holding short of a runway to cross it: the runway, whether the pilot
	// has asked to cross, and what it was doing before it stopped.
	CrossRunway       string
	RequestedCrossing bool
	ResumePhase       GroundPhase
	// GivingWay is the aircraft it's waiting for to go by, if any.
	GivingWay av.ADSBCallsign
	// ExitReplanned records that an arrival found its exit blocked and
	// looked for another.
	ExitReplanned bool
	// TaxiStartTime is when a departure was ready to leave its gate.
	TaxiStartTime Time
	// Version is groundStateVersion as of when it was made.
	Version int
}

// groundStateVersion is the version of GroundState; restoring a sim
// brings states saved by earlier versions up to date.
const groundStateVersion = 1

// Ground movement parameters.
const (
	taxiSpeedJet       = 20 // knots
	taxiSpeedOther     = 15
	turnSpeed          = 10 // for turns sharper than turnSpeedAngle
	turnSpeedAngle     = 35 // degrees
	lineupSpeed        = 8
	gateSpeed          = 6
	highSpeedExitSpeed = 40
	exitSpeed          = 15
	taxiAccel          = 2 // knots per second
	taxiDecel          = 3
	touchdownFloat     = 0.15 // nm past the threshold that braking starts
	maxHeadingRate     = 45   // degrees per second
	// Aircraft taxiing stop this far behind one ahead of them.
	followDistance = 0.03 // nm, about 180'
	// Aircraft whose paths come closer than this are in each other's way.
	conflictDistance = 0.03 // nm
	// How far ahead, in seconds, aircraft look for others in their way.
	groundLookahead = 20
	// The speed at which an aircraft that's stopped while taxiing is
	// expected to move off.
	creepSpeed = 8 // knots
	// The extra cost of a route along a taxiway that other traffic is
	// using the opposite way.
	opposedTaxiwayCost = 0.5 // nm
	// The longest the ramp holds a departure at its gate for traffic.
	maxGateHold = 3 * time.Minute
	// Arrivals are removed this long after reaching their gate.
	parkedTime = 90 * time.Second
)

// rolloutDecel returns how quickly the aircraft slows after touchdown, in
// knots per second.
func rolloutDecel(ac *Aircraft) float32 {
	switch ac.Nav.Perf.Engine.AircraftType {
	case "J":
		return 3.5
	case "T":
		return 4
	default:
		return 5
	}
}

func taxiSpeed(ac *Aircraft) float32 {
	if ac.Nav.Perf.Engine.AircraftType == "J" {
		return taxiSpeedJet
	}
	return taxiSpeedOther
}

///////////////////////////////////////////////////////////////////////////
// Surface data

var surfaceCache struct {
	sync.Mutex
	airports map[av.ICAOAirportCode]*surface.Airport
}

// surfaceAirport returns the airport's surface data, if it's available.
func surfaceAirport(icao av.ICAOAirportCode) (*surface.Airport, bool) {
	surfaceCache.Lock()
	defer surfaceCache.Unlock()

	if ap, ok := surfaceCache.airports[icao]; ok {
		return ap, ap != nil
	}
	if surfaceCache.airports == nil {
		surfaceCache.airports = make(map[av.ICAOAirportCode]*surface.Airport)
	}
	var ap *surface.Airport
	if surface.HasAirport(string(icao)) {
		var err error
		if ap, err = surface.LoadAirport(string(icao)); err != nil {
			ap = nil
		}
	}
	surfaceCache.airports[icao] = ap
	return ap, ap != nil
}

// groundMovementAirport returns the airport's surface data if the sim moves
// aircraft on its surface: a human works its tower and its surface is
// known.
func (s *Sim) groundMovementAirport(icao av.ICAOAirportCode) (*surface.Airport, bool) {
	if !s.hasHumanTower(icao) {
		return nil, false
	}
	return surfaceAirport(icao)
}

// activeRunwayEnds returns the ends of the airport's runways that are in use
// for arrivals or departures, along with their opposite ends.
func (s *Sim) activeRunwayEnds(ap *surface.Airport) []string {
	var ends []string
	add := func(icao av.ICAOAirportCode, rwy av.RunwayID) {
		if string(icao) != ap.ICAO {
			return
		}
		if r, _, ok := ap.LookupRunway(rwy.Base()); ok {
			for _, e := range r.Ends {
				if !slices.Contains(ends, e.Id) {
					ends = append(ends, e.Id)
				}
			}
		}
	}
	for _, ar := range s.State.ArrivalRunways {
		add(ar.Airport, ar.Runway)
	}
	for _, dr := range s.State.DepartureRunways {
		add(dr.Airport, dr.Runway)
	}
	return ends
}

// activeCrossing returns an end of an active runway other than the ones in
// except whose protected area the aircraft enters past the hold-short
// point, if there is one.
func activeCrossing(hs surface.HoldShort, active, except []string) (string, bool) {
	for _, end := range hs.RunwayEnds {
		if slices.Contains(active, end) && !slices.Contains(except, end) {
			return end, true
		}
	}
	return "", false
}

// countCrossings returns how many active runways other than the ones in
// except the route crosses.
func countCrossings(r surface.Route, active, except []string) int {
	n := 0
	for _, hs := range r.HoldShorts {
		if _, ok := activeCrossing(hs, active, except); ok {
			n++
		}
	}
	return n
}

// markCrossings marks the route's hold-short points for active runways
// other than the ones in except, where the aircraft stops until the tower
// clears it across, and the points where it's across them. path[i+offset]
// is r.Nodes[i].
func markCrossings(ap *surface.Airport, r surface.Route, path []GroundPoint, offset int, active, except []string) {
	for _, hs := range r.HoldShorts {
		rwy, ok := activeCrossing(hs, active, except)
		if !ok {
			continue
		}
		pt := &path[hs.Index+offset]
		pt.Stop, pt.Event, pt.Runway, pt.MaxSpeed = true, GroundEventCrossingHold, rwy, 0

		// It's across once it leaves the runway's protected area.
		for i := hs.Index + 1; i < len(r.Edges); i++ {
			if !slices.Contains(ap.Edges[r.Edges[i]].ProtectedRunwayEnds(false), rwy) {
				if path[i+offset].Event == GroundEventNone {
					path[i+offset].Event, path[i+offset].Runway = GroundEventCrossed, rwy
				}
				break
			}
		}
	}
}

///////////////////////////////////////////////////////////////////////////
// Gates

// gateScore returns how well the gate suits the aircraft; negative if it
// can't use it at all.
func gateScore(g surface.Gate, ac *Aircraft) int {
	class := map[string]string{"J": "jets", "T": "turboprops", "P": "props", "H": "helos"}[ac.Nav.Perf.Engine.AircraftType]
	if class == "" {
		class = "jets"
	}
	if len(g.Types) > 0 && !slices.Contains(g.Types, class) && !slices.Contains(g.Types, "all") {
		return -1
	}

	airline, _ := av.SplitCallsign(string(ac.ADSBCallsign))
	ga := airline == "N"
	if ga && g.Kind == "gate" {
		return -1 // general aviation doesn't use the terminal
	}
	if len(g.Airlines) > 0 && !slices.Contains(g.Airlines, airline) &&
		!slices.ContainsFunc(g.Airlines, func(a string) bool { return !slices.Contains(cargoAirlines, a) }) {
		return -1 // a cargo ramp
	}

	// Airliners prefer their airline's gates, then the terminal's other
	// gates (they're often common use), then ramps; general aviation
	// prefers ramps and tie-downs.
	score := 2
	switch {
	case ga && (g.Kind == "tie_down" || g.Kind == "misc" || g.Kind == "hangar"):
		score += 2
	case !ga && g.Kind == "gate":
		score += 2
	}
	if len(g.Airlines) > 0 {
		if slices.Contains(g.Airlines, airline) {
			score += 3
		} else {
			score--
		}
	}
	return score
}

// cargoAirlines are the cargo carriers whose gates aren't used by others.
var cargoAirlines = []string{"ABX", "ATN", "CKS", "FDX", "GTI", "PAC", "UPS"}

// gateOccupied reports whether an aircraft is at the gate or headed for it,
// or still on its way out of it.
func (s *Sim) gateOccupied(ap *surface.Airport, name string) bool {
	for _, ac := range s.Aircraft {
		if g := ac.Ground; g != nil && g.Gate == name && string(groundAirport(ac)) == ap.ICAO {
			return true
		}
	}
	return false
}

// gateBlocked reports whether an arrival can't be sent to the gate because
// another aircraft is on its way into or out of a gate that leaves the gate
// area at the same spot: one aircraft at a time.
func (s *Sim) gateBlocked(ap *surface.Airport, gate surface.Gate) bool {
	n, ok := ap.GateNode(gate)
	if !ok {
		return false
	}
	for _, ac := range s.Aircraft {
		g := ac.Ground
		if g == nil || g.Phase == GroundParked || string(groundAirport(ac)) != ap.ICAO {
			continue
		}
		if on, ok := gateNodeByName(ap, g.Gate); ok && on == n {
			return true
		}
	}
	return false
}

// groundAirport returns the airport on whose surface the aircraft is.
func groundAirport(ac *Aircraft) av.ICAOAirportCode {
	if ac.IsDeparture() {
		return ac.DepartureAirport
	}
	return ac.ArrivalAirport
}

// scoredGate is a gate and how well it suits an aircraft.
type scoredGate struct {
	gate  surface.Gate
	score int
}

// candidateGates returns the free gates the aircraft can use, best first.
// Arrivals use gates that are blocked for the time being only if there's
// nothing else.
func (s *Sim) candidateGates(ap *surface.Airport, ac *Aircraft) []scoredGate {
	var gates []scoredGate
	for _, g := range ap.Gates {
		if sc := gateScore(g, ac); sc >= 0 && !s.gateOccupied(ap, g.Name) {
			if !ac.IsDeparture() && s.gateBlocked(ap, g) {
				sc -= 100
			}
			gates = append(gates, scoredGate{g, sc})
		}
	}
	// Shuffle before the stable sort so that equally good gates are used
	// in random order.
	for i := len(gates) - 1; i > 0; i-- {
		j := s.Rand.Intn(i + 1)
		gates[i], gates[j] = gates[j], gates[i]
	}
	slices.SortStableFunc(gates, func(a, b scoredGate) int { return b.score - a.score })
	return gates
}

///////////////////////////////////////////////////////////////////////////
// Paths

// routePath returns the points of a taxi route, slowing for sharp turns.
// The route's first node is not included.
func routePath(ap *surface.Airport, r surface.Route) []GroundPoint {
	var path []GroundPoint
	for i := 1; i < len(r.Nodes); i++ {
		pt := GroundPoint{P: ap.Nodes[r.Nodes[i]].Location, Node: r.Nodes[i]}
		if i+1 < len(r.Nodes) {
			pt.MaxSpeed = turnLimit(ap.Local(ap.Nodes[r.Nodes[i-1]].Location), ap.Local(pt.P),
				ap.Local(ap.Nodes[r.Nodes[i+1]].Location))
		}
		path = append(path, pt)
	}
	return path
}

// turnLimit returns the speed limit at p1 for a path that goes p0, p1, p2:
// turnSpeed if the turn there is sharp, otherwise zero (no limit).
func turnLimit(p0, p1, p2 [2]float32) float32 {
	a, b := math.Sub2f(p1, p0), math.Sub2f(p2, p1)
	if math.Length2f(a) < 1e-5 || math.Length2f(b) < 1e-5 {
		return 0
	}
	if math.HeadingDifference(math.VectorHeading(a), math.VectorHeading(b)) > turnSpeedAngle {
		return turnSpeed
	}
	return 0
}

// departureGround plans the departure's ground movement from a gate to its
// runway, if the sim moves aircraft on the airport's surface. The departure
// is moved to the gate.
func (s *Sim) departureGround(ac *Aircraft, runway av.RunwayID) {
	ap, ok := s.groundMovementAirport(ac.DepartureAirport)
	if !ok {
		return
	}
	end := runway.Base()
	rwy, idx, ok := ap.LookupRunway(end)
	if !ok {
		return
	}
	entry, ok := ap.DepartureEntry(end)
	if !ok {
		return
	}
	active := s.activeRunwayEnds(ap)
	own := []string{rwy.Ends[0].Id, rwy.Ends[1].Id}

	// Take the best gate whose route doesn't cross an active runway or,
	// failing that, the first with a route.
	var gate surface.Gate
	var route surface.Route
	found := false
	for _, sg := range s.candidateGates(ap, ac) {
		g := sg.gate
		gateNode, ok := ap.GateNode(g)
		if !ok {
			continue
		}
		r, err := ap.FindRoute(surface.RouteRequest{From: gateNode, To: []surface.NodeID{entry}})
		if err != nil || len(r.HoldShorts) == 0 {
			continue
		}
		if crossings := countCrossings(r, active, own); !found || crossings == 0 {
			gate, route, found = g, r, true
			if crossings == 0 {
				break
			}
		}
	}

	if found {
		g := gate
		lineupHeading := math.VectorHeading(math.Sub2f(ap.Local(rwy.Ends[1-idx].Threshold), ap.Local(rwy.Ends[idx].Threshold)))
		ac.Ground = &GroundState{
			Version:       groundStateVersion,
			Phase:         GroundParked,
			Path:          taxiOutPath(ap, route, active, own),
			LastNode:      -1,
			Heading:       g.Heading,
			Gate:          g.Name,
			Runway:        end,
			LineupHeading: lineupHeading,
		}
		s.setGroundFlightState(ac, ap, ap.Local(g.Location))
		// Ground control is automated; the departure calls the tower
		// once it reaches the runway.
		ac.ControllerFrequency = ""
		return
	}
	s.lg.Warn("no gate for departure", slog.String("callsign", string(ac.ADSBCallsign)),
		slog.String("runway", end))
}

// taxiOutPath returns the path of a departure taxiing along r from its gate
// to its runway: it holds short at the route's last hold-short point, the
// one for its runway, and at those of the active runways it crosses on the
// way, then lines up at the runway entry, where the route ends.
func taxiOutPath(ap *surface.Airport, r surface.Route, active, own []string) []GroundPoint {
	// path[i] is r.Nodes[i].
	path := append([]GroundPoint{{P: ap.Nodes[r.Nodes[0]].Location, Node: r.Nodes[0], MaxSpeed: gateSpeed}},
		routePath(ap, r)...)
	markCrossings(ap, r, path, 0, active, own)
	hs := r.HoldShorts[len(r.HoldShorts)-1]
	path[hs.Index].Stop, path[hs.Index].Event, path[hs.Index].MaxSpeed = true, GroundEventHoldShort, 0
	path[len(path)-1].MaxSpeed = lineupSpeed
	path[len(path)-1].Stop, path[len(path)-1].Event = true, GroundEventLinedUp
	return path
}

// gateNodeByName returns the taxiway node where the named gate leaves the
// gate area.
func gateNodeByName(ap *surface.Airport, name string) (surface.NodeID, bool) {
	if name == "" {
		return -1, false
	}
	if i := slices.IndexFunc(ap.Gates, func(g surface.Gate) bool { return g.Name == name }); i != -1 {
		return ap.GateNode(ap.Gates[i])
	}
	return -1, false
}

// startLandingRollout starts an arrival's rollout after touchdown if the sim
// moves aircraft on the airport's surface, choosing a gate and the exit to
// take to it. It returns false if the aircraft should be removed instead.
func (s *Sim) startLandingRollout(ac *Aircraft) bool {
	appr := ac.Nav.Approach.Assigned
	ap, ok := s.groundMovementAirport(ac.ArrivalAirport)
	if !ok || appr == nil {
		return false
	}
	end := appr.Runway
	rwy, idx, ok := ap.LookupRunway(end)
	if !ok {
		return false
	}
	origin := ap.Local(rwy.Ends[idx].Threshold)
	dir := math.Normalize2f(math.Sub2f(ap.Local(rwy.Ends[1-idx].Threshold), origin))
	pos := ap.Local(ac.Position())
	along := math.Dot(math.Sub2f(pos, origin), dir)

	speed := max(ac.Nav.FlightState.GS, 60)
	decel := rolloutDecel(ac)
	active := s.activeRunwayEnds(ap)
	ownEnds := []string{rwy.Ends[0].Id, rwy.Ends[1].Id}

	// The exits ahead it can slow down for, nearest first; if it can't slow
	// for any, the farthest one.
	exitSpeedFor := func(exit surface.RunwayConnection) float32 {
		if exit.Angle <= 45 {
			return highSpeedExitSpeed
		}
		return exitSpeed
	}
	var exits, ahead []surface.RunwayConnection
	for _, exit := range ap.RunwayConnections(end) {
		if exit.Angle > 100 || exit.Distance < along+touchdownFloat {
			continue // behind it or a turn back the way it came
		}
		ahead = append(ahead, exit)
		vexit := exitSpeedFor(exit)
		if exit.Distance >= along+touchdownFloat+(speed*speed-vexit*vexit)/(2*decel*3600) {
			exits = append(exits, exit)
		}
	}
	if len(exits) == 0 && len(ahead) > 0 {
		exits = ahead[len(ahead)-1:]
	}
	// Some pilots roll long and pass up the first one.
	if len(exits) > 1 && s.Rand.Float32() < 0.25 {
		exits = append(exits[1:], exits[0])
	}

	// Take the first exit whose route to one of the gates doesn't cross an
	// active runway or meet traffic coming the other way, or failing that,
	// the first that does the least of that; the route goes to the nearest
	// of the gates.
	use := s.taxiwayUse(ap, ac, nil)
	edgeCost := opposedEdgeCost(ap, use)
	taxiTo := func(gates []surface.Gate) bool {
		gateNodes := make(map[surface.NodeID][]surface.Gate)
		var targets []surface.NodeID
		for _, g := range gates {
			if n, ok := ap.GateNode(g); ok {
				if _, seen := gateNodes[n]; !seen {
					targets = append(targets, n)
				}
				gateNodes[n] = append(gateNodes[n], g)
			}
		}
		if len(targets) == 0 {
			return false
		}

		var bestExit surface.RunwayConnection
		var bestRoute surface.Route
		bestRank := -1
		for _, exit := range exits {
			off := ap.Edges[exit.Edge].Other(exit.Node)
			r, err := ap.FindRoute(surface.RouteRequest{From: off, To: targets, EdgeCost: edgeCost})
			if err != nil {
				continue
			}
			rank := 0
			if countCrossings(r, active, ownEnds) > 0 {
				rank += 2
			}
			if use[[2]surface.NodeID{off, exit.Node}] || routeOpposed(r, use) || s.groundTrafficNear(ap, ac, ap.Nodes[off].Location) {
				rank++
			}
			if s.routeBlocked(ap, ac, exit.Node, r) {
				rank += 4
			}
			if bestRank == -1 || rank < bestRank {
				bestExit, bestRoute, bestRank = exit, r, rank
			}
			if rank == 0 {
				break
			}
		}
		if bestRank == -1 {
			return false
		}
		s.planRollout(ac, ap, bestExit, bestRoute, exitSpeedFor(bestExit), gateNodes, dir, pos, speed, end,
			active, ownEnds)
		return true
	}

	// Go to the best gates it can reach: an airline's own before the
	// terminal's others, and so forth.
	gates := s.candidateGates(ap, ac)
	for len(gates) > 0 {
		n := 1
		for n < len(gates) && gates[n].score == gates[0].score {
			n++
		}
		if taxiTo(util.MapSlice(gates[:n], func(sg scoredGate) surface.Gate { return sg.gate })) {
			return true
		}
		gates = gates[n:]
	}
	s.lg.Warn("no runway exit", slog.String("callsign", string(ac.ADSBCallsign)), slog.String("runway", end))
	return false
}

// replanRollout has an arrival rolling out take a different exit, if
// there's one ahead that suits it better, returning true if it does.
func (s *Sim) replanRollout(ac *Aircraft) bool {
	old := ac.Ground
	ac.Ground = nil // (so that its gate is free for it)
	if !s.startLandingRollout(ac) || ac.Ground.Path[0].Node == old.Path[0].Node {
		ac.Ground = old
		return false
	}
	g := ac.Ground
	g.Speed, g.FloatDistance, g.ExitReplanned = old.Speed, 0, true
	return true
}

// planRollout sets up an arrival's rollout from its current position, its
// exit off the runway and its taxi to a gate along the route from the exit.
func (s *Sim) planRollout(ac *Aircraft, ap *surface.Airport, exit surface.RunwayConnection, r surface.Route,
	vexit float32, gateNodes map[surface.NodeID][]surface.Gate, dir, pos [2]float32, speed float32, end string,
	active, ownEnds []string) {
	off := r.Nodes[0]
	gate := gateNodes[r.Nodes[len(r.Nodes)-1]][0]

	path := []GroundPoint{
		{P: ap.Nodes[exit.Node].Location, Node: exit.Node, MaxSpeed: vexit},
		{P: ap.Nodes[off].Location, Node: off, MaxSpeed: exitSpeed, Event: GroundEventExitedRunway},
	}
	taxi := routePath(ap, r)
	// It's clear of the runway once it leaves the runway's protected
	// area.
	clear := len(r.Edges)
	for i, ei := range r.Edges {
		prot := ap.Edges[ei].ProtectedRunwayEnds(false)
		if !slices.Contains(prot, ownEnds[0]) && !slices.Contains(prot, ownEnds[1]) {
			clear = i
			break
		}
	}
	if clear == 0 {
		path[1].Event = GroundEventClearOfRunway // which also ends the rollout
	} else if clear-1 < len(taxi) {
		taxi[clear-1].Event = GroundEventClearOfRunway
	}
	path = append(path, taxi...)
	// path[i+1] is r.Nodes[i].
	markCrossings(ap, r, path, 1, active, ownEnds)
	path = append(path, GroundPoint{P: gate.Location, Node: -1, MaxSpeed: gateSpeed, Stop: true, Event: GroundEventAtGate})

	ac.Ground = &GroundState{
		Version:       groundStateVersion,
		Phase:         GroundRollout,
		Path:          path,
		LastNode:      -1,
		Speed:         speed,
		Heading:       math.VectorHeading(dir),
		Gate:          gate.Name,
		Runway:        end,
		FloatDistance: touchdownFloat,
	}
	s.setGroundFlightState(ac, ap, pos)
}

// restoreGroundStates brings the ground movement of a restored sim's
// aircraft up to date: those saved before ground paths recorded the
// taxiway nodes along them get them from where the points are.
func (s *Sim) restoreGroundStates() {
	for _, ac := range s.Aircraft {
		g := ac.Ground
		if g == nil || g.Version >= groundStateVersion {
			continue
		}
		g.Version, g.LastNode = groundStateVersion, -1
		ap, ok := surfaceAirport(groundAirport(ac))
		for i := range g.Path {
			g.Path[i].Node = -1
			if !ok {
				continue
			}
			for n, node := range ap.Nodes {
				if ap.Distance(node.Location, g.Path[i].P) < 0.001 {
					g.Path[i].Node = surface.NodeID(n)
					break
				}
			}
		}
	}
}

///////////////////////////////////////////////////////////////////////////
// Movement

// setGroundFlightState sets the aircraft's flight state to its position and
// motion on the ground, so that it's reported correctly.
func (s *Sim) setGroundFlightState(ac *Aircraft, ap *surface.Airport, p [2]float32) {
	g := ac.Ground
	fs := &ac.Nav.FlightState
	fs.Position = ap.FromLocal(p)
	fs.Heading = math.TrueToMagnetic(g.Heading, s.State.MagneticVariation)
	fs.IAS, fs.GS = g.Speed, g.Speed
	fs.PrevAltitude, fs.Altitude = ap.ElevationFt, ap.ElevationFt
	fs.AltitudeRate, fs.BankAngle = 0, 0
}

// updateGround moves an aircraft on the ground for one second. It returns
// true if the aircraft was removed.
func (s *Sim) updateGround(ac *Aircraft) bool {
	g := ac.Ground
	ap, ok := surfaceAirport(groundAirport(ac))
	if !ok {
		// Shouldn't happen; let the aircraft go.
		ac.Ground = nil
		return false
	}
	pos := ap.Local(ac.Position())

	switch g.Phase {
	case GroundParked:
		if !g.ParkedTime.IsZero() && s.State.SimTime.Sub(g.ParkedTime) > parkedTime {
			s.deleteAircraft(ac, DeleteLanded)
			return true
		}
		return false
	case GroundHoldingShort:
		if !ac.ReadyForDeparture && !s.prespawn {
			s.callReadyForDeparture(ac)
		}
		return false
	case GroundLinedUp:
		return false
	case GroundHoldingToCross:
		if !g.RequestedCrossing && !s.prespawn {
			s.requestCrossing(ac)
		}
		return false
	}

	// Find the fastest it can go given the stops and speed limits ahead:
	// the speed from which it can slow to each in time.
	decel := float32(taxiDecel)
	target := taxiSpeed(ac)
	switch g.Phase {
	case GroundRollout:
		decel, target = rolloutDecel(ac), g.Speed
	case GroundLiningUp:
		target = lineupSpeed
	}
	d, prev := float32(0), pos
	for _, pt := range g.Path {
		p := ap.Local(pt.P)
		d += math.Distance2f(prev, p)
		prev = p
		limit := pt.MaxSpeed
		if pt.Stop {
			limit = 0
		} else if limit == 0 {
			continue
		}
		target = min(target, math.Sqrt(limit*limit+2*decel*3600*d))
		if pt.Stop || d > 1 {
			break
		}
	}
	limit := s.groundTrafficLimit(ac, ap, pos, decel)
	if g.Phase == GroundRollout && limit < g.Speed-decel && !g.ExitReplanned {
		// Its exit's blocked; it takes a later one if it can.
		g.ExitReplanned = true
		if s.replanRollout(ac) {
			return false
		}
	}
	target = min(target, limit)
	if leavingGate(ac) && g.Speed == 0 && s.heldAtGate(ac, ap) {
		target = 0
	}

	switch {
	case g.Phase == GroundRollout && g.FloatDistance > 0:
		// Still floating to touchdown.
		g.FloatDistance = max(0, g.FloatDistance-g.Speed/3600)
	case g.Phase == GroundRollout:
		// Brake steadily to the exit's speed.
		exit := float32(exitSpeed)
		if len(g.Path) > 0 && g.Path[0].MaxSpeed > 0 {
			exit = g.Path[0].MaxSpeed
		}
		g.Speed = min(target, max(exit, g.Speed-decel))
	case g.Speed > target:
		g.Speed = max(target, g.Speed-max(decel, taxiDecel))
	default:
		g.Speed = min(target, g.Speed+taxiAccel)
	}

	// Move along the path, handling the points reached.
	dist := g.Speed / 3600
	for dist > 0 && len(g.Path) > 0 {
		p := ap.Local(g.Path[0].P)
		seg := math.Sub2f(p, pos)
		l := math.Length2f(seg)
		if l > 1e-6 {
			g.Heading = turnToward(g.Heading, math.VectorHeading(seg))
		}
		if l > dist {
			pos = math.Add2f(pos, math.Scale2f(seg, dist/l))
			break
		}
		pos, dist = p, dist-l
		pt := g.Path[0]
		g.Path = g.Path[1:]
		if pt.Node >= 0 {
			g.LastNode = pt.Node
			if g.Phase == GroundTaxiOut {
				g.Gate = "" // out of the gate
			}
		}
		if pt.Stop {
			g.Speed, dist = 0, 0
		}
		if s.groundEvent(ac, ap, pt, pos) {
			return true
		}
	}
	if ac.Ground != nil {
		s.setGroundFlightState(ac, ap, pos)
	}
	return false
}

// turnToward turns heading toward target, at most maxHeadingRate degrees.
func turnToward(heading, target math.TrueHeading) math.TrueHeading {
	turn := math.HeadingSignedTurn(heading, target)
	turn = math.Clamp(turn, -maxHeadingRate, maxHeadingRate)
	return math.TrueHeading(math.NormalizeHeading(float32(heading) + turn))
}

// groundEvent handles an aircraft reaching a point along its path. It
// returns true if the aircraft was removed.
func (s *Sim) groundEvent(ac *Aircraft, ap *surface.Airport, pt GroundPoint, pos [2]float32) bool {
	g := ac.Ground
	switch pt.Event {
	case GroundEventHoldShort:
		g.Phase = GroundHoldingShort
		if !s.prespawn {
			s.callReadyForDeparture(ac)
		}
	case GroundEventLinedUp:
		g.Phase, g.Heading = GroundLinedUp, g.LineupHeading
		if g.TakeoffCleared {
			s.setGroundFlightState(ac, ap, pos)
			s.beginTakeoffRoll(ac)
		}
	case GroundEventExitedRunway:
		g.Phase = GroundTaxiIn
	case GroundEventClearOfRunway:
		// The exit may also be where it's clear of the runway.
		if g.Phase == GroundRollout {
			g.Phase = GroundTaxiIn
		}
		// Ground control is automated; the pilot leaves the tower's
		// frequency.
		if s.isTowerPosition(ac.ControllerFrequency, ac.ArrivalAirport) ||
			s.onTowerFrequency(ac, ac.ArrivalAirport) {
			s.setControllerFrequency(ac, "")
		}
	case GroundEventAtGate:
		g.Phase, g.ParkedTime = GroundParked, s.State.SimTime
	case GroundEventCrossingHold:
		g.ResumePhase, g.Phase = g.Phase, GroundHoldingToCross
		g.CrossRunway, g.RequestedCrossing = pt.Runway, false
		if !s.prespawn {
			s.requestCrossing(ac)
		}
	case GroundEventCrossed:
		// An arrival goes back to ground control; a departure stays with
		// the tower, whose runway it's heading for.
		if ac.IsArrival() && s.onTowerFrequency(ac, ac.ArrivalAirport) {
			s.setControllerFrequency(ac, "")
		}
	}
	return false
}

// requestCrossing has an aircraft holding short of a runway it has to cross
// switch to the tower and ask to cross.
func (s *Sim) requestCrossing(ac *Aircraft) {
	g := ac.Ground
	tower, ok := s.humanTowerPosition(groundAirport(ac), g.CrossRunway)
	if !ok {
		return
	}
	g.RequestedCrossing = true
	if !s.onTowerFrequency(ac, groundAirport(ac)) {
		s.setControllerFrequency(ac, tower)
	}
	s.enqueuePilotTransmission(ac.ADSBCallsign, TCP(ac.ControllerFrequency), PendingTransmissionRequestCrossing)
}

// crossRunway clears an aircraft holding short of a runway across it.
func (s *Sim) crossRunway(ac *Aircraft) (string, bool) {
	g := ac.Ground
	if g == nil || g.Phase != GroundHoldingToCross {
		return "", false
	}
	rwy := g.CrossRunway
	g.Phase, g.CrossRunway, g.RequestedCrossing = g.ResumePhase, "", false
	return rwy, true
}

// callReadyForDeparture has a departure holding short of its runway switch
// to the tower and call it ready.
func (s *Sim) callReadyForDeparture(ac *Aircraft) {
	tower, ok := s.humanTowerPosition(ac.DepartureAirport, ac.Ground.Runway)
	if !ok {
		return
	}
	ac.ReadyForDeparture = true
	s.setControllerFrequency(ac, tower)
	s.enqueuePilotTransmission(ac.ADSBCallsign, tower, PendingTransmissionReadyForDeparture)
}

// groundLineUp sends a departure holding short onto its runway.
func (s *Sim) groundLineUp(ac *Aircraft) {
	if g := ac.Ground; g.Phase == GroundHoldingShort || g.Phase == GroundTaxiOut {
		g.Phase = GroundLiningUp
	}
}

// groundClearForTakeoff clears a departure moving on the ground for
// takeoff: it lines up if it hasn't and starts its takeoff roll once it has.
func (s *Sim) groundClearForTakeoff(ac *Aircraft) {
	g := ac.Ground
	g.TakeoffCleared = true
	switch g.Phase {
	case GroundLinedUp:
		s.beginTakeoffRoll(ac)
	default:
		s.groundLineUp(ac)
	}
}

// beginTakeoffRoll starts a departure lined up on its runway down it,
// handing it from the ground movement to the flight model.
func (s *Sim) beginTakeoffRoll(ac *Aircraft) {
	g := ac.Ground
	ac.Ground = nil
	ac.ReadyForDeparture, ac.LinedUp = false, false

	// The departure's route starts at the runway threshold; skip whatever
	// of it is behind the aircraft, which may have lined up past it.
	hv := math.HeadingVector(g.LineupHeading)
	nmPerLongitude := ac.Nav.FlightState.NmPerLongitude
	for len(ac.Nav.Waypoints) > 1 {
		v := math.LL2NM(math.Sub2LL(ac.Nav.Waypoints[0].Location, ac.Position()), nmPerLongitude)
		if math.Dot(v, hv) > 0.02 {
			break
		}
		ac.Nav.Waypoints = ac.Nav.Waypoints[1:]
	}

	for runway, depState := range util.SortedMap(s.DepartureState[ac.DepartureAirport]) {
		for _, queue := range []*[]DepartureAircraft{&depState.ReleasedIFR, &depState.ReleasedVFR} {
			if idx := slices.IndexFunc(*queue, func(d DepartureAircraft) bool {
				return d.ADSBCallsign == ac.ADSBCallsign
			}); idx != -1 {
				dep := (*queue)[idx]
				*queue = util.DeleteSliceElement(*queue, idx)
				s.startTakeoffRoll(depState, ac.DepartureAirport, runway, dep, s.State.SimTime)
				return
			}
		}
	}
	// Not in a queue (e.g., launched by hand); just go.
	ac.WaitingForLaunch = false
}

// startTaxiOut starts a departure parked at its gate taxiing to its runway;
// the gate's free once it's out on the taxiway. It takes the route that keeps it out of the way of
// the other traffic, if there's one that doesn't cross more runways.
func (s *Sim) startTaxiOut(ac *Aircraft) {
	g := ac.Ground
	if g == nil || g.Phase != GroundParked || !g.ParkedTime.IsZero() {
		return
	}
	g.Phase, g.TaxiStartTime = GroundTaxiOut, s.State.SimTime

	ap, ok := surfaceAirport(ac.DepartureAirport)
	if !ok || len(g.Path) == 0 || g.Path[0].Node < 0 {
		return
	}
	rwy, _, ok := ap.LookupRunway(g.Runway)
	if !ok {
		return
	}
	entry, ok := ap.DepartureEntry(g.Runway)
	if !ok {
		return
	}
	active := s.activeRunwayEnds(ap)
	own := []string{rwy.Ends[0].Id, rwy.Ends[1].Id}
	r, err := ap.FindRoute(surface.RouteRequest{
		From:     g.Path[0].Node,
		To:       []surface.NodeID{entry},
		EdgeCost: opposedEdgeCost(ap, s.taxiwayUse(ap, ac, nil)),
	})
	if err != nil || len(r.HoldShorts) == 0 {
		return
	}
	crossings := 0
	for _, pt := range g.Path {
		if pt.Event == GroundEventCrossingHold {
			crossings++
		}
	}
	if countCrossings(r, active, own) <= crossings {
		g.Path = taxiOutPath(ap, r, active, own)
	}
}

// groundStatus describes an aircraft's ground movement for logging.
func (g *GroundState) String() string {
	names := []string{"parked", "taxi out", "holding short", "lining up", "lined up", "rollout", "taxi in",
		"holding short to cross"}
	var b strings.Builder
	b.WriteString(names[g.Phase])
	if g.Gate != "" {
		b.WriteString(" gate " + g.Gate)
	}
	return b.String()
}
