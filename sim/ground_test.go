// sim/ground_test.go
// Copyright(c) vice contributors, licensed under the GNU Public License, Version 3.
// SPDX: GPL-3.0-only

package sim

import (
	"slices"
	"testing"
	"time"

	av "github.com/mmp/vice/aviation"
	"github.com/mmp/vice/math"
	"github.com/mmp/vice/nav"
	"github.com/mmp/vice/surface"
)

func TestGateScore(t *testing.T) {
	jet := func(callsign av.ADSBCallsign) *Aircraft {
		ac := &Aircraft{ADSBCallsign: callsign}
		ac.Nav.Perf.Engine.AircraftType = "J"
		return ac
	}
	props := &Aircraft{ADSBCallsign: "N123AB"}
	props.Nav.Perf.Engine.AircraftType = "P"

	ual := surface.Gate{Kind: "gate", Types: []string{"jets", "turboprops"}, Airlines: []string{"UAL"}}
	common := surface.Gate{Kind: "gate", Types: []string{"jets", "turboprops"}}
	ramp := surface.Gate{Kind: "tie_down", Types: []string{"jets", "turboprops", "props"}}
	smallRamp := surface.Gate{Kind: "tie_down", Types: []string{"props"}}

	// An airline prefers its own gate, then other terminal gates, then
	// ramps.
	if a, b, c := gateScore(ual, jet("UAL1")), gateScore(common, jet("UAL1")), gateScore(ramp, jet("UAL1")); !(a > b && b > c && c >= 0) {
		t.Errorf("UAL1 scores: own gate %d, common gate %d, ramp %d", a, b, c)
	}
	if a, b := gateScore(common, jet("SWA1")), gateScore(ual, jet("SWA1")); !(a > b && b >= 0) {
		t.Errorf("SWA1 scores: common gate %d, UAL gate %d", a, b)
	}
	// General aviation uses ramps, not the terminal.
	if gateScore(ual, props) >= 0 || gateScore(common, props) >= 0 || gateScore(ramp, props) < 0 {
		t.Errorf("GA scores: UAL gate %d, common gate %d, ramp %d", gateScore(ual, props),
			gateScore(common, props), gateScore(ramp, props))
	}
	// Jets don't fit on a ramp for small aircraft.
	if gateScore(smallRamp, jet("N456CD")) >= 0 {
		t.Errorf("a jet can use a props-only ramp")
	}
	// Cargo ramps are for their carriers.
	fedex := surface.Gate{Kind: "gate", Types: []string{"heavy", "jets"}, Airlines: []string{"FDX"}}
	if gateScore(fedex, jet("SWA1")) >= 0 || gateScore(fedex, jet("FDX1")) < 0 {
		t.Errorf("FedEx ramp scores: SWA1 %d, FDX1 %d", gateScore(fedex, jet("SWA1")), gateScore(fedex, jet("FDX1")))
	}
}

func TestTurnLimit(t *testing.T) {
	if l := turnLimit([2]float32{0, 0}, [2]float32{0, 1}, [2]float32{0.01, 2}); l != 0 {
		t.Errorf("straight: got limit %v", l)
	}
	if l := turnLimit([2]float32{0, 0}, [2]float32{0, 1}, [2]float32{1, 1}); l != turnSpeed {
		t.Errorf("right angle turn: got limit %v, want %v", l, turnSpeed)
	}
}

// newGroundTestSim returns a sim with KCOS's tower worked by a human (1W)
// and 35L in use, whose surface the sim moves aircraft on.
func newGroundTestSim(t *testing.T) (*Sim, *surface.Airport) {
	t.Helper()
	s := newTowerTestSim(map[TCW]*TCPConsolidation{
		"1W": {PrimaryTCP: "1W", SecondaryTCPs: []SecondaryTCP{{TCP: "1E"}}},
	})
	s.State.Airports["KCOS"] = &av.Airport{}
	s.State.ArrivalRunways = []ArrivalRunway{{Airport: "KCOS", Runway: "35L"}}
	s.State.DepartureRunways = []DepartureRunway{{Airport: "KCOS", Runway: "35L"}}
	ap, ok := s.groundMovementAirport("KCOS")
	if !ok {
		t.Fatal("no ground movement at KCOS")
	}
	return s, ap
}

// distanceToNetwork returns the distance from p to the closest edge of the
// taxi network or gate.
func distanceToNetwork(ap *surface.Airport, p [2]float32) float32 {
	d := float32(1e10)
	for _, e := range ap.Edges {
		d = min(d, math.PointSegmentDistance(p, ap.Local(ap.Nodes[e.A].Location), ap.Local(ap.Nodes[e.B].Location)))
	}
	for _, g := range ap.Gates {
		if n, ok := ap.GateNode(g); ok {
			d = min(d, math.PointSegmentDistance(p, ap.Local(g.Location), ap.Local(ap.Nodes[n].Location)))
		}
	}
	return d
}

func TestGroundDeparture(t *testing.T) {
	s, ap := newGroundTestSim(t)

	ac := &Aircraft{
		ADSBCallsign:     "SWA1",
		TypeOfFlight:     av.FlightTypeDeparture,
		DepartureAirport: "KCOS",
		WaitingForLaunch: true,
		Nav:              nav.Nav{FlightState: nav.FlightState{NmPerLongitude: ap.NMPerLongitude()}},
	}
	ac.Nav.Perf.Engine.AircraftType = "J"
	s.Aircraft[ac.ADSBCallsign] = ac

	s.departureGround(ac, "35L")
	g := ac.Ground
	if g == nil || g.Phase != GroundParked || g.Gate == "" {
		t.Fatalf("not placed at a gate: %+v", g)
	}
	if ac.ControllerFrequency != "" {
		t.Errorf("on %q's frequency at the gate", ac.ControllerFrequency)
	}

	// It taxis along the taxi network to the hold-short line and calls the
	// tower ready. Its gate's free once it's out on the taxiway.
	gate := g.Gate
	s.startTaxiOut(ac)
	if !s.gateOccupied(ap, gate) {
		t.Errorf("gate %s free while the departure's still in it", gate)
	}
	for i := 0; ac.Ground.Phase == GroundTaxiOut; i++ {
		if i > 1200 {
			t.Fatalf("still taxiing after 20 minutes: %s", ac.Ground)
		}
		s.updateGround(ac)
		if d := distanceToNetwork(ap, ap.Local(ac.Position())); d > 0.005 {
			t.Fatalf("%.0f feet off the taxi network", d*math.NauticalMilesToFeet)
		}
		if ac.Ground.Speed > taxiSpeedJet {
			t.Fatalf("taxiing at %.0f knots", ac.Ground.Speed)
		}
	}
	if s.gateOccupied(ap, gate) {
		t.Errorf("gate %s still occupied after the departure left", gate)
	}
	if ac.Ground.Phase != GroundHoldingShort || ac.Ground.Speed != 0 || !ac.ReadyForDeparture ||
		ac.ControllerFrequency != "1W" {
		t.Fatalf("at the runway: %s, speed %.0f, ready %v, frequency %q", ac.Ground, ac.Ground.Speed,
			ac.ReadyForDeparture, ac.ControllerFrequency)
	}
	// Short of the runway.
	rwy, _, _ := ap.LookupRunway("35L")
	p0, p1 := ap.Local(rwy.Ends[0].Threshold), ap.Local(rwy.Ends[1].Threshold)
	offset := math.PointLineDistance(ap.Local(ac.Position()), p0, p1)
	if offset*math.NauticalMilesToFeet < 150 {
		t.Errorf("holding short %.0f feet from the runway centerline", offset*math.NauticalMilesToFeet)
	}

	// Line up and wait: onto the runway, on its heading.
	s.groundLineUp(ac)
	for i := 0; ac.Ground.Phase == GroundLiningUp; i++ {
		if i > 300 {
			t.Fatalf("still lining up after 5 minutes: %s", ac.Ground)
		}
		s.updateGround(ac)
	}
	if ac.Ground.Phase != GroundLinedUp || math.HeadingDifference(ac.Ground.Heading, ac.Ground.LineupHeading) > 1 {
		t.Fatalf("lined up: %s heading %.0f, want %.0f", ac.Ground, ac.Ground.Heading, ac.Ground.LineupHeading)
	}
	offset = math.PointLineDistance(ap.Local(ac.Position()), p0, p1)
	if offset*math.NauticalMilesToFeet > 30 {
		t.Errorf("lined up %.0f feet off the centerline", offset*math.NauticalMilesToFeet)
	}

	// Cleared for takeoff, it's handed to the flight model.
	s.groundClearForTakeoff(ac)
	if ac.Ground != nil || ac.WaitingForLaunch || ac.ReadyForDeparture {
		t.Errorf("after takeoff clearance: ground %v, waiting %v, ready %v", ac.Ground, ac.WaitingForLaunch,
			ac.ReadyForDeparture)
	}
}

func TestGroundArrival(t *testing.T) {
	s, ap := newGroundTestSim(t)

	rwy, _, _ := ap.LookupRunway("35L")
	ac := &Aircraft{
		ADSBCallsign:        "UAL2",
		TypeOfFlight:        av.FlightTypeArrival,
		ArrivalAirport:      "KCOS",
		ControllerFrequency: "1W",
		Nav: nav.Nav{
			FlightState: nav.FlightState{
				Position:       rwy.Ends[1].Threshold, // 35L
				GS:             140,
				NmPerLongitude: ap.NMPerLongitude(),
			},
			Approach: nav.Approach{Assigned: &av.Approach{Runway: "35L"}},
		},
	}
	ac.Nav.Perf.Engine.AircraftType = "J"
	s.Aircraft[ac.ADSBCallsign] = ac

	if !s.startLandingRollout(ac) || ac.Ground == nil || ac.Ground.Phase != GroundRollout {
		t.Fatalf("no rollout: %v", ac.Ground)
	}
	gate := ac.Ground.Gate
	threshold := ap.Local(rwy.Ends[1].Threshold)
	length := ap.Distance(rwy.Ends[0].Threshold, rwy.Ends[1].Threshold)

	exitedAt := float32(-1)
	for i := 0; ac.Ground.Phase != GroundParked; i++ {
		if i > 1800 {
			t.Fatalf("not at its gate after 30 minutes: %s", ac.Ground)
		}
		prevSpeed, phase := ac.Ground.Speed, ac.Ground.Phase
		s.updateGround(ac)
		if phase == GroundRollout && ac.Ground.Speed > prevSpeed {
			t.Fatalf("sped up during the rollout, %.0f to %.0f knots", prevSpeed, ac.Ground.Speed)
		}
		if phase == GroundRollout && ac.Ground.Phase == GroundTaxiIn {
			exitedAt = math.Distance2f(ap.Local(ac.Position()), threshold)
		}
	}
	if exitedAt < 0 || exitedAt > length {
		t.Errorf("turned off %.2fnm down a %.2fnm runway", exitedAt, length)
	}
	if ac.ControllerFrequency != "" {
		t.Errorf("still on %q's frequency after clearing the runway", ac.ControllerFrequency)
	}
	for _, g := range ap.Gates {
		if g.Name == gate && ap.Distance(g.Location, ac.Position()) > 0.001 {
			t.Errorf("parked %.0f feet from gate %s", ap.Distance(g.Location, ac.Position())*math.NauticalMilesToFeet, gate)
		}
	}
	if !s.gateOccupied(ap, gate) {
		t.Errorf("gate %s not occupied by the parked arrival", gate)
	}
}

// addTaxiing adds an aircraft taxiing in at KCOS, d nm short of the first
// node of its route on the edge from node from, where it's heading along the
// route to its last node, where it stops.
func addTaxiing(t *testing.T, s *Sim, ap *surface.Airport, callsign av.ADSBCallsign, from surface.NodeID, d float32,
	route ...surface.NodeID) *Aircraft {
	t.Helper()
	prev := from
	for _, n := range route {
		if !slices.ContainsFunc(ap.Edges, func(e surface.Edge) bool {
			return (e.A == prev && e.B == n) || (e.A == n && e.B == prev)
		}) {
			t.Fatalf("KCOS has no taxiway between nodes %d and %d", prev, n)
		}
		prev = n
	}
	p0, p1 := ap.Local(ap.Nodes[from].Location), ap.Local(ap.Nodes[route[0]].Location)
	var path []GroundPoint
	for _, n := range route {
		path = append(path, GroundPoint{P: ap.Nodes[n].Location, Node: n})
	}
	path[len(path)-1].Stop = true

	ac := &Aircraft{
		ADSBCallsign:   callsign,
		TypeOfFlight:   av.FlightTypeArrival,
		ArrivalAirport: "KCOS",
		Nav:            nav.Nav{FlightState: nav.FlightState{NmPerLongitude: ap.NMPerLongitude()}},
	}
	ac.Nav.Perf.Engine.AircraftType = "J"
	ac.Ground = &GroundState{
		Phase:    GroundTaxiIn,
		Path:     path,
		LastNode: from,
		Speed:    taxiSpeedJet,
		Heading:  math.VectorHeading(math.Sub2f(p1, p0)),
	}
	s.setGroundFlightState(ac, ap, math.Lerp2f(1-d/math.Distance2f(p0, p1), p0, p1))
	s.Aircraft[callsign] = ac
	return ac
}

// runTaxi moves the aircraft for two minutes and returns the closest they
// came to each other. Each has to have got through the junction.
func runTaxi(t *testing.T, s *Sim, ap *surface.Airport, acs ...*Aircraft) float32 {
	t.Helper()
	closest := float32(1e10)
	for range 120 {
		for _, ac := range acs {
			s.updateGround(ac)
		}
		for j, a := range acs {
			for _, b := range acs[j+1:] {
				closest = min(closest, ap.Distance(a.Position(), b.Position()))
			}
		}
	}
	for _, ac := range acs {
		if slices.ContainsFunc(ac.Ground.Path, func(pt GroundPoint) bool { return pt.Node == kcosJunction }) {
			t.Errorf("%s didn't get through the junction: %s", ac.ADSBCallsign, ac.Ground)
		}
	}
	return closest
}

// At KCOS, nodes 77, 84, 61 and 75 are north, east, south and west of
// the junction at node 88.
const kcosJunction, kcosNorth, kcosEast, kcosSouth, kcosWest = 88, 77, 84, 61, 75

func TestGroundMerge(t *testing.T) {
	for _, d := range []float32{0.02, 0.03, 0.04, 0.05, 0.06, 0.08} {
		for _, first := range []av.ADSBCallsign{"AAL1", "UAL1"} {
			s, ap := newGroundTestSim(t)
			other := map[av.ADSBCallsign]av.ADSBCallsign{"AAL1": "UAL1", "UAL1": "AAL1"}[first]
			a := addTaxiing(t, s, ap, first, kcosNorth, 0.04, kcosJunction, kcosSouth)
			b := addTaxiing(t, s, ap, other, kcosEast, d, kcosJunction, kcosSouth)
			if closest := runTaxi(t, s, ap, a, b); closest < 0.02 {
				t.Errorf("%.2fnm from the junction: merged %.0f feet apart", d, closest*math.NauticalMilesToFeet)
			}
		}
	}
}

func TestGroundCrossing(t *testing.T) {
	for _, d := range []float32{0.02, 0.03, 0.04, 0.05, 0.06, 0.08} {
		for _, first := range []av.ADSBCallsign{"AAL1", "UAL1"} {
			s, ap := newGroundTestSim(t)
			other := map[av.ADSBCallsign]av.ADSBCallsign{"AAL1": "UAL1", "UAL1": "AAL1"}[first]
			a := addTaxiing(t, s, ap, first, kcosNorth, 0.04, kcosJunction, kcosSouth)
			b := addTaxiing(t, s, ap, other, kcosWest, d, kcosJunction, kcosEast)
			if closest := runTaxi(t, s, ap, a, b); closest < 0.02 {
				t.Errorf("%.2fnm from the junction: crossed %.0f feet apart", d, closest*math.NauticalMilesToFeet)
			}
		}
	}
}

func TestRunwayOccupied(t *testing.T) {
	s, ap := newGroundTestSim(t)
	rwy, idx, _ := ap.LookupRunway("35L")
	threshold, far := ap.Local(rwy.Ends[idx].Threshold), ap.Local(rwy.Ends[1-idx].Threshold)
	length := math.Distance2f(threshold, far)

	arrival := &Aircraft{
		ADSBCallsign:   "UAL2",
		TypeOfFlight:   av.FlightTypeArrival,
		ArrivalAirport: "KCOS",
		Nav: nav.Nav{
			FlightState: nav.FlightState{Position: rwy.Ends[idx].Threshold, NmPerLongitude: ap.NMPerLongitude()},
			Approach:    nav.Approach{Assigned: &av.Approach{Runway: "35L"}},
		},
	}
	s.Aircraft[arrival.ADSBCallsign] = arrival
	if s.runwayOccupied(arrival) {
		t.Errorf("empty runway occupied")
	}

	for _, tc := range []struct {
		what     string
		ac       *Aircraft
		phase    GroundPhase
		d, off   float32 // nm down the runway and off its centerline
		occupied bool
	}{
		{"departure lined up", &Aircraft{TypeOfFlight: av.FlightTypeDeparture, DepartureAirport: "KCOS"}, GroundLinedUp, 0.05, 0, true},
		{"departure lining up", &Aircraft{TypeOfFlight: av.FlightTypeDeparture, DepartureAirport: "KCOS"}, GroundLiningUp, 0.05, 0.005, true},
		{"departure holding short", &Aircraft{TypeOfFlight: av.FlightTypeDeparture, DepartureAirport: "KCOS"}, GroundHoldingShort, 0.05, 0.05, false},
		{"arrival rolling out", &Aircraft{TypeOfFlight: av.FlightTypeArrival, ArrivalAirport: "KCOS"}, GroundRollout, 0.3, 0, true},
		{"arrival rolling out far ahead", &Aircraft{TypeOfFlight: av.FlightTypeArrival, ArrivalAirport: "KCOS"}, GroundRollout, 0.8, 0, false},
	} {
		ac := tc.ac
		ac.ADSBCallsign = "SWA1"
		ac.Nav.FlightState.NmPerLongitude = ap.NMPerLongitude()
		ac.Ground = &GroundState{Phase: tc.phase, LastNode: -1}
		p := math.Lerp2f(tc.d/length, threshold, far)
		p = math.Add2f(p, math.Scale2f(math.Normalize2f([2]float32{far[1] - threshold[1], threshold[0] - far[0]}), tc.off))
		s.setGroundFlightState(ac, ap, p)
		s.Aircraft[ac.ADSBCallsign] = ac
		if occ := s.runwayOccupied(arrival); occ != tc.occupied {
			t.Errorf("%s: occupied %v, want %v", tc.what, occ, tc.occupied)
		}
		delete(s.Aircraft, ac.ADSBCallsign)
	}

	// A departure that's rolling, until it's airborne.
	dep := &Aircraft{ADSBCallsign: "SWA3", TypeOfFlight: av.FlightTypeDeparture, DepartureAirport: "KCOS"}
	dep.Nav.FlightState = nav.FlightState{Position: ap.FromLocal(math.Lerp2f(0.3/length, threshold, far)),
		Altitude: ap.ElevationFt, NmPerLongitude: ap.NMPerLongitude()}
	s.Aircraft[dep.ADSBCallsign] = dep
	if !s.runwayOccupied(arrival) {
		t.Errorf("departure on its takeoff roll: not occupied")
	}
	dep.Nav.FlightState.Position = ap.FromLocal(math.Lerp2f(0.8/length, threshold, far))
	dep.Nav.FlightState.Altitude = ap.ElevationFt + 100
	if s.runwayOccupied(arrival) {
		t.Errorf("departure airborne 0.8nm down the runway: occupied")
	}
}

func TestHeldAtGate(t *testing.T) {
	s, ap := newGroundTestSim(t)

	// Two gates that leave the gate area at the same spot.
	var g1, g2 surface.Gate
	var node surface.NodeID
	byNode := make(map[surface.NodeID][]surface.Gate)
	for _, g := range ap.Gates {
		if n, ok := ap.GateNode(g); ok {
			byNode[n] = append(byNode[n], g)
			if len(byNode[n]) == 2 {
				g1, g2, node = byNode[n][0], byNode[n][1], n
			}
		}
	}
	if g1.Name == "" {
		t.Fatal("no gates at KCOS share a taxiway spot")
	}
	entry, _ := ap.DepartureEntry("35L")
	r, err := ap.FindRoute(surface.RouteRequest{From: node, To: []surface.NodeID{entry}})
	if err != nil {
		t.Fatal(err)
	}

	dep := &Aircraft{
		ADSBCallsign:     "SWA1",
		TypeOfFlight:     av.FlightTypeDeparture,
		DepartureAirport: "KCOS",
		Nav:              nav.Nav{FlightState: nav.FlightState{NmPerLongitude: ap.NMPerLongitude()}},
	}
	dep.Ground = &GroundState{Phase: GroundTaxiOut, Path: taxiOutPath(ap, r, nil, nil), LastNode: -1, Gate: g1.Name,
		TaxiStartTime: s.State.SimTime}
	s.setGroundFlightState(dep, ap, ap.Local(g1.Location))
	s.Aircraft[dep.ADSBCallsign] = dep
	if s.heldAtGate(dep, ap) {
		t.Errorf("held with no other traffic")
	}

	// An arrival that's just turned into the gate next door.
	arr := &Aircraft{
		ADSBCallsign:   "UAL2",
		TypeOfFlight:   av.FlightTypeArrival,
		ArrivalAirport: "KCOS",
		Nav:            nav.Nav{FlightState: nav.FlightState{NmPerLongitude: ap.NMPerLongitude()}},
	}
	arr.Ground = &GroundState{
		Phase:    GroundTaxiIn,
		Path:     []GroundPoint{{P: g2.Location, Node: -1, Stop: true, Event: GroundEventAtGate}},
		LastNode: node,
		Speed:    10,
		Gate:     g2.Name,
	}
	s.setGroundFlightState(arr, ap, math.Lerp2f(0.2, ap.Local(ap.Nodes[node].Location), ap.Local(g2.Location)))
	s.Aircraft[arr.ADSBCallsign] = arr
	if !s.heldAtGate(dep, ap) {
		t.Errorf("not held for an arrival turning into %s next to %s", g2.Name, g1.Name)
	}
	// Nor can another arrival be given that gate while it's leaving.
	if !s.gateBlocked(ap, g2) {
		t.Errorf("%s not blocked while %s's leaving %s", g2.Name, dep.ADSBCallsign, g1.Name)
	}

	// The ramp doesn't hold it forever.
	dep.Ground.TaxiStartTime = s.State.SimTime.Add(-maxGateHold - time.Second)
	if s.heldAtGate(dep, ap) {
		t.Errorf("held for more than %s", maxGateHold)
	}
	dep.Ground.TaxiStartTime = s.State.SimTime

	arr.Ground.Phase = GroundParked
	if s.heldAtGate(dep, ap) {
		t.Errorf("held once the arrival's parked")
	}
}

func TestGroundHeadOn(t *testing.T) {
	// One taxis south through the junction while the other comes north up
	// the same stretch and turns east there.
	for _, d := range []float32{0.05, 0.08, 0.12, 0.2} {
		for _, first := range []av.ADSBCallsign{"AAL1", "UAL1"} {
			s, ap := newGroundTestSim(t)
			other := map[av.ADSBCallsign]av.ADSBCallsign{"AAL1": "UAL1", "UAL1": "AAL1"}[first]
			a := addTaxiing(t, s, ap, first, kcosNorth, d, kcosJunction, kcosSouth)
			b := addTaxiing(t, s, ap, other, kcosSouth, 0.06, kcosJunction, kcosEast)
			if closest := runTaxi(t, s, ap, a, b); closest < 0.02 {
				t.Errorf("%.2fnm from the junction: passed %.0f feet apart", d, closest*math.NauticalMilesToFeet)
			}
		}
	}
}

func TestRestoreGroundStates(t *testing.T) {
	s, ap := newGroundTestSim(t)
	ac := &Aircraft{
		ADSBCallsign:     "SWA1",
		TypeOfFlight:     av.FlightTypeDeparture,
		DepartureAirport: "KCOS",
		WaitingForLaunch: true,
		Nav:              nav.Nav{FlightState: nav.FlightState{NmPerLongitude: ap.NMPerLongitude()}},
	}
	ac.Nav.Perf.Engine.AircraftType = "J"
	s.Aircraft[ac.ADSBCallsign] = ac
	s.departureGround(ac, "35L")
	want := slices.Clone(ac.Ground.Path)

	// As saved before paths recorded their nodes.
	for i := range ac.Ground.Path {
		ac.Ground.Path[i].Node = 0
	}
	ac.Ground.Version = 0
	s.restoreGroundStates()
	if !slices.Equal(ac.Ground.Path, want) || ac.Ground.LastNode != -1 {
		t.Errorf("restored path %v, last node %d; want %v, -1", ac.Ground.Path, ac.Ground.LastNode, want)
	}

	// Current ones are left as they are.
	ac.Ground.LastNode = 7
	s.restoreGroundStates()
	if ac.Ground.LastNode != 7 {
		t.Errorf("current ground state changed on restore")
	}
}
