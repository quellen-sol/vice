// sim/ground_traffic.go
// Copyright(c) vice contributors, licensed under the GNU Public License, Version 3.
// SPDX: GPL-3.0-only

package sim

import (
	"slices"

	"github.com/mmp/vice/math"
	"github.com/mmp/vice/surface"
	"github.com/mmp/vice/util"
)

// This file has how aircraft the sim moves on the ground keep out of each
// other's way (see ground.go): routes avoid taxiways that others are using
// the opposite way, departures wait at their gates while the ramp is busy,
// aircraft follow those ahead of them, take turns along stretches of
// taxiway that others are taxiing along the other way and give way to
// those whose paths they'd cut across, and none stops on a runway it's
// crossing. Arrivals go around if their runway isn't clear when they reach
// it.

///////////////////////////////////////////////////////////////////////////
// Routing and the ramp

// taxiwayUse returns the stretches of the taxi network, from one node to
// the next, that aircraft other than ac are taxiing along or will. If
// include is given, it only considers the aircraft for which it returns
// true.
func (s *Sim) taxiwayUse(ap *surface.Airport, ac *Aircraft, include func(*Aircraft) bool) map[[2]surface.NodeID]bool {
	use := make(map[[2]surface.NodeID]bool)
	for _, other := range s.Aircraft {
		og := other.Ground
		if other == ac || og == nil || og.Phase == GroundParked || string(groundAirport(other)) != ap.ICAO ||
			(include != nil && !include(other)) {
			continue
		}
		prev := og.LastNode
		for _, pt := range og.Path {
			if prev >= 0 && pt.Node >= 0 {
				use[[2]surface.NodeID{prev, pt.Node}] = true
			}
			prev = pt.Node
		}
	}
	return use
}

// opposedEdgeCost returns a route edge cost that steers routes away from
// taxiways that other traffic is using the other way.
func opposedEdgeCost(ap *surface.Airport, use map[[2]surface.NodeID]bool) func(int, surface.NodeID) float32 {
	return func(edge int, from surface.NodeID) float32 {
		if use[[2]surface.NodeID{ap.Edges[edge].Other(from), from}] {
			return opposedTaxiwayCost
		}
		return 0
	}
}

// leavingGate reports whether the aircraft is a departure that's started to
// taxi but hasn't reached the taxiway yet.
func leavingGate(ac *Aircraft) bool {
	g := ac.Ground
	return g != nil && g.Phase == GroundTaxiOut && g.LastNode == -1
}

// waitingAtGate reports whether the aircraft is a departure that's still at
// its gate, waiting to leave; others treat it as they would one parked
// there.
func waitingAtGate(ap *surface.Airport, ac *Aircraft) bool {
	if !leavingGate(ac) || ac.Ground.Speed > 0 {
		return false
	}
	i := slices.IndexFunc(ap.Gates, func(g surface.Gate) bool { return g.Name == ac.Ground.Gate })
	return i != -1 && ap.Distance(ac.Position(), ap.Gates[i].Location) < 0.003
}

// heldAtGate reports whether a departure leaving its gate waits there, as
// the ramp would hold it: for another aircraft on its way into or out of a
// gate that leaves the gate area at the same spot, other traffic about to
// pass that spot, or traffic coming the other way along its route. Of
// those waiting to leave their gates, the ones ahead in callsign order go
// first, and none is held longer than maxGateHold.
func (s *Sim) heldAtGate(ac *Aircraft, ap *surface.Airport) bool {
	g := ac.Ground
	if len(g.Path) == 0 || s.State.SimTime.Sub(g.TaxiStartTime) > maxGateHold {
		return false
	}
	node := g.Path[0].Node
	include := func(other *Aircraft) bool {
		return !leavingGate(other) || other.Ground.Speed > 0 || other.ADSBCallsign < ac.ADSBCallsign
	}
	for _, other := range s.Aircraft {
		og := other.Ground
		if other == ac || og == nil || !og.Phase.moving() || !include(other) {
			continue
		}
		if n, ok := gateNodeByName(ap, og.Gate); ok && n == node {
			return true
		}
		if ap.Distance(other.Position(), g.Path[0].P) < 0.3 &&
			slices.ContainsFunc(og.Path, func(pt GroundPoint) bool { return pt.Node == node }) {
			return true
		}
	}

	use := s.taxiwayUse(ap, ac, include)
	prev := surface.NodeID(-1)
	for _, pt := range g.Path {
		if prev >= 0 && pt.Node >= 0 && use[[2]surface.NodeID{pt.Node, prev}] {
			return true
		}
		prev = pt.Node
	}
	return false
}

// routeBlocked reports whether an aircraft is holding where it's in the way
// of the start of a route from the runway exit at node exit.
func (s *Sim) routeBlocked(ap *surface.Airport, ac *Aircraft, exit surface.NodeID, r surface.Route) bool {
	pts := [][2]float32{ap.Local(ap.Nodes[exit].Location)}
	along := float32(0)
	for _, n := range r.Nodes {
		p := ap.Local(ap.Nodes[n].Location)
		along += math.Distance2f(pts[len(pts)-1], p)
		pts = append(pts, p)
		if along > 0.3 {
			break
		}
	}
	for _, other := range s.Aircraft {
		og := other.Ground
		if other == ac || og == nil || !og.Phase.holding() || og.Phase == GroundParked ||
			string(groundAirport(other)) != ap.ICAO {
			continue
		}
		op := ap.Local(other.Position())
		for i := 0; i+1 < len(pts); i++ {
			if math.PointSegmentDistance(op, pts[i], pts[i+1]) < 0.015 {
				return true
			}
		}
	}
	return false
}

// groundTrafficNear reports whether there's an aircraft other than ac
// taxiing near p.
func (s *Sim) groundTrafficNear(ap *surface.Airport, ac *Aircraft, p math.Point2LL) bool {
	for _, other := range s.Aircraft {
		if og := other.Ground; other != ac && og != nil && og.Phase != GroundParked &&
			string(groundAirport(other)) == ap.ICAO && ap.Distance(other.Position(), p) < 2*followDistance {
			return true
		}
	}
	return false
}

// routeOpposed reports whether other traffic is using any of the route's
// taxiways the other way.
func routeOpposed(r surface.Route, use map[[2]surface.NodeID]bool) bool {
	for i := 0; i+1 < len(r.Nodes); i++ {
		if use[[2]surface.NodeID{r.Nodes[i+1], r.Nodes[i]}] {
			return true
		}
	}
	return false
}

///////////////////////////////////////////////////////////////////////////
// Taxiing

// groundTrafficLimit returns the fastest the aircraft, which slows at decel
// knots per second, can go given the other aircraft on the ground near it:
// it keeps its distance behind one ahead of it on its path and, unless it's
// rolling out after landing, gives way to one whose path it would cut
// across.
func (s *Sim) groundTrafficLimit(ac *Aircraft, ap *surface.Airport, pos [2]float32, decel float32) float32 {
	g := ac.Ground
	limit := float32(1000)
	var mine [][2]float32
	stillGivingWay := false
	// Once it's started across a runway, it only stops for traffic ahead.
	_, crossing := crossingLeft(ap, pos, g.Path)
	crossing = crossing && g.Speed > 0
	for other := range util.SortedMapValues(s.Aircraft) {
		og := other.Ground
		if other == ac || og == nil || og.Phase == GroundParked || string(groundAirport(other)) != ap.ICAO ||
			waitingAtGate(ap, other) {
			continue
		}
		op := ap.Local(other.Position())
		dist := math.Distance2f(pos, op)
		if dist > 1.5 {
			continue
		}

		// Taxiing the opposite way along the same stretch of taxiway:
		// whoever gets there first goes, and the other waits short of it.
		// (If both are already on it, there's nothing for it but to pass.)
		if g.Phase != GroundRollout && !crossing && og.Phase.moving() {
			if st, ok := opposedStretch(ap, ac, pos, other, op); ok {
				if !st.mineIn && (st.theirsIn || og.Phase == GroundRollout || !goesFirst(ac, other, st)) {
					limit = min(limit, brakingSpeed(st.myDist-followDistance, decel))
				}
				continue
			}
		}
		if dist > 0.4 {
			continue
		}

		// Ahead on its path, going its way or holding there: follow it.
		if d, hdg, ok := pathAhead(ap, pos, g.Path, op, 0.3); ok &&
			(og.Phase.holding() || math.HeadingDifference(og.Heading, hdg) < 90) {
			if g.Phase == GroundLiningUp {
				// Don't pull onto the runway behind it.
				return 0
			}
			if left, ok := crossingLeft(ap, pos, g.Path); ok && g.Speed == 0 && d < left+followDistance {
				// Cleared across a runway: wait short of it until there's
				// room on the other side.
				return 0
			}
			limit = min(limit, brakingSpeed(d-followDistance, decel))
			continue
		}

		// Otherwise it only matters if it's moving, isn't waiting for this
		// one, and their paths meet.
		if g.Phase == GroundRollout || crossing || !og.Phase.moving() || og.GivingWay == ac.ADSBCallsign {
			continue
		}
		if mine == nil {
			mine = groundTrajectory(ap, pos, g)
		}
		theirs := groundTrajectory(ap, op, og)
		if !trajectoriesConflict(mine, theirs) {
			continue
		}
		if _, _, inTheWay := pathAhead(ap, op, og.Path, pos, 1); inTheWay {
			// It's no use waiting where it's in the other's way.
			continue
		}
		if g.GivingWay != other.ADSBCallsign {
			// Whoever can stop short of the other's path gives way; if both
			// can, the one with less of a claim to go first does.
			iCan := canGiveWay(ap, ac, pos, other, op)
			theyCan := og.Phase != GroundRollout && canGiveWay(ap, other, op, ac, pos)
			if !iCan || (theyCan && !givesWay(ac, other)) {
				continue
			}
			g.GivingWay = other.ADSBCallsign
		}
		// Stop short of where it's going, or as soon as possible.
		stillGivingWay = true
		if d, ok := pathDistanceToConflict(ap, pos, g.Path, theirs); ok {
			limit = min(limit, brakingSpeed(d, decel))
		}
	}
	if !stillGivingWay {
		g.GivingWay = ""
	}
	return limit
}

// stretch describes a stretch of taxiway that two aircraft are taxiing
// along in opposite directions: how far each has to go to get to it (zero
// if it's already on it) and whether it's on it.
type stretch struct {
	myDist, theirDist float32
	mineIn, theirsIn  bool
}

// pathNode is a taxiway node along an aircraft's path and how far ahead it
// is.
type pathNode struct {
	node surface.NodeID
	dist float32
}

// pathNodes returns the nodes along an aircraft's path, starting with the
// one it last passed, if it's on a taxiway. Points that aren't nodes are
// -1.
func pathNodes(ap *surface.Airport, pos [2]float32, g *GroundState) []pathNode {
	var nodes []pathNode
	if g.LastNode >= 0 {
		nodes = append(nodes, pathNode{g.LastNode, 0})
	}
	prev, along := pos, float32(0)
	for _, pt := range g.Path {
		p := ap.Local(pt.P)
		along += math.Distance2f(prev, p)
		prev = p
		nodes = append(nodes, pathNode{pt.Node, along})
	}
	return nodes
}

// opposedStretch returns the stretch of taxiway that ac and other are
// taxiing along in opposite directions, if there is one.
func opposedStretch(ap *surface.Airport, ac *Aircraft, pos [2]float32, other *Aircraft, op [2]float32) (stretch, bool) {
	mine, theirs := pathNodes(ap, pos, ac.Ground), pathNodes(ap, op, other.Ground)
	edges := func(nodes []pathNode) map[[2]surface.NodeID]bool {
		m := make(map[[2]surface.NodeID]bool)
		for i := 0; i+1 < len(nodes); i++ {
			if nodes[i].node >= 0 && nodes[i+1].node >= 0 {
				m[[2]surface.NodeID{nodes[i].node, nodes[i+1].node}] = true
			}
		}
		return m
	}
	// entry returns where the aircraft with the given path nodes gets to the
	// first stretch the other uses the opposite way.
	entry := func(nodes []pathNode, otherEdges map[[2]surface.NodeID]bool, onTaxiway bool) (float32, bool, bool) {
		for i := 0; i+1 < len(nodes); i++ {
			if otherEdges[[2]surface.NodeID{nodes[i+1].node, nodes[i].node}] {
				in := i == 0 && onTaxiway
				if in {
					return 0, true, true
				}
				return nodes[i].dist, false, true
			}
		}
		return 0, false, false
	}
	var st stretch
	var ok bool
	if st.myDist, st.mineIn, ok = entry(mine, edges(theirs), ac.Ground.LastNode >= 0); !ok {
		return st, false
	}
	st.theirDist, st.theirsIn, _ = entry(theirs, edges(mine), other.Ground.LastNode >= 0)
	return st, true
}

// goesFirst reports whether ac goes first along a stretch of taxiway that it
// and other are taxiing along in opposite directions, neither of them yet on
// it: whoever's going to get there first, or if it's close, the one with a
// better claim to go first.
func goesFirst(ac, other *Aircraft, st stretch) bool {
	mine := st.myDist / max(ac.Ground.Speed, creepSpeed)
	theirs := st.theirDist / max(other.Ground.Speed, creepSpeed)
	if math.Abs(mine-theirs) > 2./3600 {
		return mine < theirs
	}
	return !givesWay(ac, other)
}

// crossingLeft returns how far the aircraft at pos has to go along its path
// to be across the runway it's crossing, if it's crossing one.
func crossingLeft(ap *surface.Airport, pos [2]float32, path []GroundPoint) (float32, bool) {
	prev, along := pos, float32(0)
	for _, pt := range path {
		p := ap.Local(pt.P)
		along += math.Distance2f(prev, p)
		prev = p
		if pt.Event == GroundEventCrossed {
			return along, true
		}
		if pt.Stop {
			break
		}
	}
	return 0, false
}

// canGiveWay reports whether the aircraft at pos can stop before it gets in
// the way of the other aircraft, at op: without coming near the other's
// path ahead.
func canGiveWay(ap *surface.Airport, ac *Aircraft, pos [2]float32, other *Aircraft, op [2]float32) bool {
	const near = 0.015 // nm, about 90'
	g, og := ac.Ground, other.Ground
	theirPath := pathPoints(ap, op, og.Path, 1)
	stop := g.Speed * g.Speed / (2 * taxiDecel * 3600)
	mine := pathPoints(ap, pos, g.Path, stop)
	if len(mine) == 1 {
		mine = append(mine, pos) // stopped
	}
	for i := 0; i+1 < len(mine); i++ {
		for t := float32(0); t <= 1; t += 0.1 {
			p := math.Lerp2f(t, mine[i], mine[i+1])
			for j := 0; j+1 < len(theirPath); j++ {
				if math.PointSegmentDistance(p, theirPath[j], theirPath[j+1]) < near {
					return false
				}
			}
		}
	}
	return true
}

// pathPoints returns the points of the path from pos for maxDist nm along
// it, up to its next stop.
func pathPoints(ap *surface.Airport, pos [2]float32, path []GroundPoint, maxDist float32) [][2]float32 {
	pts := [][2]float32{pos}
	along := float32(0)
	for _, pt := range path {
		p := ap.Local(pt.P)
		l := math.Distance2f(pts[len(pts)-1], p)
		if along+l >= maxDist {
			if l > 0 {
				pts = append(pts, math.Lerp2f((maxDist-along)/l, pts[len(pts)-1], p))
			}
			break
		}
		pts = append(pts, p)
		along += l
		if pt.Stop {
			break
		}
	}
	return pts
}

// brakingSpeed returns the fastest an aircraft that slows at decel knots
// per second can go and still stop within d nm.
func brakingSpeed(d, decel float32) float32 {
	return math.Sqrt(2 * decel * 3600 * max(0, d))
}

// pathAhead returns how far along the path from pos the point q is, if it's
// on the path within maxDist, and the path's heading there.
func pathAhead(ap *surface.Airport, pos [2]float32, path []GroundPoint, q [2]float32, maxDist float32) (float32, math.TrueHeading, bool) {
	const lateral = 0.012 // nm, about 70'
	prev, along := pos, float32(0)
	for i, pt := range path {
		p := ap.Local(pt.P)
		seg := math.Sub2f(p, prev)
		if l := math.Length2f(seg); l > 1e-6 {
			t := math.Dot(math.Sub2f(q, prev), seg) / (l * l)
			// (Something beside it or behind it isn't ahead.)
			if (i > 0 || t > 0) && math.PointSegmentDistance(q, prev, p) < lateral {
				return along + math.Clamp(t, 0, 1)*l, math.VectorHeading(seg), true
			}
			along += l
		}
		if along > maxDist || pt.Stop {
			break
		}
		prev = p
	}
	return 0, 0, false
}

// groundTrajectory returns where an aircraft at pos will be each second
// for the next groundLookahead seconds: along its path at its speed (at
// least creepSpeed if it's taxiing) as far as its next stop, or where it is
// if it's holding.
func groundTrajectory(ap *surface.Airport, pos [2]float32, g *GroundState) [][2]float32 {
	var speed float32
	switch {
	case g.Phase == GroundRollout:
		speed = g.Speed
	case g.Phase.moving():
		speed = max(g.Speed, creepSpeed)
	}
	traj := make([][2]float32, groundLookahead+1)
	p, path := pos, g.Path
	traj[0] = p
	for k := 1; k <= groundLookahead; k++ {
		for d := speed / 3600; d > 0 && len(path) > 0; {
			q := ap.Local(path[0].P)
			l := math.Distance2f(p, q)
			if l > d {
				p = math.Lerp2f(d/l, p, q)
				break
			}
			p, d = q, d-l
			if path[0].Stop {
				path = nil
			} else {
				path = path[1:]
			}
		}
		traj[k] = p
	}
	return traj
}

// trajectoriesConflict reports whether two aircraft following the
// trajectories come within conflictDistance of each other, closer than
// they are now.
func trajectoriesConflict(a, b [][2]float32) bool {
	d0 := math.Distance2f(a[0], b[0])
	for k := 1; k < len(a); k++ {
		if d := math.Distance2f(a[k], b[k]); d < conflictDistance && d < d0 {
			return true
		}
	}
	return false
}

// clearOf reports whether p is clear of everywhere along the trajectory.
func clearOf(p [2]float32, traj [][2]float32) bool {
	return !slices.ContainsFunc(traj, func(q [2]float32) bool {
		return math.Distance2f(p, q) < conflictDistance
	})
}

// pathDistanceToConflict returns how far an aircraft at pos can go along
// its path before it comes within conflictDistance of the trajectory.
func pathDistanceToConflict(ap *surface.Airport, pos [2]float32, path []GroundPoint, traj [][2]float32) (float32, bool) {
	const step, maxDist = 0.002, 0.3 // nm
	prev, along := pos, float32(0)
	for _, pt := range path {
		p := ap.Local(pt.P)
		l := math.Distance2f(prev, p)
		for t := float32(0); t < l; t += step {
			if !clearOf(math.Lerp2f(t/l, prev, p), traj) {
				return along + t, true
			}
		}
		along += l
		if along > maxDist || pt.Stop {
			break
		}
		prev = p
	}
	return 0, false
}

// vacating reports whether an arrival is still getting off its runway.
func vacating(g *GroundState) bool {
	return g.Phase == GroundRollout || (g.Phase == GroundTaxiIn &&
		slices.ContainsFunc(g.Path, func(pt GroundPoint) bool { return pt.Event == GroundEventClearOfRunway }))
}

// givesWay reports whether ac gives way to other when each could wait for
// the other: an arrival getting off its runway goes first, then whichever
// is going faster.
func givesWay(ac, other *Aircraft) bool {
	g, og := ac.Ground, other.Ground
	if vacating(g) != vacating(og) {
		return vacating(og)
	}
	if math.Abs(g.Speed-og.Speed) > 1 {
		return g.Speed < og.Speed
	}
	return ac.ADSBCallsign > other.ADSBCallsign
}

///////////////////////////////////////////////////////////////////////////
// Runways

// runwayOccupied reports whether an arrival reaching the threshold has to
// go around for traffic on its runway: an aircraft lined up or lining up,
// one crossing, an arrival ahead that's still slowing down or turning off
// near the threshold, or a departure ahead that's still on its takeoff
// roll.
func (s *Sim) runwayOccupied(ac *Aircraft) bool {
	appr := ac.Nav.Approach.Assigned
	if s.prespawn || appr == nil {
		return false
	}
	ap, ok := s.groundMovementAirport(ac.ArrivalAirport)
	if !ok {
		return false
	}
	rwy, idx, ok := ap.LookupRunway(appr.Runway)
	if !ok {
		return false
	}
	threshold, far := ap.Local(rwy.Ends[idx].Threshold), ap.Local(rwy.Ends[1-idx].Threshold)
	length := math.Distance2f(threshold, far)
	dir := math.Scale2f(math.Sub2f(far, threshold), 1/length)
	halfWidth := (max(rwy.WidthFt, 100)/2 + 50) / math.NauticalMilesToFeet
	// along returns how far down the runway p is and whether it's on it.
	along := func(p math.Point2LL) (float32, bool) {
		lp := ap.Local(p)
		d := math.Dot(math.Sub2f(lp, threshold), dir)
		return d, d > -0.02 && d < length+0.02 && math.PointLineDistance(lp, threshold, far) < halfWidth
	}

	for _, other := range s.Aircraft {
		if other == ac {
			continue
		}
		if og := other.Ground; og != nil {
			if og.Phase == GroundParked || (og.Phase == GroundTaxiIn && !vacating(og)) {
				continue // (Clear of the runway.)
			}
			if d, on := along(other.Position()); on && !(vacating(og) && d > 0.5) {
				return true
			}
		} else if other.IsDeparture() && other.DepartureAirport == ac.ArrivalAirport && !other.WaitingForLaunch {
			if _, on := along(other.Position()); on && other.Altitude() < ap.ElevationFt+50 {
				return true
			}
		}
	}
	return false
}
