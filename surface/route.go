// surface/route.go
// Copyright(c) vice contributors, licensed under the GNU Public License, Version 3.
// SPDX: GPL-3.0-only

package surface

import (
	"container/heap"
	"fmt"
	"slices"
	"strings"

	"github.com/mmp/vice/math"
)

// RouteRequest describes a taxi route to be found by FindRoute.
type RouteRequest struct {
	From NodeID
	// To lists the acceptable destination nodes; the route ends at
	// whichever one is cheapest to reach.
	To []NodeID
	// Via lists the taxiways (or runways, e.g. "13" or "13/31") the
	// route must follow, in order, as in "taxi via A, B, C".
	Via []string
	// TaxiOnRunways lists runways (by end, e.g. "17R", or designator, e.g.
	// "17R/35L") that the route may taxi along. Runways that appear in Via
	// are implicitly included. Runways may always be crossed.
	TaxiOnRunways []string
	// SizeCode is the aircraft's ICAO size code (A-F); edges restricted
	// to smaller aircraft are not used. If empty, edge size restrictions
	// are ignored.
	SizeCode string
}

// HoldShort is a point along a route where an aircraft must stop unless
// it has been cleared onto or across the runways it is about to enter.
type HoldShort struct {
	// Index is the index into Route.Nodes of the node where the
	// aircraft holds.
	Index int
	// RunwayEnds are the ends of the runways whose protected area begins
	// after the node, e.g. ["17R", "35L"].
	RunwayEnds []string
}

type Route struct {
	// Nodes is the sequence of nodes from the origin to the destination.
	Nodes []NodeID
	// Edges[i] is the index of the edge between Nodes[i] and Nodes[i+1].
	Edges []int
	// HoldShorts lists the route's runway hold-short points in order.
	HoldShorts []HoldShort
	// Length is the route's length in nautical miles.
	Length float32
}

// Taxiways returns the names of the taxiways the route follows, with
// consecutive duplicates removed, as a controller would read them.
func (r Route) Taxiways(ap *Airport) []string {
	var tw []string
	for _, e := range r.Edges {
		if n := ap.Edges[e].Name; n != "" && (len(tw) == 0 || tw[len(tw)-1] != n) {
			tw = append(tw, n)
		}
	}
	return tw
}

// Off-route connector edges (those not on a taxiway named in
// RouteRequest.Via) cost this much more per nm, plus a fixed penalty, so
// that routes follow the assigned taxiways wherever possible but can
// still use short unnamed or unmentioned connectors (ramp exits, runway
// entrance stubs, and the like).
const (
	connectorCostFactor = 4
	connectorPenaltyNM  = 0.02
)

// FindRoute finds the lowest-cost route that satisfies the request.
func (ap *Airport) FindRoute(req RouteRequest) (Route, error) {
	if req.From < 0 || int(req.From) >= len(ap.Nodes) {
		return Route{}, fmt.Errorf("invalid origin node %d", req.From)
	}
	if len(req.To) == 0 {
		return Route{}, fmt.Errorf("no destination given")
	}

	nStages := len(req.Via) + 1
	// stage s means the first s taxiways in Via have been reached.
	stateIndex := func(n NodeID, stage int) int { return int(n)*nStages + stage }

	matches := func(e Edge, via string) bool {
		if strings.EqualFold(e.Name, via) {
			return true
		}
		return e.Runway && runwayMatches(e.Name, via)
	}
	runwayAllowed := func(e Edge) bool {
		for _, r := range req.TaxiOnRunways {
			if runwayMatches(e.Name, r) {
				return true
			}
		}
		for _, v := range req.Via {
			if runwayMatches(e.Name, v) {
				return true
			}
		}
		return false
	}

	nStates := len(ap.Nodes) * nStages
	dist := make([]float32, nStates)
	prevState := make([]int, nStates)
	prevEdge := make([]int, nStates)
	for i := range dist {
		dist[i] = -1
		prevState[i] = -1
	}

	start := stateIndex(req.From, 0)
	dist[start] = 0
	pq := &stateQueue{{state: start, cost: 0}}
	goal := -1

	for pq.Len() > 0 {
		cur := heap.Pop(pq).(stateCost)
		if cur.cost > dist[cur.state] {
			continue // stale entry
		}
		node, stage := NodeID(cur.state/nStages), cur.state%nStages
		if stage == nStages-1 && slices.Contains(req.To, node) {
			goal = cur.state
			break
		}

		for _, ei := range ap.adjacency[node] {
			e := ap.Edges[ei]
			if e.OneWay && e.A != node {
				continue
			}
			if e.Runway && !runwayAllowed(e) {
				continue
			}
			if req.SizeCode != "" && e.WidthCode != "" && req.SizeCode > e.WidthCode {
				continue
			}

			length := ap.EdgeLength(ei)
			nextStage, cost := stage, length
			if stage < len(req.Via) && matches(e, req.Via[stage]) {
				nextStage = stage + 1
			} else if stage > 0 && matches(e, req.Via[stage-1]) {
				// Continuing along the current taxiway.
			} else if len(req.Via) > 0 {
				cost = length*connectorCostFactor + connectorPenaltyNM
			}

			next := stateIndex(e.Other(node), nextStage)
			if nc := cur.cost + cost; dist[next] < 0 || nc < dist[next] {
				dist[next] = nc
				prevState[next] = cur.state
				prevEdge[next] = ei
				heap.Push(pq, stateCost{state: next, cost: nc})
			}
		}
	}

	if goal == -1 {
		if len(req.Via) > 0 {
			return Route{}, fmt.Errorf("no route via %s", strings.Join(req.Via, " "))
		}
		return Route{}, fmt.Errorf("no route found")
	}

	var route Route
	for s := goal; s != -1; s = prevState[s] {
		route.Nodes = append(route.Nodes, NodeID(s/nStages))
		if prevState[s] != -1 {
			route.Edges = append(route.Edges, prevEdge[s])
			route.Length += ap.EdgeLength(prevEdge[s])
		}
	}
	slices.Reverse(route.Nodes)
	slices.Reverse(route.Edges)
	route.HoldShorts = ap.HoldShorts(route, false)
	return route, nil
}

// runwayMatches reports whether the runway designator (e.g. "17R/35L")
// matches rwy, which may be a designator or a single runway end.
func runwayMatches(designator, rwy string) bool {
	if strings.EqualFold(designator, rwy) {
		return true
	}
	for end := range strings.SplitSeq(designator, "/") {
		if strings.EqualFold(end, rwy) {
			return true
		}
	}
	return false
}

// HoldShorts returns the points along the route where an aircraft must
// hold short of a runway: each node after which the route enters the
// protected area of a runway it wasn't already in. If the route starts
// on a runway (e.g., an arrival exiting after landing), that runway does
// not generate a hold short.
func (ap *Airport) HoldShorts(r Route, includeILS bool) []HoldShort {
	if len(r.Nodes) == 0 {
		return nil
	}
	var current []string
	for _, ei := range ap.adjacency[r.Nodes[0]] {
		if ap.Edges[ei].Runway {
			current = mergeSorted(current, ap.Edges[ei].ProtectedRunwayEnds(includeILS))
		}
	}

	var hs []HoldShort
	for i, ei := range r.Edges {
		prot := ap.Edges[ei].ProtectedRunwayEnds(includeILS)
		var entering []string
		for _, r := range prot {
			if !slices.Contains(current, r) {
				entering = append(entering, r)
			}
		}
		if len(entering) > 0 {
			hs = append(hs, HoldShort{Index: i, RunwayEnds: entering})
		}
		current = prot
	}
	return hs
}

func mergeSorted(a, b []string) []string {
	for _, s := range b {
		if !slices.Contains(a, s) {
			a = append(a, s)
		}
	}
	slices.Sort(a)
	return a
}

///////////////////////////////////////////////////////////////////////////
// Runway entry and exit points

// IsRunwayNode reports whether the node is on a runway.
func (ap *Airport) IsRunwayNode(n NodeID) bool {
	return slices.ContainsFunc(ap.adjacency[n], func(ei int) bool { return ap.Edges[ei].Runway })
}

// runwayAxis returns the threshold of the given runway end and the unit
// vector pointing down the runway from it, in airport-local nm.
func (ap *Airport) runwayAxis(end string) (origin, dir [2]float32, ok bool) {
	rwy, idx, ok := ap.LookupRunway(end)
	if !ok {
		return
	}
	origin = ap.Local(rwy.Ends[idx].Threshold)
	dir = math.Normalize2f(math.Sub2f(ap.Local(rwy.Ends[1-idx].Threshold), origin))
	return
}

// RunwayConnection is a place where a taxiway joins a runway.
type RunwayConnection struct {
	// Node is the node on the runway.
	Node NodeID
	// Edge is the taxiway edge leaving the runway at Node.
	Edge int
	// Taxiway is the name of the taxiway edge.
	Taxiway string
	// Distance is the distance in nm from the runway end's threshold to
	// Node, measured along the runway.
	Distance float32
	// Angle is the angle in degrees between the direction of travel down
	// the runway and the taxiway; angles well under 90 degrees indicate
	// high-speed exits.
	Angle float32
	// Right is true if the taxiway leaves to the right of the direction
	// of travel.
	Right bool
}

// RunwayConnections returns the taxiways that join the given runway end's
// runway, ordered by distance from the end's threshold; the angles are
// measured relative to the direction of travel from that threshold. A
// taxiway that crosses the runway gives one connection for each side.
func (ap *Airport) RunwayConnections(end string) []RunwayConnection {
	rwy, _, ok := ap.LookupRunway(end)
	if !ok {
		return nil
	}
	origin, dir, _ := ap.runwayAxis(end)

	var conns []RunwayConnection
	for n := range ap.Nodes {
		node := NodeID(n)
		onRunway := slices.ContainsFunc(ap.adjacency[node], func(ei int) bool {
			e := ap.Edges[ei]
			return e.Runway && runwayMatches(e.Name, rwy.Id())
		})
		if !onRunway {
			continue
		}
		p := ap.Local(ap.Nodes[node].Location)
		dist := math.Dot(math.Sub2f(p, origin), dir)
		for _, ei := range ap.adjacency[node] {
			e := ap.Edges[ei]
			if e.Runway {
				continue
			}
			v := math.Normalize2f(math.Sub2f(ap.Local(ap.Nodes[e.Other(node)].Location), p))
			cosAngle := math.Clamp(math.Dot(v, dir), -1, 1)
			conns = append(conns, RunwayConnection{
				Node:     node,
				Edge:     ei,
				Taxiway:  e.Name,
				Distance: dist,
				Angle:    math.Degrees(math.SafeACos(cosAngle)),
				// With x east and y north, a negative cross product means
				// the taxiway is clockwise of (to the right of) the runway
				// direction.
				Right: dir[0]*v[1]-dir[1]*v[0] < 0,
			})
		}
	}
	slices.SortFunc(conns, func(a, b RunwayConnection) int {
		if a.Distance != b.Distance {
			if a.Distance < b.Distance {
				return -1
			}
			return 1
		}
		return int(a.Edge - b.Edge)
	})
	return conns
}

// DepartureEntry returns the node where an aircraft departing from the
// given runway end lines up for a full-length departure: the runway
// node with a taxiway connection that is closest to the threshold.
func (ap *Airport) DepartureEntry(end string) (NodeID, bool) {
	conns := ap.RunwayConnections(end)
	best, bestDist := NodeID(-1), float32(0)
	for _, c := range conns {
		if d := math.Abs(c.Distance); best == -1 || d < bestDist {
			best, bestDist = c.Node, d
		}
	}
	return best, best != -1
}

// GateNode returns the taxi network node closest to the gate that isn't
// on a runway or in a runway's protected area.
func (ap *Airport) GateNode(g Gate) (NodeID, bool) {
	n, _ := ap.NearestNode(g.Location, func(n NodeID) bool {
		return !slices.ContainsFunc(ap.adjacency[n], func(ei int) bool {
			return len(ap.Edges[ei].ProtectedRunwayEnds(false)) > 0
		})
	})
	return n, n != -1
}

///////////////////////////////////////////////////////////////////////////
// stateQueue

type stateCost struct {
	state int
	cost  float32
}

type stateQueue []stateCost

func (q stateQueue) Len() int           { return len(q) }
func (q stateQueue) Less(i, j int) bool { return q[i].cost < q[j].cost }
func (q stateQueue) Swap(i, j int)      { q[i], q[j] = q[j], q[i] }
func (q *stateQueue) Push(x any)        { *q = append(*q, x.(stateCost)) }
func (q *stateQueue) Pop() any {
	old := *q
	n := len(old)
	x := old[n-1]
	*q = old[:n-1]
	return x
}
