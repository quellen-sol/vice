// surface/aptdat.go
// Copyright(c) vice contributors, licensed under the GNU Public License, Version 3.
// SPDX: GPL-3.0-only

package surface

import (
	"bufio"
	"fmt"
	"io"
	"strconv"
	"strings"

	"github.com/mmp/vice/math"
)

const metersToFeet = 3.28084

// ParseAptDat reads the given airport from an X-Plane apt.dat file
// (format 1100 or later; see
// https://developer.x-plane.com/article/airport-data-apt-dat-12-00-file-format-specification/).
// It extracts the land runways, the taxi routing network with its runway
// active zones, the ramp start positions and the tower viewpoint.
// Pavement polygons are not read; surface polygons come from CRC ASDE-X
// maps or are synthesized from the runways.
func ParseAptDat(r io.Reader, icao string) (*Airport, error) {
	type rawEdge struct {
		a, b int
		edge Edge
	}
	var (
		ap        *Airport
		inAirport bool
		nodeIndex = make(map[int]NodeID) // apt.dat node id -> index in ap.Nodes
		edges     []rawEdge
		lineNum   int
	)

	errorf := func(format string, args ...any) error {
		return fmt.Errorf("%s: line %d: %s", icao, lineNum, fmt.Sprintf(format, args...))
	}
	parseLatLong := func(lat, lon string) (math.Point2LL, error) {
		la, err := strconv.ParseFloat(lat, 64)
		if err != nil {
			return math.Point2LL{}, errorf("%s: invalid latitude", lat)
		}
		lo, err := strconv.ParseFloat(lon, 64)
		if err != nil {
			return math.Point2LL{}, errorf("%s: invalid longitude", lon)
		}
		return math.Point2LL{float32(lo), float32(la)}, nil
	}

	scanner := bufio.NewScanner(r)
	scanner.Buffer(make([]byte, 0, 64*1024), 1024*1024)
lines:
	for scanner.Scan() {
		lineNum++
		f := strings.Fields(scanner.Text())
		if len(f) == 0 {
			continue
		}

		switch f[0] {
		case "1", "16", "17": // land airport, seaplane base, heliport headers
			if inAirport {
				// We've reached the next airport and are done.
				break lines
			}
			if f[0] == "1" && len(f) >= 5 && f[4] == icao {
				elev, err := strconv.ParseFloat(f[1], 32)
				if err != nil {
					return nil, errorf("%s: invalid elevation", f[1])
				}
				ap = &Airport{ICAO: icao, ElevationFt: float32(elev)}
				inAirport = true
			}
			continue
		case "99": // end of file
			break lines
		}
		if !inAirport {
			continue
		}

		switch f[0] {
		case "100": // land runway
			// 100 width surface shoulder smoothness centerline-lights
			// edge-lights distance-signs, then 9 fields for each end:
			// id lat lon displaced-threshold blastpad markings
			// approach-lights tdz reil.
			if len(f) < 8+2*9 {
				return nil, errorf("runway: expected %d fields, got %d", 8+2*9, len(f))
			}
			width, err := strconv.ParseFloat(f[1], 32)
			if err != nil {
				return nil, errorf("%s: invalid runway width", f[1])
			}
			rwy := Runway{WidthFt: float32(width * metersToFeet)}
			for i := range 2 {
				ef := f[8+9*i:]
				p, err := parseLatLong(ef[1], ef[2])
				if err != nil {
					return nil, err
				}
				displaced, err := strconv.ParseFloat(ef[3], 32)
				if err != nil {
					return nil, errorf("%s: invalid displaced threshold", ef[3])
				}
				rwy.Ends[i] = RunwayEnd{
					Id:                   ef[0],
					Threshold:            p,
					DisplacedThresholdFt: float32(displaced * metersToFeet),
				}
			}
			ap.Runways = append(ap.Runways, rwy)

		case "14": // tower viewpoint: 14 lat lon height draw-flag name
			if len(f) < 3 {
				return nil, errorf("tower viewpoint: too few fields")
			}
			p, err := parseLatLong(f[1], f[2])
			if err != nil {
				return nil, err
			}
			ap.TowerLocation = p

		case "1201": // taxi network node: 1201 lat lon usage id [name]
			if len(f) < 5 {
				return nil, errorf("taxi node: too few fields")
			}
			p, err := parseLatLong(f[1], f[2])
			if err != nil {
				return nil, err
			}
			id, err := strconv.Atoi(f[4])
			if err != nil {
				return nil, errorf("%s: invalid taxi node id", f[4])
			}
			if _, ok := nodeIndex[id]; ok {
				return nil, errorf("%d: repeated taxi node id", id)
			}
			nodeIndex[id] = NodeID(len(ap.Nodes))
			ap.Nodes = append(ap.Nodes, Node{Location: p, Name: strings.Join(f[5:], " ")})

		case "1202": // taxi network edge: 1202 a b oneway|twoway runway|taxiway[_X] [name]
			if len(f) < 5 {
				return nil, errorf("taxi edge: too few fields")
			}
			a, errA := strconv.Atoi(f[1])
			b, errB := strconv.Atoi(f[2])
			if errA != nil || errB != nil {
				return nil, errorf("%s %s: invalid taxi edge node ids", f[1], f[2])
			}
			e := Edge{
				OneWay: f[3] == "oneway",
				Name:   strings.Join(f[5:], " "),
			}
			switch {
			case f[4] == "runway":
				e.Runway = true
			case strings.HasPrefix(f[4], "taxiway_"):
				e.WidthCode = strings.TrimPrefix(f[4], "taxiway_")
			case f[4] == "taxiway":
			default:
				return nil, errorf("%s: unexpected taxi edge type", f[4])
			}
			edges = append(edges, rawEdge{a: a, b: b, edge: e})

		case "1204": // active zone for the preceding edge: 1204 kind rwy,rwy,...
			if len(f) < 3 {
				return nil, errorf("active zone: too few fields")
			}
			if len(edges) == 0 {
				return nil, errorf("active zone without a preceding taxi edge")
			}
			kind := ZoneKind(f[1])
			if kind != ZoneDeparture && kind != ZoneArrival && kind != ZoneILS {
				return nil, errorf("%s: unexpected active zone type", f[1])
			}
			e := &edges[len(edges)-1].edge
			e.Zones = append(e.Zones, ActiveZone{Kind: kind, RunwayEnds: strings.Split(f[2], ",")})

		case "1300": // ramp start: 1300 lat lon heading kind types name
			if len(f) < 6 {
				return nil, errorf("ramp start: too few fields")
			}
			p, err := parseLatLong(f[1], f[2])
			if err != nil {
				return nil, err
			}
			hdg, err := strconv.ParseFloat(f[3], 32)
			if err != nil {
				return nil, errorf("%s: invalid ramp start heading", f[3])
			}
			ap.Gates = append(ap.Gates, Gate{
				Name:     strings.Join(f[6:], " "),
				Location: p,
				Heading:  math.TrueHeading(math.NormalizeHeading(float32(hdg))),
				Kind:     f[4],
				Types:    strings.Split(f[5], "|"),
			})

		case "1301": // ramp start details for the preceding 1300: 1301 size operation [airlines]
			if len(ap.Gates) == 0 {
				return nil, errorf("ramp start details without a preceding ramp start")
			}
			g := &ap.Gates[len(ap.Gates)-1]
			if len(f) >= 2 {
				g.SizeCode = f[1]
			}
			if len(f) >= 4 {
				for al := range strings.SplitSeq(f[3], ",") {
					if al = strings.ToUpper(strings.TrimSpace(al)); al != "" {
						g.Airlines = append(g.Airlines, al)
					}
				}
			}

		case "1302": // metadata: 1302 key value
			if len(f) >= 3 {
				switch f[1] {
				case "datum_lat":
					if v, err := strconv.ParseFloat(f[2], 32); err == nil {
						ap.Reference[1] = float32(v)
					}
				case "datum_lon":
					if v, err := strconv.ParseFloat(f[2], 32); err == nil {
						ap.Reference[0] = float32(v)
					}
				}
			}
		}
	}
	if err := scanner.Err(); err != nil {
		return nil, err
	}
	if ap == nil {
		return nil, fmt.Errorf("%s: airport not found", icao)
	}
	if len(ap.Runways) == 0 {
		return nil, fmt.Errorf("%s: no land runways found", icao)
	}
	if len(ap.Nodes) == 0 {
		return nil, fmt.Errorf("%s: airport has no taxi routing network", icao)
	}

	// Ground vehicle routes (1206 rows, which we ignore) share the nodes
	// of the taxi network, so only keep the nodes that aircraft taxi
	// edges use.
	used := make([]bool, len(ap.Nodes))
	for _, re := range edges {
		a, okA := nodeIndex[re.a]
		b, okB := nodeIndex[re.b]
		if !okA || !okB {
			return nil, fmt.Errorf("%s: taxi edge %s references undefined node(s) %d/%d", icao, re.edge.Name, re.a, re.b)
		}
		used[a], used[b] = true, true
	}
	remap := make([]NodeID, len(ap.Nodes))
	var nodes []Node
	for i, n := range ap.Nodes {
		if used[i] {
			remap[i] = NodeID(len(nodes))
			nodes = append(nodes, n)
		}
	}
	ap.Nodes = nodes
	if len(ap.Nodes) == 0 {
		return nil, fmt.Errorf("%s: airport has no taxi routing network", icao)
	}
	for _, re := range edges {
		re.edge.A, re.edge.B = remap[nodeIndex[re.a]], remap[nodeIndex[re.b]]
		ap.Edges = append(ap.Edges, re.edge)
	}
	if ap.Reference[0] == 0 || ap.Reference[1] == 0 {
		// No datum given; use the midpoint of the first runway.
		ap.Reference = math.Mid2LL(ap.Runways[0].Ends[0].Threshold, ap.Runways[0].Ends[1].Threshold)
	}
	if ap.TowerLocation.IsZero() {
		ap.TowerLocation = ap.Reference
	}

	return ap, ap.Finalize()
}

// RenameRunwayEnd changes the identifier of a runway end everywhere it is
// used: in the runway itself, runway edge names and active zones. This is
// used to match apt.dat runway identifiers to those in the FAA CIFP when
// a runway has been renumbered.
func (ap *Airport) RenameRunwayEnd(from, to string) {
	for i := range ap.Runways {
		for j := range ap.Runways[i].Ends {
			if ap.Runways[i].Ends[j].Id == from {
				ap.Runways[i].Ends[j].Id = to
			}
		}
	}
	for i := range ap.Edges {
		e := &ap.Edges[i]
		if e.Runway {
			ends := strings.Split(e.Name, "/")
			for j := range ends {
				if ends[j] == from {
					ends[j] = to
				}
			}
			e.Name = strings.Join(ends, "/")
		}
		for j := range e.Zones {
			for k := range e.Zones[j].RunwayEnds {
				if e.Zones[j].RunwayEnds[k] == from {
					e.Zones[j].RunwayEnds[k] = to
				}
			}
		}
	}
}
