// asdex/geometry.go
// Copyright(c) vice contributors, licensed under the GNU Public License, Version 3.
// SPDX: GPL-3.0-only

package asdex

import (
	"fmt"
	"slices"
	"strconv"
	"strings"
	"unicode"

	av "github.com/mmp/vice/aviation"
	"github.com/mmp/vice/aviation/db"
	"github.com/mmp/vice/math"
	"github.com/mmp/vice/renderer"
	"github.com/mmp/vice/scope"
	"github.com/mmp/vice/surface"
	"github.com/mmp/vice/util"

	"github.com/mmp/earcut"
)

// loadAirport returns the airport's surface data. Airports without it get
// just their runways, from the aviation database, 150' wide.
func loadAirport(icao av.ICAOAirportCode) (*surface.Airport, error) {
	if surface.HasAirport(string(icao)) {
		return surface.LoadAirport(string(icao))
	}

	dbap, ok := db.DB.Airports[icao]
	if !ok {
		return nil, fmt.Errorf("%s: unknown airport", icao)
	}
	ap := &surface.Airport{
		ICAO:          string(icao),
		ElevationFt:   float32(dbap.Elevation),
		Reference:     dbap.Location,
		TowerLocation: dbap.Location,
	}
	seen := make(map[string]bool)
	for _, rwy := range dbap.Runways {
		opp, ok := av.LookupOppositeRunway(db.Lookups{}, icao, rwy.Id)
		if !ok || seen[rwy.Id] || seen[opp.Id] {
			continue
		}
		seen[rwy.Id], seen[opp.Id] = true, true
		ap.Runways = append(ap.Runways, surface.Runway{
			Ends:    [2]surface.RunwayEnd{{Id: rwy.Id, Threshold: rwy.Threshold}, {Id: opp.Id, Threshold: opp.Threshold}},
			WidthFt: 150,
		})
	}
	if len(ap.Runways) == 0 {
		return nil, fmt.Errorf("%s: no runways", icao)
	}
	// Finalize needs a node to fall back on for the reference point, but
	// the reference is set, so an empty taxi network is fine.
	if err := ap.Finalize(); err != nil {
		return nil, err
	}
	return ap, nil
}

type label struct {
	p    [2]float32 // airport-local nm
	text string
}

// surfaceGeometry is the airport's surface, ready to draw: its polygons
// triangulated in airport-local coordinates, by kind, and its labels.
type surfaceGeometry struct {
	polygons      map[surface.PolygonKind]*renderer.CommandBuffer
	runwayLabels  []label
	taxiwayLabels []label
	extent        math.Extent2D // of the runways, in airport-local nm
}

var drawOrder = []surface.PolygonKind{surface.PolygonApron, surface.PolygonTaxiway,
	surface.PolygonRunway, surface.PolygonStructure}

func buildGeometry(ap *surface.Airport) surfaceGeometry {
	g := surfaceGeometry{
		polygons: make(map[surface.PolygonKind]*renderer.CommandBuffer),
		extent:   math.EmptyExtent2D(),
	}

	polygons := ap.Polygons
	if !slices.ContainsFunc(polygons, func(p surface.Polygon) bool { return p.Kind == surface.PolygonRunway }) {
		// No surface polygons for the runways; draw their outlines.
		for _, rwy := range ap.Runways {
			outline := ap.RunwayOutline(rwy)
			polygons = append(polygons, surface.Polygon{Kind: surface.PolygonRunway, Rings: [][]math.Point2LL{outline[:]}})
		}
	}

	td := renderer.GetTrianglesDrawBuilder()
	defer renderer.ReturnTrianglesDrawBuilder(td)
	for _, kind := range drawOrder {
		td.Reset()
		n := 0
		for _, poly := range polygons {
			if poly.Kind != kind {
				continue
			}
			for _, tri := range triangulate(ap, poly) {
				td.AddTriangle(tri[0], tri[1], tri[2])
				n++
			}
		}
		if n > 0 {
			cb := &renderer.CommandBuffer{}
			td.GenerateCommands(cb)
			g.polygons[kind] = cb
		}
	}

	for _, rwy := range ap.Runways {
		for i, end := range rwy.Ends {
			p := ap.Local(end.Threshold)
			g.extent = math.Union(g.extent, p)
			// Put the label a little past the end of the runway.
			out := math.Normalize2f(math.Sub2f(p, ap.Local(rwy.Ends[1-i].Threshold)))
			g.runwayLabels = append(g.runwayLabels, label{p: math.Add2f(p, math.Scale2f(out, 0.05)), text: end.Id})
		}
	}

	g.taxiwayLabels = taxiwayLabels(ap)
	return g
}

// triangulate returns the polygon's triangles in airport-local coordinates.
func triangulate(ap *surface.Airport, poly surface.Polygon) [][3][2]float32 {
	var rings [][]earcut.Vertex
	for _, ring := range poly.Rings {
		r := make([]earcut.Vertex, len(ring))
		for i, p := range ring {
			l := ap.Local(p)
			r[i].P = [2]float64{float64(l[0]), float64(l[1])}
		}
		rings = append(rings, r)
	}
	var tris [][3][2]float32
	for _, tri := range earcut.Triangulate(earcut.Polygon{Rings: rings}) {
		var t [3][2]float32
		for i, v := range tri.Vertices {
			t[i] = [2]float32{float32(v.P[0]), float32(v.P[1])}
		}
		tris = append(tris, t)
	}
	return tris
}

// taxiwayLabels returns a label for each taxiway in the taxi network, at the
// middle of its longest segment. Names that aren't taxiway designators (the
// scenery's own names for ramp lanes and the like) are skipped.
func taxiwayLabels(ap *surface.Airport) []label {
	longest := make(map[string]int)
	for i, e := range ap.Edges {
		if e.Runway || !isTaxiwayDesignator(e.Name) {
			continue
		}
		if j, ok := longest[e.Name]; !ok || ap.EdgeLength(i) > ap.EdgeLength(j) {
			longest[e.Name] = i
		}
	}
	var labels []label
	for name, i := range util.SortedMap(longest) {
		e := ap.Edges[i]
		mid := math.Scale2f(math.Add2f(ap.Local(ap.Nodes[e.A].Location), ap.Local(ap.Nodes[e.B].Location)), 0.5)
		labels = append(labels, label{p: mid, text: name})
	}
	return labels
}

// isTaxiwayDesignator reports whether name looks like a taxiway designator:
// one or two capital letters, optionally followed by digits ("A", "C3",
// "GA").
func isTaxiwayDesignator(name string) bool {
	letters := strings.TrimRightFunc(name, unicode.IsDigit)
	if len(letters) == 0 || len(letters) > 2 || len(name) > 4 {
		return false
	}
	for _, r := range letters {
		if r < 'A' || r > 'Z' {
			return false
		}
	}
	return true
}

func (g surfaceGeometry) draw(cb *renderer.CommandBuffer) {
	for _, kind := range drawOrder {
		if sub, ok := g.polygons[kind]; ok {
			cb.SetRGB(surfaceColors[kind])
			cb.Call(*sub)
		}
	}
}

func (s *Scope) drawLabels(ctx *scope.Context, xf transforms, td *renderer.TextDrawBuilder) {
	for _, l := range s.geometry.runwayLabels {
		td.AddTextCentered(l.text, xf.windowFromLocal.TransformPoint(l.p),
			renderer.TextStyle{Font: s.tagFont, Color: runwayLabelColor})
	}
	// Taxiway labels are only useful (and only fit) zoomed in.
	if s.ShowTaxiwayLabels && s.Range < 2.5 {
		for _, l := range s.geometry.taxiwayLabels {
			td.AddTextCentered(l.text, xf.windowFromLocal.TransformPoint(l.p),
				renderer.TextStyle{Font: s.labelFont, Color: taxiwayLabelColor})
		}
	}
}

// trimFloat formats v with at most the given number of digits after the
// decimal point, dropping trailing zeros.
func trimFloat(v float32, digits int) string {
	s := strconv.FormatFloat(float64(v), 'f', digits, 32)
	if strings.Contains(s, ".") {
		s = strings.TrimRight(strings.TrimRight(s, "0"), ".")
	}
	return s
}
