// asdex/asdex_test.go
// Copyright(c) vice contributors, licensed under the GNU Public License, Version 3.
// SPDX: GPL-3.0-only

package asdex

import (
	"slices"
	"testing"

	av "github.com/mmp/vice/aviation"
	"github.com/mmp/vice/aviation/db"
	"github.com/mmp/vice/math"
	"github.com/mmp/vice/sim"
	"github.com/mmp/vice/surface"
)

func TestTransforms(t *testing.T) {
	s := &Scope{Center: [2]float32{0.5, -0.25}, Range: 2}
	extent := math.Extent2D{P0: [2]float32{0, 0}, P1: [2]float32{1000, 500}}
	xf := s.transforms(extent)

	// The center of the view is the center of the window, and the top of
	// the window is Range nm north of it.
	if p := xf.windowFromLocal.TransformPoint(s.Center); math.Abs(p[0]-500) > 0.01 || math.Abs(p[1]-250) > 0.01 {
		t.Errorf("center: got window %v", p)
	}
	if p := xf.windowFromLocal.TransformPoint(math.Add2f(s.Center, [2]float32{0, 2})); math.Abs(p[1]-500) > 0.01 {
		t.Errorf("range: got window %v", p)
	}
	for _, p := range [][2]float32{{0, 0}, {1.3, -0.7}, {-2, 2}} {
		q := xf.localFromWindow.TransformPoint(xf.windowFromLocal.TransformPoint(p))
		if math.Distance2f(p, q) > 1e-4 {
			t.Errorf("round trip of %v gave %v", p, q)
		}
	}

	// Rotated 90 degrees, east is up on the display.
	s.Rotation = 90
	xf = s.transforms(extent)
	p := xf.windowFromLocal.TransformPoint(math.Add2f(s.Center, [2]float32{1, 0}))
	if math.Abs(p[0]-500) > 0.01 || p[1] <= 250 {
		t.Errorf("rotated 90: east of the center is at window %v, want straight up", p)
	}
}

func TestIsTaxiwayDesignator(t *testing.T) {
	for name, want := range map[string]bool{
		"A": true, "C3": true, "GA": true, "TM2": true, "E10": true,
		"": false, "ms": false, "Terminal": false, "ABC": false, "A12345": false,
	} {
		if got := isTaxiwayDesignator(name); got != want {
			t.Errorf("%q: got %v, want %v", name, got, want)
		}
	}
}

func TestTagSecondLine(t *testing.T) {
	fp := &sim.FlightPlan{AircraftType: "B738"}
	for _, tc := range []struct {
		t    target
		want string
	}{
		{target{trk: &sim.Track{FlightPlan: fp, ReadyForDeparture: true, TowerRunway: "35R"}, surface: true}, "B738 RDY 35R"},
		{target{trk: &sim.Track{FlightPlan: fp, ReadyForDeparture: true, LinedUp: true, TowerRunway: "35R"}, surface: true}, "B738 LUAW 35R"},
		{target{trk: &sim.Track{FlightPlan: fp, RadarTrack: av.RadarTrack{TrueAltitude: 6830}}}, "B738 068"},
		{target{trk: &sim.Track{FlightPlan: fp, RadarTrack: av.RadarTrack{TrueAltitude: 6830}, ClearedToLand: true}}, "B738 068 CLR"},
		{target{trk: &sim.Track{}, surface: true}, ""},
		{target{trk: &sim.Track{FlightPlan: fp, HoldingShortOf: "35L"}, surface: true}, "B738 HS 35L"},
		// Below radar coverage, on the takeoff roll and just airborne.
		{target{trk: &sim.Track{FlightPlan: fp, RadarTrack: av.RadarTrack{TrueAltitude: 6187}}, surface: true}, "B738"},
		{target{trk: &sim.Track{FlightPlan: fp, RadarTrack: av.RadarTrack{TrueAltitude: 6330}}, surface: true}, "B738 063"},
	} {
		if got := tagSecondLine(tc.t, 6187); got != tc.want {
			t.Errorf("got %q, want %q", got, tc.want)
		}
	}
}

func TestFormatRange(t *testing.T) {
	for r, want := range map[float32]string{0.25: ".25", 0.5: ".5", 1.5: "1.5", 2: "2", 12.34: "12.3"} {
		if got := formatRange(r); got != want {
			t.Errorf("%v: got %q, want %q", r, got, want)
		}
	}
}

func TestBuildGeometryKCOS(t *testing.T) {
	ap, err := surface.LoadAirport("KCOS")
	if err != nil {
		t.Fatal(err)
	}
	g := buildGeometry(ap)
	for _, kind := range drawOrder {
		if g.polygons[kind] == nil {
			t.Errorf("no %s polygons", kind)
		}
	}
	var runways, taxiways []string
	for _, l := range g.runwayLabels {
		runways = append(runways, l.text)
	}
	for _, l := range g.taxiwayLabels {
		taxiways = append(taxiways, l.text)
	}
	for _, want := range []string{"17L", "35R", "17R", "35L", "13", "31"} {
		if !slices.Contains(runways, want) {
			t.Errorf("no %s runway label: %v", want, runways)
		}
	}
	for _, want := range []string{"A", "C3", "E8"} {
		if !slices.Contains(taxiways, want) {
			t.Errorf("no %s taxiway label: %v", want, taxiways)
		}
	}
	// The runways span a couple of nm.
	if g.extent.Width() < 1 || g.extent.Height() < 1 || g.extent.Width() > 5 || g.extent.Height() > 5 {
		t.Errorf("runway extent %v", g.extent)
	}
}

// An airport without surface data gets its runways from the aviation
// database.
func TestLoadAirportFallback(t *testing.T) {
	db.InitDB()
	if surface.HasAirport("KFLY") {
		t.Skip("KFLY has surface data")
	}
	ap, err := loadAirport("KFLY")
	if err != nil {
		t.Fatal(err)
	}
	if len(ap.Runways) == 0 {
		t.Fatal("no runways")
	}
	g := buildGeometry(ap)
	if g.polygons[surface.PolygonRunway] == nil || len(g.runwayLabels) != 2*len(ap.Runways) {
		t.Errorf("fallback geometry: %d runway labels for %d runways", len(g.runwayLabels), len(ap.Runways))
	}
	if _, err := loadAirport("KZZZ"); err == nil {
		t.Errorf("expected an error for an unknown airport")
	}
}
