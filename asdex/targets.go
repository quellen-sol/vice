// asdex/targets.go
// Copyright(c) vice contributors, licensed under the GNU Public License, Version 3.
// SPDX: GPL-3.0-only

package asdex

import (
	"fmt"

	av "github.com/mmp/vice/aviation"
	"github.com/mmp/vice/math"
	"github.com/mmp/vice/renderer"
	"github.com/mmp/vice/scope"
	"github.com/mmp/vice/sim"
	"github.com/mmp/vice/util"
)

// target is an aircraft shown on the display.
type target struct {
	trk     *sim.Track
	p       [2]float32 // airport-local nm
	heading math.TrueHeading
	surface bool // on the ground, not radar visible
}

var (
	departureColor = renderer.RGBFromHex(0x5ec8ff)
	arrivalColor   = renderer.RGBFromHex(0x8ee08e)
	otherColor     = renderer.RGBFromHex(0xe0e0e0)
	selectedColor  = renderer.RGBFromHex(0xffd54f)
)

// targets returns the aircraft to show: those on the ground that the tower
// is working and those in the air close to the airport and low.
func (s *Scope) targets(ctx *scope.Context) []target {
	state := ctx.Client.State
	var targets []target
	add := func(trk *sim.Track, surface bool) {
		targets = append(targets, target{
			trk:     trk,
			p:       s.airport.Local(trk.Location),
			heading: math.MagneticToTrue(trk.Heading, ctx.MagneticVariation),
			surface: surface,
		})
	}

	for _, trk := range util.SortedMap(state.Tracks) {
		if trk.IsUnsupportedDB() || trk.Location.IsZero() {
			continue
		}
		if math.Length2f(s.airport.Local(trk.Location)) > s.RangeFilter ||
			trk.TrueAltitude > s.airport.ElevationFt+float32(s.AltitudeFilter) {
			continue
		}
		add(trk, false)
	}
	for _, trk := range util.SortedMap(state.SurfaceTracks) {
		add(trk, true)
	}
	return targets
}

// targetAt returns the aircraft drawn closest to the window position p, if
// one is close enough to have been clicked.
func (s *Scope) targetAt(ctx *scope.Context, xf transforms, p [2]float32) (av.ADSBCallsign, bool) {
	var best av.ADSBCallsign
	bestDist := 15 * ctx.DrawPixelScale
	for _, t := range s.targets(ctx) {
		if d := math.Distance2f(xf.windowFromLocal.TransformPoint(t.p), p); d < bestDist {
			best, bestDist = t.trk.ADSBCallsign, d
		}
	}
	return best, best != ""
}

// airplane is a plan view of an aircraft, nose up, a unit long and wide.
var airplane = [][3][2]float32{
	// Fuselage and nose
	{{-0.07, -0.45}, {0.07, -0.45}, {0.07, 0.38}},
	{{-0.07, -0.45}, {0.07, 0.38}, {-0.07, 0.38}},
	{{-0.07, 0.38}, {0.07, 0.38}, {0, 0.5}},
	// Wings
	{{0.07, 0.2}, {0.5, -0.02}, {0.5, -0.1}},
	{{0.07, 0.2}, {0.5, -0.1}, {0.07, 0.04}},
	{{-0.07, 0.2}, {-0.5, -0.1}, {-0.5, -0.02}},
	{{-0.07, 0.2}, {-0.07, 0.04}, {-0.5, -0.1}},
	// Horizontal stabilizer
	{{0.05, -0.32}, {0.2, -0.45}, {0.2, -0.5}},
	{{0.05, -0.32}, {0.2, -0.5}, {0.05, -0.42}},
	{{-0.05, -0.32}, {-0.2, -0.5}, {-0.2, -0.45}},
	{{-0.05, -0.32}, {-0.05, -0.42}, {-0.2, -0.5}},
}

// iconSize returns the length in pixels of the aircraft's icon, by its
// wake turbulence category.
func iconSize(trk *sim.Track) float32 {
	switch trk.CWTCategory {
	case "A", "B", "C":
		return 26
	case "G", "H", "I":
		return 15
	default:
		return 20
	}
}

func (s *Scope) targetColor(t target) renderer.RGB {
	switch {
	case t.trk.ADSBCallsign == s.selected:
		return selectedColor
	case t.trk.TypeOfFlight == av.FlightTypeDeparture:
		return departureColor
	case t.trk.TypeOfFlight == av.FlightTypeArrival:
		return arrivalColor
	default:
		return otherColor
	}
}

func (s *Scope) drawTargets(ctx *scope.Context, cb *renderer.CommandBuffer, xf transforms, targets []target,
	td *renderer.TextDrawBuilder) {
	trid := renderer.GetColoredTrianglesDrawBuilder()
	defer renderer.ReturnColoredTrianglesDrawBuilder(trid)
	ld := renderer.GetColoredLinesDrawBuilder()
	defer renderer.ReturnColoredLinesDrawBuilder(ld)

	scale := ctx.DrawPixelScale
	lineHeight := float32(s.tagFont.Size) + 2*scale
	// Aircraft waiting at the same spot (several departures holding short
	// of a runway) have their data tags stacked rather than overprinted.
	tagsAt := make(map[[2]int]int)

	for _, t := range targets {
		color := s.targetColor(t)
		p := xf.windowFromLocal.TransformPoint(t.p)

		// The icon points along the aircraft's heading on the display.
		rot := math.Rotator2f(float32(t.heading) - s.Rotation)
		size := iconSize(t.trk) * scale
		for _, tri := range airplane {
			var w [3][2]float32
			for i, v := range tri {
				w[i] = math.Add2f(p, rot(math.Scale2f(v, size)))
			}
			trid.AddTriangle(w[0], w[1], w[2], color)
		}

		// The data tag goes up and to the right, with a leader line.
		key := [2]int{int(p[0]), int(p[1])}
		stack := tagsAt[key]
		tagsAt[key]++
		tag := math.Add2f(p, [2]float32{size*0.6 + 8*scale, size*0.6 + 8*scale + float32(stack)*2*lineHeight})
		ld.AddLine(math.Add2f(p, [2]float32{size * 0.4, size * 0.4}), math.Add2f(tag, [2]float32{-3 * scale, -lineHeight / 2}), color)
		style := renderer.TextStyle{Font: s.tagFont, Color: color}
		td.AddText(tagCallsign(t.trk), tag, style)
		td.AddText(tagSecondLine(t, s.airport.ElevationFt), math.Add2f(tag, [2]float32{0, -lineHeight}), style)
	}

	cb.LineWidth(1, scale)
	trid.GenerateCommands(cb)
	ld.GenerateCommands(cb)
}

func tagCallsign(trk *sim.Track) string {
	if trk.FlightPlan != nil && trk.FlightPlan.ACID != "" {
		return string(trk.FlightPlan.ACID)
	}
	return string(trk.ADSBCallsign)
}

// tagSecondLine returns the second line of the aircraft's data tag: its type
// and, waiting for departure, where it is in the sequence or, in the air,
// its altitude in hundreds of feet.
func tagSecondLine(t target, fieldElevation float32) string {
	var actype string
	if fp := t.trk.FlightPlan; fp != nil {
		actype = fp.AircraftType
	}
	trk := t.trk
	switch {
	case t.surface && trk.LinedUp:
		return actype + " LUAW " + trk.TowerRunway
	case t.surface && trk.ReadyForDeparture:
		return actype + " RDY " + trk.TowerRunway
	case t.surface && trk.TrueAltitude < fieldElevation+50:
		return actype // on the ground
	}
	line := fmt.Sprintf("%s %03d", actype, int(trk.TrueAltitude+50)/100)
	if trk.ClearedToLand {
		line += " CLR"
	}
	return line
}
