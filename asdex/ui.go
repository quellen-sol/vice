// asdex/ui.go
// Copyright(c) vice contributors, licensed under the GNU Public License, Version 3.
// SPDX: GPL-3.0-only

package asdex

import (
	"github.com/mmp/vice/platform"

	"github.com/AllenDang/cimgui-go/imgui"
)

func (s *Scope) DisplayName() string { return "ASDE-X" }

func (s *Scope) DrawUI(p platform.Platform, config *platform.Config) {
	imgui.Checkbox("Show taxiway labels", &s.ShowTaxiwayLabels)
	imgui.SliderFloatV("Rotation (degrees)", &s.Rotation, -180, 180, "%.0f", 0)
	imgui.SliderFloatV("Airborne target range (nm)", &s.RangeFilter, 1, 30, "%.0f", 0)
	alt := int32(s.AltitudeFilter)
	if imgui.SliderIntV("Airborne target ceiling (ft above field)", &alt, 500, 10000, "%d", 0) {
		s.AltitudeFilter = int(alt)
	}
	if imgui.Button("Reset view") {
		s.ViewAirport = "" // fit the view to the airport on the next draw
		s.Rotation = 0
	}
	imgui.Text("Scroll to zoom, drag with the right button to pan, click an aircraft to select it,")
	imgui.Text("and type commands (e.g. \"SWA123 CTO\") followed by Enter.")
}
