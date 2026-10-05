// asdex/asdex.go
// Copyright(c) vice contributors, licensed under the GNU Public License, Version 3.
// SPDX: GPL-3.0-only

// Package asdex implements an ASDE-X (Airport Surface Detection Equipment,
// Model X) display for tower controllers: a top-down view of the airport's
// runways, taxiways, aprons and buildings with the aircraft on and around
// it, updated every second.
package asdex

import (
	"strings"
	"time"

	av "github.com/mmp/vice/aviation"
	"github.com/mmp/vice/client"
	"github.com/mmp/vice/log"
	"github.com/mmp/vice/math"
	"github.com/mmp/vice/platform"
	"github.com/mmp/vice/renderer"
	"github.com/mmp/vice/scope"
	"github.com/mmp/vice/surface"

	"github.com/AllenDang/cimgui-go/imgui"
)

type Scope struct {
	// The view. Center is in nm east and north of the airport's
	// reference point; Range is the distance in nm from the center to the
	// top of the display; Rotation is the true heading (degrees) that
	// points up on the display.
	Center   [2]float32
	Range    float32
	Rotation float32
	// ViewAirport is the airport the view was set up for; it's reset when
	// the user works a different airport's tower.
	ViewAirport av.ICAOAirportCode

	ShowTaxiwayLabels bool
	// Airborne aircraft are shown if they're within RangeFilter nm of the
	// airport and less than AltitudeFilter feet above it.
	RangeFilter    float32
	AltitudeFilter int

	airport  *surface.Airport
	geometry surfaceGeometry

	labelFont, tagFont, textFont *renderer.Font

	// The command line: aircraft commands typed at the display.
	input         string
	feedback      string
	feedbackError bool
	feedbackTime  time.Time
	selected      av.ADSBCallsign

	lg *log.Logger
}

func NewScope() *Scope {
	return &Scope{
		Range:             1.5,
		ShowTaxiwayLabels: true,
		RangeFilter:       10,
		AltitudeFilter:    3000,
	}
}

func (s *Scope) Activate(r renderer.Renderer, p platform.Platform, lg *log.Logger) {
	s.lg = lg
	fonts := scope.CreateERAMFonts(r, p.DPIScale())
	s.labelFont = scope.FindERAMFont(fonts, "EramText-11.pcf", 13)
	s.tagFont = scope.FindERAMFont(fonts, "EramText-14.pcf", 17)
	s.textFont = scope.FindERAMFont(fonts, "EramText-14.pcf", 17)
	if s.Range == 0 {
		*s = *NewScope()
		s.lg = lg
		s.labelFont, s.tagFont, s.textFont =
			scope.FindERAMFont(fonts, "EramText-11.pcf", 13),
			scope.FindERAMFont(fonts, "EramText-14.pcf", 17),
			scope.FindERAMFont(fonts, "EramText-14.pcf", 17)
	}
}

func (s *Scope) LoadedSim(c *client.ControlClient, pl platform.Platform, lg *log.Logger) {
	s.airport = nil
}

func (s *Scope) ResetSim(c *client.ControlClient, pl platform.Platform, lg *log.Logger) {
	s.airport = nil
	s.ViewAirport = "" // fit the view to the airport
	s.input, s.feedback, s.selected = "", "", ""
}

func (s *Scope) CanTakeKeyboardFocus() bool { return true }

// UserTowerAirport returns the airport whose tower the user works, if any.
func UserTowerAirport(c *client.ControlClient) av.ICAOAirportCode {
	if c == nil {
		return ""
	}
	for _, pos := range c.State.GetPositionsForTCW(c.State.UserTCW) {
		if ctrl, ok := c.State.Controllers[pos]; ok && ctrl.Role == av.RoleLocal {
			return ctrl.Airport
		}
	}
	return ""
}

// Colors
var (
	backgroundColor = renderer.RGBFromHex(0x101820)
	surfaceColors   = map[surface.PolygonKind]renderer.RGB{
		surface.PolygonApron:     renderer.RGBFromHex(0x2a3440),
		surface.PolygonTaxiway:   renderer.RGBFromHex(0x3c4652),
		surface.PolygonRunway:    renderer.RGBFromHex(0x5c6670),
		surface.PolygonStructure: renderer.RGBFromHex(0x4a5a78),
	}
	runwayLabelColor  = renderer.RGBFromHex(0xf0f0f0)
	taxiwayLabelColor = renderer.RGBFromHex(0xd8c070)
	textColor         = renderer.RGBFromHex(0xc8d0d8)
	errorColor        = renderer.RGBFromHex(0xff6060)
)

func (s *Scope) Draw(ctx *scope.Context, cb *renderer.CommandBuffer) {
	icao := UserTowerAirport(ctx.Client)
	if icao == "" {
		s.drawMessage(ctx, cb, "ASDE-X: not working a tower position")
		return
	}
	if s.airport == nil || s.airport.ICAO != string(icao) {
		ap, err := loadAirport(icao)
		if err != nil {
			s.drawMessage(ctx, cb, "ASDE-X: "+err.Error())
			return
		}
		s.airport, s.geometry = ap, buildGeometry(ap)
	}
	if s.ViewAirport != icao {
		s.fitView(ctx)
		s.ViewAirport = icao
	}

	xf := s.transforms(ctx.DrawExtent)
	s.handleMouse(ctx, xf)
	xf = s.transforms(ctx.DrawExtent) // the view may have changed
	s.handleKeyboard(ctx)

	cb.ClearRGB(backgroundColor)

	// The surface is drawn in airport-local coordinates.
	cb.LoadProjectionMatrix(xf.ndcFromLocal)
	cb.LoadModelViewMatrix(math.Identity3x3())
	s.geometry.draw(cb)

	// Everything else is drawn in window coordinates.
	cb.LoadProjectionMatrix(xf.ndcFromWindow)
	cb.LoadModelViewMatrix(math.Identity3x3())

	td := renderer.GetTextDrawBuilder()
	defer renderer.ReturnTextDrawBuilder(td)
	s.drawLabels(ctx, xf, td)

	targets := s.targets(ctx)
	s.drawTargets(ctx, cb, xf, targets, td)
	s.drawStatus(ctx, td)

	td.GenerateCommands(cb)
}

// fitView centers the view on the airport's runways, zoomed to show them.
func (s *Scope) fitView(ctx *scope.Context) {
	ext := s.geometry.extent
	if ext.IsEmpty() {
		s.Center, s.Range = [2]float32{}, 1.5
		return
	}
	s.Center = ext.Center()
	aspect := ctx.DrawExtent.Width() / max(1, ctx.DrawExtent.Height())
	s.Range = 0.55 * max(ext.Height(), ext.Width()/max(aspect, 0.1))
	s.Range = math.Clamp(s.Range, minRange, maxRange)
}

const (
	minRange = 0.1
	maxRange = 15
)

type transforms struct {
	ndcFromLocal, ndcFromWindow      math.Matrix3
	windowFromLocal, localFromWindow math.Matrix3
}

func (s *Scope) transforms(extent math.Extent2D) transforms {
	w, h := max(extent.Width(), 1), max(extent.Height(), 1)
	aspect := w / h
	ndcFromLocal := math.Identity3x3().
		Ortho(-aspect, aspect, -1, 1).
		// Turn the world counterclockwise so that true heading Rotation
		// points up.
		Rotate(math.Radians(s.Rotation)).
		Scale(1/s.Range, 1/s.Range).
		Translate(-s.Center[0], -s.Center[1])
	ndcFromWindow := math.Identity3x3().
		Translate(-1, -1).
		Scale(2/w, 2/h)
	localFromWindow := ndcFromLocal.Inverse().PostMultiply(ndcFromWindow)
	return transforms{
		ndcFromLocal:    ndcFromLocal,
		ndcFromWindow:   ndcFromWindow,
		windowFromLocal: localFromWindow.Inverse(),
		localFromWindow: localFromWindow,
	}
}

func (s *Scope) handleMouse(ctx *scope.Context, xf transforms) {
	mouse := ctx.Mouse
	if mouse == nil {
		return
	}
	if mouse.Clicked[platform.MouseButtonPrimary] && !ctx.HaveFocus {
		ctx.KeyboardFocus.Take(s)
	}

	// Right-drag pans.
	if mouse.Dragging[platform.MouseButtonSecondary] {
		if d := mouse.DragDelta; d[0] != 0 || d[1] != 0 {
			s.Center = math.Sub2f(s.Center, xf.localFromWindow.TransformVector(d))
		}
	}

	// The wheel zooms about the mouse position.
	if wheel := mouse.Wheel[1]; wheel != 0 {
		p := xf.localFromWindow.TransformPoint(mouse.Pos)
		r := math.Clamp(s.Range*math.Pow(0.85, wheel), minRange, maxRange)
		scale := r / s.Range
		s.Center = math.Add2f(p, math.Scale2f(math.Sub2f(s.Center, p), scale))
		s.Range = r
	}

	// Clicking an aircraft selects it for the command line.
	if mouse.Clicked[platform.MouseButtonPrimary] {
		if callsign, ok := s.targetAt(ctx, xf, mouse.Pos); ok {
			s.selected = callsign
			s.input = string(callsign) + " "
		}
	}
}

func (s *Scope) handleKeyboard(ctx *scope.Context) {
	kb := ctx.Keyboard
	if kb == nil || !ctx.HaveFocus {
		return
	}
	for _, ch := range strings.ToUpper(kb.Input) {
		// The push-to-talk key defaults to the semicolon.
		if ch >= ' ' && ch <= '~' && ch != ';' {
			s.input += string(ch)
		}
	}
	if kb.WasPressed(imgui.KeyBackspace) && len(s.input) > 0 {
		s.input = s.input[:len(s.input)-1]
	}
	if kb.WasPressed(imgui.KeyEscape) {
		s.input, s.selected = "", ""
	}
	if kb.WasPressed(imgui.KeyEnter) {
		s.runCommand(ctx, strings.TrimSpace(s.input))
	}
}

// runCommand runs an aircraft command typed at the command line:
// "CALLSIGN COMMANDS", where the callsign may be given by its last
// characters.
func (s *Scope) runCommand(ctx *scope.Context, input string) {
	if input == "" {
		return
	}
	suffix, cmds, ok := strings.Cut(input, " ")
	if !ok {
		s.setFeedback("ENTER CALLSIGN AND COMMANDS", true)
		return
	}
	tracks := ctx.TracksFromACIDSuffix(suffix)
	if len(tracks) == 0 {
		s.setFeedback("NO AIRCRAFT "+suffix, true)
		return
	} else if len(tracks) > 1 {
		s.setFeedback("MULTIPLE AIRCRAFT "+suffix, true)
		return
	}
	callsign := tracks[0].ADSBCallsign
	s.input, s.selected = "", callsign
	s.setFeedback(string(callsign)+" "+cmds, false)
	ctx.Client.RunAircraftCommands(client.AircraftCommandRequest{
		Callsign: callsign,
		Commands: cmds,
	}, func(err error, remaining string) {
		if err != nil {
			s.input = string(callsign) + " " + remaining
			s.setFeedback(err.Error(), true)
		}
	})
}

func (s *Scope) setFeedback(msg string, isError bool) {
	s.feedback, s.feedbackError, s.feedbackTime = msg, isError, time.Now()
}

// drawStatus draws the display's status line at the top and the command
// line at the bottom.
func (s *Scope) drawStatus(ctx *scope.Context, td *renderer.TextDrawBuilder) {
	style := renderer.TextStyle{Font: s.textFont, Color: textColor}
	scale := ctx.DrawPixelScale
	ext := ctx.DrawExtent

	status := "ASDE-X " + string(s.ViewAirport) + "   RANGE " + formatRange(s.Range)
	td.AddText(status, [2]float32{ext.P0[0] + 10*scale, ext.P1[1] - 10*scale}, style)

	y := ext.P0[1] + 10*scale + float32(s.textFont.Size)
	if s.feedback != "" && time.Since(s.feedbackTime) < 10*time.Second {
		fb := style
		if s.feedbackError {
			fb.Color = errorColor
		}
		td.AddText(s.feedback, [2]float32{ext.P0[0] + 10*scale, y + float32(s.textFont.Size) + 4*scale}, fb)
	}
	td.AddText("> "+s.input+"_", [2]float32{ext.P0[0] + 10*scale, y}, style)
}

func formatRange(r float32) string {
	if r < 1 {
		return strings.TrimLeft(trimFloat(r, 2), "0")
	}
	return trimFloat(r, 1)
}

func (s *Scope) drawMessage(ctx *scope.Context, cb *renderer.CommandBuffer, msg string) {
	cb.ClearRGB(backgroundColor)
	if s.textFont == nil {
		return
	}
	td := renderer.GetTextDrawBuilder()
	defer renderer.ReturnTextDrawBuilder(td)
	td.AddTextCentered(msg, ctx.DrawExtent.Center(), renderer.TextStyle{Font: s.textFont, Color: textColor})
	xf := s.transforms(ctx.DrawExtent)
	cb.LoadProjectionMatrix(xf.ndcFromWindow)
	cb.LoadModelViewMatrix(math.Identity3x3())
	td.GenerateCommands(cb)
}
