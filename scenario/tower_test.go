// scenario/tower_test.go
// Copyright(c) vice contributors, licensed under the GNU Public License, Version 3.
// SPDX: GPL-3.0-only

package scenario

import (
	"fmt"
	"io"
	"log/slog"
	"strings"
	"testing"
	"time"

	av "github.com/mmp/vice/aviation"
	"github.com/mmp/vice/aviation/db"
	"github.com/mmp/vice/log"
	"github.com/mmp/vice/math"
	"github.com/mmp/vice/sim"
	"github.com/mmp/vice/speech/stt"
	"github.com/mmp/vice/surface"
	"github.com/mmp/vice/util"
	"github.com/mmp/vice/wx"
)

// TestKCOSTowerScenario works the KCOS tower scenario for an hour as local
// control (1W), with the approach controller (1A) virtual. It clears arrivals
// to land, lines departures up and clears them for takeoff when they call
// ready, and sends them to departure once they're airborne. One arrival is
// left without a landing clearance; it should go around, stay with the tower
// and be taken away for resequencing once the tower sends it to approach.
func TestKCOSTowerScenario(t *testing.T) {
	runKCOSTowerScenario(t, false)
}

// TestKCOSTowerScenarioVoice works the scenario issuing the runway
// clearances by voice: each is spoken as a controller would say it, decoded
// with the speech recognizer's context for the aircraft on the tower's
// frequency (built from the state the tower's client is sent), and the
// decoded command run.
func TestKCOSTowerScenarioVoice(t *testing.T) {
	runKCOSTowerScenario(t, true)
}

func runKCOSTowerScenario(t *testing.T, voice bool) {
	db.InitDB()
	lg := &log.Logger{Logger: slog.New(slog.NewTextHandler(io.Discard, nil)), LogDir: t.TempDir()}

	var e util.ErrorLogger
	tables, _ := Load(OverrideFiles{}, &e, lg)
	if e.HaveErrors() {
		t.Fatalf("loading scenarios: %s", e.String())
	}
	cfg, err := tables.NewSimConfigurationForScenario("COS", "KCOS Tower North")
	if err != nil {
		t.Fatal(err)
	}
	intervals := wx.FacilityTimeIntervals("COS")
	if len(intervals) == 0 {
		t.Fatal("COS: no weather in the resources")
	}
	cfg.StartTime = intervals[0].Start().Add(2 * time.Hour)
	cfg.WXProvider = wx.MakeProvider("", lg)

	const tower, approach = sim.TCP("1W"), sim.TCP("1A")
	if root, _ := cfg.ControllerConfiguration.RootPosition(); root != tower {
		t.Fatalf("scenario root position is %q, want %q", root, tower)
	}
	// The arrivals never get a handoff to a human, but they talk to the
	// tower, so they're the user's traffic rather than background.
	for _, flow := range []string{"TWR.35L", "TWR.35R"} {
		if cfg.LaunchConfig.InboundFlowIsBackground(flow, "KCOS") {
			t.Errorf("%s: arrivals to KCOS marked as background traffic", flow)
		}
	}

	s := sim.NewSim(*cfg, lg)
	s.Activate(lg, cfg.WXProvider)
	if err := s.SignOn(sim.TCW(tower), s.AllScenarioPositions()); err != nil {
		t.Fatal(err)
	}
	s.Prespawn()

	ap, ok := db.DB.Airports["KCOS"]
	if !ok {
		t.Fatal("KCOS not in the database")
	}
	fieldElevation := float32(ap.Elevation)

	transcriber := stt.NewTranscriber(nil)
	phrases := map[string]string{
		"CTL":  "runway %s cleared to land",
		"CTO":  "runway %s cleared for takeoff",
		"LUAW": "runway %s line up and wait",
	}
	// speak returns the command the speech recognizer hears for the
	// controller giving the aircraft the clearance.
	speak := func(callsign av.ADSBCallsign, cmd string) string {
		t.Helper()
		acCtx := transcriber.BuildAircraftContext(s.GetUserState(), sim.TCW(tower))
		for _, spoken := range util.SortedMapKeys(acCtx) {
			ac := acCtx[spoken]
			if ac.Callsign != string(callsign) {
				continue
			}
			if !ac.TowerControl || ac.TowerRunway == "" {
				t.Fatalf("%s: speech context %+v doesn't have the tower working it", callsign, ac)
			}
			rwy := strings.NewReplacer("L", " left", "R", " right", "C", " center").Replace(ac.TowerRunway)
			// The context's keys spell the NATO alphabet for speech
			// synthesis ("brahvo"); speak it as whisper writes it. (Whisper
			// also writes "X-ray", which the decoder can take for an ATIS
			// letter at the end of a GA callsign whoever the controller is;
			// that isn't what this test is about.)
			spoken = strings.NewReplacer("brahvo", "bravo", "pahpah", "papa", "kebeck", "quebec",
				"x-ray", "xray").Replace(strings.ToLower(spoken))
			transcript := spoken + " " + fmt.Sprintf(phrases[cmd], rwy)
			decoded, err := transcriber.DecodeTranscript(acCtx, transcript, "")
			if err != nil {
				t.Fatalf("%q: %v", transcript, err)
			}
			if want := string(callsign) + " " + cmd; decoded != want {
				t.Errorf("%q: decoded %q, want %q", transcript, decoded, want)
			}
			_, heard, _ := strings.Cut(decoded, " ")
			return heard
		}
		t.Fatalf("%s: not in the speech context", callsign)
		return ""
	}

	run := func(callsign av.ADSBCallsign, cmd, wantReadback string) {
		t.Helper()
		if _, ok := phrases[cmd]; ok && voice {
			cmd = speak(callsign, cmd)
		}
		res := s.RunAircraftControlCommands(sim.TCW(tower), callsign, cmd, 0, 0)
		if res.Error != nil {
			t.Fatalf("%s: %s: %v", callsign, cmd, res.Error)
		}
		if !strings.Contains(res.ReadbackSpokenText, wantReadback) {
			t.Errorf("%s: %s: readback %q doesn't include %q", callsign, cmd, res.ReadbackSpokenText, wantReadback)
		}
	}

	// The sim moves aircraft on KCOS's surface.
	kcos, err := surface.LoadAirport("KCOS")
	if err != nil {
		t.Fatal(err)
	}
	atGate := func(p math.Point2LL) bool {
		for _, g := range kcos.Gates {
			if kcos.Distance(p, g.Location) < 0.02 {
				return true
			}
		}
		return false
	}
	// onRamp reports whether the aircraft is parked at its gate or on its
	// way into or out of it, between the gate and the taxiway. (The sim
	// moves aircraft straight between the two, and gates' lines can pass
	// close to the gates next to them.)
	onRamp := func(ac *sim.Aircraft) bool {
		g := ac.Ground
		return g.Phase == sim.GroundParked || (g.Phase == sim.GroundTaxiOut && g.LastNode == -1) ||
			(g.Phase == sim.GroundTaxiIn && len(g.Path) == 1)
	}
	// onRunway reports whether p is on the runway with the given end.
	onRunway := func(end string, p math.Point2LL) bool {
		rwy, _, ok := kcos.LookupRunway(end)
		if !ok {
			t.Fatalf("%s: no such runway at KCOS", end)
		}
		a, b := kcos.Local(rwy.Ends[0].Threshold), kcos.Local(rwy.Ends[1].Threshold)
		return math.PointSegmentDistance(kcos.Local(p), a, b) < 0.02
	}
	// sameRunway reports whether the two runway ends are of the same runway.
	sameRunway := func(a, b string) bool {
		rwy, _, ok := kcos.LookupRunway(a)
		return ok && rwy.HasEnd(b)
	}
	// runwayTraffic returns what keeps the tower from letting the aircraft
	// onto the runway, if anything: someone else on it or lined up, or an
	// arrival within nm out.
	runwayTraffic := func(ac *sim.Aircraft, end string, within float32) string {
		for other := range util.SortedMapValues(s.Aircraft) {
			if other == ac {
				continue
			}
			if g := other.Ground; g != nil {
				if g.Phase != sim.GroundParked && onRunway(end, other.Position()) {
					return fmt.Sprintf("%s on the runway (%s)", other.ADSBCallsign, g)
				}
				if (g.Phase == sim.GroundLiningUp || g.Phase == sim.GroundLinedUp) && sameRunway(end, g.Runway) {
					return fmt.Sprintf("%s lined up on %s", other.ADSBCallsign, g.Runway)
				}
			} else if other.IsDeparture() {
				if !other.WaitingForLaunch && other.Altitude() < fieldElevation+100 && onRunway(end, other.Position()) {
					return fmt.Sprintf("%s departing", other.ADSBCallsign)
				}
			} else if appr := other.Nav.Approach.Assigned; other.IsArrival() && appr != nil && sameRunway(end, appr.Runway) &&
				!other.WentAround {
				if d := math.NMDistance2LL(other.Position(), appr.Threshold); d < within {
					return fmt.Sprintf("%s landing, %.1fnm out", other.ADSBCallsign, d)
				}
			}
		}
		return ""
	}
	runwayClear := func(ac *sim.Aircraft, end string, within float32) bool {
		return runwayTraffic(ac, end, within) == ""
	}

	type arrival struct {
		firstDistance  float32 // distance from the threshold when first seen
		switchDistance float32 // distance from the threshold when first on the tower's frequency
		lastDistance   float32 // distance from the field
		cleared        bool
		wentAround     bool
		sentToApproach bool
		touchedDown    bool
		exited         bool
		parked         bool
		gone           bool
		runway         string
	}
	arrivals := make(map[av.ADSBCallsign]*arrival)
	var goAround av.ADSBCallsign // the arrival left without a landing clearance
	type departure struct {
		ready, linedUp, clearedForTakeoff, sentToDeparture bool
		sentAltitude                                       float32
		linedUpStep, clearedStep, sentStep                 int
	}
	departures := make(map[av.ADSBCallsign]*departure)
	crossings := 0
	overlapped := make(map[[2]av.ADSBCallsign]bool)
	stoppedSince := make(map[av.ADSBCallsign]int)

	const steps = 3600
	for step := range steps {
		s.Step(time.Second)

		// Every aircraft talking to the tower is on its display, in the air
		// or, below radar coverage, on the ground.
		if step%5 == 0 {
			state := s.GetUserState()
			for ac := range util.SortedMapValues(s.Aircraft) {
				if ac.ControllerFrequency != sim.ControlPosition(tower) ||
					(ac.WaitingForLaunch && !ac.ReadyForDeparture) {
					continue
				}
				if state.Tracks[ac.ADSBCallsign] == nil && state.SurfaceTracks[ac.ADSBCallsign] == nil {
					t.Errorf("%s: talking to the tower but neither a track nor a surface track (alt %.0f)",
						ac.ADSBCallsign, ac.Altitude())
				}
			}
		}

		// Aircraft taxiing the same way keep their distance.
		for ac := range util.SortedMapValues(s.Aircraft) {
			if ac.Ground == nil || ac.Ground.Speed == 0 ||
				(ac.Ground.Phase != sim.GroundTaxiOut && ac.Ground.Phase != sim.GroundTaxiIn) {
				continue
			}
			for other := range util.SortedMapValues(s.Aircraft) {
				if other == ac || other.Ground == nil || other.Ground.Speed == 0 || other.Ground.Phase != ac.Ground.Phase ||
					math.HeadingDifference(other.Ground.Heading, ac.Ground.Heading) > 90 || (onRamp(ac) && onRamp(other)) {
					continue
				}
				if d := kcos.Distance(ac.Position(), other.Position()); d < 0.015 && ac.ADSBCallsign < other.ADSBCallsign {
					t.Errorf("%s and %s taxiing %.0f feet apart", ac.ADSBCallsign, other.ADSBCallsign,
						d*math.NauticalMilesToFeet)
				}
			}
		}
		// Nobody's stuck.
		for ac := range util.SortedMapValues(s.Aircraft) {
			if g := ac.Ground; g == nil || g.Speed > 0 || !(g.Phase == sim.GroundTaxiOut || g.Phase == sim.GroundTaxiIn ||
				g.Phase == sim.GroundRollout || g.Phase == sim.GroundLiningUp) {
				delete(stoppedSince, ac.ADSBCallsign)
			} else if since, ok := stoppedSince[ac.ADSBCallsign]; !ok {
				stoppedSince[ac.ADSBCallsign] = step
			} else if step-since == 600 {
				t.Errorf("%s: stopped for 10 minutes (%s), giving way to %q", ac.ADSBCallsign, g, g.GivingWay)
			}
		}
		// And nobody taxis into anybody, moving or not, other than on the
		// ramps.
		for ac := range util.SortedMapValues(s.Aircraft) {
			for other := range util.SortedMapValues(s.Aircraft) {
				pair := [2]av.ADSBCallsign{ac.ADSBCallsign, other.ADSBCallsign}
				if ac.ADSBCallsign >= other.ADSBCallsign || ac.Ground == nil || other.Ground == nil ||
					ac.Ground.Phase == sim.GroundParked || other.Ground.Phase == sim.GroundParked ||
					(onRamp(ac) && onRamp(other)) || overlapped[pair] {
					continue
				}
				if d := kcos.Distance(ac.Position(), other.Position()); d < 0.01 {
					overlapped[pair] = true
					t.Errorf("%s (%s) and %s (%s) %.0f feet apart", ac.ADSBCallsign, ac.Ground, other.ADSBCallsign,
						other.Ground, d*math.NauticalMilesToFeet)
				}
			}
		}

		for ac := range util.SortedMapValues(s.Aircraft) {
			callsign := ac.ADSBCallsign
			// Aircraft that ask to cross a runway are cleared across once
			// it's clear.
			if g := ac.Ground; g != nil && g.Phase == sim.GroundHoldingToCross && g.RequestedCrossing &&
				ac.ControllerFrequency == sim.ControlPosition(tower) && runwayClear(ac, g.CrossRunway, 3) {
				crossings++
				run(callsign, "CROSS", "cross")
			}
			switch {
			case ac.IsArrival() && ac.ArrivalAirport == "KCOS":
				ar := arrivals[callsign]
				if ar == nil {
					ar = &arrival{}
					arrivals[callsign] = ar
				}
				ar.lastDistance = math.NMDistance2LL(ac.Position(), ap.Location)
				onTower := ac.ControllerFrequency == sim.ControlPosition(tower)
				if appr := ac.Nav.Approach.Assigned; appr != nil {
					ar.runway = appr.Runway
					d := math.NMDistance2LL(ac.Position(), appr.Threshold)
					if ar.firstDistance == 0 {
						ar.firstDistance = d
					}
					if onTower && ar.switchDistance == 0 {
						ar.switchDistance = d
					}
				}

				// After landing, it rolls out, turns off the runway before its
				// end and taxis to a gate.
				if g := ac.Ground; g != nil {
					ar.touchedDown = true
					if appr := ac.Nav.Approach.Assigned; g.Phase == sim.GroundTaxiIn && !ar.exited && appr != nil {
						ar.exited = true
						length := math.NMDistance2LL(appr.Threshold, appr.OppositeThreshold)
						if d := math.NMDistance2LL(ac.Position(), appr.Threshold); d > length {
							t.Errorf("%s: turned off the runway %.2fnm from the threshold of a %.2fnm runway",
								callsign, d, length)
						}
					}
					if g.Phase == sim.GroundParked {
						ar.parked = true
						if !atGate(ac.Position()) {
							t.Errorf("%s: parked away from a gate", callsign)
						}
					}
				}

				if ac.WentAround {
					if !ar.wentAround {
						ar.wentAround = true
						if callsign != goAround {
							t.Errorf("%s: went around with a landing clearance: %s", callsign,
								runwayTraffic(ac, ar.runway, 0))
						}
					}
					if !ar.sentToApproach && ac.Altitude() > fieldElevation+1500 {
						if !onTower {
							t.Errorf("%s: on %q's frequency after going around, want the tower's", callsign, ac.ControllerFrequency)
						}
						ar.sentToApproach = true
						run(callsign, "FC", "")
					}
				} else if onTower && !ar.cleared && ac.Nav.Approach.Cleared {
					if goAround == "" {
						goAround = callsign // never cleared to land
					} else if callsign != goAround {
						ar.cleared = true
						run(callsign, "CTL", "cleared to land")
					}
				}

			case ac.IsDeparture() && ac.DepartureAirport == "KCOS":
				dep := departures[callsign]
				if dep == nil {
					dep = &departure{}
					departures[callsign] = dep
					// Departures start at a gate. (Those that were already
					// taxiing when the sim started are an exception.)
					if ac.Ground == nil && ac.WaitingForLaunch {
						t.Errorf("%s: departure without ground movement", callsign)
					} else if step > 0 && !atGate(ac.Position()) {
						t.Errorf("%s: departure didn't start at a gate", callsign)
					}
				}
				if ac.FlightPlan != nil && ac.FlightPlan.TrackingController != approach {
					t.Errorf("%s: departure tracked by %q, want %q", callsign, ac.FlightPlan.TrackingController, approach)
				}

				if ac.WaitingForLaunch {
					if !ac.ReadyForDeparture {
						continue
					}
					if !dep.ready {
						// The tower's client is told about it so that it can
						// be given clearances, though it isn't radar visible.
						trk := s.GetUserState().SurfaceTracks[callsign]
						if trk == nil || !trk.ReadyForDeparture || trk.TowerRunway == "" {
							t.Errorf("%s: ready departure's surface track: %+v", callsign, trk)
						}
						// It called ready stopped at the hold-short line.
						if g := ac.Ground; g == nil || g.Phase != sim.GroundHoldingShort || g.Speed != 0 {
							t.Errorf("%s: called ready but not holding short: %+v", callsign, g)
						}
					}
					dep.ready = true
					if ac.ControllerFrequency != sim.ControlPosition(tower) {
						t.Errorf("%s: called ready on %q's frequency", callsign, ac.ControllerFrequency)
					}
					// Line up and wait when the runway's clear with room
					// ahead of the next arrival, then clear for takeoff once
					// in position.
					g := ac.Ground
					if g == nil {
						t.Fatalf("%s: ready departure without ground movement", callsign)
					}
					if !dep.linedUp {
						if runwayClear(ac, g.Runway, 6) {
							dep.linedUp = true
							run(callsign, "LUAW", "wait")
						}
					} else if !dep.clearedForTakeoff && g.Phase == sim.GroundLinedUp {
						if runwayClear(ac, g.Runway, 2.5) {
							dep.clearedForTakeoff, dep.clearedStep = true, step
							run(callsign, "CTO", "cleared for takeoff")
						} else if dep.linedUpStep == 0 {
							dep.linedUpStep = step
						} else if step-dep.linedUpStep == 300 {
							t.Errorf("%s: lined up on %s for 5 minutes: %s", callsign, g.Runway,
								runwayTraffic(ac, g.Runway, 2.5))
						}
					} else if dep.clearedForTakeoff && step-dep.clearedStep > 90 {
						t.Fatalf("%s: hasn't started its takeoff roll 90 seconds after CTO: %+v",
							callsign, ac.Ground)
					}
					continue
				}

				if !dep.clearedForTakeoff {
					t.Fatalf("%s: took off without a takeoff clearance", callsign)
				}
				if !dep.sentToDeparture {
					if ac.ControllerFrequency != sim.ControlPosition(tower) {
						t.Errorf("%s: on %q's frequency before the tower sent it to departure", callsign, ac.ControllerFrequency)
					}
					if ac.Altitude() > fieldElevation+1500 {
						dep.sentToDeparture, dep.sentAltitude, dep.sentStep = true, ac.Altitude(), step
						run(callsign, "FC", "")
					}
				} else if step-dep.sentStep == 120 {
					if ac.ControllerFrequency != sim.ControlPosition(approach) {
						t.Errorf("%s: on %q's frequency after FC, want %q", callsign, ac.ControllerFrequency, approach)
					}
					if ac.Altitude() < dep.sentAltitude+1000 {
						t.Errorf("%s: only climbed from %.0f to %.0f in the 2 minutes after FC", callsign,
							dep.sentAltitude, ac.Altitude())
					}
				}
			}
		}

		for callsign, ar := range arrivals {
			if _, ok := s.Aircraft[callsign]; !ok && !ar.gone {
				ar.gone = true
				landed := ar.lastDistance < 3 && !ar.wentAround
				if landed && !ar.cleared {
					t.Errorf("%s: landed without a landing clearance", callsign)
				}
				if landed && !ar.parked {
					t.Errorf("%s: left the sim without taxiing to a gate (touched down %v, exited %v)",
						callsign, ar.touchedDown, ar.exited)
				}
			}
		}
	}

	var switched, landed, parked int
	for callsign, ar := range arrivals {
		if ar.parked {
			parked++
		}
		if ar.switchDistance != 0 {
			switched++
			// The switch happens at 6nm; the pilot changes frequency a few
			// seconds later. (Arrivals already inside 6nm when the sim
			// started were switched during prespawn.)
			if ar.firstDistance > 6.5 && (ar.switchDistance > 6 || ar.switchDistance < 4.5) {
				t.Errorf("%s: switched to tower %.1fnm from the threshold", callsign, ar.switchDistance)
			}
		}
		if ar.gone && ar.cleared {
			landed++
		}
	}
	if switched < 5 || landed < 5 {
		t.Errorf("only %d arrivals were switched to the tower and %d landed in an hour", switched, landed)
	}
	if ar := arrivals[goAround]; ar == nil || !ar.wentAround || !ar.sentToApproach || !ar.gone {
		t.Errorf("%s: arrival without a landing clearance: %+v; want it to go around, be sent to approach "+
			"and be resequenced", goAround, ar)
	}

	var tookOff, sent int
	for _, dep := range departures {
		if dep.clearedForTakeoff {
			tookOff++
		}
		if dep.sentToDeparture {
			sent++
		}
	}
	if tookOff < 5 || sent < 5 {
		t.Errorf("only %d departures took off and %d were sent to departure in an hour", tookOff, sent)
	}
	if parked < 5 {
		t.Errorf("only %d arrivals taxied to a gate", parked)
	}
	t.Logf("arrivals: %d switched to the tower, %d landed, %d parked at a gate; departures: %d took off, %d sent to departure; %d runway crossings",
		switched, landed, parked, tookOff, sent, crossings)
}
