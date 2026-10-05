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

	type arrival struct {
		firstDistance  float32 // distance from the threshold when first seen
		switchDistance float32 // distance from the threshold when first on the tower's frequency
		lastDistance   float32 // distance from the field
		cleared        bool
		wentAround     bool
		sentToApproach bool
		gone           bool
	}
	arrivals := make(map[av.ADSBCallsign]*arrival)
	var goAround av.ADSBCallsign // the arrival left without a landing clearance
	type departure struct {
		ready, linedUp, clearedForTakeoff, sentToDeparture bool
		sentAltitude                                       float32
		sentStep                                           int
	}
	departures := make(map[av.ADSBCallsign]*departure)

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

		for ac := range util.SortedMapValues(s.Aircraft) {
			callsign := ac.ADSBCallsign
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
					d := math.NMDistance2LL(ac.Position(), appr.Threshold)
					if ar.firstDistance == 0 {
						ar.firstDistance = d
					}
					if onTower && ar.switchDistance == 0 {
						ar.switchDistance = d
					}
				}

				if ac.WentAround {
					if !ar.wentAround {
						ar.wentAround = true
						if callsign != goAround {
							t.Errorf("%s: went around with a landing clearance", callsign)
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
					}
					dep.ready = true
					if ac.ControllerFrequency != sim.ControlPosition(tower) {
						t.Errorf("%s: called ready on %q's frequency", callsign, ac.ControllerFrequency)
					}
					// Line up and wait first, then clear for takeoff a minute
					// later.
					if !dep.linedUp {
						dep.linedUp = true
						run(callsign, "LUAW", "wait")
					} else if step%60 == 0 {
						dep.clearedForTakeoff = true
						run(callsign, "CTO", "cleared for takeoff")
						if ac.WaitingForLaunch {
							t.Errorf("%s: still waiting to launch after CTO", callsign)
						}
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
			}
		}
	}

	var switched, landed int
	for callsign, ar := range arrivals {
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
	t.Logf("arrivals: %d switched to the tower, %d landed; departures: %d took off, %d sent to departure",
		switched, landed, tookOff, sent)
}
