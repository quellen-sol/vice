// scenario/tower_test.go
// Copyright(c) vice contributors, licensed under the GNU Public License, Version 3.
// SPDX: GPL-3.0-only

package scenario

import (
	"io"
	"log/slog"
	"testing"
	"time"

	av "github.com/mmp/vice/aviation"
	"github.com/mmp/vice/aviation/db"
	"github.com/mmp/vice/log"
	"github.com/mmp/vice/math"
	"github.com/mmp/vice/sim"
	"github.com/mmp/vice/util"
	"github.com/mmp/vice/wx"
)

// TestKCOSTowerScenario runs the KCOS tower scenario with the user working
// local control (1W) and the approach controller (1A) virtual, and checks
// that arrivals are switched to the tower on final and land, and that
// departures talk to the tower until the tower sends them to departure.
func TestKCOSTowerScenario(t *testing.T) {
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
	type arrival struct {
		switchDistance float32 // distance from the threshold when first on the tower's frequency
		lastDistance   float32 // distance from the field
		landed         bool
	}
	arrivals := make(map[av.ADSBCallsign]*arrival)
	var departuresOnTower int
	var sentCallsign av.ADSBCallsign
	var sentAltitude float32
	sentTime := 0

	for step := range 3600 {
		s.Step(time.Second)

		for callsign, ac := range s.Aircraft {
			switch {
			case ac.IsArrival() && ac.ArrivalAirport == "KCOS":
				ar := arrivals[callsign]
				if ar == nil {
					ar = &arrival{}
					arrivals[callsign] = ar
				}
				ar.lastDistance = math.NMDistance2LL(ac.Position(), ap.Location)
				if appr := ac.Nav.Approach.Assigned; appr != nil &&
					ac.ControllerFrequency == sim.ControlPosition(tower) && ar.switchDistance == 0 {
					ar.switchDistance = math.NMDistance2LL(ac.Position(), appr.Threshold)
				}

			case ac.IsDeparture() && ac.DepartureAirport == "KCOS" && !ac.WaitingForLaunch:
				if ac.FlightPlan != nil && ac.FlightPlan.TrackingController != approach {
					t.Errorf("%s: departure tracked by %q, want %q", callsign, ac.FlightPlan.TrackingController, approach)
				}
				if callsign == sentCallsign {
					continue
				}
				if ac.ControllerFrequency != sim.ControlPosition(tower) {
					t.Errorf("%s: departure on %q's frequency before the tower sent it to departure",
						callsign, ac.ControllerFrequency)
				}
				departuresOnTower++
				// Once one is well clear of the field, send it to departure.
				if sentCallsign == "" && ac.Altitude() > float32(ap.Elevation)+2000 {
					res := s.RunAircraftControlCommands(sim.TCW(tower), callsign, "FC", 0, 0)
					if res.Error != nil {
						t.Fatalf("%s: FC: %v", callsign, res.Error)
					}
					sentCallsign, sentAltitude, sentTime = callsign, ac.Altitude(), step
				}
			}
		}
		// Arrivals that left the sim close to the field landed.
		for callsign, ar := range arrivals {
			if _, ok := s.Aircraft[callsign]; !ok && !ar.landed {
				ar.landed = ar.lastDistance < 3
			}
		}
	}

	var switched, landed int
	for callsign, ar := range arrivals {
		if ar.switchDistance != 0 {
			switched++
			// The switch happens at 6nm; the pilot changes frequency a
			// few seconds later.
			if ar.switchDistance > 6 || ar.switchDistance < 4.5 {
				t.Errorf("%s: switched to tower %.1fnm from the field", callsign, ar.switchDistance)
			}
		}
		if ar.landed {
			landed++
			if ar.switchDistance == 0 {
				t.Errorf("%s: landed without talking to the tower", callsign)
			}
		}
	}
	if switched < 5 || landed < 5 {
		t.Errorf("only %d arrivals were switched to the tower and %d landed in an hour", switched, landed)
	}
	if departuresOnTower == 0 || sentCallsign == "" {
		t.Fatalf("no departures talked to the tower")
	}

	// The departure the tower sent to departure is now working the
	// (virtual) approach controller, who sends it on course and climbs it.
	if ac, ok := s.Aircraft[sentCallsign]; ok {
		if ac.ControllerFrequency != sim.ControlPosition(approach) {
			t.Errorf("%s: on %q's frequency after FC, want %q", sentCallsign, ac.ControllerFrequency, approach)
		}
		if 3600-sentTime > 300 && ac.Altitude() < sentAltitude+3000 {
			t.Errorf("%s: only climbed from %.0f to %.0f after FC", sentCallsign, sentAltitude, ac.Altitude())
		}
	}
	t.Logf("%d arrivals switched to the tower, %d landed; %d departure-seconds on the tower's frequency",
		switched, landed, departuresOnTower)
}
