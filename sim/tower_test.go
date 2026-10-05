// sim/tower_test.go
// Copyright(c) vice contributors, licensed under the GNU Public License, Version 3.
// SPDX: GPL-3.0-only

package sim

import (
	"errors"
	"io"
	"log/slog"
	"strings"
	"testing"

	av "github.com/mmp/vice/aviation"
	"github.com/mmp/vice/log"
	"github.com/mmp/vice/rand"
	"github.com/mmp/vice/speech"
)

// newTowerTestSim returns a sim with KCOS's two local control positions,
// West (1W) working 17R/35L and East (1E) working 17L/35R, both of which
// humans may staff, and a virtual approach controller (1A).
func newTowerTestSim(consolidation map[TCW]*TCPConsolidation) *Sim {
	s := NewTestSim(&log.Logger{Logger: slog.New(slog.NewTextHandler(io.Discard, nil))})
	s.ControlPositions = map[TCP]*av.Controller{
		"1W": {Position: "1W", Role: av.RoleLocal, Airport: "KCOS", Runways: []string{"17R", "35L"}},
		"1E": {Position: "1E", Role: av.RoleLocal, Airport: "KCOS", Runways: []string{"17L", "35R"}},
		"1A": {Position: "1A"},
	}
	s.ScenarioDefaultConsolidation = PositionConsolidation{"1W": {"1E"}}
	s.State.CurrentConsolidation = consolidation
	return s
}

func TestHumanTowerPosition(t *testing.T) {
	check := func(s *Sim, airport av.ICAOAirportCode, runway string, want TCP) {
		t.Helper()
		got, ok := s.humanTowerPosition(airport, runway)
		if want == "" {
			if ok {
				t.Errorf("%s %q: got tower %q, want none", airport, runway, got)
			}
		} else if !ok || got != want {
			t.Errorf("%s %q: got tower %q (%v), want %q", airport, runway, got, ok, want)
		}
	}

	// With East local combined into West, West works every runway.
	s := newTowerTestSim(map[TCW]*TCPConsolidation{
		"1W": {PrimaryTCP: "1W", SecondaryTCPs: []SecondaryTCP{{TCP: "1E"}}},
	})
	for _, rwy := range []string{"35L", "35R", "17L", "17R", "13", ""} {
		check(s, "KCOS", rwy, "1W")
	}
	if !s.hasHumanTower("KCOS") {
		t.Errorf("KCOS: no human tower")
	}

	// Split, each runway goes to the position that works it. Runway
	// suffixes are ignored, and runways neither position lists go to the
	// first position.
	s = newTowerTestSim(map[TCW]*TCPConsolidation{
		"1W": {PrimaryTCP: "1W"},
		"1E": {PrimaryTCP: "1E"},
	})
	for rwy, want := range map[string]TCP{"35L": "1W", "17R": "1W", "35R": "1E", "17L": "1E",
		"35L.TWR": "1W", "13": "1E", "": "1E"} {
		check(s, "KCOS", rwy, want)
	}

	// No tower at other airports.
	check(s, "KFLY", "33", "")
	if s.hasHumanTower("KFLY") {
		t.Errorf("KFLY: unexpected human tower")
	}

	// When the scenario doesn't let humans staff the tower positions, the
	// tower is virtual and the sim handles it as it always has.
	s.ScenarioDefaultConsolidation = PositionConsolidation{"1A": nil}
	check(s, "KCOS", "35L", "")
}

// newTowerClearanceTestSim returns a tower test sim with the tower combined
// at the 1W TCW and a radar controller at the 1A TCW, with an arrival on
// final for 35L and a departure queued for 35L, both talking to the tower.
func newTowerClearanceTestSim(t *testing.T) (*Sim, *Aircraft, *Aircraft) {
	t.Helper()
	s := newTowerTestSim(map[TCW]*TCPConsolidation{
		"1W": {PrimaryTCP: "1W", SecondaryTCPs: []SecondaryTCP{{TCP: "1E"}}},
		"1A": {PrimaryTCP: "1A"},
	})
	s.PrivilegedTCWs = map[TCW]bool{}
	s.State.Airports["KCOS"] = &av.Airport{}

	arr := MakeTestAircraft("AAL1", "35L") // 5nm from the threshold
	arr.ArrivalAirport = "KCOS"
	arr.ControllerFrequency = "1W"
	arr.Nav.Approach.Cleared = true
	s.Aircraft[arr.ADSBCallsign] = arr

	dep := &Aircraft{
		ADSBCallsign:        "SWA2",
		TypeOfFlight:        av.FlightTypeDeparture,
		DepartureAirport:    "KCOS",
		ControllerFrequency: "1W",
		WaitingForLaunch:    true,
	}
	s.Aircraft[dep.ADSBCallsign] = dep
	s.DepartureState["KCOS"] = map[av.RunwayID]*RunwayLaunchState{
		"35L": {ReleasedIFR: []DepartureAircraft{{ADSBCallsign: dep.ADSBCallsign}}},
	}
	return s, arr, dep
}

// render returns what the intent has the pilot say.
func render(t *testing.T, intent speech.CommandIntent) string {
	t.Helper()
	if intent == nil {
		t.Fatal("no readback")
	}
	rt := &speech.RadioTransmission{}
	intent.Render(rt, rand.Make())
	rd, err := rt.Render(rand.Make())
	if err != nil {
		t.Fatal(err)
	}
	return rd.Written
}

func TestTowerClearancesNeedTower(t *testing.T) {
	s, arr, dep := newTowerClearanceTestSim(t)

	// The radar controller can't issue runway clearances, even to aircraft
	// it can talk to.
	arr.ControllerFrequency, dep.ControllerFrequency = "1A", "1A"
	for _, cmd := range []string{"CTL", "GOAR"} {
		if _, err := s.runOneControlCommand("1A", arr.ADSBCallsign, cmd, 0); !errors.Is(err, ErrNotTowerPosition) {
			t.Errorf("%s from the radar controller: got error %v, want %v", cmd, err, ErrNotTowerPosition)
		}
	}
	for _, cmd := range []string{"CTO", "LUAW"} {
		if _, err := s.runOneControlCommand("1A", dep.ADSBCallsign, cmd, 0); !errors.Is(err, ErrNotTowerPosition) {
			t.Errorf("%s from the radar controller: got error %v, want %v", cmd, err, ErrNotTowerPosition)
		}
	}

	// The tower can only talk to aircraft on its frequency.
	if _, err := s.runOneControlCommand("1W", arr.ADSBCallsign, "CTL", 0); !errors.Is(err, av.ErrOtherControllerHasTrack) {
		t.Errorf("CTL to an aircraft on another frequency: got error %v", err)
	}
	if arr.ClearedToLand {
		t.Errorf("cleared to land by a controller who can't")
	}
}

func TestTowerClearedToLand(t *testing.T) {
	s, arr, _ := newTowerClearanceTestSim(t)

	if !s.mustGoAroundWithoutClearance(arr) {
		t.Errorf("an arrival without a landing clearance doesn't have to go around")
	}
	intent, err := s.runOneControlCommand("1W", arr.ADSBCallsign, "CTL", 0)
	if err != nil {
		t.Fatal(err)
	}
	if rb := render(t, intent); !strings.Contains(rb, "cleared to land") {
		t.Errorf("readback %q", rb)
	}
	if !arr.ClearedToLand || s.mustGoAroundWithoutClearance(arr) {
		t.Errorf("CTL didn't clear the aircraft to land")
	}

	// Not on the approach: the pilot is unable.
	arr.ClearedToLand, arr.Nav.Approach.Cleared = false, false
	intent, _ = s.runOneControlCommand("1W", arr.ADSBCallsign, "CTL", 0)
	if _, ok := intent.(speech.UnableIntent); !ok || arr.ClearedToLand {
		t.Errorf("CTL off the approach: got %#v", intent)
	}
}

func TestTowerTakeoffClearance(t *testing.T) {
	s, _, dep := newTowerClearanceTestSim(t)

	// Still taxiing: unable.
	for _, cmd := range []string{"LUAW", "CTO"} {
		intent, err := s.runOneControlCommand("1W", dep.ADSBCallsign, cmd, 0)
		if err != nil {
			t.Fatal(err)
		}
		if _, ok := intent.(speech.UnableIntent); !ok {
			t.Errorf("%s before ready: got %#v, want unable", cmd, intent)
		}
	}
	if dep.LinedUp || !dep.WaitingForLaunch {
		t.Fatalf("departure lined up or launched before it was ready")
	}

	dep.ReadyForDeparture = true
	intent, err := s.runOneControlCommand("1W", dep.ADSBCallsign, "LUAW", 0)
	if err != nil {
		t.Fatal(err)
	}
	if rb := render(t, intent); !strings.Contains(rb, "wait") || !dep.LinedUp || !dep.WaitingForLaunch {
		t.Errorf("LUAW: readback %q, lined up %v, waiting %v", rb, dep.LinedUp, dep.WaitingForLaunch)
	}

	intent, err = s.runOneControlCommand("1W", dep.ADSBCallsign, "CTO", 0)
	if err != nil {
		t.Fatal(err)
	}
	if rb := render(t, intent); !strings.Contains(rb, "cleared for takeoff") {
		t.Errorf("CTO readback %q", rb)
	}
	state := s.DepartureState["KCOS"]["35L"]
	if dep.WaitingForLaunch || len(state.ReleasedIFR) != 0 || state.LastDeparture == nil ||
		state.LastDeparture.ADSBCallsign != dep.ADSBCallsign {
		t.Errorf("CTO didn't launch the departure: waiting %v, queue %v", dep.WaitingForLaunch, state.ReleasedIFR)
	}

	intent, _ = s.runOneControlCommand("1W", dep.ADSBCallsign, "CTO", 0)
	if _, ok := intent.(speech.UnableIntent); !ok {
		t.Errorf("CTO when airborne: got %#v, want unable", intent)
	}
}

func TestTowerPilotCalls(t *testing.T) {
	s, arr, dep := newTowerClearanceTestSim(t)
	dep.ReadyForDeparture = true

	for _, tc := range []struct {
		pc   PendingContact
		want []string
	}{
		{PendingContact{ID: 1, ADSBCallsign: arr.ADSBCallsign, TCP: "1W", Type: PendingTransmissionArrival},
			[]string{"5 mile final", "35L"}},
		{PendingContact{ID: 2, ADSBCallsign: dep.ADSBCallsign, TCP: "1W", Type: PendingTransmissionReadyForDeparture},
			[]string{"ready", "35L"}},
		{PendingContact{ID: 3, ADSBCallsign: arr.ADSBCallsign, TCP: "1W", Type: PendingTransmissionRequestLandingClearance},
			[]string{"short final", "35L"}},
	} {
		if !s.contactApplies(tc.pc) {
			t.Errorf("type %d: contact doesn't apply", tc.pc.Type)
			continue
		}
		pt, err := s.renderContact(tc.pc)
		if err != nil {
			t.Fatal(err)
		}
		for _, w := range tc.want {
			if !strings.Contains(pt.Written, w) {
				t.Errorf("type %d: %q doesn't include %q", tc.pc.Type, pt.Written, w)
			}
		}
	}

	// The calls are moot once the tower has issued the clearance.
	dep.LinedUp = true
	arr.ClearedToLand = true
	for _, pc := range []PendingContact{
		{ADSBCallsign: dep.ADSBCallsign, TCP: "1W", Type: PendingTransmissionReadyForDeparture},
		{ADSBCallsign: arr.ADSBCallsign, TCP: "1W", Type: PendingTransmissionRequestLandingClearance},
	} {
		if s.contactApplies(pc) {
			t.Errorf("type %d: contact still applies after the clearance", pc.Type)
		}
	}

	// Short of 2nm, the pilot asks for the landing clearance, once.
	arr.ClearedToLand = false
	s.checkLandingClearance(arr)
	if arr.RequestedLandingClearance {
		t.Errorf("asked for a landing clearance 5nm out")
	}
	arr.Nav.FlightState.Position = [2]float32{0, 1.5 / 60}
	// But not before it has checked in.
	s.addPendingContact(PendingContact{ADSBCallsign: arr.ADSBCallsign, TCP: "1W", Type: PendingTransmissionArrival})
	s.checkLandingClearance(arr)
	if arr.RequestedLandingClearance {
		t.Errorf("asked for a landing clearance before checking in")
	}
	s.PendingContacts = nil
	s.checkLandingClearance(arr)
	s.checkLandingClearance(arr)
	if !arr.RequestedLandingClearance || len(s.PendingContacts["1W"]) != 1 {
		t.Errorf("landing clearance requests: %v", s.PendingContacts["1W"])
	}
}
