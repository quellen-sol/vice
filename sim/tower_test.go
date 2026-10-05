// sim/tower_test.go
// Copyright(c) vice contributors, licensed under the GNU Public License, Version 3.
// SPDX: GPL-3.0-only

package sim

import (
	"io"
	"log/slog"
	"testing"

	av "github.com/mmp/vice/aviation"
	"github.com/mmp/vice/log"
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
