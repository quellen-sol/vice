// speech/stt/tower_test.go
// Copyright(c) vice contributors, licensed under the GNU Public License, Version 3.
// SPDX: GPL-3.0-only

package stt

import (
	"strings"
	"testing"
)

// towerTestAircraft returns a departure holding short of 35R and an arrival
// on final for 35L, both talking to a controller who works the tower when
// tower is set.
func towerTestAircraft(tower bool) map[string]Aircraft {
	rwy := func(r string) string {
		if tower {
			return r
		}
		return ""
	}
	return map[string]Aircraft{
		"Southwest 739": {Callsign: "SWA739", State: "departure", TowerControl: tower, TowerRunway: rwy("35R")},
		"SkyWest 8712": {Callsign: "SKW8712", State: "cleared approach", AssignedApproach: "ILS Runway 35L",
			TowerControl: tower, TowerRunway: rwy("35L")},
		"November 123AB": {Callsign: "N123AB", State: "vfr flight following", TowerControl: tower},
	}
}

func TestTowerCommands(t *testing.T) {
	tests := []struct {
		transcript string
		expected   string
	}{
		// Takeoff clearances.
		{"southwest seven three nine runway three five right cleared for takeoff", "SWA739 CTO"},
		{"southwest 739 cleared for takeoff runway 35 right", "SWA739 CTO"},
		{"southwest 739 runway 35 right cleared for take-off", "SWA739 CTO"},
		{"southwest 739 cleared for take off", "SWA739 CTO"},
		{"southwest 739 runway three five right line up and wait", "SWA739 LUAW"},
		{"southwest 739 lineup and wait", "SWA739 LUAW"},

		// Landing clearances.
		{"skywest eight seven one two runway three five left cleared to land", "SKW8712 CTL"},
		{"skywest 8712 cleared to land", "SKW8712 CTL"},
		{"skywest 8712 clear to land runway 35 left", "SKW8712 CTL"},
		{"skywest 8712 go around", "SKW8712 GOAR"},

		// Wind and other additions controllers make.
		{"skywest 8712 wind three three zero at one zero runway three five left cleared to land", "SKW8712 CTL"},
		{"southwest 739 runway three five right wind calm cleared for takeoff", "SWA739 CTO"},
		{"skywest 8712 go-around", "SKW8712 GOAR"},
		{"skywest 8712 cross runway three five left", "SKW8712 CROSS"},
		{"southwest 739 cross runway 35 left", "SWA739 CROSS"},
		// A garbled callsign can leave a word unmatched; what follows
		// isn't heard as a speed for an aircraft on the ground.
		{"southwest 739 zulu runway three five right line up and wait", "SWA739 LUAW"},

		// The tower's other instructions still work.
		{"southwest 739 contact departure", "SWA739 FC"},
		// "Go ahead" is still "go ahead".
		{"november one two three alpha bravo go ahead", "N123AB GA"},
	}

	provider := NewTranscriber(nil)
	for _, tt := range tests {
		t.Run(tt.transcript, func(t *testing.T) {
			result, err := provider.DecodeTranscript(towerTestAircraft(true), tt.transcript, "")
			if err != nil {
				t.Fatalf("unexpected error: %v", err)
			}
			if result != tt.expected {
				t.Errorf("got %q, want %q", result, tt.expected)
			}
		})
	}
}

// TestTowerCommandsNeedTower checks that the runway clearances aren't heard
// from a controller who doesn't work the aircraft's tower.
func TestTowerCommandsNeedTower(t *testing.T) {
	provider := NewTranscriber(nil)
	for _, transcript := range []string{
		"southwest 739 runway three five right cleared for takeoff",
		"southwest 739 line up and wait",
		"skywest 8712 runway three five left cleared to land",
		"skywest 8712 go around",
	} {
		result, err := provider.DecodeTranscript(towerTestAircraft(false), transcript, "")
		if err != nil {
			t.Fatalf("%s: unexpected error: %v", transcript, err)
		}
		for _, cmd := range strings.Fields(result)[min(1, len(strings.Fields(result))):] {
			if isTowerCommand(cmd) {
				t.Errorf("%q from a radar position: got %q", transcript, result)
			}
		}
	}
}

func TestValidateTowerCommands(t *testing.T) {
	arrival := Aircraft{State: "cleared approach", TowerControl: true}
	departure := Aircraft{State: "departure", TowerControl: true}
	for _, tc := range []struct {
		cmd   string
		ac    Aircraft
		valid bool
	}{
		{"CTL", arrival, true},
		{"GOAR", arrival, true},
		{"CTO", departure, true},
		{"LUAW", departure, true},
		{"CTL", departure, false},
		{"CTO", arrival, false},
		{"LUAW", arrival, false},
		{"CTL", Aircraft{State: "cleared approach"}, false}, // not the tower
		{"S250", departure, false},                          // on the ground
		{"S250", Aircraft{State: "departure", Speed: 180, TowerControl: true}, true},
		{"SQ1234", departure, true},
	} {
		if err := validateCommand(tc.cmd, tc.ac); (err == "") != tc.valid {
			t.Errorf("%s for %+v: got error %q, want valid %v", tc.cmd, tc.ac, err, tc.valid)
		}
	}
	for _, cmd := range []string{"CTL", "CTO", "LUAW", "GOAR"} {
		if cat := getCommandCategory(cmd); cat != "runway_clearance" {
			t.Errorf("%s: category %q", cmd, cat)
		}
	}
}
