// speech/stt/tower.go
// Copyright(c) vice contributors, licensed under the GNU Public License, Version 3.
// SPDX: GPL-3.0-only

package stt

import (
	"fmt"
	"reflect"
	"strings"

	av "github.com/mmp/vice/aviation"
	"github.com/mmp/vice/sim"
)

// The tower's runway clearances. Their templates are only tried for aircraft
// whose airport's tower the controller works (Aircraft.TowerControl), so that
// "cleared to land" can't be heard from a radar position and phrases like
// "go around" don't compete with radar controllers' "go ahead".

// isTowerCommand reports whether cmd is one of the tower's runway clearances.
func isTowerCommand(cmd string) bool {
	switch cmd {
	case "CTL", "CTO", "LUAW", "GOAR":
		return true
	}
	return false
}

// validateTowerCommand checks that a runway clearance suits the aircraft:
// landing clearances and go-arounds are for arrivals, takeoff clearances for
// departures.
func validateTowerCommand(cmd string, ac Aircraft) string {
	if !ac.TowerControl {
		return "runway clearance from a controller not working the tower"
	}
	switch cmd {
	case "CTL", "GOAR":
		if ac.State != "arrival" && ac.State != "cleared approach" {
			return "landing clearance or go around for an aircraft that isn't arriving"
		}
	case "CTO", "LUAW":
		if ac.State != "departure" {
			return "takeoff clearance for an aircraft that isn't departing"
		}
	}
	return ""
}

func registerTowerCommands() {
	// They outrank the approach clearance templates, which "cleared ..."
	// would otherwise also match.
	const towerPriority = 20

	// "Runway three five left, cleared to land" (or the runway after).
	registerSTTCommand(
		"[runway {tower_runway}] cleared [to] land [runway {tower_runway}]",
		func(before, after *string) string { return "CTL" },
		WithName("cleared_to_land"),
		WithPriority(towerPriority),
		WithTowerOnly(),
	)
	registerSTTCommand(
		"[runway {tower_runway}] cleared [for] takeoff [runway {tower_runway}]",
		func(before, after *string) string { return "CTO" },
		WithName("cleared_for_takeoff"),
		WithPriority(towerPriority),
		WithTowerOnly(),
	)
	registerSTTCommand(
		"[runway {tower_runway}] line|lineup [up] [and] wait [runway {tower_runway}]",
		func(before, after *string) string { return "LUAW" },
		WithName("line_up_and_wait"),
		WithPriority(towerPriority),
		WithTowerOnly(),
	)
	registerSTTCommand(
		"go around",
		func() string { return "GOAR" },
		WithName("go_around"),
		WithPriority(towerPriority),
		WithTowerOnly(),
	)
}

// validateOnGround rejects a speed assignment for a departure the tower is
// working that is still on the ground, which has no speed to adjust. (When a
// garbled callsign leaves a word or two unmatched, "... runway three five"
// can otherwise be read as "slow to 350".)
func validateOnGround(cmd string, ac Aircraft) string {
	if ac.TowerControl && ac.State == "departure" && ac.Speed == 0 &&
		len(cmd) > 1 && cmd[0] == 'S' && IsNumber(cmd[1:]) {
		return "speed assignment for an aircraft on the ground"
	}
	return ""
}

// towerRunwayParser extracts the runway of a runway clearance ("three five
// left"). Any runway is accepted: the aircraft uses the runway it was
// assigned whichever the controller names.
type towerRunwayParser struct{}

func (p *towerRunwayParser) goType() reflect.Type {
	return reflect.TypeFor[string]()
}

func (p *towerRunwayParser) parse(tokens []Token, pos int, ac Aircraft) (any, int, string) {
	if pos >= len(tokens) || tokens[pos].Type != TokenNumber || tokens[pos].Value < 1 || tokens[pos].Value > 36 {
		return nil, 0, ""
	}
	runway, consumed := fmt.Sprintf("%d", tokens[pos].Value), 1
	if pos+1 < len(tokens) {
		switch strings.ToLower(tokens[pos+1].Text) {
		case "left", "l":
			runway, consumed = runway+"L", 2
		case "right", "r":
			runway, consumed = runway+"R", 2
		case "center", "c":
			runway, consumed = runway+"C", 2
		}
	}
	return runway, consumed, ""
}

// userWorksTower reports whether the controller at userTCW works local
// control at the airport the track departs from or is landing at.
func userWorksTower(state *sim.UserState, userTCW sim.TCW, trk *sim.Track) bool {
	airport := trk.ArrivalAirport
	if trk.TypeOfFlight == av.FlightTypeDeparture {
		airport = trk.DepartureAirport
	}
	for _, pos := range state.GetPositionsForTCW(userTCW) {
		if ctrl, ok := state.Controllers[pos]; ok && ctrl.Role == av.RoleLocal && ctrl.Airport == airport {
			return true
		}
	}
	return false
}
