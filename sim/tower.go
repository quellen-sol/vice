// sim/tower.go
// Copyright(c) vice contributors, licensed under the GNU Public License, Version 3.
// SPDX: GPL-3.0-only

package sim

import (
	av "github.com/mmp/vice/aviation"
	"github.com/mmp/vice/math"
	"github.com/mmp/vice/util"
)

// This file has the sim's handling of airports where a human works local
// control ("tower"). At those airports, departures talk to the tower until
// the tower sends them to departure, and arrivals that a virtual approach
// controller has cleared for the approach are switched to the tower on
// final. Airports whose tower is virtual are handled as they always have
// been: the sim launches departures and arrivals land on "_TOWER".

// Arrivals that a virtual controller has cleared for the approach are
// switched to a human tower controller once they are this close to the
// runway threshold (nm), roughly at the final approach fix.
const towerSwitchDistance = 6

// humanTowerPosition returns the position of the controller working local
// control for the given runway at the airport, if a human works it in the
// scenario. When the airport's runways are split between local control
// positions, the one that lists the runway is used (with an empty runway,
// any of them); the result is whoever that position is currently
// consolidated to.
func (s *Sim) humanTowerPosition(airport av.ICAOAirportCode, runway string) (TCP, bool) {
	// Prefer a position that works the runway and then take the first in
	// sorted order (without sorting, since this is called for each
	// aircraft each update) so that the choice is deterministic.
	var tower TCP
	towerWorksRunway := false
	for tcp, ctrl := range s.ControlPositions {
		if ctrl.Role != av.RoleLocal || ctrl.Airport != airport ||
			!s.ScenarioDefaultConsolidation.IsHumanPosition(tcp) {
			continue
		}
		works := runway != "" && ctrl.WorksRunway(av.RunwayID(runway))
		if tower == "" || (works && !towerWorksRunway) || (works == towerWorksRunway && tcp < tower) {
			tower, towerWorksRunway = tcp, works
		}
	}
	if tower == "" {
		return "", false
	}
	return s.State.ResolveController(tower), true
}

// hasHumanTower reports whether a human works local control at the airport.
func (s *Sim) hasHumanTower(airport av.ICAOAirportCode) bool {
	_, ok := s.humanTowerPosition(airport, "")
	return ok
}

// assignTowerDepartureController sets up a departure from an airport whose
// tower a human works: the departure controller tracks it, but the pilot
// talks to the tower, which releases it and later sends it to departure.
func (s *Sim) assignTowerDepartureController(ac *Aircraft, nasFp *FlightPlan, ap *av.Airport,
	exitRoute *av.ExitRoute, departureAirport av.ICAOAirportCode, runway string, tower TCP) {
	dep := ap.DepartureController
	if dep == "" {
		dep = exitRoute.DepartureController
	}
	if dep == "" {
		dep = s.GetDepartureController(departureAirport, runway, exitRoute.SID)
	}
	if dep == "" {
		dep = s.ScenarioRootPosition()
	}

	nasFp.TrackingController = dep
	nasFp.OwningTCW = s.tcwForPosition(dep)
	nasFp.InboundHandoffController = util.Select(s.isVirtualController(dep), exitRoute.HandoffController, dep)

	ac.ControllerFrequency = tower
	// The tower sends the departure to the departure controller; it never
	// checks in on its own (-1 also keeps it from being taken for one
	// waiting on a /tc point).
	ac.DepartureContactAltitude = -1
	ac.HoldForRelease = false
}

// checkSwitchToTower switches an arrival that a virtual controller has
// cleared for the approach to the human tower controller once it's close
// to the runway. It returns true if the aircraft was deleted.
func (s *Sim) checkSwitchToTower(ac *Aircraft) bool {
	appr := ac.Nav.Approach.Assigned
	if appr == nil || !ac.Nav.Approach.Cleared || ac.GotContactTower ||
		!s.isVirtualController(ac.ControllerFrequency) {
		return false
	}
	tower, ok := s.humanTowerPosition(ac.ArrivalAirport, appr.Runway)
	if !ok || math.NMDistance2LL(ac.Position(), appr.Threshold) > towerSwitchDistance {
		return false
	}

	if s.prespawnUncontrolledOnly {
		// Don't start the sim with arrivals about to call the tower.
		s.deleteAircraft(ac, DeletePrespawn)
		return true
	}
	if s.prespawn {
		// Leave it for once the sim is running.
		return false
	}

	ac.GotContactTower = true
	s.enqueueControllerContact(ac, tower, ac.ControllerFrequency, 0)
	return false
}

// sendToHumanTower sends an arrival that a radar controller has told to
// contact the tower to the airport's human tower controller, if there is
// one. It returns false if the tower is virtual.
func (s *Sim) sendToHumanTower(tcw TCW, ac *Aircraft) bool {
	var runway string
	if appr := ac.Nav.Approach.Assigned; appr != nil {
		runway = appr.Runway
	}
	tower, ok := s.humanTowerPosition(ac.ArrivalAirport, runway)
	if !ok {
		return false
	}
	s.cancelFutureFrequencyChange(ac.ADSBCallsign)
	s.setControllerFrequency(ac, "")
	s.enqueueControllerContact(ac, tower, s.State.PrimaryPositionForTCW(tcw), 0)
	return true
}
