// sim/tower.go
// Copyright(c) vice contributors, licensed under the GNU Public License, Version 3.
// SPDX: GPL-3.0-only

package sim

import (
	"time"

	av "github.com/mmp/vice/aviation"
	"github.com/mmp/vice/math"
	"github.com/mmp/vice/speech"
	"github.com/mmp/vice/util"
)

// This file has the sim's handling of airports where a human works local
// control ("tower"). At those airports, departures talk to the tower, call
// it ready at the runway, take off when it clears them, and stay with it
// until it sends them to departure. Arrivals that a virtual approach
// controller has cleared for the approach are switched to the tower on
// final; they need its landing clearance, asking for it on short final and
// going around without it. Airports whose tower is virtual are handled as
// they always have been: the sim launches departures and arrivals land on
// "_TOWER".

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
	// In the controlled tail of prespawn, switch as usual: the pilot's call
	// is waiting when the controller takes over.

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

// isTowerPosition reports whether pos is local control for the airport.
func (s *Sim) isTowerPosition(pos ControlPosition, airport av.ICAOAirportCode) bool {
	ctrl, ok := s.ControlPositions[TCP(pos)]
	return ok && ctrl.Role == av.RoleLocal && ctrl.Airport == airport
}

// onTowerFrequency reports whether the aircraft is talking to whoever works
// the tower at the airport.
func (s *Sim) onTowerFrequency(ac *Aircraft, airport av.ICAOAirportCode) bool {
	return ac.ControllerFrequency != "" && s.tcwWorksTower(s.State.TCWForPosition(ac.ControllerFrequency), airport)
}

// tcwWorksTower reports whether the controller at tcw works local control at
// the airport.
func (s *Sim) tcwWorksTower(tcw TCW, airport av.ICAOAirportCode) bool {
	for _, pos := range s.State.GetPositionsForTCW(tcw) {
		if s.isTowerPosition(pos, airport) {
			return true
		}
	}
	return false
}

// towerAirport returns the airport whose tower works the aircraft: the one it
// departs from or the one it is landing at.
func towerAirport(ac *Aircraft) av.ICAOAirportCode {
	if ac.IsDeparture() {
		return ac.DepartureAirport
	}
	return ac.ArrivalAirport
}

///////////////////////////////////////////////////////////////////////////
// Runway clearances

// The tower's runway clearance commands. They are matched exactly, ahead of
// the other commands, several of which they would otherwise be taken for:
// "CT<tcp>" is contact a controller, "C<approach>" an approach clearance and
// "L###" a left turn.
const (
	cmdClearedToLand     = "CTL"
	cmdClearedForTakeoff = "CTO"
	cmdLineUpAndWait     = "LUAW"
	cmdGoAround          = "GOAR"
)

// runTowerCommand runs command if it is one of the tower's runway
// clearances, reporting whether it was. Only a controller working the tower
// at the aircraft's airport can issue them.
func (s *Sim) runTowerCommand(tcw TCW, callsign av.ADSBCallsign, command string) (speech.CommandIntent, bool, error) {
	var clearance func(ac *Aircraft) speech.CommandIntent
	switch command {
	case cmdClearedToLand:
		clearance = s.clearToLand
	case cmdClearedForTakeoff:
		clearance = s.clearForTakeoff
	case cmdLineUpAndWait:
		clearance = s.lineUpAndWait
	case cmdGoAround:
		clearance = s.towerGoAround
	default:
		return nil, false, nil
	}

	intent, err := s.dispatchAircraftCommand(tcw, callsign,
		func(tcw TCW, ac *Aircraft) error {
			if !s.tcwWorksTower(tcw, towerAirport(ac)) {
				return ErrNotTowerPosition
			}
			if !s.TCWCanCommandAircraft(tcw, ac) {
				return av.ErrOtherControllerHasTrack
			}
			return nil
		},
		func(tcw TCW, ac *Aircraft) speech.CommandIntent {
			ac.LastInstructionTime = s.State.SimTime
			ac.LastInstructionFrequency = ac.ControllerFrequency
			return clearance(ac)
		})
	return intent, true, err
}

func (s *Sim) clearToLand(ac *Aircraft) speech.CommandIntent {
	appr := ac.Nav.Approach.Assigned
	if !ac.IsArrival() || appr == nil {
		return speech.MakeUnableIntent("unable, we're not landing")
	} else if !ac.Nav.Approach.Cleared {
		return speech.MakeUnableIntent("unable, we're not on the approach")
	}
	ac.ClearedToLand = true
	return speech.ClearedToLandIntent{Runway: appr.Runway}
}

func (s *Sim) towerGoAround(ac *Aircraft) speech.CommandIntent {
	if !ac.IsArrival() || ac.Nav.Approach.Assigned == nil || !ac.Nav.Approach.Cleared {
		return speech.MakeUnableIntent("unable, we're not on the approach")
	}
	ac.SentAroundByTower = true
	s.goAround(ac)
	return speech.GoAroundIntent{}
}

func (s *Sim) clearForTakeoff(ac *Aircraft) speech.CommandIntent {
	depState, runway, queue, idx, intent := s.departureAtRunway(ac)
	if intent != nil {
		return intent
	}

	dep := (*queue)[idx]
	*queue = util.DeleteSliceElement(*queue, idx)
	ac.ReadyForDeparture, ac.LinedUp = false, false
	s.startTakeoffRoll(depState, ac.DepartureAirport, runway, dep, s.State.SimTime)
	return speech.ClearedForTakeoffIntent{Runway: runway.Base()}
}

func (s *Sim) lineUpAndWait(ac *Aircraft) speech.CommandIntent {
	_, runway, _, _, intent := s.departureAtRunway(ac)
	if intent != nil {
		return intent
	}
	ac.LinedUp = true
	return speech.LineUpAndWaitIntent{Runway: runway.Base()}
}

// departureAtRunway finds a departure that has reached the runway in its
// runway's queue. If it isn't there, it returns what the pilot says instead.
func (s *Sim) departureAtRunway(ac *Aircraft) (*RunwayLaunchState, av.RunwayID, *[]DepartureAircraft, int, speech.CommandIntent) {
	if !ac.IsDeparture() {
		return nil, "", nil, 0, speech.MakeUnableIntent("unable, we're not departing")
	} else if !ac.WaitingForLaunch {
		return nil, "", nil, 0, speech.MakeUnableIntent("unable, we're already airborne")
	}

	for runway, depState := range util.SortedMap(s.DepartureState[ac.DepartureAirport]) {
		for _, queue := range []*[]DepartureAircraft{&depState.ReleasedIFR, &depState.ReleasedVFR} {
			for idx, dep := range *queue {
				if dep.ADSBCallsign != ac.ADSBCallsign {
					continue
				}
				if !ac.ReadyForDeparture && !ac.LinedUp {
					return nil, "", nil, 0, speech.MakeUnableIntent("unable, we're still taxiing to the runway")
				}
				return depState, runway, queue, idx, nil
			}
		}
	}
	return nil, "", nil, 0, speech.MakeUnableIntent("unable, we're not ready yet")
}

// At most this many departures wait at a runway having called a human tower
// ready; the others stay in the queue behind them.
const maxTowerReady = 3

// callTowerReady has the departures in a runway's queue reach the runway and
// call the tower ready, a few at a time, in the order they joined it.
func (s *Sim) callTowerReady(depState *RunwayLaunchState, airport av.ICAOAirportCode,
	depRunway av.RunwayID, now Time) {
	if s.prespawn {
		// Let them call once the sim is running.
		return
	}
	tower, ok := s.humanTowerPosition(airport, string(depRunway))
	if !ok {
		return
	}

	ready := 0
	for _, queue := range [][]DepartureAircraft{depState.ReleasedIFR, depState.ReleasedVFR} {
		for _, dep := range queue {
			if s.Aircraft[dep.ADSBCallsign].ReadyForDeparture {
				ready++
			}
		}
	}

	for _, queue := range []*[]DepartureAircraft{&depState.ReleasedIFR, &depState.ReleasedVFR} {
		for i := range *queue {
			dep := &(*queue)[i]
			ac := s.Aircraft[dep.ADSBCallsign]
			if ac.ReadyForDeparture {
				continue
			}
			if dep.ReadyCallTime.IsZero() {
				// Taxi time from the gate.
				dep.ReadyCallTime = dep.QueuedTime.Add(s.Rand.DurationRange(45*time.Second, 150*time.Second))
			}
			if ready >= maxTowerReady || now.Before(dep.ReadyCallTime) {
				continue
			}

			ac.ReadyForDeparture = true
			// The tower is the one to call even if the consolidation has
			// changed since the aircraft spawned.
			s.setControllerFrequency(ac, tower)
			s.enqueuePilotTransmission(ac.ADSBCallsign, tower, PendingTransmissionReadyForDeparture)
			ready++
		}
	}
}

// Pilots who haven't been cleared to land ask for the clearance this close
// to the threshold (nm).
const landingClearanceRequestDistance = 2

// checkLandingClearance has a pilot on a human tower's frequency without a
// landing clearance ask for it on short final.
func (s *Sim) checkLandingClearance(ac *Aircraft) {
	appr := ac.Nav.Approach.Assigned
	if appr == nil || !ac.Nav.Approach.Cleared || ac.ClearedToLand || ac.RequestedLandingClearance ||
		!s.onTowerFrequency(ac, ac.ArrivalAirport) ||
		math.NMDistance2LL(ac.Position(), appr.Threshold) > landingClearanceRequestDistance ||
		s.hasPendingCheckIn(ac.ADSBCallsign) { // check in first
		return
	}
	ac.RequestedLandingClearance = true
	s.enqueuePilotTransmission(ac.ADSBCallsign, TCP(ac.ControllerFrequency), PendingTransmissionRequestLandingClearance)
}

// mustGoAroundWithoutClearance reports whether an arrival reaching the
// runway has to go around because a human tower hasn't cleared it to land.
func (s *Sim) mustGoAroundWithoutClearance(ac *Aircraft) bool {
	return ac.Nav.Approach.Assigned != nil && !ac.ClearedToLand && s.hasHumanTower(ac.ArrivalAirport)
}

// queuedDepartureRunway returns the runway whose queue the departure is in.
func (s *Sim) queuedDepartureRunway(ac *Aircraft) (av.RunwayID, bool) {
	for runway, depState := range util.SortedMap(s.DepartureState[ac.DepartureAirport]) {
		for _, queue := range [][]DepartureAircraft{depState.ReleasedIFR, depState.ReleasedVFR} {
			for _, dep := range queue {
				if dep.ADSBCallsign == ac.ADSBCallsign {
					return runway, true
				}
			}
		}
	}
	return "", false
}

// towerCheckIn returns what an arrival says when it first calls a human
// tower, if it's calling one: where it is on final.
func (s *Sim) towerCheckIn(ac *Aircraft, tcp TCP) (*speech.RadioTransmission, bool) {
	appr := ac.Nav.Approach.Assigned
	if appr == nil || !s.tcwWorksTower(s.State.TCWForPosition(tcp), ac.ArrivalAirport) {
		return nil, false
	}
	miles := int(math.NMDistance2LL(ac.Position(), appr.Threshold) + 0.5)
	if miles < 2 {
		return speech.MakeContactTransmission("[short final|on short final] runway {rwy}", appr.Runway), true
	}
	return speech.MakeContactTransmission("[{num} mile final runway {rwy}|{num} mile final for {rwy}|on a {num} mile final, runway {rwy}]",
		miles, appr.Runway), true
}

// towerGoAroundProcedure adjusts a go-around at an airport with a human
// tower: the aircraft stays with the tower, which sends it to departure when
// it chooses and decides for itself when to resume departures. A pilot who
// went around on their own tells the tower. It reports whether the airport
// has a human tower.
func (s *Sim) towerGoAroundProcedure(ac *Aircraft, proc *GoAroundProcedure, runway string) bool {
	if _, ok := s.humanTowerPosition(ac.ArrivalAirport, runway); !ok {
		return false
	}

	proc.HandoffController = ""
	ac.ClearedToLand, ac.RequestedLandingClearance = false, false
	// The track may have been dropped as the aircraft neared the runway; the
	// tower will want to send it to approach.
	sfp := ac.FlightPlan
	if sfp == nil {
		sfp = s.STARSComputer.lookupFlightPlanByACID(ACID(ac.ADSBCallsign))
	}
	s.reassociateDroppedFlightPlan(ac, sfp)
	if !ac.SentAroundByTower && s.onTowerFrequency(ac, ac.ArrivalAirport) {
		s.enqueuePilotTransmission(ac.ADSBCallsign, TCP(ac.ControllerFrequency), PendingTransmissionGoAround)
	}
	ac.SentAroundByTower = false
	return true
}

// resequenceTowerGoArounds removes the aircraft that went around at an
// airport with a human tower and have since been sent to a virtual
// controller: the approach controller would take them around for another
// approach, which the sim doesn't model.
func (s *Sim) resequenceTowerGoArounds() {
	var resequence []*Aircraft
	for ac := range util.SortedMapValues(s.Aircraft) {
		if ac.WentAround && s.isVirtualController(ac.ControllerFrequency) && s.hasHumanTower(ac.ArrivalAirport) {
			resequence = append(resequence, ac)
		}
	}
	for _, ac := range resequence {
		s.deleteAircraft(ac, DeleteResequenced)
	}
}

// surfaceTracks returns tracks for the aircraft on the ground that a human
// tower works: the departures that have called it ready. They aren't radar
// visible, so they aren't in the derived state's Tracks, but the tower
// issues them clearances.
func (s *Sim) surfaceTracks() map[av.ADSBCallsign]*Track {
	var tracks map[av.ADSBCallsign]*Track
	for callsign, ac := range util.SortedMap(s.Aircraft) {
		if s.isRadarVisible(ac) || !ac.IsDeparture() || !ac.WaitingForLaunch ||
			!(ac.ReadyForDeparture || ac.LinedUp) || !s.hasHumanTower(ac.DepartureAirport) {
			continue
		}
		if tracks == nil {
			tracks = make(map[av.ADSBCallsign]*Track)
		}
		tracks[callsign] = s.makeTrack(ac)
	}
	return tracks
}

// setTowerTrackState fills in a track's runway clearance state if a human
// tower works the aircraft.
func (s *Sim) setTowerTrackState(ac *Aircraft, trk *Track) {
	if !s.hasHumanTower(towerAirport(ac)) {
		return
	}
	if ac.IsDeparture() && ac.WaitingForLaunch {
		if rwy, ok := s.queuedDepartureRunway(ac); ok {
			trk.TowerRunway = rwy.Base()
		}
	} else if appr := ac.Nav.Approach.Assigned; appr != nil && ac.IsArrival() {
		trk.TowerRunway = appr.Runway
	}
	trk.ReadyForDeparture = ac.ReadyForDeparture
	trk.LinedUp = ac.LinedUp
	trk.ClearedToLand = ac.ClearedToLand
}
