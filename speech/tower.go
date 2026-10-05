// speech/tower.go
// Copyright(c) vice contributors, licensed under the GNU Public License, Version 3.
// SPDX: GPL-3.0-only

package speech

import (
	"github.com/mmp/vice/rand"
)

// Pilot readbacks of the tower's runway clearances.

// ClearedToLandIntent is the readback of "runway 35L, cleared to land".
type ClearedToLandIntent struct {
	Runway string
}

func (c ClearedToLandIntent) Render(rt *RadioTransmission, r *rand.Rand) {
	rt.Add("[cleared to land|cleared to land runway {rwy}|runway {rwy}, cleared to land]", c.Runway)
}

// ClearedForTakeoffIntent is the readback of "runway 35L, cleared for
// takeoff".
type ClearedForTakeoffIntent struct {
	Runway string
}

func (c ClearedForTakeoffIntent) Render(rt *RadioTransmission, r *rand.Rand) {
	rt.Add("[cleared for takeoff|cleared for takeoff runway {rwy}|runway {rwy}, cleared for takeoff]", c.Runway)
}

// LineUpAndWaitIntent is the readback of "runway 35L, line up and wait".
type LineUpAndWaitIntent struct {
	Runway string
}

func (l LineUpAndWaitIntent) Render(rt *RadioTransmission, r *rand.Rand) {
	rt.Add("[line up and wait|line up and wait runway {rwy}|runway {rwy}, line up and wait|lining up and waiting runway {rwy}]", l.Runway)
}

// CrossRunwayIntent is the readback of "cross runway 35L".
type CrossRunwayIntent struct {
	Runway string
}

func (c CrossRunwayIntent) Render(rt *RadioTransmission, r *rand.Rand) {
	rt.Add("[cross runway {rwy}|crossing runway {rwy}|cross {rwy}]", c.Runway)
}

// GoAroundIntent is the readback of the tower's "go around".
type GoAroundIntent struct{}

func (GoAroundIntent) Render(rt *RadioTransmission, r *rand.Rand) {
	rt.Add("[going around|on the go|going around now]")
}
