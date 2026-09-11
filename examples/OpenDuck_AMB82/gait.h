/*
 * gait.h
 *
 * Gait generator for the OpenDuck Mini v2.
 *
 * Produces the five controllable foot variables (x, y, z, roll, heading) in the
 * hip frame from a single walking phase. The legs are separated by half a
 * cycle so the robot can stand on either foot.
 */
#ifndef _GAIT_H
#define _GAIT_H

#include "Arduino.h"
#include "kinematics.h"

struct GaitConfig {
  float phaseHz;   // walking cycle frequency (Hz)
  float stepLen;   // forward / backward swing (metres)
  float lift;      // vertical swing amplitude (metres)
  float footZ;     // standing height (metres)
  float sway;      // lateral body sway (metres)
  float roll;      // target roll during stance (metres)
};

// Set the gait profile. Defaults: level ground, short step, small sway.
void gaitConfig(GaitConfig &config);

// Advance the gait phase by dt seconds.
void gaitAdvance(float dt);

// Compute feet poses for the given legs.
void gaitFeet(FootPose &footL, FootPose &footR, float legSpacing);

// Query the phase (0..1) within the current cycle, for diagnostics/testing.
float gaitPhase();

// Set the gait phase directly, for testing.
void gaitSetPhase(float phase);

#endif
