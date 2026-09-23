#ifndef _GAIT_H
#define _GAIT_H

#include "kinematics.h"

constexpr float GAIT_PERIOD_S = 2.0f;
// Leave knee flexion room for 20 mm lift without exceeding the 90-degree
// model/calibration limit (the former -0.125 m stance was too crouched).
constexpr float GAIT_STANCE_Z = -0.135f;
constexpr float GAIT_SWING_START = 0.15f;
constexpr float GAIT_SWING_END = 0.50f;
constexpr float GAIT_BODY_SHIFT = 0.018f;
constexpr float GAIT_STEP_LENGTH = 0.020f;
// Original v2 placo_defaults.json requests 20 mm of foot clearance.
constexpr float GAIT_STEP_HEIGHT = 0.020f;
// Match the old walk engine's rise_duration=0.2: hold peak height
// between 40% and 60% of swing, with smooth lift and landing.
constexpr float GAIT_LIFT_HOLD_FRACTION = 0.20f;

void footTrajectory(
    float phase,
    float stepLength,
    float stepHeight,
    float z0,
    FootTarget &foot);

void generateFeet(
    float gaitPhase,
    FootTarget &left,
    FootTarget &right);

#endif
