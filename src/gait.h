#ifndef _GAIT_H
#define _GAIT_H

#include "kinematics.h"

constexpr float GAIT_PERIOD_S = 2.0f;
constexpr float GAIT_STANCE_Z = -0.125f;
constexpr float GAIT_SWING_START = 0.15f;
constexpr float GAIT_SWING_END = 0.50f;
constexpr float GAIT_BODY_SHIFT = 0.018f;

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
