#ifndef _GAIT_H
#define _GAIT_H

#include "kinematics.h"

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