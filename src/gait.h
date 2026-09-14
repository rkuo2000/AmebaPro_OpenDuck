#ifndef _GAIT_H
#define _GAIT_H

#include "kinematics.h"

struct FootTarget {
    float x;      // forward (m)
    float y;      // left (m)
    float z;      // up (m)
    float yaw;    // foot yaw (rad)
    float pitch;  // foot pitch (rad)
};

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