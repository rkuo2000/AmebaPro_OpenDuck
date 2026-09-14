#ifndef _KINEMATICS_H
#define _KINEMATICS_H

#include <Arduino.h>
#include <math.h>

constexpr float L1 = 0.07865f;  // thigh link length (m)
constexpr float L2 = 0.0865f;   // lower link length (m) - typically slightly longer

struct FootTarget {
    float x;      // forward (m)
    float y;      // left (m)
    float z;      // up (m)
    float yaw;    // foot yaw (rad)
    float pitch;  // foot pitch (rad)
};

struct LegAngles {
    float hipYaw;     // hip yaw (rad) - from foot yaw
    float hipRoll;    // hip roll = atan2(y, -z) after yaw transform (rad)
    float hipPitch;   // hip pitch = alpha - beta (rad)
    float knee;       // knee angle (rad)
    float ankle;      // ankle angle = foot.pitch - hipPitch - knee (rad)
};

inline float clampf(float x, float lo, float hi)
{
    if (x < lo) return lo;
    if (x > hi) return hi;
    return x;
}

bool legIK(const FootTarget &foot, LegAngles &q);

#endif