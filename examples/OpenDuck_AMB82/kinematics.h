/*
 * kinematics.h
 *
 * 2-D leg inverse forward kinematics for the OpenDuck Mini v2.
 *
 * Each leg has 5 actuated joints (hip yaw, hip roll, hip pitch, knee, ankle)
 * and NO ankle-roll. Therefore the foot pose reduces to 5 independent
 * variables: x (forward), y (lateral), z (height), roll (about +y), and
 * heading (about +z). This solver targets exactly those five.
 *
 * Hip coordinate system:
 *   +X = forward, +Y = robot left, +Z = up
 *   Hip origin = hip centre, first link = thigh (L1), second link = shank (L2).
 */
#ifndef _KINEMATICS_H
#define _KINEMATICS_H

#include "Arduino.h"

struct Point3 {
  float x;
  float y;
  float z;
};

struct LegAngles {
  float hipYaw;
  float hipRoll;
  float hipPitch;
  float knee;
  float ankle;
};

// Foot pose for the five controllable DOF.
struct FootPose {
  Point3 position; // foot tip in hip frame: X forward, Y left, Z up
  Point3 velocity; // foot tip velocity in hip frame (for feed-forward)
  float  roll;     // desired foot roll (about +y) in hip frame
  float  rollV;    // foot roll rate
  float  heading;  // desired foot heading (about +z) in hip frame
  float  headingV; // foot heading rate
};

// Inverse kinematics.
//
// 1. Decompose the yaw/heading about +z.
// 2. Rotate the target into the hip-roll plane; roll = angle of the leg plane.
// 3. Solve the planar 2-link leg (thigh L1, shank L2) for the hip-pitch and
//    knee angles using the cosine rule, choosing the extended configuration.
// 4. Ankle = foot roll - hip pitch - knee (keeps the foot flat in the plane).
//
// Returns true on success, false if the target is out of reach or the knee is
// fully extended.
bool legIK(const FootPose &foot, float &hipYaw, float &hipRoll,
           float &hipPitch, float &knee, float &ankle);

// Forward kinematics. Given joint angles, returns the foot tip position.
bool legFK(float hipYaw, float hipRoll, float hipPitch, float knee,
           Point3 &point);
#endif
