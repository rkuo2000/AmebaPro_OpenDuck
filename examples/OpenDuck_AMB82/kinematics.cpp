/*
 * kinematics.cpp
 *
 * 2-D leg kinematics for OpenDuck Mini v2.
 *
 * Hip frame: +X forward, +Y robot left, +Z up. Thigh = L1, shank = L2.
 * The leg has 5 actuated joints and NO ankle-roll, so the foot pose reduces to
 * the five controllable variables handled here.
 *
 * This is the "sagittal embedding" convention:
 *   Rz(yaw) * Rx(roll) * [a, 0, b]
 * which matches the derivation in the README. With this convention:
 *   - hipRoll = atan2(y, -z)            (foot out -> +roll, foot in  -> -roll)
 *   - hipPitch = atan2(b, a) - cos^{-1}(...)  (positive pitch lifts the foot)
 * The ankle equals -hipPitch - knee, so the foot stays flat in the leg plane.
 */
#include "kinematics.h"
#include <math.h>
#include "robot_config.h"

#ifndef PI
#define PI 3.14159265358979323846
#endif

#define L1 BODY_L1
#define L2 BODY_L2

#define LEG_MIN_D (0.001f + fabsf((float)L1 - (float)L2))
#define LEG_MAX_D ((float)L1 + (float)L2 - 0.001f)

bool legFK(float hipYaw, float hipRoll, float hipPitch, float knee, Point3 &point)
{
  const float cy = cosf(hipYaw);
  const float sy = sinf(hipYaw);

  const float a_leg = (float)L1 * cosf(hipPitch) + (float)L2 * cosf(hipPitch + knee);
  const float b_leg = (float)L1 * sinf(hipPitch) + (float)L2 * sinf(hipPitch + knee);

  // Leg-plane coordinates: forward = a_leg, planar vertical = b_leg. The roll
  // splits b_leg between world lateral (y) and world vertical (z), with z
  // negative = foot down:  z = -b*cos(roll),  y_planar = b*sin(roll).
  const float yr = b_leg * sinf(hipRoll);
  const float z  = -b_leg * cosf(hipRoll);

  point.x =  cy * a_leg - sy * yr;
  point.y =  sy * a_leg + cy * yr;
  point.z =  z;

  return true;
}

bool legIK(const FootPose &foot, float &hipYaw, float &hipRoll,
           float &hipPitch, float &knee, float &ankle)
{
  const float x = foot.position.x;
  const float y = foot.position.y;
  const float z = foot.position.z;

  // 1. Hip yaw / heading about +z.
  hipYaw = foot.heading;
  const float cy = cosf(-hipYaw);
  const float sy = sinf(-hipYaw);

  const float xr = cy * x - sy * y;
  const float yr = sy * x + cy * y;

  // 2. Hip roll (angle of the leg plane). +roll when the foot lifts out.
  hipRoll = atan2f(yr, -z);

  // Sagittal-plane vertical distance (the leg-plane altitude).
  const float b = sqrtf(yr * yr + z * z);

  // 3. Planar 2-link leg.
  const float a = sqrtf(xr * xr + b * b);
  if (a < LEG_MIN_D || a > LEG_MAX_D) {
    return false;
  }

  const float cosK = ((float)L1 * (float)L1 + (float)L2 * (float)L2 - a * a) /
                     (2.0f * (float)L1 * (float)L2);
  const float clamped = (cosK > 1.0f) ? 1.0f : ((cosK < -1.0f) ? -1.0f : cosK);
  knee = fabsf(PI - acosf(clamped));

  const float cosBeta = ((float)L1 * (float)L1 + a * a - (float)L2 * (float)L2) /
                        (2.0f * (float)L1 * a);
  const float bclamp = (cosBeta > 1.0f) ? 1.0f : ((cosBeta < -1.0f) ? -1.0f : cosBeta);
  const float beta = acosf(bclamp);

  hipPitch = atan2f(b, xr) - beta;

  // 4. Ankle keeps the foot flat in the leg plane.
  ankle = foot.roll - hipPitch - knee;

  return true;
}
