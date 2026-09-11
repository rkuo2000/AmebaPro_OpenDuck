/*
 * gait.cpp
 *
 * Gait generator for the OpenDuck Mini v2.
 */
#include "gait.h"
#include <math.h>
#include "robot_config.h"

float gaitPhase()
{
  return _gaitState.phase;
}

void gaitSetPhase(float phase)
{
  _gaitState.phase = phase;
  while (_gaitState.phase >= 1.0f) {
    _gaitState.phase -= 1.0f;
  }
}

void gaitConfig(GaitConfig &config)
{
  config.phaseHz = WALK_PHASE_HZ;
  config.stepLen = FOOT_STEP;
  config.lift = FOOT_HEIGHT;
  config.footZ = -BODY_L1 - BODY_L2 + FOOT_HEIGHT;
  config.sway = HIP_W * 0.5f;
  config.roll = 0.0f;
}

static void footTrajectory(float phase, float s, float stepLen, float lift,
                           float footZ, float footHeightCap, FootPose &foot)
{
  // phase 0..0.5 = stance, 0.5..1 = swing.
  float lift2 = (lift < footHeightCap) ? lift : footHeightCap;

  if (phase < 0.5f) {
    float t = phase / 0.5f;
    foot.position.x = stepLen * (0.5f - t);
    foot.position.z = footZ;
  } else {
    float t = (phase - 0.5f) / 0.5f;
    foot.position.x = stepLen * (-0.5f + t);
    foot.position.z = footZ + lift2 * sinf(PI * t);
    if (foot.position.z > footHeightCap) {
      foot.position.z = footHeightCap;
    }
  }
}

void gaitFeet(FootPose &footL, FootPose &footR, float legSpacing)
{
  GaitConfig &c = _gaitState.config;

  float half = legSpacing / 2.0f;
  float bodySway = c.sway * sinf(2.0f * PI * _gaitState.phase);

  footL.position.y = half - bodySway;
  footR.position.y = -half - bodySway;

  float phaseL = _gaitState.phase;
  float phaseR = (_gaitState.phase + 0.5f);
  while (phaseR >= 1.0f) {
    phaseR -= 1.0f;
  }

  footL.roll = c.roll;
  footL.heading = 0.0f;
  footR.roll = c.roll;
  footR.heading = 0.0f;

  footTrajectory(phaseL, 0.0f, c.stepLen, c.lift, c.footZ, MAX_FOOT_HEIGHT, footL);
  footTrajectory(phaseR, 0.0f, c.stepLen, c.lift, c.footZ, MAX_FOOT_HEIGHT, footR);
}
