#ifndef OPEN_DUCK_STANDING_POSE_H
#define OPEN_DUCK_STANDING_POSE_H

#include "calibration.h"

// Verified InitialStand pose, in model radians. Source: init_pos in
// experiments/v2/onnx_AWD_mujoco_motor_control.py.
// Each leg: hip yaw, hip roll, hip pitch, knee, ankle.
constexpr uint8_t STANDING_IDS[] = {20, 21, 22, 23, 24, 10, 11, 12, 13, 14};
constexpr float STANDING_ANGLES[] = {
     0.002f,  0.053f, -0.630f, 1.368f, -0.784f,
    -0.003f, -0.065f,  0.635f, 1.379f, -0.796f
};
static_assert(sizeof(STANDING_IDS) / sizeof(STANDING_IDS[0]) == NUM_JOINTS,
              "Standing pose must include every calibrated leg servo");
static_assert(sizeof(STANDING_ANGLES) / sizeof(STANDING_ANGLES[0]) == NUM_JOINTS,
              "Standing pose must include every calibrated leg angle");

#endif
