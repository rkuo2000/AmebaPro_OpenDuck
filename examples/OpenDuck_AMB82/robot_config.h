/*
 * robot_config.h
 *
 * OpenDuck Mini v2 joint mapping, geometry and calibration for the AMB82.
 *
 * The ID/direction/center entries below are EXAMPLES to wire up the robot.
 * The direction and sign choices must be confirmed by physically moving each
 * joint through its range; the limits follow the OpenDuck Mini v2 MuJoCo model.
 */
#ifndef _ROBOT_CONFIG_H
#define _ROBOT_CONFIG_H

#include "Arduino.h"

// ----------------------------------------------------------------------------------
// Leg body lengths (metres).
// OpenDuck Mini v2 confirms successive knee/ankle offsets of ~0.07865 m.
// ----------------------------------------------------------------------------------
#define BODY_L1 0.07865f
#define BODY_L2 0.07865f

// Hip width (m). Distance the feet sit apart from the robot's yaw axis.
#define HIP_W 0.035f

// ----------------------------------------------------------------------------------
// Serial bus.
// SCServo uses hardware Serial2 (GPIO18 = S_RXD, GPIO19 = S_TXD) at 1 Mbps.
// ----------------------------------------------------------------------------------
#define UART_SERVO Serial2
#define UART_SERVO_BAUD 1000000
#define UART_LOG Serial
#define UART_LOG_BAUD   115200

// ----------------------------------------------------------------------------------
// Gait defaults.
// ----------------------------------------------------------------------------------
#define WALK_PHASE_HZ 2.0f
#define GAIT_DEG      30.0f
#define FOOT_HEIGHT   0.012f
#define FOOT_STEP     0.010f

// Control loop cycle time. Must divide by 1000 for seconds.
#define TASK_MS 10

// ----------------------------------------------------------------------------------
// Joint ordering.
// ----------------------------------------------------------------------------------
enum Joint {
  L_HIP_YAW,
  L_HIP_ROLL,
  L_HIP_PITCH,
  L_KNEE,
  L_ANKLE,

  R_HIP_YAW,
  R_HIP_ROLL,
  R_HIP_PITCH,
  R_KNEE,
  R_ANKLE,

  NUM_JOINTS
};

struct JointCalibration {
  uint8_t id;
  float   sign;
  int     center;
  float   minAngle;
  float   maxAngle;
};

// ID/direction/center are EXAMPLES. Confirm each entry by physically driving the
// joint. Angle limits follow the OpenDuck Mini v2 model.
// {  ID,     sign,  center,  minAngle,  maxAngle           }
static const JointCalibration config[NUM_JOINTS] = {
  { 20, +1.0f, 2048, -45.0f,  45.0f }, // L hip yaw
  { 21, +1.0f, 2048, -25.0f,  25.0f }, // L hip roll
  { 22, -1.0f, 2048, -70.0f,  70.0f }, // L hip pitch
  { 23, +1.0f, 2048, -100.0f, 0.0f   }, // L knee
  { 24, +1.0f, 2048, -100.0f, 100.0f }, // L ankle

  { 10, +1.0f, 2048, -45.0f,  45.0f }, // R hip yaw
  { 11, -1.0f, 2048, -25.0f,  25.0f }, // R hip roll
  { 12, +1.0f, 2048, -70.0f,  70.0f }, // R hip pitch
  { 13, +1.0f, 2048, -100.0f, 0.0f   }, // R knee
  { 14, -1.0f, 2048, -100.0f, 100.0f }, // R ankle
};

// ----------------------------------------------------------------------------------
// Servo safety.
// ----------------------------------------------------------------------------------
#define MAX_FOOT_HEIGHT 0.06f        // metres
#define MAX_CURRENT     500          // mA, per servo
#define CURRENT_SAMPLES 8            // samples taken before faulting
#define HEAT_THRESHOLD  65           // deg C

#endif
