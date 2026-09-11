/*
 * OpenDuck_AMB82.ino
 *
 * Minimal OpenDuck Mini v2 bipedal kinematics + STS3215 control for the AMB82
 * Mini, derived from the README examples and SCServo sync-write samples.
 *
 * Gait -> feet pose -> 5-DOF leg IK -> joint limits -> radians -> STS3215
 * ticks -> single SyncWrite. Runs the walking loop at ~100 Hz.
 */
#include <SCServo.h>
#include "kinematics.h"
#include "gait.h"
#include "sts3215_bus.h"
#include "robot_config.h"

Sts3215Bus bus;
GaitConfig gait;

void setup()
{
  UART_LOG.begin(UART_LOG_BAUD);
  delay(1000);

  UART_LOG.println("[init] OpenDuck Mini v2 @ AMB82");

  if (!bus.init()) {
    UART_LOG.println("[fatal] Servos initialisation failed. Holding.");
    while (1)
      ;
  }

  gaitConfig(gait);
  bus.torqueAll(true);
}

void loop()
{
  static uint32_t lastTask = millis();
  uint32_t now = millis();

  if (now - lastTask < TASK_MS) {
    return;
  }
  lastTask += TASK_MS;

  gaitAdvance(1.0 / (2.0f * gait.phaseHz));

  FootPose footL, footR;
  gaitFeet(footL, footR, _gaitState.config.sway);

  LegAngles qL, qR;
  bool okL = legIK(footL, qL.hipYaw, qL.hipRoll, qL.hipPitch, qL.knee, qL.ankle);
  bool okR = legIK(footR, qR.hipYaw, qR.hipRoll, qR.hipPitch, qR.knee, qR.ankle);

  if (!okL || !okR) {
    bus.emergencyStop();
    return;
  }

  int goal[NUM_JOINTS];
  goal[L_HIP_YAW]   = bus.angleToServo(qL.hipYaw,   config[L_HIP_YAW].minAngle, config[L_HIP_YAW].maxAngle, config[L_HIP_YAW].center,   config[L_HIP_YAW].sign);
  goal[L_HIP_ROLL]  = bus.angleToServo(qL.hipRoll,  config[L_HIP_ROLL].minAngle, config[L_HIP_ROLL].maxAngle, config[L_HIP_ROLL].center,   config[L_HIP_ROLL].sign);
  goal[L_HIP_PITCH] = bus.angleToServo(qL.hipPitch, config[L_HIP_PITCH].minAngle, config[L_HIP_PITCH].maxAngle, config[L_HIP_PITCH].center, config[L_HIP_PITCH].sign);
  goal[L_KNEE]      = bus.angleToServo(qL.knee,     config[L_KNEE].minAngle, config[L_KNEE].maxAngle, config[L_KNEE].center,       config[L_KNEE].sign);
  goal[L_ANKLE]     = bus.angleToServo(qL.ankle,    config[L_ANKLE].minAngle, config[L_ANKLE].maxAngle, config[L_ANKLE].center,      config[L_ANKLE].sign);
  goal[R_HIP_YAW]   = bus.angleToServo(qR.hipYaw,   config[R_HIP_YAW].minAngle, config[R_HIP_YAW].maxAngle, config[R_HIP_YAW].center,   config[R_HIP_YAW].sign);
  goal[R_HIP_ROLL]  = bus.angleToServo(qR.hipRoll,  config[R_HIP_ROLL].minAngle, config[R_HIP_ROLL].maxAngle, config[R_HIP_ROLL].center,   config[R_HIP_ROLL].sign);
  goal[R_HIP_PITCH] = bus.angleToServo(qR.hipPitch, config[R_HIP_PITCH].minAngle, config[R_HIP_PITCH].maxAngle, config[R_HIP_PITCH].center, config[R_HIP_PITCH].sign);
  goal[R_KNEE]      = bus.angleToServo(qR.knee,     config[R_KNEE].minAngle, config[R_KNEE].maxAngle, config[R_KNEE].center,       config[R_KNEE].sign);
  goal[R_ANKLE]     = bus.angleToServo(qR.ankle,    config[R_ANKLE].minAngle, config[R_ANKLE].maxAngle, config[R_ANKLE].center,      config[R_ANKLE].sign);

  bus.updateSafety();
  bus.writeAll(goal);
}
