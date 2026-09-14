#ifndef _AMB82MINI_H
#define _AMB82MINI_H

#include <Arduino.h>
#include "kinematics.h"
#include "gait.h"
#include "imu.h"
#include "calibration.h"
#include "SMS_STS.h"

#define NUM_JOINTS 10
#define CONTROL_LOOP_US 10000  // 100 Hz

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

struct JointCalibration {
    uint8_t id;
    int center;       // servo center position (ticks)
    int offset;       // additional offset (ticks)
    int direction;    // +1 or -1 for sign
    float minAngle;   // min angle (rad)
    float maxAngle;   // max angle (rad)
};

class AMB82Mini {
public:
    AMB82Mini();
    void begin(HardwareSerial &serial);
    void loop();
    void setGaitPhase(float phase);
    float getGaitPhase() const;

    // IMU access
    IMU &imu() { return imu_; }

    // Calibration access
    const JointCalibration *calibration() const { return jointCalibration; }

private:
    HardwareSerial &serial_;
    IMU imu_;
    SMS_STS smsSts_;
    float gaitPhase_;
    uint32_t lastControlUs_;
};

#endif