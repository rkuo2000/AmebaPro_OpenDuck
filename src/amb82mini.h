#ifndef _AMB82MINI_H
#define _AMB82MINI_H

#include <Arduino.h>
#include "kinematics.h"
#include "gait.h"
#include "imu.h"
#include "calibration.h"
#include "SMS_STS.h"

#define CONTROL_LOOP_US 10000  // 100 Hz

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
    IMU imu_;
    SMS_STS smsSts_;
    float gaitPhase_;
    uint32_t lastControlUs_;
};

#endif