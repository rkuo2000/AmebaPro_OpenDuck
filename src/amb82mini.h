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
    // Initializes the IMU without commanding motion. False means IMU failure.
    bool begin(HardwareSerial &serial);
    // Attach the servo bus without initializing the IMU or commanding motion.
    void beginServos(HardwareSerial &serial);
    // Initialize/calibrate while holding the current pose; false prevents walking.
    bool beginIMU();
    // Command and hold the BasicStand pose without requiring an IMU.
    bool stand();
    bool standingReached(int toleranceTicks = 20);
    bool startWalking(uint32_t transitionMs = 2000);
    bool isWalking() const { return walking_; }
    // Diagnostic read: target is the last transmitted tick position; actual
    // is servo feedback. False means no target, invalid index or read failure.
    bool readJointFeedback(uint8_t jointIndex, int &target, int &actual);
    void loop();
    void setGaitPhase(float phase);
    float getGaitPhase() const;

    // IMU access
    IMU &imu() { return imu_; }

    // Calibration access
    const JointCalibration *calibration() const { return jointCalibration; }

private:
    bool writePose(const float *angles, uint16_t speed, uint8_t acceleration);
    IMU imu_;
    SMS_STS smsSts_;
    float gaitPhase_;
    uint32_t lastControlUs_;
    bool imuReady_;
    bool standingReady_;
    bool walking_;
    uint32_t walkStartMs_;
    uint32_t transitionMs_;
    int16_t standingTicks_[NUM_JOINTS];
    int16_t commandedTicks_[NUM_JOINTS];
    bool poseCommanded_;
};

#endif
