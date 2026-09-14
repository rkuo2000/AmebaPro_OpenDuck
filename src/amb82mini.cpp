#include "amb82mini.h"
#include "SMS_STS.h"
#include "kinematics.h"
#include "gait.h"
#include "imu.h"
#include "calibration.h"

AMB82Mini::AMB82Mini()
    : imu_()
    , smsSts_(0)  // STS servos use little-endian byte order
    , gaitPhase_(0.0f)
    , lastControlUs_(0)
{
}

void AMB82Mini::begin(HardwareSerial &serial)
{
    imu_.begin();
    smsSts_.pSerial = &serial;
    gaitPhase_ = 0.0f;
    lastControlUs_ = 0;
}

void AMB82Mini::setGaitPhase(float phase)
{
    gaitPhase_ = fmodf(phase, 1.0f);
}

float AMB82Mini::getGaitPhase() const
{
    return gaitPhase_;
}

static int angleToServo(float angle, const JointCalibration &j)
{
    angle = clampf(angle, j.minAngle, j.maxAngle);

    constexpr float RAD_TO_TICKS = 4096.0f / (2.0f * PI);

    int ticks =
        j.center +
        j.offset +
        (int)(j.direction * angle * RAD_TO_TICKS);

    if (ticks < 0)
        ticks = 0;

    if (ticks > 4095)
        ticks = 4095;

    return ticks;
}

void AMB82Mini::loop()
{
    uint32_t nowUs = micros();

    if (nowUs - lastControlUs_ >= CONTROL_LOOP_US) {
        lastControlUs_ = nowUs;

        // 1. Read IMU for balance correction
        IMUData imuData;
        imu_.read(imuData);

        // Calculate balance corrections from IMU
        float rollError = 0.0f - imuData.roll;    // desired roll = 0
        float pitchError = 0.0f - imuData.pitch;  // desired pitch = 0 (level)

        // Simple P correction - gains tuned for biped walking
        float rollCorr = rollError * 1.5f;
        float pitchCorr = pitchError * 1.5f;

        // 2. Update gait phase
        gaitPhase_ += 0.010f;  // 10ms step = 100 Hz
        if (gaitPhase_ >= 1.0f) {
            gaitPhase_ -= 1.0f;
        }

        // 3. Generate desired feet positions
        FootTarget leftFoot;
        FootTarget rightFoot;

        generateFeet(gaitPhase_, leftFoot, rightFoot);

        // 4. Apply IMU balance corrections to foot pitches
        float correctedLeftPitch = leftFoot.pitch + pitchCorr;
        float correctedRightPitch = rightFoot.pitch - pitchCorr;  // opposite sign for right leg

        // Re-generate foot targets with corrected pitch
        footTrajectory(gaitPhase_, 0.030f, 0.012f, -0.140f, leftFoot);
        leftFoot.pitch = correctedLeftPitch;

        footTrajectory(gaitPhase_, 0.030f, 0.012f, -0.140f, rightFoot);
        rightFoot.pitch = correctedRightPitch;

        // 5. IK
        LegAngles qL;
        LegAngles qR;

        bool okL = legIK(leftFoot, qL);
        bool okR = legIK(rightFoot, qR);

        if (!okL || !okR) {
            // emergency hold - set all joints to center position
            for (int i = 0; i < NUM_JOINTS; i++) {
                smsSts_.WritePosEx(jointCalibration[i].id, 2048, 0, 0);
            }
            return;
        }

        // Apply IMU roll correction to hip roll joints
        qL.hipRoll += rollCorr;
        qR.hipRoll += rollCorr;

        // Apply pitch correction: add to hip pitch, subtract from ankle
        qL.hipPitch += pitchCorr;
        qR.hipPitch += pitchCorr;
        qL.ankle -= pitchCorr;
        qR.ankle -= pitchCorr;

        // 6. Map to joint angles per leg
        float q[10];

        q[0] = qL.hipYaw;   // L hip yaw
        q[1] = qL.hipRoll;  // L hip roll
        q[2] = qL.hipPitch; // L hip pitch
        q[3] = qL.knee;     // L knee
        q[4] = qL.ankle;    // L ankle

        q[5] = qR.hipYaw;   // R hip yaw
        q[6] = qR.hipRoll;  // R hip roll
        q[7] = qR.hipPitch; // R hip pitch
        q[8] = qR.knee;     // R knee
        q[9] = qR.ankle;    // R ankle

        // 7. Map radians to servo ticks with calibration
        int position[10];

        for (int i = 0; i < NUM_JOINTS; i++) {
            position[i] = angleToServo(q[i], jointCalibration[i]);
        }

        // 8. ONE SyncWrite packet using SMS_STS SyncWritePosEx
        // Extract servo IDs from calibration
        u8 ids[NUM_JOINTS];
        for (int i = 0; i < NUM_JOINTS; i++) {
            ids[i] = jointCalibration[i].id;
        }

        s16 positionsArr[NUM_JOINTS];
        u16 speeds[NUM_JOINTS] = {0};  // speed 0 = hold position
        u8 accels[NUM_JOINTS] = {0};  // accel 0 = use default

        for (int i = 0; i < NUM_JOINTS; i++) {
            positionsArr[i] = position[i];
        }

        // Send synchronized position write to all 10 servos
        smsSts_.SyncWritePosEx(
            ids,                 // ID array
            NUM_JOINTS,          // number of servos
            positionsArr,        // position array (ticks)
            speeds,              // speed array
            accels               // acceleration array
        );
    }
}