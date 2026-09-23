#include "amb82mini.h"
#include "standing_pose.h"

AMB82Mini::AMB82Mini()
    : imu_(), smsSts_(0), gaitPhase_(0.0f), lastControlUs_(0),
      imuReady_(false), standingReady_(false), walking_(false),
      walkStartMs_(0), transitionMs_(2000), standingTicks_{}
{
}

bool AMB82Mini::begin(HardwareSerial &serial)
{
    beginServos(serial);
    return beginIMU();
}

void AMB82Mini::beginServos(HardwareSerial &serial)
{
    smsSts_.pSerial = &serial;
    imuReady_ = false;
    walking_ = false;
    standingReady_ = false;
    gaitPhase_ = 0.0f;
    lastControlUs_ = micros();
}

bool AMB82Mini::beginIMU()
{
    walking_ = false;
    imuReady_ = imu_.begin();
    lastControlUs_ = micros();
    return imuReady_;
}

void AMB82Mini::setGaitPhase(float phase)
{
    if (!isfinite(phase)) return;
    gaitPhase_ = fmodf(phase, 1.0f);
    if (gaitPhase_ < 0.0f) gaitPhase_ += 1.0f;
}

float AMB82Mini::getGaitPhase() const
{
    return gaitPhase_;
}

static bool angleToServo(float angle, const JointCalibration &joint, s16 &ticks)
{
    if (!isfinite(angle) || angle < joint.minAngle || angle > joint.maxAngle ||
        (joint.direction != 1 && joint.direction != -1)) return false;
    const float target = static_cast<float>(joint.center) + joint.offset +
        joint.direction * angle * (4096.0f / (2.0f * PI));
    if (!isfinite(target) || target < 0.0f || target > 4095.0f) return false;
    ticks = static_cast<s16>(target);
    return true;
}

bool AMB82Mini::writePose(const float *angles, uint16_t speed, uint8_t acceleration)
{
    u8 ids[NUM_JOINTS];
    s16 positions[NUM_JOINTS];
    u16 speeds[NUM_JOINTS];
    u8 accels[NUM_JOINTS];
    // Validate the entire packet before writing any joint. On failure the
    // servos retain their previous targets, rather than jumping to 2048.
    for (int i = 0; i < NUM_JOINTS; ++i) {
        if (jointCalibration[i].id != STANDING_IDS[i] ||
            !angleToServo(angles[i], jointCalibration[i], positions[i])) return false;
        ids[i] = jointCalibration[i].id;
        speeds[i] = speed;
        accels[i] = acceleration;
    }
    smsSts_.SyncWritePosEx(ids, NUM_JOINTS, positions, speeds, accels);
    return true;
}

bool AMB82Mini::stand()
{
    walking_ = false;
    standingReady_ = false;
    // Same targets, speed and acceleration as BasicStand. Check feedback
    // before motion, but never use it to overwrite calibration centers.
    for (int i = 0; i < NUM_JOINTS; ++i) {
        if (jointCalibration[i].id != STANDING_IDS[i] ||
            !angleToServo(STANDING_ANGLES[i], jointCalibration[i], standingTicks_[i])) {
            return false;
        }
        const int position = smsSts_.ReadPos(jointCalibration[i].id);
        if (position < 0 || position > 4095) return false;
    }
    standingReady_ = writePose(STANDING_ANGLES, 200, 10);
    return standingReady_;
}

bool AMB82Mini::standingReached(int toleranceTicks)
{
    if (!standingReady_ || walking_ || toleranceTicks < 0) return false;
    for (int i = 0; i < NUM_JOINTS; ++i) {
        const int position = smsSts_.ReadPos(jointCalibration[i].id);
        if (position < 0 || position > 4095 ||
            abs(position - standingTicks_[i]) > toleranceTicks) return false;
    }
    return true;
}

bool AMB82Mini::startWalking(uint32_t transitionMs)
{
    if (!imuReady_ || !standingReached()) return false;
    transitionMs_ = transitionMs;
    gaitPhase_ = 0.0f;
    walkStartMs_ = millis();
    lastControlUs_ = micros();
    walking_ = true;
    return true;
}

void AMB82Mini::loop()
{
    const uint32_t nowUs = micros();
    const uint32_t elapsedUs = nowUs - lastControlUs_;
    if (elapsedUs < CONTROL_LOOP_US) return;
    lastControlUs_ = nowUs;

    // Drain fresh IMU samples while standing too, without advancing the gait.
    IMUData imuData{};
    if (imuReady_) imu_.read(imuData);
    if (!walking_) return;

    // Cap a delayed update instead of skipping a large part of the step cycle.
    const float dt = clampf(elapsedUs * 1.0e-6f, 0.0f, 0.05f);
    setGaitPhase(gaitPhase_ + dt / GAIT_PERIOD_S);
    float rollCorr = 0.0f;
    float pitchCorr = 0.0f;
    if (imuData.valid && isfinite(imuData.roll) && isfinite(imuData.pitch)) {
        rollCorr = clampf(-imuData.roll * 1.5f, -0.10f, 0.10f);
        pitchCorr = clampf(-imuData.pitch * 1.5f, -0.10f, 0.10f);
    }

    FootTarget leftFoot{}, rightFoot{};
    generateFeet(gaitPhase_, leftFoot, rightFoot);
    // Preserve the half-cycle separation from generateFeet. Apply pitch
    // correction once in the common geometric frame before model mapping.
    leftFoot.pitch += pitchCorr;
    rightFoot.pitch += pitchCorr;
    LegAngles left{}, right{};
    if (!legIK(leftFoot, left) || !legIK(rightFoot, right)) {
        walking_ = false;
        return;
    }
    left.hipRoll += rollCorr;
    right.hipRoll += rollCorr;
    float angles[NUM_JOINTS] = {
        left.hipYaw, left.hipRoll, left.hipPitch, left.knee, left.ankle,
        right.hipYaw, right.hipRoll, -right.hipPitch, right.knee, right.ankle
    };
    // The right hip-pitch axis is reversed in the model. The servo calibration
    // still uses the standard +1 model-to-tick convention for both legs.
    float blend = transitionMs_ == 0 ? 1.0f :
        clampf(static_cast<float>(static_cast<uint32_t>(millis() - walkStartMs_)) /
                   transitionMs_, 0.0f, 1.0f);
    blend = blend * blend * (3.0f - 2.0f * blend);  // smooth start/end slope
    for (int i = 0; i < NUM_JOINTS; ++i) {
        angles[i] = STANDING_ANGLES[i] + blend * (angles[i] - STANDING_ANGLES[i]);
    }
    // Allow tracking margin; zero does not mean "hold position" on STS3215.
    if (!writePose(angles, 2000, 50)) walking_ = false;
}
