#include "imu.h"
#include <Wire.h>

IMU::IMU()
    : mpu_(), dmpReady_(false), packetSize_(0), fifoBuffer_{}, lastData_{}
{
}

bool IMU::begin(uint8_t addr)
{
    if (dmpReady_) {
        mpu_.setDMPEnabled(false);
    }
    dmpReady_ = false;
    packetSize_ = 0;
    lastData_ = IMUData{};
    if (addr != 0x68 && addr != 0x69) {
        return false;
    }

    Wire.begin();
    Wire.setClock(400000);
    mpu_.setAddress(addr);
    mpu_.initialize();
    if (!mpu_.testConnection()) {
        return false;
    }
    if (mpu_.dmpInitialize(MPU6050_DMP_FIFO_RATE_DIVISOR, addr) != 0) {
        mpu_.setDMPEnabled(false);
        return false;
    }

    // Seed offsets from IMU_MPU6050.ino, then calibrate this sensor.
    mpu_.setXGyroOffset(51);
    mpu_.setYGyroOffset(8);
    mpu_.setZGyroOffset(21);
    mpu_.setXAccelOffset(1150);
    mpu_.setYAccelOffset(-50);
    mpu_.setZAccelOffset(1060);
    mpu_.CalibrateAccel(6);
    mpu_.CalibrateGyro(6);

    packetSize_ = mpu_.dmpGetFIFOPacketSize();
    if (packetSize_ == 0 || packetSize_ > sizeof(fifoBuffer_)) {
        mpu_.setDMPEnabled(false);
        return false;
    }
    mpu_.resetFIFO();
    mpu_.setDMPEnabled(true);
    mpu_.getIntStatus();
    dmpReady_ = true;
    return true;
}

void IMU::read(IMUData &data)
{
    data = lastData_;
    data.valid = false;
    if (!dmpReady_) {
        return;
    }
    // Skip incomplete packets. The latest-packet helper handles FIFO overflow
    // and discards older packets; it may briefly wait during overflow recovery.
    if (mpu_.getFIFOCount() < packetSize_ ||
        !mpu_.dmpGetCurrentFIFOPacket(fifoBuffer_)) {
        return;
    }

    Quaternion q;
    VectorFloat gravity;
    VectorInt16 gyro;
    float ypr[3];
    mpu_.dmpGetQuaternion(&q, fifoBuffer_);
    mpu_.dmpGetGravity(&gravity, &q);
    mpu_.dmpGetYawPitchRoll(ypr, &q, &gravity);
    mpu_.dmpGetGyro(&gyro, fifoBuffer_);

    data.roll = ypr[2];
    data.pitch = ypr[1];
    data.yaw = ypr[0];
    // MotionApps 6.12 uses +/-2000 deg/s (16.4 LSB/deg/s).
    // These are body-axis velocities, not Euler angle derivatives.
    constexpr float GYRO_TO_RAD_S = (PI / 180.0f) / 16.4f;
    data.rollRate = gyro.x * GYRO_TO_RAD_S;
    data.pitchRate = gyro.y * GYRO_TO_RAD_S;
    data.yawRate = gyro.z * GYRO_TO_RAD_S;
    data.valid = true;
    lastData_ = data;
}

float IMU::getRoll() const { return lastData_.roll; }
float IMU::getPitch() const { return lastData_.pitch; }
float IMU::getYaw() const { return lastData_.yaw; }
