#ifndef _IMU_H
#define _IMU_H

#include <Arduino.h>
#include <MPU6050_IMU_libraries/MPU6050_6Axis_MotionApps612.h>

struct IMUData {
    float roll;     // roll angle (rad)
    float pitch;    // pitch angle (rad)
    float yaw;      // yaw angle (rad)
    float rollRate; // sensor X angular velocity (rad/s)
    float pitchRate;// sensor Y angular velocity (rad/s)
    float yawRate;  // sensor Z angular velocity (rad/s)
    bool  valid;    // true only when read() obtains a fresh DMP packet
};

class IMU {
public:
    IMU();
    // Keep the sensor still and level during startup calibration.
    // AD0 low: 0x68; AD0 high: 0x69. Returns false on initialization failure.
    bool begin(uint8_t addr = 0x68);
    // Without a fresh packet, returns the last sample with valid = false.
    void read(IMUData &data);
    // Latest angles in radians; zero until the first successful read().
    float getRoll() const;
    float getPitch() const;
    float getYaw() const;

private:
    // The bundled MPU6050 library has no public address setter.
    class Device : public MPU6050_6Axis_MotionApps612 {
    public:
        void setAddress(uint8_t addr) { devAddr = addr; }
    };

    Device mpu_;
    bool dmpReady_;
    uint16_t packetSize_;
    uint8_t fifoBuffer_[128];
    IMUData lastData_;
};

#endif
