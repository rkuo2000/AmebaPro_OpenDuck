/*
  BasicWalk - Example sketch for AMB82Mini biped robot
  
  Demonstrates:
  - Gait generator with phase control
  - 5-DOF inverse kinematics for both legs
  - MPU6050 IMU integration for balance correction
  - STS3215 synchronized servo control
  
  Hardware:
  - AMB82-Mini board
  - 10x STS3215 serial servos (5 per leg)
  - MPU6050 IMU over I2C
  - UART connection to servo bus at 1 Mbps
*/

#include "I2Cdev.h"
#include "Wire.h"
#include <MPU6050_IMU_libraries/MPU6050_6Axis_MotionApps612.h>
#include <SCServo.h>
#include <amb82mini.h>

// Joint calibration - these values should be measured during actual calibration
// Joint IDs match the STS3215 motor IDs from the Open Duck Mini v2 schematic
AMB82Mini robot;

void setup() {
    // Initialize hardware serial for USB debugging
    Serial.begin(115200);
    Serial2.begin(1000000, SERIAL_8N1);
    
    // Initialize the robot controller
    robot.begin(Serial2);
    
    // Set initial gait phase
    robot.setGaitPhase(0.0f);
    
    Serial.println("AMB82Mini biped robot initialized");
    Serial.println("Sending robots to standing pose...");
    
    // Wait for stabilization
    delay(1000);
}

void loop() {
    // The robot loop runs at 100 Hz internally
    // IMU balance correction and gait generation happen automatically
    robot.loop();
    
    // Optional: output gait phase for debugging
    static unsigned long lastPrint = 0;
    if (millis() - lastPrint > 1000) {
        lastPrint = millis();
        Serial.print("Gait phase: ");
        Serial.println(robot.getGaitPhase(), 3);
    }
}
