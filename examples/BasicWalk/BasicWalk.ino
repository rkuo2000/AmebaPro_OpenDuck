/*
  BasicWalk - Example sketch for AMB82Mini biped robot
  
  Demonstrates:
  - Gait generator with phase control
  - 5-DOF inverse kinematics for both legs
  - BNO055 IMU integration for balance correction
  - STS3215 synchronized servo control
  
  Hardware:
  - AMB82-Mini board
  - 10x STS3215 serial servos (5 per leg)
  - BNO055 IMU over I2C
  - UART connection to servo bus at 1 Mbps
*/

#include <AMB82Mini.h>

// Hardware serial for servo bus (use Serial1 on AMB82 if available)
HardwareSerial servoSerial(1);

// Joint calibration - these values should be measured during actual calibration
// Joint IDs match the STS3215 motor IDs from the Open Duck Mini v2 schematic
AMB82Mini robot;

void setup() {
    // Initialize hardware serial for USB debugging
    Serial.begin(115200);
    
    // Initialize servo bus serial
    servoSerial.begin(1000000);  // 1 Mbps for STS3215
    servoSerial.setTimeout(100);
    
    // Initialize the robot controller
    robot.begin(servoSerial);
    
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