/*
  BasicStand - fixed standing pose for Open Duck Mini v2 / AMB82-Mini.

  Requires SCServo and AmebaPro_OpenDuck (using this repository's src files).
  Servo bus: Serial2, 1 Mbps, GPIO 18 RX / GPIO 19 TX through the bus adapter.
  Power the servos from their external supply and share ground with the board.

  Before use, calibrate src/calibration.cpp for your assembled robot (the
  supplied centers/offsets are nominal; signs follow standard v2). Configure the
  STS3215 servos in position mode with torque enabled. Support the robot during
  startup: it moves automatically and holds the standing pose indefinitely
  while powered with torque enabled. This is an open-loop pose, without IMU
  balance control. Only the ten leg servos are commanded.
*/
#include <amb82mini.h>

AMB82Mini robot;

void setup() {
    Serial.begin(115200);
    Serial2.begin(1000000, SERIAL_8N1);
    robot.beginServos(Serial2);

    Serial.println("Moving to standing pose; holding indefinitely.");
    if (!robot.stand()) {
        Serial.println("Stopped: check leg servo communication and calibration.");
        return;
    }
    Serial.println("Standing targets sent; servos will hold with torque enabled.");
}

void loop() {
    // Hold forever: STS3215 position control retains the last target.
    // No walking or IMU initialization is started here.
    delay(100);
}
