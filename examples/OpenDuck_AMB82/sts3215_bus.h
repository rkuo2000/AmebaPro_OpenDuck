/*
 * sts3215_bus.h
 *
 * Thin wrapper over the SCServo SMS_STS API that drives all legs through a
 * single SyncWrite command so every joint goal updates together. Also
 * implements basic safety: torque enable, current and temperature cuts.
 */
#ifndef _STS3215_BUS_H
#define _STS3215_BUS_H

#include "Arduino.h"
#include <SCServo.h>
#include "robot_config.h"

class Sts3215Bus {
public:
  Sts3215Bus();

  // Initialise the serial bus. Returns true when all servos answered a ping.
  bool init();

  // Enable or disable torque on every servo.
  void torqueAll(bool enable);

  // Broadcast goals in ticks. All joints update together (SyncWrite).
  void writeAll(const int goals[NUM_JOINTS]);

  // Read back a servo's current position in ticks (or -1 on error).
  int readPosition(uint8_t id) const;

  // Return the goal-time low byte for a desired move duration in milliseconds.
  uint16_t goalTimeMs(uint16_t ms) const;

  // Safety reads. Returns true when safe to continue.
  void updateSafety();
  bool checkTemperature();
  bool checkCurrent();

  // Emergency: disable torque everywhere.
  void emergencyStop();

  int angleToServo(float angle, float minA, float maxA, int center,
                   float direction);
  int  errCode() const { return _st.getErr(); }
  void printErrors(uint8_t id);

private:
  SMS_STS _st;
};

#endif
