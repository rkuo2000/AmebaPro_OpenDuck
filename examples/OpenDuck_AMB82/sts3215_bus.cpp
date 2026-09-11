/*
 * sts3215_bus.cpp
 *
 * Thin wrapper over the SCServo SMS_STS API.
 *
 * All joint goals are broadcast through a single SyncWrite instruction so the
 * ten STS3215 motors update their positions synchronously. Basic safety
 * (torque, current and temperature) is layered on top.
 *
 * The SCServo SyncWritePosEx() signature is:
 *   void SyncWritePosEx(u8 ID[], u8 IDN, s16 Position[], u16 Speed[], u8 ACC[]);
 */
#include "sts3215_bus.h"
#include <math.h>
#include "robot_config.h"

#ifndef PI
#define PI 3.14159265358979323846
#endif

static float clampAngle(float angle, float minA, float maxA)
{
  if (angle < minA) return minA;
  if (angle > maxA) return maxA;
  return angle;
}

static const float RAD_TO_TICKS = 4096.0f / (2.0f * PI);

Sts3215Bus::Sts3215Bus()
{
}

bool Sts3215Bus::init()
{
  UART_SERVO.begin(UART_SERVO_BAUD, SERIAL_8N1);
  _st.pSerial = &UART_SERVO;

  // Give the servos a moment to power up and answer pings.
  delay(1000);

  for (int i = 0; i < NUM_JOINTS; i++) {
    if (_st.Ping(config[i].id) == -1) {
      UART_LOG.print("[bus] Ping failed: ");
      UART_LOG.println(config[i].id, DEC);
      return false;
    }
  }

  for (int i = 0; i < NUM_JOINTS; i++) {
    _st.EnableTorque(config[i].id, 1);
    // Move each servo to its calmed centre then disable torque briefly so the
    // user can handle the robot while power is on, before re-enabling torque.
    _st.writeByte(config[i].id, SMS_STS_TORQUE_ENABLE, 128);
    _st.EnableTorque(config[i].id, 1);
  }

  UART_LOG.println("[bus] All servos initialised.");
  return true;
}

int Sts3215Bus::angleToServo(float angle, float minA, float maxA, int center,
                             float direction)
{
  float clamped = clampAngle(angle, minA, maxA);
  int ticks = (int)floorf(center + direction + clamped * RAD_TO_TICKS);
  if (ticks < 0) {
    ticks = 0;
  }
  if (ticks > 4095) {
    ticks = 4095;
  }
  return ticks;
}

void Sts3215Bus::writeAll(const int goals[NUM_JOINTS])
{
  static uint8_t ids[NUM_JOINTS];
  static int     positions[NUM_JOINTS];
  static int     speeds[NUM_JOINTS];
  static uint8_t accs[NUM_JOINTS];

  for (int i = 0; i < NUM_JOINTS; i++) {
    ids[i]     = config[i].id;
    positions[i] = goals[i];
    speeds[i]  = 3400;
    accs[i]    = 50;
  }

  _st.SyncWritePosEx(ids, NUM_JOINTS, positions, speeds, accs);
}

void Sts3215Bus::torqueAll(bool enable)
{
  for (int i = 0; i < NUM_JOINTS; i++) {
    _st.EnableTorque(config[i].id, enable ? 1 : 0);
  }
}

int Sts3215Bus::readPosition(uint8_t id) const
{
  return _st.ReadPos(id);
}

uint16_t Sts3215Bus::goalTimeMs(uint16_t ms) const
{
  return ms + 100;
}

bool Sts3215Bus::checkCurrent()
{
  for (int i = 0; i < NUM_JOINTS; i++) {
    int c = _st.ReadCurrent(config[i].id);
    if (c != -1 && c > MAX_CURRENT) {
      UART_LOG.print("[fault] Over-current ID ");
      UART_LOG.print(config[i].id, DEC);
      UART_LOG.print(" = ");
      UART_LOG.println(c, DEC);
      return false;
    }
  }
  return true;
}

bool Sts3215Bus::checkTemperature()
{
  for (int i = 0; i < NUM_JOINTS; i++) {
    int t = _st.ReadTemper(config[i].id);
    if (t != -1 && t > HEAT_THRESHOLD) {
      UART_LOG.print("[fault] Over-temp ID ");
      UART_LOG.print(config[i].id, DEC);
      UART_LOG.print(" = ");
      UART_LOG.println(t, DEC);
      return false;
    }
  }
  return true;
}

void Sts3215Bus::updateSafety()
{
  if (!checkTemperature()) {
    emergencyStop();
  }
  if (!checkCurrent()) {
    emergencyStop();
  }
}

void Sts3215Bus::emergencyStop()
{
  UART_LOG.println("[fault] Emergency stop: torque disabled.");
  torqueAll(false);
}

void Sts3215Bus::printErrors(uint8_t id)
{
  UART_LOG.print("ID ");
  UART_LOG.print(id, DEC);
  UART_LOG.print(" err: ");
  UART_LOG.println((uint16_t)_st.getErr());
}
