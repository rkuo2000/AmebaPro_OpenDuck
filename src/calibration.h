#ifndef _CALIBRATION_H
#define _CALIBRATION_H

#include <stdint.h>

#define NUM_JOINTS 10

struct JointCalibration {
    uint8_t id;
    int center;       // servo position at zero joint angle (ticks, 0-4095)
    int offset;       // additional offset (ticks)
    int direction;    // model radians to tick sign; standard v2 assembly: +1
    float minAngle;   // min angle (rad)
    float maxAngle;   // max angle (rad)
};

// Default calibration - IDs match STS3215 motor IDs from README
// Standard v2 model signs; centers/offsets must be calibrated for the robot.
extern const JointCalibration jointCalibration[NUM_JOINTS];

#endif
