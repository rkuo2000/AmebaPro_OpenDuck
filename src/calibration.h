#ifndef _CALIBRATION_H
#define _CALIBRATION_H

#include <stdint.h>

#define NUM_JOINTS 10

struct JointCalibration {
    uint8_t id;
    int center;       // servo center position (ticks, 0-4095)
    int offset;       // additional offset (ticks)
    int direction;    // +1 or -1 for sign
    float minAngle;   // min angle (rad)
    float maxAngle;   // max angle (rad)
};

// Default calibration - IDs match STS3215 motor IDs from README
// These are placeholder values; user should calibrate for their specific robot
extern const JointCalibration jointCalibration[NUM_JOINTS];

#endif