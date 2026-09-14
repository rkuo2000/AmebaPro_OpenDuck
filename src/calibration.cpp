#include "calibration.h"

const JointCalibration jointCalibration[NUM_JOINTS] = {
    // Left leg joints (IDs from README Table 21)
    {20, 2048, 0, +1, -0.524f, +0.524f}, // L hip yaw   ±30°
    {21, 2048, 0, +1, -0.436f, +0.436f}, // L hip roll  ±25°
    {22, 2048, 0, -1, -1.222f, +0.524f}, // L hip pitch
    {23, 2048, 0, +1, -1.571f, +1.571f}, // L knee      ±90°
    {24, 2048, 0, -1, -1.571f, +1.571f}, // L ankle     ±90°

    // Right leg joints (IDs from README Table 21)
    {10, 2048, 0, -1, -0.524f, +0.524f}, // R hip yaw   ±30° (inverted direction)
    {11, 2048, 0, +1, -0.436f, +0.436f}, // R hip roll  ±25°
    {12, 2048, 0, -1, -1.222f, +0.524f}, // R hip pitch
    {13, 2048, 0, +1, -1.571f, +1.571f}, // R knee      ±90°
    {14, 2048, 0, -1, -1.571f, +1.571f}  // R ankle     ±90°
};