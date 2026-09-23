#include "calibration.h"

// Model-coordinate defaults for the standard Open Duck Mini v2 assembly.
// Upstream rustypot_position_hwi.py sends model angles plus measured offsets
// without per-joint sign inversions. All directions therefore default to +1.
// Reference: https://github.com/apirrone/Open_Duck_Mini_Runtime/blob/v2/mini_bdx_runtime/mini_bdx_runtime/rustypot_position_hwi.py
// Centers/offsets still need verification against the assembled robot.
const JointCalibration jointCalibration[NUM_JOINTS] = {
    // Left leg joints (IDs from README Table 21)
    {20, 2048, 0, +1, -0.524f, +0.524f}, // L hip yaw   ±30°
    {21, 2048, 0, +1, -0.436f, +0.436f}, // L hip roll  ±25°
    {22, 2048, 0, +1, -1.222f, +0.524f}, // L hip pitch
    {23, 2048, 0, +1, -1.571f, +1.571f}, // L knee      ±90°
    {24, 2048, 0, +1, -1.571f, +1.571f}, // L ankle     ±90°

    // Right leg joints (IDs from README Table 21)
    {10, 2048, 0, +1, -0.524f, +0.524f}, // R hip yaw   ±30°
    {11, 2048, 0, +1, -0.436f, +0.436f}, // R hip roll  ±25°
    {12, 2048, 0, +1, -0.524f, +1.222f}, // R hip pitch (model: -30° to +70°)
    {13, 2048, 0, +1, -1.571f, +1.571f}, // R knee      ±90°
    {14, 2048, 0, +1, -1.571f, +1.571f}  // R ankle     ±90°
};
