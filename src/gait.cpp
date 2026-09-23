#include "gait.h"

static float smoothStep(float t)
{
    return t * t * (3.0f - 2.0f * t);
}

void footTrajectory(
    float phase,
    float stepLength,
    float stepHeight,
    float z0,
    FootTarget &foot)
{
    phase -= floorf(phase);
    if (phase >= GAIT_SWING_START && phase < GAIT_SWING_END) {
        const float s = (phase - GAIT_SWING_START) /
            (GAIT_SWING_END - GAIT_SWING_START);
        const float swingFraction = GAIT_SWING_END - GAIT_SWING_START;
        const float tangent = -swingFraction / (1.0f - swingFraction);
        // Match the backward stance velocity at both ends of the swing.
        foot.x = stepLength * (smoothStep(s) - 0.5f +
            tangent * s * (1.0f - s) * (1.0f - 2.0f * s));
        // Zero vertical velocity at lift-off and touchdown.
        const float lift = sinf(PI * s);
        foot.z = z0 + stepHeight * lift * lift;
    } else {
        // Stance wraps through phase zero. Move the planted foot backward
        // relative to the body so the body travels forward.
        const float stancePhase = phase >= GAIT_SWING_END ?
            phase - GAIT_SWING_END : phase + 1.0f - GAIT_SWING_END;
        const float s = stancePhase /
            (1.0f - GAIT_SWING_END + GAIT_SWING_START);
        // Equal stance velocities keep the feet's separation fixed during
        // double support instead of asking two planted feet to twist the body.
        foot.x = stepLength * (0.5f - s);
        foot.z = z0;
    }
    foot.yaw = 0.0f;
    foot.pitch = 0.0f;
    foot.y = 0.0f;
}

void generateFeet(float gaitPhase, FootTarget &left, FootTarget &right)
{
    const float phase = gaitPhase - floorf(gaitPhase);
    footTrajectory(phase, 0.020f, 0.010f, GAIT_STANCE_Z, left);
    footTrajectory(fmodf(phase + 0.5f, 1.0f),
                   0.020f, 0.010f, GAIT_STANCE_Z, right);

    // Shift the body while both feet are down, then hold the shift through
    // swing. Positive foot Y moves the body toward the right support foot
    // before the left foot lifts. Both targets remain hip-local.
    const float halfPhase = fmodf(phase, 0.5f);
    const float transfer = smoothStep(clampf(halfPhase / GAIT_SWING_START, 0.0f, 1.0f));
    const float lateral = GAIT_BODY_SHIFT * (2.0f * transfer - 1.0f) *
        (phase < 0.5f ? 1.0f : -1.0f);
    left.y = lateral;
    right.y = lateral;
}
