#include "gait.h"

void footTrajectory(
    float phase,
    float stepLength,
    float stepHeight,
    float z0,
    FootTarget &foot)
{
    if (phase < 0.5f) {

        // STANCE
        float s = phase / 0.5f;

        foot.x = stepLength * (0.5f - s);
        foot.z = z0;

    } else {

        // SWING
        float s = (phase - 0.5f) / 0.5f;

        foot.x = stepLength * (-0.5f + s);
        foot.z = z0 + stepHeight * sinf(PI * s);

    }

    foot.yaw = 0;
    foot.pitch = 0;
}

void generateFeet(
    float gaitPhase,
    FootTarget &left,
    FootTarget &right)
{
    float pL = gaitPhase;
    float pR = fmodf(gaitPhase + 0.5f, 1.0f);

    footTrajectory(
        pL,
        0.030f,     // 30 mm stride
        0.012f,     // 12 mm lift
        -0.140f,
        left
    );

    footTrajectory(
        pR,
        0.030f,
        0.012f,
        -0.140f,
        right
    );

    left.y  = +0.035f;
    right.y = -0.035f;
}