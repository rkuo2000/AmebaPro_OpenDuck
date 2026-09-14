#include "kinematics.h"

bool legIK(const FootTarget &foot, LegAngles &q)
{
    float x = foot.x;
    float y = foot.y;
    float z = foot.z;

    // 1. hip yaw
    q.hipYaw = foot.yaw;

    // Transform target into frame after yaw.
    float cy = cosf(-q.hipYaw);
    float sy = sinf(-q.hipYaw);

    float xr = cy * x - sy * y;
    float yr = sy * x + cy * y;

    // 2. hip roll
    q.hipRoll = atan2f(yr, -z);

    // sagittal-plane vertical distance
    float zs = sqrtf(yr * yr + z * z);

    // 3. planar 2-link IK
    float d = sqrtf(xr * xr + zs * zs);

    const float minD = fabsf(L1 - L2) + 0.001f;
    const float maxD = L1 + L2 - 0.001f;

    if (d < minD || d > maxD)
        return false;

    float ck =
        (L1 * L1 + L2 * L2 - d * d) /
        (2.0f * L1 * L2);

    ck = clampf(ck, -1.0f, 1.0f);

    float kneeInternal = acosf(ck);

    q.knee = M_PI - kneeInternal;

    float ch =
        (L1 * L1 + d * d - L2 * L2) /
        (2.0f * L1 * d);

    ch = clampf(ch, -1.0f, 1.0f);

    float beta = acosf(ch);
    float alpha = atan2f(xr, zs);

    q.hipPitch = alpha - beta;

    // Keep desired foot pitch
    q.ankle =
        foot.pitch -
        q.hipPitch -
        q.knee;

    return true;
}