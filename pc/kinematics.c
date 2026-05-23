#include <stdio.h>
#include <math.h>
#include "kinematics.h"
#include "config.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

static float clampf(float v, float lo, float hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

void kinematics_pixel_to_mm(uint32_t px_x, uint32_t px_y,
                             float *out_x_mm, float *out_y_mm)
{
    *out_x_mm = ((float)px_x / (float)IMG_WIDTH)  * WORKSPACE_X_MM;
    *out_y_mm = ((float)px_y / (float)IMG_HEIGHT) * WORKSPACE_Y_MM;
}

ArmAngles kinematics_solve_ik(float x_mm, float y_mm)
{
    ArmAngles home = {
        .base_deg     = HOME_BASE_DEG,
        .shoulder_deg = HOME_SHOULDER_DEG,
        .elbow_deg    = HOME_ELBOW_DEG
    };

    const float L1 = LINK1_MM;
    const float L2 = LINK2_MM;

    float base_rad = atan2f(y_mm, x_mm);
    float base_deg = base_rad * (180.0f / (float)M_PI);
    base_deg = clampf(base_deg, SERVO_BASE_MIN, SERVO_BASE_MAX);

    float reach = sqrtf(x_mm * x_mm + y_mm * y_mm);

    if (reach > L1 + L2) {
        printf("[IK] WARNING: target (%.1f, %.1f) mm is OUT OF REACH "
               "(reach=%.1f, max=%.1f)\n",
               x_mm, y_mm, reach, L1 + L2);
        return home;
    }
    if (reach < fabsf(L1 - L2)) {
        printf("[IK] WARNING: target (%.1f, %.1f) mm is TOO CLOSE "
               "(reach=%.1f, min=%.1f)\n",
               x_mm, y_mm, reach, fabsf(L1 - L2));
        return home;
    }

    float cos_elbow = (reach * reach - L1 * L1 - L2 * L2)
                      / (2.0f * L1 * L2);
    cos_elbow = clampf(cos_elbow, -1.0f, 1.0f);

    float elbow_rad = acosf(cos_elbow);
    float elbow_deg = elbow_rad * (180.0f / (float)M_PI);
    elbow_deg = clampf(elbow_deg, SERVO_ELBOW_MIN, SERVO_ELBOW_MAX);

    float beta_rad    = atan2f(L2 * sinf(elbow_rad),
                                L1 + L2 * cosf(elbow_rad));
    float shoulder_rad = atan2f(y_mm, x_mm) - beta_rad;
    float shoulder_deg = shoulder_rad * (180.0f / (float)M_PI);
    shoulder_deg = clampf(shoulder_deg, SERVO_SHOULDER_MIN, SERVO_SHOULDER_MAX);

    ArmAngles angles = {
        .base_deg     = base_deg,
        .shoulder_deg = shoulder_deg,
        .elbow_deg    = elbow_deg
    };

    printf("[IK] Solved  base: %6.1f°  shoulder: %6.1f°  elbow: %6.1f°\n",
           base_deg, shoulder_deg, elbow_deg);

    return angles;
}

ArmAngles kinematics_bin_angles(uint32_t bin_px_x, uint32_t bin_px_y)
{
    float x_mm, y_mm;
    kinematics_pixel_to_mm(bin_px_x, bin_px_y, &x_mm, &y_mm);
    return kinematics_solve_ik(x_mm, y_mm);
}
