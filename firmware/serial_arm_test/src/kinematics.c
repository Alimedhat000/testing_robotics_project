#include "kinematics.h"
#include "config.h"
#include <math.h>
#include <stdio.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif
#define RAD2DEG (180.0f / (float)M_PI)

static float clampf(float v, float lo, float hi) {
  if (v < lo)
    return lo;
  if (v > hi)
    return hi;
  return v;
}

void kinematics_forward(float t1, float t2, float t3, float *x, float *y,
                        float *z) {
  const float L1 = LINK1_MM;
  const float L2 = LINK2_MM;

  float c1 = cosf(t1), s1 = sinf(t1);
  float c2 = cosf(t2), s2 = sinf(t2);
  float c23 = cosf(t2 + t3);
  float s23 = sinf(t2 + t3);

  float r_plane = L1 * c2 + L2 * c23;

  *x = c1 * r_plane;
  *y = s1 * r_plane;
  *z = -(L1 * s2 + L2 * s23);
}

ArmAngles kinematics_solve_ik(float x_t, float y_t, float z_t) {
  const float L1 = LINK1_MM;
  const float L2 = LINK2_MM;

  ArmAngles home = {HOME_BASE_DEG, HOME_SHOULDER_DEG, HOME_ELBOW_DEG};

  float zs = z_t - SHOULDER_Z_OFFSET_MM;
  float r_xy = sqrtf(x_t * x_t + y_t * y_t);
  float reach = sqrtf(r_xy * r_xy + zs * zs);

  if (reach > L1 + L2) {
    printf("[IK] UNREACHABLE reach=%.1f > max=%.1f\n", reach, L1 + L2);
    return home;
  }
  if (reach < fabsf(L1 - L2)) {
    printf("[IK] UNREACHABLE reach=%.1f < min=%.1f\n", reach, fabsf(L1 - L2));
    return home;
  }

  float t1 = atan2f(y_t, x_t);

  float cos_t3 = (reach * reach - L1 * L1 - L2 * L2) / (2.0f * L1 * L2);
  cos_t3 = clampf(cos_t3, -1.0f, 1.0f);

  float best_t2 = 0, best_t3 = 0;
  bool found = false;

  for (int sign = 1; sign >= -1; sign -= 2) {
    float t3 = sign * acosf(cos_t3);
    float t2 = atan2f(zs, r_xy) - atan2f(L2 * sinf(t3), L1 + L2 * cosf(t3));

    float sd = t2 * RAD2DEG;
    float ed = t3 * RAD2DEG;

    if (sd >= SERVO_SHOULDER_MIN && sd <= SERVO_SHOULDER_MAX &&
        ed >= SERVO_ELBOW_MIN && ed <= SERVO_ELBOW_MAX) {
      best_t2 = t2;
      best_t3 = t3;
      found = true;
      break;
    }
    if (!found) {
      best_t2 = t2;
      best_t3 = t3;
    }
  }

  float base_deg = clampf(t1 * RAD2DEG, SERVO_BASE_MIN, SERVO_BASE_MAX);

  float shoulder_deg =
      clampf(best_t2 * RAD2DEG, SERVO_SHOULDER_MIN, SERVO_SHOULDER_MAX);

  float elbow_deg = clampf(best_t3 * RAD2DEG, SERVO_ELBOW_MIN, SERVO_ELBOW_MAX);

  printf("[IK] raw: base=%.1f  shoulder=%.1f  elbow=%.1f"
         "  (r_xy=%.1f zs=%.1f reach=%.1f cos_t3=%.3f)\n",
         t1 * RAD2DEG, best_t2 * RAD2DEG, best_t3 * RAD2DEG, r_xy, zs, reach,
         cos_t3);
  printf("[IK] out: base=%.1f  shoulder=%.1f  elbow=%.1f\n", base_deg,
         shoulder_deg, elbow_deg);

  ArmAngles result = {base_deg, shoulder_deg, elbow_deg};
  return result;
}

void kinematics_pixel_to_mm(uint32_t px_x, uint32_t px_y, float *out_x_mm,
                            float *out_y_mm) {
  *out_x_mm = ((float)px_x / (float)IMG_WIDTH) * WORKSPACE_X_MM;
  *out_y_mm = ((float)px_y / (float)IMG_HEIGHT) * WORKSPACE_Y_MM;
}

ArmAngles kinematics_bin_angles(uint32_t bin_px_x, uint32_t bin_px_y) {
  float x_mm, y_mm;
  kinematics_pixel_to_mm(bin_px_x, bin_px_y, &x_mm, &y_mm);
  return kinematics_solve_ik(x_mm, y_mm, 0.0f);
}
