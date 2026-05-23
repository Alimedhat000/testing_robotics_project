#ifndef KINEMATICS_H
#define KINEMATICS_H

#include <stdint.h>
#include "config.h"

#ifdef __cplusplus
extern "C" {
#endif

void kinematics_forward(float base_rad, float shoulder_rad, float elbow_rad,
                        float *x_mm, float *y_mm, float *z_mm);

void kinematics_pixel_to_mm(uint32_t px_x, uint32_t px_y,
                             float *out_x_mm, float *out_y_mm);

ArmAngles kinematics_solve_ik(float x_mm, float y_mm, float z_mm);

ArmAngles kinematics_bin_angles(uint32_t bin_px_x, uint32_t bin_px_y);

#ifdef __cplusplus
}
#endif

#endif
