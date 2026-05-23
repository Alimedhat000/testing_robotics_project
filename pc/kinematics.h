#ifndef KINEMATICS_H
#define KINEMATICS_H

#include <stdint.h>
#include "config.h"

ArmAngles kinematics_solve_ik(float x_mm, float y_mm);
ArmAngles kinematics_bin_angles(uint32_t bin_px_x, uint32_t bin_px_y);

#endif
