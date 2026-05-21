/* =============================================================
 * kinematics.h
 * Public interface for coordinate transformation and inverse
 * kinematics.
 * ============================================================= 
 */

#ifndef KINEMATICS_H
#define KINEMATICS_H

#include <stdint.h>
#include "config.h"

#define M_PI 3.14159265358979323846

/*
 * Convert image-space pixel coordinates to real-world mm.
 * Assumes a top-down (or fixed-angle) stationary camera whose
 * field of view exactly covers the workspace rectangle defined
 * in config.h.  CALIBRATE those workspace dimensions.
 *
 * px_x, px_y : pixel centroid from vision module
 * out_x_mm   : pointer to receive X in mm
 * out_y_mm   : pointer to receive Y in mm
 */
void kinematics_pixel_to_mm(uint32_t px_x, uint32_t px_y,
                             float *out_x_mm, float *out_y_mm);

/*
 * 2-link planar inverse kinematics (joints 2 & 3).
 * Base rotation (joint 1) is derived from the X/Y direction.
 *
 * x_mm, y_mm : target position in robot workspace (mm)
 * Returns ArmAngles with base_deg, shoulder_deg, elbow_deg.
 * If the target is unreachable, returns the home angles and
 * prints a warning.
 */
ArmAngles kinematics_solve_ik(float x_mm, float y_mm);

/*
 * Convert bin pixel location → mm → servo angles.
 * Convenience wrapper used by the arm controller.
 */
ArmAngles kinematics_bin_angles(uint32_t bin_px_x, uint32_t bin_px_y);

#endif /* KINEMATICS_H */
