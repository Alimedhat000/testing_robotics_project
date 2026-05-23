#ifndef KINEMATICS_H
#define KINEMATICS_H

#include <stdint.h>
#include "config.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Convert image-space pixel coordinates to real-world mm.
 *
 * When calibrated, uses the homography matrix from calibrate_apply().
 * Otherwise falls back to a simple linear mapping:
 *   x_mm = (px_x / IMG_WIDTH)  * WORKSPACE_X_MM
 *   y_mm = (px_y / IMG_HEIGHT) * WORKSPACE_Y_MM
 *
 * @param px_x   Pixel X coordinate (0–IMG_WIDTH-1)
 * @param px_y   Pixel Y coordinate (0–IMG_HEIGHT-1)
 * @param out_x_mm  Output: X in robot base frame (mm)
 * @param out_y_mm  Output: Y in robot base frame (mm)
 */
void kinematics_pixel_to_mm(uint32_t px_x, uint32_t px_y,
                             float *out_x_mm, float *out_y_mm);

/**
 * @brief 2-link planar inverse kinematics solver.
 *
 * Given a target (x_mm, y_mm) in the robot's base frame, computes:
 *   - base_deg    : rotation around Z axis (atan2 of target)
 *   - shoulder_deg: angle of the upper arm (link 1)
 *   - elbow_deg   : angle of the forearm (link 2, elbow-up config)
 *
 * Uses law of cosines on links LINK1_MM and LINK2_MM.
 * All angles are clamped to SERVO_*_MIN / SERVO_*_MAX.
 *
 * @param x_mm  Target X in robot base frame (mm)
 * @param y_mm  Target Y in robot base frame (mm)
 * @return ArmAngles struct with base_deg, shoulder_deg, elbow_deg.
 *         Returns home angles if the target is unreachable.
 */
ArmAngles kinematics_solve_ik(float x_mm, float y_mm);

/**
 * @brief Convenience: bin pixel position → servo angles.
 * Wraps kinematics_pixel_to_mm() + kinematics_solve_ik().
 * @param bin_px_x  Bin centroid pixel X
 * @param bin_px_y  Bin centroid pixel Y
 * @return ArmAngles for the bin's real-world position.
 */
ArmAngles kinematics_bin_angles(uint32_t bin_px_x, uint32_t bin_px_y);

#ifdef __cplusplus
}
#endif

#endif
