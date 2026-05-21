/* =============================================================
 * kinematics.c
 * Coordinate transformation: image pixels → mm → servo degrees.
 *
 * Assumptions:
 *   • Camera is stationary and top-down (or fixed angle).
 *   • Workspace is a flat rectangular table, objects always at
 *     the same Z height, so we only solve 2D IK (X, Y).
 *   • Arm is a 2-link planar manipulator (shoulder + elbow).
 *     Base servo rotates around the Z axis to face the target.
 * ============================================================= 
 */

#include <stdio.h>
#include <math.h>       /* sqrtf, atan2f, acosf, link with -lm */
#include "kinematics.h"
#include "config.h"

/* ------------------------------------------------------------- */
/*  Helper: clamp a float to [lo, hi]                            */
/* ------------------------------------------------------------- */

static float clampf(float v, float lo, float hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

/* ------------------------------------------------------------- */
/*  Step A: Pixel coordinates → real-world mm                    */
/*                                                               */
/*  Simple linear mapping.  This works well when the camera is   */
/*  directly above (orthographic projection).  If the camera is  */
/*  at an angle you need a perspective correction matrix here,   */
/*  PLACEHOLDER: replace with calibrated homography if needed.   */
/*                                                               */
/*  Pixel (0,0) = top-left of image maps to workspace corner.    */
/*  CALIBRATE: WORKSPACE_X_MM, WORKSPACE_Y_MM in config.h.       */
/* ------------------------------------------------------------- */

void kinematics_pixel_to_mm(uint32_t px_x, uint32_t px_y,
                             float *out_x_mm, float *out_y_mm)
{
    *out_x_mm = ((float)px_x / (float)IMG_WIDTH)  * WORKSPACE_X_MM;
    *out_y_mm = ((float)px_y / (float)IMG_HEIGHT) * WORKSPACE_Y_MM;

    printf("[IK] Pixel (%lu, %lu) → (%.1f mm, %.1f mm)\n",
           px_x, px_y, *out_x_mm, *out_y_mm);
}

/* ------------------------------------------------------------- */
/*  Step B: 2-link planar IK (shoulder + elbow)                  */
/*                                                               */
/*  Given a target (x_mm, y_mm) in the robot's base frame:       */
/*                                                               */
/*  1. Base angle   = atan2(y, x)  →  rotate base servo          */
/*  2. Reach        = sqrt(x² + y²)  (horizontal distance)       */
/*  3. Use law of cosines to find elbow angle (θ2)               */
/*  4. Back-calculate shoulder angle (θ1)                        */
/*                                                               */
/*  All angles converted from radians to degrees for servos.     */
/*                                                               */
/*  CALIBRATE: LINK1_MM, LINK2_MM in config.h.                   */
/*  CALIBRATE: servo zero-position offsets if your servos don't  */
/*             sit at 0° when the link is horizontal.            */
/* ------------------------------------------------------------- */

ArmAngles kinematics_solve_ik(float x_mm, float y_mm)
{
    /* Home angles: returned if target is unreachable */
    ArmAngles home = {
        .base_deg     = HOME_BASE_DEG,
        .shoulder_deg = HOME_SHOULDER_DEG,
        .elbow_deg    = HOME_ELBOW_DEG
    };

    const float L1 = LINK1_MM;
    const float L2 = LINK2_MM;

    /* --- Base rotation: angle to face the target in the XY plane --- */
    float base_rad = atan2f(y_mm, x_mm);
    float base_deg = base_rad * (180.0f / (float)M_PI);

    /* Clamp to servo physical limits */
    base_deg = clampf(base_deg, SERVO_BASE_MIN, SERVO_BASE_MAX);

    /* --- Horizontal reach from base to target --- */
    float reach = sqrtf(x_mm * x_mm + y_mm * y_mm);

    /* --- Reachability check ---
     * Target must be within (L1+L2) and outside |L1-L2|.
     * If unreachable, warn and return home.                         */
    if (reach > L1 + L2) {
        printf("[IK] WARNING: target (%.1f, %.1f) mm is OUT OF REACH "
               "(reach=%.1f, max=%.1f) : returning home\n",
               x_mm, y_mm, reach, L1 + L2);
        return home;
    }
    if (reach < fabsf(L1 - L2)) {
        printf("[IK] WARNING: target (%.1f, %.1f) mm is TOO CLOSE "
               "(reach=%.1f, min=%.1f) : returning home\n",
               x_mm, y_mm, reach, fabsf(L1 - L2));
        return home;
    }

    /* --- Elbow angle via law of cosines ---
     *
     *  cos(θ2) = (reach² - L1² - L2²) / (2 * L1 * L2)
     *
     *  θ2 > 0  → elbow-up configuration (preferred for tabletop work) */
    float cos_elbow = (reach * reach - L1 * L1 - L2 * L2)
                      / (2.0f * L1 * L2);

    /* Clamp rounding errors before acos */
    cos_elbow = clampf(cos_elbow, -1.0f, 1.0f);

    float elbow_rad = acosf(cos_elbow);   /* elbow-up: positive angle */
    float elbow_deg = elbow_rad * (180.0f / (float)M_PI);
    elbow_deg = clampf(elbow_deg, SERVO_ELBOW_MIN, SERVO_ELBOW_MAX);

    /* --- Shoulder angle ---
     *
     *  α = atan2(y, x)   (angle to target, same as base in 2D projection)
     *  β = atan2(L2 * sin(θ2), L1 + L2 * cos(θ2))
     *  θ1 = α - β                                                    */
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

    printf("[IK] Solved base: %.1f°  shoulder: %.1f°  elbow: %.1f°\n",
           base_deg, shoulder_deg, elbow_deg);

    return angles;
}

/* ------------------------------------------------------------- */
/*  Convenience: bin pixel location → angles                     */
/*  Bins are defined as pixel positions in config.h.             */
/*  CALIBRATE: RED_BIN_PX_X, GREEN_BIN_PX_X, BLUE_BIN_PX_X etc.  */
/* ------------------------------------------------------------- */

ArmAngles kinematics_bin_angles(uint32_t bin_px_x, uint32_t bin_px_y)
{
    float x_mm, y_mm;
    kinematics_pixel_to_mm(bin_px_x, bin_px_y, &x_mm, &y_mm);
    return kinematics_solve_ik(x_mm, y_mm);
}
