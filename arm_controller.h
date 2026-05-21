/* =============================================================
 * arm_controller.h
 * Public interface for the arm controller module.
 * Handles all servo motion: moving to a position, picking,
 * placing, and returning home.
 * ============================================================= 
 */

#ifndef ARM_CONTROLLER_H
#define ARM_CONTROLLER_H

#include <stdint.h>
#include "config.h"

/*
 * Initialize servos to home position.
 * Call once at startup.
 */
void arm_init(void);

/*
 * Move the three positional servos (1, 2, 3) to the given angles.
 * Does NOT move the gripper (servo 4).
 * Blocks until motion is complete (uses delay, PLACEHOLDER for
 * feedback-based waiting if you add encoders later).
 */
void arm_move_to_angles(ArmAngles angles);

/*
 * Move servos to hover above a pixel target (arm raised).
 * px_x, px_y: object centroid from vision module.
 */
void arm_hover_over(uint32_t px_x, uint32_t px_y);

/*
 * Lower arm to pick height (fixed Z) then raise back to hover.
 * Gripper must already be open before calling this.
 */
void arm_lower_and_pick(void);

/*
 * Move arm to the bin for the given color and drop the object.
 * Gripper will be opened at the bin.
 */
void arm_move_to_bin(Color color);

/*
 * Open gripper (servo 4).
 */
void gripper_open(void);

/*
 * Close gripper (servo 4).
 */
void gripper_close(void);

/*
 * Return all servos to the home position.
 */
void arm_return_home(void);

#endif /* ARM_CONTROLLER_H */
