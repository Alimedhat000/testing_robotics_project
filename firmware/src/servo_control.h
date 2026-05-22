#ifndef SERVO_CONTROL_H
#define SERVO_CONTROL_H

#include <stdint.h>
#include <stdbool.h>
#include "config.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Attach all servos to their pins and home to 0°.
 * Call once in setup() after calibrate_init().  Each servo is
 * attached with 500–2500 µs pulse range, homed sequentially with
 * a 300 ms delay between them.
 */
void servo_init(void);

/**
 * @brief Return all servos to the home position (0°).
 * Uses servo_write_all() for simultaneous smooth interpolation.
 */
void servo_home(void);

/**
 * @brief Move all NUM_SERVOS servos to target angles simultaneously.
 *
 * Each angle is clamped to its per-servo hardware limit from config.h.
 * Movement uses stepped interpolation (1°/step, 15 ms/step) so all
 * servos arrive at their targets at the same time.
 *
 * @param angles_deg  Array of NUM_SERVOS target angles (logical degrees,
 *                    0 = center, mapped to servo.write() via toPhysical)
 */
void servo_write_all(const int angles_deg[NUM_SERVOS]);

/**
 * @brief Move a single servo to a target angle.
 * Other servos remain at their current position.
 * @param index       Servo index (0 = base, 1 = shoulder, 2 = elbow, 3 = gripper)
 * @param logical_deg  Target angle in logical degrees
 */
void servo_write_single(int index, int logical_deg);

/**
 * @brief Interactive serial servo test handler.
 * Reads space-separated angles from Serial and calls servo_write_all().
 * Also handles "HOME" and "END" commands.
 * Activated by the SERVOTEST command in main.cpp's dispatch_serial().
 */
void servo_test_serial(void);

#ifdef __cplusplus
}
#endif

#endif
