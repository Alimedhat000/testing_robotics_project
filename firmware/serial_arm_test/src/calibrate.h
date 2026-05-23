#ifndef CALIBRATE_H
#define CALIBRATE_H

#include <stdbool.h>
#include <stdint.h>

#define CALIB_N_POINTS 4

/* Stub — serial_arm_test doesn't use homography.
   kinematics.c includes this for kinematics_pixel_to_mm;
   we never call that path (we send raw mm positions). */
static inline bool calibrate_is_done(void) { return false; }
static inline void calibrate_apply(uint32_t px, uint32_t py, float *x, float *y) {
    (void)px; (void)py; *x = 0; *y = 0;
}

#endif
