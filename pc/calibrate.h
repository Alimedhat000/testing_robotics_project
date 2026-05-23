#ifndef CALIBRATE_H
#define CALIBRATE_H

#include <stdint.h>
#include <stdbool.h>
#include "config.h"
#include "types.h"

#define CALIB_N_POINTS 4

extern const float CALIB_MM[CALIB_N_POINTS][2];

int  calibrate_find_dots(Pixel *pixels, int w, int h,
                          uint32_t out_ux[CALIB_N_POINTS],
                          uint32_t out_uy[CALIB_N_POINTS]);

int  calibrate_find_dots_rgb565(const uint16_t *rgb565, int w, int h,
                                 uint32_t out_ux[CALIB_N_POINTS],
                                 uint32_t out_uy[CALIB_N_POINTS]);

bool calibrate_solve(const uint32_t ux[CALIB_N_POINTS],
                      const uint32_t uy[CALIB_N_POINTS],
                      float H[3][3], float *rms);

void calibrate_reproject(const float H[3][3],
                          const uint32_t ux[CALIB_N_POINTS],
                          const uint32_t uy[CALIB_N_POINTS]);
                          // Estimate missing 4th dot pixel position from 3 known correspondences
// missing_idx: which dot (0=top,1=left,2=right,3=bottom) is absent
// Returns false if estimation is not possible
bool calibrate_estimate_missing_dot(
        const uint32_t ux[CALIB_N_POINTS],
        const uint32_t uy[CALIB_N_POINTS],
        int present[CALIB_N_POINTS],   // 1=found, 0=missing
        uint32_t out_ux[CALIB_N_POINTS],
        uint32_t out_uy[CALIB_N_POINTS]);

#endif
