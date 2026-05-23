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

#endif
