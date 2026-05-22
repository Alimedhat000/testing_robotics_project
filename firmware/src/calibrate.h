#ifndef CALIBRATE_H
#define CALIBRATE_H

#include <stdint.h>
#include <stdbool.h>
#include "config.h"
#include "types.h"

#define CALIB_N_POINTS 4

#ifdef __cplusplus
extern "C" {
#endif

extern const float CALIB_MM[CALIB_N_POINTS][2];

void calibrate_init(void);
bool calibrate_is_done(void);
void calibrate_apply(uint32_t px, uint32_t py, float *mm_x, float *mm_y);

int  calibrate_find_dots(Pixel *pixels, int w, int h,
                          uint32_t out_ux[CALIB_N_POINTS],
                          uint32_t out_uy[CALIB_N_POINTS]);
bool calibrate_solve(const uint32_t ux[CALIB_N_POINTS],
                      const uint32_t uy[CALIB_N_POINTS],
                      float H[3][3], float *rms);
void calibrate_reproject(const float H[3][3],
                          const uint32_t ux[CALIB_N_POINTS],
                          const uint32_t uy[CALIB_N_POINTS]);

void calibrate_save_matrix(const float H[3][3]);
bool calibrate_load_matrix(float H[3][3]);
void calibrate_erase(void);

#ifdef __cplusplus
}
#endif

#endif
