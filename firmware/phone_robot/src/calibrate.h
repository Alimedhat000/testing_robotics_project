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

/**
 * @brief Load homography matrix from NVS at boot.
 * Must be called once in setup() before any other calibrate function.
 * If no saved matrix exists, falls back to linear pixel→mm mapping.
 */
void calibrate_init(void);

/**
 * @brief Check if a valid homography matrix is loaded.
 * @return true if homography is available (calibrated), false for fallback.
 */
bool calibrate_is_done(void);

/**
 * @brief Transform pixel coordinates to robot-base millimeters.
 * Uses the loaded homography matrix H. Falls back to linear mapping
 * if calibrate_is_done() is false.
 * @param px  Pixel X coordinate (0–IMG_WIDTH-1)
 * @param py  Pixel Y coordinate (0–IMG_HEIGHT-1)
 * @param mm_x  Output: X in robot base frame (mm)
 * @param mm_y  Output: Y in robot base frame (mm)
 */
void calibrate_apply(uint32_t px, uint32_t py, float *mm_x, float *mm_y);

/**
 * @brief Detect 4 dark calibration dots in a captured frame.
 * Divides image into quadrants, finds the dark-pixel centroid in each.
 * A pixel is "dark" when R, G, and B are all below DARK_THRESHOLD.
 * @param pixels  RGB888 pixel matrix (IMG_HEIGHT × IMG_WIDTH)
 * @param w  Image width in pixels
 * @param h  Image height in pixels
 * @param out_ux  Output array of 4 pixel X coordinates (quadrant order: TL, TR, BL, BR)
 * @param out_uy  Output array of 4 pixel Y coordinates
 * @return Number of dots found (0–4)
 */
int  calibrate_find_dots(Pixel *pixels, int w, int h,
                          uint32_t out_ux[CALIB_N_POINTS],
                          uint32_t out_uy[CALIB_N_POINTS]);

int  calibrate_find_dots_rgb565(const uint16_t *rgb565, int w, int h,
                                 uint32_t out_ux[CALIB_N_POINTS],
                                 uint32_t out_uy[CALIB_N_POINTS]);

/**
 * @brief Solve homography via DLT (Direct Linear Transform).
 * Builds an 8×8 system from 4 pixel→mm point pairs and solves via
 * Gaussian elimination with partial pivoting.  Also computes RMS
 * reprojection error.
 * @param ux  Pixel X coordinates of 4 dots (from calibrate_find_dots)
 * @param uy  Pixel Y coordinates of 4 dots
 * @param H   Output: 3×3 homography matrix (H[2][2] = 1.0)
 * @param rms Output: RMS reprojection error in mm
 * @return true on success
 */
bool calibrate_solve(const uint32_t ux[CALIB_N_POINTS],
                      const uint32_t uy[CALIB_N_POINTS],
                      float H[3][3], float *rms);

/**
 * @brief Print reprojection error for each dot.
 * Evaluates the given H matrix against stored pixel→mm correspondences
 * and prints per-dot error + RMS to serial.
 * @param H  3×3 homography matrix
 * @param ux  Pixel X coordinates of 4 dots
 * @param uy  Pixel Y coordinates of 4 dots
 */
void calibrate_reproject(const float H[3][3],
                          const uint32_t ux[CALIB_N_POINTS],
                          const uint32_t uy[CALIB_N_POINTS]);

/**
 * @brief Save homography matrix to NVS (ESP32 Preferences).
 * Persists the 3×3 matrix as 9 floats + a valid flag.
 * Also updates the live internal matrix used by calibrate_apply().
 * @param H  3×3 homography matrix to persist
 */
void calibrate_save_matrix(const float H[3][3]);

/**
 * @brief Load homography matrix from NVS.
 * @param H  Output: loaded 3×3 matrix (or identity on failure)
 * @return true if a valid matrix was loaded
 */
bool calibrate_load_matrix(float H[3][3]);

/**
 * @brief Delete saved homography from NVS.
 * After calling, calibrate_is_done() returns false until re-calibrated.
 */
void calibrate_erase(void);

#ifdef __cplusplus
}
#endif

#endif
