#ifndef COLOR_DETECT_H
#define COLOR_DETECT_H

#include <stdint.h>
#include "config.h"
#include "types.h"

#define MAX(a, b) ((a) > (b) ? (a) : (b))
#define MIN(a, b) ((a) < (b) ? (a) : (b))

/* HSV Thresholds (0–255 hue scale, calibrate for your lighting) */

/* Red — wraps around 0 on the hue wheel, needs both ends */
#define RED_H_MIN      0
#define RED_H_MAX     10
#define RED_H_WRAP_MIN 200   /* high-end wrap for red */
#define RED_S_MIN    120
#define RED_V_MIN     80

/* Green */
#define GREEN_H_MIN   60
#define GREEN_H_MAX  100
#define GREEN_S_MIN  100
#define GREEN_V_MIN   80

/* Blue */
#define BLUE_H_MIN   140
#define BLUE_H_MAX   180
#define BLUE_S_MIN   100
#define BLUE_V_MIN    80

/* Yellow */
#define YELLOW_H_MIN   20
#define YELLOW_H_MAX   50
#define YELLOW_S_MIN  100
#define YELLOW_V_MIN   80

/*
 * Scan an RGB565 frame buffer and find the centroid of pixels
 * matching the given target color.
 *   fb     — RGB565 frame buffer (w × h pixels)
 *   w, h   — image dimensions (allows non-QVGA input)
 *   target — which color to look for
 * Returns a DetectionResult with:
 *   .found      = true if a blob larger than MIN_BLOB_SIZE was found
 *   .centroid_x / .centroid_y = blob center in pixels
 *   .color      = target (set only when .found is true)
 */
#ifdef __cplusplus
extern "C" {
#endif

DetectionResult color_detect_scan(uint16_t *fb, int w, int h, Color target);

DetectionResult color_detect_scan_pixels(Pixel *pixels, int w, int h, Color target);

void draw_centroid(Pixel *pixels, int w, int h, int cx, int cy, Color color);

#ifdef __cplusplus
}
#endif

#endif /* COLOR_DETECT_H */
