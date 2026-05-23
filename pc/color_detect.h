#ifndef COLOR_DETECT_H
#define COLOR_DETECT_H

#include <stdint.h>
#include "config.h"
#include "types.h"

#define MAX(a, b) ((a) > (b) ? (a) : (b))
#define MIN(a, b) ((a) < (b) ? (a) : (b))

#define RED_H_MIN      0
#define RED_H_MAX     10
#define RED_H_WRAP_MIN 200
#define RED_S_MIN    120
#define RED_V_MIN     80

#define GREEN_H_MIN   60
#define GREEN_H_MAX  100
#define GREEN_S_MIN  100
#define GREEN_V_MIN   80

#define BLUE_H_MIN   140
#define BLUE_H_MAX   180
#define BLUE_S_MIN   100
#define BLUE_V_MIN    80

#define YELLOW_H_MIN   20
#define YELLOW_H_MAX   50
#define YELLOW_S_MIN  100
#define YELLOW_V_MIN   80

DetectionResult color_detect_scan(uint16_t *fb, int w, int h, Color target);
DetectionResult color_detect_scan_pixels(Pixel *pixels, int w, int h, Color target);
void draw_centroid(Pixel *pixels, int w, int h, int cx, int cy, Color color);

#endif
