#include <stdio.h>
#include "color_detect.h"
#include "config.h"

/* Convert one RGB888 pixel to HSV (integer arithmetic, 0–255 hue scale) */
static HSV rgb_to_hsv(uint8_t r, uint8_t g, uint8_t b)
{
    uint8_t max_val = MAX(r, MAX(g, b));
    uint8_t min_val = MIN(r, MIN(g, b));
    uint8_t delta   = max_val - min_val;

    HSV hsv;
    hsv.v = max_val;
    hsv.s = (max_val == 0) ? 0 : (uint8_t)(255u * delta / max_val);

    if (delta == 0) {
        hsv.h = 0;
        return hsv;
    }

    if (max_val == r)
        hsv.h = (uint8_t)(43 * (int)(g - b) / delta);
    else if (max_val == g)
        hsv.h = (uint8_t)(85 + 43 * (int)(b - r) / delta);
    else
        hsv.h = (uint8_t)(171 + 43 * (int)(r - g) / delta);

    return hsv;
}

/* Classify a pixel's HSV into a Color */
static Color classify_hsv(HSV hsv)
{
    /* Red wraps around 0: check both low and high ranges */
    if ((hsv.h <= RED_H_MAX || hsv.h >= RED_H_WRAP_MIN) &&
        hsv.s >= RED_S_MIN && hsv.v >= RED_V_MIN)
        return COLOR_RED;

    if (hsv.h >= GREEN_H_MIN && hsv.h <= GREEN_H_MAX &&
        hsv.s >= GREEN_S_MIN && hsv.v >= GREEN_V_MIN)
        return COLOR_GREEN;

    if (hsv.h >= BLUE_H_MIN  && hsv.h <= BLUE_H_MAX  &&
        hsv.s >= BLUE_S_MIN  && hsv.v >= BLUE_V_MIN)
        return COLOR_BLUE;

    if (hsv.h >= YELLOW_H_MIN && hsv.h <= YELLOW_H_MAX &&
        hsv.s >= YELLOW_S_MIN && hsv.v >= YELLOW_V_MIN)
        return COLOR_YELLOW;

    return COLOR_NONE;
}

/*
 * Scan an RGB565 frame buffer, accumulating centroid of pixels
 * matching the target color.
 */
DetectionResult color_detect_scan(uint16_t *fb, int w, int h, Color target)
{
    DetectionResult result = {false, 0, 0, COLOR_NONE};

    uint32_t sum_x = 0;
    uint32_t sum_y = 0;
    uint32_t count = 0;

    for (int y = 0; y < h; y++) {
        for (int x = 0; x < w; x++) {
            uint16_t pixel = fb[y * w + x];
            uint8_t r = ((pixel >> 11) & 0x1F) << 3;
            uint8_t g = ((pixel >>  5) & 0x3F) << 2;
            uint8_t b = ( pixel        & 0x1F) << 3;

            HSV   hsv   = rgb_to_hsv(r, g, b);
            Color color = classify_hsv(hsv);

            if (color == target) {
                sum_x += (uint32_t)x;
                sum_y += (uint32_t)y;
                count++;
            }
        }
    }

    if (count < MIN_BLOB_SIZE) {
        return result;
    }

    result.found      = true;
    result.centroid_x = sum_x / count;
    result.centroid_y = sum_y / count;
    result.color      = target;

    return result;
}
