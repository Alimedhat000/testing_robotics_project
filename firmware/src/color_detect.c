#include <stdio.h>
#include "color_detect.h"
#include "config.h"
#include "types.h"

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

/* Scan a Pixel* (RGB888) matrix — same logic, no RGB565 conversion */
DetectionResult color_detect_scan_pixels(Pixel *pixels, int w, int h, Color target)
{
    DetectionResult result = {false, 0, 0, COLOR_NONE};

    uint32_t sum_x = 0;
    uint32_t sum_y = 0;
    uint32_t count = 0;

    for (int y = 0; y < h; y++) {
        for (int x = 0; x < w; x++) {
            Pixel p = pixels[y * w + x];
            HSV   hsv   = rgb_to_hsv(p.r, p.g, p.b);
            Color color = classify_hsv(hsv);

            if (color == target) {
                sum_x += (uint32_t)x;
                sum_y += (uint32_t)y;
                count++;
            }
        }
    }

    if (count >= MIN_BLOB_SIZE) {
        result.found      = true;
        result.centroid_x = sum_x / count;
        result.centroid_y = sum_y / count;
        result.color      = target;
    }

    return result;
}

/* 5x7 bitmap letters: each row is a 5-bit mask, MSB = leftmost pixel */
static const uint8_t letter_R[7] = {0x1E, 0x11, 0x1E, 0x11, 0x11, 0x11, 0x11};
static const uint8_t letter_G[7] = {0x0E, 0x11, 0x10, 0x13, 0x11, 0x11, 0x0E};
static const uint8_t letter_B[7] = {0x1E, 0x11, 0x11, 0x1E, 0x11, 0x11, 0x1E};
static const uint8_t letter_Y[7] = {0x11, 0x11, 0x0A, 0x04, 0x04, 0x04, 0x04};
static const uint8_t * const letter_map[] = {NULL, letter_R, letter_G, letter_B, letter_Y};

/* Draw a white-filled circle with black outline ring and color letter inside */
void draw_centroid(Pixel *pixels, int w, int h, int cx, int cy, Color color)
{
    int r = 14;

    int x0 = cx - r - 1; if (x0 < 0) x0 = 0;
    int y0 = cy - r - 1; if (y0 < 0) y0 = 0;
    int x1 = cx + r + 1; if (x1 >= w) x1 = w - 1;
    int y1 = cy + r + 1; if (y1 >= h) y1 = h - 1;

    int r2   = r * r;
    int rin2 = (r - 2) * (r - 2);

    for (int y = y0; y <= y1; y++) {
        for (int x = x0; x <= x1; x++) {
            int dx = x - cx, dy = y - cy;
            int d2 = dx * dx + dy * dy;

            if (d2 > r2) continue;

            Pixel *p = &pixels[y * w + x];
            if (d2 >= rin2) {
                p->r = 0;   p->g = 0;   p->b = 0;    /* outline */
            } else {
                p->r = 255; p->g = 255; p->b = 255;   /* fill */
            }
        }
    }

    /* Draw color letter inside the circle */
    const uint8_t *bm = letter_map[color];
    if (!bm) return;

    int scale = 2;
    int bw = 5, bh = 7;
    int lx = cx - (bw * scale) / 2;
    int ly = cy - (bh * scale) / 2;

    for (int row = 0; row < bh; row++) {
        for (int col = 0; col < bw; col++) {
            if (!(bm[row] & (1 << (bw - 1 - col))))
                continue;
            for (int dy = 0; dy < scale; dy++) {
                for (int dx = 0; dx < scale; dx++) {
                    int px = lx + col * scale + dx;
                    int py = ly + row * scale + dy;
                    if (px >= 0 && px < w && py >= 0 && py < h) {
                        Pixel *p = &pixels[py * w + px];
                        p->r = 0; p->g = 0; p->b = 0;
                    }
                }
            }
        }
    }
}
