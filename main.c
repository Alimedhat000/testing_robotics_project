#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "color_detect.h"
#include "config.h"

#define CHK(cond, msg) do { if (!(cond)) { fprintf(stderr, "Error: %s\n", msg); goto cleanup; } } while(0)

static uint16_t rgb888_to_rgb565(uint8_t r, uint8_t g, uint8_t b)
{
    return ((r >> 3) << 11) | ((g >> 2) << 5) | (b >> 3);
}

/* Draw a filled circle with an outline ring */
static void draw_circle(uint8_t *rgb, int w, int h, int cx, int cy, int r)
{
    uint8_t white[3] = {255, 255, 255};
    uint8_t black[3] = {0, 0, 0};

    int x0 = cx - r - 1; if (x0 < 0) x0 = 0;
    int y0 = cy - r - 1; if (y0 < 0) y0 = 0;
    int x1 = cx + r + 1; if (x1 >= w) x1 = w - 1;
    int y1 = cy + r + 1; if (y1 >= h) y1 = h - 1;

    for (int y = y0; y <= y1; y++) {
        for (int x = x0; x <= x1; x++) {
            int dx = x - cx, dy = y - cy;
            int dist = (int)sqrtf((float)(dx*dx + dy*dy));
            uint8_t *p = &rgb[(y * w + x) * 3];

            if (dist == r || dist == r - 1) {
                /* Outline ring */
                p[0] = black[0]; p[1] = black[1]; p[2] = black[2];
            } else if (dist < r - 1) {
                /* Filled interior */
                p[0] = white[0]; p[1] = white[1]; p[2] = white[2];
            }
        }
    }
}

/* 5x7 bitmap letter definitions (each row is a 5-bit mask, MSB = leftmost pixel) */
static const uint8_t letter_R[7] = {0x1E, 0x11, 0x1E, 0x11, 0x11, 0x11, 0x11};
static const uint8_t letter_G[7] = {0x0E, 0x11, 0x10, 0x13, 0x11, 0x11, 0x0E};
static const uint8_t letter_B[7] = {0x1E, 0x11, 0x11, 0x1E, 0x11, 0x11, 0x1E};
static const uint8_t letter_Y[7] = {0x11, 0x11, 0x0A, 0x04, 0x04, 0x04, 0x04};
static const uint8_t * const letter_map[] = {NULL, letter_R, letter_G, letter_B, letter_Y};

/* Draw a monochrome bitmap letter (fg color on bg, or transparent) at (cx,cy) with given scale */
static void draw_letter(uint8_t *rgb, int w, int h, int cx, int cy,
                        const uint8_t *bitmap, int bw, int bh, int scale,
                        const uint8_t fg[3])
{
    for (int row = 0; row < bh; row++) {
        for (int col = 0; col < bw; col++) {
            if (!(bitmap[row] & (1 << (bw - 1 - col))))
                continue;
            for (int dy = 0; dy < scale; dy++)
                for (int dx = 0; dx < scale; dx++) {
                    int px = cx + col * scale + dx;
                    int py = cy + row * scale + dy;
                    if (px >= 0 && px < w && py >= 0 && py < h) {
                        uint8_t *p = &rgb[(py * w + px) * 3];
                        p[0] = fg[0]; p[1] = fg[1]; p[2] = fg[2];
                    }
                }
        }
    }
}

int main(int argc, char **argv)
{
    const char *input_path  = (argc > 1) ? argv[1] : "input.png";
    const char *output_path = (argc > 2) ? argv[2] : "output.png";

    uint8_t  *rgb_in = NULL;
    uint16_t *fb     = NULL;
    int w = 0, h = 0, ch = 0;

    rgb_in = stbi_load(input_path, &w, &h, &ch, 3);
    CHK(rgb_in != NULL, "Cannot read input image (PNG/JPG/BMP)");

    fb = malloc((size_t)w * h * sizeof(uint16_t));
    CHK(fb != NULL, "malloc frame buffer");

    for (int y = 0; y < h; y++)
        for (int x = 0; x < w; x++) {
            int idx = (y * w + x) * 3;
            fb[y * w + x] = rgb888_to_rgb565(rgb_in[idx], rgb_in[idx+1], rgb_in[idx+2]);
        }

    static const Color scan_colors[] = {COLOR_RED, COLOR_GREEN, COLOR_BLUE, COLOR_YELLOW};
    static const char *color_names[] = {"NONE", "RED", "GREEN", "BLUE", "YELLOW"};
    int found_any = 0;

    for (int i = 0; i < 4; i++) {
        Color c = scan_colors[i];
        DetectionResult res = color_detect_scan(fb, w, h, c);

        if (res.found) {
            found_any = 1;
            int cx = (int)res.centroid_x, cy = (int)res.centroid_y;
            printf("%s centroid: (%u, %u)\n", color_names[c], res.centroid_x, res.centroid_y);

            uint8_t black[3] = {0, 0, 0};
            draw_circle(rgb_in, w, h, cx, cy, 14);
            draw_letter(rgb_in, w, h, cx - 5, cy - 6,
                        letter_map[c], 5, 7, 2, black);
        } else {
            printf("%s centroid: (none)\n", color_names[c]);
        }
    }

    CHK(stbi_write_png(output_path, w, h, 3, rgb_in, w * 3) != 0,
        "Cannot write output PNG");
    printf("Saved: %s\n", output_path);

cleanup:
    stbi_image_free(rgb_in);
    free(fb);
    return found_any ? 0 : 1;
}
