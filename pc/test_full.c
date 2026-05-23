#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "camera.h"
#include "calibrate.h"
#include "color_detect.h"
#include "kinematics.h"
#include "types.h"
#include "config.h"

/*
 * test_full — Offline pipeline test.
 *
 * Loads a PNG, detects 4 dark calibration dots, computes the homography
 * matrix via DLT, then detects colored blocks and prints their real-world
 * (mm) positions and IK servo angles.
 *
 * Usage:
 *   ./test_full [image.png]
 *
 * The image should contain:
 *   - 4 dark dots (~10mm) at the corners of a 200×200mm square,
 *     centered in the frame
 *   - Color blocks (red, green, blue, yellow) matching the HSV thresholds
 *
 * Calibration dots expected at (±100, ±100)mm in robot base frame.
 */

// ── Apply homography: pixel(u,v) → robot mm(x,y) ──────────────────────
static void apply_h(const float H[3][3], uint32_t u, uint32_t v,
                     float *x, float *y)
{
    float w = H[2][0]*(float)u + H[2][1]*(float)v + 1.0f;
    *x = (H[0][0]*(float)u + H[0][1]*(float)v + H[0][2]) / w;
    *y = (H[1][0]*(float)u + H[1][1]*(float)v + H[1][2]) / w;
}

// ── Convert raw RGB888 buffer to Pixel struct array ────────────────────
static Pixel* rgb_to_pixels(const uint8_t *rgb, int w, int h)
{
    Pixel *pix = (Pixel*)malloc((size_t)w * h * sizeof(Pixel));
    if (!pix) return NULL;
    for (int i = 0; i < w * h; i++) {
        pix[i].r = rgb[i * 3 + 0];
        pix[i].g = rgb[i * 3 + 1];
        pix[i].b = rgb[i * 3 + 2];
    }
    return pix;
}

int main(int argc, char **argv)
{
    const char *path = (argc > 1) ? argv[1] : "test_images/input.png";

    // ── Load image ───────────────────────────────────────────────────
    int w = 0, h = 0;
    uint8_t *rgb = camera_load_image(path, &w, &h);
    if (!rgb) {
        fprintf(stderr, "Error: cannot load %s\n", path);
        return 1;
    }
    printf("Loaded: %s  (%d × %d)\n\n", path, w, h);

    Pixel *pixels = rgb_to_pixels(rgb, w, h);
    if (!pixels) {
        fprintf(stderr, "Error: malloc failed\n");
        camera_free_image(rgb);
        return 1;
    }

    // ─────────────────────────────────────────────────────────────────
    //  Step 1 — Calibration: detect 4 dark dots, compute homography
    // ─────────────────────────────────────────────────────────────────
    printf("=== Step 1: Calibration ===\n");

    uint32_t dot_ux[4], dot_uy[4];
    int ndots = calibrate_find_dots(pixels, w, h, dot_ux, dot_uy);
    printf("  Detected %d/4 calibration dots\n", ndots);

    if (ndots < 4) {
        printf("  ERROR: need all 4 dots visible. Check image content.\n");
        free(pixels);
        camera_free_image(rgb);
        return 1;
    }

    float H[3][3];
    float rms;
    calibrate_solve(dot_ux, dot_uy, H, &rms);

    printf("\n  Homography matrix H:\n");
    printf("  [%9.5f  %9.5f  %9.5f]\n", H[0][0], H[0][1], H[0][2]);
    printf("  [%9.5f  %9.5f  %9.5f]\n", H[1][0], H[1][1], H[1][2]);
    printf("  [%9.5f  %9.5f  %9.5f]\n", H[2][0], H[2][1], H[2][2]);
    printf("  RMS reprojection error: %.1f mm\n\n", rms);

    // ─────────────────────────────────────────────────────────────────
    //  Step 2 — Object detection: locate colored blocks
    // ─────────────────────────────────────────────────────────────────
    printf("=== Step 2: Object Detection ===\n");

    static const Color colors[] = {COLOR_RED, COLOR_GREEN, COLOR_BLUE, COLOR_YELLOW};
    static const char *names[]  = {"RED", "GREEN", "BLUE", "YELLOW"};
    int obj_count = 0;

    for (int i = 0; i < 4; i++) {
        DetectionResult r = color_detect_scan_pixels(pixels, w, h, colors[i]);
        if (!r.found) {
            printf("  %s: (none)\n", names[i]);
            continue;
        }
        obj_count++;

        // Pixel → mm via homography
        float mm_x, mm_y;
        apply_h(H, r.centroid_x, r.centroid_y, &mm_x, &mm_y);

        // mm → servo angles via IK
        ArmAngles angles = kinematics_solve_ik(mm_x, mm_y);

        printf("  %s:  pixel(%3u, %3u)  →  mm(%7.1f, %7.1f)\n",
               names[i], r.centroid_x, r.centroid_y, mm_x, mm_y);
        printf("         IK: base=%6.1f°  shoulder=%6.1f°  elbow=%6.1f°\n",
               angles.base_deg, angles.shoulder_deg, angles.elbow_deg);
    }

    if (obj_count == 0) {
        printf("  No objects detected.\n");
    }

    // ── Cleanup ──────────────────────────────────────────────────────
    free(pixels);
    camera_free_image(rgb);
    printf("\nDone.\n");
    return 0;
}
