#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"

#include "calibrate.h"
#include "camera.h"
#include "color_detect.h"
#include "config.h"
#include "kinematics.h"
#include "types.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/*
 * test_full — Offline pipeline test with visual output.
 *
 * Loads an image, detects 4 dark calibration dots, computes the homography
 * matrix via DLT, then detects colored blocks and prints their real-world
 * (mm) positions and IK servo angles.
 *
 * Produces test_images/output.png with annotations:
 *   - Green crosshairs at calibration dot positions
 *   - White circles with R/G/B/Y letters at detected object centroids
 *
 * Usage:
 *   ./test_full [image.png]
 *
 * The image should contain:
 *   - 4 dark dots (~10mm) at the corners of a 200×200mm square,
 *     centered in the frame
 *   - Color blocks (red, green, blue, yellow) matching the HSV thresholds
 */

// ── Apply homography: pixel(u,v) → robot mm(x,y) ──────────────────────
static void apply_h(const float H[3][3], uint32_t u, uint32_t v, float *x,
                    float *y) {
  float w = H[2][0] * (float)u + H[2][1] * (float)v + 1.0f;
  *x = (H[0][0] * (float)u + H[0][1] * (float)v + H[0][2]) / w;
  *y = (H[1][0] * (float)u + H[1][1] * (float)v + H[1][2]) / w;
}

// ── Convert raw RGB888 buffer to Pixel struct array ────────────────────
static Pixel *rgb_to_pixels(const uint8_t *rgb, int w, int h) {
  Pixel *pix = (Pixel *)malloc((size_t)w * h * sizeof(Pixel));
  if (!pix)
    return NULL;
  for (int i = 0; i < w * h; i++) {
    pix[i].r = rgb[i * 3 + 0];
    pix[i].g = rgb[i * 3 + 1];
    pix[i].b = rgb[i * 3 + 2];
  }
  return pix;
}

// ── Convert annotated Pixel array back to flat RGB888 ──────────────────
static void pixels_to_rgb(const Pixel *pixels, uint8_t *rgb, int w, int h) {
  for (int i = 0; i < w * h; i++) {
    rgb[i * 3 + 0] = pixels[i].r;
    rgb[i * 3 + 1] = pixels[i].g;
    rgb[i * 3 + 2] = pixels[i].b;
  }
}

// ── Draw a green crosshair at a pixel position ─────────────────────────
static void draw_crosshair(Pixel *pixels, int w, int h, int cx, int cy) {
  int size = 6;
  for (int d = -size; d <= size; d++) {
    int px = cx + d;
    int py = cy;
    if (px >= 0 && px < w) {
      pixels[py * w + px].r = 0;
      pixels[py * w + px].g = 255;
      pixels[py * w + px].b = 0;
    }
  }
  for (int d = -size; d <= size; d++) {
    int px = cx;
    int py = cy + d;
    if (py >= 0 && py < h) {
      pixels[py * w + px].r = 0;
      pixels[py * w + px].g = 255;
      pixels[py * w + px].b = 0;
    }
  }
}

// ── Brightness = simple luminance (avg) ─────────────────────────────
static inline int brightness(const Pixel *p) {
  return ((int)p->r + (int)p->g + (int)p->b) / 3;
}

// ── Average brightness in a 5×5 neighborhood ────────────────────────
static int local_brightness(const Pixel *pixels, int w, int h, int cx, int cy) {
  int sum = 0, n = 0;
  for (int dy = -2; dy <= 2; dy++)
    for (int dx = -2; dx <= 2; dx++) {
      int px = cx + dx, py = cy + dy;
      if (px < 0 || px >= w || py < 0 || py >= h) continue;
      sum += brightness(&pixels[py * w + px]);
      n++;
    }
  return n > 0 ? sum / n : 255;
}

// ── Flood-fill using contrast-based threshold (mirrors calibrate.c) ───
static int dbg_flood(Pixel *pixels, int w, int h, int sx, int sy,
                     uint8_t visited[], int *qx, int *qy, int max_q,
                     int *min_x, int *max_x, int *min_y, int *max_y,
                     int *out_chroma, int thresh, int abs_thresh, int max_span)
{
  int head = 0, tail = 0, count = 0;
  *min_x = sx; *max_x = sx; *min_y = sy; *max_y = sy;
  int chroma_sum = 0;
  qx[tail] = sx; qy[tail] = sy; tail++;
  visited[sy * w + sx] = 1;

  static const int dx[] = {0, 0, -1, 1};
  static const int dy[] = {-1, 1, 0, 0};
  while (head < tail) {
    int x = qx[head], y = qy[head]; head++;
    count++;
    {
      Pixel *p = &pixels[y * w + x];
      int maxc = p->r > p->g ? (int)p->r : (int)p->g;
      int minc = p->r < p->g ? (int)p->r : (int)p->g;
      if ((int)p->b > maxc) maxc = (int)p->b;
      if ((int)p->b < minc) minc = (int)p->b;
      chroma_sum += maxc - minc;
    }
    if (x < *min_x) *min_x = x;
    if (x > *max_x) *max_x = x;
    if (y < *min_y) *min_y = y;
    if (y > *max_y) *max_y = y;
    if ((*max_x - *min_x) > max_span || (*max_y - *min_y) > max_span) {
      for (int i = 0; i < tail; i++)
        visited[qy[i] * w + qx[i]] = 0;
      visited[sy * w + sx] = 1;
      return 0;
    }
    for (int d = 0; d < 4; d++) {
      int nx = x + dx[d], ny = y + dy[d];
      if (nx < 0 || nx >= w || ny < 0 || ny >= h) continue;
      if (visited[ny * w + nx]) continue;
      int nb = brightness(&pixels[ny * w + nx]);
      if (nb >= thresh) continue;
      if (nb >= abs_thresh) continue;
      visited[ny * w + nx] = 1;
      if (tail >= max_q) continue;
      qx[tail] = nx; qy[tail] = ny;
      tail++;
    }
  }
  *out_chroma = count > 0 ? (chroma_sum / count) : 0;
  return count;
}

// ── Draw bounding boxes around contrast-based dark blobs ────────────
static int draw_blob_boxes(Pixel *pixels, int w, int h, int abs_thresh,
                           int filtered_only, int verbose) {
  uint8_t *visited = (uint8_t *)calloc((size_t)w * h, 1);
  int *qmem = (int *)malloc((size_t)w * h * 2 * sizeof(int));
  if (!visited || !qmem) { free(visited); free(qmem); return 0; }
  int *qx = qmem, *qy = qmem + (size_t)w * h;

  int min_blob = MIN_DARK_BLOB > w * h / 10000 ? MIN_DARK_BLOB : w * h / 10000;
  int max_span = (w > h ? w : h) / 8;
  int dot_max_span = max_span / 3;
  if (dot_max_span < 8) dot_max_span = 8;
  int dot_min_contrast = SEED_CONTRAST;

  if (verbose) {
    printf("\n=== Debug: All Dark Blobs (SEED_CONTRAST=%d, FLOOD_CONTRAST=%d) ===\n",
           SEED_CONTRAST, FLOOD_CONTRAST);
    printf("  min_blob=%d  max_span=%d  abs_thresh=%d\n\n",
           min_blob, max_span, abs_thresh);
  }

  int idx = 0;
  for (int y = 0; y < h && idx < 48; y++) {
    for (int x = 0; x < w && idx < 48; x++) {
      if (visited[y * w + x]) continue;

      int seed_bright = brightness(&pixels[y * w + x]);
      if (seed_bright > abs_thresh) continue;
      int local_bright = local_brightness(pixels, w, h, x, y);
      int seed_thresh = local_bright - SEED_CONTRAST;
      if (seed_bright > seed_thresh) continue;

      int flood_thresh = local_bright - FLOOD_CONTRAST;

      int min_x, max_x, min_y, max_y;
      int avg_chroma = 0;
      int count = dbg_flood(pixels, w, h, x, y, visited, qx, qy, w * h,
                            &min_x, &max_x, &min_y, &max_y, &avg_chroma,
                            flood_thresh, abs_thresh, max_span);
      if (count < min_blob) continue;
      int span_x = max_x - min_x;
      int span_y = max_y - min_y;
      int span = span_x > span_y ? span_x : span_y;
      uint32_t cx = (uint32_t)(min_x + max_x) / 2;
      uint32_t cy = (uint32_t)(min_y + max_y) / 2;
      int contrast = local_bright - seed_bright;
      if (filtered_only && span > dot_max_span) continue;
      if (filtered_only && contrast < dot_min_contrast) continue;
      if (filtered_only && avg_chroma > CALIB_DOT_MAX_CHROMA) continue;

      if (verbose) {
        printf("  blob%2d: pixel(%4u,%4u) span=%d count=%d contrast=%d  "
               "bbox(%d,%d)-(%d,%d)%s\n",
               idx, cx, cy, span, count, contrast,
               min_x, min_y, max_x, max_y,
               span <= max_span ? "" : "  (rejected by max_span)");
      }

      uint8_t r = span <= max_span ? 0 : 255;
      uint8_t g = 255;
      uint8_t b = span <= max_span ? 0 : 128;

      for (int bx = min_x; bx <= max_x; bx++) {
        if (bx >= 0 && bx < w) {
          if (min_y >= 0 && min_y < h) { pixels[min_y * w + bx].r = r; pixels[min_y * w + bx].g = g; pixels[min_y * w + bx].b = b; }
          if (max_y >= 0 && max_y < h) { pixels[max_y * w + bx].r = r; pixels[max_y * w + bx].g = g; pixels[max_y * w + bx].b = b; }
        }
      }
      for (int by = min_y; by <= max_y; by++) {
        if (by >= 0 && by < h) {
          if (min_x >= 0 && min_x < w) { pixels[by * w + min_x].r = r; pixels[by * w + min_x].g = g; pixels[by * w + min_x].b = b; }
          if (max_x >= 0 && max_x < w) { pixels[by * w + max_x].r = r; pixels[by * w + max_x].g = g; pixels[by * w + max_x].b = b; }
        }
      }
      idx++;
    }
  }
  if (verbose) {
    printf("\n  Total blobs found: %d  (min_blob=%d, max_span=%d)\n",
           idx, min_blob, max_span);
  }

  free(visited);
  free(qmem);
  return idx;
}

static Pixel *dup_pixels(const Pixel *src, int w, int h) {
  size_t bytes = (size_t)w * h * sizeof(Pixel);
  Pixel *copy = (Pixel *)malloc(bytes);
  if (!copy) return NULL;
  memcpy(copy, src, bytes);
  return copy;
}

static void brightness_stats(const Pixel *pixels, int w, int h,
                             int *min_bright, int *max_bright) {
  int min_v = 255;
  int max_v = 0;
  for (int i = 0; i < w * h; i++) {
    int b = brightness(&pixels[i]);
    if (b < min_v) min_v = b;
    if (b > max_v) max_v = b;
  }
  *min_bright = min_v;
  *max_bright = max_v;
}

static void save_contrast_map(const Pixel *pixels, int w, int h,
                              uint8_t *rgb, const char *path) {
  for (int y = 0; y < h; y++) {
    for (int x = 0; x < w; x++) {
      int idx = y * w + x;
      int local = local_brightness(pixels, w, h, x, y);
      int b = brightness(&pixels[idx]);
      int contrast = local - b;
      if (contrast < 0) contrast = 0;
      if (contrast > 255) contrast = 255;
      uint8_t v = (uint8_t)contrast;
      rgb[idx * 3 + 0] = v;
      rgb[idx * 3 + 1] = v;
      rgb[idx * 3 + 2] = v;
    }
  }
  camera_save_image(path, rgb, w, h);
}

static void output_dir_from_path(const char *path, char *out_dir,
                                 size_t out_dir_size) {
  const char *slash = strrchr(path, '/');
  if (!slash) {
    snprintf(out_dir, out_dir_size, ".");
    return;
  }
  size_t len = (size_t)(slash - path);
  if (len >= out_dir_size) len = out_dir_size - 1;
  memcpy(out_dir, path, len);
  out_dir[len] = '\0';
}

int main(int argc, char **argv) {
  const char *path = (argc > 1) ? argv[1] : "pc/test_images/sample_1.jpeg";

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

  int min_bright = 0;
  int max_bright = 0;
  brightness_stats(pixels, w, h, &min_bright, &max_bright);
  int abs_thresh = min_bright + (max_bright - min_bright) / 3;
  printf("Brightness range: %d..%d  abs_thresh=%d\n", min_bright, max_bright,
         abs_thresh);

  char out_dir[512];
  output_dir_from_path(path, out_dir, sizeof(out_dir));

  char debug_path[600];
  snprintf(debug_path, sizeof(debug_path), "%s/debug_01_seed_contrast.png",
           out_dir);
  save_contrast_map(pixels, w, h, rgb, debug_path);

  Pixel *dbg_pixels = dup_pixels(pixels, w, h);
  if (dbg_pixels) {
    snprintf(debug_path, sizeof(debug_path), "%s/debug_02_all_blobs.png",
             out_dir);
    draw_blob_boxes(dbg_pixels, w, h, abs_thresh, 0, 1);
    pixels_to_rgb(dbg_pixels, rgb, w, h);
    camera_save_image(debug_path, rgb, w, h);
    free(dbg_pixels);
  }

  // ─────────────────────────────────────────────────────────────────
  //  Step 1 — Calibration: detect 4 dark dots, compute homography
  // ─────────────────────────────────────────────────────────────────
  printf("\n=== Step 1: Calibration ===\n");

  uint32_t dot_ux[4], dot_uy[4];
  int ndots = calibrate_find_dots(pixels, w, h, dot_ux, dot_uy);
  printf("  Detected %d/4 calibration dots\n", ndots);

  dbg_pixels = dup_pixels(pixels, w, h);
  if (dbg_pixels) {
    snprintf(debug_path, sizeof(debug_path), "%s/debug_03_filtered_blobs.png",
             out_dir);
    draw_blob_boxes(dbg_pixels, w, h, abs_thresh, 1, 0);
    pixels_to_rgb(dbg_pixels, rgb, w, h);
    camera_save_image(debug_path, rgb, w, h);
    free(dbg_pixels);
  }

  float H[3][3] = {{0}};
  float rms = 0.0f;
  int have_h = 0;
  if (ndots >= 4) {
    calibrate_solve(dot_ux, dot_uy, H, &rms);
    have_h = 1;

    printf("\n  Homography matrix H:\n");
    printf("  [%9.5f  %9.5f  %9.5f]\n", H[0][0], H[0][1], H[0][2]);
    printf("  [%9.5f  %9.5f  %9.5f]\n", H[1][0], H[1][1], H[1][2]);
    printf("  [%9.5f  %9.5f  %9.5f]\n", H[2][0], H[2][1], H[2][2]);
    printf("  RMS reprojection error: %.1f mm\n\n", rms);

    // Draw green crosshairs on the calibration dots
    for (int i = 0; i < 4; i++)
      draw_crosshair(pixels, w, h, (int)dot_ux[i], (int)dot_uy[i]);
  } else {
    printf("  ERROR: need all 4 dots visible. Check image content.\n");
  }

  // ─────────────────────────────────────────────────────────────
  //  Step 2 — Object detection: locate colored blocks
  // ─────────────────────────────────────────────────────────────
  printf("=== Step 2: Object Detection ===\n");

  static const Color colors[] = {COLOR_RED, COLOR_GREEN, COLOR_BLUE,
                                 COLOR_YELLOW};
  static const char *names[] = {"RED", "GREEN", "BLUE", "YELLOW"};
  int obj_count = 0;

  for (int i = 0; i < 4; i++) {
    DetectionResult r = color_detect_scan_pixels(pixels, w, h, colors[i]);
    if (!r.found) {
      printf("  %s: (none)\n", names[i]);
      continue;
    }
    obj_count++;

    if (have_h) {
      float mm_x, mm_y;
      apply_h(H, r.centroid_x, r.centroid_y, &mm_x, &mm_y);
      ArmAngles angles = kinematics_solve_ik(mm_x, mm_y);
      printf("  %s:  pixel(%3u, %3u)  →  mm(%7.1f, %7.1f)\n", names[i],
             r.centroid_x, r.centroid_y, mm_x, mm_y);
      printf("         IK: base=%6.1f°  shoulder=%6.1f°  elbow=%6.1f°\n",
             angles.base_deg, angles.shoulder_deg, angles.elbow_deg);
    } else {
      printf("  %s:  pixel(%3u, %3u)\n", names[i], r.centroid_x,
             r.centroid_y);
    }

    // Draw white circle with color letter on the annotated image
    draw_centroid(pixels, w, h, (int)r.centroid_x, (int)r.centroid_y,
                  colors[i]);
  }

  if (obj_count == 0)
    printf("  No objects detected.\n");

  // ── Save annotated output image ──────────────────────────────────
  snprintf(debug_path, sizeof(debug_path), "%s/debug_04_calib_dots.png",
           out_dir);
  pixels_to_rgb(pixels, rgb, w, h);
  camera_save_image(debug_path, rgb, w, h);

  char output_path[600];
  snprintf(output_path, sizeof(output_path), "%s/output.png", out_dir);
  if (camera_save_image(output_path, rgb, w, h))
    printf("\nAnnotated image saved: %s\n", output_path);
  else
    fprintf(stderr, "Warning: failed to save %s\n", output_path);

  // ── Cleanup ──────────────────────────────────────────────────────
  free(pixels);
  camera_free_image(rgb);
  printf("Done.\n");
  return 0;
}
