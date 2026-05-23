#include "calibrate.h"
#include "config.h"
#include "types.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

const float CALIB_MM[CALIB_N_POINTS][2] = {
    {   0.0f,  100.0f },   // dot 0: top     (smallest Y)
    {-150.0f,    0.0f },   // dot 1: left    (smallest X)
    { 150.0f,    0.0f },   // dot 2: right   (largest X)
    {   0.0f, -100.0f },   // dot 3: bottom  (largest Y)
};

// ── Brightness = simple luminance (avg) ─────────────────────────────
// Using average is more stable across colored objects than max(r,g,b).
static inline int brightness(const Pixel *p)
{
    return ((int)p->r + (int)p->g + (int)p->b) / 3;
}

// ── Average brightness in a 5×5 neighborhood ────────────────────────
// Returns 255 if the pixel is too close to the image border.
static int local_brightness(const Pixel *pixels, int w, int h, int cx, int cy)
{
    int sum = 0, n = 0;
    for (int dy = -2; dy <= 2; dy++) {
        for (int dx = -2; dx <= 2; dx++) {
            int px = cx + dx, py = cy + dy;
            if (px < 0 || px >= w || py < 0 || py >= h) continue;
            sum += brightness(&pixels[py * w + px]);
            n++;
        }
    }
    return n > 0 ? sum / n : 255;
}

// ── Flood-fill using contrast-based threshold ───────────────────────
// Computes the darkness threshold once at the seed pixel's local
// brightness. Neighbors pass if their brightness < (seed_local - thresh)
// AND below a global absolute-dark threshold.
// Returns pixel count, or 0 if below min_blob or span > max_span.
static int flood_blob(Pixel *pixels, int w, int h, int sx, int sy,
                      uint8_t visited[], int *qx, int *qy, int max_q,
                      uint32_t *out_x, uint32_t *out_y, int *out_span,
                      int *out_chroma, int *out_perimeter,
                      int *out_bbox_w, int *out_bbox_h,
                      int thresh, int abs_thresh,
                      int min_blob, int max_span)
{
    int head = 0, tail = 0;
    uint32_t sum_x = 0, sum_y = 0;
    int count = 0;
    int min_x = sx, max_x = sx, min_y = sy, max_y = sy;
    int chroma_sum = 0;

    qx[tail] = sx; qy[tail] = sy; tail++;
    visited[sy * w + sx] = 1;

    while (head < tail) {
        int x = qx[head], y = qy[head]; head++;
        sum_x += (uint32_t)x;
        sum_y += (uint32_t)y;
        count++;
        {
            Pixel *p = &pixels[y * w + x];
            int maxc = p->r > p->g ? (int)p->r : (int)p->g;
            int minc = p->r < p->g ? (int)p->r : (int)p->g;
            if ((int)p->b > maxc) maxc = (int)p->b;
            if ((int)p->b < minc) minc = (int)p->b;
            chroma_sum += maxc - minc;
        }
        if (x < min_x) min_x = x;
        if (x > max_x) max_x = x;
        if (y < min_y) min_y = y;
        if (y > max_y) max_y = y;

        // Early-abort: prevents giant floods from consuming visited[] and
        // hiding real dots elsewhere in the frame.
        if ((max_x - min_x) > max_span || (max_y - min_y) > max_span) {
            for (int i = 0; i < tail; i++)
                visited[qy[i] * w + qx[i]] = 0;
            visited[sy * w + sx] = 1;
            return 0;
        }

        static const int dx[] = {0, 0, -1, 1};
        static const int dy[] = {-1, 1, 0, 0};
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

    // ── Post-BFS: compute perimeter ────────────────────────────────
    int perimeter = 0;
    static const int pdx[] = {0, 0, -1, 1};
    static const int pdy[] = {-1, 1, 0, 0};
    for (int i = 0; i < tail; i++) {
        int px = qx[i], py = qy[i];
        for (int d = 0; d < 4; d++) {
            int nx = px + pdx[d], ny = py + pdy[d];
            if (nx < 0 || nx >= w || ny < 0 || ny >= h) { perimeter++; break; }
            int nb = brightness(&pixels[ny * w + nx]);
            if (nb >= thresh || nb >= abs_thresh) { perimeter++; break; }
            if (!visited[ny * w + nx]) { perimeter++; break; }
        }
    }
    *out_perimeter = perimeter;

    if (count < min_blob) return 0;
    int span_x = max_x - min_x;
    int span_y = max_y - min_y;
    int span = span_x > span_y ? span_x : span_y;
    if (span > max_span) return 0;
    *out_x = sum_x / count;
    *out_y = sum_y / count;
    *out_span = span;
    *out_chroma = count > 0 ? (chroma_sum / count) : 0;
    *out_bbox_w = max_x - min_x + 1;
    *out_bbox_h = max_y - min_y + 1;
    return (int)count;
}

// ── Detect 4 calibration dots by contrast-based flood-fill ─────────────
// Finds the 4 darkest small spots relative to their local neighborhood.
// Resolution-independent: span and blob size scale with image dims.
int calibrate_find_dots(Pixel *pixels, int w, int h,
                          uint32_t out_ux[CALIB_N_POINTS],
                          uint32_t out_uy[CALIB_N_POINTS])
{
    int min_bright = 255;
    int max_bright = 0;
    for (int i = 0; i < w * h; i++) {
        int b = brightness(&pixels[i]);
        if (b < min_bright) min_bright = b;
        if (b > max_bright) max_bright = b;
    }
    int abs_thresh = min_bright + (max_bright - min_bright) / CALIB_ABS_THRESH_DIV;

#if CALIB_DEBUG
    printf("[CALIB] Brightness range %d..%d, abs_thresh=%d\n",
           min_bright, max_bright, abs_thresh);
#endif

    uint8_t *visited = (uint8_t *)calloc((size_t)w * h, 1);
    int *qmem = (int *)malloc((size_t)w * h * 2 * sizeof(int));
    if (!visited || !qmem) {
        free(visited); free(qmem);
        return 0;
    }

    int *qx = qmem, *qy = qmem + (size_t)w * h;

    // Dynamic sizing — scales with resolution
    int min_blob = MIN_DARK_BLOB > w * h / 10000
                   ? MIN_DARK_BLOB : w * h / 10000;
    int max_span = (w > h ? w : h) / 8;

    // Blob storage: x, y, size, span, contrast, circularity
    int blobs[32][6];
    int nblobs = 0;

    int dot_max_span = max_span / 3;
    if (dot_max_span < 8) dot_max_span = 8;
    int dot_min_contrast = SEED_CONTRAST;

    for (int y = 0; y < h && nblobs < 32; y++) {
        for (int x = 0; x < w && nblobs < 32; x++) {
            if (visited[y * w + x]) continue;

            // Seed must be significantly darker than surroundings
            int seed_bright = brightness(&pixels[y * w + x]);
            if (seed_bright > abs_thresh) continue;
            int local_bright = local_brightness(pixels, w, h, x, y);
            int seed_thresh = local_bright - SEED_CONTRAST;
            // Inclusive: allow seeds exactly at the contrast threshold.
            if (seed_bright > seed_thresh) continue;

            uint32_t cx, cy;
            int flood_thresh = local_bright - FLOOD_CONTRAST;
            int span, avg_chroma = 0, perimeter = 0, bbox_w = 0, bbox_h = 0;
            int sz = flood_blob(pixels, w, h, x, y, visited, qx, qy, w * h,
                                &cx, &cy, &span, &avg_chroma, &perimeter,
                                &bbox_w, &bbox_h,
                                flood_thresh, abs_thresh, min_blob, max_span);
            if (sz > 0) {
                int contrast = local_bright - seed_bright;
                if (span > dot_max_span) continue;
                if (contrast < dot_min_contrast) continue;
                if (avg_chroma > CALIB_DOT_MAX_CHROMA) continue;
                float circ = 4.0f * 3.14159f * sz / (float)(perimeter * perimeter);
                if (circ < CALIB_DOT_MIN_CIRCULARITY) continue;
                float aspect = (float)(bbox_w < bbox_h ? bbox_w : bbox_h) /
                               (float)(bbox_w > bbox_h ? bbox_w : bbox_h);
                if (aspect < CALIB_DOT_MIN_ASPECT) continue;
                float fill = (float)sz / (float)(bbox_w * bbox_h);
                if (fill < CALIB_DOT_MIN_FILL) continue;
                blobs[nblobs][0] = (int)cx;
                blobs[nblobs][1] = (int)cy;
                blobs[nblobs][2] = sz;
                blobs[nblobs][3] = span;
                blobs[nblobs][4] = contrast;
                blobs[nblobs][5] = (int)(circ * 100 + 0.5f);
#if CALIB_DEBUG
                printf("[CALIB] blob%2d: pixel(%4u,%4u) span=%d count=%d contrast=%d chroma=%d circ=%d aspect=%d fill=%d\n",
                       nblobs, (unsigned)cx, (unsigned)cy, span, sz, contrast,
                       avg_chroma, blobs[nblobs][5],
                       (int)(aspect * 100), (int)(fill * 100));
#endif
                nblobs++;
            }
        }
    }

    free(visited);
    free(qmem);

    if (nblobs < 4) {
        for (int i = 0; i < nblobs && i < 4; i++) {
            out_ux[i] = (uint32_t)blobs[i][0];
            out_uy[i] = (uint32_t)blobs[i][1];
        }
        return nblobs < 4 ? nblobs : 4;
    }

    // Sort by contrast strength descending (most contrast = most dot-like)
    // Contrast is in blobs[i][4]
    for (int i = 0; i < nblobs - 1; i++)
        for (int j = i + 1; j < nblobs; j++)
            if (blobs[j][4] > blobs[i][4]) {
                int t[6]; memcpy(t, blobs[i], sizeof(t));
                memcpy(blobs[i], blobs[j], sizeof(t));
                memcpy(blobs[j], t, sizeof(t));
            }

    // Assign by axis-distance scoring (pick blob closest to each dot's expected position)
    // Expected positions form a cross: top(w/2,0), left(0,h/2), right(w,h/2), bottom(w/2,h)
    int used[32] = {0};
    float cx = w * 0.5f, cy = h * 0.5f;

    for (int role = 0; role < 4; role++) {
        float best_score = 1e20f;
        int best_i = -1;
        for (int i = 0; i < nblobs; i++) {
            if (used[i]) continue;
            float dx, dy;
            switch (role) {
                case 0: dx = blobs[i][0] - cx; dy = blobs[i][1];         break; // top
                case 1: dx = blobs[i][0];       dy = blobs[i][1] - cy;   break; // left
                case 2: dx = w - blobs[i][0];   dy = blobs[i][1] - cy;   break; // right
                case 3: dx = blobs[i][0] - cx; dy = h - blobs[i][1];    break; // bottom
            }
            if (dx < 0) dx = -dx;
            if (dy < 0) dy = -dy;
            float score = dx + dy;
            if (score < best_score) { best_score = score; best_i = i; }
        }
        if (best_i < 0) break; // shouldn't happen since nblobs >= 4
        out_ux[role] = (uint32_t)blobs[best_i][0];
        out_uy[role] = (uint32_t)blobs[best_i][1];
        used[best_i] = 1;
    }

    return 4;
}

#define N 8

static void gauss_elim(float A[N][N], float b[N]) {
  for (int col = 0; col < N; col++) {
    int best = col;
    for (int row = col + 1; row < N; row++)
      if (fabsf(A[row][col]) > fabsf(A[best][col]))
        best = row;
    if (best != col) {
      for (int j = col; j < N; j++) {
        float t = A[col][j];
        A[col][j] = A[best][j];
        A[best][j] = t;
      }
      float t = b[col];
      b[col] = b[best];
      b[best] = t;
    }
    float piv = A[col][col];
    if (fabsf(piv) < 1e-12f) continue;
    for (int row = col + 1; row < N; row++) {
      float f = A[row][col] / piv;
      for (int j = col; j < N; j++)
        A[row][j] -= f * A[col][j];
      b[row] -= f * b[col];
    }
  }
  for (int row = N - 1; row >= 0; row--) {
    float s = b[row];
    for (int j = row + 1; j < N; j++)
      s -= A[row][j] * b[j];
    b[row] = s / A[row][row];
  }
}

bool calibrate_solve(const uint32_t ux[CALIB_N_POINTS],
                      const uint32_t uy[CALIB_N_POINTS], float H[3][3],
                      float *rms) {
  float A[N][N] = {0};
  float b[N] = {0};
  for (int i = 0; i < 4; i++) {
    float u = (float)ux[i], v = (float)uy[i];
    float x = CALIB_MM[i][0], y = CALIB_MM[i][1];
    A[2*i][0] = u;   A[2*i][1] = v;   A[2*i][2] = 1.0f;
    A[2*i][6] = -u*x; A[2*i][7] = -v*x;
    b[2*i] = x;
    A[2*i+1][3] = u;  A[2*i+1][4] = v;  A[2*i+1][5] = 1.0f;
    A[2*i+1][6] = -u*y; A[2*i+1][7] = -v*y;
    b[2*i+1] = y;
  }
  gauss_elim(A, b);
  H[0][0] = b[0]; H[0][1] = b[1]; H[0][2] = b[2];
  H[1][0] = b[3]; H[1][1] = b[4]; H[1][2] = b[5];
  H[2][0] = b[6]; H[2][1] = b[7]; H[2][2] = 1.0f;
  float sum = 0;
  for (int i = 0; i < 4; i++) {
    float u = (float)ux[i], v = (float)uy[i];
    float w = H[2][0]*u + H[2][1]*v + 1.0f;
    float px = (H[0][0]*u + H[0][1]*v + H[0][2]) / w;
    float py = (H[1][0]*u + H[1][1]*v + H[1][2]) / w;
    float dx = px - CALIB_MM[i][0], dy = py - CALIB_MM[i][1];
    float err = sqrtf(dx*dx + dy*dy);
    printf("  Dot %d: pixel(%3u,%3u) → mm(%+.0f,%+.0f)  "
           "reconstructed(%7.1f,%7.1f)  error %.1f mm\n",
           i, ux[i], uy[i], CALIB_MM[i][0], CALIB_MM[i][1], px, py, err);
    sum += err * err;
  }
  *rms = sqrtf(sum / 4.0f);
  return true;
}

void calibrate_reproject(const float H[3][3], const uint32_t ux[CALIB_N_POINTS],
                          const uint32_t uy[CALIB_N_POINTS]) {
  float sum = 0;
  for (int i = 0; i < 4; i++) {
    float u = (float)ux[i], v = (float)uy[i];
    float w = H[2][0]*u + H[2][1]*v + 1.0f;
    float px = (H[0][0]*u + H[0][1]*v + H[0][2]) / w;
    float py = (H[1][0]*u + H[1][1]*v + H[1][2]) / w;
    float dx = px - CALIB_MM[i][0], dy = py - CALIB_MM[i][1];
    float err = sqrtf(dx*dx + dy*dy);
    printf("  Dot %d: pixel(%3u,%3u) → mm(%+.0f,%+.0f)  "
           "reconstructed(%7.1f,%7.1f)  error %.1f mm\n",
           i, ux[i], uy[i], CALIB_MM[i][0], CALIB_MM[i][1], px, py, err);
    sum += err * err;
  }
  printf("  RMS reprojection error: %.1f mm\n", sqrtf(sum / 4.0f));
}

// ── RGB565 calibration path ────────────────────────────────────────

static inline int brightness_rgb565(uint16_t p) {
    int r = (p >> 11) & 0x1F;
    int g = (p >>  5) & 0x3F;
    int b = p & 0x1F;
    return ((r << 3) + (g << 2) + (b << 3)) / 3;
}

static int local_brightness_rgb565(const uint16_t *rgb565, int w, int h,
                                    int cx, int cy) {
    int sum = 0, n = 0;
    for (int dy = -2; dy <= 2; dy++)
        for (int dx = -2; dx <= 2; dx++) {
            int px = cx + dx, py = cy + dy;
            if (px < 0 || px >= w || py < 0 || py >= h) continue;
            sum += brightness_rgb565(rgb565[py * w + px]);
            n++;
        }
    return n > 0 ? sum / n : 255;
}

static int flood_blob_rgb565(const uint16_t *rgb565, int w, int h,
                              int sx, int sy, uint8_t visited[],
                              int *qx, int *qy, int max_q,
                              uint32_t *out_x, uint32_t *out_y, int *out_span,
                              int *out_chroma, int *out_perimeter,
                              int *out_bbox_w, int *out_bbox_h,
                              int thresh, int abs_thresh,
                              int min_blob, int max_span) {
    int head = 0, tail = 0;
    uint32_t sum_x = 0, sum_y = 0;
    int count = 0;
    int min_x = sx, max_x = sx, min_y = sy, max_y = sy;
    int chroma_sum = 0;

    qx[tail] = sx; qy[tail] = sy; tail++;
    visited[sy * w + sx] = 1;

    while (head < tail) {
        int x = qx[head], y = qy[head]; head++;
        sum_x += (uint32_t)x;
        sum_y += (uint32_t)y;
        count++;
        {
            uint16_t p = rgb565[y * w + x];
            int r = (p >> 11) & 0x1F;
            int g = (p >>  5) & 0x3F;
            int b = p & 0x1F;
            int maxc = r > g ? r : g; if (b > maxc) maxc = b;
            int minc = r < g ? r : g; if (b < minc) minc = b;
            chroma_sum += maxc - minc;
        }
        if (x < min_x) min_x = x;
        if (x > max_x) max_x = x;
        if (y < min_y) min_y = y;
        if (y > max_y) max_y = y;

        if ((max_x - min_x) > max_span || (max_y - min_y) > max_span) {
            for (int i = 0; i < tail; i++)
                visited[qy[i] * w + qx[i]] = 0;
            visited[sy * w + sx] = 1;
            return 0;
        }

        static const int dx[] = {0, 0, -1, 1};
        static const int dy[] = {-1, 1, 0, 0};
        for (int d = 0; d < 4; d++) {
            int nx = x + dx[d], ny = y + dy[d];
            if (nx < 0 || nx >= w || ny < 0 || ny >= h) continue;
            if (visited[ny * w + nx]) continue;
            int nb = brightness_rgb565(rgb565[ny * w + nx]);
            if (nb >= thresh) continue;
            if (nb >= abs_thresh) continue;
            visited[ny * w + nx] = 1;
            if (tail >= max_q) continue;
            qx[tail] = nx; qy[tail] = ny;
            tail++;
        }
    }

    // ── Post-BFS: compute perimeter ────────────────────────────────
    int perimeter = 0;
    static const int pdx[] = {0, 0, -1, 1};
    static const int pdy[] = {-1, 1, 0, 0};
    for (int i = 0; i < tail; i++) {
        int px = qx[i], py = qy[i];
        for (int d = 0; d < 4; d++) {
            int nx = px + pdx[d], ny = py + pdy[d];
            if (nx < 0 || nx >= w || ny < 0 || ny >= h) { perimeter++; break; }
            int nb = brightness_rgb565(rgb565[ny * w + nx]);
            if (nb >= thresh || nb >= abs_thresh) { perimeter++; break; }
            if (!visited[ny * w + nx]) { perimeter++; break; }
        }
    }
    *out_perimeter = perimeter;

    if (count < min_blob) return 0;
    int span_x = max_x - min_x;
    int span_y = max_y - min_y;
    int span = span_x > span_y ? span_x : span_y;
    if (span > max_span) return 0;
    *out_x = sum_x / count;
    *out_y = sum_y / count;
    *out_span = span;
    *out_chroma = count > 0 ? (chroma_sum / count) : 0;
    *out_bbox_w = max_x - min_x + 1;
    *out_bbox_h = max_y - min_y + 1;
    return count;
}

int calibrate_find_dots_rgb565(const uint16_t *rgb565, int w, int h,
                                uint32_t out_ux[CALIB_N_POINTS],
                                uint32_t out_uy[CALIB_N_POINTS]) {
    int min_bright = 255, max_bright = 0;
    for (int i = 0; i < w * h; i++) {
        int b = brightness_rgb565(rgb565[i]);
        if (b < min_bright) min_bright = b;
        if (b > max_bright) max_bright = b;
    }
    int abs_thresh = min_bright + (max_bright - min_bright) / CALIB_ABS_THRESH_DIV;

#if CALIB_DEBUG
    printf("[CALIB_RGB565] Brightness range %d..%d, abs_thresh=%d\n",
           min_bright, max_bright, abs_thresh);
#endif

    uint8_t *visited = (uint8_t *)calloc((size_t)w * h, 1);
    int *qmem = (int *)malloc((size_t)w * h * 2 * sizeof(int));
    if (!visited || !qmem) { free(visited); free(qmem); return 0; }

    int *qx = qmem, *qy = qmem + (size_t)w * h;

    int min_blob = MIN_DARK_BLOB > w * h / 10000
                   ? MIN_DARK_BLOB : w * h / 10000;
    int max_span = (w > h ? w : h) / 8;

    int blobs[32][6];
    int nblobs = 0;

    int dot_max_span = max_span / 3;
    if (dot_max_span < 8) dot_max_span = 8;
    int dot_min_contrast = SEED_CONTRAST;

    for (int y = 0; y < h && nblobs < 32; y++) {
        for (int x = 0; x < w && nblobs < 32; x++) {
            if (visited[y * w + x]) continue;

            int seed_bright = brightness_rgb565(rgb565[y * w + x]);
            if (seed_bright > abs_thresh) continue;
            int local_bright = local_brightness_rgb565(rgb565, w, h, x, y);
            int seed_thresh = local_bright - SEED_CONTRAST;
            if (seed_bright > seed_thresh) continue;

            uint32_t cx, cy;
            int flood_thresh = local_bright - FLOOD_CONTRAST;
            int span, avg_chroma = 0, perimeter = 0, bbox_w = 0, bbox_h = 0;
            int sz = flood_blob_rgb565(rgb565, w, h, x, y, visited, qx, qy,
                                        w * h, &cx, &cy, &span, &avg_chroma,
                                        &perimeter, &bbox_w, &bbox_h,
                                        flood_thresh, abs_thresh, min_blob, max_span);
            if (sz > 0) {
                int contrast = local_bright - seed_bright;
                if (span > dot_max_span) continue;
                if (contrast < dot_min_contrast) continue;
                if (avg_chroma > CALIB_DOT_MAX_CHROMA) continue;
                float circ = 4.0f * 3.14159f * sz / (float)(perimeter * perimeter);
                if (circ < CALIB_DOT_MIN_CIRCULARITY) continue;
                float aspect = (float)(bbox_w < bbox_h ? bbox_w : bbox_h) /
                               (float)(bbox_w > bbox_h ? bbox_w : bbox_h);
                if (aspect < CALIB_DOT_MIN_ASPECT) continue;
                float fill = (float)sz / (float)(bbox_w * bbox_h);
                if (fill < CALIB_DOT_MIN_FILL) continue;
                blobs[nblobs][0] = (int)cx;
                blobs[nblobs][1] = (int)cy;
                blobs[nblobs][2] = sz;
                blobs[nblobs][3] = span;
                blobs[nblobs][4] = contrast;
                blobs[nblobs][5] = (int)(circ * 100 + 0.5f);
#if CALIB_DEBUG
                printf("[CALIB_RGB565] blob%2d: pixel(%4u,%4u) span=%d count=%d contrast=%d chroma=%d circ=%d aspect=%d fill=%d\n",
                       nblobs, (unsigned)cx, (unsigned)cy, span, sz, contrast,
                       avg_chroma, blobs[nblobs][5],
                       (int)(aspect * 100), (int)(fill * 100));
#endif
                nblobs++;
            }
        }
    }

    free(visited); free(qmem);

    if (nblobs < 4) {
        for (int i = 0; i < nblobs && i < 4; i++) {
            out_ux[i] = (uint32_t)blobs[i][0];
            out_uy[i] = (uint32_t)blobs[i][1];
        }
        return nblobs < 4 ? nblobs : 4;
    }

    for (int i = 0; i < nblobs - 1; i++)
        for (int j = i + 1; j < nblobs; j++)
            if (blobs[j][4] > blobs[i][4]) {
                int t[6]; memcpy(t, blobs[i], sizeof(t));
                memcpy(blobs[i], blobs[j], sizeof(t));
                memcpy(blobs[j], t, sizeof(t));
            }

    // Safety cap — axis-scoring works with many candidates, just prevent runaway
    if (nblobs > 12) nblobs = 12;

    // Assign by axis-distance scoring: pick blob closest to each dot's expected position
    int used[32] = {0};
    float cx = w * 0.5f, cy = h * 0.5f;

    for (int role = 0; role < 4; role++) {
        float best_score = 1e20f;
        int best_i = -1;
        for (int i = 0; i < nblobs; i++) {
            if (used[i]) continue;
            float dx, dy;
            switch (role) {
                case 0: dx = blobs[i][0] - cx; dy = blobs[i][1];         break; // top
                case 1: dx = blobs[i][0];       dy = blobs[i][1] - cy;   break; // left
                case 2: dx = w - blobs[i][0];   dy = blobs[i][1] - cy;   break; // right
                case 3: dx = blobs[i][0] - cx; dy = h - blobs[i][1];    break; // bottom
            }
            if (dx < 0) dx = -dx;
            if (dy < 0) dy = -dy;
            float score = dx + dy;
            if (score < best_score) { best_score = score; best_i = i; }
        }
        if (best_i < 0) break;
        out_ux[role] = (uint32_t)blobs[best_i][0];
        out_uy[role] = (uint32_t)blobs[best_i][1];
        used[best_i] = 1;
    }

    return 4;
}
