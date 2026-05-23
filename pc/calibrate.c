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

// ── Brightness = max(r,g,b) ─────────────────────────────────────────
static inline int brightness(const Pixel *p)
{
    int v = p->r > p->g ? (int)p->r : (int)p->g;
    return v > (int)p->b ? v : (int)p->b;
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
// brightness.  Neighbors pass if their brightness < (seed_local - thresh).
// Returns pixel count, or 0 if below min_blob or span > max_span.
static int flood_blob(Pixel *pixels, int w, int h, int sx, int sy,
                      uint8_t visited[], int *qx, int *qy, int max_q,
                      uint32_t *out_x, uint32_t *out_y, int *out_span,
                      int thresh, int min_blob, int max_span)
{
    int head = 0, tail = 0;
    uint32_t sum_x = 0, sum_y = 0;
    int count = 0;
    int min_x = sx, max_x = sx, min_y = sy, max_y = sy;

    qx[tail] = sx; qy[tail] = sy; tail++;
    visited[sy * w + sx] = 1;

    while (head < tail) {
        int x = qx[head], y = qy[head]; head++;
        sum_x += (uint32_t)x;
        sum_y += (uint32_t)y;
        count++;
        if (x < min_x) min_x = x;
        if (x > max_x) max_x = x;
        if (y < min_y) min_y = y;
        if (y > max_y) max_y = y;

        static const int dx[] = {0, 0, -1, 1};
        static const int dy[] = {-1, 1, 0, 0};
        for (int d = 0; d < 4; d++) {
            int nx = x + dx[d], ny = y + dy[d];
            if (nx < 0 || nx >= w || ny < 0 || ny >= h) continue;
            if (visited[ny * w + nx]) continue;
            if (brightness(&pixels[ny * w + nx]) >= thresh)
                continue;
            visited[ny * w + nx] = 1;
            if (tail >= max_q) continue;
            qx[tail] = nx; qy[tail] = ny;
            tail++;
        }
    }

    if (count < min_blob) return 0;
    int span_x = max_x - min_x;
    int span_y = max_y - min_y;
    int span = span_x > span_y ? span_x : span_y;
    if (span > max_span) return 0;
    *out_x = sum_x / count;
    *out_y = sum_y / count;
    *out_span = span;
    return (int)count;
}

// ── Detect 4 calibration dots by contrast-based flood-fill ─────────────
// Finds the 4 darkest small spots relative to their local neighborhood.
// Resolution-independent: span and blob size scale with image dims.
int calibrate_find_dots(Pixel *pixels, int w, int h,
                         uint32_t out_ux[CALIB_N_POINTS],
                         uint32_t out_uy[CALIB_N_POINTS])
{
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

    // Blob storage: x, y, size, span, contrast
    int blobs[32][5];
    int nblobs = 0;

    for (int y = 0; y < h && nblobs < 32; y++) {
        for (int x = 0; x < w && nblobs < 32; x++) {
            if (visited[y * w + x]) continue;

            // Seed must be significantly darker than surroundings
            int seed_bright = brightness(&pixels[y * w + x]);
            int local_bright = local_brightness(pixels, w, h, x, y);
            int seed_thresh = local_bright - SEED_CONTRAST;
            if (seed_bright >= seed_thresh) continue;

            uint32_t cx, cy;
            int flood_thresh = local_bright - FLOOD_CONTRAST;
            int span;
            int sz = flood_blob(pixels, w, h, x, y, visited, qx, qy, w * h,
                                &cx, &cy, &span, flood_thresh, min_blob, max_span);
            if (sz > 0) {
                int contrast = local_bright - seed_bright;
                blobs[nblobs][0] = (int)cx;
                blobs[nblobs][1] = (int)cy;
                blobs[nblobs][2] = sz;
                blobs[nblobs][3] = span;
                blobs[nblobs][4] = contrast;
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
                int t[5]; memcpy(t, blobs[i], sizeof(t));
                memcpy(blobs[i], blobs[j], sizeof(t));
                memcpy(blobs[j], t, sizeof(t));
            }

    // Assign by position, removing each blob after use to prevent duplicates
    int used[4] = {0};

    int yi = 0;
    for (int i = 1; i < 4; i++)
        if (blobs[i][1] < blobs[yi][1]) yi = i;
    out_ux[0] = (uint32_t)blobs[yi][0]; out_uy[0] = (uint32_t)blobs[yi][1];
    used[yi] = 1;

    int xi_min = -1;
    for (int i = 0; i < 4; i++) {
        if (used[i]) continue;
        if (xi_min < 0 || blobs[i][0] < blobs[xi_min][0]) xi_min = i;
    }
    out_ux[1] = (uint32_t)blobs[xi_min][0]; out_uy[1] = (uint32_t)blobs[xi_min][1];
    used[xi_min] = 1;

    int xi_max = -1;
    for (int i = 0; i < 4; i++) {
        if (used[i]) continue;
        if (xi_max < 0 || blobs[i][0] > blobs[xi_max][0]) xi_max = i;
    }
    out_ux[2] = (uint32_t)blobs[xi_max][0]; out_uy[2] = (uint32_t)blobs[xi_max][1];
    used[xi_max] = 1;

    int yi_max = -1;
    for (int i = 0; i < 4; i++) {
        if (used[i]) continue;
        if (yi_max < 0 || blobs[i][1] > blobs[yi_max][1]) yi_max = i;
    }
    out_ux[3] = (uint32_t)blobs[yi_max][0]; out_uy[3] = (uint32_t)blobs[yi_max][1];

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
