#include <stdio.h>
#include <math.h>
#include <string.h>
#include "calibrate.h"
#include "config.h"
#include "types.h"

const float CALIB_MM[CALIB_N_POINTS][2] = {
    {-100.0f,  100.0f},
    { 100.0f,  100.0f},
    {-100.0f, -100.0f},
    { 100.0f, -100.0f},
};

static bool find_dark_centroid(Pixel *pixels, int w, int h,
                                int x0, int y0, int x1, int y1,
                                uint32_t *out_x, uint32_t *out_y)
{
    (void)h;
    uint32_t sum_x = 0, sum_y = 0, count = 0;
    for (int y = y0; y < y1; y++) {
        for (int x = x0; x < x1; x++) {
            Pixel p = pixels[y * w + x];
            if (p.r < DARK_THRESHOLD && p.g < DARK_THRESHOLD && p.b < DARK_THRESHOLD) {
                sum_x += (uint32_t)x;
                sum_y += (uint32_t)y;
                count++;
            }
        }
    }
    if (count < MIN_DARK_BLOB)
        return false;
    *out_x = sum_x / count;
    *out_y = sum_y / count;
    return true;
}

int calibrate_find_dots(Pixel *pixels, int w, int h,
                         uint32_t out_ux[CALIB_N_POINTS],
                         uint32_t out_uy[CALIB_N_POINTS])
{
    int hw = w / 2, hh = h / 2;
    int found = 0;

    if (find_dark_centroid(pixels, w, h, 0, 0, hw, hh,
                            &out_ux[0], &out_uy[0])) found++;
    if (find_dark_centroid(pixels, w, h, hw, 0, w, hh,
                            &out_ux[1], &out_uy[1])) found++;
    if (find_dark_centroid(pixels, w, h, 0, hh, hw, h,
                            &out_ux[2], &out_uy[2])) found++;
    if (find_dark_centroid(pixels, w, h, hw, hh, w, h,
                            &out_ux[3], &out_uy[3])) found++;

    return found;
}

#define N 8

static void gauss_elim(float A[N][N], float b[N])
{
    for (int col = 0; col < N; col++) {
        int best = col;
        for (int row = col + 1; row < N; row++)
            if (fabsf(A[row][col]) > fabsf(A[best][col]))
                best = row;
        if (best != col) {
            for (int j = col; j < N; j++) {
                float t = A[col][j]; A[col][j] = A[best][j]; A[best][j] = t;
            }
            float t = b[col]; b[col] = b[best]; b[best] = t;
        }

        float piv = A[col][col];
        if (fabsf(piv) < 1e-12f)
            continue;

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
                      const uint32_t uy[CALIB_N_POINTS],
                      float H[3][3], float *rms)
{
    float A[N][N] = {0};
    float b[N]    = {0};

    for (int i = 0; i < 4; i++) {
        float u = (float)ux[i];
        float v = (float)uy[i];
        float x = CALIB_MM[i][0];
        float y = CALIB_MM[i][1];

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
        float dx = px - CALIB_MM[i][0];
        float dy = py - CALIB_MM[i][1];
        float err = sqrtf(dx*dx + dy*dy);
        printf("  Dot %d: pixel(%3u,%3u) → mm(%+.0f,%+.0f)  reconstructed(%7.1f,%7.1f)  error %.1f mm\n",
               i, ux[i], uy[i], CALIB_MM[i][0], CALIB_MM[i][1], px, py, err);
        sum += err * err;
    }
    *rms = sqrtf(sum / 4.0f);
    return true;
}

void calibrate_reproject(const float H[3][3],
                          const uint32_t ux[CALIB_N_POINTS],
                          const uint32_t uy[CALIB_N_POINTS])
{
    float sum = 0;
    for (int i = 0; i < 4; i++) {
        float u = (float)ux[i], v = (float)uy[i];
        float w = H[2][0]*u + H[2][1]*v + 1.0f;
        float px = (H[0][0]*u + H[0][1]*v + H[0][2]) / w;
        float py = (H[1][0]*u + H[1][1]*v + H[1][2]) / w;
        float dx = px - CALIB_MM[i][0];
        float dy = py - CALIB_MM[i][1];
        float err = sqrtf(dx*dx + dy*dy);
        printf("  Dot %d: pixel(%3u,%3u) → mm(%+.0f,%+.0f)  reconstructed(%7.1f,%7.1f)  error %.1f mm\n",
               i, ux[i], uy[i], CALIB_MM[i][0], CALIB_MM[i][1], px, py, err);
        sum += err * err;
    }
    printf("  RMS reprojection error: %.1f mm\n", sqrtf(sum / 4.0f));
}
