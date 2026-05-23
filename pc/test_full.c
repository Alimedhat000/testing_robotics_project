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
 * test_full -- Offline pipeline test with annotated debug image output.
 *
 * Debug images produced in the same directory as the input:
 *   debug_01_seed_contrast.png  -- local-contrast heat-map (bright = high contrast)
 *   debug_02_all_blobs.png      -- all dark blobs found, colour-coded by reject reason
 *   debug_03_seed_pixels.png    -- grayscale + red seed-pixel overlay
 *   debug_04_filtered_blobs.png -- blobs surviving all shape filters with stats labels
 *   debug_05_selected_dots.png  -- the 4 chosen calibration dots with role labels
 *   debug_06_final.png          -- full annotated output (objects + calib crosshairs)
 *
 * Usage:
 *   ./test_full [image.png/jpeg]
 */

/* ═══════════════════════════════════════════════════════════════════
 * Drawing primitives
 * ═══════════════════════════════════════════════════════════════════ */

static inline int brightness(const Pixel *p)
{
    return ((int)p->r + (int)p->g + (int)p->b) / 3;
}

static int local_brightness(const Pixel *pixels, int w, int h, int cx, int cy)
{
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

static Pixel *rgb_to_pixels(const uint8_t *rgb, int w, int h)
{
    Pixel *pix = (Pixel *)malloc((size_t)w * h * sizeof(Pixel));
    if (!pix) return NULL;
    for (int i = 0; i < w * h; i++) {
        pix[i].r = rgb[i*3+0];
        pix[i].g = rgb[i*3+1];
        pix[i].b = rgb[i*3+2];
    }
    return pix;
}

static void pixels_to_rgb(const Pixel *pixels, uint8_t *rgb, int w, int h)
{
    for (int i = 0; i < w * h; i++) {
        rgb[i*3+0] = pixels[i].r;
        rgb[i*3+1] = pixels[i].g;
        rgb[i*3+2] = pixels[i].b;
    }
}

static Pixel *dup_pixels(const Pixel *src, int w, int h)
{
    size_t bytes = (size_t)w * h * sizeof(Pixel);
    Pixel *copy  = (Pixel *)malloc(bytes);
    if (!copy) return NULL;
    memcpy(copy, src, bytes);
    return copy;
}

static inline void set_px(Pixel *pixels, int w, int h,
                           int x, int y, uint8_t r, uint8_t g, uint8_t b)
{
    if (x < 0 || x >= w || y < 0 || y >= h) return;
    pixels[y * w + x].r = r;
    pixels[y * w + x].g = g;
    pixels[y * w + x].b = b;
}

static void draw_line(Pixel *pixels, int w, int h,
                      int x0, int y0, int x1, int y1, int thick,
                      uint8_t r, uint8_t g, uint8_t b)
{
    int dx = abs(x1-x0), sx = x0 < x1 ? 1 : -1;
    int dy = abs(y1-y0), sy = y0 < y1 ? 1 : -1;
    int err = (dx > dy ? dx : -dy) / 2, e2;
    for (;;) {
        for (int ty = -thick; ty <= thick; ty++)
            for (int tx = -thick; tx <= thick; tx++)
                set_px(pixels, w, h, x0+tx, y0+ty, r, g, b);
        if (x0 == x1 && y0 == y1) break;
        e2 = err;
        if (e2 > -dx) { err -= dy; x0 += sx; }
        if (e2 <  dy) { err += dx; y0 += sy; }
    }
}

static void fill_circle(Pixel *pixels, int w, int h,
                        int cx, int cy, int r,
                        uint8_t pr, uint8_t pg, uint8_t pb)
{
    for (int dy = -r; dy <= r; dy++)
        for (int dx = -r; dx <= r; dx++)
            if (dx*dx + dy*dy <= r*r)
                set_px(pixels, w, h, cx+dx, cy+dy, pr, pg, pb);
}

static void draw_ring(Pixel *pixels, int w, int h,
                      int cx, int cy, int r, int thick,
                      uint8_t pr, uint8_t pg, uint8_t pb)
{
    int r_out = r + thick, r_in = r - thick;
    for (int dy = -r_out; dy <= r_out; dy++)
        for (int dx = -r_out; dx <= r_out; dx++) {
            int d2 = dx*dx + dy*dy;
            if (d2 <= r_out*r_out && d2 >= r_in*r_in)
                set_px(pixels, w, h, cx+dx, cy+dy, pr, pg, pb);
        }
}

static void draw_rect(Pixel *pixels, int w, int h,
                      int x0, int y0, int x1, int y1,
                      uint8_t r, uint8_t g, uint8_t b)
{
    for (int x = x0; x <= x1; x++) {
        set_px(pixels, w, h, x, y0, r, g, b);
        set_px(pixels, w, h, x, y1, r, g, b);
    }
    for (int y = y0; y <= y1; y++) {
        set_px(pixels, w, h, x0, y, r, g, b);
        set_px(pixels, w, h, x1, y, r, g, b);
    }
}

static void draw_crosshair(Pixel *pixels, int w, int h,
                           int cx, int cy, int size, int thick,
                           uint8_t r, uint8_t g, uint8_t b)
{
    draw_line(pixels, w, h, cx-size, cy, cx+size, cy, thick, r, g, b);
    draw_line(pixels, w, h, cx, cy-size, cx, cy+size, thick, r, g, b);
}

/* ── 5x7 pixel font ──────────────────────────────────────────────── */
static const uint8_t font5x7[128][7] = {
    ['0'] = {0x0E,0x11,0x13,0x15,0x19,0x11,0x0E},
    ['1'] = {0x04,0x0C,0x04,0x04,0x04,0x04,0x0E},
    ['2'] = {0x0E,0x11,0x01,0x02,0x04,0x08,0x1F},
    ['3'] = {0x1F,0x02,0x04,0x02,0x01,0x11,0x0E},
    ['4'] = {0x02,0x06,0x0A,0x12,0x1F,0x02,0x02},
    ['5'] = {0x1F,0x10,0x1E,0x01,0x01,0x11,0x0E},
    ['6'] = {0x06,0x08,0x10,0x1E,0x11,0x11,0x0E},
    ['7'] = {0x1F,0x01,0x02,0x04,0x04,0x04,0x04},
    ['8'] = {0x0E,0x11,0x11,0x0E,0x11,0x11,0x0E},
    ['9'] = {0x0E,0x11,0x11,0x0F,0x01,0x02,0x0C},
    ['.'] = {0x00,0x00,0x00,0x00,0x00,0x0C,0x0C},
    ['-'] = {0x00,0x00,0x00,0x1F,0x00,0x00,0x00},
    ['+'] = {0x00,0x04,0x04,0x1F,0x04,0x04,0x00},
    ['%'] = {0x18,0x19,0x02,0x04,0x08,0x13,0x03},
    ['A'] = {0x04,0x0A,0x11,0x11,0x1F,0x11,0x11},
    ['B'] = {0x1E,0x11,0x11,0x1E,0x11,0x11,0x1E},
    ['C'] = {0x0E,0x11,0x10,0x10,0x10,0x11,0x0E},
    ['D'] = {0x1E,0x09,0x09,0x09,0x09,0x09,0x1E},
    ['E'] = {0x1F,0x10,0x10,0x1E,0x10,0x10,0x1F},
    ['F'] = {0x1F,0x10,0x10,0x1E,0x10,0x10,0x10},
    ['G'] = {0x0E,0x11,0x10,0x17,0x11,0x11,0x0F},
    ['H'] = {0x11,0x11,0x11,0x1F,0x11,0x11,0x11},
    ['I'] = {0x0E,0x04,0x04,0x04,0x04,0x04,0x0E},
    ['L'] = {0x10,0x10,0x10,0x10,0x10,0x10,0x1F},
    ['N'] = {0x11,0x19,0x15,0x13,0x11,0x11,0x11},
    ['O'] = {0x0E,0x11,0x11,0x11,0x11,0x11,0x0E},
    ['P'] = {0x1E,0x11,0x11,0x1E,0x10,0x10,0x10},
    ['R'] = {0x1E,0x11,0x11,0x1E,0x14,0x12,0x11},
    ['S'] = {0x0F,0x10,0x10,0x0E,0x01,0x01,0x1E},
    ['T'] = {0x1F,0x04,0x04,0x04,0x04,0x04,0x04},
    ['X'] = {0x11,0x11,0x0A,0x04,0x0A,0x11,0x11},
    ['Y'] = {0x11,0x11,0x0A,0x04,0x04,0x04,0x04},
    ['Z'] = {0x1F,0x01,0x02,0x04,0x08,0x10,0x1F},
    ['c'] = {0x00,0x00,0x0E,0x10,0x10,0x11,0x0E},
    ['i'] = {0x04,0x00,0x0C,0x04,0x04,0x04,0x0E},
    ['k'] = {0x10,0x10,0x12,0x14,0x18,0x14,0x12},
    ['n'] = {0x00,0x00,0x16,0x19,0x11,0x11,0x11},
    ['o'] = {0x00,0x00,0x0E,0x11,0x11,0x11,0x0E},
    ['p'] = {0x00,0x00,0x1E,0x11,0x1E,0x10,0x10},
    ['r'] = {0x00,0x00,0x16,0x19,0x10,0x10,0x10},
    ['s'] = {0x00,0x00,0x0E,0x10,0x0E,0x01,0x0E},
    ['t'] = {0x08,0x08,0x1C,0x08,0x08,0x09,0x06},
    ['x'] = {0x00,0x00,0x11,0x0A,0x04,0x0A,0x11},
    [' '] = {0x00,0x00,0x00,0x00,0x00,0x00,0x00},
    [':'] = {0x00,0x0C,0x0C,0x00,0x0C,0x0C,0x00},
    ['/'] = {0x01,0x01,0x02,0x04,0x08,0x10,0x10},
    ['='] = {0x00,0x00,0x1F,0x00,0x1F,0x00,0x00},
};

static void draw_char(Pixel *pixels, int w, int h,
                      int ox, int oy, char ch, int sc,
                      uint8_t r, uint8_t g, uint8_t b)
{
    unsigned char uc = (unsigned char)ch;
    if (uc >= 128) return;
    const uint8_t *bm = font5x7[uc];
    for (int row = 0; row < 7; row++)
        for (int col = 0; col < 5; col++)
            if (bm[row] & (0x10 >> col))
                for (int dy = 0; dy < sc; dy++)
                    for (int dx = 0; dx < sc; dx++)
                        set_px(pixels, w, h,
                               ox + col*sc + dx,
                               oy + row*sc + dy,
                               r, g, b);
}

static void draw_str(Pixel *pixels, int w, int h,
                     int ox, int oy, const char *s, int sc,
                     uint8_t r, uint8_t g, uint8_t b)
{
    int x = ox;
    for (; *s; s++, x += (5 + 1) * sc)
        draw_char(pixels, w, h, x, oy, *s, sc, r, g, b);
}

static void draw_str_shadow(Pixel *pixels, int w, int h,
                            int ox, int oy, const char *s, int sc,
                            uint8_t r, uint8_t g, uint8_t b)
{
    draw_str(pixels, w, h, ox+sc, oy+sc, s, sc, 0, 0, 0);
    draw_str(pixels, w, h, ox,    oy,    s, sc, r, g, b);
}

/* ═══════════════════════════════════════════════════════════════════
 * Blob detection helper (mirrors calibrate.c, for debug only)
 * ═══════════════════════════════════════════════════════════════════ */

typedef struct {
    int cx, cy;           /* centroid */
    int count;            /* pixel area */
    int span;             /* max(bbox_w, bbox_h) */
    int bbox_w, bbox_h;
    int contrast;         /* local_bright - seed_bright */
    int chroma;           /* avg saturation inside blob */
    int perimeter;
    float circ;           /* 4*PI*area/perim^2 */
    float aspect;         /* short/long bbox side */
    float fill;           /* area/(bbox_w*bbox_h) */
    int score;            /* composite quality */
    int reject;           /* 0=pass, 1..6=rejected at stage N */
} BlobInfo;

/*
 * One-shot flood that collects all stats.
 * Returns pixel count, or 0 on early-abort.
 */
static int dbg_flood(Pixel *pixels, int w, int h,
                     int sx, int sy, uint8_t visited[],
                     int *qx, int *qy, int max_q,
                     int *minx, int *maxx, int *miny, int *maxy,
                     int *chroma_out, int *perim_out,
                     int thresh, int abs_thresh, int max_span)
{
    int head = 0, tail = 0, count = 0;
    *minx = sx; *maxx = sx; *miny = sy; *maxy = sy;
    int chroma_sum = 0;

    qx[tail] = sx; qy[tail] = sy; tail++;
    visited[sy * w + sx] = 1;

    static const int ddx[] = {0, 0, -1, 1};
    static const int ddy[] = {-1, 1, 0, 0};

    while (head < tail) {
        int x = qx[head], y = qy[head]; head++;
        count++;
        { Pixel *p = &pixels[y*w+x];
          int maxc = p->r > p->g ? p->r : p->g;
          int minc = p->r < p->g ? p->r : p->g;
          if (p->b > maxc) maxc = p->b; if (p->b < minc) minc = p->b;
          chroma_sum += maxc - minc; }
        if (x < *minx) *minx = x; if (x > *maxx) *maxx = x;
        if (y < *miny) *miny = y; if (y > *maxy) *maxy = y;
        if ((*maxx - *minx) > max_span || (*maxy - *miny) > max_span) {
            for (int i = 0; i < tail; i++) visited[qy[i]*w+qx[i]] = 0;
            visited[sy*w+sx] = 1;
            return 0;
        }
        for (int d = 0; d < 4; d++) {
            int nx = x+ddx[d], ny = y+ddy[d];
            if (nx < 0 || nx >= w || ny < 0 || ny >= h) continue;
            if (visited[ny*w+nx]) continue;
            int nb = brightness(&pixels[ny*w+nx]);
            if (nb >= thresh || nb >= abs_thresh) continue;
            visited[ny*w+nx] = 1;
            if (tail < max_q) { qx[tail] = nx; qy[tail] = ny; tail++; }
        }
    }

    *chroma_out = count ? chroma_sum / count : 0;

    /* perimeter */
    int perim = 0;
    for (int i = 0; i < tail; i++) {
        int px = qx[i], py = qy[i];
        for (int d = 0; d < 4; d++) {
            int nx = px+ddx[d], ny = py+ddy[d];
            if (nx < 0 || nx >= w || ny < 0 || ny >= h) { perim++; break; }
            int nb = brightness(&pixels[ny*w+nx]);
            if (nb >= thresh || nb >= abs_thresh) { perim++; break; }
            if (!visited[ny*w+nx])                { perim++; break; }
        }
    }
    *perim_out = perim;
    return count;
}

/* Collect all blobs into BlobInfo array */
static int collect_blobs(Pixel *pixels, int w, int h, int abs_thresh,
                         BlobInfo *out, int max_out)
{
    uint8_t *visited = (uint8_t *)calloc((size_t)w * h, 1);
    int     *qmem    = (int *)malloc((size_t)w * h * 2 * sizeof(int));
    if (!visited || !qmem) { free(visited); free(qmem); return 0; }
    int *qx = qmem, *qy = qmem + (size_t)w * h;

    int min_blob     = MIN_DARK_BLOB > w * h / 10000
                       ? MIN_DARK_BLOB : w * h / 10000;
    int max_span     = (w > h ? w : h) / 8;
    int dot_max_span = max_span / 3;
    if (dot_max_span < 8) dot_max_span = 8;

    int n = 0;
    for (int y = 0; y < h && n < max_out; y++) {
        for (int x = 0; x < w && n < max_out; x++) {
            if (visited[y * w + x]) continue;
            int sb = brightness(&pixels[y * w + x]);
            if (sb > abs_thresh) continue;
            int lb = local_brightness(pixels, w, h, x, y);
            if (sb > lb - SEED_CONTRAST) continue;

            int minx, maxx, miny, maxy, chroma = 0, perim = 0;
            int count = dbg_flood(pixels, w, h, x, y, visited, qx, qy, w * h,
                                  &minx, &maxx, &miny, &maxy, &chroma, &perim,
                                  lb - FLOOD_CONTRAST, abs_thresh, max_span);
            if (count < min_blob) continue;

            BlobInfo *b = &out[n];
            b->bbox_w   = maxx - minx + 1;
            b->bbox_h   = maxy - miny + 1;
            b->cx       = (minx + maxx) / 2;
            b->cy       = (miny + maxy) / 2;
            b->count    = count;
            b->span     = b->bbox_w > b->bbox_h ? b->bbox_w : b->bbox_h;
            b->contrast = lb - sb;
            b->chroma   = chroma;
            b->perimeter= perim;
            b->circ     = 4.0f * 3.14159f * count /
                          (float)(perim * perim + 1);
            b->aspect   = (float)(b->bbox_w < b->bbox_h ? b->bbox_w : b->bbox_h) /
                          (float)(b->bbox_w > b->bbox_h ? b->bbox_w : b->bbox_h + 1);
            b->fill     = (float)count / (float)(b->bbox_w * b->bbox_h + 1);
            b->score    = (int)(b->contrast * b->circ * b->circ *
                                b->aspect * b->fill * 10000 + 0.5f);

            /* classify rejection stage */
            b->reject = 0;
            if (b->span     > dot_max_span)          b->reject = 1;
            else if (b->contrast < SEED_CONTRAST)    b->reject = 2;
            else if (b->chroma > CALIB_DOT_MAX_CHROMA) b->reject = 3;
            else if (b->circ   < CALIB_DOT_MIN_CIRCULARITY) b->reject = 4;
            else if (b->aspect < CALIB_DOT_MIN_ASPECT)      b->reject = 5;
            else if (b->fill   < CALIB_DOT_MIN_FILL)        b->reject = 6;

            n++;
        }
    }
    free(visited); free(qmem);
    return n;
}

/* ═══════════════════════════════════════════════════════════════════
 * Debug image savers
 * ═══════════════════════════════════════════════════════════════════ */

/* debug_01: local-contrast heat-map */
static void save_contrast_map(const Pixel *pixels, int w, int h,
                              uint8_t *rgb, const char *path)
{
    for (int y = 0; y < h; y++)
        for (int x = 0; x < w; x++) {
            int i  = y * w + x;
            int lb = local_brightness(pixels, w, h, x, y);
            int c  = lb - brightness(&pixels[i]);
            if (c < 0) c = 0; if (c > 255) c = 255;
            rgb[i*3+0] = (uint8_t)c;
            rgb[i*3+1] = (uint8_t)c;
            rgb[i*3+2] = (uint8_t)c;
        }
    camera_save_image(path, rgb, w, h);
}

/* Reject stage colours:
 *   0 = pass       -> bright green
 *   1 = too large  -> orange
 *   2 = low contrast-> yellow
 *   3 = coloured   -> magenta
 *   4 = not round  -> red
 *   5 = elongated  -> cyan
 *   6 = sparse     -> blue
 */
static const uint8_t REJECT_COLOR[7][3] = {
    { 80, 255,  80},  /* 0 pass        */
    {255, 140,   0},  /* 1 too large   */
    {255, 255,   0},  /* 2 low contrast*/
    {255,   0, 255},  /* 3 coloured    */
    {255,  60,  60},  /* 4 not round   */
    {  0, 220, 220},  /* 5 elongated   */
    { 60,  60, 255},  /* 6 sparse      */
};
static const char *REJECT_NAME[7] = {
    "PASS","LARGE","CONTRAST","CHROMA","CIRC","ASPECT","FILL"
};

/* debug_02: all blobs with bounding boxes, colour-coded by reject stage */
static void save_all_blobs(Pixel *pixels, int w, int h,
                           const BlobInfo *blobs, int n,
                           uint8_t *rgb, const char *path)
{
    Pixel *dbg = dup_pixels(pixels, w, h);
    if (!dbg) return;

    for (int i = 0; i < n; i++) {
        const BlobInfo *b = &blobs[i];
        int r = b->reject < 7 ? b->reject : 6;
        uint8_t cr = REJECT_COLOR[r][0];
        uint8_t cg = REJECT_COLOR[r][1];
        uint8_t cb = REJECT_COLOR[r][2];

        int x0 = b->cx - b->bbox_w/2;
        int y0 = b->cy - b->bbox_h/2;
        int x1 = x0 + b->bbox_w - 1;
        int y1 = y0 + b->bbox_h - 1;
        draw_rect(dbg, w, h, x0, y0, x1, y1, cr, cg, cb);

        fill_circle(dbg, w, h, b->cx, b->cy, 2, cr, cg, cb);

        char label[64];
        snprintf(label, sizeof(label), "%s %d", REJECT_NAME[r], b->score);
        draw_str_shadow(dbg, w, h, x0, y0 - 9, label, 1, cr, cg, cb);
    }

    /* Legend in top-left */
    {
        int lx = 4, ly = 4;
        draw_str_shadow(dbg, w, h, lx, ly, "ALL BLOBS", 1, 255,255,255);
        ly += 10;
        for (int s = 0; s < 7; s++) {
            fill_circle(dbg, w, h, lx+3, ly+3, 3,
                        REJECT_COLOR[s][0], REJECT_COLOR[s][1], REJECT_COLOR[s][2]);
            draw_str_shadow(dbg, w, h, lx+9, ly, REJECT_NAME[s], 1,
                            REJECT_COLOR[s][0], REJECT_COLOR[s][1], REJECT_COLOR[s][2]);
            ly += 10;
        }
        char nbuf[16]; snprintf(nbuf, sizeof(nbuf), "n=%d", n);
        draw_str_shadow(dbg, w, h, lx, ly+2, nbuf, 1, 200,200,200);
    }

    pixels_to_rgb(dbg, rgb, w, h);
    camera_save_image(path, rgb, w, h);
    free(dbg);
}

/* debug_03: seed pixels overlay on grayscale */
static void save_seed_map(const Pixel *pixels, int w, int h,
                          int abs_thresh, uint8_t *rgb, const char *path)
{
    for (int i = 0; i < w * h; i++) {
        int b = brightness(&pixels[i]);
        rgb[i*3+0] = rgb[i*3+1] = rgb[i*3+2] = (uint8_t)b;
    }
    int nseeds = 0;
    for (int y = 2; y < h-2; y++)
        for (int x = 2; x < w-2; x++) {
            int sb = brightness(&pixels[y*w+x]);
            if (sb > abs_thresh) continue;
            int lb = local_brightness(pixels, w, h, x, y);
            if (sb > lb - SEED_CONTRAST) continue;
            rgb[(y*w+x)*3+0] = 255;
            rgb[(y*w+x)*3+1] = 0;
            rgb[(y*w+x)*3+2] = 0;
            nseeds++;
        }
    printf("  Seed pixels: %d\n", nseeds);
    camera_save_image(path, rgb, w, h);
}

/* debug_04: only passing blobs (reject==0), with detailed stats label */
static void save_filtered_blobs(Pixel *pixels, int w, int h,
                                const BlobInfo *blobs, int n,
                                uint8_t *rgb, const char *path)
{
    Pixel *dbg = dup_pixels(pixels, w, h);
    if (!dbg) return;

    int npassed = 0;
    for (int i = 0; i < n; i++) {
        const BlobInfo *b = &blobs[i];
        if (b->reject != 0) continue;
        npassed++;

        int x0 = b->cx - b->bbox_w/2;
        int y0 = b->cy - b->bbox_h/2;
        int x1 = x0 + b->bbox_w - 1;
        int y1 = y0 + b->bbox_h - 1;

        /* Thick green bounding box (3-pixel border) */
        for (int t = -1; t <= 1; t++) {
            draw_rect(dbg, w, h, x0+t, y0+t, x1-t, y1-t, 0, 255, 0);
        }

        draw_crosshair(dbg, w, h, b->cx, b->cy, 6, 0, 0, 200, 0);

        /* Stats label below the box */
        char line1[64], line2[64];
        snprintf(line1, sizeof(line1),
                 "cnt%d sp%d ct%d", b->count, b->span, b->contrast);
        snprintf(line2, sizeof(line2),
                 "ci%d as%d fi%d sc%d",
                 (int)(b->circ*100), (int)(b->aspect*100),
                 (int)(b->fill*100), b->score);
        draw_str_shadow(dbg, w, h, x0, y1+2,  line1, 1, 100,255,100);
        draw_str_shadow(dbg, w, h, x0, y1+12, line2, 1, 100,255,100);
    }

    char hdr[32]; snprintf(hdr, sizeof(hdr), "FILTERED: %d pass", npassed);
    draw_str_shadow(dbg, w, h, 4, 4, hdr, 1, 255,255,255);

    pixels_to_rgb(dbg, rgb, w, h);
    camera_save_image(path, rgb, w, h);
    free(dbg);
}

/* debug_05: the 4 selected calibration dots with role labels & stats */
static void save_selected_dots(Pixel *pixels, int w, int h,
                               const uint32_t du[4], const uint32_t dv[4],
                               uint8_t *rgb, const char *path)
{
    Pixel *dbg = dup_pixels(pixels, w, h);
    if (!dbg) return;

    /* Role: 0=top, 1=left, 2=right, 3=bottom */
    static const uint8_t COL[4][3] = {
        {  0, 255, 255},  /* top    = cyan    */
        {255,  80,  80},  /* left   = red     */
        { 80, 160, 255},  /* right  = blue    */
        {255, 220,   0},  /* bottom = yellow  */
    };
    static const char *ROLE[4] = {"TOP","LEFT","RGHT","BOT "};

    for (int i = 0; i < 4; i++) {
        int cx = (int)du[i], cy = (int)dv[i];
        uint8_t r = COL[i][0], g = COL[i][1], b = COL[i][2];

        /* Large crosshair */
        draw_crosshair(dbg, w, h, cx, cy, 14, 1, r, g, b);

        /* Double ring */
        draw_ring(dbg, w, h, cx, cy, 16, 2, r, g, b);
        draw_ring(dbg, w, h, cx, cy, 10, 1, 0, 0, 0);

        /* Pixel/role label */
        char label[32];
        snprintf(label, sizeof(label), "%s (%u,%u)", ROLE[i], du[i], dv[i]);
        draw_str_shadow(dbg, w, h, cx + 18, cy - 3, label, 1, r, g, b);

        /* Robot-mm coordinate target */
        char mmstr[32];
        snprintf(mmstr, sizeof(mmstr), "mm(%+.0f,%+.0f)",
                 CALIB_MM[i][0], CALIB_MM[i][1]);
        draw_str_shadow(dbg, w, h, cx + 18, cy + 8, mmstr, 1, r, g, b);
    }

    /* Lines connecting them in calibration order: top, right, bottom, left */
    int order[5] = {0, 2, 3, 1, 0};
    for (int s = 0; s < 4; s++) {
        int a = order[s], b2 = order[s+1];
        draw_line(dbg, w, h,
                  (int)du[a], (int)dv[a], (int)du[b2], (int)dv[b2],
                  0, 180, 180, 180);
    }

    draw_str_shadow(dbg, w, h, 4, 4, "SELECTED DOTS", 1, 255,255,255);

    pixels_to_rgb(dbg, rgb, w, h);
    camera_save_image(path, rgb, w, h);
    free(dbg);
}

/* ═══════════════════════════════════════════════════════════════════
 * Homography apply
 * ═══════════════════════════════════════════════════════════════════ */
static void apply_h(const float H[3][3], uint32_t u, uint32_t v,
                    float *x, float *y)
{
    float ww = H[2][0]*(float)u + H[2][1]*(float)v + 1.0f;
    *x = (H[0][0]*(float)u + H[0][1]*(float)v + H[0][2]) / ww;
    *y = (H[1][0]*(float)u + H[1][1]*(float)v + H[1][2]) / ww;
}

/* ── Output dir from file path ───────────────────────────────────── */
static void output_dir_from_path(const char *path, char *out, size_t sz)
{
    const char *slash = strrchr(path, '/');
    if (!slash) { snprintf(out, sz, "."); return; }
    size_t len = (size_t)(slash - path);
    if (len >= sz) len = sz - 1;
    memcpy(out, path, len);
    out[len] = '\0';
}

/* ═══════════════════════════════════════════════════════════════════
 * main
 * ═══════════════════════════════════════════════════════════════════ */
int main(int argc, char **argv)
{
    const char *path = (argc > 1) ? argv[1] : "pc/test_images/sample_1.jpeg";

    int w = 0, h = 0;
    uint8_t *rgb = camera_load_image(path, &w, &h);
    if (!rgb) { fprintf(stderr, "Error: cannot load %s\n", path); return 1; }
    printf("Loaded: %s  (%d x %d)\n\n", path, w, h);

    Pixel *pixels = rgb_to_pixels(rgb, w, h);
    if (!pixels) {
        fprintf(stderr, "Error: malloc failed\n");
        camera_free_image(rgb);
        return 1;
    }

    /* ── Brightness range ─────────────────────────────────────────── */
    int min_bright = 255, max_bright = 0;
    for (int i = 0; i < w * h; i++) {
        int b = brightness(&pixels[i]);
        if (b < min_bright) min_bright = b;
        if (b > max_bright) max_bright = b;
    }
    int abs_thresh = min_bright + (max_bright - min_bright) / CALIB_ABS_THRESH_DIV;
    printf("Brightness range: %d..%d  abs_thresh=%d\n\n",
           min_bright, max_bright, abs_thresh);

    char out_dir[512];
    output_dir_from_path(path, out_dir, sizeof(out_dir));

    char dbg_path[600];

    /* ── debug_01: contrast heat-map ─────────────────────────────── */
    snprintf(dbg_path, sizeof(dbg_path), "%s/debug_01_seed_contrast.png", out_dir);
    save_contrast_map(pixels, w, h, rgb, dbg_path);
    printf("Saved %s\n", dbg_path);

    /* ── Collect all blobs (once, shared across debug images) ─────── */
    BlobInfo blobs[64];
    int nblobs = collect_blobs(pixels, w, h, abs_thresh, blobs, 64);
    printf("Raw blobs detected: %d\n", nblobs);

    int npassed = 0;
    for (int i = 0; i < nblobs; i++)
        if (blobs[i].reject == 0) npassed++;
    printf("Blobs passing all filters: %d\n\n", npassed);

    /* ── debug_02: all blobs colour-coded by reject stage ─────────── */
    snprintf(dbg_path, sizeof(dbg_path), "%s/debug_02_all_blobs.png", out_dir);
    save_all_blobs(pixels, w, h, blobs, nblobs, rgb, dbg_path);
    printf("Saved %s\n", dbg_path);

    /* ── debug_03: seed pixel map ─────────────────────────────────── */
    snprintf(dbg_path, sizeof(dbg_path), "%s/debug_03_seed_pixels.png", out_dir);
    save_seed_map(pixels, w, h, abs_thresh, rgb, dbg_path);
    printf("Saved %s\n", dbg_path);

    /* ── debug_04: filtered blobs with stats ─────────────────────── */
    snprintf(dbg_path, sizeof(dbg_path), "%s/debug_04_filtered_blobs.png", out_dir);
    save_filtered_blobs(pixels, w, h, blobs, nblobs, rgb, dbg_path);
    printf("Saved %s\n", dbg_path);

    /* ── RGB565 conversion ───────────────────────────────────────── */
    uint16_t *rgb565 = (uint16_t *)malloc((size_t)w * h * sizeof(uint16_t));
    if (rgb565)
        for (int i = 0; i < w * h; i++)
            rgb565[i] = (uint16_t)(((pixels[i].r >> 3) << 11) |
                                   ((pixels[i].g >> 2) << 5)  |
                                    (pixels[i].b >> 3));

    /* ═════════════════════════════════════════════════════════════
     * Step 1 -- Calibration
     * ═════════════════════════════════════════════════════════════ */
    printf("\n=== Step 1: Calibration (RGB888) ===\n");
    uint32_t dot_ux[4], dot_uy[4];
    int ndots = calibrate_find_dots(pixels, w, h, dot_ux, dot_uy);
    printf("  Detected %d/4 calibration dots\n", ndots);

    printf("\n=== Step 1b: Calibration (RGB565) ===\n");
    if (rgb565) {
        uint32_t dot_ux5[4], dot_uy5[4];
        int ndots5 = calibrate_find_dots_rgb565(rgb565, w, h, dot_ux5, dot_uy5);
        printf("  Detected %d/4 calibration dots\n", ndots5);
        if (ndots == ndots5) {
            int match = 1;
            for (int i = 0; i < ndots && i < 4; i++)
                if (dot_ux[i] != dot_ux5[i] || dot_uy[i] != dot_uy5[i]) match = 0;
            printf("  RGB888 vs RGB565: %s\n", match ? "MATCH" : "DIFFER");
        } else {
            printf("  RGB888 vs RGB565: DIFFER (%d vs %d dots)\n", ndots, ndots5);
        }
    }

    /* ── debug_05: selected dots ─────────────────────────────────── */
    if (ndots >= 4) {
        snprintf(dbg_path, sizeof(dbg_path),
                 "%s/debug_05_selected_dots.png", out_dir);
        save_selected_dots(pixels, w, h, dot_ux, dot_uy, rgb, dbg_path);
        printf("Saved %s\n", dbg_path);
    }

    /* ── Homography ──────────────────────────────────────────────── */
    float H[3][3] = {{0}};
    float rms = 0.0f;
    int   have_h = 0;

    if (ndots >= 4) {
        calibrate_solve(dot_ux, dot_uy, H, &rms);
        have_h = 1;
        printf("\n  Homography matrix H:\n");
        printf("  [%9.5f  %9.5f  %9.5f]\n", H[0][0], H[0][1], H[0][2]);
        printf("  [%9.5f  %9.5f  %9.5f]\n", H[1][0], H[1][1], H[1][2]);
        printf("  [%9.5f  %9.5f  %9.5f]\n", H[2][0], H[2][1], H[2][2]);
        printf("  RMS reprojection error: %.2f mm\n\n", rms);

        /* Green crosshairs on calibration dots in final image */
        for (int i = 0; i < 4; i++)
            draw_crosshair(pixels, w, h,
                           (int)dot_ux[i], (int)dot_uy[i],
                           10, 1, 0, 255, 0);
    } else {
        printf("  ERROR: need 4 dots -- only found %d.\n", ndots);
    }

    /* ═════════════════════════════════════════════════════════════
     * Step 2 -- Object detection
     * ═════════════════════════════════════════════════════════════ */
    printf("=== Step 2: Object Detection ===\n");
    static const Color  colors[] = {COLOR_RED, COLOR_GREEN, COLOR_BLUE, COLOR_YELLOW};
    static const char  *names[]  = {"RED", "GREEN", "BLUE", "YELLOW"};
    int obj_count = 0;

    for (int i = 0; i < 4; i++) {
        DetectionResult r = color_detect_scan_pixels(pixels, w, h, colors[i]);
        if (!r.found) { printf("  %s: (none)\n", names[i]); continue; }
        obj_count++;

        if (have_h) {
            float mm_x, mm_y;
            apply_h(H, r.centroid_x, r.centroid_y, &mm_x, &mm_y);
            ArmAngles ang = kinematics_solve_ik(mm_x, mm_y);
            printf("  %s:  pixel(%3u, %3u)  ->  mm(%+7.1f, %+7.1f)\n",
                   names[i], r.centroid_x, r.centroid_y, mm_x, mm_y);
            printf("       IK: base=%6.1f deg  shoulder=%6.1f deg  elbow=%6.1f deg\n",
                   ang.base_deg, ang.shoulder_deg, ang.elbow_deg);
        } else {
            printf("  %s:  pixel(%3u, %3u)\n",
                   names[i], r.centroid_x, r.centroid_y);
        }
        draw_centroid(pixels, w, h,
                      (int)r.centroid_x, (int)r.centroid_y, colors[i]);
    }
    if (obj_count == 0) printf("  No objects detected.\n");

    /* ── debug_06: final annotated image ─────────────────────────── */
    snprintf(dbg_path, sizeof(dbg_path), "%s/debug_06_final.png", out_dir);

    /* RMS overlay on final image */
    if (have_h) {
        char rms_str[32];
        snprintf(rms_str, sizeof(rms_str), "RMS %.1fmm", rms);
        draw_str_shadow(pixels, w, h, 6, 6, rms_str, 2, 0, 255, 0);
    }

    pixels_to_rgb(pixels, rgb, w, h);
    camera_save_image(dbg_path, rgb, w, h);
    printf("Saved %s\n", dbg_path);

    char out_path[600];
    snprintf(out_path, sizeof(out_path), "%s/output.png", out_dir);
    if (camera_save_image(out_path, rgb, w, h))
        printf("Annotated image saved: %s\n", out_path);
    else
        fprintf(stderr, "Warning: failed to save %s\n", out_path);

    /* ── Cleanup ─────────────────────────────────────────────────── */
    free(rgb565);
    free(pixels);
    camera_free_image(rgb);
    printf("Done.\n");
    return 0;
}
