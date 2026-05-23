#pragma once
#include <stdint.h>
#include "config.h"

typedef struct { uint8_t r, g, b; } Pixel;

extern Pixel* imageMatrix;

#ifdef __cplusplus
inline Pixel& imageAt(Pixel* mat, int row, int col) {
    return mat[row * IMG_WIDTH + col];
}
#endif