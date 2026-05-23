#ifndef CAMERA_H
#define CAMERA_H

#include <stdint.h>
#include <stdbool.h>

uint8_t* camera_load_image(const char *path, int *w, int *h);
bool camera_save_image(const char *path, const uint8_t *rgb, int w, int h);
void camera_free_image(uint8_t *rgb);

#endif
