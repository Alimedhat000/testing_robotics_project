#include "stb_image.h"
#include "stb_image_write.h"
#include <stdint.h>
#include <stdbool.h>
#include "camera.h"

uint8_t* camera_load_image(const char *path, int *w, int *h)
{
    int ch;
    return stbi_load(path, w, h, &ch, 3);
}

bool camera_save_image(const char *path, const uint8_t *rgb, int w, int h)
{
    return stbi_write_png(path, w, h, 3, rgb, w * 3) != 0;
}

void camera_free_image(uint8_t *rgb)
{
    stbi_image_free(rgb);
}
