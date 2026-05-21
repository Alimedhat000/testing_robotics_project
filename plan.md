## Microcontroller
- ESP32-S3 : Has a camera interface, Wi-Fi for debugging, good community support (ESP-IDF is C)

## Camera Module
- OV7670:
    - Set output format to RGB565 (not YUV) — makes color extraction trivial in C
    - Set resolution to QVGA (320×240) or even QQVGA (160×120) to keep frame buffer small

## The Big Picture — 4 States the Robot Loops Through
```
SCAN → DETECT → PICK → PLACE → back to SCAN
```