#ifndef CONFIG_H
#define CONFIG_H

#include <stdint.h>
#include <stdbool.h>

#define IMG_WIDTH       320
#define IMG_HEIGHT      240
#define MIN_BLOB_SIZE   50

/* Calibration dot detection: contrast-based (no absolute threshold) */
#define SEED_CONTRAST   3      /* Seed pixel contrast threshold (must be this much darker) */
#define FLOOD_CONTRAST  3      /* Flood-fill neighbour contrast threshold (relaxed) */
#define MIN_DARK_BLOB   20     /* Min pixel count (also scaled by w*h/10000 dynamically) */

#define CALIB_DOT_MAX_CHROMA 12 /* Max average (max-min) allowed for dots */

#define CALIB_DEBUG     1

#define WORKSPACE_X_MM  200.0f
#define WORKSPACE_Y_MM  150.0f

#define LINK1_MM        117.5f
#define LINK2_MM        200.0f

#define NUM_SERVOS      4
#define SERVO1_PIN      12
#define SERVO2_PIN      14
#define SERVO3_PIN      33
#define SERVO4_PIN      32

#define SERVO_BASE_MIN      -90
#define SERVO_BASE_MAX      90
#define SERVO_SHOULDER_MIN  -90
#define SERVO_SHOULDER_MAX  20
#define SERVO_ELBOW_MIN     0
#define SERVO_ELBOW_MAX     60
#define SERVO_GRIPPER_MIN   -20
#define SERVO_GRIPPER_MAX   40

#define SERVO_GRIPPER_OPEN  30
#define SERVO_GRIPPER_CLOSE -10

#define HOME_BASE_DEG       0
#define HOME_SHOULDER_DEG   0
#define HOME_ELBOW_DEG      0

#define RED_BIN_PX_X    40
#define RED_BIN_PX_Y    120
#define GREEN_BIN_PX_X  160
#define GREEN_BIN_PX_Y  120
#define BLUE_BIN_PX_X   280
#define BLUE_BIN_PX_Y   120

typedef struct {
    uint8_t h;
    uint8_t s;
    uint8_t v;
} HSV;

typedef enum {
    COLOR_NONE   = 0,
    COLOR_RED    = 1,
    COLOR_GREEN  = 2,
    COLOR_BLUE   = 3,
    COLOR_YELLOW = 4
} Color;

typedef struct {
    bool    found;
    uint32_t centroid_x;
    uint32_t centroid_y;
    Color   color;
} DetectionResult;

typedef struct {
    float base_deg;
    float shoulder_deg;
    float elbow_deg;
} ArmAngles;

typedef enum {
    STATE_SCAN,
    STATE_DETECT,
    STATE_PICK,
    STATE_PLACE,
    STATE_HOME
} RobotState;

#endif
