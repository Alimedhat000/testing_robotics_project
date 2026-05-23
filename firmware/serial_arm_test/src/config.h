#ifndef CONFIG_H
#define CONFIG_H

#include <stdbool.h>
#include <stdint.h>

#define IMG_WIDTH 320
#define IMG_HEIGHT 240
#define MIN_BLOB_SIZE 50

#define SEED_CONTRAST 2
#define FLOOD_CONTRAST 1
#define LOCAL_BRIGHT_RADIUS 5
#define MIN_DARK_BLOB 4
#define CALIB_ABS_THRESH_DIV 3
#define CALIB_DOT_MAX_CHROMA 20
#define CALIB_DOT_MIN_CIRCULARITY 0.40f
#define CALIB_DOT_MIN_ASPECT 0.40f
#define CALIB_DOT_MIN_FILL 0.30f
#define CALIB_DEBUG 1

#define WORKSPACE_X_MM 200.0f
#define WORKSPACE_Y_MM 150.0f

/* ── Arm dimensions ── */
#define LINK1_MM 120.0f             /* Shoulder → elbow (12 cm) */
#define LINK2_MM 200.0f             /* Elbow → wrist (15.78 cm) */
#define GRIPPER_LENGTH_MM 90.0f     /* Gripper extension (9 cm) */
#define SHOULDER_Z_OFFSET_MM 100.0f /* Base(59) + shoulder(41) ≈ 100 mm */

/* ── Servo pins ── */
#define NUM_SERVOS 4
#define SERVO1_PIN 12
#define SERVO2_PIN 14
#define SERVO3_PIN 33
#define SERVO4_PIN 32

/* ── Angle limits ── */
#define SERVO_BASE_MIN -90
#define SERVO_BASE_MAX 90
#define SERVO_SHOULDER_MIN 0
#define SERVO_SHOULDER_MAX 90
#define SERVO_ELBOW_MIN -90
#define SERVO_ELBOW_MAX 0
#define SERVO_GRIPPER_MIN -20
#define SERVO_GRIPPER_MAX 40

#define SERVO_GRIPPER_OPEN -10
#define SERVO_GRIPPER_CLOSE 30

#define HOME_BASE_DEG 0
#define HOME_SHOULDER_DEG 0
#define HOME_ELBOW_DEG 0

#define SHOULDER_HOVER_DEG 10
#define SHOULDER_PICK_DEG -30
#define SHOULDER_DROP_DEG -20

#define RED_BIN_PX_X 40
#define RED_BIN_PX_Y 120
#define GREEN_BIN_PX_X 160
#define GREEN_BIN_PX_Y 120
#define BLUE_BIN_PX_X 280
#define BLUE_BIN_PX_Y 120

/* ── Shared types ── */
typedef struct {
  uint8_t h, s, v;
} HSV;

typedef enum {
  COLOR_NONE = 0,
  COLOR_RED = 1,
  COLOR_GREEN = 2,
  COLOR_BLUE = 3,
  COLOR_YELLOW = 4
} Color;

typedef struct {
  bool found;
  uint32_t centroid_x;
  uint32_t centroid_y;
  Color color;
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
