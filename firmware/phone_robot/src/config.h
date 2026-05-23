/* =============================================================
 * config.h
 * Global configuration constants and shared data types.
 * All tunable parameters live here, edit this file to
 * calibrate the system for your physical setup.
 * =============================================================
 */

#ifndef CONFIG_H
#define CONFIG_H

#include <stdbool.h>
#include <stdint.h>

/* ------------------------------------------------------------- */
/*  Camera / Image                                               */
/* ------------------------------------------------------------- */

#define IMG_WIDTH 320    /* OV7670 QVGA width  (pixels) */
#define IMG_HEIGHT 240   /* OV7670 QVGA height (pixels) */
#define MIN_BLOB_SIZE 50 /* Minimum pixel count to treat as a real object */

/* ------------------------------------------------------------- */
/*  Calibration — contrast-based dot detection                   */
/*                                                               */
/*  AT 320×240 THE DOTS ARE ONLY ~3–6 px WIDE.                  */
/*  All thresholds below are tuned for this resolution.          */
/*  If you switch to a higher resolution (e.g. 640×480) you     */
/*  will need to re-tune these.                                  */
/* ------------------------------------------------------------- */

/*
 * SEED_CONTRAST — how much darker a seed pixel must be than
 * its local neighbourhood.
 *
 * FLOOD_CONTRAST — neighbour threshold for flood-fill growth.
 */
#define SEED_CONTRAST   2
#define FLOOD_CONTRAST  1

/*
 * LOCAL_BRIGHT_RADIUS — half-width of local brightness window.
 * At 320×240 a dot is 3–6 px, so 5 → 11×11 window reaches well
 * outside the dot and gives an accurate "paper white" average.
 * Must be wired into calibrate.c's local_brightness() functions.
 */
#define LOCAL_BRIGHT_RADIUS 5

/*
 * MIN_DARK_BLOB — minimum pixel count for a blob to survive.
 * At 320×240 a ~10mm dot ≈ 3–6 px diameter ≈ 7–28 px area.
 * w*h/10000 gives 7 as the dynamic floor; this is the absolute min.
 */
#define MIN_DARK_BLOB 4

/*
 * CALIB_ABS_THRESH_DIV — brightness global-gate divisor.
 * abs_thresh = min_bright + (range / div).
 */
#define CALIB_ABS_THRESH_DIV 3

/* Shape filters relaxed for tiny blobs at low resolution */
#define CALIB_DOT_MAX_CHROMA       20
#define CALIB_DOT_MIN_CIRCULARITY  0.40f
#define CALIB_DOT_MIN_ASPECT       0.40f
#define CALIB_DOT_MIN_FILL         0.30f

#define CALIB_DEBUG 1

/* ------------------------------------------------------------- */
/*  Workspace geometry                                           */
/*  CALIBRATE: measure your robot's reachable table area in mm   */
/* ------------------------------------------------------------- */

#define WORKSPACE_X_MM 200.0f /* Total workspace width  (mm), <-PLACEHOLDER->  \
                               */
#define WORKSPACE_Y_MM 150.0f /* Total workspace height (mm), <-PLACEHOLDER->  \
                               */

/* ------------------------------------------------------------- */
/*  Arm link lengths (mm)                                        */
/*  CALIBRATE: measure your physical arm links                   */
/* ------------------------------------------------------------- */

#define LINK1_MM 117.5f /* Shoulder → elbow length (mm) */
#define LINK2_MM 200.0f /* Elbow → wrist  length (mm)  */

/* ------------------------------------------------------------- */
/*  Servo pins                                                   */
/* ------------------------------------------------------------- */

#define NUM_SERVOS 4

#define SERVO1_PIN 12 /* Base */
#define SERVO2_PIN 14 /* Shoulder */
#define SERVO3_PIN 33 /* Elbow */
#define SERVO4_PIN 32 /* Gripper */

/* ------------------------------------------------------------- */
/*  Servo angle limits (logical degrees, 0 = center)             */
/*  These are enforced by the IK solver and servo_control.       */
/*  toPhysical() maps logical -90..90 → servo.write() 180..0.   */
/* ------------------------------------------------------------- */

#define SERVO_BASE_MIN -90
#define SERVO_BASE_MAX 90
#define SERVO_SHOULDER_MIN -90
#define SERVO_SHOULDER_MAX 20
#define SERVO_ELBOW_MIN 0
#define SERVO_ELBOW_MAX 60
#define SERVO_GRIPPER_MIN -20
#define SERVO_GRIPPER_MAX 40

#define SERVO_GRIPPER_OPEN 30   /* Within [-20, 40], <-CALIBRATE-> */
#define SERVO_GRIPPER_CLOSE -10 /* Within [-20, 40], <-CALIBRATE-> */

/* ------------------------------------------------------------- */
/*  Fixed Z heights (encoded as shoulder degrees)                */
/*  CALIBRATE: find the servo angles for these heights           */
/* ------------------------------------------------------------- */

#define SHOULDER_HOVER_DEG                                                     \
  10 /* Arm raised, hovering above table, <-PLACEHOLDER-> */
#define SHOULDER_PICK_DEG                                                      \
  -30 /* Arm lowered to pick height     , <-PLACEHOLDER-> */
#define SHOULDER_DROP_DEG                                                      \
  -20 /* Arm lowered to drop height     , <-PLACEHOLDER-> */

/* ------------------------------------------------------------- */
/*  Home position (all servos at center = 0°)                     */
/* ------------------------------------------------------------- */

#define HOME_BASE_DEG 0
#define HOME_SHOULDER_DEG 0
#define HOME_ELBOW_DEG 0

/* ------------------------------------------------------------- */
/*  Bin locations in image-space pixels                          */
/*  CALIBRATE: place each bin and note its pixel centroid        */
/* ------------------------------------------------------------- */

#define RED_BIN_PX_X 40  /* Red   bin X pixel, <-PLACEHOLDER-> */
#define RED_BIN_PX_Y 120 /* Red   bin Y pixel, <-PLACEHOLDER-> */

#define GREEN_BIN_PX_X 160 /* Green bin X pixel, <-PLACEHOLDER-> */
#define GREEN_BIN_PX_Y 120 /* Green bin Y pixel, <-PLACEHOLDER-> */

#define BLUE_BIN_PX_X 280 /* Blue  bin X pixel, <-PLACEHOLDER-> */
#define BLUE_BIN_PX_Y 120 /* Blue  bin Y pixel, <-PLACEHOLDER-> */

/* ------------------------------------------------------------- */
/*  Shared data types used across all modules                    */
/* ------------------------------------------------------------- */

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

#endif /* CONFIG_H */
