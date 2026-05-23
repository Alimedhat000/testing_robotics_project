/* =============================================================
 * config.h
 * Global configuration constants and shared data types.
 * All tunable parameters live here, edit this file to
 * calibrate the system for your physical setup.
 * ============================================================= 
 */

#ifndef CONFIG_H
#define CONFIG_H

#include <stdint.h>
#include <stdbool.h>

/* ------------------------------------------------------------- */
/*  Camera / Image                                               */
/* ------------------------------------------------------------- */

#define IMG_WIDTH       320     /* OV7670 QVGA width  (pixels) */
#define IMG_HEIGHT      240     /* OV7670 QVGA height (pixels) */
#define MIN_BLOB_SIZE   50      /* Minimum pixel count to treat as a real object */

/* ------------------------------------------------------------- */
/*  Calibration — contrast-based dot detection                   */
/*  No absolute brightness threshold; uses local 5x5 contrast.   */
/* ------------------------------------------------------------- */

#define SEED_CONTRAST   2      /* Seed pixel contrast threshold (must be this much darker) */
#define FLOOD_CONTRAST  2      /* Flood-fill neighbour contrast threshold (relaxed) */
#define MIN_DARK_BLOB   20     /* Min pixel count (also scaled by w*h/10000) */

#define CALIB_DOT_MAX_CHROMA 18 /* Max average (max-min) allowed for dots */
#define CALIB_DOT_MIN_CIRCULARITY 0.60f /* Min circularity 4π·area/perim² — rejects shadows/creases */
#define CALIB_DOT_MIN_ASPECT 0.40f      /* Min aspect ratio (small/large bbox side) — rejects elongated shadows */
#define CALIB_DOT_MIN_FILL 0.35f        /* Min fill ratio (area/bbox_area) — rejects sparse/porous blobs */
#define CALIB_ABS_THRESH_DIV 2  /* abs_thresh = min + (range / div) */

#define CALIB_DEBUG     1

/* ------------------------------------------------------------- */
/*  Workspace geometry                                           */
/*  CALIBRATE: measure your robot's reachable table area in mm   */
/* ------------------------------------------------------------- */

#define WORKSPACE_X_MM  200.0f  /* Total workspace width  (mm), <-PLACEHOLDER-> */
#define WORKSPACE_Y_MM  150.0f  /* Total workspace height (mm), <-PLACEHOLDER-> */

/* ------------------------------------------------------------- */
/*  Arm link lengths (mm)                                        */
/*  CALIBRATE: measure your physical arm links                   */
/* ------------------------------------------------------------- */

#define LINK1_MM        117.5f  /* Shoulder → elbow length (mm) */
#define LINK2_MM        200.0f  /* Elbow → wrist  length (mm)  */

/* ------------------------------------------------------------- */
/*  Servo pins                                                   */
/* ------------------------------------------------------------- */

#define NUM_SERVOS      4

#define SERVO1_PIN      12      /* Base */
#define SERVO2_PIN      14      /* Shoulder */
#define SERVO3_PIN      33      /* Elbow */
#define SERVO4_PIN      32      /* Gripper */

/* ------------------------------------------------------------- */
/*  Servo angle limits (logical degrees, 0 = center)             */
/*  These are enforced by the IK solver and servo_control.       */
/*  toPhysical() maps logical -90..90 → servo.write() 180..0.   */
/* ------------------------------------------------------------- */

#define SERVO_BASE_MIN      -90
#define SERVO_BASE_MAX      90
#define SERVO_SHOULDER_MIN  -90
#define SERVO_SHOULDER_MAX  20
#define SERVO_ELBOW_MIN     0
#define SERVO_ELBOW_MAX     60
#define SERVO_GRIPPER_MIN   -20
#define SERVO_GRIPPER_MAX   40

#define SERVO_GRIPPER_OPEN  30      /* Within [-20, 40], <-CALIBRATE-> */
#define SERVO_GRIPPER_CLOSE -10     /* Within [-20, 40], <-CALIBRATE-> */

/* ------------------------------------------------------------- */
/*  Fixed Z heights (encoded as shoulder degrees)                */
/*  CALIBRATE: find the servo angles for these heights           */
/* ------------------------------------------------------------- */

#define SHOULDER_HOVER_DEG  10  /* Arm raised, hovering above table, <-PLACEHOLDER-> */
#define SHOULDER_PICK_DEG   -30 /* Arm lowered to pick height     , <-PLACEHOLDER-> */
#define SHOULDER_DROP_DEG  -20  /* Arm lowered to drop height     , <-PLACEHOLDER-> */

/* ------------------------------------------------------------- */
/*  Home position (all servos at center = 0°)                     */
/* ------------------------------------------------------------- */

#define HOME_BASE_DEG       0
#define HOME_SHOULDER_DEG   0
#define HOME_ELBOW_DEG      0

/* ------------------------------------------------------------- */
/*  Bin locations in image-space pixels                          */
/*  CALIBRATE: place each bin and note its pixel centroid        */
/* ------------------------------------------------------------- */

#define RED_BIN_PX_X    40      /* Red   bin X pixel, <-PLACEHOLDER-> */
#define RED_BIN_PX_Y    120     /* Red   bin Y pixel, <-PLACEHOLDER-> */

#define GREEN_BIN_PX_X  160     /* Green bin X pixel, <-PLACEHOLDER-> */
#define GREEN_BIN_PX_Y  120     /* Green bin Y pixel, <-PLACEHOLDER-> */

#define BLUE_BIN_PX_X   280     /* Blue  bin X pixel, <-PLACEHOLDER-> */
#define BLUE_BIN_PX_Y   120     /* Blue  bin Y pixel, <-PLACEHOLDER-> */

/* ------------------------------------------------------------- */
/*  Shared data types used across all modules                    */
/* ------------------------------------------------------------- */

typedef struct { uint8_t h, s, v; } HSV;

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

#endif /* CONFIG_H */
