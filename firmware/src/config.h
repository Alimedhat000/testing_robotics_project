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
/*  Calibration                                                  */
/* ------------------------------------------------------------- */

#define MIN_DARK_BLOB   20      /* Minimum dark pixels to count as a calibration dot */
#define DARK_THRESHOLD  80      /* Max RGB value for a "dark" pixel (0–255) */

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

#define LINK1_MM        100.0f  /* Shoulder → elbow length (mm), <-PLACEHOLDER-> */
#define LINK2_MM        100.0f  /* Elbow → wrist  length (mm) , <-PLACEHOLDER-> */

/* ------------------------------------------------------------- */
/*  Servo angle limits (degrees)                                 */
/*  CALIBRATE: match your servo hardware limits                  */
/* ------------------------------------------------------------- */

#define SERVO_BASE_MIN      0
#define SERVO_BASE_MAX      180
#define SERVO_SHOULDER_MIN  0
#define SERVO_SHOULDER_MAX  180
#define SERVO_ELBOW_MIN     0
#define SERVO_ELBOW_MAX     180
#define SERVO_GRIPPER_OPEN  30   /* Degrees for open  position, <-PLACEHOLDER-> */
#define SERVO_GRIPPER_CLOSE 90   /* Degrees for closed position, <-PLACEHOLDER-> */

/* ------------------------------------------------------------- */
/*  Fixed Z heights (encoded as shoulder offset degrees)         */
/*  CALIBRATE: find the servo angles for these heights           */
/* ------------------------------------------------------------- */

#define SHOULDER_HOVER_DEG  60  /* Arm raised, hovering above table, <-PLACEHOLDER-> */
#define SHOULDER_PICK_DEG   30  /* Arm lowered to pick height     , <-PLACEHOLDER-> */
#define SHOULDER_DROP_DEG   35  /* Arm lowered to drop height     , <-PLACEHOLDER-> */

/* ------------------------------------------------------------- */
/*  Home position (all servos in degrees)                        */
/* ------------------------------------------------------------- */

#define HOME_BASE_DEG       90
#define HOME_SHOULDER_DEG   90
#define HOME_ELBOW_DEG      90

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

/* HSV color in 0–255 scale */
typedef struct {
    uint8_t h;
    uint8_t s;
    uint8_t v;
} HSV;

/* Color identifiers */
typedef enum {
    COLOR_NONE   = 0,
    COLOR_RED    = 1,
    COLOR_GREEN  = 2,
    COLOR_BLUE   = 3,
    COLOR_YELLOW = 4
} Color;

/* Result returned by the vision module after scanning one frame */
typedef struct {
    bool    found;          /* true if a valid object was detected */
    uint32_t centroid_x;   /* object centroid in image pixels      */
    uint32_t centroid_y;
    Color   color;          /* detected color of the object         */
} DetectionResult;

/* Servo angles for the three positional joints (degrees) */
typedef struct {
    float base_deg;         /* Servo 1, base rotation  */
    float shoulder_deg;     /* Servo 2, shoulder joint */
    float elbow_deg;        /* Servo 3, elbow joint    */
} ArmAngles;

/* Robot state machine states */
typedef enum {
    STATE_SCAN,
    STATE_DETECT,
    STATE_PICK,
    STATE_PLACE,
    STATE_HOME
} RobotState;

#endif /* CONFIG_H */
