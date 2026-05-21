/* =============================================================
 * arm_controller.c
 * Arm motion module.  Translates high-level commands (hover,
 * pick, place, home) into servo angle commands, calling the
 * kinematics module for all coordinate math.
 *
 * All HAL_* calls and servo_set_angle() are PLACEHOLDERS.
 * Replace them with your MCU's PWM/HAL calls.
 * ============================================================= 
 */

#include <stdio.h>
#include "arm_controller.h"
#include "kinematics.h"
#include "config.h"

/* ------------------------------------------------------------- */
/*  PLACEHOLDER: Low-level servo driver                          */
/*                                                               */
/*  Replace this function with your actual PWM output.           */
/*  On STM32 this would be:                                      */
/*      TIMx->CCRy = angle_to_pulse(degrees);                    */
/*  On ESP32 (ESP-IDF):                                          */
/*      mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, ...);  */
/*                                                               */
/*  servo_id: 1=base  2=shoulder  3=elbow  4=gripper             */
/* ------------------------------------------------------------- */

static void servo_set_angle(uint8_t servo_id, float degrees)
{
    /* PLACEHOLDER: write degrees → PWM pulse to servo_id */
    printf("[SERVO] Servo %d → %.1f°\n", servo_id, degrees);
}

/* ------------------------------------------------------------- */
/*  PLACEHOLDER: Blocking delay                                  */
/*                                                               */
/*  Replace with HAL_Delay(ms) on STM32, vTaskDelay on FreeRTOS, */
/*  or esp_rom_delay_us(ms*1000) on ESP32.                       */
/* ------------------------------------------------------------- */

static void delay_ms(uint32_t ms)
{
    /* PLACEHOLDER: HAL_Delay(ms); */
    (void)ms;  /* suppress unused warning in simulation */
    printf("[DELAY] %lu ms\n", ms);
}

/* ------------------------------------------------------------- */
/*  Servo 4: Gripper                                             */
/* ------------------------------------------------------------- */

void gripper_open(void)
{
    printf("[ARM] Gripper OPEN\n");
    servo_set_angle(4, SERVO_GRIPPER_OPEN);
    delay_ms(400);  /* wait for gripper to open fully */
}

void gripper_close(void)
{
    printf("[ARM] Gripper CLOSE\n");
    servo_set_angle(4, SERVO_GRIPPER_CLOSE);
    delay_ms(400);  /* wait for gripper to close fully */
}

/* ------------------------------------------------------------- */
/*  Core motion: set all 3 positional servos at once             */
/*                                                               */
/*  Motion sequencing note:                                      */
/*  For safety, move shoulder first (raise) before rotating the  */
/*  base, avoids the arm sweeping into obstacles at low height.  */
/*  PLACEHOLDER: add interpolation / speed control here if your  */
/*  servos need smooth ramping.                                  */
/* ------------------------------------------------------------- */

void arm_move_to_angles(ArmAngles angles)
{
    printf("[ARM] Moving → base: %.1f°  shoulder: %.1f°  elbow: %.1f°\n",
           angles.base_deg, angles.shoulder_deg, angles.elbow_deg);

    /* Step 1: raise shoulder first to clear obstacles */
    servo_set_angle(2, angles.shoulder_deg);
    delay_ms(300);

    /* Step 2: rotate base to face target */
    servo_set_angle(1, angles.base_deg);
    delay_ms(300);

    /* Step 3: set elbow */
    servo_set_angle(3, angles.elbow_deg);
    delay_ms(300);
}

/* ------------------------------------------------------------- */
/*  Initialize: move to home on startup                          */
/* ------------------------------------------------------------- */

void arm_init(void)
{
    printf("[ARM] Initializing: moving to home position\n");

    /* Open gripper first so we don't clamp anything */
    gripper_open();

    ArmAngles home = {
        .base_deg     = HOME_BASE_DEG,
        .shoulder_deg = HOME_SHOULDER_DEG,
        .elbow_deg    = HOME_ELBOW_DEG
    };
    arm_move_to_angles(home);

    printf("[ARM] Ready\n");
}

/* ------------------------------------------------------------- */
/*  Return to home                                               */
/* ------------------------------------------------------------- */

void arm_return_home(void)
{
    printf("[ARM] Returning to home position\n");

    /* Raise shoulder first before swinging base */
    ArmAngles home = {
        .base_deg     = HOME_BASE_DEG,
        .shoulder_deg = HOME_SHOULDER_DEG,
        .elbow_deg    = HOME_ELBOW_DEG
    };
    arm_move_to_angles(home);
}

/* ------------------------------------------------------------- */
/*  Hover over a pixel target                                    */
/*                                                               */
/*  1. Converts pixel → mm via kinematics module                 */
/*  2. Solves IK to get joint angles                             */
/*  3. Overrides shoulder with HOVER height angle                */
/*  4. Moves servos                                              */
/* ------------------------------------------------------------- */

void arm_hover_over(uint32_t px_x, uint32_t px_y)
{
    printf("[ARM] Hovering over pixel (%lu, %lu)\n", px_x, px_y);

    /* Convert pixel centroid → mm in robot workspace */
    float x_mm, y_mm;
    kinematics_pixel_to_mm(px_x, px_y, &x_mm, &y_mm);

    /* Solve IK for base and elbow, shoulder overridden for hover height */
    ArmAngles angles = kinematics_solve_ik(x_mm, y_mm);

    /* Override shoulder to hover height so arm stays raised */
    angles.shoulder_deg = SHOULDER_HOVER_DEG;

    arm_move_to_angles(angles);
    delay_ms(200);  /* settle */
}

/* ------------------------------------------------------------- */
/*  Lower arm to pick height, close gripper, raise back up       */
/*                                                               */
/*  Call AFTER arm_hover_over() and gripper_open().              */
/*                                                               */
/*  CALIBRATE: SHOULDER_PICK_DEG in config.h controls how far    */
/*  down the arm lowers, adjust until gripper reaches objects.   */
/* ------------------------------------------------------------- */

void arm_lower_and_pick(void)
{
    printf("[ARM] Lowering to pick height\n");

    /* Lower shoulder to pick height (only shoulder changes) */
    servo_set_angle(2, SHOULDER_PICK_DEG);
    delay_ms(500);  /* wait for arm to reach pick height */

    /* Close gripper to grab object */
    gripper_close();
    delay_ms(200);

    /* Raise shoulder back to hover height with object held */
    printf("[ARM] Raising with object\n");
    servo_set_angle(2, SHOULDER_HOVER_DEG);
    delay_ms(500);
}

/* ------------------------------------------------------------- */
/*  Move to color bin and drop object                            */
/*                                                               */
/*  Bin positions are defined as pixel coordinates in config.h.  */
/*  They go through the same pixel→mm→IK pipeline as object      */
/*  detection, so you calibrate them the same way.               */
/* ------------------------------------------------------------- */

void arm_move_to_bin(Color color)
{
    uint32_t bin_px_x, bin_px_y;
    const char *color_name;

    /* Select bin pixel location from config */
    switch (color) {
        case COLOR_RED:
            bin_px_x = RED_BIN_PX_X;
            bin_px_y = RED_BIN_PX_Y;
            color_name = "RED";
            break;
        case COLOR_GREEN:
            bin_px_x = GREEN_BIN_PX_X;
            bin_px_y = GREEN_BIN_PX_Y;
            color_name = "GREEN";
            break;
        case COLOR_BLUE:
            bin_px_x = BLUE_BIN_PX_X;
            bin_px_y = BLUE_BIN_PX_Y;
            color_name = "BLUE";
            break;
        default:
            printf("[ARM] Unknown color: cannot move to bin\n");
            return;
    }

    printf("[ARM] Moving to %s bin at pixel (%lu, %lu)\n",
           color_name, bin_px_x, bin_px_y);

    /* Solve IK for bin position (same pipeline as object pick) */
    ArmAngles bin_angles = kinematics_bin_angles(bin_px_x, bin_px_y);

    /* Move to hover height above bin */
    bin_angles.shoulder_deg = SHOULDER_HOVER_DEG;
    arm_move_to_angles(bin_angles);
    delay_ms(300);

    /* Lower to drop height */
    printf("[ARM] Lowering to drop height\n");
    servo_set_angle(2, SHOULDER_DROP_DEG);
    delay_ms(400);

    /* Release object */
    gripper_open();
    delay_ms(300);

    /* Raise back to hover height */
    servo_set_angle(2, SHOULDER_HOVER_DEG);
    delay_ms(400);
}
