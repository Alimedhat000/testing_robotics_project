#include <Arduino.h>
#include "servo_control.h"
#include "kinematics.h"
#include "config.h"

#include "soc/timer_group_struct.h"
#include "soc/timer_group_reg.h"
static void disable_hw_wdt()
{
    TIMERG1.wdt_wprotect = TIMG_WDT_WKEY_VALUE;
    TIMERG1.wdt_config0.val = 0;
    TIMERG1.wdt_feed = 1;
    TIMERG1.wdt_wprotect = 0;
}

static float current_x_mm = 0.0f;
static float current_y_mm = 0.0f;
static float target_z_mm = 50.0f;
static int   gripper_deg = 0;

static char   line_buf[80];
static size_t line_pos = 0;

static void go_to(float x_mm, float y_mm)
{
    ArmAngles ang = kinematics_solve_ik(x_mm, y_mm, target_z_mm);

    if (ang.base_deg == HOME_BASE_DEG && ang.shoulder_deg == HOME_SHOULDER_DEG
        && ang.elbow_deg == HOME_ELBOW_DEG && !(x_mm == 0 && y_mm == 0)) {
        Serial.println(F("IK: UNREACHABLE"));
        return;
    }

    current_x_mm = x_mm;
    current_y_mm = y_mm;

    int tgt[NUM_SERVOS];
    tgt[0] = (int)roundf(ang.base_deg);
    tgt[1] = (int)roundf(ang.shoulder_deg - 90.0f);
    tgt[2] = (int)roundf(-ang.elbow_deg);
    tgt[3] = gripper_deg;
    servo_write_all(tgt);

    Serial.print(F("G "));
    Serial.print(x_mm, 1); Serial.print(' ');
    Serial.print(y_mm, 1);
    Serial.print(F("  →  base="));
    Serial.print(ang.base_deg, 1);
    Serial.print(F(" shoulder="));
    Serial.print(ang.shoulder_deg, 1);
    Serial.print(F(" elbow="));
    Serial.print(ang.elbow_deg, 1);
    Serial.print(F("  z="));
    Serial.print(target_z_mm, 1);
    Serial.println(F(" mm"));
}

static void print_help()
{
    Serial.println(F("── Serial Arm Test ──"));
    Serial.println(F("G X Y   → Go to (X,Y) mm at current Z"));
    Serial.println(F("Z MM    → Set target height (mm above table)"));
    Serial.println(F("O       → Open gripper"));
    Serial.println(F("C       → Close gripper"));
    Serial.println(F("S       → Show status"));
    Serial.println(F("H       → Home all servos to 0°"));
    Serial.println(F("?       → This help"));
}

static void show_status()
{
    ArmAngles ang = kinematics_solve_ik(current_x_mm, current_y_mm, target_z_mm);
    Serial.print(F("Pos: "));
    Serial.print(current_x_mm, 1); Serial.print(F(", "));
    Serial.print(current_y_mm, 1); Serial.print(F(" mm"));
    Serial.print(F("  Z="));
    Serial.print(target_z_mm, 1);
    Serial.print(F(" mm  |  base="));
    Serial.print(ang.base_deg, 1);
    Serial.print(F("  shoulder="));
    Serial.print(ang.shoulder_deg, 1);
    Serial.print(F("  elbow="));
    Serial.print(ang.elbow_deg, 1);
    Serial.print(F("  gripper="));
    Serial.print(gripper_deg);
    Serial.println();
}

void setup()
{
    disable_hw_wdt();
    Serial.begin(115200);
    delay(500);

    Serial.println(F("\n===== Serial Arm Test ====="));
    servo_init();
    Serial.println(F("Ready — type ? for help."));
}

static void process_line(const char *line)
{
    char cmd = toupper((unsigned char)line[0]);

    switch (cmd) {
    case 'G': {
        float x = 0, y = 0;
        if (sscanf(line + 1, "%f %f", &x, &y) >= 2) {
            go_to(x, y);
        } else {
            Serial.println(F("G: usage G X Y"));
        }
        break;
    }
    case 'Z': {
        float mm = 0;
        if (sscanf(line + 1, "%f", &mm) == 1) {
            if (mm < -50.0f) mm = -50.0f;
            if (mm > 200.0f) mm = 200.0f;
            target_z_mm = mm;
            go_to(current_x_mm, current_y_mm);
            Serial.print(F("Z = "));
            Serial.print(mm, 1);
            Serial.println(F(" mm"));
        } else {
            Serial.println(F("Z: usage Z MM"));
        }
        break;
    }
    case 'O':
        gripper_deg = -10;
        servo_write_single(3, gripper_deg);
        Serial.println(F("Gripper OPEN"));
        break;

    case 'C':
        gripper_deg = 30;
        servo_write_single(3, gripper_deg);
        Serial.println(F("Gripper CLOSE"));
        break;

    case 'S':
        show_status();
        break;

    case 'H':
        servo_home();
        current_x_mm = 0;
        current_y_mm = 0;
        gripper_deg = 0;
        Serial.println(F("Homed — all servos at 0°"));
        break;

    case '?':
        print_help();
        break;

    default:
        Serial.print(F("Unknown: "));
        Serial.println(line);
        Serial.println(F("Type ? for help."));
        break;
    }
}

void loop()
{
    while (Serial.available()) {
        char c = (char)Serial.read();
        if (c == '\n' || c == '\r') {
            if (line_pos > 0) {
                line_buf[line_pos] = '\0';
                process_line(line_buf);
                line_pos = 0;
            }
        } else if (line_pos < sizeof(line_buf) - 1) {
            line_buf[line_pos++] = c;
        }
    }
}
