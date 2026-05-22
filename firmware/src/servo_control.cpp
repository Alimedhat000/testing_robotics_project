#include <Arduino.h>
#include <ESP32Servo.h>
#include "servo_control.h"
#include "config.h"

static Servo servos[NUM_SERVOS];
static int   currentAngle[NUM_SERVOS];
static int   targetAngle[NUM_SERVOS];

static const uint8_t servoPins[NUM_SERVOS] = {
    SERVO1_PIN, SERVO2_PIN, SERVO3_PIN, SERVO4_PIN
};

static const int servoMin[NUM_SERVOS] = {
    SERVO_BASE_MIN, SERVO_SHOULDER_MIN, SERVO_ELBOW_MIN, SERVO_GRIPPER_MIN
};

static const int servoMax[NUM_SERVOS] = {
    SERVO_BASE_MAX, SERVO_SHOULDER_MAX, SERVO_ELBOW_MAX, SERVO_GRIPPER_MAX
};

#define STEP_DELAY_MS 15

// ── Convert logical angle (-90..90) to servo.write() value (180..0) ──
static int toPhysical(int logical)
{
    return map(logical, -90, 90, 180, 0);
}

// ── Clamp angle to per-servo hardware limits ──
static int clampServo(int index, int angle)
{
    if (angle < servoMin[index]) return servoMin[index];
    if (angle > servoMax[index]) return servoMax[index];
    return angle;
}

// ── Move all servos simultaneously with stepped interpolation ──
static void moveAllServosSimultaneously(void)
{
    bool moving = true;
    while (moving) {
        moving = false;
        for (int i = 0; i < NUM_SERVOS; i++) {
            if (currentAngle[i] < targetAngle[i]) {
                currentAngle[i]++;
                servos[i].write(toPhysical(currentAngle[i]));
                moving = true;
            } else if (currentAngle[i] > targetAngle[i]) {
                currentAngle[i]--;
                servos[i].write(toPhysical(currentAngle[i]));
                moving = true;
            }
        }
        delay(STEP_DELAY_MS);
    }
}

// ─────────────────────────────────────────────────────────────────
//  Public API
// ─────────────────────────────────────────────────────────────────

void servo_init(void)
{
    for (int i = 0; i < NUM_SERVOS; i++) {
        servos[i].attach(servoPins[i], 500, 2500);
        currentAngle[i] = 0;
        targetAngle[i] = 0;
        servos[i].write(toPhysical(0));
        Serial.printf("[SERVO] Servo %d (pin %d) initialized at 0°\n",
                       i + 1, servoPins[i]);
        delay(300);
    }

    Serial.println("[SERVO] All servos homed to 0°");
    Serial.printf("[SERVO] Limits: 1=[%d,%d] 2=[%d,%d] 3=[%d,%d] 4=[%d,%d]\n",
                   servoMin[0], servoMax[0], servoMin[1], servoMax[1],
                   servoMin[2], servoMax[2], servoMin[3], servoMax[3]);
}

void servo_home(void)
{
    int angles[NUM_SERVOS] = {HOME_BASE_DEG, HOME_SHOULDER_DEG, HOME_ELBOW_DEG, 0};
    servo_write_all(angles);
    Serial.println("[SERVO] Returned to home.");
}

void servo_write_all(const int angles_deg[NUM_SERVOS])
{
    for (int i = 0; i < NUM_SERVOS; i++) {
        targetAngle[i] = clampServo(i, angles_deg[i]);
    }
    moveAllServosSimultaneously();

    Serial.print("[SERVO] Moved:");
    for (int i = 0; i < NUM_SERVOS; i++) {
        Serial.printf(" %d°", targetAngle[i]);
    }
    Serial.println();
}

void servo_write_single(int index, int logical_deg)
{
    if (index < 0 || index >= NUM_SERVOS) return;
    int clamped = clampServo(index, logical_deg);
    targetAngle[index] = clamped;
    moveAllServosSimultaneously();
    Serial.printf("[SERVO] Servo %d → %d°\n", index + 1, clamped);
}

// ─────────────────────────────────────────────────────────────────
//  Interactive servo test: enter angles, servos move
//  Activated by SERVOTEST command from main.cpp
// ─────────────────────────────────────────────────────────────────

void servo_test_serial(void)
{
    if (Serial.available() == 0) return;

    String input = Serial.readStringUntil('\n');
    input.trim();
    if (input.length() == 0) return;

    // Allow "HOME" to return to home
    if (input.equalsIgnoreCase("HOME")) {
        servo_home();
        return;
    }

    int angles[NUM_SERVOS];
    int index = 0;
    int start = 0;

    while (index < NUM_SERVOS) {
        int spacePos = input.indexOf(' ', start);
        String valueStr;
        if (spacePos == -1)
            valueStr = input.substring(start);
        else
            valueStr = input.substring(start, spacePos);

        valueStr.trim();
        angles[index] = valueStr.toInt();
        index++;

        if (spacePos == -1) break;
        start = spacePos + 1;
    }

    // Fill remaining with current positions
    for (int i = index; i < NUM_SERVOS; i++)
        angles[i] = currentAngle[i];

    servo_write_all(angles);
}
