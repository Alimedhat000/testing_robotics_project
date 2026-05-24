#include "servo_control.h"
#include "config.h"
#include <Arduino.h>
#include <ESP32Servo.h>

static Servo servos[NUM_SERVOS];
static int currentAngle[NUM_SERVOS];
static int targetAngle[NUM_SERVOS];

static const uint8_t servoPins[NUM_SERVOS] = {SERVO1_PIN, SERVO2_PIN,
                                              SERVO3_PIN, SERVO4_PIN};

static const int servoMin[NUM_SERVOS] = {
    -90, // base
    -90, // shoulder (servo-space: -90..0 after IK conversion)
    0,   // elbow   (servo-space: 0..90 after IK conversion)
    -20  // gripper
};

static const int servoMax[NUM_SERVOS] = {
    90, // base
    0,  // shoulder
    90, // elbow
    40  // gripper
};

#define STEP_DELAY_MS 15

/**
 * @brief Convert logical angle (-90..90) to servo.write() value.
 *
 * Servo.write() takes 0–180 where 90 is center.
 * toPhysical maps the -90..90 logical convention so that:
 *   - logical 0°  → servo.write(90)  (center)
 *   - logical -90° → servo.write(180) (full CCW)
 *   - logical 90°  → servo.write(0)   (full CW)
 *
 * @param logical  Logical angle in degrees (-90 to 90)
 * @return Physical value for servo.write() (180 to 0)
 */
static int toPhysical(int logical) { return map(logical, -90, 90, 180, 0); }

/**
 * @brief Clamp a logical angle to the per-servo hardware limits.
 * @param index  Servo index (0–NUM_SERVOS-1)
 * @param angle  Desired logical angle
 * @return Clamped angle within [servoMin[index], servoMax[index]]
 */
static int clampServo(int index, int angle) {
  if (angle < servoMin[index])
    return servoMin[index];
  if (angle > servoMax[index])
    return servoMax[index];
  return angle;
}

/**
 * @brief Stepped interpolation loop.
 *
 * Moves all servos from currentAngle[] toward targetAngle[]
 * one degree at a time (15 ms between steps).  This ensures all
 * servos arrive simultaneously regardless of the distance traveled.
 */
static void moveAllServosSimultaneously(void) {
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
    yield();
  }
}

// ─────────────────────────────────────────────────────────────────
//  Public API
// ─────────────────────────────────────────────────────────────────

/**
 * @brief Attach all servos, home to 0°, and report limits.
 *
 * Each servo is attached with 500–2500 µs pulse range.
 * A 300 ms delay between attachments prevents power surge.
 */
void servo_init(void) {
  for (int i = 0; i < NUM_SERVOS; i++) {
    servos[i].attach(servoPins[i], 500, 2500);
    currentAngle[i] = 0;
    targetAngle[i] = 0;
    servos[i].write(toPhysical(0));
    Serial.printf("[SERVO] Servo %d (pin %d) initialized at 0°\n", i + 1,
                  servoPins[i]);
    delay(300);
  }

  Serial.println("[SERVO] All servos homed to 0°");
  Serial.printf("[SERVO] Limits: 1=[%d,%d] 2=[%d,%d] 3=[%d,%d] 4=[%d,%d]\n",
                servoMin[0], servoMax[0], servoMin[1], servoMax[1], servoMin[2],
                servoMax[2], servoMin[3], servoMax[3]);
}

/**
 * @brief Return all servos to the home position (0°).
 */
void servo_home(void) {
  int angles[NUM_SERVOS] = {HOME_BASE_DEG, HOME_SHOULDER_DEG, HOME_ELBOW_DEG,
                            0};
  servo_write_all(angles);
  Serial.println("[SERVO] Returned to home.");
}

/**
 * @brief Move all servos to target angles simultaneously.
 *
 * Each angle is clamped to its per-servo limit, then all servos
 * move together via stepped interpolation.
 *
 * @param angles_deg  Array of NUM_SERVOS logical target angles
 */
void servo_write_all(const int angles_deg[NUM_SERVOS]) {
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

/**
 * @brief Move a single servo, leaving others unchanged.
 * @param index        Servo index (0 = base, 1 = shoulder, 2 = elbow, 3 =
 * gripper)
 * @param logical_deg  Target angle in logical degrees
 */
void servo_write_single(int index, int logical_deg) {
  if (index < 0 || index >= NUM_SERVOS)
    return;
  int clamped = clampServo(index, logical_deg);
  targetAngle[index] = clamped;
  moveAllServosSimultaneously();
  Serial.printf("[SERVO] Servo %d → %d°\n", index + 1, clamped);
}

// ─────────────────────────────────────────────────────────────────
//  Interactive servo test
//  Activated by SERVOTEST command from main.cpp
// ─────────────────────────────────────────────────────────────────

/**
 * @brief Interactive serial servo test handler.
 *
 * Parses space-separated logical angle values from a single line,
 * then calls servo_write_all() to move them simultaneously.
 * Supports special commands: HOME (return to 0°), END (exit mode).
 * If fewer than NUM_SERVOS values are provided, remaining servos
 * stay at their current position.
 */
void servo_test_serial(void) {
  if (Serial.available() == 0)
    return;

  String input = Serial.readStringUntil('\n');
  input.trim();
  if (input.length() == 0)
    return;

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

    if (spacePos == -1)
      break;
    start = spacePos + 1;
  }

  for (int i = index; i < NUM_SERVOS; i++)
    angles[i] = currentAngle[i];

  servo_write_all(angles);
}
