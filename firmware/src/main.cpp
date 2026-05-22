#include "esp_camera.h"
#include "board_config.h"
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"
#include "types.h"
#include "color_detect.h"
#include "calibrate.h"
#include "kinematics.h"
#include "servo_control.h"
#include <Preferences.h>

// ─── Global state ─────────────────────────────────────────────────────────
Pixel*     imageMatrix = nullptr;

// ─── Mode flags ────────────────────────────────────────────────────────────
static bool inCalibMode     = false;
static bool inServoTestMode = false;

// ─── Calibration state ─────────────────────────────────────────────────────
static bool   scanned     = false;
static bool   solved      = false;
static uint32_t dot_px[4], dot_py[4];
static int    dot_count   = 0;
static float  calib_H[3][3];
static float  old_H[3][3];
static bool   had_previous_H = false;

// ─── NVS persistence bridge (C → C++ for Preferences) ──────────────────────
extern "C" {

bool calibrate_nvs_load(float H[3][3])
{
    Preferences prefs;
    prefs.begin("calib", true);
    if (!prefs.getBool("valid", false)) {
        prefs.end();
        return false;
    }
    H[0][0] = prefs.getFloat("h00", 1.0f); H[0][1] = prefs.getFloat("h01", 0.0f); H[0][2] = prefs.getFloat("h02", 0.0f);
    H[1][0] = prefs.getFloat("h10", 0.0f); H[1][1] = prefs.getFloat("h11", 1.0f); H[1][2] = prefs.getFloat("h12", 0.0f);
    H[2][0] = prefs.getFloat("h20", 0.0f); H[2][1] = prefs.getFloat("h21", 0.0f); H[2][2] = 1.0f;
    prefs.end();
    return true;
}

void calibrate_nvs_save(const float H[3][3])
{
    Preferences prefs;
    prefs.begin("calib", false);
    prefs.putFloat("h00", H[0][0]); prefs.putFloat("h01", H[0][1]); prefs.putFloat("h02", H[0][2]);
    prefs.putFloat("h10", H[1][0]); prefs.putFloat("h11", H[1][1]); prefs.putFloat("h12", H[1][2]);
    prefs.putFloat("h20", H[2][0]); prefs.putFloat("h21", H[2][1]);
    prefs.putBool("valid", true);
    prefs.end();
}

void calibrate_nvs_erase(void)
{
    Preferences prefs;
    prefs.begin("calib", false);
    prefs.putBool("valid", false);
    prefs.end();
}

} // extern "C"

// ─── Camera init ──────────────────────────────────────────────────────────
bool initCamera() {
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer   = LEDC_TIMER_0;
  config.pin_d0       = Y2_GPIO_NUM;
  config.pin_d1       = Y3_GPIO_NUM;
  config.pin_d2       = Y4_GPIO_NUM;
  config.pin_d3       = Y5_GPIO_NUM;
  config.pin_d4       = Y6_GPIO_NUM;
  config.pin_d5       = Y7_GPIO_NUM;
  config.pin_d6       = Y8_GPIO_NUM;
  config.pin_d7       = Y9_GPIO_NUM;
  config.pin_xclk     = XCLK_GPIO_NUM;
  config.pin_pclk     = PCLK_GPIO_NUM;
  config.pin_vsync    = VSYNC_GPIO_NUM;
  config.pin_href     = HREF_GPIO_NUM;
  config.pin_sccb_sda = SIOD_GPIO_NUM;
  config.pin_sccb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn     = PWDN_GPIO_NUM;
  config.pin_reset    = RESET_GPIO_NUM;

  config.xclk_freq_hz = 10000000;
  config.pixel_format = PIXFORMAT_RGB565;
  config.frame_size   = FRAMESIZE_QVGA;
  config.grab_mode    = CAMERA_GRAB_WHEN_EMPTY;
  config.fb_location  = CAMERA_FB_IN_PSRAM;
  config.fb_count     = 1;
  config.jpeg_quality = 12;

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed: 0x%x\n", err);
    return false;
  }

  sensor_t* s = esp_camera_sensor_get();
  s->set_vflip(s, 1);
  s->set_brightness(s, 1);
  s->set_saturation(s, -1);
  return true;
}

// ─── Decode RGB565 word → Pixel ───────────────────────────────────────────
inline Pixel decodeRGB565(const uint8_t* buf, int row, int col) {
  int idx       = (row * IMG_WIDTH + col) * 2;
  uint16_t word = ((uint16_t)buf[idx] << 8) | buf[idx + 1];
  Pixel p;
  p.r = (uint8_t)((word >> 11) & 0x1F) << 3;
  p.g = (uint8_t)((word >>  5) & 0x3F) << 2;
  p.b = (uint8_t)( word        & 0x1F) << 3;
  return p;
}

// ─── Allocate matrix in PSRAM ─────────────────────────────────────────────
bool allocMatrix() {
  imageMatrix = (Pixel*) heap_caps_malloc(
    IMG_HEIGHT * IMG_WIDTH * sizeof(Pixel), MALLOC_CAP_SPIRAM);
  if (!imageMatrix) {
    Serial.println("ERROR: PSRAM allocation failed");
    return false;
  }
  return true;
}

// ─── Capture → fill matrix ────────────────────────────────────────────────
bool captureToMatrix() {
  camera_fb_t* fb = esp_camera_fb_get();
  if (!fb) { Serial.println("ERROR: capture failed"); return false; }

  for (int row = 0; row < IMG_HEIGHT; row++)
    for (int col = 0; col < IMG_WIDTH; col++)
      imageAt(imageMatrix, row, col) = decodeRGB565(fb->buf, row, col);

  esp_camera_fb_return(fb);
  return true;
}

// ─── Scan imageMatrix and print object positions ─────────────────────────
void run_detection() {
    if (!imageMatrix) return;

    static const Color colors[] = {COLOR_RED, COLOR_GREEN, COLOR_BLUE, COLOR_YELLOW};
    static const char* names[]  = {"RED", "GREEN", "BLUE", "YELLOW"};
    bool any = false;

    for (int i = 0; i < 4; i++) {
        DetectionResult r = color_detect_scan_pixels(
            imageMatrix, IMG_WIDTH, IMG_HEIGHT, colors[i]);
        if (r.found) {
            float mm_x, mm_y;
            kinematics_pixel_to_mm(r.centroid_x, r.centroid_y, &mm_x, &mm_y);
            Serial.printf("%s → pixel(%3u, %3u)  mm(%7.1f, %7.1f)\n",
                          names[i], r.centroid_x, r.centroid_y, mm_x, mm_y);
            any = true;
        }
    }

    if (!any)
        Serial.println("No objects detected.");
}

// ── Calibration mode ──────────────────────────────────────────────────

static void print_calib_help() {
    Serial.println("--- Calibration commands ---");
    Serial.println("SCAN           — capture frame, detect 4 dark dots");
    Serial.println("SOLVE          — compute homography from SCAN data");
    Serial.println("REVIEW         — show last SCAN pixel coords");
    Serial.println("SAVE           — persist homography to NVS");
    Serial.println("ERASE          — remove saved homography from NVS");
    Serial.println("END            — exit calibration mode");
    Serial.println("----------------------------");
}

static void handle_calib_serial(String &line) {
    if (line.equalsIgnoreCase("HELP")) {
        print_calib_help();
        return;
    }

    if (line.equalsIgnoreCase("END")) {
        inCalibMode = false;
        scanned = false;
        solved = false;
        Serial.println("[CALIB] Exiting. Resuming normal operation.");
        return;
    }

    if (line.equalsIgnoreCase("SCAN")) {
        if (!captureToMatrix()) {
            Serial.println("[CALIB] ERROR: capture failed.");
            return;
        }
        dot_count = calibrate_find_dots(imageMatrix, IMG_WIDTH, IMG_HEIGHT,
                                        dot_px, dot_py);
        scanned = true;
        solved = false;

        if (dot_count == 4) {
            Serial.printf("[CALIB] Found %d dots:\n", dot_count);
            for (int i = 0; i < 4; i++)
                Serial.printf("  Dot %d: pixel(%u, %u)\n", i, dot_px[i], dot_py[i]);
            Serial.println("[CALIB] All 4 dots detected. Send SOLVE to compute homography.");
        } else {
            Serial.printf("[CALIB] ERROR: found %d dot(s), need 4.\n", dot_count);
        }
        return;
    }

    if (line.equalsIgnoreCase("REVIEW")) {
        if (!scanned) { Serial.println("[CALIB] No SCAN data."); return; }
        Serial.printf("[CALIB] Last SCAN: %d dots\n", dot_count);
        for (int i = 0; i < dot_count; i++)
            Serial.printf("  Dot %d: pixel(%u, %u)\n", i, dot_px[i], dot_py[i]);
        return;
    }

    if (line.equalsIgnoreCase("SOLVE")) {
        if (!scanned) { Serial.println("[CALIB] No SCAN data."); return; }
        if (dot_count < 4) { Serial.println("[CALIB] Not enough dots."); return; }

        float rms;
        if (!calibrate_solve(dot_px, dot_py, calib_H, &rms)) {
            Serial.println("[CALIB] Solver failed.");
            return;
        }

        solved = true;
        Serial.println("[CALIB] Homography matrix H:");
        Serial.printf("  [%7.4f  %7.4f  %7.4f]\n", calib_H[0][0], calib_H[0][1], calib_H[0][2]);
        Serial.printf("  [%7.4f  %7.4f  %7.4f]\n", calib_H[1][0], calib_H[1][1], calib_H[1][2]);
        Serial.printf("  [%7.4f  %7.4f  %7.4f]\n", calib_H[2][0], calib_H[2][1], calib_H[2][2]);
        Serial.printf("[CALIB] RMS error: %.1f mm\n", rms);
        return;
    }

    if (line.equalsIgnoreCase("SAVE")) {
        if (!solved) { Serial.println("[CALIB] Nothing to save."); return; }
        calibrate_save_matrix(calib_H);
        return;
    }

    if (line.equalsIgnoreCase("ERASE")) {
        calibrate_erase();
        solved = false;
        scanned = false;
        return;
    }

    Serial.printf("[CALIB] Unknown: %s\n", line.c_str());
    print_calib_help();
}

// ── Servo test mode ───────────────────────────────────────────────────

static void handle_servotest_serial(String &line) {
    if (line.equalsIgnoreCase("END")) {
        inServoTestMode = false;
        Serial.println("[SERVO] Test mode exited.");
        return;
    }

    if (line.equalsIgnoreCase("HOME")) {
        servo_home();
        return;
    }

    int angles[NUM_SERVOS];
    int index = 0;
    int start = 0;

    while (index < NUM_SERVOS) {
        int spacePos = line.indexOf(' ', start);
        String val;
        if (spacePos == -1)
            val = line.substring(start);
        else
            val = line.substring(start, spacePos);

        val.trim();
        angles[index] = val.toInt();
        index++;
        if (spacePos == -1) break;
        start = spacePos + 1;
    }

    for (int i = index; i < NUM_SERVOS; i++)
        angles[i] = 0;

    servo_write_all(angles);
}

// ─── Serial command dispatcher ─────────────────────────────────────────

static void print_help() {
    Serial.println("--- Commands ---");
    Serial.println("CALIB          — camera calibration mode");
    Serial.println("SERVOTEST      — interactive servo test mode");
    Serial.println("HOME           — return all servos to 0°");
    Serial.println("----------------");
}

static void dispatch_serial() {
    if (!Serial.available()) return;

    String line = Serial.readStringUntil('\n');
    line.trim();

    if (line.equalsIgnoreCase("HELP")) {
        print_help();
        return;
    }

    if (line.equalsIgnoreCase("CALIB") && !inCalibMode && !inServoTestMode) {
        inCalibMode = true;
        scanned = false;
        solved = false;
        dot_count = 0;
        had_previous_H = calibrate_is_done();
        if (had_previous_H)
            calibrate_load_matrix(old_H);
        Serial.println("[CALIB] Mode active. Normal capture loop paused.");
        Serial.println("[CALIB] Place 200x200mm paper with 4 dark dots centered under robot base.");
        print_calib_help();
        return;
    }

    if (line.equalsIgnoreCase("SERVOTEST") && !inCalibMode && !inServoTestMode) {
        inServoTestMode = true;
        Serial.println("[SERVO] Test mode active. Enter angles separated by spaces, e.g.:");
        Serial.println("  45 -30 20 10");
        Serial.println("  HOME  — return to 0°");
        Serial.println("  END   — exit servo test mode");
        return;
    }

    if (!inCalibMode && !inServoTestMode && line.equalsIgnoreCase("HOME")) {
        servo_home();
        return;
    }

    if (inCalibMode)
        handle_calib_serial(line);
    else if (inServoTestMode)
        handle_servotest_serial(line);
}

// ─── setup ────────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  delay(500);

  if (!initCamera()) return;
  if (!allocMatrix()) return;

  camera_fb_t* w1 = esp_camera_fb_get(); esp_camera_fb_return(w1);
  camera_fb_t* w2 = esp_camera_fb_get(); esp_camera_fb_return(w2);
  delay(100);

  calibrate_init();
  servo_init();

  captureToMatrix();
  Serial.println("First frame captured.");
  run_detection();

  Serial.println("Auto-capture running every 2s. Send HELP for commands.");
  print_help();
}

// ─── loop ─────────────────────────────────────────────────────────────────
void loop() {
  dispatch_serial();

  if (inCalibMode || inServoTestMode) {
    delay(10);
    return;
  }

  captureToMatrix();
  run_detection();
  delay(2000);
}
