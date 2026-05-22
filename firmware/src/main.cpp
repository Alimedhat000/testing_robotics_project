#include "esp_camera.h"
#include "board_config.h"
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"
#include "types.h"
#include <WiFi.h>
#include <WebServer.h>
#include "img_converters.h"
#include "color_detect.h"
#include "calibrate.h"
#include "kinematics.h"
#include <Preferences.h>

// ─── WiFi credentials ─────────────────────────────────────────────────────
const char* ssid     = "Rwan'sHouse";
const char* password = "rwanshouse2024";

// ─── Global state ─────────────────────────────────────────────────────────
Pixel*     imageMatrix = nullptr;
WebServer  server(80);

// ─── Calibration state ─────────────────────────────────────────────────────
static bool   inCalibMode = false;
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

// ─── Re-encode matrix → JPEG and send to browser ──────────────────────────
void handleCapture() {
  if (!imageMatrix) {
    server.send(503, "text/plain", "No frame captured yet");
    return;
  }

  uint8_t* jpegBuf = nullptr;
  size_t   jpegLen = 0;

  bool ok = fmt2jpg(
    (uint8_t*) imageMatrix,
    IMG_WIDTH * IMG_HEIGHT * 3,
    IMG_WIDTH,
    IMG_HEIGHT,
    PIXFORMAT_RGB888,
    80,
    &jpegBuf,
    &jpegLen
  );

  if (!ok || !jpegBuf) {
    server.send(500, "text/plain", "JPEG conversion failed");
    return;
  }

  server.send_P(200, "image/jpeg", (const char*)jpegBuf, jpegLen);
  free(jpegBuf);
}

// ─── Root page ────────────────────────────────────────────────────────────
void handleRoot() {
  String html = "";
  html += "<!DOCTYPE html><html><head>";
  html += "<meta charset='utf-8'>";
  html += "<title>ESP32 Capture</title>";
  html += "<style>";
  html += "body{background:#111;display:flex;flex-direction:column;";
  html += "align-items:center;justify-content:center;min-height:100vh;";
  html += "margin:0;font-family:sans-serif;color:#eee;}";
  html += "img{border:2px solid #444;border-radius:6px;max-width:90vw;}";
  html += "button{margin-top:16px;padding:10px 24px;font-size:16px;";
  html += "background:#2563eb;color:#fff;border:none;border-radius:6px;cursor:pointer;}";
  html += "button:hover{background:#1d4ed8;}";
  html += "</style></head><body>";
  html += "<h2>ESP32-WROVER OV2640 - last capture</h2>";
  html += "<img id='pic' src='/capture' alt='captured frame'>";
  html += "<button onclick=\"document.getElementById('pic').src='/capture?t='+Date.now()\">";
  html += "Capture new frame";
  html += "</button>";
  html += "</body></html>";

  server.send(200, "text/html", html);
}

// ─── Scan imageMatrix and draw centroids ────────────────────────────────
void run_detection() {
    if (!imageMatrix) return;

    static const Color colors[] = {COLOR_RED, COLOR_GREEN, COLOR_BLUE, COLOR_YELLOW};
    static const char* names[]  = {"RED", "GREEN", "BLUE", "YELLOW"};

    for (int i = 0; i < 4; i++) {
        DetectionResult r = color_detect_scan_pixels(
            imageMatrix, IMG_WIDTH, IMG_HEIGHT, colors[i]);
        if (r.found) {
            float mm_x, mm_y;
            kinematics_pixel_to_mm(r.centroid_x, r.centroid_y, &mm_x, &mm_y);
            Serial.printf("%s centroid: (%u, %u) → (%.1f, %.1f) mm\n",
                          names[i], r.centroid_x, r.centroid_y, mm_x, mm_y);
            draw_centroid(imageMatrix, IMG_WIDTH, IMG_HEIGHT,
                          (int)r.centroid_x, (int)r.centroid_y, colors[i]);
        } else {
            Serial.printf("%s centroid: (none)\n", names[i]);
        }
    }
}

// ─── Calibration serial CLI ─────────────────────────────────────────────
static void calibrate_print_help() {
    Serial.println("--- Calibration commands ---");
    Serial.println("CALIB          — enter calibration mode");
    Serial.println("SCAN           — capture frame, detect 4 dark dots");
    Serial.println("SOLVE          — compute homography from SCAN data");
    Serial.println("REVIEW         — show last SCAN pixel coords");
    Serial.println("SAVE           — persist homography to NVS");
    Serial.println("ERASE          — remove saved homography from NVS");
    Serial.println("END            — exit calibration mode");
    Serial.println("HELP           — show this list");
    Serial.println("----------------------------");
}

static void calibrate_handle_serial() {
    if (!Serial.available()) return;

    String line = Serial.readStringUntil('\n');
    line.trim();

    if (!inCalibMode && line.equalsIgnoreCase("CALIB")) {
        inCalibMode = true;
        scanned = false;
        solved = false;
        dot_count = 0;
        had_previous_H = calibrate_is_done();
        if (had_previous_H)
            calibrate_load_matrix(old_H);
        Serial.println("[CALIB] Mode active. Normal capture loop paused.");
        Serial.println("[CALIB] Place 200x200mm paper with 4 dark dots centered under robot base.");
        calibrate_print_help();
        return;
    }

    if (!inCalibMode) return;

    if (line.equalsIgnoreCase("HELP")) {
        calibrate_print_help();
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
            Serial.println("[CALIB] ERROR: capture failed. Check camera.");
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
            Serial.println("[CALIB] Check paper placement, lighting, and focus. Re-SCAN.");
        }
        return;
    }

    if (line.equalsIgnoreCase("REVIEW")) {
        if (!scanned) {
            Serial.println("[CALIB] No SCAN data. Run SCAN first.");
            return;
        }
        Serial.printf("[CALIB] Last SCAN: %d dots\n", dot_count);
        for (int i = 0; i < dot_count; i++)
            Serial.printf("  Dot %d: pixel(%u, %u) → (%+.0f, %+.0f) mm\n",
                          i, dot_px[i], dot_py[i],
                          calibrate_is_done() ? -100.0f : 0.0f,
                          calibrate_is_done() ? 100.0f : 0.0f);
        return;
    }

    if (line.equalsIgnoreCase("SOLVE")) {
        if (!scanned) {
            Serial.println("[CALIB] No SCAN data. Run SCAN first.");
            return;
        }
        if (dot_count < 4) {
            Serial.println("[CALIB] Not enough dots found. Re-SCAN.");
            return;
        }

        float rms;
        if (!calibrate_solve(dot_px, dot_py, calib_H, &rms)) {
            Serial.println("[CALIB] ERROR: solver failed.");
            return;
        }

        solved = true;
        Serial.println("[CALIB] Homography matrix H:");
        Serial.printf("  [%7.4f  %7.4f  %7.4f]\n", calib_H[0][0], calib_H[0][1], calib_H[0][2]);
        Serial.printf("  [%7.4f  %7.4f  %7.4f]\n", calib_H[1][0], calib_H[1][1], calib_H[1][2]);
        Serial.printf("  [%7.4f  %7.4f  %7.4f]\n", calib_H[2][0], calib_H[2][1], calib_H[2][2]);
        Serial.printf("[CALIB] RMS error: %.1f mm\n", rms);
        Serial.println("[CALIB] Send SAVE to persist, or re-SCAN for a better capture.");
        return;
    }

    if (line.equalsIgnoreCase("SAVE")) {
        if (!solved) {
            Serial.println("[CALIB] Nothing to save. Run SCAN then SOLVE first.");
            return;
        }
        calibrate_save_matrix(calib_H);
        return;
    }

    if (line.equalsIgnoreCase("ERASE")) {
        calibrate_erase();
        solved = false;
        scanned = false;
        return;
    }

    Serial.printf("[CALIB] Unknown command: %s\n", line.c_str());
    calibrate_print_help();
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

  captureToMatrix();
  Serial.println("First frame captured.");
  run_detection();

  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) { delay(500); Serial.print("."); }
  Serial.println("\nConnected!");
  Serial.printf("Open http://%s/ in your browser\n",
                WiFi.localIP().toString().c_str());

  server.on("/",        handleRoot);
  server.on("/capture", handleCapture);
  server.begin();
}

// ─── loop ─────────────────────────────────────────────────────────────────
void loop() {
  server.handleClient();

  bool wasCalib = inCalibMode;
  calibrate_handle_serial();

  if (inCalibMode || wasCalib) {
    if (inCalibMode)
      delay(10);
    return;
  }

  captureToMatrix();
  run_detection();
  delay(5000);
}
