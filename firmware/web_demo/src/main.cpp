#include "board_config.h"
#include "calibrate.h"
#include "color_detect.h"
#include "config.h"
#include "esp_camera.h"
#include "img_converters.h"
#include "kinematics.h"
#include "types.h"
#include <WebServer.h>
#include <WiFi.h>

// ─── NVS bridge stubs ─────────────────────────────────────────────────────
// calibrate.c references these via extern "C".  No persistence needed here.
extern "C" {
bool calibrate_nvs_load(float H[3][3]) {
  (void)H;
  return false;
}
void calibrate_nvs_save(const float H[3][3]) { (void)H; }
void calibrate_nvs_erase(void) {}
}

// ─── WiFi credentials ─━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
// Fill in your home WiFi name and password before flashing.
#define WIFI_SSID "Ali's Room"
#define WIFI_PASS "MAAA2011@2011"

// ─── Global state ─────────────────────────────────────────────────────────
WebServer server(80);

static const char INDEX_HTML[] =
    "<!DOCTYPE html>"
    "<html><head><meta charset='utf-8'>"
    "<title>ESP32-CAM Demo</title>"
    "<style>"
    "body{font-family:sans-serif;text-align:center;margin:20px;background:#"
    "f5f5f5}"
    "img{max-width:90vw;border:2px solid #333;border-radius:4px}"
    "button{font-size:18px;padding:10px 30px;margin:10px;cursor:pointer}"
    ".info{font-size:14px;color:#666}"
    "</style></head><body>"
    "<h2>ESP32-CAM &mdash; Calibration + Detection</h2>"
    "<img src='/capture' id='frame'>"
    "<br><button onclick=\"var "
    "i=document.getElementById('frame');i.src='/capture?r='+Date.now()\">"
    "Refresh</button>"
    "<p class='info'>Auto-refreshes every 5 s</p>"
    "<script>setInterval(function(){"
    "var i=document.getElementById('frame');i.src='/capture?r='+Date.now()"
    "},5000)</script>"
    "</body></html>";

// ─── Utility: decode one RGB565 word → Pixel ─────────────────────────────
static Pixel decodeRGB565(const uint8_t *buf, int row, int col) {
  int idx = (row * IMG_WIDTH + col) * 2;
  uint16_t word = ((uint16_t)buf[idx] << 8) | buf[idx + 1];
  Pixel p;
  p.r = (uint8_t)((word >> 11) & 0x1F) << 3;
  p.g = (uint8_t)((word >> 5) & 0x3F) << 2;
  p.b = (uint8_t)(word & 0x1F) << 3;
  return p;
}

// ─── Utility: green crosshair at pixel (cx, cy) ──────────────────────────
static void draw_crosshair(Pixel *pixels, int w, int h, int cx, int cy) {
  int size = 6;
  for (int d = -size; d <= size; d++) {
    int px = cx + d, py = cy;
    if (px >= 0 && px < w) {
      Pixel *p = &pixels[py * w + px];
      p->r = 0;
      p->g = 255;
      p->b = 0;
    }
  }
  for (int d = -size; d <= size; d++) {
    int px = cx, py = cy + d;
    if (py >= 0 && py < h) {
      Pixel *p = &pixels[py * w + px];
      p->r = 0;
      p->g = 255;
      p->b = 0;
    }
  }
}

// ─── Utility: apply homography H to pixel (u,v) → mm (x,y) ───────────────
static void apply_h(const float H[3][3], uint32_t u, uint32_t v, float *x,
                    float *y) {
  float w = H[2][0] * (float)u + H[2][1] * (float)v + 1.0f;
  *x = (H[0][0] * (float)u + H[0][1] * (float)v + H[0][2]) / w;
  *y = (H[1][0] * (float)u + H[1][1] * (float)v + H[1][2]) / w;
}

// ─── Camera init (identical to main firmware) ────────────────────────────
static bool initCamera() {
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sccb_sda = SIOD_GPIO_NUM;
  config.pin_sccb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;

  config.xclk_freq_hz = 10000000;
  config.pixel_format = PIXFORMAT_RGB565;
  config.frame_size = FRAMESIZE_QVGA;
  config.grab_mode = CAMERA_GRAB_WHEN_EMPTY;
  config.fb_location = CAMERA_FB_IN_PSRAM;
  config.fb_count = 1;
  config.jpeg_quality = 12;

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("[CAM] Init failed: 0x%x\n", err);
    return false;
  }

  sensor_t *s = esp_camera_sensor_get();
  s->set_vflip(s, 1);
  s->set_brightness(s, 1);
  s->set_saturation(s, -1);
  return true;
}

// ─── /capture handler: capture → calibrate → detect → annotate → JPEG ───
static void handleCapture() {
  uint32_t t0 = millis();

  // 1. Capture RGB565 frame
  camera_fb_t *fb = esp_camera_fb_get();
  uint32_t t1 = millis();
  if (!fb) {
    server.send(500, "text/plain", "Capture failed");
    return;
  }
  Serial.printf("[DEMO] Capture: %u ms  (%dx%d, %u bytes RAW)\n",
                (unsigned)(t1 - t0), fb->width, fb->height, fb->len);

  // 2. Convert to Pixel array (RGB888) in PSRAM + brightness stats
  size_t npixels = (size_t)IMG_WIDTH * IMG_HEIGHT;
  Pixel *pixels =
      (Pixel *)heap_caps_malloc(npixels * sizeof(Pixel), MALLOC_CAP_SPIRAM);
  if (!pixels) {
    esp_camera_fb_return(fb);
    server.send(500, "text/plain", "PSRAM alloc failed");
    return;
  }
  int min_b = 255, max_b = 0;
  for (int row = 0; row < IMG_HEIGHT; row++) {
    for (int col = 0; col < IMG_WIDTH; col++) {
      Pixel p = decodeRGB565(fb->buf, row, col);
      pixels[row * IMG_WIDTH + col] = p;
      int b = ((int)p.r + (int)p.g + (int)p.b) / 3;
      if (b < min_b)
        min_b = b;
      if (b > max_b)
        max_b = b;
    }
  }
  esp_camera_fb_return(fb);
  uint32_t t2 = millis();
  int abs_thresh = min_b + (max_b - min_b) / 3;
  Serial.printf("[DEMO] Convert: %u ms  brightness %d..%d  abs_thresh=%d\n",
                (unsigned)(t2 - t1), min_b, max_b, abs_thresh);
  Serial.printf("[DEMO] PSRAM: allocated %u bytes for Pixel array\n",
                (unsigned)(npixels * sizeof(Pixel)));

  // 3. Calibration dot detection
  uint32_t dot_ux[4], dot_uy[4];
  int ndots =
      calibrate_find_dots(pixels, IMG_WIDTH, IMG_HEIGHT, dot_ux, dot_uy);
  uint32_t t3 = millis();
  Serial.printf("[DEMO] Calibration: %u ms  dots=%d/4\n", (unsigned)(t3 - t2),
                ndots);
  for (int i = 0; i < ndots; i++)
    Serial.printf("  Dot %d: pixel(%u,%u)\n", i, dot_ux[i], dot_uy[i]);

  // 4. Solve homography if we have 4 dots
  float H[3][3] = {{0}};
  float rms = 0;
  bool have_h = false;
  if (ndots >= 4) {
    have_h = calibrate_solve(dot_ux, dot_uy, H, &rms);
    if (have_h) {
      Serial.printf("[DEMO] Homography RMS: %.1f mm\n", rms);
      Serial.printf(
          "  H = [%8.4f %8.4f %8.4f; %8.4f %8.4f %8.4f; %8.4f %8.4f %8.4f]\n",
          H[0][0], H[0][1], H[0][2], H[1][0], H[1][1], H[1][2], H[2][0],
          H[2][1], H[2][2]);
      calibrate_save_matrix(H); // make internal H available for kinematics
    }
  } else {
    Serial.println(
        "[DEMO] Not enough dots for homography. Using pixel coords only.");
  }

  // 5. Draw crosshairs at calibration dot positions
  for (int i = 0; i < ndots; i++)
    draw_crosshair(pixels, IMG_WIDTH, IMG_HEIGHT, (int)dot_ux[i],
                   (int)dot_uy[i]);

  // 6. Detect colored blocks and draw centroids
  static const Color colors[] = {COLOR_RED, COLOR_GREEN, COLOR_BLUE,
                                 COLOR_YELLOW};
  static const char *names[] = {"RED", "GREEN", "BLUE", "YELLOW"};
  int found = 0;

  for (int i = 0; i < 4; i++) {
    DetectionResult r =
        color_detect_scan_pixels(pixels, IMG_WIDTH, IMG_HEIGHT, colors[i]);
    if (!r.found) {
      Serial.printf("[DEMO] %s: not found\n", names[i]);
      continue;
    }
    found++;

    draw_centroid(pixels, IMG_WIDTH, IMG_HEIGHT, (int)r.centroid_x,
                  (int)r.centroid_y, colors[i]);

    if (have_h) {
      float mm_x, mm_y;
      apply_h(H, r.centroid_x, r.centroid_y, &mm_x, &mm_y);
      Serial.printf("[DEMO] %s  centroid pixel(%3u,%3u)  mm(%7.1f,%7.1f)\n",
                    names[i], r.centroid_x, r.centroid_y, mm_x, mm_y);
    } else {
      Serial.printf("[DEMO] %s  centroid pixel(%3u,%3u)\n", names[i],
                    r.centroid_x, r.centroid_y);
    }
  }

  uint32_t t4 = millis();
  if (found == 0)
    Serial.println("[DEMO] No colored blocks detected.");
  else
    Serial.printf("[DEMO] Detection: %u ms  found %d color(s)\n",
                  (unsigned)(t4 - t3), found);

  // 7. Convert annotated Pixel array to JPEG
  uint8_t *jpg_buf = NULL;
  size_t jpg_len = 0;
  bool ok = fmt2jpg((uint8_t *)pixels, npixels * sizeof(Pixel), IMG_WIDTH,
                    IMG_HEIGHT, PIXFORMAT_RGB888, 50, &jpg_buf, &jpg_len);
  free(pixels);
  uint32_t t5 = millis();

  if (!ok || !jpg_buf) {
    server.send(500, "text/plain", "JPEG conversion failed");
    Serial.println("[DEMO] JPEG conversion FAILED");
    return;
  }

  Serial.printf("[DEMO] JPEG encode: %u ms  size=%u bytes\n",
                (unsigned)(t5 - t4), (unsigned)jpg_len);

  // 8. Send HTTP response with JPEG binary
  WiFiClient cl = server.client();
  cl.printf("HTTP/1.1 200 OK\r\n"
            "Content-Type: image/jpeg\r\n"
            "Content-Length: %u\r\n"
            "Cache-Control: no-cache\r\n"
            "Connection: close\r\n\r\n",
            (unsigned)jpg_len);
  cl.write(jpg_buf, jpg_len);
  cl.flush();
  free(jpg_buf);

  uint32_t t6 = millis();
  Serial.printf("[DEMO] Serve: %u ms  TOTAL: %u ms\n", (unsigned)(t6 - t5),
                (unsigned)(t6 - t0));
  Serial.println("[DEMO] --- done ---\n");
}

// ─── setup ────────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("\n═══════════════════════════════════════════");
  Serial.println("  ESP32-CAM Calibration + Detection Demo");
  Serial.println("═══════════════════════════════════════════\n");

  // PSRAM info
  Serial.printf("[SYS] PSRAM total: %u bytes  free: %u bytes\n",
                (unsigned)ESP.getPsramSize(), (unsigned)ESP.getFreePsram());

  if (!initCamera()) {
    Serial.println("[DEMO] FATAL: Camera init failed. Halt.");
    while (1)
      delay(100);
  }

  // Camera sensor info
  Serial.printf("[CAM] Resolution: %dx%d  Pixel format: RGB565\n", IMG_WIDTH,
                IMG_HEIGHT);

  // warm‑up frames
  uint32_t tw = millis();
  camera_fb_t *w1 = esp_camera_fb_get();
  esp_camera_fb_return(w1);
  camera_fb_t *w2 = esp_camera_fb_get();
  esp_camera_fb_return(w2);
  Serial.printf("[CAM] Warm-up frames: %u ms\n", (unsigned)(millis() - tw));

  calibrate_init();

  // WiFi station (DHCP)
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  Serial.printf("[WIFI] Connecting to %s", WIFI_SSID);
  uint32_t twifi = millis();
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.printf("  (%u ms)\n", (unsigned)(millis() - twifi));
  Serial.printf("[WIFI] Connected.  IP: %s  RSSI: %d dBm\n",
                WiFi.localIP().toString().c_str(), (int)WiFi.RSSI());

  // HTTP server
  server.on("/", HTTP_GET, []() { server.send(200, "text/html", INDEX_HTML); });
  server.on("/capture", HTTP_GET, handleCapture);
  server.begin();
  Serial.println("[DEMO] HTTP server ready on port 80");
  Serial.printf("[DEMO] Open http://%s/ in your browser\n",
                WiFi.localIP().toString().c_str());
}

// ─── loop ─────────────────────────────────────────────────────────────────
void loop() {
  server.handleClient();
  delay(10);
}
