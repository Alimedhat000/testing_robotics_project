#include "esp_camera.h"
#include "board_config.h"
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"
#include "types.h"
#include <WiFi.h>
#include <WebServer.h>
#include "img_converters.h"
#include "color_detect.h"

// ─── WiFi credentials ─────────────────────────────────────────────────────
const char* ssid     = "Rwan'sHouse";
const char* password = "rwanshouse2024";

// ─── Global state ─────────────────────────────────────────────────────────
Pixel*     imageMatrix = nullptr;
WebServer  server(80);

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
//    fmt2jpg converts a raw RGB888 buffer to JPEG in one call.
//    We first convert our Pixel* (RGB888) straight into that function.
void handleCapture() {
  if (!imageMatrix) {
    server.send(503, "text/plain", "No frame captured yet");
    return;
  }

  // fmt2jpg needs a flat RGB888 buffer: [r,g,b, r,g,b, ...]
  // Our Pixel struct IS rgb888 laid out exactly that way, so we can
  // cast the pointer directly — no extra copy needed.
  uint8_t* jpegBuf = nullptr;
  size_t   jpegLen = 0;

  bool ok = fmt2jpg(
    (uint8_t*) imageMatrix,   // source: our RGB888 pixel matrix
    IMG_WIDTH * IMG_HEIGHT * 3,
    IMG_WIDTH,
    IMG_HEIGHT,
    PIXFORMAT_RGB888,         // tell the encoder what format the source is
    80,                       // JPEG quality (0–100, higher = better)
    &jpegBuf,
    &jpegLen
  );

  if (!ok || !jpegBuf) {
    server.send(500, "text/plain", "JPEG conversion failed");
    return;
  }

  // Send as a proper JPEG image — the browser renders it directly
  server.send_P(200, "image/jpeg", (const char*)jpegBuf, jpegLen);

  free(jpegBuf);   // fmt2jpg allocates with malloc, we free it
}

// ─── Root page: simple HTML with the image and a refresh button ───────────
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
            Serial.printf("%s centroid: (%u, %u)\n",
                          names[i], r.centroid_x, r.centroid_y);
            draw_centroid(imageMatrix, IMG_WIDTH, IMG_HEIGHT,
                          (int)r.centroid_x, (int)r.centroid_y, colors[i]);
        } else {
            Serial.printf("%s centroid: (none)\n", names[i]);
        }
    }
}

// ─── setup ────────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  delay(500);

  if (!initCamera()) return;
  if (!allocMatrix()) return;

  // Warm-up frames
  camera_fb_t* w1 = esp_camera_fb_get(); esp_camera_fb_return(w1);
  camera_fb_t* w2 = esp_camera_fb_get(); esp_camera_fb_return(w2);
  delay(100);

  // Capture and detect
  captureToMatrix();
  Serial.println("First frame captured.");
  run_detection();

  // Connect WiFi
  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) { delay(500); Serial.print("."); }
  Serial.println("\nConnected!");
  Serial.printf("Open http://%s/ in your browser\n",
                WiFi.localIP().toString().c_str());

  // Register routes
  server.on("/",        handleRoot);
  server.on("/capture", handleCapture);
  server.begin();
}

// ─── loop ─────────────────────────────────────────────────────────────────
void loop() {
  server.handleClient();

  captureToMatrix();
  run_detection();
  delay(5000);
}