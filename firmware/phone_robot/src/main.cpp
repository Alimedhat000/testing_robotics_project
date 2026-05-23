#include "calibrate.h"
#include "color_detect.h"
#include "config.h"
#include "esp_camera.h"
#include "img_converters.h"
#include "kinematics.h"
#include "servo_control.h"
#include "types.h"
#include <Arduino.h>
#include <WebServer.h>
#include <WiFi.h>

// ─── WiFi credentials ─────────────────────────────────────────────────────
#define WIFI_SSID "Ali's Room"
#define WIFI_PASS "MAAA2011@2011"

// ─── NVS bridge stubs ─────────────────────────────────────────────────────
extern "C" {
bool calibrate_nvs_load(float H[3][3]) {
  (void)H;
  return false;
}
void calibrate_nvs_save(const float H[3][3]) { (void)H; }
void calibrate_nvs_erase(void) {}
}

// ─── Global state ─────────────────────────────────────────────────────────
WebServer server(80);

static uint8_t *upload_jpg = NULL;
static size_t upload_len = 0;
static bool upload_ready = false;
static bool upload_rejected = false;

#define MAX_UPLOAD_JPEG (120 * 1024)

typedef struct {
  bool have_h;
  int ndots;
  uint32_t dot_ux[4], dot_uy[4];
  float H[3][3];
  float rms;
  struct {
    bool found;
    uint32_t px, py;
    float mm_x, mm_y;
  } colors[4];
} ProcessResult;

static ProcessResult g_res;
static bool g_has_results = false;

// JPEG decode buffer (RGB565)
static uint8_t *decode_buf = NULL;
static size_t decode_buf_len = 0;

// ─── apply homography ────────────────────────────────────────────────────
static void apply_h(const float H[3][3], uint32_t u, uint32_t v, float *x,
                    float *y) {
  float w = H[2][0] * (float)u + H[2][1] * (float)v + 1.0f;
  *x = (H[0][0] * (float)u + H[0][1] * (float)v + H[0][2]) / w;
  *y = (H[1][0] * (float)u + H[1][1] * (float)v + H[1][2]) / w;
}

// ─── serve JPEG helper (forward decl) ────────────────────────────────────
static void serve_jpeg(uint8_t *buf, size_t len);

// ─── green crosshair ─────────────────────────────────────────────────────
static void draw_crosshair(Pixel *pixels, int w, int h, int cx, int cy) {
  for (int d = -6; d <= 6; d++) {
    int px = cx + d, py = cy;
    if (px >= 0 && px < w) {
      Pixel *p = &pixels[py * w + px];
      p->r = 0;
      p->g = 255;
      p->b = 0;
    }
  }
  for (int d = -6; d <= 6; d++) {
    int px = cx, py = cy + d;
    if (py >= 0 && py < h) {
      Pixel *p = &pixels[py * w + px];
      p->r = 0;
      p->g = 255;
      p->b = 0;
    }
  }
}

// ─── file upload handler ─────────────────────────────────────────────────
static void handleUpload() {
  HTTPUpload &up = server.upload();
  if (up.status == UPLOAD_FILE_START) {
    upload_ready = false;
    upload_rejected = false;
    upload_len = 0;
    if (upload_jpg) {
      free(upload_jpg);
      upload_jpg = NULL;
    }
    size_t cap = up.totalSize > 0 ? up.totalSize : MAX_UPLOAD_JPEG;
    if (cap > MAX_UPLOAD_JPEG) {
      upload_rejected = true;
      cap = MAX_UPLOAD_JPEG;
    }
    upload_jpg = (uint8_t *)heap_caps_malloc(cap + 1, MALLOC_CAP_SPIRAM);
  } else if (up.status == UPLOAD_FILE_WRITE && upload_jpg) {
    if (upload_rejected) return;
    if (upload_len + up.currentSize > MAX_UPLOAD_JPEG) {
      upload_rejected = true;
      return;
    }
    memcpy(upload_jpg + upload_len, up.buf, up.currentSize);
    upload_len += up.currentSize;
  } else if (up.status == UPLOAD_FILE_END && upload_jpg) {
    if (upload_rejected) {
      upload_ready = false;
      Serial.printf("[UPL] Rejected upload (%u bytes)\n", (unsigned)upload_len);
      return;
    }
    upload_jpg[upload_len] = 0;
    upload_ready = true;
    Serial.printf("[UPL] Received %u bytes JPEG\n", (unsigned)upload_len);
  }
}

// ─── /process handler ────────────────────────────────────────────────────
static void handleProcess() {
  if (upload_rejected) {
    upload_rejected = false;
    server.send(413, "text/plain", "Image too large. Use Capture or downscale before upload.");
    return;
  }
  if (!upload_ready || !upload_jpg || upload_len == 0) {
    server.send(400, "text/plain", "No image");
    return;
  }

  size_t npixels = (size_t)IMG_WIDTH * IMG_HEIGHT;
  Pixel *pixels =
      (Pixel *)heap_caps_malloc(npixels * sizeof(Pixel), MALLOC_CAP_SPIRAM);
  if (!pixels) {
    server.send(500, "text/plain", "PSRAM alloc failed");
    free(upload_jpg);
    upload_jpg = NULL;
    upload_ready = false;
    return;
  }

  // JPEG -> RGB565 first, then expand to RGB888 Pixel
  size_t rgb565_len = npixels * 2;
  if (!decode_buf || decode_buf_len < rgb565_len) {
    if (decode_buf) { free(decode_buf); decode_buf = NULL; }
    decode_buf = (uint8_t *)heap_caps_malloc(rgb565_len, MALLOC_CAP_SPIRAM);
    decode_buf_len = decode_buf ? rgb565_len : 0;
  }
  if (!decode_buf) {
    server.send(500, "text/plain", "PSRAM alloc failed (decode buffer)");
    free(upload_jpg);
    upload_jpg = NULL;
    upload_ready = false;
    free(pixels);
    return;
  }

  if (!jpg2rgb565(upload_jpg, upload_len, decode_buf, JPG_SCALE_NONE)) {
    server.send(500, "text/plain", "JPEG decode failed");
    free(upload_jpg);
    upload_jpg = NULL;
    upload_ready = false;
    free(pixels);
    return;
  }
  free(upload_jpg);
  upload_jpg = NULL;
  upload_ready = false;

  for (size_t i = 0; i < npixels; i++) {
    uint16_t word = ((uint16_t)decode_buf[i * 2 + 1] << 8) | decode_buf[i * 2];
    Pixel p;
    p.r = (uint8_t)((word >> 11) & 0x1F) << 3;
    p.g = (uint8_t)((word >>  5) & 0x3F) << 2;
    p.b = (uint8_t)( word        & 0x1F) << 3;
    pixels[i] = p;
  }

  Serial.printf("[PROC] Decoded %ux%u (RGB565->RGB888)\n", IMG_WIDTH, IMG_HEIGHT);

  // Brightness
  int mn = 255, mx = 0;
  for (size_t i = 0; i < npixels; i++) {
    int b = ((int)pixels[i].r + (int)pixels[i].g + (int)pixels[i].b) / 3;
    if (b < mn)
      mn = b;
    if (b > mx)
      mx = b;
  }
  Serial.printf("[PROC] Brightness %d..%d  abs_thresh=%d\n", mn, mx,
                mn + (mx - mn) / 3);

  // Calibration
  uint32_t du[4], dv[4];
  int ndots = calibrate_find_dots(pixels, IMG_WIDTH, IMG_HEIGHT, du, dv);
  Serial.printf("[PROC] Dots: %d/4\n", ndots);
  for (int i = 0; i < ndots; i++)
    Serial.printf("  Dot %d: pixel(%u,%u)\n", i, du[i], dv[i]);

  float H[3][3] = {{0}};
  float rms = 0;
  bool have_h = false;
  if (ndots >= 4) {
    have_h = calibrate_solve(du, dv, H, &rms);
    if (have_h) {
      Serial.printf("[PROC] H RMS: %.1f mm\n", rms);
      calibrate_save_matrix(H);
    }
  } else
    Serial.println("[PROC] Not enough dots.");

  for (int i = 0; i < ndots; i++)
    draw_crosshair(pixels, IMG_WIDTH, IMG_HEIGHT, (int)du[i], (int)dv[i]);

  g_res.have_h = have_h;
  g_res.ndots = ndots;
  memcpy(g_res.dot_ux, du, sizeof(du));
  memcpy(g_res.dot_uy, dv, sizeof(dv));
  memcpy(g_res.H, H, sizeof(H));
  g_res.rms = rms;

  // Color detection
  static const Color cols[] = {COLOR_RED, COLOR_GREEN, COLOR_BLUE,
                               COLOR_YELLOW};
  static const char *cn[] = {"RED", "GREEN", "BLUE", "YELLOW"};
  int nf = 0;
  for (int i = 0; i < 4; i++) {
    DetectionResult r =
        color_detect_scan_pixels(pixels, IMG_WIDTH, IMG_HEIGHT, cols[i]);
    g_res.colors[i].found = r.found;
    if (!r.found) {
      Serial.printf("[PROC] %s: -\n", cn[i]);
      continue;
    }
    nf++;
    g_res.colors[i].px = r.centroid_x;
    g_res.colors[i].py = r.centroid_y;
    draw_centroid(pixels, IMG_WIDTH, IMG_HEIGHT, (int)r.centroid_x,
                  (int)r.centroid_y, cols[i]);
    if (have_h) {
      apply_h(H, r.centroid_x, r.centroid_y, &g_res.colors[i].mm_x,
              &g_res.colors[i].mm_y);
      Serial.printf("[PROC] %s pixel(%u,%u) mm(%.1f,%.1f)\n", cn[i],
                    r.centroid_x, r.centroid_y, g_res.colors[i].mm_x,
                    g_res.colors[i].mm_y);
    } else
      Serial.printf("[PROC] %s pixel(%u,%u)\n", cn[i], r.centroid_x,
                    r.centroid_y);
  }
  Serial.printf("[PROC] %d color(s)\n", nf);
  g_has_results = true;

  // Annotated JPEG
  uint8_t *jo = NULL;
  size_t jl = 0;
  bool ok = fmt2jpg((uint8_t *)pixels, npixels * sizeof(Pixel), IMG_WIDTH,
                    IMG_HEIGHT, PIXFORMAT_RGB888, 50, &jo, &jl);
  free(pixels);
  if (!ok || !jo) {
    server.send(500, "text/plain", "JPEG encode failed");
    return;
  }
  Serial.printf("[PROC] Annotated JPEG: %u bytes\n", (unsigned)jl);
  serve_jpeg(jo, jl);
  free(jo);
  Serial.println("[PROC] Done\n");
}

// ─── /results handler ────────────────────────────────────────────────────
static void handleResults() {
  if (!g_has_results) {
    server.send(200, "application/json", "{\"ready\":false}");
    return;
  }
  String j = "{\"ready\":true,\"dots\":" + String(g_res.ndots) +
             ",\"have_h\":" + String(g_res.have_h ? "true" : "false") +
             ",\"colors\":[";
  const char *cn[] = {"RED", "GREEN", "BLUE", "YELLOW"};
  for (int i = 0; i < 4; i++) {
    if (i)
      j += ",";
    j += "{\"name\":\"" + String(cn[i]) +
         "\",\"found\":" + String(g_res.colors[i].found ? "true" : "false");
    if (g_res.colors[i].found) {
      j += ",\"px\":" + String(g_res.colors[i].px) +
           ",\"py\":" + String(g_res.colors[i].py);
      if (g_res.have_h)
        j += ",\"mm_x\":" + String(g_res.colors[i].mm_x, 1) +
             ",\"mm_y\":" + String(g_res.colors[i].mm_y, 1);
    }
    j += "}";
  }
  j += "]}";
  server.send(200, "application/json", j);
}

// ─── /pick handler ───────────────────────────────────────────────────────
static void handlePick() {
  if (!g_has_results || !g_res.have_h) {
    server.send(400, "text/plain",
                "No valid results to pick from. Capture first.");
    return;
  }
  server.send(200, "text/plain", "Picking...\n");
  server.client().flush();

  const char *cn[] = {"RED", "GREEN", "BLUE", "YELLOW"};
  static const uint32_t bins[4][2] = {{RED_BIN_PX_X, RED_BIN_PX_Y},
                                      {GREEN_BIN_PX_X, GREEN_BIN_PX_Y},
                                      {BLUE_BIN_PX_X, BLUE_BIN_PX_Y},
                                      {0, 0}};

  for (int ci = 0; ci < 4; ci++) {
    if (!g_res.colors[ci].found)
      continue;
    if (ci == 3) {
      Serial.println("[PICK] YELLOW has no bin, skipping.");
      continue;
    }

    Serial.printf("[PICK] === %s ===\n", cn[ci]);

    float mm_x, mm_y;
    kinematics_pixel_to_mm(g_res.colors[ci].px, g_res.colors[ci].py, &mm_x,
                           &mm_y);
    ArmAngles blk = kinematics_solve_ik(mm_x, mm_y);

    int cmd[4];
    cmd[0] = (int)blk.base_deg;
    cmd[1] = (int)blk.shoulder_deg;
    cmd[2] = (int)blk.elbow_deg;
    cmd[3] = SERVO_GRIPPER_OPEN;
    Serial.printf("[PICK] Move to block  base=%d sh=%d el=%d grip=%d\n", cmd[0],
                  cmd[1], cmd[2], cmd[3]);
    servo_write_all(cmd);
    delay(300);

    Serial.println("[PICK] Grip");
    servo_write_single(3, SERVO_GRIPPER_CLOSE);
    delay(300);

    cmd[1] -= 20;
    if (cmd[1] < SERVO_SHOULDER_MIN)
      cmd[1] = SERVO_SHOULDER_MIN;
    cmd[2] += 15;
    if (cmd[2] > SERVO_ELBOW_MAX)
      cmd[2] = SERVO_ELBOW_MAX;
    Serial.println("[PICK] Lift");
    servo_write_all(cmd);
    delay(200);

    ArmAngles bin = kinematics_bin_angles(bins[ci][0], bins[ci][1]);
    cmd[0] = (int)bin.base_deg;
    cmd[1] = (int)bin.shoulder_deg;
    cmd[2] = (int)bin.elbow_deg;
    cmd[3] = SERVO_GRIPPER_CLOSE;
    Serial.printf("[PICK] Move to bin  base=%d sh=%d el=%d\n", cmd[0], cmd[1],
                  cmd[2]);
    servo_write_all(cmd);
    delay(300);

    Serial.println("[PICK] Release");
    servo_write_single(3, SERVO_GRIPPER_OPEN);
    delay(300);

    Serial.println("[PICK] Home");
    servo_home();
    delay(500);

    Serial.printf("[PICK] %s done\n", cn[ci]);
  }
  Serial.println("[PICK] All done\n");
}

// ─── HTML page ───────────────────────────────────────────────────────────
static const char PAGE[] = R"HTML(
<!DOCTYPE html>
<html>
<head>
  <meta charset="utf-8" />
  <meta name="viewport" content="width=device-width,initial-scale=1" />
  <title>Robot Controller</title>
  <style>
    body { font-family: sans-serif; margin: 10px; text-align: center; background: #eee; }
    video, canvas, img { max-width: 100%; border: 2px solid #555; border-radius: 6px; background: #fff; }
    button { font-size: 20px; padding: 14px 0; margin: 6px; width: 90%; max-width: 320px; border: 0; border-radius: 8px; color: #fff; }
    #cap { background: #e74c3c; }
    #pick { background: #27ae60; }
    #pick:disabled { background: #999; }
    #start { background: #2980b9; }
    #info { font-size: 14px; color: #555; margin: 6px; white-space: pre-wrap; }
    .hidden { display: none; }
  </style>
</head>
<body>
  <h2>Robot Controller</h2>
  <video id="v" autoplay playsinline class="hidden"></video>
  <canvas id="c" width="320" height="240" class="hidden"></canvas>
  <img id="r" class="hidden" />
  <div id="info">Tap Start Camera to begin</div>
  <button id="start">Start Camera</button>
  <label style="display:block;margin:10px">
    <input id="upload" type="file" accept="image/*" capture="environment" />
    Upload photo
  </label>
  <button id="cap" disabled>Capture</button>
  <button id="pick" disabled>Pick &amp; Place</button>

  <script>
    const v = document.getElementById('v');
    const c = document.getElementById('c');
    const r = document.getElementById('r');
    const info = document.getElementById('info');
    const start = document.getElementById('start');
    const cap = document.getElementById('cap');
    const pick = document.getElementById('pick');
    const upload = document.getElementById('upload');

    function showResults(j) {
      let t = `Dots: ${j.dots}/4`;
      if (j.ready) {
        j.colors.forEach(co => {
          t += `\n${co.name}: `;
          if (co.found) {
            t += `pixel(${co.px},${co.py})`;
            if (co.mm_x !== undefined) t += ` mm(${co.mm_x.toFixed(1)},${co.mm_y.toFixed(1)})`;
          } else {
            t += 'not found';
          }
        });
        pick.disabled = !(j.ready && j.have_h);
      } else {
        t += '\nNo results';
      }
      info.textContent = t;
    }

    async function sendAndShow(fd) {
      info.textContent = 'Processing...';
      const resp = await fetch('/process', { method: 'POST', body: fd });
      const blob = await resp.blob();
      r.src = URL.createObjectURL(blob);
      r.classList.remove('hidden');
      v.classList.add('hidden');
      c.classList.add('hidden');
      const meta = await fetch('/results').then(r => r.json());
      showResults(meta);
    }

    start.onclick = async () => {
      if (!navigator.mediaDevices) {
        info.textContent = 'Camera not available — use Upload or Firefox/phone.';
        return;
      }
      start.disabled = true;
      info.textContent = 'Starting camera...';
      try {
        const stream = await navigator.mediaDevices.getUserMedia({ video: { facingMode: 'environment', width: 320, height: 240 } });
        v.srcObject = stream;
        v.classList.remove('hidden');
        info.textContent = 'Camera ready';
        cap.disabled = false;
      } catch (e) {
        info.textContent = 'Camera error: ' + e;
        start.disabled = false;
      }
    };

    cap.onclick = () => {
      const ctx = c.getContext('2d');
      ctx.drawImage(v, 0, 0, 320, 240);
      c.toBlob(blob => {
        const fd = new FormData();
        fd.append('image', blob, 'cap.jpg');
        sendAndShow(fd).catch(err => info.textContent = 'Error: ' + err);
      }, 'image/jpeg', 0.7);
    };

    upload.onchange = async () => {
      if (!upload.files[0]) return;
      info.textContent = 'Resizing...';
      try {
        const bmp = await createImageBitmap(upload.files[0]);
        const ctx = c.getContext('2d');
        ctx.drawImage(bmp, 0, 0, 320, 240);
        if (bmp.close) bmp.close();
        c.toBlob(blob => {
          const fd = new FormData();
          fd.append('image', blob, 'upload.jpg');
          sendAndShow(fd).catch(err => info.textContent = 'Error: ' + err);
        }, 'image/jpeg', 0.7);
      } catch (e) {
        info.textContent = 'Upload error: ' + e;
      }
    };

    pick.onclick = async () => {
      pick.disabled = true;
      info.textContent = 'Moving arm...';
      try {
        await fetch('/pick', { method: 'POST' });
        info.textContent = 'Pick sequence complete';
      } catch (e) {
        info.textContent = 'Error: ' + e;
      }
    };
  </script>
</body>
</html>
)HTML";

// ─── serve jpeg to current client ────────────────────────────────────────
static void serve_jpeg(uint8_t *buf, size_t len) {
  WiFiClient cl = server.client();
  if (!cl)
    return;
  cl.printf("HTTP/1.1 200 OK\r\n"
            "Content-Type: image/jpeg\r\n"
            "Content-Length: %u\r\n"
            "Cache-Control: no-cache\r\n"
            "Connection: close\r\n\r\n",
            (unsigned)len);
  for (size_t s = 0; s < len; s += 1400)
    cl.write(buf + s, (len - s > 1400) ? 1400 : (len - s));
  cl.flush();
}

// ─── setup ───────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("\n===== Phone Robot =====");
  Serial.printf("PSRAM: %u free, %u total\n", (unsigned)ESP.getFreePsram(),
                (unsigned)ESP.getPsramSize());

  calibrate_init();

  servo_init();

  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  Serial.printf("WiFi: connecting to %s", WIFI_SSID);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.printf("\nWiFi: %s  RSSI: %d dBm\n", WiFi.localIP().toString().c_str(),
                (int)WiFi.RSSI());

  server.on("/", HTTP_GET, []() { server.send(200, "text/html", PAGE); });
  server.on("/process", HTTP_POST, handleProcess, handleUpload);
  server.on("/results", HTTP_GET, handleResults);
  server.on("/pick", HTTP_POST, handlePick);
  server.begin();
  Serial.printf("HTTP: http://%s/\n", WiFi.localIP().toString().c_str());
}

// ─── loop ─────────────────────────────────────────────────────────────────
void loop() {
  server.handleClient();
  delay(10);
}
