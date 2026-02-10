//-----------1-----------
// ESP32-S3 CAM Zonio Controller - TASK-BASED STREAM
// Verze: 1.4.3-DEV-UNSTABLE - ESP32-S3 N16R8 with OV2640/OV5640 Camera
// Hardware: ESP32-S3-WROOM CAM (Freenove clone)
// Changes: Rebuild blocking single loop to FreeRTOS task for non-blocking stream on Core 0. Single core solution caused massive memory leak.
//          Memory leak partially fixed, unit dont brick itself after 1600s
//-----------------------

#include <WiFi.h>
#include <WiFiClient.h>
#include <PubSubClient.h>
#include <WebServer.h>
#include <DNSServer.h>
#include <Preferences.h>
#include <ArduinoJson.h>
#include <HTTPClient.h>
#include <Update.h>
#include "esp_camera.h"
#include "esp_timer.h"
#include "img_converters.h"
#include "fb_gfx.h"
#include "mbedtls/base64.h"

// ===== FIRMWARE VERSION =====
const char* FIRMWARE_VERSION = "1.4.3-TASK-S3-CAM";
const char* DEVICE_NAME_BASE = "ESP32S3-CAM-ZONIO";

// ===== DEBUG FLAGS =====
#define DEBUG_DIAGNOSTICS 1
#define DEBUG_TIMING 1
#define DIAG_INTERVAL_MS 10000

// ===== XOR ENCRYPTION KEY =====
const char* XOR_KEY = "MojeTajneHeslo1234567890";

// ===== BOOT BUTTON =====
#define BOOT_BUTTON_PIN 0

// ===== ESP32-S3-WROOM CAM PIN MAP =====
#define PWDN_GPIO_NUM    -1
#define RESET_GPIO_NUM   -1
#define XCLK_GPIO_NUM    15
#define SIOD_GPIO_NUM    4
#define SIOC_GPIO_NUM    5
#define Y9_GPIO_NUM      16
#define Y8_GPIO_NUM      17
#define Y7_GPIO_NUM      18
#define Y6_GPIO_NUM      12
#define Y5_GPIO_NUM      10
#define Y4_GPIO_NUM      8
#define Y3_GPIO_NUM      9
#define Y2_GPIO_NUM      11
#define VSYNC_GPIO_NUM   6
#define HREF_GPIO_NUM    7
#define PCLK_GPIO_NUM    13

// ===== LED PINS =====
#define LED_GPIO_NUM     48
#define FLASH_LED_PIN    2

// ===== GLOBAL OBJECTS =====
WebServer server(80);
WebServer streamServer(81);
DNSServer dnsServer;
Preferences preferences;
WiFiClient espClient;
PubSubClient mqtt(espClient);

// ===== RUNTIME VARS =====
String deviceId = "UNKNOWN";
String mqttPrefix = "zonio/cam/UNKNOWN";
bool apMode = false;
bool streamingEnabled = true;
bool flashEnabled = false;
unsigned long lastHeartbeat = 0;
unsigned long lastRegistration = 0;

// ===== CONFIGURATION VARS =====
String wifi_ssid = "";
String wifi_pass = "";
String mqtt_server = "";
int mqtt_port = 1883;
String mqtt_user = "";
String mqtt_pass = "";
String backend_url = "";

// Camera settings
int frameSize = FRAMESIZE_VGA;
int jpegQuality = 12;

// ===== STREAM TASK CONTROL =====
TaskHandle_t streamTaskHandle = NULL;
SemaphoreHandle_t streamMutex = NULL;
volatile bool streamTaskRunning = false;
volatile int activeStreamCount = 0;

//-----------1-----------

//-----------2-----------
// ===== HELPERS =====

void setFlash(bool state) {
  #if FLASH_LED_PIN >= 0
  digitalWrite(FLASH_LED_PIN, state ? HIGH : LOW);
  #endif
}

void setLed(bool state) {
  #if LED_GPIO_NUM >= 0
  digitalWrite(LED_GPIO_NUM, state ? HIGH : LOW);
  #endif
}

// NON-BLOCKING BLINK
int blinkCount = 0;
int blinkTarget = 0;
unsigned long blinkTimer = 0;
bool blinkState = false;

void startNonBlockingBlink(int times) {
  blinkCount = 0;
  blinkTarget = times;
  blinkTimer = millis();
  blinkState = true;
  setFlash(true);
}

void processNonBlockingBlink() {
  if (blinkTarget == 0 || blinkCount >= blinkTarget) return;
  if (millis() - blinkTimer > 100) {
    blinkState = !blinkState;
    setFlash(blinkState);
    if (!blinkState) blinkCount++;
    blinkTimer = millis();
  }
}

// ===== XOR ENCRYPTION =====
static uint8_t* xorBuffer = nullptr;
static const int XOR_BUFFER_SIZE = 512;
static uint8_t mqttDecryptBuffer[256];

void initXorBuffer() {
  xorBuffer = (uint8_t*)ps_malloc(XOR_BUFFER_SIZE);
}

void xorPayload(uint8_t* buffer, int len) {
  int keyLen = strlen(XOR_KEY);
  for (int i = 0; i < len; i++) {
    buffer[i] = buffer[i] ^ (uint8_t)XOR_KEY[i % keyLen];
  }
}

void publishEncrypted(const String& topic, const String& payload) {
  if (!mqtt.connected() || !xorBuffer) return;

  int payloadLen = payload.length();
  if (payloadLen == 0 || payloadLen >= XOR_BUFFER_SIZE) return;

  memcpy(xorBuffer, payload.c_str(), payloadLen);
  xorPayload(xorBuffer, payloadLen);
  mqtt.publish(topic.c_str(), xorBuffer, payloadLen, false);
}

String getChipId() {
  uint64_t chipid = ESP.getEfuseMac();
  uint32_t low = (uint32_t)chipid;
  char sn[16];
  snprintf(sn, 16, "S3-%08lX", low);
  return String(sn);
}

String getResolutionName() {
  switch (frameSize) {
    case FRAMESIZE_QQVGA: return "QQVGA";
    case FRAMESIZE_QVGA:  return "QVGA";
    case FRAMESIZE_VGA:   return "VGA";
    case FRAMESIZE_SVGA:  return "SVGA";
    case FRAMESIZE_XGA:   return "XGA";
    case FRAMESIZE_SXGA:  return "SXGA";
    case FRAMESIZE_UXGA:  return "UXGA";
    default: return "VGA";
  }
}

void wipeNVS() {
  preferences.begin("cam-config", false);
  preferences.clear();
  preferences.end();
  ESP.restart();
}

//-----------2-----------

//-----------3-----------
// ===== CAMERA INITIALIZATION =====

bool initCamera() {
  Serial.println("\n📷 Initializing ESP32-S3 camera...");
  
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
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;
  config.grab_mode = CAMERA_GRAB_LATEST;
  
  Serial.println("   ESP32-S3 with PSRAM");
  config.frame_size = (framesize_t)frameSize;
  config.jpeg_quality = jpegQuality;
  config.fb_count = 2;
  config.fb_location = CAMERA_FB_IN_PSRAM;

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("❌ Camera init failed: 0x%x\n", err);
    return false;
  }

  sensor_t * s = esp_camera_sensor_get();
  if (s) {
    Serial.printf("   Sensor PID: 0x%02X\n", s->id.PID);
    s->set_brightness(s, 0);
    s->set_contrast(s, 0);
    s->set_saturation(s, 0);
    s->set_whitebal(s, 1);
    s->set_awb_gain(s, 1);
    s->set_exposure_ctrl(s, 1);
    s->set_gain_ctrl(s, 1);
    s->set_hmirror(s, 0);
    s->set_vflip(s, 0);
  }

  Serial.println("✅ Camera initialized");
  Serial.printf("   Resolution: %s, Quality: %d\n", getResolutionName().c_str(), jpegQuality);
  
  return true;
}

//-----------3-----------

//-----------4-----------
// ===== STREAM TASK (RUNS ON CORE 0) =====

#define PART_BOUNDARY "123456789000000000000987654321"
static const char* STREAM_CONTENT_TYPE = "multipart/x-mixed-replace;boundary=" PART_BOUNDARY;
static const char* STREAM_BOUNDARY = "\r\n--" PART_BOUNDARY "\r\n";
static const char* STREAM_PART = "Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n";

void streamTask(void *parameter) {
  WiFiClient* client = (WiFiClient*)parameter;
  
  if (!client || !client->connected()) {
    Serial.println("⚠️ Stream task: Invalid client");
    vTaskDelete(NULL);
    return;
  }

  activeStreamCount++;
  streamTaskRunning = true;
  Serial.printf("📹 Stream task started on core %d (clients: %d)\n", xPortGetCoreID(), activeStreamCount);

  // Send HTTP headers
  client->println("HTTP/1.1 200 OK");
  client->printf("Content-Type: %s\r\n", STREAM_CONTENT_TYPE);
  client->println("Access-Control-Allow-Origin: *");
  client->println("X-Framerate: 10");
  client->println();

  unsigned long lastFrameTime = millis();
  unsigned long streamStartTime = millis();
  int frameCount = 0;
  
  // Pre-allocate buffers
  char part_buf[64];
  const size_t chunkSize = 4096;
  
  while (client->connected() && streamingEnabled) {
    // Timeout check
    if (!client->available() && (millis() - lastFrameTime > 10000)) {
      Serial.println("⚠️ Stream timeout - no activity");
      break;
    }

    camera_fb_t * fb = esp_camera_fb_get();
    if (!fb) {
      Serial.println("⚠️ Camera frame failed");
      vTaskDelay(100 / portTICK_PERIOD_MS);
      continue;
    }

    unsigned long writeStart = millis();
    bool writeOk = true;

    // Send boundary
    if (client->write(STREAM_BOUNDARY, strlen(STREAM_BOUNDARY)) != strlen(STREAM_BOUNDARY)) {
      writeOk = false;
    }

    // Send JPEG header
    if (writeOk) {
      snprintf(part_buf, 64, STREAM_PART, fb->len);
      if (client->write(part_buf, strlen(part_buf)) != strlen(part_buf)) {
        writeOk = false;
      }
    }

    // Send JPEG data in chunks
    if (writeOk) {
      size_t remaining = fb->len;
      size_t offset = 0;
      
      while (remaining > 0 && writeOk) {
        size_t toWrite = (remaining > chunkSize) ? chunkSize : remaining;
        size_t written = client->write(fb->buf + offset, toWrite);
        
        if (written != toWrite) {
          writeOk = false;
          break;
        }
        
        offset += written;
        remaining -= written;
        
        // Timeout protection
        if (millis() - writeStart > 2000) {
          Serial.println("⚠️ Write timeout");
          writeOk = false;
          break;
        }
        
        // Allow other tasks to run
        taskYIELD();
      }
    }

    esp_camera_fb_return(fb);

    if (!writeOk) {
      Serial.println("❌ Stream write failed");
      break;
    }

    lastFrameTime = millis();
    frameCount++;

    // Frame rate control (~10 FPS)
    vTaskDelay(100 / portTICK_PERIOD_MS);
  }

  unsigned long duration = (millis() - streamStartTime) / 1000;
  Serial.printf("📹 Stream ended: %d frames in %lu sec (%.1f fps)\n", 
                frameCount, duration, duration > 0 ? (float)frameCount / duration : 0);
  
  client->stop();
  delete client;
  
  activeStreamCount--;
  streamTaskRunning = false;
  
  vTaskDelete(NULL);
}

void handleStreamRequest() {
  if (!streamingEnabled) {
    streamServer.send(503, "text/plain", "Streaming disabled");
    return;
  }

  // Limit concurrent streams
  if (activeStreamCount >= 2) {
    streamServer.send(503, "text/plain", "Max clients reached");
    Serial.println("⚠️ Stream rejected: max clients");
    return;
  }

  // Create new client on heap (will be deleted by task)
  WiFiClient* client = new WiFiClient(streamServer.client());
  
  if (!client->connected()) {
    Serial.println("⚠️ Client not connected");
    delete client;
    streamServer.send(500, "text/plain", "Connection failed");
    return;
  }

  // Create stream task on Core 0 (loop runs on Core 1)
  xTaskCreatePinnedToCore(
    streamTask,           // Task function
    "streamTask",         // Name
    8192,                 // Stack size (8KB)
    (void*)client,        // Parameter (client pointer)
    1,                    // Priority
    &streamTaskHandle,    // Task handle
    0                     // Core 0
  );

  Serial.printf("📹 Stream task created (core 0)\n");
}

//-----------4-----------

//-----------5-----------
// ===== SNAPSHOT ENDPOINT =====

void handleCapture() {
  camera_fb_t * fb = esp_camera_fb_get();
  if (!fb) {
    streamServer.send(500, "text/plain", "Camera failed");
    return;
  }
  
  streamServer.sendHeader("Content-Disposition", "inline; filename=capture.jpg");
  streamServer.sendHeader("Access-Control-Allow-Origin", "*");
  streamServer.send_P(200, "image/jpeg", (const char *)fb->buf, fb->len);
  
  esp_camera_fb_return(fb);
}

//-----------5-----------

//-----------6-----------
// ===== BOOT BUTTON HANDLER =====

unsigned long bootButtonPressStart = 0;
bool bootButtonPressed = false;

void checkBootButton() {
  bool currentState = (digitalRead(BOOT_BUTTON_PIN) == LOW);
  
  if (currentState && !bootButtonPressed) {
    bootButtonPressStart = millis();
    bootButtonPressed = true;
    Serial.println("🔘 BOOT button pressed");
  }
  
  if (!currentState && bootButtonPressed) {
    unsigned long pressDuration = millis() - bootButtonPressStart;
    bootButtonPressed = false;
    
    if (pressDuration > 15000) {
      Serial.println("🔥 FACTORY RESET (15s hold)");
      startNonBlockingBlink(10);
      delay(1000);
      wipeNVS();
    } else if (pressDuration > 5000) {
      Serial.println("🔧 Entering AP Mode (5s hold)");
      startNonBlockingBlink(5);
      delay(500);
      ESP.restart();
    }
  }
}

//-----------6-----------

//-----------7-----------
// ===== CONFIGURATION WEB PORTAL =====

void startAPMode() {
  apMode = true;
  
  String apName = String(DEVICE_NAME_BASE) + "-" + deviceId;
  WiFi.mode(WIFI_AP);
  WiFi.softAP(apName.c_str());
  
  IPAddress IP = WiFi.softAPIP();
  Serial.printf("\n📡 AP Mode Started\n");
  Serial.printf("   SSID: %s\n", apName.c_str());
  Serial.printf("   IP: %s\n", IP.toString().c_str());
  
  dnsServer.start(53, "*", IP);
  
  server.on("/", HTTP_GET, handleRoot);
  server.on("/config", HTTP_POST, handleConfig);
  server.onNotFound(handleRoot);
  server.begin();
  
  Serial.println("   Config portal: http://192.168.4.1");
}

void handleRoot() {
  String html = "<!DOCTYPE html><html><head>";
  html += "<meta name='viewport' content='width=device-width,initial-scale=1'>";
  html += "<style>body{font-family:Arial;margin:20px;background:#f0f0f0}";
  html += ".container{max-width:500px;margin:auto;background:white;padding:20px;border-radius:8px;box-shadow:0 2px 4px rgba(0,0,0,0.1)}";
  html += "h1{color:#333;text-align:center}input,button{width:100%;padding:10px;margin:8px 0;box-sizing:border-box;border:1px solid #ddd;border-radius:4px}";
  html += "button{background:#007bff;color:white;border:none;cursor:pointer;font-size:16px}button:hover{background:#0056b3}";
  html += ".info{background:#e7f3ff;padding:10px;border-radius:4px;margin:10px 0;font-size:14px}</style></head><body>";
  html += "<div class='container'><h1>📷 ESP32-S3 CAM</h1>";
  html += "<div class='info'>Device ID: <b>" + deviceId + "</b><br>Version: " + String(FIRMWARE_VERSION) + "</div>";
  html += "<form action='/config' method='POST'>";
  html += "<input name='wifi_ssid' placeholder='WiFi SSID' value='" + wifi_ssid + "' required>";
  html += "<input name='wifi_pass' type='password' placeholder='WiFi Password' value='" + wifi_pass + "'>";
  html += "<input name='mqtt_server' placeholder='MQTT Server' value='" + mqtt_server + "' required>";
  html += "<input name='mqtt_port' placeholder='MQTT Port' value='" + String(mqtt_port) + "' required>";
  html += "<input name='mqtt_user' placeholder='MQTT Username' value='" + mqtt_user + "'>";
  html += "<input name='mqtt_pass' type='password' placeholder='MQTT Password' value='" + mqtt_pass + "'>";
  html += "<input name='backend_url' placeholder='Backend URL (optional)' value='" + backend_url + "'>";
  html += "<button type='submit'>💾 Save & Restart</button></form></div></body></html>";
  
  server.send(200, "text/html", html);
}

void handleConfig() {
  wifi_ssid = server.arg("wifi_ssid");
  wifi_pass = server.arg("wifi_pass");
  mqtt_server = server.arg("mqtt_server");
  mqtt_port = server.arg("mqtt_port").toInt();
  mqtt_user = server.arg("mqtt_user");
  mqtt_pass = server.arg("mqtt_pass");
  backend_url = server.arg("backend_url");
  
  saveSettings();
  
  String html = "<!DOCTYPE html><html><head><meta name='viewport' content='width=device-width,initial-scale=1'>";
  html += "<style>body{font-family:Arial;text-align:center;padding:50px;background:#f0f0f0}";
  html += ".success{background:white;padding:30px;border-radius:8px;display:inline-block;box-shadow:0 2px 4px rgba(0,0,0,0.1)}</style></head><body>";
  html += "<div class='success'><h1>✅ Saved!</h1><p>Device restarting...</p></div>";
  html += "<script>setTimeout(()=>window.location='/',3000)</script></body></html>";
  
  server.send(200, "text/html", html);
  delay(1000);
  ESP.restart();
}

void saveSettings() {
  preferences.begin("cam-config", false);
  preferences.putString("wifi_ssid", wifi_ssid);
  preferences.putString("wifi_pass", wifi_pass);
  preferences.putString("mqtt_server", mqtt_server);
  preferences.putInt("mqtt_port", mqtt_port);
  preferences.putString("mqtt_user", mqtt_user);
  preferences.putString("mqtt_pass", mqtt_pass);
  preferences.putString("backend_url", backend_url);
  preferences.putInt("frameSize", frameSize);
  preferences.putInt("jpegQuality", jpegQuality);
  preferences.end();
  Serial.println("💾 Settings saved to NVS");
}

void loadSettings() {
  preferences.begin("cam-config", true);
  wifi_ssid = preferences.getString("wifi_ssid", "");
  wifi_pass = preferences.getString("wifi_pass", "");
  mqtt_server = preferences.getString("mqtt_server", "");
  mqtt_port = preferences.getInt("mqtt_port", 1883);
  mqtt_user = preferences.getString("mqtt_user", "");
  mqtt_pass = preferences.getString("mqtt_pass", "");
  backend_url = preferences.getString("backend_url", "");
  frameSize = preferences.getInt("frameSize", FRAMESIZE_VGA);
  jpegQuality = preferences.getInt("jpegQuality", 12);
  preferences.end();
  
  Serial.println("📖 Settings loaded from NVS");
  if (wifi_ssid.length() > 0) {
    Serial.printf("   WiFi: %s\n", wifi_ssid.c_str());
    Serial.printf("   MQTT: %s:%d\n", mqtt_server.c_str(), mqtt_port);
  }
}

//-----------7-----------

//-----------8-----------
// ===== MQTT HANDLERS =====

void setupMQTT() {
  mqtt.setServer(mqtt_server.c_str(), mqtt_port);
  mqtt.setCallback(mqttCallback);
  mqtt.setBufferSize(512);
  mqtt.setKeepAlive(60);
  mqtt.setSocketTimeout(15);
  connectMQTT();
}

void connectMQTT() {
  if (WiFi.status() != WL_CONNECTED) return;
  if (mqtt.connected()) return;
  
  Serial.print("🔗 Connecting to MQTT... ");
  
  String clientId = deviceId + "-" + String(random(0xffff), HEX);
  
  bool connected;
  if (mqtt_user.length() > 0) {
    connected = mqtt.connect(clientId.c_str(), mqtt_user.c_str(), mqtt_pass.c_str());
  } else {
    connected = mqtt.connect(clientId.c_str());
  }
  
  if (connected) {
    Serial.println("✅ Connected");
    
    String controlTopic = mqttPrefix + "/control";
    mqtt.subscribe(controlTopic.c_str());
    
    publishStatus();
  } else {
    Serial.printf("❌ Failed (state: %d)\n", mqtt.state());
  }
}

void mqttCallback(char* topic, byte* payload, unsigned int length) {
  if (length > sizeof(mqttDecryptBuffer)) return;
  
  memcpy(mqttDecryptBuffer, payload, length);
  xorPayload(mqttDecryptBuffer, length);
  
  JsonDocument doc;
  DeserializationError error = deserializeJson(doc, mqttDecryptBuffer, length);
  
  if (error) {
    Serial.printf("❌ JSON parse error: %s\n", error.c_str());
    return;
  }
  
  const char* cmd = doc["cmd"];
  if (!cmd) return;
  
  Serial.printf("📬 MQTT: %s\n", cmd);
  
  if (strcmp(cmd, "flash") == 0) {
    bool state = doc["state"] | false;
    setFlash(state);
  }
  else if (strcmp(cmd, "stream") == 0) {
    streamingEnabled = doc["enabled"] | true;
    Serial.printf("   Stream: %s\n", streamingEnabled ? "ON" : "OFF");
  }
  else if (strcmp(cmd, "quality") == 0) {
    int q = doc["quality"] | 12;
    jpegQuality = constrain(q, 0, 63);
    sensor_t * s = esp_camera_sensor_get();
    if (s) s->set_quality(s, jpegQuality);
    Serial.printf("   Quality: %d\n", jpegQuality);
  }
  else if (strcmp(cmd, "resolution") == 0) {
    const char* res = doc["resolution"];
    if (res) {
      if (strcmp(res, "VGA") == 0) frameSize = FRAMESIZE_VGA;
      else if (strcmp(res, "SVGA") == 0) frameSize = FRAMESIZE_SVGA;
      else if (strcmp(res, "XGA") == 0) frameSize = FRAMESIZE_XGA;
      else if (strcmp(res, "SXGA") == 0) frameSize = FRAMESIZE_SXGA;
      
      sensor_t * s = esp_camera_sensor_get();
      if (s) s->set_framesize(s, (framesize_t)frameSize);
      Serial.printf("   Resolution: %s\n", getResolutionName().c_str());
    }
  }
  else if (strcmp(cmd, "restart") == 0) {
    Serial.println("🔄 Restart requested");
    ESP.restart();
  }
}

void publishStatus() {
  if (!mqtt.connected()) return;
  
  JsonDocument doc;
  doc["device_id"] = deviceId;
  doc["version"] = FIRMWARE_VERSION;
  doc["uptime"] = millis() / 1000;
  doc["heap"] = ESP.getFreeHeap();
  doc["psram"] = ESP.getFreePsram();
  doc["wifi_rssi"] = WiFi.RSSI();
  doc["resolution"] = getResolutionName();
  doc["quality"] = jpegQuality;
  doc["streaming"] = streamingEnabled;
  doc["active_streams"] = activeStreamCount;
  doc["ip"] = WiFi.localIP().toString();
  
  String statusTopic = mqttPrefix + "/status";
  String payload;
  serializeJson(doc, payload);
  
  publishEncrypted(statusTopic, payload);
}

//-----------8-----------

//-----------9-----------
// ===== STREAM SERVER SETUP =====

void setupStreamServer() {
  streamServer.on("/stream", HTTP_GET, handleStreamRequest);
  streamServer.on("/capture", HTTP_GET, handleCapture);
  streamServer.begin();
  Serial.println("📹 Stream server started on port 81");
}

// ===== BACKEND REGISTRATION =====
void registerWithBackend() {
  if (backend_url.length() == 0) return;
  if (WiFi.status() != WL_CONNECTED) return;
  
  HTTPClient http;
  http.setTimeout(5000);
  
  String url = backend_url + "/api/devices/register";
  
  if (!http.begin(url)) {
    Serial.println("⚠️ Backend registration: URL failed");
    return;
  }
  
  http.addHeader("Content-Type", "application/json");
  
  JsonDocument doc;
  doc["device_id"] = deviceId;
  doc["device_type"] = "camera";
  doc["version"] = FIRMWARE_VERSION;
  doc["ip"] = WiFi.localIP().toString();
  doc["stream_url"] = "http://" + WiFi.localIP().toString() + ":81/stream";
  doc["snapshot_url"] = "http://" + WiFi.localIP().toString() + ":81/capture";
  doc["chip"] = "ESP32-S3";
  
  String body;
  serializeJson(doc, body);
  
  int httpCode = http.POST(body);
  
  if (httpCode > 0) {
    Serial.printf("📡 Backend registered (HTTP %d)\n", httpCode);
  } else {
    Serial.printf("⚠️ Backend registration failed: %s\n", http.errorToString(httpCode).c_str());
  }
  
  http.end();
}

// ===== WIFI CONNECTION =====
bool connectWiFi() {
  if (wifi_ssid.length() == 0) return false;

  Serial.printf("📶 Connecting to: %s\n", wifi_ssid.c_str());
  
  WiFi.mode(WIFI_STA);
  WiFi.begin(wifi_ssid.c_str(), wifi_pass.c_str());

  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 30) {
    delay(500);
    Serial.print(".");
    attempts++;
    setLed(attempts % 2 == 0);
  }
  
  setLed(false);
  Serial.println();

  if (WiFi.status() == WL_CONNECTED) {
    Serial.printf("✅ Connected! IP: %s\n", WiFi.localIP().toString().c_str());
    return true;
  }
  Serial.println("❌ WiFi failed");
  return false;
}

//-----------9-----------

//-----------10-----------
// ===== MAIN FUNCTIONS =====

void setup() {
  Serial.begin(115200);
  delay(100);
  
  Serial.println("\n\n╔════════════════════════════════════════╗");
  Serial.println("║   📷 ESP32-S3 CAM ZONIO v1.2          ║");
  Serial.println("╚════════════════════════════════════════╝");
  Serial.printf("   Firmware: %s\n", FIRMWARE_VERSION);
  Serial.printf("   PSRAM: %lu MB\n", ESP.getPsramSize() / 1024 / 1024);
  
  // Init pins
  pinMode(BOOT_BUTTON_PIN, INPUT_PULLUP);
  #if FLASH_LED_PIN >= 0
  pinMode(FLASH_LED_PIN, OUTPUT);
  setFlash(false);
  #endif
  #if LED_GPIO_NUM >= 0
  pinMode(LED_GPIO_NUM, OUTPUT);
  setLed(false);
  #endif
  
  deviceId = getChipId();
  mqttPrefix = "zonio/cam/" + deviceId;
  Serial.printf("   Device ID: %s\n", deviceId.c_str());
  
  if (!initCamera()) {
    Serial.println("❌ Camera failed!");
  }
  
  loadSettings();
  initXorBuffer();
  
  // Create mutex for stream task coordination
  streamMutex = xSemaphoreCreateMutex();
  
  if (digitalRead(BOOT_BUTTON_PIN) == LOW) {
    Serial.println("🔧 BOOT button held on startup → AP Mode");
    delay(1000);
    if (digitalRead(BOOT_BUTTON_PIN) == LOW) {
      startAPMode();
      return;
    }
  }
  
  if (wifi_ssid.length() == 0 || mqtt_server.length() == 0) {
    Serial.println("⚠️ Not configured → AP Mode");
    startAPMode();
    return;
  }
  
  if (!connectWiFi()) {
    Serial.println("⚠️ WiFi failed → AP Mode");
    startAPMode();
    return;
  }
  
  setupMQTT();
  setupStreamServer();
  registerWithBackend();
  
  startNonBlockingBlink(3);
  
  Serial.println("\n✅ ESP32-S3 CAM READY");
  Serial.printf("   Stream: http://%s:81/stream\n", WiFi.localIP().toString().c_str());
  Serial.printf("   Snapshot: http://%s:81/capture\n", WiFi.localIP().toString().c_str());
  Serial.printf("   Loop running on Core: %d\n", xPortGetCoreID());
  Serial.println("\n💡 Tip: Hold BOOT 5-15s for AP mode, >15s for factory reset");
}

void loop() {
  #if DEBUG_TIMING
  static unsigned long maxMqttLoop = 0;
  static unsigned long totalMqttLoop = 0;
  static unsigned long timingCycles = 0;
  #endif
  
  checkBootButton();
  
  // Handle config portal in AP mode
  if (apMode) {
    dnsServer.processNextRequest();
    server.handleClient();
    processNonBlockingBlink();
    yield();
    return;
  }
  
  // Handle stream server requests (just accepts connections, task does the work)
  streamServer.handleClient();
  
  processNonBlockingBlink();
  yield();
  
  #if DEBUG_TIMING
  timingCycles++;
  #endif
  
  // WiFi watchdog
  static unsigned long lastWiFiCheck = 0;
  if (millis() - lastWiFiCheck > 60000) {
    if (WiFi.status() != WL_CONNECTED) {
      Serial.println("📶 WiFi lost, reconnecting...");
      WiFi.reconnect();
    }
    lastWiFiCheck = millis();
  }
  
  // MQTT handling
  #if DEBUG_TIMING
  unsigned long t4 = micros();
  #endif
  
  if (!mqtt.connected()) {
    static unsigned long lastReconnect = 0;
    if (millis() - lastReconnect > 5000) {
      connectMQTT();
      lastReconnect = millis();
    }
  } else {
    mqtt.loop();
  }
  
  #if DEBUG_TIMING
  unsigned long mqttTime = micros() - t4;
  if (mqttTime > maxMqttLoop) maxMqttLoop = mqttTime;
  totalMqttLoop += mqttTime;
  #endif
  
  // Heartbeat
  if (millis() - lastHeartbeat > 30000) {
    publishStatus();
    lastHeartbeat = millis();
  }
  
  // Backend registration
  if (millis() - lastRegistration > 300000) {
    registerWithBackend();
    lastRegistration = millis();
  }
  
  // ===== DIAGNOSTICS =====
  #if DEBUG_DIAGNOSTICS
  static unsigned long lastDiag = 0;
  static uint32_t minHeap = 0xFFFFFFFF;
  static uint32_t minPsram = 0xFFFFFFFF;
  static uint32_t loopCounter = 0;
  static unsigned long lastLoopCount = 0;
  loopCounter++;
  
  uint32_t currentHeap = ESP.getFreeHeap();
  uint32_t currentPsram = ESP.getFreePsram();
  if (currentHeap < minHeap) minHeap = currentHeap;
  if (currentPsram < minPsram) minPsram = currentPsram;
  
  if (millis() - lastDiag > DIAG_INTERVAL_MS) {
    unsigned long uptime = millis() / 1000;
    uint32_t loopsPerSec = (loopCounter - lastLoopCount) / (DIAG_INTERVAL_MS / 1000);
    
    Serial.println("\n╔══════════════════════════════════════════════════════╗");
    Serial.printf("║ 🔍 DIAG @ %lu s uptime                              ║\n", uptime);
    Serial.println("╠══════════════════════════════════════════════════════╣");
    Serial.printf("║ HEAP:     %3lu KB free (min: %3lu KB)                 ║\n", 
        (unsigned long)(currentHeap / 1024), (unsigned long)(minHeap / 1024));
    Serial.printf("║ PSRAM:   %4lu KB free (min: %4lu KB)                  ║\n", 
        (unsigned long)(currentPsram / 1024), (unsigned long)(minPsram / 1024));
    Serial.println("╠══════════════════════════════════════════════════════╣");
    Serial.printf("║ WiFi:  RSSI=%d dBm, Status=%d                        ║\n", 
        WiFi.RSSI(), WiFi.status());
    Serial.printf("║ MQTT:  connected=%d, state=%d                        ║\n", 
        mqtt.connected(), mqtt.state());
    Serial.printf("║ Stream tasks: %d active                              ║\n", 
        activeStreamCount);
    Serial.printf("║ Loop:  %lu iter/s (Core %d)                          ║\n", 
        (unsigned long)loopsPerSec, xPortGetCoreID());
    
    #if DEBUG_TIMING
    Serial.println("╠══════════════════════════════════════════════════════╣");
    Serial.println("║ ⏱️  TIMING (microseconds):                           ║");
    Serial.printf("║   mqttLoop:      avg=%lu, max=%lu us                 ║\n",
        timingCycles > 0 ? totalMqttLoop / timingCycles : 0, maxMqttLoop);
    
    maxMqttLoop = 0;
    totalMqttLoop = 0;
    timingCycles = 0;
    #endif
    
    Serial.println("╚══════════════════════════════════════════════════════╝");
    
    if (currentHeap < 50000) {
      Serial.println("⚠️ WARNING: HEAP CRITICALLY LOW!");
    }
    if (loopsPerSec < 1000 && activeStreamCount == 0) {
      Serial.println("⚠️ WARNING: LOOP RATE SLOW WITHOUT ACTIVE STREAMS!");
    }
    
    lastDiag = millis();
    lastLoopCount = loopCounter;
  }
  #endif
}

//-----------10-----------
