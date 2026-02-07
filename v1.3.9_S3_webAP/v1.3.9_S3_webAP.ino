//-----------1-----------
// ESP32-S3 CAM Zonio Controller v1.1
// Verze: 1.3.9-S3-CAM - ESP32-S3 N16R8 with OV2640/OV5640 Camera
// Hardware: ESP32-S3-WROOM CAM (Freenove clone)
// Developed with Claude Opus 4.5 (Thinking)
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
#include "mbedtls/base64.h"  // For SSE Photo Stream encoding

// ===== FIRMWARE VERSION =====
const char* FIRMWARE_VERSION = "1.3.9-S3-CAM-ZONIO";
const char* DEVICE_NAME_BASE = "ESP32S3-CAM-ZONIO";

// ===== XOR ENCRYPTION KEY - MUST MATCH BACKEND! =====
const char* XOR_KEY = "MojeTajneHeslo1234567890";

// ===== BOOT BUTTON (pro AP mode a factory reset) =====
#define BOOT_BUTTON_PIN 0  // GPIO 0 = BOOT button na většině ESP32-S3

// ===== ESP32-S3-WROOM CAM PIN MAP (Freenove/Generic) =====
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
#define LED_GPIO_NUM     48   // Onboard LED (pokud existuje)
#define FLASH_LED_PIN    2    // External flash LED

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

// ===== CONFIGURATION VARS (Loaded from NVS) =====
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

// NON-BLOCKING BLINK - state machine
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

// ===== XOR ENCRYPTION HELPERS =====
// Static buffers in PSRAM - NO HEAP FRAGMENTATION
static uint8_t* xorBuffer = nullptr;
static const int XOR_BUFFER_SIZE = 512;
static uint8_t mqttDecryptBuffer[256];  // Static decrypt buffer

void initXorBuffer() {
  xorBuffer = (uint8_t*)ps_malloc(XOR_BUFFER_SIZE);  // Allocate once in PSRAM
}

void xorPayload(uint8_t* buffer, int len) {
  int keyLen = strlen(XOR_KEY);
  for (int i = 0; i < len; i++) {
    buffer[i] = buffer[i] ^ (uint8_t)XOR_KEY[i % keyLen];
  }
}

// BURST SHOT: Static buffer, no malloc
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

// ===== FACTORY RESET =====
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
  
  // ESP32-S3 N16R8 má 8MB PSRAM
  Serial.println("   ESP32-S3 with PSRAM");
  config.frame_size = (framesize_t)frameSize;
  config.jpeg_quality = jpegQuality;
  config.fb_count = 1;  // SINGLE BUFFER = always latest frame, no stale data
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
// ===== MJPEG STREAMING =====

// Flush camera buffer - discard stale frames
void flushCameraBuffer() {
  camera_fb_t * fb;
  for (int i = 0; i < 3; i++) {
    fb = esp_camera_fb_get();
    if (fb) esp_camera_fb_return(fb);
  }
}

#define PART_BOUNDARY "123456789000000000000987654321"
static const char* STREAM_CONTENT_TYPE = "multipart/x-mixed-replace;boundary=" PART_BOUNDARY;
static const char* STREAM_BOUNDARY = "\r\n--" PART_BOUNDARY "\r\n";
static const char* STREAM_PART = "Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n";

void handleStream() {
  if (!streamingEnabled) {
    streamServer.send(503, "text/plain", "Streaming disabled");
    return;
  }

  WiFiClient client = streamServer.client();
  client.setNoDelay(true);  // Disable Nagle - immediate send
  client.setTimeout(2000);  // 2s timeout to prevent blocking on network stall
  
  Serial.println("▶️ MJPEG Stream started");

  client.println("HTTP/1.1 200 OK");
  client.print("Content-Type: ");
  client.println(STREAM_CONTENT_TYPE);
  client.println("Access-Control-Allow-Origin: *");
  client.println("Connection: close");
  client.println();

  // FLUSH: Discard any stale frames before streaming
  flushCameraBuffer();

  unsigned long lastMqttLoop = 0;
  unsigned long lastFrame = 0;
  unsigned long streamStart = millis();
  unsigned long framesSent = 0;
  const unsigned long FRAME_INTERVAL = 50;
  const unsigned long STREAM_TIMEOUT = 300000;  // 5 min max - thermal protection

  while (client.connected()) {
    unsigned long now = millis();
    
    // Timeout protection - prevents thermal damage
    if (now - streamStart > STREAM_TIMEOUT) {
      Serial.println("🛑 Stream timeout (5 min)");
      break;
    }
    
    // Frame rate limiting
    if (now - lastFrame < FRAME_INTERVAL) {
      yield();
      continue;
    }
    
    camera_fb_t * fb = esp_camera_fb_get();
    if (!fb) {
      Serial.println("❌ Camera capture failed");
      break;
    }

    client.print(STREAM_BOUNDARY);
    char partBuf[64];
    snprintf(partBuf, 64, STREAM_PART, fb->len);
    client.print(partBuf);
    
    // Write frame data - if this fails/times out, loop breaks
    if (client.write(fb->buf, fb->len) == 0) {
      Serial.println("❌ Stream write failed (client disconnected?)");
      esp_camera_fb_return(fb);
      break;
    }
    
    esp_camera_fb_return(fb);
    lastFrame = now;
    framesSent++;

    // Periodic Health Check (every 20 frames ~ 1 sec)
    if (framesSent % 20 == 0) {
        Serial.printf("📊 MJPEG health: frames=%lu, heap=%u, psram=%u\n", 
          framesSent, ESP.getFreeHeap(), ESP.getFreePsram());
        
        // Blink LED briefly to show activity
        setLed(true);
        delay(1);
        setLed(false);
    }
    
    yield();
    if (now - lastMqttLoop > 500) {
      if (mqtt.connected()) mqtt.loop();
      lastMqttLoop = now;
    }
  }
  
  // CLEANUP: Close client only - DON'T disconnect MQTT!
  client.stop();
  Serial.println("⏹️ MJPEG Stream ended");
}

// ===== SSE PHOTO STREAM (3 FPS) =====
// Non-blocking alternative to MJPEG - sends base64 JPEG via Server-Sent Events
void handlePhotoStream() {
  if (!streamingEnabled) {
    streamServer.send(503, "text/plain", "Streaming disabled");
    return;
  }

  WiFiClient client = streamServer.client();
  client.setNoDelay(true);
  client.setTimeout(2000); // 2s timeout
  
  Serial.println("▶️ SSE Stream started");

  // SSE Headers
  client.println("HTTP/1.1 200 OK");
  client.println("Content-Type: text/event-stream");
  client.println("Cache-Control: no-cache");
  client.println("Access-Control-Allow-Origin: *");
  client.println("Connection: keep-alive");
  client.println();

  // Flush stale frames
  flushCameraBuffer();

  unsigned long lastFrame = 0;
  unsigned long lastMqttLoop = 0;
  unsigned long streamStart = millis();
  unsigned long framesSent = 0;
  const unsigned long FRAME_INTERVAL = 333;  // ~3 FPS
  const unsigned long STREAM_TIMEOUT = 300000;  // 5 min max

  // Static buffer for base64 encoding (in PSRAM)
  static char* base64Buffer = nullptr;
  static const size_t BASE64_BUFFER_SIZE = 150000;  // ~100KB JPEG -> ~133KB base64
  
  if (!base64Buffer) {
    base64Buffer = (char*)ps_malloc(BASE64_BUFFER_SIZE);
    if (!base64Buffer) {
      client.println("event: error");
      client.println("data: {\"error\":\"Memory allocation failed\"}");
      client.println();
      client.stop();
      return;
    }
  }

  while (client.connected()) {
    unsigned long now = millis();
    
    // Timeout protection
    if (now - streamStart > STREAM_TIMEOUT) {
      client.println("event: timeout");
      client.println("data: {\"reason\":\"Stream timeout (5 min max)\"}");
      client.println();
      break;
    }
    
    // Frame rate limiting - 3 FPS
    if (now - lastFrame < FRAME_INTERVAL) {
      yield();
      // Keep MQTT alive during wait
      if (now - lastMqttLoop > 100) {
        if (mqtt.connected()) mqtt.loop();
        lastMqttLoop = now;
      }
      continue;
    }
    
    camera_fb_t* fb = esp_camera_fb_get();
    if (!fb) {
      client.println("event: error");
      client.println("data: {\"error\":\"Capture failed\"}");
      client.println();
      break;
    }

    // Check if image fits in buffer
    size_t base64Len = ((fb->len + 2) / 3) * 4 + 1;
    if (base64Len > BASE64_BUFFER_SIZE) {
      esp_camera_fb_return(fb);
      client.println("event: error");
      client.println("data: {\"error\":\"Image too large\"}");
      client.println();
      break;
    }

    // Encode to base64
    size_t outLen = 0;
    mbedtls_base64_encode((unsigned char*)base64Buffer, BASE64_BUFFER_SIZE, 
                          &outLen, fb->buf, fb->len);
    base64Buffer[outLen] = '\0';
    
    esp_camera_fb_return(fb);

    // Send SSE event with data URL
    client.println("event: frame");
    client.print("data: data:image/jpeg;base64,");
    client.println(base64Buffer);
    client.println();
    
    lastFrame = now;
    framesSent++;

    if (framesSent % 5 == 0) { // Log every 5 frames (approx 1.5s)
        Serial.printf("📊 SSE health: frames=%lu, heap=%u\n", framesSent, ESP.getFreeHeap());
    }

    yield();
  }
  
  client.stop();
  Serial.println("⏹️ SSE Stream ended");
}

void handleCapture() {
  camera_fb_t * fb = esp_camera_fb_get();
  if (!fb) {
    streamServer.send(500, "text/plain", "Capture failed");
    return;
  }

  Serial.printf("📸 Snapshot: %u bytes\n", fb->len);
  streamServer.sendHeader("Content-Type", "image/jpeg");
  streamServer.sendHeader("Access-Control-Allow-Origin", "*");
  streamServer.send_P(200, "image/jpeg", (const char*)fb->buf, fb->len);
  esp_camera_fb_return(fb);
}

void handleControl() {
  String cmd = streamServer.arg("cmd");
  String response = "{\"success\":true}";

  if (cmd == "flash_on") { setFlash(true); flashEnabled = true; }
  else if (cmd == "flash_off") { setFlash(false); flashEnabled = false; }
  else if (cmd == "stream_on") { streamingEnabled = true; }
  else if (cmd == "stream_off") { streamingEnabled = false; }
  else if (cmd == "resolution") {
    String size = streamServer.arg("size");
    sensor_t * s = esp_camera_sensor_get();
    if (s) {
      if (size == "QVGA") s->set_framesize(s, FRAMESIZE_QVGA);
      else if (size == "VGA") s->set_framesize(s, FRAMESIZE_VGA);
      else if (size == "SVGA") s->set_framesize(s, FRAMESIZE_SVGA);
      else if (size == "XGA") s->set_framesize(s, FRAMESIZE_XGA);
    }
  } else {
    response = "{\"success\":false,\"error\":\"Unknown command\"}";
  }

  streamServer.sendHeader("Access-Control-Allow-Origin", "*");
  streamServer.send(200, "application/json", response);
}

void handleInfo() {
  JsonDocument doc;
  doc["device"] = deviceId;
  doc["firmware"] = FIRMWARE_VERSION;
  doc["chip"] = "ESP32-S3";
  doc["psram"] = ESP.getFreePsram();
  doc["ip"] = WiFi.localIP().toString();
  doc["streaming"] = streamingEnabled;
  doc["resolution"] = getResolutionName();
  
  String response;
  serializeJson(doc, response);
  streamServer.sendHeader("Access-Control-Allow-Origin", "*");
  streamServer.send(200, "application/json", response);
}

void setupStreamServer() {
  streamServer.on("/stream", HTTP_GET, handleStream);       // Legacy MJPEG (blocking)
  streamServer.on("/photostream", HTTP_GET, handlePhotoStream);  // NEW: SSE 3 FPS
  streamServer.on("/capture", HTTP_GET, handleCapture);     // Single snapshot
  streamServer.on("/control", HTTP_GET, handleControl);
  streamServer.on("/info", HTTP_GET, handleInfo);
  streamServer.begin();
  Serial.println("🎥 Stream server on :81");
  Serial.println("   /photostream = SSE 3 FPS (recommended)");
  Serial.println("   /stream = MJPEG (legacy)");
}

//-----------4-----------

//-----------5-----------
// ===== MQTT CONTROL =====

void mqttCallback(char* topic, byte* payload, unsigned int length) {
  // Use static buffer - no malloc
  if (length >= sizeof(mqttDecryptBuffer)) return;
  
  memcpy(mqttDecryptBuffer, payload, length);
  xorPayload(mqttDecryptBuffer, length);
  mqttDecryptBuffer[length] = '\0';

  char* message = (char*)mqttDecryptBuffer;  // Direct pointer, no String

  JsonDocument doc;
  if (!deserializeJson(doc, message)) {
    const char* cmd = doc["cmd"] | "";
    
    if (strcmp(cmd, "snapshot") == 0) { publishEncrypted(mqttPrefix + "/snapshot/notify", "{\"ready\":true}"); }
    else if (strcmp(cmd, "stream_on") == 0) { streamingEnabled = true; publishStatus(); }
    else if (strcmp(cmd, "stream_off") == 0) { streamingEnabled = false; publishStatus(); }
    else if (strcmp(cmd, "flash_on") == 0) { setFlash(true); flashEnabled = true; }
    else if (strcmp(cmd, "flash_off") == 0) { setFlash(false); flashEnabled = false; }
    else if (strcmp(cmd, "restart") == 0) { ESP.restart(); }
    else if (strcmp(cmd, "status") == 0) { publishStatus(); }
    else if (strcmp(cmd, "resolution") == 0) {
      const char* size = doc["size"] | "VGA";
      sensor_t * s = esp_camera_sensor_get();
      if (s) {
        if (strcmp(size, "QVGA") == 0) { s->set_framesize(s, FRAMESIZE_QVGA); frameSize = FRAMESIZE_QVGA; }
        else if (strcmp(size, "VGA") == 0) { s->set_framesize(s, FRAMESIZE_VGA); frameSize = FRAMESIZE_VGA; }
        else if (strcmp(size, "SVGA") == 0) { s->set_framesize(s, FRAMESIZE_SVGA); frameSize = FRAMESIZE_SVGA; }
        publishStatus();
      }
    }
  }
}

void publishStatus() {
  JsonDocument doc;
  doc["online"] = true;
  doc["ip"] = WiFi.localIP().toString();
  doc["streaming"] = streamingEnabled;
  doc["resolution"] = getResolutionName();
  doc["flash"] = flashEnabled;
  doc["rssi"] = WiFi.RSSI();
  doc["version"] = FIRMWARE_VERSION;
  doc["uptime"] = millis() / 1000;
  doc["chip"] = "ESP32-S3";
  doc["heap"] = ESP.getFreeHeap() / 1024;    // KB free - MEMORY MONITOR
  doc["psram"] = ESP.getFreePsram() / 1024;  // KB free - MEMORY MONITOR

  char buffer[512];
  serializeJson(doc, buffer);
  publishEncrypted(mqttPrefix + "/status", buffer);
}

void setupMQTT() {
  mqtt.setServer(mqtt_server.c_str(), mqtt_port);
  mqtt.setCallback(mqttCallback);
  mqtt.setBufferSize(512);
}

bool connectMQTT() {
  if (mqtt.connected()) return true;

  String clientId = deviceId + "-" + String(millis() & 0xFFFF);
  Serial.print("🔌 MQTT... ");
  
  bool connected = mqtt_user.length() > 0 
    ? mqtt.connect(clientId.c_str(), mqtt_user.c_str(), mqtt_pass.c_str())
    : mqtt.connect(clientId.c_str());

  if (connected) {
    Serial.println("OK");
    String controlTopic = mqttPrefix + "/control";
    mqtt.subscribe(controlTopic.c_str());
    publishStatus();
    return true;
  }
  Serial.printf("FAIL (rc=%d)\n", mqtt.state());
  return false;
}

//-----------5-----------

//-----------6-----------
// ===== NVS CONFIGURATION =====

void loadSettings() {
  Serial.println("\n📂 Loading NVS settings...");
  
  preferences.begin("cam-config", false);
  wifi_ssid = preferences.getString("wifi_ssid", "");
  wifi_pass = preferences.getString("wifi_pass", "");
  mqtt_server = preferences.getString("mqtt_server", "");
  mqtt_port = preferences.getInt("mqtt_port", 1883);
  mqtt_user = preferences.getString("mqtt_user", "");
  mqtt_pass = preferences.getString("mqtt_pass", "");
  backend_url = preferences.getString("backend_url", "");
  frameSize = preferences.getInt("frame_size", FRAMESIZE_VGA);
  jpegQuality = preferences.getInt("jpeg_quality", 12);
  preferences.end();
  
  Serial.printf("   WiFi: %s, MQTT: %s:%d\n", wifi_ssid.c_str(), mqtt_server.c_str(), mqtt_port);
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
  preferences.putInt("frame_size", frameSize);
  preferences.putInt("jpeg_quality", jpegQuality);
  preferences.end();
}

//-----------6-----------

//-----------7-----------
// ===== AP MODE / WEB CONFIGURATION =====

const char* HTML_HEAD = R"(
<!DOCTYPE html>
<html>
<head>
<meta charset='UTF-8'>
<meta name='viewport' content='width=device-width,initial-scale=1'>
<title>ESP32-S3 CAM Setup</title>
<style>
body{font-family:Arial,sans-serif;margin:0;padding:20px;background:#1a1a2e}
.container{max-width:500px;margin:auto;background:#16213e;padding:20px;border-radius:8px;color:#fff}
h2{color:#e94560;border-bottom:2px solid #e94560;padding-bottom:10px}
h3{color:#4a90d9;margin-top:20px}
label{display:block;margin-top:10px;color:#a0a0a0}
input,select{width:100%;padding:10px;margin-top:5px;border:1px solid #0f3460;border-radius:4px;box-sizing:border-box;background:#1a1a2e;color:#fff}
button{background:#e94560;color:#fff;border:none;padding:12px 20px;margin-top:20px;border-radius:4px;cursor:pointer;width:100%;font-size:16px}
button:hover{background:#ff6b6b}
.info{background:#0f3460;padding:15px;border-radius:4px;margin-bottom:15px}
.device-id{font-size:24px;color:#4ade80;font-family:monospace;text-align:center;padding:15px;background:#0f3460;border-radius:8px;margin-bottom:15px}
.preview{width:100%;max-height:250px;object-fit:contain;border-radius:4px;margin-top:10px}
</style>
</head>
<body>
<div class='container'>
)";

void handleRoot() {
  String html = HTML_HEAD;
  html += "<h2>📷 ESP32-S3 CAM Setup</h2>";

  // DEVICE ID - velké a zřetelné
  html += "<div class='device-id'>";
  html += "🆔 " + deviceId;
  html += "</div>";

  html += "<div class='info'>";
  html += "<strong>Firmware:</strong> " + String(FIRMWARE_VERSION) + "<br>";
  html += "<strong>Chip:</strong> ESP32-S3 N16R8<br>";
  html += "<strong>PSRAM:</strong> " + String(ESP.getPsramSize() / 1024 / 1024) + " MB<br>";
  html += "<strong>Free Heap:</strong> " + String(ESP.getFreeHeap() / 1024) + " KB";
  html += "</div>";

  html += "<h3>📷 Live Preview</h3>";
  html += "<img class='preview' src='/capture' id='preview'>";
  html += "<button type='button' onclick='document.getElementById(\"preview\").src=\"/capture?\"+Date.now()'>🔄 Refresh</button>";

  html += "<form method='POST' action='/save'>";

  html += "<h3>📶 WiFi</h3>";
  html += "<label>SSID</label><input name='ssid' value='" + wifi_ssid + "'>";
  html += "<label>Password</label><input type='password' name='pass' value='" + wifi_pass + "'>";

  html += "<h3>📡 MQTT Broker</h3>";
  html += "<label>Server</label><input name='mqtt_server' value='" + mqtt_server + "'>";
  html += "<label>Port</label><input type='number' name='mqtt_port' value='" + String(mqtt_port) + "'>";
  html += "<label>Username</label><input name='mqtt_user' value='" + mqtt_user + "'>";
  html += "<label>Password</label><input type='password' name='mqtt_pass' value='" + mqtt_pass + "'>";

  html += "<h3>🖥️ Backend</h3>";
  html += "<label>Backend URL</label><input name='backend_url' value='" + backend_url + "' placeholder='http://192.168.x.x:3004'>";

  html += "<h3>📐 Camera</h3>";
  html += "<label>Resolution</label><select name='resolution'>";
  html += "<option value='3'" + String(frameSize == FRAMESIZE_QVGA ? " selected" : "") + ">QVGA (320x240)</option>";
  html += "<option value='6'" + String(frameSize == FRAMESIZE_VGA ? " selected" : "") + ">VGA (640x480)</option>";
  html += "<option value='7'" + String(frameSize == FRAMESIZE_SVGA ? " selected" : "") + ">SVGA (800x600)</option>";
  html += "<option value='8'" + String(frameSize == FRAMESIZE_XGA ? " selected" : "") + ">XGA (1024x768)</option>";
  html += "</select>";

  html += "<label>JPEG Quality (0-63, lower=better)</label>";
  html += "<input type='number' name='quality' value='" + String(jpegQuality) + "' min='0' max='63'>";

  html += "<button type='submit'>💾 Save & Restart</button>";
  html += "</form></div></body></html>";

  server.send(200, "text/html", html);
}

void handleSave() {
  wifi_ssid = server.arg("ssid");
  wifi_pass = server.arg("pass");
  mqtt_server = server.arg("mqtt_server");
  mqtt_port = server.arg("mqtt_port").toInt();
  mqtt_user = server.arg("mqtt_user");
  mqtt_pass = server.arg("mqtt_pass");
  backend_url = server.arg("backend_url");
  frameSize = server.arg("resolution").toInt();
  jpegQuality = server.arg("quality").toInt();

  saveSettings();

  String html = HTML_HEAD;
  html += "<h2>✅ Saved!</h2><p>Device is restarting...</p>";
  html += "<p>Device ID: <strong>" + deviceId + "</strong></p>";
  html += "</div></body></html>";
  server.send(200, "text/html", html);

  delay(1000);
  ESP.restart();
}

void handleAPCapture() {
  camera_fb_t * fb = esp_camera_fb_get();
  if (!fb) {
    server.send(500, "text/plain", "Capture failed");
    return;
  }
  server.sendHeader("Content-Type", "image/jpeg");
  server.send_P(200, "image/jpeg", (const char*)fb->buf, fb->len);
  esp_camera_fb_return(fb);
}

void startAPMode() {
  apMode = true;
  Serial.println("\n🔧 Starting AP Mode...");
  
  WiFi.mode(WIFI_AP);
  String apName = "ESP32S3-CAM-" + deviceId.substring(3);
  WiFi.softAP(apName.c_str(), "12345678");

  IPAddress IP = WiFi.softAPIP();
  Serial.printf("📡 AP: %s (pass: 12345678)\n", apName.c_str());
  Serial.printf("   IP: %s\n", IP.toString().c_str());
  Serial.printf("   Device ID: %s\n", deviceId.c_str());

  dnsServer.start(53, "*", IP);

  server.on("/", handleRoot);
  server.on("/save", HTTP_POST, handleSave);
  server.on("/capture", handleAPCapture);
  server.on("/generate_204", handleRoot);
  server.on("/fwlink", handleRoot);
  server.onNotFound(handleRoot);

  server.begin();
  Serial.println("🌐 Web config portal started");

  // AP mode loop
  while (true) {
    dnsServer.processNextRequest();
    server.handleClient();
    
    // Blink LED in AP mode
    static unsigned long lastBlink = 0;
    if (millis() - lastBlink > 500) {
      static bool state = false;
      setLed(state);
      state = !state;
      lastBlink = millis();
    }
  }
}

//-----------7-----------

//-----------8-----------
// ===== BOOT BUTTON HANDLER =====
// Podle SolaxCloudBridge:
// 5-15s = AP Mode
// >15s = Factory Reset
//
// DEBUG: Verbose logging pro diagnostiku

void checkBootButton() {
  static unsigned long btnPressStart = 0;
  static bool btnPressed = false;

  int btnState = digitalRead(BOOT_BUTTON_PIN);

  // NO DEBUG SPAM - causes performance issues

  if (btnState == LOW) {  // Pressed (Active LOW)
    if (!btnPressed) {
      btnPressed = true;
      btnPressStart = millis();
      Serial.println("═══════════════════════════════════════");
      Serial.println("🔘 BOOT BUTTON PRESSED!");
      Serial.println("   Hold 5-15s for AP Mode");
      Serial.println("   Hold >15s for Factory Reset");
      Serial.println("═══════════════════════════════════════");
    }

    unsigned long duration = millis() - btnPressStart;

    // Print duration while holding
    if (duration > 1000 && (duration / 1000) != ((duration - 100) / 1000)) {
      Serial.printf("⏱️ Button held: %lu seconds\n", duration / 1000);
    }

    // 15s = Factory Reset (execute immediately while still pressed)
    if (duration > 15000) {
      Serial.println("═══════════════════════════════════════");
      Serial.println("⚠️ 15+ SECONDS → FACTORY RESET!");
      Serial.println("═══════════════════════════════════════");
      wipeNVS();  // Will restart
      return;
    }
    
    // 5s = Visual feedback for AP mode pending
    if (duration > 5000) {
      setLed((millis() / 100) % 2 == 0);  // Fast blink
      
      // Print once when crossing 5s threshold
      static bool printedAPReady = false;
      if (!printedAPReady) {
        Serial.println("✨ 5s reached - Release now for AP Mode");
        printedAPReady = true;
      }
    }

  } else { // Released (HIGH)
    if (btnPressed) {
      unsigned long duration = millis() - btnPressStart;
      setLed(false);
      
      Serial.printf("🔘 Button RELEASED after %lu ms\n", duration);
      
      if (duration > 5000 && duration < 15000) {
        Serial.println("═══════════════════════════════════════");
        Serial.println("🔧 5-15s DETECTED → STARTING AP MODE!");
        Serial.println("═══════════════════════════════════════");
        startAPMode();  // Never returns (infinite loop)
      } else if (duration < 5000) {
        Serial.println("🔘 Short press (<5s), ignoring");
      }
      
      btnPressed = false;
    }
  }
}


//-----------8-----------

//-----------9-----------
// ===== BACKEND REGISTRATION =====

void registerWithBackend() {
  if (backend_url.length() == 0) return;

  HTTPClient http;
  String url = backend_url + "/api/camera/register";
  
  http.begin(url);
  http.setTimeout(2000);  // 2s max - non-blocking-ish
  http.addHeader("Content-Type", "application/json");
  
  JsonDocument doc;
  doc["deviceId"] = deviceId;
  doc["ip"] = WiFi.localIP().toString();
  doc["port"] = 81;
  doc["resolution"] = getResolutionName();
  doc["streaming"] = streamingEnabled;
  doc["chip"] = "ESP32-S3";
  
  String body;
  serializeJson(doc, body);
  
  http.POST(body);  // Fire and forget
  http.end();
}

// ===== WIFI CONNECTION =====

bool connectWiFi() {
  if (wifi_ssid.length() == 0) return false;

  Serial.printf("📶 Connecting to: %s\n", wifi_ssid.c_str());
  
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false); // DISABLE POWER SAVE - Critical for smooth streaming!
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
    WiFi.setSleep(false); // Ensure it stays off
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
  Serial.println("║   📷 ESP32-S3 CAM ZONIO v1.1          ║");
  Serial.println("╚════════════════════════════════════════╝");
  Serial.printf("   Firmware: %s\n", FIRMWARE_VERSION);
  Serial.printf("   PSRAM: %d MB\n", ESP.getPsramSize() / 1024 / 1024);
  
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
  
  // Get device ID
  deviceId = getChipId();
  mqttPrefix = "zonio/cam/" + deviceId;
  Serial.printf("   Device ID: %s\n", deviceId.c_str());
  
  // Init camera
  if (!initCamera()) {
    Serial.println("❌ Camera failed!");
  }
  
  // Load settings
  loadSettings();
  
  // Init static PSRAM buffer for XOR encryption
  initXorBuffer();
  
  // Check for immediate AP mode on boot (button held during startup)
  if (digitalRead(BOOT_BUTTON_PIN) == LOW) {
    Serial.println("🔧 BOOT button held on startup → AP Mode");
    delay(1000); // Debounce
    if (digitalRead(BOOT_BUTTON_PIN) == LOW) {
      startAPMode();
      return;
    }
  }
  
  // Check if WiFi configured
  if (wifi_ssid.length() == 0 || mqtt_server.length() == 0) {
    Serial.println("⚠️ Not configured → AP Mode");
    startAPMode();
    return;
  }
  
  // Connect WiFi
  if (!connectWiFi()) {
    Serial.println("⚠️ WiFi failed → AP Mode");
    startAPMode();
    return;
  }
  
  // Setup services
  setupMQTT();
  setupStreamServer();
  registerWithBackend();
  
  startNonBlockingBlink(3);
  
  Serial.println("\n✅ ESP32-S3 CAM READY");
  Serial.printf("   Stream: http://%s:81/stream\n", WiFi.localIP().toString().c_str());
  Serial.printf("   Snapshot: http://%s:81/capture\n", WiFi.localIP().toString().c_str());
  Serial.println("\n💡 Tip: Hold BOOT 5-15s for AP mode, >15s for factory reset");
}

void loop() {
  checkBootButton();
  streamServer.handleClient();
  processNonBlockingBlink();
  yield();  // Feed WDT
  
  // MQTT handling
  if (!mqtt.connected()) {
    static unsigned long lastReconnect = 0;
    if (millis() - lastReconnect > 5000) {
      connectMQTT();
      lastReconnect = millis();
    }
  } else {
    mqtt.loop();
  }
  
  // Heartbeat - burst shot
  if (millis() - lastHeartbeat > 30000) {
    publishStatus();
    lastHeartbeat = millis();
  }
  
  // Backend registration - 5 min interval (was 1 min)
  if (millis() - lastRegistration > 300000) {
    registerWithBackend();
    lastRegistration = millis();
  }
  // NO DELAY - yield() handles WDT
}

//-----------10-----------
