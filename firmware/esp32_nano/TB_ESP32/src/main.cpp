#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <AsyncTCP.h>
#include <ArduinoJson.h>
#include <Wire.h>

#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// -------------------- USER CONFIG --------------------
static const char* WIFI_SSID = "Shaon";
static const char* WIFI_PASS = "shA12345";
static const uint16_t WS_PORT = 9000;

// Robot kinematics (tune these)
static const float WHEEL_BASE_M = 0.10f;  // distance between wheels (m) - adjust
// -----------------------------------------------------

// -------------------- PINOUT (from schematic) --------------------
// DRV8835 inputs (IN/IN style, two pins per motor)
static const int PIN_MTA1 = A6;   // MTA1 -> A6
static const int PIN_MTA2 = A7;   // MTA2 -> A7
static const int PIN_MTB1 = D8;   // MTB1 -> D8
static const int PIN_MTB2 = D9;   // MTB2 -> D9

// OLED I2C pins (from schematic)
static const int PIN_SDA  = A4;   // SDA -> A4
static const int PIN_SCL  = A5;   // SCL -> A5
// -----------------------------------------------------------------

// -------------------- OLED --------------------
static const int OLED_W = 128;
static const int OLED_H = 64;
static const int OLED_RST = -1;
Adafruit_SSD1306 display(OLED_W, OLED_H, &Wire, OLED_RST);

// -------------------- Web Server / WS --------------------
AsyncWebServer server(WS_PORT);
AsyncWebSocket ws("/ws"); // WS endpoint

// -------------------- Motor PWM (ESP32 LEDC) --------------------
static const int PWM_FREQ = 20000;     // 20kHz
static const int PWM_RES  = 10;        // 10-bit => 0..1023
static const int PWM_MAX  = (1 << PWM_RES) - 1;

static const int CH_A1 = 0;
static const int CH_A2 = 1;
static const int CH_B1 = 2;
static const int CH_B2 = 3;

volatile uint32_t last_cmd_ms = 0;
static const uint32_t CMD_TIMEOUT_MS = 500;

// Track last PWM actually applied (for motor_state stream)
static volatile int g_left_pwm = 0;
static volatile int g_right_pwm = 0;

// -------------------- Stream gating (cfg) --------------------
struct StreamCfg {
  bool en = false;
  float hz = 0.0f;
  uint32_t period_ms = 0;
  uint32_t last_ms = 0;
};

static StreamCfg g_stream_motors;
static StreamCfg g_stream_imu;
static StreamCfg g_stream_ir;
static StreamCfg g_stream_lidar;
static StreamCfg g_stream_lidar360;

// Defaults if cfg sets hz <= 0
static const float DEF_MOTORS_HZ   = 20.0f;
static const float DEF_IMU_HZ      = 50.0f;
static const float DEF_IR_HZ       = 10.0f;
static const float DEF_LIDAR_HZ    = 5.0f;
static const float DEF_LIDAR360_HZ = 5.0f;

// -------------------- MPU6050 (raw I2C, no extra library) --------------------
static const uint8_t MPU_ADDR = 0x68;

// MPU registers
static const uint8_t REG_PWR_MGMT_1 = 0x6B;
static const uint8_t REG_ACCEL_XOUT_H = 0x3B;

// Scale factors for default config:
// Accel ±2g => 16384 LSB/g
// Gyro  ±250 dps => 131 LSB/(deg/s)
static const float ACC_LSB_PER_G = 16384.0f;
static const float GYRO_LSB_PER_DPS = 131.0f;
static const float G_TO_MSS = 9.80665f;
static const float DEG_TO_RAD_F = 0.017453292519943295f;

// -------------------- helpers ----------
static int dutyFromNorm(float x) {
  x = fabsf(x);
  if (x > 1.0f) x = 1.0f;
  return (int)(x * PWM_MAX);
}

// IN/IN style: one pin PWM for forward, the other PWM for reverse
static void setMotorININ(int ch_fwd, int ch_rev, float speed_norm) {
  int duty = dutyFromNorm(speed_norm);

  if (speed_norm >= 0.0f) {
    ledcWrite(ch_fwd, duty);
    ledcWrite(ch_rev, 0);
  } else {
    ledcWrite(ch_fwd, 0);
    ledcWrite(ch_rev, duty);
  }
}

static void stopAll() {
  ledcWrite(CH_A1, 0);
  ledcWrite(CH_A2, 0);
  ledcWrite(CH_B1, 0);
  ledcWrite(CH_B2, 0);

  g_left_pwm = 0;
  g_right_pwm = 0;
}

static void drawStatus(const char* line2 = nullptr) {
  display.clearDisplay();
  display.setTextSize(1);
  display.setRotation(2);
  display.setTextColor(SSD1306_WHITE);

  display.setCursor(0, 0);
  display.print("Mini Turtlebot");

  display.setCursor(0, 12);
  display.print("SSID: ");
  display.print(WIFI_SSID);

  display.setCursor(0, 24);
  display.print("IP: ");
  display.print(WiFi.localIP());

  display.setCursor(0, 36);
  display.print("WS: ");
  display.print(WS_PORT);
  display.print("  /ws");

  if (line2) {
    display.setCursor(0, 52);
    display.print(line2);
  }

  display.display();
}

static void setStream(StreamCfg& s, bool en, float hz, float def_hz) {
  s.en = en;
  s.hz = (hz > 0.0f) ? hz : def_hz;
  if (!s.en) {
    s.period_ms = 0;
    return;
  }
  float used = (s.hz > 0.0f) ? s.hz : def_hz;
  s.period_ms = (uint32_t)max(1.0f, 1000.0f / used);
  s.last_ms = millis();
}

// -------------------- Robot control --------------------
static void handleCmdVel(float linear_x, float angular_z) {
  // differential drive:
  float v_left  = linear_x - angular_z * (WHEEL_BASE_M * 0.5f);
  float v_right = linear_x + angular_z * (WHEEL_BASE_M * 0.5f);

  // Map to normalized [-1..1]
  const float VMAX = 0.25f; // tune
  float left_norm  = v_left  / VMAX;
  float right_norm = v_right / VMAX;

  if (left_norm >  1.0f) left_norm =  1.0f;
  if (left_norm < -1.0f) left_norm = -1.0f;
  if (right_norm >  1.0f) right_norm =  1.0f;
  if (right_norm < -1.0f) right_norm = -1.0f;

  // Motor A = left, Motor B = right (swap if needed)
  setMotorININ(CH_A1, CH_A2, left_norm);
  setMotorININ(CH_B1, CH_B2, -right_norm);

  g_left_pwm  = (int)(left_norm  * PWM_MAX);
  g_right_pwm = (int)(right_norm * PWM_MAX);

  last_cmd_ms = millis();
}

// -------------------- WS JSONL helpers --------------------
static void wsTextAllJsonl(const JsonDocument& doc) {
  String out;
  serializeJson(doc, out);
  out += "\n";
  ws.textAll(out);
}

static void wsTextClientJsonl(AsyncWebSocketClient* client, const JsonDocument& doc) {
  if (!client) return;
  String out;
  serializeJson(doc, out);
  out += "\n";
  client->text(out);
}

// -------------------- MPU6050 low-level --------------------
static bool mpuWriteReg(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.write(value);
  return (Wire.endTransmission() == 0);
}

static bool mpuReadBytes(uint8_t startReg, uint8_t* buf, size_t n) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(startReg);
  if (Wire.endTransmission(false) != 0) return false;

  size_t got = Wire.requestFrom((int)MPU_ADDR, (int)n, (int)true);
  if (got != n) return false;

  for (size_t i = 0; i < n; i++) buf[i] = Wire.read();
  return true;
}

static bool mpuInit() {
  // Wake up MPU6050 (clear sleep bit)
  // PWR_MGMT_1 = 0x00
  if (!mpuWriteReg(REG_PWR_MGMT_1, 0x00)) {
    return false;
  }
  delay(50);
  return true;
}

static bool mpuReadAccelGyro(float& ax_mss, float& ay_mss, float& az_mss,
                             float& gx_rads, float& gy_rads, float& gz_rads) {
  uint8_t b[14];
  if (!mpuReadBytes(REG_ACCEL_XOUT_H, b, sizeof(b))) return false;

  int16_t ax = (int16_t)((b[0] << 8) | b[1]);
  int16_t ay = (int16_t)((b[2] << 8) | b[3]);
  int16_t az = (int16_t)((b[4] << 8) | b[5]);
  // temp: b[6..7] ignored
  int16_t gx = (int16_t)((b[8] << 8) | b[9]);
  int16_t gy = (int16_t)((b[10] << 8) | b[11]);
  int16_t gz = (int16_t)((b[12] << 8) | b[13]);

  // Convert accel to m/s^2
  ax_mss = ((float)ax / ACC_LSB_PER_G) * G_TO_MSS;
  ay_mss = ((float)ay / ACC_LSB_PER_G) * G_TO_MSS;
  az_mss = ((float)az / ACC_LSB_PER_G) * G_TO_MSS;

  // Convert gyro to rad/s
  float gx_dps = ((float)gx / GYRO_LSB_PER_DPS);
  float gy_dps = ((float)gy / GYRO_LSB_PER_DPS);
  float gz_dps = ((float)gz / GYRO_LSB_PER_DPS);

  gx_rads = gx_dps * DEG_TO_RAD_F;
  gy_rads = gy_dps * DEG_TO_RAD_F;
  gz_rads = gz_dps * DEG_TO_RAD_F;

  return true;
}

// -------------------- Periodic publishers --------------------
static void publishHeartbeat() {
  static uint32_t last_ms = 0;
  static uint32_t i = 0;
  uint32_t now = millis();
  if (now - last_ms < 1000) return;
  last_ms = now;

  StaticJsonDocument<192> hb;
  hb["v"] = 1;
  hb["type"] = "hb";
  hb["ts_ms"] = now;
  hb["i"] = i++;
  wsTextAllJsonl(hb);
}

static void publishMotorsIfDue() {
  if (!g_stream_motors.en || g_stream_motors.period_ms == 0) return;
  uint32_t now = millis();
  if (now - g_stream_motors.last_ms < g_stream_motors.period_ms) return;
  g_stream_motors.last_ms = now;

  StaticJsonDocument<256> m;
  m["v"] = 1;
  m["type"] = "state";
  m["state"] = "motors";
  m["ts_ms"] = now;

  // placeholders until you have encoders
  m["left_rads"] = 0.0f;
  m["right_rads"] = 0.0f;
  m["left_pwm"] = (int)g_left_pwm;
  m["right_pwm"] = (int)g_right_pwm;

  wsTextAllJsonl(m);
}

static void publishImuIfDue() {
  if (!g_stream_imu.en || g_stream_imu.period_ms == 0) return;
  uint32_t now = millis();
  if (now - g_stream_imu.last_ms < g_stream_imu.period_ms) return;
  g_stream_imu.last_ms = now;

  float ax, ay, az, gx, gy, gz;
  if (!mpuReadAccelGyro(ax, ay, az, gx, gy, gz)) {
    // Don't spam; just skip if read fails
    return;
  }

  StaticJsonDocument<256> j;
  j["v"] = 1;
  j["type"] = "state";
  j["state"] = "imu";
  j["ts_ms"] = now;

  // accel (m/s^2)
  j["ax"] = ax;
  j["ay"] = ay;
  j["az"] = az;

  // gyro (rad/s)
  j["gx"] = gx;
  j["gy"] = gy;
  j["gz"] = gz;

  wsTextAllJsonl(j);
}

// -------------------- cfg handler --------------------
static void handleCfg(AsyncWebSocketClient* client, const JsonDocument& doc) {
  int id = doc["id"] | 0;

  JsonObjectConst sObj = doc["streams"].as<JsonObjectConst>();
  if (sObj.isNull()) {
    StaticJsonDocument<192> ack;
    ack["v"] = 1;
    ack["type"] = "ack";
    ack["id"] = id;
    ack["ok"] = false;
    ack["msg"] = "missing streams object";
    wsTextClientJsonl(client, ack);
    return;
  }

  auto upd = [&](const char* name, StreamCfg& cfg, float defhz) {
    if (!sObj[name].is<JsonObjectConst>()) return;
    JsonObjectConst o = sObj[name].as<JsonObjectConst>();
    bool en = o["en"] | false;
    float hz = o["hz"] | 0.0f;
    setStream(cfg, en, hz, defhz);
  };

  upd("motors",   g_stream_motors,   DEF_MOTORS_HZ);
  upd("imu",      g_stream_imu,      DEF_IMU_HZ);
  upd("ir",       g_stream_ir,       DEF_IR_HZ);
  upd("lidar",    g_stream_lidar,    DEF_LIDAR_HZ);
  upd("lidar360", g_stream_lidar360, DEF_LIDAR360_HZ);

  StaticJsonDocument<192> ack;
  ack["v"] = 1;
  ack["type"] = "ack";
  ack["id"] = id;
  ack["ok"] = true;
  ack["msg"] = "streams updated";
  wsTextClientJsonl(client, ack);
}

// -------------------- Message dispatcher (one JSON line) --------------------
static void handleOneJsonLine(AsyncWebSocketClient* client, const String& line) {
  if (line.length() == 0) return;

  // cfg has nested objects, give enough space
  StaticJsonDocument<768> doc;
  DeserializationError err = deserializeJson(doc, line);
  if (err) {
    Serial.printf("[ws] JSON parse error: %s\n", err.c_str());
    return;
  }

  const char* type_s = doc["type"] | "";
  const char* cmd_s  = doc["cmd"]  | "";

  if (strcmp(type_s, "cmd") == 0) {
    if (strcmp(cmd_s, "vel") == 0) {
      float linear  = doc["linear"]  | 0.0f;
      float angular = doc["angular"] | 0.0f;
      handleCmdVel(linear, angular);
    } else if (strcmp(cmd_s, "stop") == 0) {
      stopAll();
    } else if (strcmp(cmd_s, "estop") == 0) {
      bool enabled = doc["enabled"] | true;
      if (enabled) stopAll();
    } else if (strcmp(cmd_s, "ping") == 0) {
      StaticJsonDocument<128> ack;
      ack["v"] = 1;
      ack["type"] = "ack";
      ack["ok"] = true;
      ack["msg"] = "pong";
      wsTextClientJsonl(client, ack);
    } else {
      Serial.printf("[ws] unknown cmd: %s\n", cmd_s);
    }
  }
  else if (strcmp(type_s, "cfg") == 0) {
    handleCfg(client, doc);
  }
  // Optional backward compatibility
  else if (strcmp(type_s, "cmd_vel") == 0) {
    float linear  = doc["linear_x"]  | 0.0f;
    float angular = doc["angular_z"] | 0.0f;
    handleCmdVel(linear, angular);
  }
  else {
    Serial.printf("[ws] unknown msg type: %s\n", type_s);
  }
}

// -------------------- WS events --------------------
void onWsEvent(AsyncWebSocket* server_,
               AsyncWebSocketClient* client,
               AwsEventType type,
               void* arg,
               uint8_t* data,
               size_t len) {
  (void)server_;
  (void)arg;

  if (type == WS_EVT_CONNECT) {
    Serial.printf("[ws] client #%u connected\n", client->id());
    drawStatus("WS client connected");
    return;
  }
  if (type == WS_EVT_DISCONNECT) {
    Serial.printf("[ws] client #%u disconnected\n", client->id());
    drawStatus("WS client disconnected");
    stopAll();
    return;
  }
  if (type != WS_EVT_DATA) return;

  AwsFrameInfo* info = (AwsFrameInfo*)arg;
  if (!info->final || info->index != 0 || info->len != len) return;
  if (info->opcode != WS_TEXT) return;

  // Copy into String
  String msg;
  msg.reserve(len + 1);
  for (size_t i = 0; i < len; i++) msg += (char)data[i];

  // JSONL: handle one or multiple lines
  int start = 0;
  while (start < (int)msg.length()) {
    int end = msg.indexOf('\n', start);
    if (end < 0) end = msg.length();
    String line = msg.substring(start, end);
    line.trim();
    start = end + 1;
    if (line.length() == 0) continue;
    handleOneJsonLine(client, line);
  }
}

void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println("\n[boot] Mini Turtlebot ESP32");

  // I2C OLED + IMU share the same bus
  Wire.begin(PIN_SDA, PIN_SCL); // SDA=A4, SCL=A5

  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println("[oled] SSD1306 init failed");
  } else {
    drawStatus("Booting...");
  }

  // Init MPU6050
  if (mpuInit()) {
    Serial.println("[imu] MPU6050 init OK");
  } else {
    Serial.println("[imu] MPU6050 init FAIL (check wiring/address)");
  }

  // PWM setup
  ledcSetup(CH_A1, PWM_FREQ, PWM_RES);
  ledcSetup(CH_A2, PWM_FREQ, PWM_RES);
  ledcSetup(CH_B1, PWM_FREQ, PWM_RES);
  ledcSetup(CH_B2, PWM_FREQ, PWM_RES);

  ledcAttachPin(PIN_MTA1, CH_A1);
  ledcAttachPin(PIN_MTA2, CH_A2);
  ledcAttachPin(PIN_MTB1, CH_B1);
  ledcAttachPin(PIN_MTB2, CH_B2);

  stopAll();

  // Default streams: motors ON at 1Hz so you can see it, imu OFF by default
  setStream(g_stream_motors, true,  1.0f, DEF_MOTORS_HZ);
  setStream(g_stream_imu,    false, 0.0f, DEF_IMU_HZ);
  setStream(g_stream_ir,     false, 0.0f, DEF_IR_HZ);
  setStream(g_stream_lidar,  false, 0.0f, DEF_LIDAR_HZ);
  setStream(g_stream_lidar360, false, 0.0f, DEF_LIDAR360_HZ);

  // WiFi
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);

  Serial.print("[wifi] connecting");
  while (WiFi.status() != WL_CONNECTED) {
    delay(250);
    Serial.print(".");
  }
  Serial.println();
  Serial.print("[wifi] connected, IP=");
  Serial.println(WiFi.localIP());

  drawStatus("WiFi connected");

  // WebSocket at /ws
  ws.onEvent(onWsEvent);
  server.addHandler(&ws);

  // Simple landing page at /
  server.on("/", HTTP_GET, [](AsyncWebServerRequest* request) {
    request->send(200, "text/plain", "Mini Turtlebot WS server. Use ws://<ip>:9000/ws");
  });

  server.begin();
  Serial.printf("[server] listening on http://0.0.0.0:%u/\n", WS_PORT);
  Serial.printf("[ws] ws://<ip>:%u/ws\n", WS_PORT);

  last_cmd_ms = millis();
}

void loop() {
  ws.cleanupClients();

  uint32_t now = millis();

  // Always-on heartbeat
  publishHeartbeat();

  // Gated streams
  publishMotorsIfDue();
  publishImuIfDue();

  // SAFETY WATCHDOG — STOP ONLY IF STALE
  static bool already_stopped = false;
  if ((now - last_cmd_ms) > CMD_TIMEOUT_MS) {
    if (!already_stopped) {
      Serial.printf("[SAFETY] cmd timeout (%lu ms) -> STOP\n",
                    (unsigned long)(now - last_cmd_ms));
      stopAll();
      already_stopped = true;
    }
  } else {
    already_stopped = false;
  }

  delay(5); // faster loop helps stream timing
}
