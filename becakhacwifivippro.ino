#include <WiFi.h>
#include <WebServer.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <ESP32Servo.h>
#include <Preferences.h>
#include <time.h>
#include <math.h>

// =====================================
// WIFI & CLOUD (DIEU KHIEN KHAC WIFI)
// =====================================
const char* ssid = "TP-Link_9C56";
const char* password = "35825159";


// Cau hinh Broker Cloud mien phi phuc vu dieu khien khac WiFi / 4G
const char* MQTT_HOST = "broker.emqx.io";
const int   MQTT_PORT = 1883;
String deviceId = "aqua-v3-uet"; // Ma dinh danh thiet bi (Co the doi tren Web)

WebServer server(80);
WiFiClient mqttNet;
Preferences prefs;

Adafruit_SSD1306 oled(128, 64, &Wire, -1);
OneWire oneWire(4);
DallasTemperature ds(&oneWire);
Servo feeder;

// RELAY
const int relayPins[4] = {25, 26, 27, 14};
const int RELAY_ON = LOW;
const int RELAY_OFF = HIGH;

bool relayState[4] = {false, false, false, false};

// SENSOR
float distanceCm = -1;
float temperatureC = -127;

float drainCm = 12;
float fillCm = 5;
float tankHeightCm = 20;

int maxMinutes = 5;
int feedCount = 0;

bool scheduleEnabled = true;
bool clockValid = false;
bool servoBusy = false;
bool autoPending = false;

String feed1 = "08:00";
String feed2 = "17:00";
String lastFeedKey = "";

String statusText = "San sang";
String faultText = "";

unsigned long lastDist = 0;
unsigned long lastTempStart = 0;
unsigned long lastOled = 0;
unsigned long lastSchedule = 0;
unsigned long lastWifi = 0;

// Bien phuc vu ket noi Cloud khac WiFi
unsigned long lastMqttRetry = 0;
unsigned long lastMqttPing = 0;
unsigned long lastMqttPub = 0;
bool mqttConnected = false;

bool tempPending = false;
bool oledReady = false;

unsigned long servoAt = 0;
unsigned long phaseAt = 0;
unsigned long cycleAt = 0;

int servoPhase = 0;
int validCount = 0;
int badCount = 0;

float recent[5] = {0};
int recentN = 0;
int recentPos = 0;

enum Mode {
  IDLE,
  DRAIN,
  SETTLE,
  FILL,
  FAULT
};

Mode mode = IDLE;

// Khai bao nguyen mau ham phuc vu Cloud
void publishCloudState();
void publishCloudConfig();
void publishCloudToast(String msg, bool ok = true);

// =====================================
// RELAY CONTROL
// =====================================

void relaySet(int ch, bool on) {
  if (ch < 1 || ch > 4) return;

  if (ch == 1 && on && relayState[1]) return;
  if (ch == 2 && on && relayState[0]) return;

  digitalWrite(
    relayPins[ch - 1],
    on ? RELAY_ON : RELAY_OFF
  );

  relayState[ch - 1] = on;
}

void pumpsOff() {
  relaySet(1, false);
  relaySet(2, false);
}

void fault(String reason) {
  pumpsOff();
  mode = FAULT;
  faultText = reason;
  statusText = "LOI: " + reason;
  autoPending = false;
  publishCloudState();
}

void stopCycle() {
  pumpsOff();
  mode = IDLE;
  autoPending = false;
  statusText = "Da dung chu trinh";
  publishCloudState();
}

// =====================================
// LOAD SETTINGS
// =====================================

void loadSettings() {
  prefs.begin("aqua-v3", false);

  drainCm = prefs.getFloat("drain", 12);
  fillCm = prefs.getFloat("fill", 5);
  tankHeightCm = prefs.getFloat("height", 20);

  maxMinutes = prefs.getInt("maxMin", 5);

  feed1 = prefs.getString("feed1", "08:00");
  feed2 = prefs.getString("feed2", "17:00");

  scheduleEnabled = prefs.getBool("schedule", true);

  feedCount = prefs.getInt("feeds", 0);
  lastFeedKey = prefs.getString("lastKey", "");
  deviceId = prefs.getString("devId", "aqua-v3-uet");
  deviceId.trim();
  if (deviceId.length() < 3) deviceId = "aqua-v3-uet";

  if (!isfinite(tankHeightCm) ||
      tankHeightCm < 10 ||
      tankHeightCm > 300) {
    tankHeightCm = 20;
  }

  if (!isfinite(fillCm) ||
      !isfinite(drainCm) ||
      fillCm < 3 ||
      drainCm <= fillCm + 1 ||
      drainCm > tankHeightCm - 2) {

    fillCm = 5;
    drainCm = 12;
    tankHeightCm = 20;
  }

  if (maxMinutes < 1 || maxMinutes > 30) {
    maxMinutes = 5;
  }

  if (feedCount < 0 || feedCount > 5) {
    feedCount = 0;
  }
}

void saveSettings() {
  prefs.putFloat("drain", drainCm);
  prefs.putFloat("fill", fillCm);
  prefs.putFloat("height", tankHeightCm);

  prefs.putInt("maxMin", maxMinutes);

  prefs.putString("feed1", feed1);
  prefs.putString("feed2", feed2);

  prefs.putBool("schedule", scheduleEnabled);
  prefs.putInt("feeds", feedCount);
  prefs.putString("devId", deviceId);
}

// =====================================
// TIME VALIDATION
// =====================================

bool validClock(String s) {
  if (s.length() != 5 || s[2] != ':') return false;

  for (int i : {0, 1, 3, 4}) {
    if (s[i] < '0' || s[i] > '9') return false;
  }

  int h = s.substring(0, 2).toInt();
  int m = s.substring(3, 5).toInt();

  return h < 24 && m < 60;
}

// =====================================
// HC-SR04
// =====================================

float rawDistance() {
  digitalWrite(5, LOW);
  delayMicroseconds(3);

  digitalWrite(5, HIGH);
  delayMicroseconds(10);

  digitalWrite(5, LOW);

  unsigned long t = pulseIn(18, HIGH, 28000UL);

  if (t == 0) return -1;

  float d = t * 0.0343f / 2.0f;

  return (d >= 2 && d <= 400) ? d : -1;
}

void updateDistance() {
  if (millis() - lastDist < 180) return;

  lastDist = millis();

  float d = rawDistance();

  if (d < 0 || d > tankHeightCm + 5) {
    badCount++;

    if (badCount >= 3) {
      distanceCm = -1;
      recentN = 0;
      validCount = 0;
    }

    return;
  }

  badCount = 0;

  recent[recentPos] = d;
  recentPos = (recentPos + 1) % 5;

  if (recentN < 5) recentN++;

  float s = 0;

  for (int i = 0; i < recentN; i++) {
    s += recent[i];
  }

  distanceCm = s / recentN;

  if (validCount < 5) validCount++;
}

// =====================================
// DS18B20
// =====================================

void updateTemperature() {
  if (!tempPending) {
    ds.requestTemperatures();
    lastTempStart = millis();
    tempPending = true;
  }

  else if (millis() - lastTempStart >= 750) {
    float t = ds.getTempCByIndex(0);

    temperatureC =
      (t == DEVICE_DISCONNECTED_C ||
       t < -50 || t > 100) ? -127 : t;

    tempPending = false;
  }
}

// =====================================
// SERVO FEEDING
// =====================================

void startFeed() {
  if (servoBusy || mode == FAULT) return;

  feeder.write(90);

  servoBusy = true;
  servoPhase = 1;
  servoAt = millis();
  publishCloudState();
}

void updateFeed() {
  if (!servoBusy) return;

  if (servoPhase == 1 &&
      millis() - servoAt >= 1000) {

    feeder.write(0);

    servoPhase = 2;
    servoAt = millis();
  }

  else if (servoPhase == 2 &&
           millis() - servoAt >= 600) {

    servoBusy = false;
    servoPhase = 0;

    if (feedCount < 5) feedCount++;

    prefs.putInt("feeds", feedCount);

    if (feedCount >= 5) {
      autoPending = true;
      statusText = "Da du 5 lan an - cho thay nuoc";
    }
    publishCloudState();
  }
}

// =====================================
// START WATER CYCLE
// =====================================

void startCycle() {
  if (mode != IDLE ||
      servoBusy ||
      validCount < 5 ||
      distanceCm < 0) {

    statusText = "Chua the thay nuoc: kiem tra cam bien";
    return;
  }

  if (distanceCm >= drainCm - 0.3f) {
    statusText = "Muc nuoc da thap - khong xa";
    return;
  }

  if (distanceCm < 3 ||
      distanceCm > tankHeightCm) {

    statusText = "Khoang cach khong an toan";
    return;
  }

  pumpsOff();

  cycleAt = millis();
  phaseAt = millis();

  mode = DRAIN;
  statusText = "Dang xa nuoc";

  relaySet(2, true);
  publishCloudState();
}

// =====================================
// WATER CYCLE
// =====================================

void updateCycle() {
  if (mode != DRAIN &&
      mode != FILL &&
      mode != SETTLE) return;

  if (validCount < 5 || distanceCm < 0) {
    fault("Mat tin hieu HC-SR04");
    return;
  }

  if (millis() - cycleAt >
      (unsigned long)maxMinutes * 60000UL) {

    fault("Qua thoi gian chu trinh");
    return;
  }

  if (mode == DRAIN) {

    if (distanceCm >= drainCm) {
      relaySet(2, false);

      mode = SETTLE;
      phaseAt = millis();

      statusText = "Cho nuoc on dinh";
      publishCloudState();
    }
  }

  else if (mode == SETTLE) {

    if (millis() - phaseAt >= 1500) {

      if (distanceCm < drainCm - 0.5f) {
        fault("Muc nuoc xa chua dat");
        return;
      }

      mode = FILL;
      phaseAt = millis();

      relaySet(1, true);

      statusText = "Dang cap nuoc";
      publishCloudState();
    }
  }

  else if (mode == FILL) {

    if (distanceCm <= fillCm) {
      relaySet(1, false);

      mode = IDLE;
      statusText = "Thay nuoc hoan tat";

      autoPending = false;
      feedCount = 0;

      prefs.putInt("feeds", feedCount);
      publishCloudState();
    }
  }
}

// =====================================
// AUTOMATIC FEEDING SCHEDULE
// =====================================

void updateSchedule() {
  if (millis() - lastSchedule < 1000) return;

  lastSchedule = millis();

  struct tm t;

  clockValid = getLocalTime(&t, 0) &&
               t.tm_year + 1900 >= 2025;

  if (!clockValid ||
      !scheduleEnabled ||
      servoBusy ||
      mode != IDLE ||
      autoPending) return;

  char hhmm[6];
  char key[20];

  snprintf(
    hhmm, sizeof(hhmm),
    "%02d:%02d",
    t.tm_hour, t.tm_min
  );

  snprintf(
    key, sizeof(key),
    "%04d%02d%02d%02d%02d",
    t.tm_year + 1900,
    t.tm_mon + 1,
    t.tm_mday,
    t.tm_hour,
    t.tm_min
  );

  if ((feed1 == hhmm || feed2 == hhmm) &&
      lastFeedKey != key) {

    lastFeedKey = key;

    prefs.putString("lastKey", lastFeedKey);

    startFeed();
  }
}

// =====================================
// OLED
// =====================================

void updateOLED() {
  if (millis() - lastOled < 500) return;

  lastOled = millis();

  oled.clearDisplay();
  oled.setTextColor(SSD1306_WHITE);
  oled.setTextSize(1);
  oled.setCursor(0, 0);

  oled.print("AQUA V3 ");
  oled.println(mqttConnected ? "[CLOUD OK]" : "[LOCAL]");

  oled.print("D: ");

  if (distanceCm >= 0) oled.print(distanceCm, 1);
  else oled.print("ERR");

  oled.println(" cm");

  oled.print("T: ");

  if (temperatureC > -100) oled.print(temperatureC, 1);
  else oled.print("ERR");

  oled.println(" C");

  oled.print("Feed: ");
  oled.print(feedCount);
  oled.println("/5");

  oled.print("R1:");
  oled.print(relayState[0]);

  oled.print(" R2:");
  oled.print(relayState[1]);

  oled.print(" R3:");
  oled.println(relayState[2]);

  oled.println(
    WiFi.status() == WL_CONNECTED
    ? WiFi.localIP().toString()
    : "WiFi disconnected"
  );

  oled.display();
}

// =====================================
// WEB JSON & CORS
// =====================================

void addCors() {
  server.sendHeader("Access-Control-Allow-Origin", "*");
  server.sendHeader("Access-Control-Allow-Methods", "GET, POST, OPTIONS");
  server.sendHeader("Access-Control-Allow-Headers", "Content-Type");
}

void sendMsg(String msg, int code = 200) {
  addCors();
  server.send(
    code,
    "application/json",
    "{\"message\":\"" + msg + "\"}"
  );
}

String jsonBool(bool v) {
  return v ? "true" : "false";
}

String buildDataJson() {
  String j = "{";

  j += "\"distance\":" + String(distanceCm, 2) + ",";
  j += "\"temp\":" + String(temperatureC, 2) + ",";

  j += "\"drain\":" + String(drainCm, 1) + ",";
  j += "\"fill\":" + String(fillCm, 1) + ",";
  j += "\"height\":" + String(tankHeightCm, 1) + ",";

  j += "\"feeds\":" + String(feedCount) + ",";
  j += "\"pending\":" + jsonBool(autoPending) + ",";
  j += "\"servo\":" + jsonBool(servoBusy) + ",";

  j += "\"mode\":" + String((int)mode) + ",";
  j += "\"status\":\"" + statusText + "\",";

  j += "\"r1\":" + jsonBool(relayState[0]) + ",";
  j += "\"r2\":" + jsonBool(relayState[1]) + ",";
  j += "\"r3\":" + jsonBool(relayState[2]) + ",";
  j += "\"r4\":" + jsonBool(relayState[3]) + ",";

  j += "\"clock\":" + jsonBool(clockValid) + ",";
  j += "\"cloud\":" + jsonBool(mqttConnected) + ",";
  j += "\"devId\":\"" + deviceId + "\",";
  j += "\"ip\":\"" + WiFi.localIP().toString() + "\"";

  j += "}";
  return j;
}

String buildConfigJson() {
  String j = "{";

  j += "\"feed1\":\"" + feed1 + "\",";
  j += "\"feed2\":\"" + feed2 + "\",";
  j += "\"schedule\":" + jsonBool(scheduleEnabled);

  j += ",\"drain\":" + String(drainCm, 1);
  j += ",\"fill\":" + String(fillCm, 1);
  j += ",\"height\":" + String(tankHeightCm, 1);
  j += ",\"maxMinutes\":" + String(maxMinutes);
  j += ",\"devId\":\"" + deviceId + "\"";

  j += "}";
  return j;
}

void sendData() {
  addCors();
  server.send(200, "application/json", buildDataJson());
}

void getConfig() {
  addCors();
  server.send(200, "application/json", buildConfigJson());
}

// =====================================
// XU LY LENH DUNG CHUNG (LOCAL & CLOUD)
// =====================================

bool applyConfigParams(String a, String b, float d, float f, float h, int m, bool sched, String newDevId, String &outMsg) {
  if (mode != IDLE) {
    outMsg = "Dung chu trinh truoc khi doi nguong";
    return false;
  }

  if (!validClock(a) ||
      !validClock(b) ||
      !isfinite(d) ||
      !isfinite(f) ||
      !isfinite(h) ||
      h < 10 ||
      h > 300 ||
      f < 3 ||
      d <= f + 1 ||
      d > h - 2 ||
      m < 1 ||
      m > 30) {

    outMsg = "Cau hinh khong hop le: can 3 <= cap < xa < chieu cao be";
    return false;
  }

  feed1 = a;
  feed2 = b;
  drainCm = d;
  fillCm = f;
  tankHeightCm = h;
  maxMinutes = m;
  scheduleEnabled = sched;

  newDevId.trim();
  bool devIdChanged = false;
  if (newDevId.length() >= 3 && newDevId != deviceId) {
    deviceId = newDevId;
    devIdChanged = true;
  }

  saveSettings();
  if (devIdChanged && mqttNet.connected()) {
    mqttNet.stop();
    mqttConnected = false;
  } else {
    publishCloudConfig();
    publishCloudState();
  }

  outMsg = "Da luu cau hinh thanh cong";
  return true;
}

void saveConfig() {
  if (!server.hasArg("feed1") ||
      !server.hasArg("feed2") ||
      !server.hasArg("drain") ||
      !server.hasArg("fill") ||
      !server.hasArg("height") ||
      !server.hasArg("maxMinutes") ||
      !server.hasArg("schedule")) {

    sendMsg("Thieu tham so", 400);
    return;
  }

  String msg;
  String newId = server.hasArg("devId") ? server.arg("devId") : deviceId;
  bool ok = applyConfigParams(
    server.arg("feed1"),
    server.arg("feed2"),
    server.arg("drain").toFloat(),
    server.arg("fill").toFloat(),
    server.arg("height").toFloat(),
    server.arg("maxMinutes").toInt(),
    server.arg("schedule") == "1",
    newId,
    msg
  );

  sendMsg(msg, ok ? 200 : (mode != IDLE ? 409 : 400));
}

// =====================================
// GIAO THUC CLOUD MQTT NHE (KHONG CAN THU VIEN)
// =====================================

void mqttWriteRemLen(uint8_t* buf, int &pos, int len) {
  do {
    uint8_t b = len % 128;
    len /= 128;
    if (len > 0) b |= 128;
    buf[pos++] = b;
  } while (len > 0);
}

bool mqttPublishRaw(String topic, String payload, bool retain = false) {
  if (!mqttNet.connected()) return false;
  int tLen = topic.length();
  int pLen = payload.length();
  int remLen = 2 + tLen + pLen;

  uint8_t hdr[6];
  int hPos = 0;
  hdr[hPos++] = retain ? 0x31 : 0x30;
  mqttWriteRemLen(hdr, hPos, remLen);

  mqttNet.write(hdr, hPos);
  uint8_t tl[2] = {(uint8_t)(tLen >> 8), (uint8_t)(tLen & 0xFF)};
  mqttNet.write(tl, 2);
  mqttNet.print(topic);
  mqttNet.print(payload);
  return true;
}

bool mqttSubscribeRaw(String topic) {
  if (!mqttNet.connected()) return false;
  int tLen = topic.length();
  int remLen = 2 + 2 + tLen + 1;

  uint8_t hdr[6];
  int hPos = 0;
  hdr[hPos++] = 0x82;
  mqttWriteRemLen(hdr, hPos, remLen);

  mqttNet.write(hdr, hPos);
  uint8_t pktId[2] = {0x00, 0x01};
  mqttNet.write(pktId, 2);
  uint8_t tl[2] = {(uint8_t)(tLen >> 8), (uint8_t)(tLen & 0xFF)};
  mqttNet.write(tl, 2);
  mqttNet.print(topic);
  uint8_t qos = 0x00;
  mqttNet.write(&qos, 1);
  return true;
}

void publishCloudState() {
  if (!mqttConnected) return;
  mqttPublishRaw("aquasmart_v3/" + deviceId + "/data", buildDataJson(), true);
}

void publishCloudConfig() {
  if (!mqttConnected) return;
  mqttPublishRaw("aquasmart_v3/" + deviceId + "/config", buildConfigJson(), true);
}

void publishCloudToast(String msg, bool ok) {
  if (!mqttConnected) return;
  String j = "{\"ok\":" + jsonBool(ok) + ",\"message\":\"" + msg + "\"}";
  mqttPublishRaw("aquasmart_v3/" + deviceId + "/resp", j, false);
}

String getQueryVal(String qs, String key) {
  String k = key + "=";
  int idx = qs.indexOf(k);
  while (idx != -1) {
    if (idx == 0 || qs[idx - 1] == '&') {
      int endIdx = qs.indexOf('&', idx + k.length());
      if (endIdx == -1) return qs.substring(idx + k.length());
      return qs.substring(idx + k.length(), endIdx);
    }
    idx = qs.indexOf(k, idx + 1);
  }
  return "";
}

void handleCloudCommand(String cmd) {
  cmd.trim();
  if (cmd == "SYNC") {
    publishCloudConfig();
    publishCloudState();
  }
  else if (cmd == "FEED") {
    if (servoBusy || mode == FAULT || autoPending || feedCount >= 5) {
      publishCloudToast("Dang ban hoac cho thay nuoc", false);
      return;
    }
    startFeed();
    publishCloudToast("Da bat dau cho an", true);
  }
  else if (cmd == "WATER_START") {
    startCycle();
    if (mode == DRAIN) {
      publishCloudToast("Da bat dau xa nuoc", true);
    } else {
      publishCloudToast(statusText, false);
    }
  }
  else if (cmd == "WATER_STOP") {
    stopCycle();
    publishCloudToast("Da dung chu trinh", true);
  }
  else if (cmd.startsWith("RELAY:")) {
    int ch = cmd.substring(6).toInt();
    if (ch != 3 && ch != 4) {
      publishCloudToast("Chi dieu khien duoc relay 3/4", false);
      return;
    }
    relaySet(ch, !relayState[ch - 1]);
    publishCloudState();
    publishCloudToast("Da doi trang thai relay", true);
  }
  else if (cmd.startsWith("CFG:")) {
    String qs = cmd.substring(4);
    String msg;
    String newId = getQueryVal(qs, "devId");
    if (newId == "") newId = deviceId;
    bool ok = applyConfigParams(
      getQueryVal(qs, "feed1"),
      getQueryVal(qs, "feed2"),
      getQueryVal(qs, "drain").toFloat(),
      getQueryVal(qs, "fill").toFloat(),
      getQueryVal(qs, "height").toFloat(),
      getQueryVal(qs, "maxMinutes").toInt(),
      getQueryVal(qs, "schedule") == "1",
      newId,
      msg
    );
    publishCloudToast(msg, ok);
  }
}

bool connectCloudMqtt() {
  if (!mqttNet.connect(MQTT_HOST, MQTT_PORT, 1500)) return false;

  String cid = "esp32-" + deviceId + "-" + String(random(1000, 9999));
  int cLen = cid.length();
  int remLen = 10 + 2 + cLen;

  uint8_t pkt[64];
  int pos = 0;
  pkt[pos++] = 0x10;
  mqttWriteRemLen(pkt, pos, remLen);
  pkt[pos++] = 0x00; pkt[pos++] = 0x04;
  pkt[pos++] = 'M';  pkt[pos++] = 'Q'; pkt[pos++] = 'T'; pkt[pos++] = 'T';
  pkt[pos++] = 0x04;
  pkt[pos++] = 0x02;
  pkt[pos++] = 0x00; pkt[pos++] = 0x3C;
  pkt[pos++] = (uint8_t)(cLen >> 8);
  pkt[pos++] = (uint8_t)(cLen & 0xFF);

  mqttNet.write(pkt, pos);
  mqttNet.print(cid);

  unsigned long t0 = millis();
  while (mqttNet.available() < 4) {
    if (millis() - t0 > 1200) {
      mqttNet.stop();
      return false;
    }
    delay(5);
  }

  uint8_t ack[4];
  mqttNet.read(ack, 4);
  if (ack[0] != 0x20 || ack[3] != 0x00) {
    mqttNet.stop();
    return false;
  }

  mqttConnected = true;
  lastMqttPing = millis();
  mqttSubscribeRaw("aquasmart_v3/" + deviceId + "/cmd");
  publishCloudConfig();
  publishCloudState();
  Serial.println("Da ket noi Cloud MQTT! Device ID: " + deviceId);
  return true;
}

void updateCloudMqtt() {
  if (WiFi.status() != WL_CONNECTED) {
    mqttConnected = false;
    return;
  }

  if (!mqttNet.connected()) {
    mqttConnected = false;
    // Chi thu ket noi lai khi he thong ranh de khong anh huong Servo va Bom
    if (mode == IDLE && !servoBusy && millis() - lastMqttRetry >= 8000) {
      lastMqttRetry = millis();
      connectCloudMqtt();
    }
    return;
  }

  // Doc goi tin MQTT den (Non-blocking)
  while (mqttNet.available() > 0) {
    uint8_t hdr = mqttNet.read();
    int mult = 1;
    int remLen = 0;
    unsigned long t0 = millis();
    uint8_t enc;
    do {
      while (!mqttNet.available()) {
        if (millis() - t0 > 300) return;
      }
      enc = mqttNet.read();
      remLen += (enc & 127) * mult;
      mult *= 128;
    } while ((enc & 128) != 0);

    if (remLen <= 0) continue;
    if (remLen > 512) {
      while (remLen-- > 0 && mqttNet.available()) mqttNet.read();
      continue;
    }

    uint8_t buf[513];
    int nRead = 0;
    t0 = millis();
    while (nRead < remLen && millis() - t0 < 400) {
      if (mqttNet.available()) buf[nRead++] = mqttNet.read();
    }
    if (nRead < remLen) continue;

    if ((hdr & 0xF0) == 0x30 && remLen >= 2) {
      int tLen = (buf[0] << 8) | buf[1];
      int offset = 2 + tLen;
      if ((hdr & 0x06) != 0) offset += 2;
      if (offset <= remLen) {
        String payload = "";
        for (int i = offset; i < remLen; i++) payload += (char)buf[i];
        handleCloudCommand(payload);
      }
    }
  }

  // Gui Ping giu ket noi moi 25s
  if (millis() - lastMqttPing >= 25000) {
    lastMqttPing = millis();
    uint8_t ping[2] = {0xC0, 0x00};
    mqttNet.write(ping, 2);
  }

  // Dong bo du lieu len Cloud moi 1.2s
  if (millis() - lastMqttPub >= 1200) {
    lastMqttPub = millis();
    publishCloudState();
  }
}

// =====================================
// WEB PAGE (HO TRO CA CUNG WIFI & KHAC WIFI/4G)
// =====================================

const char PAGE[] PROGMEM = R"HTML(
<!DOCTYPE html>
<html lang="vi">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Aqua Smart V3</title>
<script src="https://unpkg.com/mqtt/dist/mqtt.min.js"></script>

<style>
:root{
 --navy:#0c2539;
 --blue:#057ca4;
 --teal:#10b3ae;
 --ink:#18354b;
 --muted:#73889b;
 --bg:#eef5f9
}

*{box-sizing:border-box}

body{
 margin:0;
 background:var(--bg);
 font-family:system-ui,-apple-system,Segoe UI,sans-serif;
 color:var(--ink)
}

header{
 background:linear-gradient(120deg,#09243b,#08698c);
 color:#fff;
 padding:26px max(18px,calc((100vw - 1100px)/2))
}

header h1{
 margin:0;
 font-size:28px;
 letter-spacing:.5px;
 display:flex;
 align-items:center;
 gap:10px;
 flex-wrap:wrap
}

header p{
 margin:8px 0 0;
 color:#c6e5ef;
 font-size:13px
}

.remoteBar{
 margin-top:12px;
 background:rgba(255,255,255,0.12);
 border:1px solid rgba(255,255,255,0.22);
 border-radius:12px;
 padding:10px 12px;
 display:flex;
 align-items:center;
 justify-content:space-between;
 gap:10px;
 flex-wrap:wrap
}

.remoteBar input{
 width:165px;
 padding:7px 10px;
 font-size:13px;
 border-radius:8px;
 border:0;
 font-weight:700;
 color:#09243b
}

.wrap{
 max-width:1100px;
 margin:22px auto;
 padding:0 16px
}

.layout{
 display:grid;
 grid-template-columns:1.15fr 1fr;
 gap:16px
}

.card{
 background:white;
 border:1px solid #e0ebf2;
 border-radius:20px;
 padding:20px;
 margin-bottom:16px;
 box-shadow:0 8px 28px #103c5610
}

.card h2{
 font-size:17px;
 margin:0 0 14px
}

.muted{
 color:var(--muted);
 font-size:13px
}

.metrics{
 display:grid;
 grid-template-columns:1fr 1fr;
 gap:10px;
 margin-bottom:14px
}

.metric{
 background:#f2f8fb;
 border-radius:14px;
 padding:14px
}

.metric strong{
 display:block;
 font-size:28px;
 color:#066c92;
 margin-top:4px
}

.tankWrap{
 display:flex;
 gap:20px;
 align-items:center;
 justify-content:center;
 padding:8px 0
}

.tank{
 width:min(280px,65vw);
 height:300px;
 position:relative;
 border:7px solid #92b7c9;
 border-top:10px solid #b9d4e1;
 border-radius:12px 12px 24px 24px;
 overflow:hidden;
 background:linear-gradient(#f1fcff,#e5f5f8);
 box-shadow:inset 0 0 18px #8bb8ca44
}

.water{
 position:absolute;
 bottom:0;
 left:0;
 width:100%;
 height:0%;
 background:linear-gradient(180deg,#4fd5ed,#057ba9);
 transition:height 1.3s ease;
 overflow:hidden
}

.water:before{
 content:"";
 position:absolute;
 top:-10px;
 left:-10%;
 width:120%;
 height:22px;
 border-radius:50%;
 background:#b8f6ff9e;
 animation:wave 2.8s ease-in-out infinite alternate
}

@keyframes wave{
 from{transform:translateX(-7px)}
 to{transform:translateX(7px)}
}

.mark{
 position:absolute;
 left:0;
 width:100%;
 border-top:2px dashed;
 z-index:2;
 pointer-events:none
}

.mark span{
 position:absolute;
 right:4px;
 top:-20px;
 font-size:11px;
 font-weight:700;
 background:#ffffffd9;
 padding:2px 5px;
 border-radius:4px
}

.mark.high{
 border-color:#10a977;
 color:#086f52
}

.mark.low{
 border-color:#f19c36;
 color:#9b5507
}

.fish{
 position:absolute;
 bottom:22%;
 left:26%;
 font-size:40px;
 z-index:1;
 animation:swim 4s ease-in-out infinite alternate
}

@keyframes swim{
 from{transform:translateX(-10px)}
 to{transform:translateX(32px)}
}

.info{min-width:125px}

.info b{
 display:block;
 font-size:16px;
 margin:5px 0 16px
}

.pill{
 display:inline-block;
 padding:6px 10px;
 border-radius:30px;
 background:#e2f5f3;
 color:#087e77;
 font-size:12px;
 font-weight:700
}

.row{
 display:flex;
 align-items:center;
 justify-content:space-between;
 gap:10px;
 padding:12px 0;
 border-bottom:1px solid #e9f0f4
}

.row:last-child{border:0}

button{
 cursor:pointer;
 border:0;
 border-radius:11px;
 padding:11px 14px;
 font-weight:700;
 background:#087fa5;
 color:white
}

button.secondary{
 background:#e9f2f6;
 color:#1c536c
}

button.danger{background:#dc5363}
button.green{background:#0a9a78}
button:disabled{opacity:.5;cursor:not-allowed}

.actions{
 display:flex;
 gap:8px;
 flex-wrap:wrap;
 margin-top:12px
}

label{
 font-size:13px;
 font-weight:700;
 display:block;
 margin:12px 0 5px
}

input{
 border:1px solid #cbdde7;
 border-radius:10px;
 padding:11px;
 width:100%;
 font-size:15px
}

.two{
 display:grid;
 grid-template-columns:1fr 1fr;
 gap:12px
}

.notice{
 background:#eff8fb;
 border-radius:12px;
 padding:12px;
 margin-top:12px;
 font-size:13px
}

.toast{
 position:fixed;
 bottom:18px;
 left:50%;
 transform:translateX(-50%);
 background:#15364c;
 color:white;
 border-radius:12px;
 padding:12px 18px;
 display:none;
 max-width:90vw;
 z-index:10
}

.count{
 font-size:36px;
 font-weight:800;
 color:#087fa5
}

.status{
 font-weight:700;
 color:#0d7b8b
}

@media(max-width:760px){
 .layout{grid-template-columns:1fr}
 .tankWrap{gap:10px}
 .info{min-width:105px}
 .tank{height:260px}
 header{padding:24px 18px}
}
</style>
</head>

<body>

<header>
<h1>
 🐠 AQUA SMART <span style="font-size:13px;opacity:.8">V3.0</span>
 <span class="pill" id="modeBadge" style="background:#0a9a78;color:#fff">Đang kiểm tra mạng...</span>
</h1>
<p>Giám sát mực nước • Cho ăn thông minh • Tự động thay nước (Hỗ trợ Cùng WiFi & Khác WiFi/4G)</p>
<p id="connection">Đang kết nối ESP32...</p>

<div class="remoteBar">
 <div style="display:flex;align-items:center;gap:8px;flex-wrap:wrap">
  <span style="font-size:13px;font-weight:700">🔑 Mã thiết bị từ xa:</span>
  <input id="devId" type="text" value="aqua-v3-uet" placeholder="Nhập mã thiết bị">
  <button class="secondary" style="padding:7px 11px;font-size:12px" onclick="reconnectCloud()">Kết nối Cloud</button>
 </div>
 <button class="green" style="padding:8px 12px;font-size:12px" onclick="downloadRemoteApp()">📲 Tải File Điều Khiển Từ Xa (Khác WiFi / 4G)</button>
</div>
</header>

<div class="wrap">
<div class="layout">

<section>

<div class="card">
<h2>🌊 Mực nước trong bể</h2>

<div class="metrics">
<div class="metric">
<span class="muted">Khoảng cách cảm biến</span>
<strong id="distance">-- cm</strong>
</div>

<div class="metric">
<span class="muted">Nhiệt độ nước</span>
<strong id="temp">-- °C</strong>
</div>
</div>

<div class="tankWrap">

<div class="tank">
<div class="water" id="water">
<span class="fish">🐟</span>
</div>

<div class="mark high" id="highMark">
<span id="highLabel">Ngưỡng cấp</span>
</div>

<div class="mark low" id="lowMark">
<span id="lowLabel">Ngưỡng xả</span>
</div>
</div>

<div class="info">
<span class="muted">Chiều cao nước ước tính</span>
<b id="level">-- cm</b>

<span class="muted">Xu hướng</span>
<b id="trend">Đang đo...</b>

<span class="muted">Chu trình</span>
<b id="cycle">Chờ</b>
</div>

</div>

<div class="notice" id="status">Đang lấy dữ liệu...</div>

<p class="muted">
Mô phỏng theo HC-SR04. Chiều cao bể là khoảng cách từ cảm biến đến đáy;
ngưỡng là khoảng cách từ cảm biến xuống mặt nước.
</p>
</div>

<div class="card">
<h2>💧 Tự động thay nước</h2>

<div class="two">
<div>
<label>Ngưỡng dừng xả (cm)</label>
<input id="drain" type="number" step="0.1" min="3">
</div>

<div>
<label>Ngưỡng dừng cấp (cm)</label>
<input id="fill" type="number" step="0.1" min="3">
</div>
</div>

<div class="two">
<div>
<label>Cảm biến đến đáy bể (cm)</label>
<input id="height" type="number" step="0.1" min="10">
</div>

<div>
<label>Giới hạn chu trình (phút)</label>
<input id="maxMinutes" type="number" min="1" max="30">
</div>
</div>

<div class="actions">
<button onclick="saveConfig()">Lưu ngưỡng</button>
<button class="green" onclick="act('/water/start')">Bắt đầu thay nước</button>
<button class="danger" onclick="act('/water/stop')">Dừng bơm</button>
</div>
</div>

</section>

<section>

<div class="card">
<h2>🍽️ Cho ăn thông minh</h2>

<div class="row">
<div>
<span class="muted">Số lượt đã cho ăn</span>
<div class="count"><span id="feeds">0</span>/5</div>
</div>

<span class="pill" id="feedState">Sẵn sàng</span>
</div>

<p class="muted">
Sau lượt thứ 5, hệ thống tự thay nước theo ngưỡng đã lưu.
Bộ đếm chỉ trở về 0 sau khi thay nước thành công.
</p>

<button class="green" onclick="act('/feed')">🍚 Cho ăn ngay</button>

<div class="row">
<label for="schedule">Bật lịch tự động</label>
<input id="schedule" type="checkbox" style="width:22px;height:22px">
</div>

<div class="two">
<div>
<label>Giờ ăn 1</label>
<input id="feed1" type="time">
</div>

<div>
<label>Giờ ăn 2</label>
<input id="feed2" type="time">
</div>
</div>

<div class="actions">
<button onclick="saveConfig()">Lưu lịch cho ăn</button>
</div>
</div>

<div class="card">
<h2>⚙️ Trạng thái thiết bị</h2>

<div class="row">
<span>Bơm 1 • Cấp nước</span>
<b id="r1">OFF</b>
</div>

<div class="row">
<span>Bơm 2 • Xả nước</span>
<b id="r2">OFF</b>
</div>

<div class="row">
<span>Bơm 3 • Sục khí</span>
<b id="r3">OFF</b>
<button class="secondary" onclick="act('/relay?ch=3')">Bật/Tắt</button>
</div>

<div class="row">
<span>Relay 4 • Đèn LED</span>
<b id="r4">OFF</b>
<button class="secondary" onclick="act('/relay?ch=4')">Bật/Tắt</button>
</div>

<div class="notice">
Khi cảm biến mất tín hiệu hoặc chu trình quá thời gian,
bơm cấp và xả sẽ dừng. Hãy kiểm tra thiết bị trước khi thử lại.
</div>
</div>

</section>
</div>
</div>

<div id="toast" class="toast"></div>

<script>
const $ = id => document.getElementById(id);

let previous = null;
let previousAt = 0;
let configLoaded = false;

// Trang thai ket noi: uu tien Local HTTP neu dang mo truc tiep tren IP ESP32,
// tu dong chuyen sang Cloud MQTT over WebSocket khi mo khac WiFi / 4G
let isLocalMode = (location.protocol === 'http:' && location.hostname !== '');
let mqttClient = null;
let cloudConnected = false;

const savedDevId = localStorage.getItem('aqua_v3_devId');
if (savedDevId) $('devId').value = savedDevId;

function toast(s) {
  $('toast').textContent = s;
  $('toast').style.display = 'block';

  setTimeout(() => {
    $('toast').style.display = 'none';
  }, 3500);
}

function applyConfigToInputs(c) {
  for (const k of ['feed1','feed2','drain','fill','height','maxMinutes']) {
    if (c[k] !== undefined && document.activeElement !== $(k)) {
      $(k).value = c[k];
    }
  }
  if (c.schedule !== undefined) $('schedule').checked = c.schedule;
  if (c.devId && document.activeElement !== $('devId')) {
    $('devId').value = c.devId;
    localStorage.setItem('aqua_v3_devId', c.devId);
  }
  configLoaded = true;
}

function renderData(d) {
  const modeStr = isLocalMode
    ? ('Cùng WiFi (IP: ' + d.ip + (d.cloud ? ' • Cloud: Sẵn sàng' : '') + ')')
    : ('Điều khiển từ xa qua Internet (IP bể: ' + d.ip + ')');

  $('connection').textContent =
    'ESP32: ' + modeStr +
    ' • Đồng hồ: ' +
    (d.clock ? 'Đã đồng bộ' : 'Chưa đồng bộ NTP');

  $('modeBadge').textContent = isLocalMode ? '🏠 Cùng WiFi (LAN)' : '🌐 Khác WiFi / 4G (Cloud)';
  $('modeBadge').style.background = isLocalMode ? '#0a9a78' : '#0284c7';

  $('distance').textContent =
    d.distance >= 0
    ? d.distance.toFixed(1) + ' cm'
    : 'LỖI';

  $('temp').textContent =
    d.temp > -100
    ? d.temp.toFixed(1) + ' °C'
    : '--';

  $('feeds').textContent = d.feeds;

  $('feedState').textContent =
    d.servo
    ? 'Đang cho ăn'
    : d.pending
    ? 'Chờ thay nước'
    : 'Sẵn sàng';

  $('status').textContent = d.status;

  ['r1','r2','r3','r4'].forEach(k => {
    $(k).textContent = d[k] ? 'ON' : 'OFF';
  });

  $('cycle').textContent = [
    'Chờ',
    'Đang xả ↓',
    'Chờ ổn định',
    'Đang cấp ↑',
    'LỖI'
  ][d.mode] || '--';

  const h = d.height;
  const dist = d.distance;

  if (dist >= 0 && h > 0) {
    const pct = Math.max(
      0,
      Math.min(100, (h - dist) / h * 100)
    );

    $('water').style.height = pct + '%';

    $('level').textContent =
      Math.max(0, h - dist).toFixed(1) + ' cm';

    if (previous !== null &&
        Date.now() - previousAt >= 1500) {

      const delta = dist - previous;

      $('trend').textContent =
        delta > 0.35
        ? 'Nước đang hạ ↓'
        : delta < -0.35
        ? 'Nước đang dâng ↑'
        : 'Gần như ổn định';

      previous = dist;
      previousAt = Date.now();

    } else if (previous === null) {
      previous = dist;
      previousAt = Date.now();
    }

  } else {
    $('trend').textContent = 'Không có tín hiệu';
    $('water').style.height = '0%';
    previous = null;
  }

  $('highMark').style.bottom =
    Math.max(
      0,
      Math.min(100, (h - d.fill) / h * 100)
    ) + '%';

  $('lowMark').style.bottom =
    Math.max(
      0,
      Math.min(100, (h - d.drain) / h * 100)
    ) + '%';

  $('highLabel').textContent =
    'Dừng cấp ' + d.fill.toFixed(1) + 'cm';

  $('lowLabel').textContent =
    'Dừng xả ' + d.drain.toFixed(1) + 'cm';
}

// =====================================
// KET NOI CLOUD MQTT OVER WEBSOCKETS
// =====================================

function initCloudMqtt() {
  if (typeof mqtt === 'undefined') return;
  if (mqttClient) {
    try { mqttClient.end(true); } catch(e) {}
  }

  const devId = ($('devId').value || 'aqua-v3-uet').trim();
  localStorage.setItem('aqua_v3_devId', devId);

  const baseTopic = 'aquasmart_v3/' + devId;
  mqttClient = mqtt.connect('wss://broker.emqx.io:8084/mqtt', {
    clientId: 'web-v3-' + Math.random().toString(16).slice(2, 10),
    clean: true,
    reconnectPeriod: 3000
  });

  mqttClient.on('connect', () => {
    cloudConnected = true;
    mqttClient.subscribe([baseTopic + '/data', baseTopic + '/config', baseTopic + '/resp']);
    mqttClient.publish(baseTopic + '/cmd', 'SYNC');
    if (!isLocalMode) {
      $('connection').textContent = 'Đã kết nối Cloud (' + devId + ') • Đang đợi dữ liệu từ ESP32...';
    }
  });

  mqttClient.on('message', (topic, message) => {
    try {
      const payload = JSON.parse(message.toString());
      if (topic.endsWith('/data')) {
        if (!isLocalMode) renderData(payload);
      } else if (topic.endsWith('/config')) {
        if (!isLocalMode || !configLoaded) applyConfigToInputs(payload);
      } else if (topic.endsWith('/resp')) {
        if (!isLocalMode && payload.message) toast(payload.message);
      }
    } catch(e) {}
  });
}

function reconnectCloud() {
  isLocalMode = false;
  initCloudMqtt();
  toast('Đang kết nối Cloud với mã: ' + $('devId').value);
}

// Tai file HTML doc lap de mo tren dien thoai / may tinh khi khac WiFi / 4G
function downloadRemoteApp() {
  let html = document.documentElement.outerHTML;
  const currentId = ($('devId').value || 'aqua-v3-uet').trim();
  html = html.replace('value="aqua-v3-uet"', 'value="' + currentId + '"');
  const blob = new Blob(['<!DOCTYPE html>\n' + html], { type: 'text/html;charset=utf-8' });
  const a = document.createElement('a');
  a.href = URL.createObjectURL(blob);
  a.download = 'AquaSmart_V3_TuXa_' + currentId + '.html';
  a.click();
  toast('Đã tải file điều khiển từ xa! Mở file này trên 4G/WiFi bất kỳ để điều khiển.');
}

async function api(url, options = {}) {
  const ctrl = new AbortController();
  const timer = setTimeout(() => ctrl.abort(), 2200);
  options.signal = ctrl.signal;

  const r = await fetch(url, options);
  clearTimeout(timer);
  const d = await r.json();

  if (!r.ok) {
    throw Error(d.message || ('HTTP ' + r.status));
  }

  return d;
}

async function loadConfig() {
  if (!isLocalMode) return;
  try {
    const c = await api('/config');
    applyConfigToInputs(c);
  } catch(e) {
    isLocalMode = false;
  }
}

async function saveConfig() {
  const p = new URLSearchParams();

  for (const k of [
    'feed1',
    'feed2',
    'drain',
    'fill',
    'height',
    'maxMinutes'
  ]) {
    p.set(k, $(k).value);
  }

  p.set('schedule', $('schedule').checked ? '1' : '0');
  p.set('devId', ($('devId').value || 'aqua-v3-uet').trim());

  if (isLocalMode) {
    try {
      const d = await api('/config', {
        method: 'POST',
        headers: {
          'Content-Type': 'application/x-www-form-urlencoded'
        },
        body: p.toString()
      });

      toast(d.message);
      await loadConfig();
      return;
    } catch(e) {
      if (e.name !== 'AbortError' && e.message && !e.message.includes('Failed to fetch')) {
        return toast(e.message);
      }
      isLocalMode = false;
    }
  }

  // Gui qua Cloud MQTT khi o khac WiFi / 4G
  if (mqttClient && cloudConnected) {
    const devId = ($('devId').value || 'aqua-v3-uet').trim();
    mqttClient.publish('aquasmart_v3/' + devId + '/cmd', 'CFG:' + p.toString());
  } else {
    toast('Chưa kết nối được Cloud MQTT');
  }
}

async function act(url) {
  if (isLocalMode) {
    try {
      const d = await api(url, { method: 'POST' });
      toast(d.message);
      await updateData();
      return;
    } catch(e) {
      if (e.name !== 'AbortError' && e.message && !e.message.includes('Failed to fetch')) {
        return toast(e.message);
      }
      isLocalMode = false;
    }
  }

  // Chuyen lenh sang Cloud MQTT khi dieu khien khac WiFi / 4G
  if (!mqttClient || !cloudConnected) {
    return toast('Đang kết nối Cloud MQTT, vui lòng thử lại...');
  }

  const devId = ($('devId').value || 'aqua-v3-uet').trim();
  const cmdTopic = 'aquasmart_v3/' + devId + '/cmd';

  if (url === '/feed') {
    mqttClient.publish(cmdTopic, 'FEED');
  } else if (url === '/water/start') {
    mqttClient.publish(cmdTopic, 'WATER_START');
  } else if (url === '/water/stop') {
    mqttClient.publish(cmdTopic, 'WATER_STOP');
  } else if (url.startsWith('/relay?ch=')) {
    const ch = url.split('=')[1];
    mqttClient.publish(cmdTopic, 'RELAY:' + ch);
  }
}

async function updateData() {
  if (!isLocalMode) return;
  try {
    const d = await api('/data');
    renderData(d);
  } catch(e) {
    isLocalMode = false;
    $('modeBadge').textContent = '🌐 Khác WiFi / 4G (Cloud)';
    $('modeBadge').style.background = '#0284c7';
    $('connection').textContent = 'Chuyển sang chế độ Điều khiển từ xa (Khác WiFi / 4G)...';
  }
}

initCloudMqtt();
loadConfig();
updateData();

setInterval(updateData, 1000);
</script>

</body>
</html>
)HTML";

// =====================================
// SETUP
// =====================================

void setup() {
  Serial.begin(115200);

  for (int i = 0; i < 4; i++) {
    digitalWrite(relayPins[i], RELAY_OFF);
    pinMode(relayPins[i], OUTPUT);
    digitalWrite(relayPins[i], RELAY_OFF);
  }

  pinMode(5, OUTPUT);
  pinMode(18, INPUT);

  digitalWrite(5, LOW);

  loadSettings();

  Wire.begin(21, 22);

  oledReady = oled.begin(
    SSD1306_SWITCHCAPVCC,
    0x3C
  );

  if (!oledReady) {
    Serial.println("OLED ERROR - continue without OLED");
  }

  ds.begin();
  ds.setWaitForConversion(false);

  feeder.setPeriodHertz(50);
  feeder.attach(13, 500, 2400);
  feeder.write(0);

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  unsigned long start = millis();

  while (WiFi.status() != WL_CONNECTED &&
         millis() - start < 15000) {
    delay(250);
  }

  if (WiFi.status() == WL_CONNECTED) {
    configTime(
      7 * 3600,
      0,
      "pool.ntp.org",
      "time.google.com"
    );

    Serial.print("IP Noi bo: ");
    Serial.println(WiFi.localIP());
    Serial.print("Ma thiet bi dieu khien tu xa (Device ID): ");
    Serial.println(deviceId);

    connectCloudMqtt();

  } else {
    Serial.println(
      "WiFi chua ket noi. Thu hotspot 2.4GHz."
    );
  }

  server.on("/", HTTP_GET, []() {
    addCors();
    server.send_P(
      200,
      "text/html; charset=utf-8",
      PAGE
    );
  });

  server.on("/data", HTTP_GET, sendData);
  server.on("/config", HTTP_GET, getConfig);
  server.on("/config", HTTP_POST, saveConfig);

  server.on("/feed", HTTP_POST, []() {
    if (servoBusy ||
        mode == FAULT ||
        autoPending ||
        feedCount >= 5) {

      sendMsg("Dang ban hoac cho thay nuoc", 409);
      return;
    }

    startFeed();
    sendMsg("Da bat dau cho an");
  });

  server.on("/water/start", HTTP_POST, []() {
    startCycle();

    if (mode == DRAIN) {
      sendMsg("Da bat dau xa nuoc");
    } else {
      sendMsg(statusText, 409);
    }
  });

  server.on("/water/stop", HTTP_POST, []() {
    stopCycle();
    sendMsg("Da dung chu trinh");
  });

  server.on("/relay", HTTP_POST, []() {
    int ch = server.arg("ch").toInt();

    if (ch != 3 && ch != 4) {
      sendMsg("Chi dieu khien duoc relay 3/4", 400);
      return;
    }

    relaySet(ch, !relayState[ch - 1]);
    publishCloudState();

    sendMsg("Da doi trang thai relay");
  });

  server.begin();

  // Bat bom suc khi sau khi khoi tao
  relaySet(3, true);

  statusText = "San sang";
}

// =====================================
// LOOP
// =====================================

void loop() {
  server.handleClient();

  updateDistance();
  updateTemperature();
  updateFeed();
  updateSchedule();

  if (autoPending && mode == IDLE && !servoBusy) {
    startCycle();

    if (mode != DRAIN) {
      autoPending = false;
      fault("Khong the tu thay nuoc - kiem tra muc nuoc");
    }
  }

  updateCycle();
  updateCloudMqtt();

  if (oledReady) {
    updateOLED();
  }

  if (millis() - lastWifi > 15000) {
    lastWifi = millis();

    if (WiFi.status() != WL_CONNECTED) {
      WiFi.reconnect();
    }

    else if (!clockValid) {
      configTime(
        7 * 3600,
        0,
        "pool.ntp.org",
        "time.google.com"
      );
    }
  }
}

