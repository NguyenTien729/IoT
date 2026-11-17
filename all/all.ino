/*
 * HỆ THỐNG AN NINH PHÒNG – ESP32 38 chân (bản 6: bản 5 + chế độ an ninh và còi báo động)
 * Serial Monitor: 115200 baud
 *
 * - MQ2: đọc mỗi 50 ms, lọc nhiễu, báo gas khi tăng vọt so với mức nền (phản ứng < 1 s)
 * - PIR: bỏ qua 15 s đầu; một lần HIGH đơn lẻ, ngắn KHÔNG tính là có người.
 *   Báo "có người" khi:
 *     + HIGH kéo dài liên tục từ 4 s (người đang cử động, jumper để H), hoặc
 *     + PIR đang HIGH và trong 5 s gần đó có tiếng động (micro) hoặc cửa vừa mở.
 *   Mỗi lần HIGH kết thúc, code in ra nó kéo dài bao lâu để chỉnh PIR_LONG_MS.
 * - In kết quả mỗi 1 s
 * - Chế độ an ninh (bật/tắt qua MQTT, nhớ cả khi mất điện):
 *     + Bật an ninh -> 10 s đếm ngược (bíp mỗi giây) để rời phòng, sau đó mới có hiệu lực
 *     + Đang bật an ninh mà cửa mở hoặc có người -> còi hú nhanh (báo xâm nhập)
 *     + Có gas -> còi hú chậm, LUÔN báo dù đang tắt an ninh
 *     + Còi hú tối đa 60 s rồi tự dừng; còi không chặn chương trình, cảm biến và MQTT vẫn chạy
 *     + Đang tắt an ninh: chỉ bíp ngắn khi cửa mở hoặc có người như trước
 * - MQTT (Node-RED):
 *     nhom20/phong1/sensors : JSON dữ liệu cảm biến, mỗi 2 s
 *     nhom20/phong1/events  : JSON sự kiện (motion, door, gas), gửi ngay khi xảy ra
 *     nhom20/phong1/status  : "online" / "offline" (retain, offline do broker tự gửi khi mất kết nối)
 *     nhom20/phong1/cmd     : nhận "beep", "buzzer_on", "buzzer_off",
 *                             "arm" (bật an ninh), "disarm" (tắt an ninh), "silence" (tắt còi đang hú)
 *
 * Nối dây:
 *   PIR        : VCC->5V, GND->GND,  OUT->P27
 *   Công tắc   : 1 dây->GND, dây kia->P18
 *   BH1750     : VCC->3V3, GND->GND, SDA->P21, SCL->P22
 *   BMP280     : VCC->3V3, GND->GND, SDA->P21, SCL->P22
 *   MQ2        : VCC->3V3, GND->GND, AO->P34, DO bỏ trống
 *   INMP441    : VDD->3V3, GND+L/R->GND, SD->P35, SCK->P14, WS->P13
 *   Còi KY-012 : S->P26, chân - ->GND, chân giữa bỏ trống
 *
 * Thư viện: Adafruit BMP280 Library, BH1750 (Christopher Laws),
 *           PubSubClient (Nick O'Leary), ArduinoJson (Benoit Blanchon, bản 7).
 * ESP_I2S.h có sẵn trong board esp32 bản 3.x.
 */

#include <Wire.h>
#include <Adafruit_BMP280.h>
#include <BH1750.h>
#include <ESP_I2S.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <Preferences.h>

#define PIN_PIR    27
#define PIN_DOOR   18
#define PIN_MQ2    34
#define PIN_SDA    21
#define PIN_SCL    22
#define I2S_SCK    14
#define I2S_WS     13
#define I2S_SD     35
#define PIN_BUZZER 26

// ===== WiFi + MQTT: sửa theo mạng của nhóm =====
const char *WIFI_SSID = "Nupacachi";          // chỉ WiFi 2.4 GHz
const char *WIFI_PASS = "tuan2005";
const char *MQTT_HOST = "172.20.10.2";      // IP máy chạy broker MQTT (ipconfig -> IPv4)192.168.1.105

// const char *WIFI_SSID = "TP-Link_0C6D";          // chỉ WiFi 2.4 GHz
// const char *WIFI_PASS = "eeeeeeeee";
// const char *MQTT_HOST = "192.168.1.103";
const int   MQTT_PORT = 1883;
const char *MQTT_USER = "";                  // để trống nếu broker không đặt mật khẩu
const char *MQTT_PASS = "";
const unsigned long PUBLISH_MS = 2000;       // gửi dữ liệu cảm biến mỗi 2 s

#define TOPIC_SENSORS "nhom20/phong1/sensors"
#define TOPIC_EVENTS  "nhom20/phong1/events"
#define TOPIC_STATUS  "nhom20/phong1/status"
#define TOPIC_CMD     "nhom20/phong1/cmd"

// ===== Cấu hình lọc: chỉnh các số ở đây =====
const unsigned long REPORT_MS       = 1000;    // in kết quả mỗi 1 s
const unsigned long MQ2_SAMPLE_MS   = 50;     // đọc MQ2 mỗi 50 ms
const unsigned long MQ2_WARMUP_MS   = 60000;  // 60 s đầu chỉ học mức nền, chưa báo gas
const float         GAS_ON_MV       = 300;    // báo gas khi cao hơn mức nền 300 mV
const float         GAS_OFF_MV      = 150;    // hết báo khi còn cao hơn nền dưới 150 mV
const unsigned long PIR_WARMUP_MS   = 15000;  // bỏ qua 15 s đầu sau khi bật
const unsigned long PIR_GLITCH_MS   = 500;    // HIGH ngắn hơn 0.5 s: nhiễu, bỏ
const unsigned long PIR_LONG_MS     = 4000;   // 1 lần HIGH kéo dài từ 4 s: có người
                                              // (đặt dài hơn các lần HIGH nhầm mà code in ra)
const unsigned long PIR_FUSE_MS     = 5000;   // PIR HIGH + tiếng động/cửa mở trong 5 s: có người
const float         SOUND_CONFIRM   = 1000000; // mức âm coi là "có tiếng động" (xem dòng INMP441 để chỉnh)
const unsigned long PIR_HOLD_MS     = 10000;  // giữ trạng thái "có người" thêm 10 s

// ===== Báo động =====
const unsigned long ARM_DELAY_MS    = 10000;  // bật an ninh xong có 10 s để rời phòng
const unsigned long ALARM_MAX_MS    = 60000;  // còi hú tối đa 60 s rồi tự dừng (vẫn giữ chế độ an ninh)

Adafruit_BMP280 bmp;
BH1750 lightMeter;
I2SClass i2s;
bool bmpOK = false, bh1750OK = false, micOK = false;
float soundPeak = 0, soundPeakPub = 0;
WiFiClient net;
PubSubClient mqtt(net);

unsigned long lastBeepEnd = 0, lastLoudMs = 0, lastDoorOpenMs = 0;
bool buzzerManualOn = false; // Bật/tắt còi từ Web qua MQTT

// ---------- Trạng thái an ninh / báo động ----------
Preferences prefs;
bool armed = false;                 // đang bật chế độ an ninh
unsigned long armedAt = 0;          // lúc bật an ninh (để tính 10 s đếm ngược)
enum AlarmType { ALARM_NONE, ALARM_INTRUSION, ALARM_GAS };
AlarmType alarmType = ALARM_NONE;
unsigned long alarmStart = 0;
bool sirenPinOn = false;            // trạng thái chân còi do phần báo động điều khiển

// ---------- Còi ----------
void beep(int ms) {
  // Nếu Web đang bật còi liên tục, hoặc còi đang hú báo động, không chen tiếng bíp vào.
  if (buzzerManualOn || alarmType != ALARM_NONE) return;
  digitalWrite(PIN_BUZZER, HIGH);
  delay(ms);
  digitalWrite(PIN_BUZZER, LOW);
  lastBeepEnd = millis();   // để micro không coi tiếng còi là tiếng người
}

// ---------- WiFi + MQTT ----------
void publishEvent(const char *type, const char *detail) {
  if (!mqtt.connected()) return;
  JsonDocument doc;
  doc["type"] = type;
  doc["detail"] = detail;
  doc["uptime_s"] = millis() / 1000;
  char buf[192];
  size_t n = serializeJson(doc, buf, sizeof(buf));
  mqtt.publish(TOPIC_EVENTS, (const uint8_t *)buf, n, false);
}

const char *alarmName() {
  return alarmType == ALARM_GAS ? "gas" : (alarmType == ALARM_INTRUSION ? "intrusion" : "none");
}

// An ninh đã có hiệu lực chưa (đã bật và hết 10 s đếm ngược)
bool armReady() { return armed && millis() - armedAt >= ARM_DELAY_MS; }

void startAlarm(AlarmType t, const char *why) {
  if (alarmType == ALARM_GAS && t == ALARM_INTRUSION) { alarmStart = millis(); return; }  // gas ưu tiên hơn
  bool isNew = alarmType != t;
  alarmType = t;
  alarmStart = millis();            // còn sự kiện mới thì tính lại 60 s
  if (isNew) {
    Serial.printf("!!! BÁO ĐỘNG: %s\n", why);
    publishEvent("alarm", why);
  }
}

void stopAlarm(const char *why) {
  if (alarmType == ALARM_NONE) return;
  alarmType = ALARM_NONE;
  if (!buzzerManualOn) digitalWrite(PIN_BUZZER, LOW);
  sirenPinOn = false;
  lastBeepEnd = millis();
  Serial.printf("!!! Dừng báo động (%s)\n", why);
  publishEvent("alarm", "stopped");
}

void setArmed(bool on) {
  armed = on;
  armedAt = millis();
  prefs.putBool("armed", on);       // nhớ trạng thái cả khi mất điện
  if (!on) stopAlarm("tắt an ninh");
  publishEvent("armed", on ? "on" : "off");
  Serial.println(on ? "AN NINH: BẬT (10 s nữa có hiệu lực)" : "AN NINH: TẮT");
}

// Gọi liên tục trong loop: điều khiển còi báo động mà không dùng delay
void updateSiren() {
  unsigned long now = millis();
  if (alarmType != ALARM_NONE && now - alarmStart > ALARM_MAX_MS) stopAlarm("hết 60 s");
  if (buzzerManualOn) return;       // web đang bật còi liên tục thì để nguyên

  bool on;
  if (alarmType == ALARM_INTRUSION)  on = (now - alarmStart) % 400 < 200;    // hú nhanh
  else if (alarmType == ALARM_GAS)   on = (now - alarmStart) % 1000 < 500;   // hú chậm
  else on = armed && now - armedAt < ARM_DELAY_MS && (now - armedAt) % 1000 < 60;   // đếm ngược: bíp mỗi giây

  if (on != sirenPinOn) {
    digitalWrite(PIN_BUZZER, on ? HIGH : LOW);
    sirenPinOn = on;
  }
  if (on || alarmType != ALARM_NONE) lastBeepEnd = now;   // micro không coi tiếng còi là tiếng người
}

void onMqttMessage(char *topic, byte *payload, unsigned int len) {
  String msg;
  for (unsigned int i = 0; i < len; i++) msg += (char)payload[i];
  msg.trim();
  Serial.printf("<<< Lệnh MQTT: %s\n", msg.c_str());
  if (msg == "buzzer_on") {
    buzzerManualOn = true;
    digitalWrite(PIN_BUZZER, HIGH);
    lastBeepEnd = millis();
    publishEvent("buzzer", "on");
    Serial.println("COI: BAT bang Web");
  } else if (msg == "buzzer_off") {
    buzzerManualOn = false;
    digitalWrite(PIN_BUZZER, LOW);
    sirenPinOn = false;
    lastBeepEnd = millis();
    stopAlarm("tắt còi bằng web");   // bấm tắt còi trên web thì dừng luôn còi báo động
    publishEvent("buzzer", "off");
    Serial.println("COI: TAT bang Web");
  } else if (msg == "beep") {
    beep(300);
  } else if (msg == "arm") {
    setArmed(true);
  } else if (msg == "disarm") {
    setArmed(false);
  } else if (msg == "silence") {
    stopAlarm("tắt còi, vẫn giữ an ninh");
  }
}

void connectWifi() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  Serial.print("Kết nối WiFi");
  unsigned long t = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t < 10000) { delay(300); Serial.print("."); }
  if (WiFi.status() == WL_CONNECTED) Serial.printf(" OK, IP: %s\n", WiFi.localIP().toString().c_str());
  else Serial.println(" chưa được, sẽ tự thử lại (cảm biến vẫn chạy bình thường)");
}

// Gọi liên tục trong loop: tự nối lại WiFi/MQTT, không làm dừng cảm biến lâu
void keepConnected() {
  static unsigned long lastTry = 0;
  if (WiFi.status() != WL_CONNECTED) {
    if (millis() - lastTry > 5000) { lastTry = millis(); WiFi.reconnect(); }
    return;
  }
  if (mqtt.connected()) { mqtt.loop(); return; }
  if (millis() - lastTry < 3000) return;
  lastTry = millis();
  String id = "nhom20-esp32-" + String((uint32_t)ESP.getEfuseMac(), HEX);
  bool ok = strlen(MQTT_USER)
    ? mqtt.connect(id.c_str(), MQTT_USER, MQTT_PASS, TOPIC_STATUS, 0, true, "offline")
    : mqtt.connect(id.c_str(), TOPIC_STATUS, 0, true, "offline");
  if (ok) {
    mqtt.publish(TOPIC_STATUS, "online", true);
    mqtt.subscribe(TOPIC_CMD);
    Serial.println("MQTT: đã kết nối broker");
  } else {
    Serial.printf("MQTT: chưa kết nối được (mã %d), thử lại sau 3 s\n", mqtt.state());
  }
}

// ---------- I2C ----------
void scanI2C() {
  Serial.println("Quét bus I2C...");
  int found = 0;
  for (uint8_t addr = 1; addr < 127; addr++) {
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0) {
      Serial.printf("  Tìm thấy 0x%02X", addr);
      if (addr == 0x23 || addr == 0x5C) Serial.print("  (BH1750)");
      if (addr == 0x76 || addr == 0x77) Serial.print("  (BMP280)");
      Serial.println();
      found++;
    }
  }
  if (found == 0) Serial.println("  Không thấy thiết bị nào -> kiểm tra hàng P21, P22 và ray phải");
}

uint8_t readChipID(uint8_t addr) {
  Wire.beginTransmission(addr);
  Wire.write(0xD0);
  if (Wire.endTransmission() != 0) return 0;
  Wire.requestFrom((int)addr, 1);
  return Wire.available() ? Wire.read() : 0;
}

bool setupBMP() {
  const uint8_t addrs[] = {0x76, 0x77};
  for (uint8_t a : addrs) {
    uint8_t id = readChipID(a);
    if (id && bmp.begin(a, id)) return true;
  }
  return false;
}

// ---------- Micro INMP441 ----------
bool setupMic() {
  i2s.setPins(I2S_SCK, I2S_WS, -1, I2S_SD);   // BCLK, WS, DOUT (không dùng), DIN
  // Nếu mức âm luôn = 0, đổi I2S_STD_SLOT_LEFT thành I2S_STD_SLOT_RIGHT
  return i2s.begin(I2S_MODE_STD, 16000, I2S_DATA_BIT_WIDTH_32BIT, I2S_SLOT_MODE_MONO, I2S_STD_SLOT_LEFT);
}

float readSoundLevel() {
  static int32_t buf[256];
  size_t bytesRead = i2s.readBytes((char *)buf, sizeof(buf));
  int n = bytesRead / sizeof(int32_t);
  if (n == 0) return 0;
  double mean = 0;
  for (int i = 0; i < n; i++) mean += (buf[i] >> 8);
  mean /= n;
  double sumSq = 0;
  for (int i = 0; i < n; i++) {
    double s = (buf[i] >> 8) - mean;
    sumSq += s * s;
  }
  return sqrt(sumSq / n);
}

// ---------- MQ2: đọc nhanh + so với mức nền ----------
float gasFast = 0, gasBase = 0;   // giá trị đã lọc và mức nền (mV)
bool gasReady = false, gasAlarm = false;

void updateGas() {
  static unsigned long last = 0;
  if (millis() - last < MQ2_SAMPLE_MS) return;
  last = millis();

  float mv = analogReadMilliVolts(PIN_MQ2);
  if (!gasReady) { gasFast = gasBase = mv; gasReady = true; return; }

  gasFast += 0.3f * (mv - gasFast);                 // bám theo giá trị mới trong khoảng 0.2 s
  if (millis() < MQ2_WARMUP_MS) { gasBase = gasFast; return; }   // đang khởi động: chỉ học nền
  if (!gasAlarm) gasBase += 0.002f * (gasFast - gasBase);        // nền trôi rất chậm (~25 s)

  float rise = gasFast - gasBase;
  if (!gasAlarm && rise > GAS_ON_MV) {
    gasAlarm = true;
    Serial.printf(">>> CÓ GAS! Tăng %.0f mV so với nền\n", rise);
    publishEvent("gas", "on");
    startAlarm(ALARM_GAS, "gas");     // gas luôn báo động, kể cả khi tắt an ninh
  } else if (gasAlarm && rise < GAS_OFF_MV) {
    gasAlarm = false;
    Serial.println(">>> Hết gas");
    publishEvent("gas", "off");
    if (alarmType == ALARM_GAS) stopAlarm("hết gas");
  }
}

// ---------- PIR: lọc báo nhầm ----------
unsigned long pirHighSince = 0, lastConfirmMs = 0;
bool motionConfirmed = false;
const char *motionReason = "";

void confirmMotion(unsigned long now, const char *why) {
  lastConfirmMs = now;
  if (!motionConfirmed) {
    motionConfirmed = true;
    motionReason = why;
    Serial.printf(">>> CÓ NGƯỜI (%s)\n", why);
    publishEvent("motion", why);
    if (armReady()) startAlarm(ALARM_INTRUSION, "có người");
    else beep(80);
  }
}

void updatePir() {
  unsigned long now = millis();
  if (now < PIR_WARMUP_MS) { pirHighSince = 0; return; }

  if (digitalRead(PIN_PIR) == HIGH) {
    if (pirHighSince == 0) pirHighSince = now;
    unsigned long dur = now - pirHighSince;
    if (dur >= PIR_GLITCH_MS) {
      if (dur >= PIR_LONG_MS)                                          confirmMotion(now, "HIGH kéo dài");
      else if (lastLoudMs && now - lastLoudMs <= PIR_FUSE_MS)          confirmMotion(now, "kèm tiếng động");
      else if (lastDoorOpenMs && now - lastDoorOpenMs <= PIR_FUSE_MS)  confirmMotion(now, "kèm cửa mở");
    }
  } else if (pirHighSince != 0) {
    unsigned long dur = now - pirHighSince;
    if (dur >= PIR_GLITCH_MS) Serial.printf("    (PIR vừa HIGH %.1f s)\n", dur / 1000.0);
    pirHighSince = 0;
  }

  if (motionConfirmed && now - lastConfirmMs > PIR_HOLD_MS) {
    motionConfirmed = false;
    Serial.println(">>> Hết chuyển động");
    publishEvent("motion", "end");
  }
}

// ---------- Gửi dữ liệu cảm biến ----------
void publishSensors(int doorOpen) {
  static unsigned long last = 0;
  if (!mqtt.connected() || millis() - last < PUBLISH_MS) return;
  last = millis();
  JsonDocument doc;
  if (bmpOK) {
    doc["temp"] = round(bmp.readTemperature() * 10) / 10.0;          // °C
    doc["pressure"] = round(bmp.readPressure() / 10.0) / 10.0;       // hPa
  }
  if (bh1750OK) doc["lux"] = round(lightMeter.readLightLevel() * 10) / 10.0;
  doc["gas_mv"] = (int)gasFast;
  doc["gas_rise"] = (int)(gasFast - gasBase);
  doc["gas_alarm"] = gasAlarm;
  doc["sound"] = (long)soundPeakPub;
  doc["door_open"] = doorOpen == 1;
  doc["motion"] = motionConfirmed;
  doc["armed"] = armed;
  doc["alarm"] = alarmName();
  doc["rssi"] = WiFi.RSSI();
  char buf[320];
  size_t n = serializeJson(doc, buf, sizeof(buf));
  mqtt.publish(TOPIC_SENSORS, (const uint8_t *)buf, n, false);
  soundPeakPub = 0;
}

// ---------- Chương trình chính ----------
void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("\n=== HỆ THỐNG AN NINH PHÒNG (bản 6) ===");

  pinMode(PIN_PIR, INPUT_PULLDOWN);   // dây OUT lỏng thì vẫn đọc LOW, không báo nhầm
  pinMode(PIN_DOOR, INPUT_PULLUP);
  pinMode(PIN_BUZZER, OUTPUT);
  digitalWrite(PIN_BUZZER, LOW); // Mặc định còi tắt khi khởi động
  beep(150);
  analogReadResolution(12);

  Wire.begin(PIN_SDA, PIN_SCL);
  scanI2C();

  bmpOK = setupBMP();
  Serial.println(bmpOK ? "BMP280 : OK" : "BMP280 : LỖI");
  bh1750OK = lightMeter.begin(BH1750::CONTINUOUS_HIGH_RES_MODE);
  Serial.println(bh1750OK ? "BH1750 : OK" : "BH1750 : LỖI");
  micOK = setupMic();
  Serial.println(micOK ? "INMP441: OK" : "INMP441: LỖI khởi tạo I2S");

  prefs.begin("anninh", false);
  armed = prefs.getBool("armed", false);   // lấy lại trạng thái an ninh trước khi mất điện
  armedAt = millis();
  Serial.println(armed ? "AN NINH: đang BẬT (có hiệu lực sau 10 s)" : "AN NINH: đang TẮT");

  connectWifi();
  mqtt.setServer(MQTT_HOST, MQTT_PORT);
  mqtt.setCallback(onMqttMessage);
  mqtt.setBufferSize(512);
  mqtt.setSocketTimeout(2);

  Serial.println("PIR bỏ qua 15 s đầu, MQ2 học mức nền trong 60 s đầu.\n");
}

void loop() {
  keepConnected();
  // Tránh micro nhận tiếng còi đang bật liên tục là tiếng xâm nhập.
  if (buzzerManualOn) lastBeepEnd = millis();
  updateSiren();
  if (micOK) {                       // mỗi lần đọc micro mất khoảng 16 ms
    float s = readSoundLevel();
    if (s > soundPeak) soundPeak = s;
    if (s > soundPeakPub) soundPeakPub = s;
    if (s > SOUND_CONFIRM && millis() - lastBeepEnd > 800) lastLoudMs = millis();
  }
  updateGas();

  int doorOpen = digitalRead(PIN_DOOR);   // 1 = cửa mở
  static int lastDoor = 0;
  if (doorOpen && !lastDoor) {
    lastDoorOpenMs = millis();
    Serial.println(">>> CỬA VỪA MỞ");
    publishEvent("door", "open");
    if (armReady()) startAlarm(ALARM_INTRUSION, "cửa mở");
    else beep(80);
  }
  if (!doorOpen && lastDoor) publishEvent("door", "closed");
  lastDoor = doorOpen;

  updatePir();
  publishSensors(doorOpen);

  static unsigned long last = 0;
  if (millis() - last < REPORT_MS) return;
  last = millis();

  unsigned long now = millis();
  Serial.println("--------------------------------");
  if (!armed)               Serial.println("An ninh  : TẮT");
  else if (!armReady())     Serial.printf("An ninh  : BẬT, còn %lu s để rời phòng\n", (ARM_DELAY_MS - (now - armedAt)) / 1000 + 1);
  else                      Serial.println("An ninh  : BẬT");
  if (alarmType != ALARM_NONE)
    Serial.printf("Báo động : %s (còn tối đa %lu s)\n", alarmType == ALARM_GAS ? "GAS" : "XÂM NHẬP",
                  (ALARM_MAX_MS - (now - alarmStart)) / 1000);
  if (now < PIR_WARMUP_MS)
    Serial.printf("PIR      : đang ổn định, còn %lu s\n", (PIR_WARMUP_MS - now) / 1000);
  else {
    if (pirHighSince) Serial.printf("PIR      : HIGH %.1f s | ", (now - pirHighSince) / 1000.0);
    else              Serial.printf("PIR      : low | ");
    if (motionConfirmed) Serial.printf("CÓ NGƯỜI (%s)\n", motionReason);
    else                 Serial.println("không có người");
  }
  Serial.printf("Cửa      : %s\n", doorOpen ? "MỞ" : "đóng");
  Serial.printf("MQ2      : %.0f mV (nền %.0f, %+.0f)%s\n", gasFast, gasBase, gasFast - gasBase,
                now < MQ2_WARMUP_MS ? "  đang học mức nền" : (gasAlarm ? "  CÓ GAS" : ""));
  if (bmpOK)
    Serial.printf("BMP280   : %.2f °C, %.2f hPa\n", bmp.readTemperature(), bmp.readPressure() / 100.0F);
  if (bh1750OK)
    Serial.printf("BH1750   : %.1f lux\n", lightMeter.readLightLevel());
  if (micOK)
    Serial.printf("INMP441  : mức âm %.0f%s\n", soundPeak,
                  soundPeak == 0 ? "  (=0: kiểm tra SD, L/R, hoặc đổi sang I2S_STD_SLOT_RIGHT)" : "");
  soundPeak = 0;
}
