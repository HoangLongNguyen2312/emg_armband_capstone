/*
 * ============================================================
 *  EMG BIONIC CONTROLLER - FINAL UNIVERSAL VERSION
 *
 *  Pipeline xử lý tín hiệu 3 tầng:
 *    RAW → [High-pass] → filteredEMG
 *              ↓
 *         [Asymmetric envelope] → envelope  (dùng để detect)
 *              ↓
 *         [Display smoother] → displayEnv  (chỉ dùng để vẽ OLED)
 *
 *  Calibration 3 phase:
 *    Phase 1: Đo REST (median 200 mẫu)
 *    Phase 2: Đo GỒNG CƠ (top 20% của 200 mẫu)
 *    Phase 3: TEST THẬT - gồng 1 nhịp để verify threshold
 *             → Nếu không detect được, tự hạ threshold và thử lại
 *             → Tối đa 3 lần hạ, mỗi lần -10%
 *
 *  Timer tối ưu gameplay:
 *    CONFIRM_COUNT = 3   (~40ms lag, đủ lọc spike 50Hz)
 *    HOLD_MS       = 60  (nhả nhanh, không kẹt lệnh)
 *    REFRACTORY_MS = 200 (chống double-click)
 * ============================================================
 */

#include <Wire.h>
#include <Adafruit_ADS1X15.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Preferences.h>
#include <WiFi.h>
#include <WebSocketsServer.h>
#include <ArduinoJson.h>

// ============================================================
//  PHẦN CỨNG
// ============================================================
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define BOOT_PIN 0

Adafruit_SSD1306  display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);
Adafruit_ADS1115  ads;
Preferences       prefs;
WebSocketsServer  wsServer = WebSocketsServer(81);

const char* SSID     = "iPhone (68)";
const char* PASSWORD = "12345678";
bool wsClientConnected = false;

// ============================================================
//  THAM SỐ BỘ LỌC
// ============================================================
const float ALPHA_HP      = 0.85f;
const float ALPHA_ATTACK  = 0.10f;
const float ALPHA_DECAY   = 0.25f;
const float ALPHA_DISPLAY = 0.06f;

// ============================================================
//  NGƯỠNG RUNTIME - thay đổi theo từng người sau calib
// ============================================================
float norm_threshold_on  = 0.40f;
float norm_threshold_off = 0.12f;

// ============================================================
//  TIMER TỐI ƯU GAMEPLAY
//
//  CONFIRM_COUNT = 3  : bắt lệnh ~40ms, đủ lọc spike 50Hz
//  HOLD_MS       = 60 : nhả cò nhanh, không kẹt lệnh
//  REFRACTORY_MS = 200: chống double-click do run cơ
// ============================================================
const int           CONFIRM_COUNT  = 3;
const unsigned long HOLD_MS        = 60;
const unsigned long REFRACTORY_MS  = 200;

// ============================================================
//  BIẾN TÍN HIỆU
// ============================================================
float prevRawmV     = 0.0f;
float filteredEMG   = 0.0f;
float envelope      = 0.0f;
float displayEnv    = 0.0f;
float normalizedEMG = 0.0f;

// ============================================================
//  CALIBRATION
// ============================================================
float calib_rest  = 0.0f;
float calib_max   = 500.0f;
float calib_range = 500.0f;
float threshold_ON  = 300.0f;
float threshold_OFF = 100.0f;

// ============================================================
//  LOGIC
// ============================================================
int           binaryOutput = 0;
int           confirmOn    = 0;
unsigned long lastOnTime   = 0;
unsigned long lastOffTime  = 0;

// ============================================================
//  TIỀN KHAI BÁO
// ============================================================
void runCalibration();
void processEMG();
void updateDetection();
void updateOLED();
void onWsEvent(uint8_t num, WStype_t type, uint8_t* payload, size_t length);

// ============================================================
//  XỬ LÝ TÍN HIỆU - 3 TẦNG
// ============================================================
void processEMG() {
  int16_t raw       = ads.readADC_SingleEnded(0);
  float   currentmV = raw * 0.125f;

  filteredEMG = ALPHA_HP * (filteredEMG + currentmV - prevRawmV);
  prevRawmV   = currentmV;

  float absSignal = fabsf(filteredEMG);
  float alpha     = (absSignal > envelope) ? ALPHA_ATTACK : ALPHA_DECAY;
  envelope        = alpha * absSignal + (1.0f - alpha) * envelope;

  displayEnv = ALPHA_DISPLAY * envelope + (1.0f - ALPHA_DISPLAY) * displayEnv;

  if (calib_range > 10.0f) {
    normalizedEMG = (envelope - calib_rest) / calib_range;
    normalizedEMG = constrain(normalizedEMG, -0.2f, 2.0f);
  } else {
    normalizedEMG = envelope / 500.0f;
  }
}

// ============================================================
//  DETECTION
// ============================================================
void updateDetection() {
  unsigned long now = millis();

  if (binaryOutput == 0) {
    if (now - lastOffTime < REFRACTORY_MS) { confirmOn = 0; return; }
    if (normalizedEMG > norm_threshold_on) {
      if (++confirmOn >= CONFIRM_COUNT) {
        binaryOutput = 1;
        confirmOn    = 0;
        lastOnTime   = now;
      }
    } else {
      confirmOn = 0;
    }
  } else {
    if (normalizedEMG >= norm_threshold_off) lastOnTime = now;
    if (normalizedEMG < norm_threshold_off && (now - lastOnTime > HOLD_MS)) {
      binaryOutput = 0;
      lastOffTime  = now;
    }
  }
}

// ============================================================
//  AUTO-ADJUST THRESHOLD theo range cá nhân
//
//  range >= 150 : tín hiệu mạnh  → ON=50%, OFF=15%
//  range 80-149 : tín hiệu TB    → ON=40%, OFF=12%
//  range 30-79  : tín hiệu yếu   → ON=32%, OFF=10%
// ============================================================
void autoAdjustThreshold(float range) {
  if (range >= 150.0f) {
    norm_threshold_on  = 0.50f;
    norm_threshold_off = 0.15f;
    Serial.println("[CAL] STRONG -> ON=50% OFF=15%");
  } else if (range >= 80.0f) {
    norm_threshold_on  = 0.40f;
    norm_threshold_off = 0.12f;
    Serial.println("[CAL] MEDIUM -> ON=40% OFF=12%");
  } else {
    norm_threshold_on  = 0.32f;
    norm_threshold_off = 0.10f;
    Serial.println("[CAL] WEAK -> ON=32% OFF=10%");
  }
}

// ============================================================
//  CALIBRATION - 3 PHASE
//
//  Phase 1: Đo REST (median, bỏ noise check)
//  Phase 2: Đo GỒNG CƠ (top 20%)
//  Phase 3: TEST THẬT 1 nhịp gồng-thả
//           → Nếu detect OK: lưu và xong
//           → Nếu không detect: hạ ON xuống 10%, thử lại
//           → Tối đa 3 lần hạ (tổng có thể xuống tới ON-30%)
// ============================================================
void runCalibration() {
  filteredEMG  = 0.0f;
  prevRawmV    = 0.0f;
  envelope     = 0.0f;
  displayEnv   = 0.0f;
  binaryOutput = 0;
  confirmOn    = 0;

  // ---- WARM-UP 300 MẪU ----
  display.clearDisplay();
  display.setTextSize(1); display.setTextColor(WHITE);
  display.setCursor(0, 0); display.println(">> CALIBRATION <<");
  display.println("Warm-up bo loc...");
  display.println("Giu yen tay!");
  display.display();
  for (int i = 0; i < 300; i++) { processEMG(); delay(8); }
  delay(500);

  // Chờ envelope thực sự settle về baseline thấp
  // Vấn đề: sau warm-up envelope có thể còn cao nếu tay có chút chuyển động
  // Fix: đợi đến khi envelope < 80 hoặc tối đa 2 giây thêm
  {
    unsigned long settleStart = millis();
    display.clearDisplay(); display.setCursor(0, 0);
    display.println(">> CALIBRATION <<");
    display.println("Cho bo loc on dinh");
    display.println("Giu yen tay...");
    display.display();
    while (envelope > 80.0f && (millis() - settleStart) < 2000) {
      processEMG(); delay(8);
    }
    // Thêm 30 mẫu buffer sau khi settle
    for (int i = 0; i < 30; i++) { processEMG(); delay(8); }
  }

  // ---- PHASE 1: NGHỈ ----
  display.clearDisplay(); display.setCursor(0, 0);
  display.println(">> CALIBRATION <<");
  display.println("1/3: THA LONG TAY");
  display.println("   Buong xuong!");
  display.println("   (3 giay)");
  display.display();
  for (int i = 0; i < 60; i++) { processEMG(); delay(15); }

  const int N = 200;
  float restBuf[N];
  for (int i = 0; i < N; i++) {
    processEMG();
    restBuf[i] = envelope;
    display.drawRect(1, 55, 126, 7, WHITE);
    display.fillRect(1, 55, i * 126 / N, 7, WHITE);
    display.display();
    delay(15);
  }

  // REST = MEDIAN
  float sortBuf[N];
  memcpy(sortBuf, restBuf, sizeof(restBuf));
  for (int i = 0; i < N - 1; i++)
    for (int j = 0; j < N - i - 1; j++)
      if (sortBuf[j] > sortBuf[j + 1]) {
        float tmp = sortBuf[j]; sortBuf[j] = sortBuf[j + 1]; sortBuf[j + 1] = tmp;
      }
  float vRest = (sortBuf[N / 2 - 1] + sortBuf[N / 2]) / 2.0f;

  // ---- PHASE 2: GỒNG CƠ ----
  display.clearDisplay(); display.setCursor(0, 0);
  display.println(">> CALIBRATION <<");
  display.println("2/3: GONG CO");
  display.println("   Vua suc, giu deu");
  display.println("   (3 giay)");
  display.display();

  float buf[N];
  for (int i = 0; i < N; i++) {
    processEMG();
    buf[i] = envelope;
    display.drawRect(1, 55, 126, 7, WHITE);
    display.fillRect(1, 55, i * 126 / N, 7, WHITE);
    display.display();
    delay(15);
  }

  // Top 20%
  int   topN   = N / 5;
  float sumTop = 0;
  bool  used[N]; memset(used, 0, sizeof(used));
  for (int t = 0; t < topN; t++) {
    float mv = -1; int mi = 0;
    for (int j = 0; j < N; j++)
      if (!used[j] && buf[j] > mv) { mv = buf[j]; mi = j; }
    used[mi] = true; sumTop += mv;
  }
  float vMax  = sumTop / topN;
  float range = vMax - vRest;

  // Kiểm tra range tối thiểu
  display.clearDisplay(); display.setCursor(0, 0);
  display.println(">> CALIBRATION <<");
  if (range < 30.0f) {
    display.println("!! LOI: Range qua nho");
    display.println("Gong manh hon!");
    display.print("Range: "); display.println(range, 0);
    display.display();
    Serial.print("[CAL] FAIL range="); Serial.println(range, 1);
    delay(3000); return;
  }

  // Auto-adjust threshold theo range
  autoAdjustThreshold(range);

  // Tính threshold tuyệt đối
  float proposed_ON  = vRest + norm_threshold_on  * range;
  float proposed_OFF = vRest + norm_threshold_off * range;

  // Enforce hysteresis tối thiểu 20% range
  // Vấn đề: nếu calib_range sai (quá lớn) thì ON-OFF rất gần nhau → chập chờn
  // Đảm bảo khoảng cách ON-OFF >= 20% range luôn luôn
  float min_hysteresis = range * 0.20f;
  if ((proposed_ON - proposed_OFF) < min_hysteresis) {
    proposed_OFF = proposed_ON - min_hysteresis;
    norm_threshold_off = (proposed_OFF - vRest) / range;
    norm_threshold_off = max(norm_threshold_off, 0.07f);
    proposed_OFF = vRest + norm_threshold_off * range;
    Serial.print("[CAL] Hysteresis enforced: OFF adjusted to ");
    Serial.println(proposed_OFF, 1);
  }

  // Sanity check
  if (proposed_ON < 50.0f) {
    display.println("!! LOI: Nguong ON < 50");
    display.println("Tha ky hon, calib lai!");
    display.print("th_ON="); display.println(proposed_ON, 0);
    display.display();
    Serial.print("[CAL] FAIL sanity th_ON="); Serial.println(proposed_ON, 1);
    delay(3000); return;
  }

  // Ghi tạm vào biến để phase 3 dùng
  calib_rest    = vRest;
  calib_max     = vMax;
  calib_range   = range;
  threshold_ON  = proposed_ON;
  threshold_OFF = proposed_OFF;

  // ---- PHASE 3: TEST THẬT - tự verify & tự hạ threshold ----
  // Reset detection state trước khi test
  binaryOutput = 0;
  confirmOn    = 0;
  lastOnTime   = 0;
  lastOffTime  = 0;

  bool detectOK = false;
  int  attempts = 0;

  while (!detectOK && attempts < 3) {
    // Tính lại threshold với norm hiện tại
    threshold_ON  = calib_rest + norm_threshold_on  * calib_range;
    threshold_OFF = calib_rest + norm_threshold_off * calib_range;

    display.clearDisplay(); display.setCursor(0, 0);
    display.println(">> CALIBRATION <<");
    display.print("3/3: TEST - lan "); display.println(attempts + 1);
    display.println("   GONG CO 1 NHIP");
    display.println("   roi THA RA!");
    display.print("   ON=");
    display.print((int)(norm_threshold_on * 100));
    display.println("%");
    display.display();

    // Đếm ngược 2 giây cho người chuẩn bị
    for (int c = 2; c > 0; c--) {
      display.setCursor(110, 46);
      display.print(c); display.print("s");
      display.display();
      delay(1000);
    }

    // Đo 3 giây để bắt 1 nhịp gồng-thả
    unsigned long testStart = millis();
    int peakDetected  = 0;  // đã detect ON
    int relayDetected = 0;  // sau ON đã về OFF

    while (millis() - testStart < 3000) {
      processEMG();

      // Hiển thị thanh realtime trong lúc test
      float dispNorm = (calib_range > 10.0f)
        ? (displayEnv - calib_rest) / calib_range : displayEnv / 500.0f;
      dispNorm = constrain(dispNorm, 0.0f, 1.2f);
      int barW = constrain((int)(dispNorm * 126.0f), 0, 126);
      display.drawRect(1, 55, 126, 7, WHITE);
      display.fillRect(1, 55, barW, 7, WHITE);
      // Vạch ON
      int thOnBar = constrain((int)(norm_threshold_on * 126.0f), 1, 125);
      display.drawLine(thOnBar, 55, thOnBar, 62,
        (barW > thOnBar) ? BLACK : WHITE);
      display.display();

      // Logic detect đơn giản: tìm normalizedEMG vượt ON rồi xuống OFF
      float norm = (calib_range > 10.0f)
        ? (envelope - calib_rest) / calib_range
        : envelope / 500.0f;

      if (!peakDetected && norm > norm_threshold_on) {
        peakDetected = 1;
        Serial.print("[CAL] Phase3: peak detected norm=");
        Serial.println(norm, 2);
      }
      if (peakDetected && !relayDetected && norm < norm_threshold_off) {
        relayDetected = 1;
        Serial.println("[CAL] Phase3: relay detected -> PASS");
      }

      delay(8);
    }

    if (peakDetected && relayDetected) {
      detectOK = true;
    } else {
      // Không detect được → hạ ON xuống 10%, OFF xuống tỉ lệ
      attempts++;
      Serial.print("[CAL] Phase3 FAIL attempt="); Serial.print(attempts);
      Serial.print(" norm_ON="); Serial.print(norm_threshold_on, 2);
      Serial.println(" -> lowering...");

      norm_threshold_on  -= 0.08f;
      norm_threshold_off -= 0.02f;
      // Giới hạn không hạ quá thấp
      norm_threshold_on  = max(norm_threshold_on,  0.20f);
      norm_threshold_off = max(norm_threshold_off, 0.07f);

      if (attempts < 3) {
        display.clearDisplay(); display.setCursor(0, 0);
        display.println(">> CALIBRATION <<");
        display.println("Chua detect duoc!");
        display.print("Ha nguong: ON=");
        display.print((int)(norm_threshold_on * 100)); display.println("%");
        display.println("Thu lai...");
        display.display();
        delay(1500);
      }
    }
  }

  // ---- KẾT QUẢ ----
  display.clearDisplay(); display.setCursor(0, 0);
  display.println(">> CALIBRATION <<");

  if (!detectOK) {
    // Sau 3 lần vẫn không detect → cảnh báo nhưng vẫn lưu với threshold thấp nhất
    display.println("!! CANH BAO:");
    display.println("Tin hieu yeu/nhieu.");
    display.println("Da luu, thu dung thu!");
    Serial.println("[CAL] Phase3: all attempts failed, saving low threshold");
  } else {
    display.println(">> CALIB DONE! <<");
  }

  // Tính lại threshold_ON/OFF cuối cùng
  threshold_ON  = calib_rest + norm_threshold_on  * calib_range;
  threshold_OFF = calib_rest + norm_threshold_off * calib_range;

  display.print("Rest:"); display.print(calib_rest, 0);
  display.print(" Range:"); display.println(calib_range, 0);
  display.print("ON:"); display.print(threshold_ON, 0);
  display.print("("); display.print((int)(norm_threshold_on * 100)); display.print("%)");
  display.print(" OFF:"); display.println(threshold_OFF, 0);
  display.print("Signal: ");
  if (calib_range >= 150.0f)      display.println("STRONG");
  else if (calib_range >= 80.0f)  display.println("MEDIUM");
  else                            display.println("WEAK");
  display.display();

  // Lưu vào Preferences
  prefs.putFloat("c_rest",   calib_rest);
  prefs.putFloat("c_max",    calib_max);
  prefs.putFloat("c_range",  calib_range);
  prefs.putFloat("th_on",    threshold_ON);
  prefs.putFloat("th_off",   threshold_OFF);
  prefs.putFloat("norm_on",  norm_threshold_on);
  prefs.putFloat("norm_off", norm_threshold_off);

  Serial.println("[CAL] === DONE ===");
  Serial.print("[CAL] rest=");   Serial.print(calib_rest, 1);
  Serial.print(" range=");       Serial.print(calib_range, 1);
  Serial.print(" th_ON=");       Serial.print(threshold_ON, 1);
  Serial.print("("); Serial.print((int)(norm_threshold_on * 100)); Serial.print("%)");
  Serial.print(" th_OFF=");      Serial.print(threshold_OFF, 1);
  Serial.print("("); Serial.print((int)(norm_threshold_off * 100)); Serial.println("%)");

  // Reset detection sau calib
  binaryOutput = 0;
  confirmOn    = 0;
  lastOnTime   = 0;
  lastOffTime  = millis(); // refractory ngay sau calib
  delay(3000);
}

// ============================================================
//  OLED
// ============================================================
void updateOLED() {
  display.clearDisplay();
  display.setTextSize(1);

  int pct = constrain((int)(normalizedEMG * 100.0f), -9, 199);
  display.setCursor(0, 0);
  display.print("EMG:"); display.print(pct);
  display.print("% ON:"); display.print((int)(norm_threshold_on  * 100));
  display.print("% OF:"); display.print((int)(norm_threshold_off * 100));
  display.println("%");

  float dispNorm = (calib_range > 10.0f)
    ? (displayEnv - calib_rest) / calib_range
    : displayEnv / 500.0f;
  dispNorm = constrain(dispNorm, 0.0f, 1.2f);

  display.drawRect(0, 12, 128, 8, WHITE);
  int barW = constrain((int)(dispNorm * 128.0f), 0, 128);
  display.fillRect(0, 12, barW, 8, WHITE);

  int thOnBar  = constrain((int)(norm_threshold_on  * 128.0f), 1, 127);
  int thOffBar = constrain((int)(norm_threshold_off * 128.0f), 1, 127);
  display.drawLine(thOnBar, 12, thOnBar, 19, (barW > thOnBar)  ? BLACK : WHITE);
  for (int y = 12; y <= 19; y += 2)
    display.drawPixel(thOffBar, y, (barW > thOffBar) ? BLACK : WHITE);

  display.setCursor(0, 26); display.setTextSize(2);
  display.print(binaryOutput == 1 ? ">> JUMP <<" : "  REST    ");

  display.setTextSize(1); display.setCursor(0, 55);
  if (calib_range < 10.0f)       display.print("!! Bam BOOT calibrate");
  else if (WiFi.status() == WL_CONNECTED) {
    display.print("IP:"); display.print(WiFi.localIP().toString());
  } else                         display.print("WiFi: Disconnected");

  display.display();
}

// ============================================================
//  WEBSOCKET
// ============================================================
void onWsEvent(uint8_t num, WStype_t type, uint8_t* payload, size_t length) {
  if      (type == WStype_CONNECTED)    { wsClientConnected = true;  Serial.println("[WS] Connected!"); }
  else if (type == WStype_DISCONNECTED) { wsClientConnected = false; Serial.println("[WS] Disconnected."); }
  else if (type == WStype_TEXT) {
    if (String((char*)payload).indexOf("cal_start") >= 0) runCalibration();
  }
}

// ============================================================
//  SETUP
// ============================================================
void setup() {
  Serial.begin(115200);
  pinMode(BOOT_PIN, INPUT_PULLUP);
  Wire.begin(27, 12);

  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { while (1); }
  display.clearDisplay(); display.setTextColor(WHITE); display.setTextSize(1);

  if (!ads.begin()) {
    display.println("ADS1115 ERROR!"); display.display(); while (1);
  }
  ads.setGain(GAIN_ONE);
  ads.setDataRate(RATE_ADS1115_128SPS);

  prefs.begin("emg_data", false);
  calib_rest  = prefs.getFloat("c_rest",  0.0f);
  calib_max   = prefs.getFloat("c_max",   0.0f);
  calib_range = prefs.getFloat("c_range", 0.0f);
  threshold_ON  = prefs.getFloat("th_on",  300.0f);
  threshold_OFF = prefs.getFloat("th_off", 100.0f);
  norm_threshold_on  = prefs.getFloat("norm_on",  0.40f);
  norm_threshold_off = prefs.getFloat("norm_off", 0.12f);

  Serial.print("[INIT] rest=");  Serial.print(calib_rest, 1);
  Serial.print(" range=");       Serial.print(calib_range, 1);
  Serial.print(" norm_ON=");     Serial.print((int)(norm_threshold_on * 100));
  Serial.print("% norm_OFF=");   Serial.print((int)(norm_threshold_off * 100));
  Serial.println("%");
  if (calib_range < 10.0f) Serial.println("[WARN] Chua calibrate! Bam BOOT.");

  display.setCursor(0, 0);
  display.println("Connecting WiFi...");
  display.println(SSID); display.display();

  WiFi.begin(SSID, PASSWORD);
  int att = 0;
  while (WiFi.status() != WL_CONNECTED && att < 20) {
    delay(500); att++; display.print("."); display.display();
  }

  display.clearDisplay(); display.setCursor(0, 0);
  if (WiFi.status() == WL_CONNECTED) {
    display.println("WiFi OK!");
    display.print("IP: "); display.println(WiFi.localIP().toString());
    Serial.print("[WIFI] IP: "); Serial.println(WiFi.localIP());
  } else {
    display.println("WiFi FAILED");
    Serial.println("[WIFI] Failed.");
  }
  display.display(); delay(1500);

  wsServer.begin();
  wsServer.onEvent(onWsEvent);
  Serial.println("[WS] Port 81 ready");
  Serial.println("envelope,disp_env,norm_pct,threshold_ON,threshold_OFF,binary");
}

// ============================================================
//  LOOP
// ============================================================
void loop() {
  wsServer.loop();

  static bool bootWasHigh = true;
  bool bootNow = (digitalRead(BOOT_PIN) == LOW);
  if (bootNow && bootWasHigh) {
    delay(50);
    if (digitalRead(BOOT_PIN) == LOW) runCalibration();
  }
  bootWasHigh = !bootNow;

  static unsigned long lastSample = 0;
  unsigned long now = millis();
  if (now - lastSample >= 8) {
    lastSample = now;

    processEMG();
    updateDetection();

    Serial.print("envelope:");      Serial.print((int)envelope);
    Serial.print(",disp_env:");     Serial.print((int)displayEnv);
    Serial.print(",norm_pct:");     Serial.print((int)(normalizedEMG * 100.0f));
    Serial.print(",threshold_ON:"); Serial.print((int)threshold_ON);
    Serial.print(",threshold_OFF:");Serial.print((int)threshold_OFF);
    Serial.print(",binary:");       Serial.println(binaryOutput * 200);

    if (wsClientConnected) {
      JsonDocument doc;
      doc["env"]      = (int)displayEnv;
      doc["norm"]     = (int)(normalizedEMG * 100.0f);
      doc["state"]    = binaryOutput;
      doc["on"]       = (int)threshold_ON;
      doc["off"]      = (int)threshold_OFF;
      doc["calib"]    = (calib_range > 10.0f);
      doc["norm_on"]  = (int)(norm_threshold_on  * 100);
      doc["norm_off"] = (int)(norm_threshold_off * 100);
      String json;
      serializeJson(doc, json);
      wsServer.broadcastTXT(json);
    }
  }

  static unsigned long lastOLED = 0;
  if (millis() - lastOLED >= 80) {
    lastOLED = millis();
    updateOLED();
  }
}
