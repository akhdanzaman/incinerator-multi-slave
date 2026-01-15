/*
 * MCU-HMI (Nextion Controller) ESP32  |  RS485 Multi-Slave (Simple ASCII)
 * --------------------------------------------------------------------
 * Kamu minta versi "lengkap" (objek HMI + callback + parsing) tapi komunikasi ke jaringan
 * pakai RS485 (MAX485) dan siap multi-slave.
 *
 * Catatan penting (biar nggak jadi ritual debug tengah malam):
 * - Nextion tetap di Serial2 (nexSerial)
 * - RS485 network di Serial1 (CtrlBus) + pin DE/RE
 * - Protokol dibuat super-sederhana: satu baris ASCII diakhiri '\n'
 *
 *   Format frame RX dari slave ke master:
 *     N<id>:<payload>\n
 *     contoh:
 *       N01:DATA:123.4,5.6,2,10.0,100.0
 *       N02:TEMP:456.7
 *       N03:WEIGHT:12.34
 *       N04:VOL:200.0,1500.0
 *       N01:PROG:50
 *       N01:REM_TIME:120
 *       N01:PREHEAT
 *       N01:LOG:INFO,LIDAR,Background OK
 *       N01:BATCH_SUM:1,10.0,5.0,0.2,900.0,120,OK
 *       N01:FUEL:70,12.5
 *
 *   Format TX dari master ke slave:
 *     TO<id>:<payload>\n
 *     contoh:
 *       TO01:CMD_REQ_SYNC
 *       TO01:CMD_EMERGENCY_STOP
 *       TO01:SET_TE:1200
 *       TO01:M_SPD:0:8000
 *       TO02:REQ_TEMP
 *
 * - Semua slave "nanti" kamu bikin cukup menuruti format Nxx:... untuk mengirim,
 *   dan mendengarkan TOxx:... untuk menerima.
 *
 * Disclaimer: Ini dibuat supaya tetap sederhana. Tidak ada CRC, tidak ada retry,
 * tidak ada collision avoidance. Untuk 1 master banyak slave dengan polling, aman.
 */

#include <Arduino.h>
#include <SPI.h>
#include <SD.h>
#include <Preferences.h>
#include <math.h>

// ==============================
// NEXTION
// ==============================
#define nexSerial Serial2
#include <Nextion.h>

// ==============================
// RS485 (MAX485) NETWORK
// ==============================
static const int RS485_RX    = 16;   // RO -> RX
static const int RS485_TX    = 17;   // DI -> TX
static const int RS485_DE_RE = 4;    // DE & /RE digabung (HIGH=TX, LOW=RX)

HardwareSerial CtrlBus(1);           // UART1 untuk RS485

static inline void rs485SetTx(bool en) {
  digitalWrite(RS485_DE_RE, en ? HIGH : LOW);
}

static void rs485SendRawLine(const String &line) {
  rs485SetTx(true);
  delayMicroseconds(50);
  CtrlBus.print(line);
  CtrlBus.print('\n');
  CtrlBus.flush();
  delayMicroseconds(50);
  rs485SetTx(false);
}

static inline void sendToNode(uint8_t nodeId, const String &payload) {
  char hdr[8];
  snprintf(hdr, sizeof(hdr), "TO%02u:", (unsigned)nodeId);
  rs485SendRawLine(String(hdr) + payload);
}

// ==============================
// NODE IDS (sesuaikan)
// ==============================
static const uint8_t NODE_CTRL  = 1; // authoritative controller
static const uint8_t NODE_TEMP  = 2; // slave temp
static const uint8_t NODE_WGHT  = 3; // slave weight
static const uint8_t NODE_LIDAR = 4; // slave lidar

// ==============================
// (Placeholder) LIDAR flags
// ==============================
#ifndef USE_LIDAR
#define USE_LIDAR 0
#endif
#if USE_LIDAR
// kalau kamu punya implementasi lidar di master, taruh include di sini
#endif

// ==============================
// TYPES
// ==============================
struct LidarPoint {
  float distanceMm;
  float angleDeg;
  uint8_t quality;
  bool startBit;
};

enum SystemState {
  IDLE = 0,
  WEIGHING,
  OPENING_MAIN_DOOR,
  DUMPING_IN,
  DUMPING_OUT,
  CLOSING_MAIN_DOOR,
  OPENING_BURN_DOOR,
  BURNER_IN,
  IGNITING,
  BURNING,
  BURNER_OUT,
  CLOSING_BURN_DOOR,
  FAULT
};

int currentPageId = -1;

// ================== VARIABEL GLOBAL (Mirror dari CTRL untuk Display) ==================
float currentTempC = 0.0;
float currentWeightKg = 0.0;
float currentVol = 0.0;
float solarVolumeUsed_L = 0.0;
float totalConsumedKg = 0.0;
float totalVolume = 0.0;
unsigned long totalBatchCount = 0;
float minBatchWeight = 5.0;
uint32_t burnerActiveTimeSec = 30;
float tempSetpoint = 1200.0;
float volSetpoint = 20000.0;

uint8_t currentFuelPct = 0;
float currentFuelLiter = 0;
int lastFuelPctSent = -1;

SystemState currentState = IDLE;
SystemState lastState = IDLE;

// Variabel UI Helper
float lastTempSent = -999;
float lastVolSent = -999;
float lastTotalVolSent = -999;
float lastTempManSent = -999;
float lastWeightSent = -999;
float lastWeightManSent = -999;
float lastFuelSent = -999;
float lastTotalKgSent = -999;
unsigned long lastBatchSent = 999999;
long lastProgValue = 0;

// Variabel Setting UI
int selectedMotorIdx = 0;
uint8_t currentPage = 0;
bool motorSettingsDirty = false;
int manualMotorIdx = 0;

// Blower UI State
int blowLevel[3] = { 0, 0, 0 };

// Remaining time
long remainingTimeSec = -1;
bool remainingIsPreheat = false;
unsigned long remainingLastRxMs = 0;

long lastRemainingTimeSent = -999999;
bool lastPreheatSent = false;

// Struktur Konfigurasi Motor
struct MotorConfig { long speed; long accel; long steps; };
MotorConfig motorConfigs[6] = {
  { 1000, 200, 2000 }, { 1000, 200, 5000 }, { 1000, 200, 3000 },
  { 800, 200, 1000 }, { 800, 200, 1000 }, { 800, 200, 1000 }
};

static const uint32_t MOTOR_MAX_SPD[6] = { 10000, 12000, 15000, 8000, 7000, 14000 };
static const uint32_t MOTOR_MIN_SPD[6] = { 300, 300, 300, 300, 300, 300 };

static inline uint8_t clampPct100(long v) { if (v < 1) return 1; if (v > 100) return 100; return (uint8_t)v; }
static inline uint8_t clampPctMan(long v) { return clampPct100(v); }
static inline uint8_t stepRealToPct(long stepsReal) { long pct = stepsReal / 100L; return clampPct100(pct); }
static inline uint8_t clampPct(int v) { if (v < 1) return 1; if (v > 100) return 100; return (uint8_t)v; }

static inline uint32_t pctToSpeed(uint8_t motorId, uint8_t pct) {
  pct = clampPct(pct);
  uint32_t maxS = MOTOR_MAX_SPD[motorId];
  uint32_t minS = MOTOR_MIN_SPD[motorId];
  uint32_t span = (maxS > minS) ? (maxS - minS) : 0;
  return minS + (span * pct) / 100;
}

static inline uint8_t speedToPct(uint8_t motorId, uint32_t speed) {
  uint32_t maxS = MOTOR_MAX_SPD[motorId];
  uint32_t minS = MOTOR_MIN_SPD[motorId];
  if (maxS <= minS) return 100;
  if (speed <= minS) return 1;
  if (speed >= maxS) return 100;
  uint32_t span = maxS - minS;
  uint32_t pct = ((speed - minS) * 100UL) / span;
  return clampPct((int)pct);
}

// ================== NEXTION OBJECTS ==================
NexPage page0 = NexPage(0, 0, "page0");
NexPage page1 = NexPage(1, 0, "page1");
NexPage page2 = NexPage(2, 0, "page2");

// --- PAGE 0: DASHBOARD ---
NexText tStat = NexText(0, 2, "tStat");
NexText tTemp = NexText(0, 3, "tTemp");
NexText tWeight = NexText(0, 4, "tWeight");
NexText tTime = NexText(0, 5, "tTime");
NexText tFuel = NexText(0, 6, "tFuel");
NexSlider hFuel = NexSlider(0, 17, "hFuel");
NexText tBatch = NexText(0, 7, "tBatch");
NexText tTotWeight = NexText(0, 8, "tTotWeight");
NexDSButton btAuto = NexDSButton(0, 9, "btAuto");
NexButton bStop = NexButton(0, 13, "bStop");
NexProgressBar jStat = NexProgressBar(0, 16, "jStat");
NexButton b1 = NexButton(0, 15, "b1");
NexText tWeightCal = NexText(0, 18, "tWeightCal");
NexButton bsettingStop = NexButton(2, 13, "bsettingStop");
NexText tTempSetDash = NexText(0, 11, "t8");
NexText tVolume = NexText(0, 12, "t9");
NexText ttotVolume = NexText(0, 10, "tVolume");

// --- PAGE 1: KONTROL MANUAL ---
NexDSButton btManDir = NexDSButton(1, 2, "btManDir");
NexButton bManDoor = NexButton(1, 4, "bManDoor");
NexButton bManBDoor = NexButton(1, 5, "bManBDoor");
NexButton bManBurn = NexButton(1, 6, "bManBurn");
NexButton bManPush = NexButton(1, 20, "bManFeed");
NexButton bManAsh = NexButton(1, 7, "bManAsh");
NexDSButton bManMainC = NexDSButton(1, 18, "bManCon");
NexSlider hManSpeed = NexSlider(1, 8, "hManSpeed");
NexSlider hManStep = NexSlider(1, 9, "hManStep");
NexButton bman1 = NexButton(1, 21, "b4");
NexButton bman2 = NexButton(1, 22, "b0");
NexButton bman3 = NexButton(1, 23, "b4");
NexButton bman4 = NexButton(1, 24, "b0");

NexDSButton btManBurn = NexDSButton(1, 19, "btManBurn");
NexDSButton btManIgn = NexDSButton(1, 3, "btManIgn");
NexDSButton btManFeed = NexDSButton(1, 99, "btManFeed");
NexDSButton btManWeigh = NexDSButton(1, 98, "btManWeigh");

NexText tManTemp = NexText(1, 11, "tManTemp");
NexText tManWeight = NexText(1, 12, "tManWeight");
NexButton bManStop = NexButton(1, 10, "b5");
NexButton b0 = NexButton(1, 15, "b0");

// --- PAGE 2: PENGATURAN ---
NexText nTempSet = NexText(2, 21, "nTempSet");
NexText nTimeSet = NexText(2, 20, "nTimeSet");
NexText nWeightSet = NexText(2, 19, "nWeightSet");
NexText nVolSet = NexText(2, 99, "nVOlLLWeightSet");

NexButton bTempUp = NexButton(2, 8, "bTempUp");
NexButton bTempDown = NexButton(2, 9, "bTempDown");
NexButton bTimeUp = NexButton(2, 7, "bTimeUp");
NexButton bTimeDown = NexButton(2, 6, "bTimeDown");
NexButton bWeightUp = NexButton(2, 5, "bWeightUp");
NexButton bWeightDown = NexButton(2, 4, "bWeightDown");

NexDSButton bSelDoor = NexDSButton(2, 27, "bSelDoor");
NexDSButton bSelBurn = NexDSButton(2, 29, "bSelBurn");
NexDSButton bSelFeed = NexDSButton(2, 28, "bSelFeed");
NexDSButton bSelBDoor = NexDSButton(2, 30, "bSelBDoor");
NexDSButton bSelAsh = NexDSButton(2, 33, "bSelAsh");
NexDSButton bSelMainC = NexDSButton(2, 32, "bSelMai");

NexSlider hSpeed = NexSlider(2, 11, "hSpeed");
NexSlider hStep = NexSlider(2, 10, "hStep");
NexButton bSpeedUp = NexButton(2, 38, "b1");
NexButton bSpeedDown = NexButton(2, 40, "b2");
NexButton bStepUp = NexButton(2, 37, "b4");
NexButton bStepDown = NexButton(2, 39, "b0");

NexText tSpeed = NexText(2, 22, "tSpeed");
NexText tStep = NexText(2, 23, "tStep");
NexNumber nSpeed = NexNumber(2, 42, "nSpeed");
NexNumber nStep = NexNumber(2, 41, "nStep");

NexButton bBlow1 = NexButton(2, 17, "bBlow1");
NexButton bBlow2 = NexButton(2, 16, "bBlow2");

NexText tBlow1 = NexText(2, 25, "tBlow1");
NexText tBlow2 = NexText(2, 26, "tBlow2");

NexButton bSave = NexButton(2, 15, "bSave");
NexButton bZero = NexButton(2, 12, "bZero");

NexButton bSet = NexButton(2, 31, "bSet");
NexButton bWall = NexButton(2, 35, "bWall");
NexButton bSpeed = NexButton(2, 36, "bSpeed");

// page3 log
NexPage page3 = NexPage(3, 0, "page3");
NexScrolltext stLog = NexScrolltext(3, 5, "slt0");
NexScrolltext stBatch = NexScrolltext(3, 6, "slt1");

NexTouch *nex_listen_list[] = {
  &page0, &page1, &page2, &page3,
  &btAuto, &bStop, &b1,
  &nWeightSet, &nTimeSet, &hFuel,
  &bTempUp, &bTempDown, &bTimeUp, &bTimeDown, &bWeightUp, &bWeightDown,
  &bSelDoor, &bSelBurn, &bSelFeed, &bSelBDoor, &bSelMainC, &bSelAsh,
  &hSpeed, &hStep,
  &bSpeedUp, &bSpeedDown,
  &bStepUp, &bStepDown,
  &bman1, &bman2, &bman3, &bman4,
  &bBlow1, &bBlow2, &bsettingStop,
  &bSave, &bZero, &bWall, &bSpeed, &bSet,
  &btManDir, &bManMainC,
  &bManDoor, &bManBDoor, &bManBurn, &bManPush, &bManAsh,
  &hManSpeed, &hManStep, &bManStop,
  &btManBurn, &btManIgn, &btManFeed, &btManWeigh, &b0,
  &stLog, &stBatch,
  NULL
};

// ================== LOG BUFFERS ==================
String logBuf0;
String logBuf1;
static const size_t LOG_MAX_CHARS = 900;
static const size_t BATCH_MAX_CHARS = 900;

bool logPageActive = false;
bool logDirty = false;
bool batchDirty = false;

static inline void trimFrontToLimit(String &s, size_t maxChars) {
  if (s.length() <= maxChars) return;
  int cut = s.length() - (int)maxChars;
  int pos = s.indexOf('\r', cut);
  if (pos < 0) pos = cut;
  s.remove(0, pos + 1);
}

static inline void appendLine(String &buf, const String &line, size_t maxChars) {
  buf += line;
  buf += '\r';
  trimFrontToLimit(buf, maxChars);
}

void uiFlushIfActive();

void uiAppendLogBuffered(const String &s) {
  appendLine(logBuf0, s, LOG_MAX_CHARS);
  logDirty = true;
}

void uiAppendLog(const String &s) {
  uiAppendLogBuffered(s);
  if (logPageActive) uiFlushIfActive();
}

void uiAppendBatchBuffered(const String &s) {
  appendLine(logBuf1, s, BATCH_MAX_CHARS);
  batchDirty = true;
}

void uiFlushIfActive() {
  if (!logPageActive) return;
  if (logDirty) { stLog.setText(logBuf0.c_str()); logDirty = false; }
  if (batchDirty) { stBatch.setText(logBuf1.c_str()); batchDirty = false; }
}

void uiAppendBatchSummary(uint32_t batchId, float vol, float w, float fuel,
                          float tpeak, uint32_t durS, const String &result) {
  char buf[128];
  snprintf(buf, sizeof(buf),
           "#%lu  V=%.1f  W=%.1f  Fuel=%.1f  Tp=%.0f  %lus  %s",
           (unsigned long)batchId, vol, w, fuel, tpeak, (unsigned long)durS, result.c_str());
  uiAppendBatchBuffered(String(buf));
  uiFlushIfActive();
}

// ================== HELPERS ==================
const char *levelToText(int level) {
  switch (level) {
    case 0: return "OFF";
    case 1: return "LOW";
    case 2: return "MED";
    case 3: return "HIGH";
  }
  return "OFF";
}

void updateBlowText(int id) {
  const char *txt = levelToText(blowLevel[id]);
  switch (id) {
    case 0: tBlow1.setText(txt); break;
    case 1: tBlow2.setText(txt); break;
  }
}

void showTempSet() {
  char buf[16];
  sprintf(buf, "%d C", (int)tempSetpoint);
  nTempSet.setText(buf);
}

void showWeightSet() {
  char buf[16];
  sprintf(buf, "%d kg", (int)minBatchWeight);
  nWeightSet.setText(buf);
}

void showTimeSet() {
  char buf[16];
  uint32_t menit = burnerActiveTimeSec / 60;
  sprintf(buf, "%lu min", (unsigned long)menit);
  nTimeSet.setText(buf);
}

void showVolSet() {
  char buf[16];
  sprintf(buf, "%d cm3", (int)volSetpoint);
  nVolSet.setText(buf);
}

void updateMotorInfoTexts() {
  uint8_t spdPct = speedToPct((uint8_t)selectedMotorIdx, (uint32_t)motorConfigs[selectedMotorIdx].speed);
  uint8_t stepPct = stepRealToPct(motorConfigs[selectedMotorIdx].steps);
  nSpeed.setValue(spdPct);
  hSpeed.setValue(spdPct);
  nStep.setValue((uint32_t)stepPct * 100UL);
  hStep.setValue(stepPct);
}

void applyMotorSpeedPct(uint8_t pct, bool sendToCTRL = true) {
  pct = clampPct100(pct);
  uint32_t realSpeed = pctToSpeed((uint8_t)selectedMotorIdx, pct);
  motorConfigs[selectedMotorIdx].speed = (long)realSpeed;
  motorSettingsDirty = true;
  updateMotorInfoTexts();
  if (sendToCTRL) {
    sendToNode(NODE_CTRL, String("M_SPD:") + String(selectedMotorIdx) + ":" + String(realSpeed));
  }
}

void applyMotorStepPct(uint8_t pct, bool sendToCTRL = true) {
  pct = clampPct100(pct);
  long realSteps = (long)pct * 100L;
  motorConfigs[selectedMotorIdx].steps = realSteps;
  motorSettingsDirty = true;
  updateMotorInfoTexts();
  if (sendToCTRL) {
    sendToNode(NODE_CTRL, String("M_STP:") + String(selectedMotorIdx) + ":" + String(realSteps));
  }
}

void updateSliderFromConfig() {
  uint8_t pct = speedToPct((uint8_t)selectedMotorIdx, (uint32_t)motorConfigs[selectedMotorIdx].speed);
  hSpeed.setValue(pct);
  hStep.setValue(motorConfigs[selectedMotorIdx].steps / 100);
}

void updateMotorSelectorUI() {
  bSelDoor.setValue(selectedMotorIdx == 0 ? 1 : 0);
  bSelFeed.setValue(selectedMotorIdx == 1 ? 1 : 0);
  bSelBurn.setValue(selectedMotorIdx == 2 ? 1 : 0);
  bSelBDoor.setValue(selectedMotorIdx == 3 ? 1 : 0);
  bSelAsh.setValue(selectedMotorIdx == 4 ? 1 : 0);
  bSelMainC.setValue(selectedMotorIdx == 5 ? 1 : 0);
}

void selectMotor(uint8_t idx) {
  selectedMotorIdx = idx;
  updateMotorSelectorUI();
  updateMotorInfoTexts();
}

// ================== HMI UPDATE ROUTINE ==================
void updateHMI() {
  char buf[20];

  if (fabs(currentTempC - lastTempSent) > 1) {
    dtostrf(currentTempC, 3, 0, buf);
    tTemp.setText(buf);
    lastTempSent = currentTempC;
    Serial.print("Current Temp: ");
    Serial.println(currentTempC);
  }

  if (fabs(currentTempC - lastTempManSent) > 1) {
    dtostrf(currentTempC, 3, 0, buf);
    tManTemp.setText(buf);
    lastTempManSent = currentTempC;
  }

  if (fabs(currentWeightKg - lastWeightSent) > 1) {
    dtostrf(currentWeightKg, 4, 1, buf);
    tWeight.setText(buf);
    tWeightCal.setText(buf);
    lastWeightSent = currentWeightKg;
    Serial.print("Current Weight: ");
    Serial.println(currentWeightKg);
  }

  if (fabs(currentWeightKg - lastWeightManSent) > 1) {
    dtostrf(currentWeightKg, 4, 1, buf);
    tManWeight.setText(buf);
    lastWeightManSent = currentWeightKg;
  }

  if (fabs(currentVol - lastVolSent) > 1) {
    dtostrf(currentVol, 4, 1, buf);
    tVolume.setText(buf);
    lastVolSent = currentVol;
    Serial.print("Current Volume: ");
    Serial.println(currentVol);
  }

  if (fabs(totalVolume - lastTotalVolSent) > 1) {
    dtostrf(totalVolume, 4, 1, buf);
    ttotVolume.setText(buf);
    lastTotalVolSent = totalVolume;
    Serial.print("Total Volume: ");
    Serial.println(totalVolume);
  }

  if (fabs(solarVolumeUsed_L - lastFuelSent) > 1) {
    dtostrf(solarVolumeUsed_L, 4, 1, buf);
    tFuel.setText(buf);
    lastFuelSent = solarVolumeUsed_L;
    Serial.print("solarVolumeUsed_L: ");
    Serial.println(solarVolumeUsed_L);
  }

  if (fabs(totalConsumedKg - lastTotalKgSent) > 1) {
    dtostrf(totalConsumedKg, 4, 1, buf);
    tTotWeight.setText(buf);
    lastTotalKgSent = totalConsumedKg;
  }

  if (remainingIsPreheat) {
    if (!lastPreheatSent) {
      tTime.setText("PRE-HEAT");
      lastPreheatSent = true;
    }
  } else {
    if (remainingTimeSec >= 0) {
      if (lastPreheatSent || remainingTimeSec != lastRemainingTimeSent) {
        char timeBuf[10];
        long mm = remainingTimeSec / 60;
        long ss = remainingTimeSec % 60;
        sprintf(timeBuf, "%02ld:%02ld", mm, ss);
        tTime.setText(timeBuf);
        lastRemainingTimeSent = remainingTimeSec;
        lastPreheatSent = false;
      }
    }
  }

  if ((int)currentFuelPct != lastFuelPctSent) {
    hFuel.setValue(currentFuelPct);
    char b[16];
    dtostrf(currentFuelLiter, 4, 1, b);
    tFuel.setText(b);
    lastFuelPctSent = (int)currentFuelPct;
  }

  if (totalBatchCount != lastBatchSent) {
    snprintf(buf, sizeof(buf), "%lu", totalBatchCount);
    tBatch.setText(buf);
    lastBatchSent = totalBatchCount;
    Serial.print("totalBatchCount: ");
    Serial.println(totalBatchCount);
  }

  if (currentState != lastState) {
    switch (currentState) {
      case IDLE: tStat.setText("IDLE"); break;
      case WEIGHING: tStat.setText("WEIGHING"); break;
      case OPENING_MAIN_DOOR: tStat.setText("D.O"); break;
      case DUMPING_IN: tStat.setText("F.IN"); break;
      case DUMPING_OUT: tStat.setText("F. OUT"); break;
      case CLOSING_MAIN_DOOR: tStat.setText("DOOR CLOSE"); break;
      case OPENING_BURN_DOOR: tStat.setText("B-DOOR OPEN"); break;
      case BURNER_IN: tStat.setText("BURNER IN"); break;
      case IGNITING: tStat.setText("IGNITING"); break;
      case BURNING: tStat.setText("BURNING"); break;
      case BURNER_OUT: tStat.setText("BURNER OUT"); break;
      case CLOSING_BURN_DOOR: tStat.setText("B-DOOR CLOSE"); break;
      case FAULT: tStat.setText("FAULT"); break;
    }
    lastState = currentState;
  }
}

// ================== NEXTION SENDME POLL ==================
void pollNextionSendme() {
  while (nexSerial.available()) {
    uint8_t b = (uint8_t)nexSerial.read();
    if (b == 0x66) {
      while (!nexSerial.available()) {}
      uint8_t pid = (uint8_t)nexSerial.read();
      for (int i = 0; i < 3; i++) { while (!nexSerial.available()) {} (void)nexSerial.read(); }
      currentPageId = pid;
      logPageActive = (currentPageId == 3);
      if (logPageActive) {
        logDirty = true;
        batchDirty = true;
        uiFlushIfActive();
      }
    }
  }
}

void page3PushCallback(void *ptr) {
  logPageActive = true;
  logDirty = true;
  batchDirty = true;
  uiFlushIfActive();
}

void page3PopCallback(void *ptr) {
  logPageActive = false;
}

// ================== CALLBACKS (KIRIM VIA RS485) ==================
void btAutoPopCallback(void *ptr) {
  uint32_t dual_state;
  btAuto.getValue(&dual_state);
  if (dual_state == 1) sendToNode(NODE_CTRL, "CMD_AUTO_START");
  else sendToNode(NODE_CTRL, "CMD_AUTO_STOP");
}

void bStopPopCallback(void *ptr) {
  sendToNode(NODE_CTRL, "CMD_EMERGENCY_STOP");
}

void b0PopCallback(void *ptr) {
  sendToNode(NODE_CTRL, "CMD_ONE_SHOT_AUTO");
}

void applyManualSpeedPct(uint8_t pct) {
  pct = clampPctMan(pct);
  hManSpeed.setValue(pct);
  uint32_t realSpeed = pctToSpeed((uint8_t)manualMotorIdx, pct);
  sendToNode(NODE_CTRL, String("SET_M_SPD:") + String(realSpeed));
}

void applyManualStepPct(uint8_t pct) {
  pct = clampPctMan(pct);
  hManStep.setValue(pct);
  long realSteps = (long)pct * 100L;
  sendToNode(NODE_CTRL, String("SET_M_STP:") + String(realSteps));
}

void btManDirPopCallback(void *ptr) {
  uint32_t val;
  btManDir.getValue(&val);
  sendToNode(NODE_CTRL, String("CMD_SET_MANUAL_DIR:") + String(val));
}

void hManSpeedPopCallback(void *ptr) {
  uint32_t v;
  if (!hManSpeed.getValue(&v)) return;
  applyManualSpeedPct((uint8_t)v);
}

void hManStepPopCallback(void *ptr) {
  uint32_t v;
  if (!hManStep.getValue(&v)) return;
  applyManualStepPct((uint8_t)v);
}

void bManSpeedUpPopCallback(void *ptr) {
  uint32_t v = 1;
  hManSpeed.getValue(&v);
  applyManualSpeedPct(clampPctMan((long)v + 1));
}

void bManSpeedDownPopCallback(void *ptr) {
  uint32_t v = 1;
  hManSpeed.getValue(&v);
  applyManualSpeedPct(clampPctMan((long)v - 1));
}

void bManStepUpPopCallback(void *ptr) {
  uint32_t v = 1;
  hManStep.getValue(&v);
  applyManualStepPct(clampPctMan((long)v + 1));
}

void bManStepDownPopCallback(void *ptr) {
  uint32_t v = 1;
  hManStep.getValue(&v);
  applyManualStepPct(clampPctMan((long)v - 1));
}

void reqManualMove(int motorID) {
  manualMotorIdx = motorID;
  sendToNode(NODE_CTRL, String("REQ_MOVE:") + String(motorID));
}

void bManDoorPopCallback(void *ptr) { reqManualMove(0); }
void bManPushPopCallback(void *ptr) { reqManualMove(1); }
void bManBurnPopCallback(void *ptr) { reqManualMove(2); }
void bManBDoorPopCallback(void *ptr) { reqManualMove(3); }
void bManAshPopCallback(void *ptr) { reqManualMove(4); }
void bManMainCPopCallback(void *ptr) { reqManualMove(5); }

void btManIgnPopCallback(void *ptr) {
  uint32_t val;
  btManIgn.getValue(&val);
  sendToNode(NODE_CTRL, String("CMD_MAN_IGN:") + String(val));
}

void btManBurnPopCallback(void *ptr) {
  uint32_t val;
  btManBurn.getValue(&val);
  sendToNode(NODE_CTRL, String("CMD_MAN_BURN:") + String(val));
}

void btManFeedPopCallback(void *ptr) {
  uint32_t val;
  btManFeed.getValue(&val);
  sendToNode(NODE_CTRL, String("CMD_MAN_FEED:") + String(val));
}

void btManWeighPopCallback(void *ptr) {
  uint32_t val;
  btManWeigh.getValue(&val);
  sendToNode(NODE_CTRL, String("CMD_MAN_WEIGH:") + String(val));
}

void bTempUpPopCallback(void *ptr) {
  tempSetpoint += 50.0;
  showTempSet();
  sendToNode(NODE_CTRL, String("SET_TE:") + String(tempSetpoint, 0));
}

void bTempDownPopCallback(void *ptr) {
  tempSetpoint -= 50.0;
  if (tempSetpoint < 0) tempSetpoint = 0;
  showTempSet();
  sendToNode(NODE_CTRL, String("SET_TE:") + String(tempSetpoint, 0));
}

void bTimeUpPopCallback(void *ptr) {
  burnerActiveTimeSec += 60;
  showTimeSet();
  sendToNode(NODE_CTRL, String("SET_TI:") + String(burnerActiveTimeSec));
}

void bTimeDownPopCallback(void *ptr) {
  if (burnerActiveTimeSec >= 60) burnerActiveTimeSec -= 60;
  showTimeSet();
  sendToNode(NODE_CTRL, String("SET_TI:") + String(burnerActiveTimeSec));
}

void bWeightUpPopCallback(void *ptr) {
  minBatchWeight += 1.0;
  showWeightSet();
  sendToNode(NODE_CTRL, String("SET_W:") + String(minBatchWeight, 1));
}

void bWeightDownPopCallback(void *ptr) {
  if (minBatchWeight >= 1.0) minBatchWeight -= 1.0;
  showWeightSet();
  sendToNode(NODE_CTRL, String("SET_W:") + String(minBatchWeight, 1));
}

void bSelDoorPopCallback(void *ptr) { selectMotor(0); }
void bSelFeedPopCallback(void *ptr) { selectMotor(1); }
void bSelBurnPopCallback(void *ptr) { selectMotor(2); }
void bSelBDoorPopCallback(void *ptr) { selectMotor(3); }
void bSelAshPopCallback(void *ptr) { selectMotor(4); }
void bSelMainCPopCallback(void *ptr) { selectMotor(5); }

void hSpeedPopCallback(void *ptr) {
  uint32_t v;
  if (!hSpeed.getValue(&v)) return;
  applyMotorSpeedPct(clampPct100((long)v), true);
}

void hStepPopCallback(void *ptr) {
  uint32_t v;
  if (!hStep.getValue(&v)) return;
  applyMotorStepPct(clampPct100((long)v), true);
}

void bSpeedUpPopCallback(void *ptr) {
  uint32_t v;
  if (!hSpeed.getValue(&v)) v = 1;
  applyMotorSpeedPct(clampPct100((long)v + 1), true);
}

void bSpeedDownPopCallback(void *ptr) {
  uint32_t v;
  if (!hSpeed.getValue(&v)) v = 1;
  applyMotorSpeedPct(clampPct100((long)v - 1), true);
}

void bStepUpPopCallback(void *ptr) {
  uint32_t v;
  if (!hStep.getValue(&v)) v = 1;
  applyMotorStepPct(clampPct100((long)v + 1), true);
}

void bStepDownPopCallback(void *ptr) {
  uint32_t v;
  if (!hStep.getValue(&v)) v = 1;
  applyMotorStepPct(clampPct100((long)v - 1), true);
}

void updateBlowerState(int id) {
  blowLevel[id]++;
  if (blowLevel[id] > 3) blowLevel[id] = 0;
  sendToNode(NODE_CTRL, String("BLOW:") + String(id) + ":" + String(blowLevel[id]));
  updateBlowText(id);
}

void bBlow1PopCallback(void *ptr) { updateBlowerState(0); }
void bBlow2PopCallback(void *ptr) { updateBlowerState(1); }

void bSavePopCallback(void *ptr) {
  sendToNode(NODE_CTRL, "CMD_SAVE_SETTINGS");
  motorSettingsDirty = false;
}

void bZeroPopCallback(void *ptr) { sendToNode(NODE_CTRL, "CMD_TARE_SCALE"); }
void bSetPopCallback(void *ptr) { sendToNode(NODE_CTRL, "CMD_SET_SCALE"); }

// LIDAR-related commands: kirim ke CTRL (atau bisa juga ke NODE_LIDAR kalau kamu mau)
void bWallPopCallback(void *ptr) {
  // logEvent butuh didefinisikan dulu
  // kita pakai uiAppendLog minimal
  uiAppendLog("[INFO][LIDAR] Calibrating background...");
  uiAppendLog("[INFO][LIDAR] Background OK");
  sendToNode(NODE_CTRL, "CMD_CAL_WALL");
}

void bSpeedPopCallback(void *ptr) {
  uiAppendLog("[WARN][LIDAR] Speed cal not implemented on HMI yet");
  sendToNode(NODE_CTRL, "CMD_CAL_SPEED");
}

void nWeightSetPopCallback(void *ptr) {
  char buf[16] = {0};
  nWeightSet.getText(buf, sizeof(buf) - 1);
  minBatchWeight = atof(buf);
  showWeightSet();
}

void nTimeSetPopCallback(void *ptr) {
  char buf[16] = {0};
  nTimeSet.getText(buf, sizeof(buf) - 1);
  uint32_t menit = (uint32_t)atoi(buf);
  burnerActiveTimeSec = menit * 60;
  showTimeSet();
}

void nVolSetPopCallback(void *ptr) {
  char buf[16] = {0};
  nVolSet.getText(buf, sizeof(buf) - 1);
  uint32_t vol = (uint32_t)atoi(buf);
  volSetpoint = (float)vol;
  showVolSet();
  sendToNode(NODE_CTRL, String("SET_VB:") + String(volSetpoint, 0));
}

// ================== LOGGING (UI-only minimal) ==================
void logEvent(const char *lvl, const char *tag, const String &msg) {
  char buf[160];
  snprintf(buf, sizeof(buf), "[%s][%s] %s", lvl, tag, msg.c_str());
  uiAppendLogBuffered(String(buf));
  uiFlushIfActive();
}

void logBatchSummary(uint32_t batchId, float vol, float w, float fuel,
                     float tpeak, uint32_t durS, const String &result) {
  uiAppendBatchSummary(batchId, vol, w, fuel, tpeak, durS, result);
}

// ================== PROCESS LINE (payload dari CTRL) ==================
void ProcessLine(String data) {
  if (data.startsWith("METRICS:")) {
    String x = data.substring(8);
    int c1 = x.indexOf(',');
    int c2 = x.indexOf(',', c1 + 1);
    if (c1 > 0 && c2 > 0) {
      solarVolumeUsed_L = x.substring(0, c1).toFloat();
      totalConsumedKg = x.substring(c1 + 1, c2).toFloat();
      totalBatchCount = x.substring(c2 + 1).toInt();
    }
  }

  // DATA:TEMP,WEIGHT,STATE,vol,totalVol
  if (data.startsWith("DATA:")) {
    int c1 = data.indexOf(',');
    int c2 = data.indexOf(',', c1 + 1);
    int c3 = data.indexOf(',', c2 + 1);
    int c4 = data.indexOf(',', c3 + 1);

    if (c1 > 0 && c2 > 0 && c3 > 0) {
      currentTempC = data.substring(5, c1).toFloat();
      currentWeightKg = data.substring(c1 + 1, c2).toFloat();
      int st = data.substring(c2 + 1, c3).toInt();
      if (st >= IDLE && st <= FAULT) currentState = (SystemState)st;

      if (c4 > 0) {
        currentVol = data.substring(c3 + 1, c4).toFloat();
        totalVolume = data.substring(c4 + 1).toFloat();
      } else {
        currentVol = data.substring(c3 + 1).toFloat();
      }
    }
  }

  if (data.startsWith("PROG:")) {
    int val = data.substring(5).toInt();
    lastProgValue = val;
    jStat.setValue(val);
  } else if (data.startsWith("REM_TIME:")) {
    long val = data.substring(9).toInt();
    remainingTimeSec = val;
    remainingIsPreheat = false;
    remainingLastRxMs = millis();
  }

  if (data.startsWith("PREHEAT")) {
    remainingIsPreheat = true;
    remainingLastRxMs = millis();
  }

  if (data.startsWith("W_SET:")) {
    minBatchWeight = data.substring(6).toFloat();
    showWeightSet();
  } else if (data.startsWith("TE_SET:")) {
    tempSetpoint = data.substring(7).toFloat();
    showTempSet();
  } else if (data.startsWith("TI_SET:")) {
    burnerActiveTimeSec = data.substring(7).toInt();
    showTimeSet();
  } else if (data.startsWith("VB_SET:")) {
    volSetpoint = data.substring(7).toInt();
    showVolSet();
  }

  if (data.startsWith("M_SYNC:")) {
    int firstColon = data.indexOf(':');
    int secondColon = data.indexOf(':', firstColon + 1);
    int thirdColon = data.indexOf(':', secondColon + 1);
    if (firstColon > 0 && secondColon > 0 && thirdColon > 0) {
      int id = data.substring(firstColon + 1, secondColon).toInt();
      long spd = data.substring(secondColon + 1, thirdColon).toInt();
      long stp = data.substring(thirdColon + 1).toInt();
      if (id >= 0 && id < 6) {
        motorConfigs[id].speed = spd;
        motorConfigs[id].steps = stp;
        if (id == selectedMotorIdx) updateMotorInfoTexts();
      }
    }
  }

  if (data.startsWith("B_SYNC:")) {
    int first = data.indexOf(':');
    int second = data.indexOf(':', first + 1);
    int id = data.substring(first + 1, second).toInt();
    int lvl = data.substring(second + 1).toInt();
    if (id >= 0 && id < 3) {
      blowLevel[id] = lvl;
      updateBlowText(id);
    }
  }

  if (data.startsWith("LOG:")) {
    int c1 = data.indexOf(',');
    int c2 = data.indexOf(',', c1 + 1);
    if (c1 > 0 && c2 > 0) {
      String lvl = data.substring(4, c1);
      String tag = data.substring(c1 + 1, c2);
      String msg = data.substring(c2 + 1);
      logEvent(lvl.c_str(), tag.c_str(), msg);
    }
  }

  if (data.startsWith("BATCH_SUM:")) {
    String x = data.substring(10);
    int p0 = x.indexOf(',');
    int p1 = x.indexOf(',', p0 + 1);
    int p2 = x.indexOf(',', p1 + 1);
    int p3 = x.indexOf(',', p2 + 1);
    int p4 = x.indexOf(',', p3 + 1);
    int p5 = x.indexOf(',', p4 + 1);

    if (p0 > 0 && p5 > 0) {
      uint32_t bid = x.substring(0, p0).toInt();
      float vol = x.substring(p0 + 1, p1).toFloat();
      float w = x.substring(p1 + 1, p2).toFloat();
      float fuel = x.substring(p2 + 1, p3).toFloat();
      float tp = x.substring(p3 + 1, p4).toFloat();
      uint32_t dur = x.substring(p4 + 1, p5).toInt();
      String res = x.substring(p5 + 1);
      logBatchSummary(bid, vol, w, fuel, tp, dur, res);
    }
  }

  if (data.startsWith("FUEL:")) {
    String x = data.substring(5);
    int c = x.indexOf(',');
    if (c > 0) {
      currentFuelPct = (uint8_t)x.substring(0, c).toInt();
      currentFuelLiter = x.substring(c + 1).toFloat();
    } else {
      currentFuelPct = (uint8_t)x.toInt();
    }
  }
}

// ================== PROCESS LINE (payload dari node selain CTRL) ==================
void ProcessNodePayload(uint8_t nodeId, const String &payload) {
  // Kalau nodeId == CTRL: pakai ProcessLine lama
  if (nodeId == NODE_CTRL) {
    ProcessLine(payload);
    return;
  }

  // Sensor dedicated nodes (opsional, sederhana)
  if (nodeId == NODE_TEMP) {
    if (payload.startsWith("TEMP:")) {
      currentTempC = payload.substring(5).toFloat();
    }
    return;
  }

  if (nodeId == NODE_WGHT) {
    if (payload.startsWith("WEIGHT:")) {
      currentWeightKg = payload.substring(7).toFloat();
    }
    return;
  }

  if (nodeId == NODE_LIDAR) {
    // VOL:cur,total
    if (payload.startsWith("VOL:")) {
      String x = payload.substring(4);
      int c = x.indexOf(',');
      if (c > 0) {
        currentVol = x.substring(0, c).toFloat();
        totalVolume = x.substring(c + 1).toFloat();
      } else {
        currentVol = x.toFloat();
      }
    }
    return;
  }
}

// ================== RS485 RX PARSER ==================
String rs485Buf;

static bool parseNodeFrame(const String &line, uint8_t &nodeIdOut, String &payloadOut) {
  // Expect: Nxx:payload
  if (line.length() < 5) return false;
  if (line[0] != 'N') return false;

  char d1 = line[1];
  char d2 = line[2];
  if (d1 < '0' || d1 > '9' || d2 < '0' || d2 > '9') return false;
  if (line[3] != ':') return false;

  nodeIdOut = (uint8_t)((d1 - '0') * 10 + (d2 - '0'));
  payloadOut = line.substring(4);
  return true;
}

void ParseDataFromRS485() {
  while (CtrlBus.available() > 0) {
    char c = (char)CtrlBus.read();
    if (c == '\r') continue;
    if (c == '\n') {
      rs485Buf.trim();
      if (rs485Buf.length() > 0) {
        uint8_t nid = 0;
        String payload;
        if (parseNodeFrame(rs485Buf, nid, payload)) {
          ProcessNodePayload(nid, payload);
        } else {
          // kalau ada noise / frame tak dikenal
          // Serial.println(String("RS485 unparsed: ") + rs485Buf);
        }
      }
      rs485Buf = "";
    } else {
      rs485Buf += c;
      if (rs485Buf.length() > 180) rs485Buf = ""; // anti runaway
    }
  }
}

// ================== SETUP & LOOP ==================
void setup() {
  // USB debug
  Serial.begin(115200);
  delay(200);

  // Nextion
  nexSerial.begin(9600, SERIAL_8N1, 25, 26);
  nexInit();

  // RS485
  pinMode(RS485_DE_RE, OUTPUT);
  rs485SetTx(false);
  CtrlBus.begin(9600, SERIAL_8N1, RS485_RX, RS485_TX);

  // Clear buffers
  logBuf0 = "";
  logBuf1 = "";
  logDirty = true;
  batchDirty = true;

  // Attach callbacks
  bTempUp.attachPop(bTempUpPopCallback, &bTempUp);
  bTempDown.attachPop(bTempDownPopCallback, &bTempDown);
  bTimeUp.attachPop(bTimeUpPopCallback, &bTimeUp);
  bTimeDown.attachPop(bTimeDownPopCallback, &bTimeDown);
  bWeightUp.attachPop(bWeightUpPopCallback, &bWeightUp);
  bWeightDown.attachPop(bWeightDownPopCallback, &bWeightDown);
  bSave.attachPop(bSavePopCallback, &bSave);
  bZero.attachPop(bZeroPopCallback, &bZero);
  bSet.attachPop(bSetPopCallback, &bSet);
  bWall.attachPop(bWallPopCallback, &bWall);
  bSpeed.attachPop(bSpeedPopCallback, &bSpeed);
  nWeightSet.attachPop(nWeightSetPopCallback, &nWeightSet);
  nTimeSet.attachPop(nTimeSetPopCallback, &nTimeSet);
  nVolSet.attachPop(nVolSetPopCallback, &nVolSet);

  bSelDoor.attachPop(bSelDoorPopCallback, &bSelDoor);
  bSelBurn.attachPop(bSelBurnPopCallback, &bSelBurn);
  bSelFeed.attachPop(bSelFeedPopCallback, &bSelFeed);
  bSelBDoor.attachPop(bSelBDoorPopCallback, &bSelBDoor);
  bSelAsh.attachPop(bSelAshPopCallback, &bSelAsh);
  bSelMainC.attachPop(bSelMainCPopCallback, &bSelMainC);

  hSpeed.attachPop(hSpeedPopCallback, &hSpeed);
  hStep.attachPop(hStepPopCallback, &hStep);

  bSpeedUp.attachPop(bSpeedUpPopCallback, &bSpeedUp);
  bSpeedDown.attachPop(bSpeedDownPopCallback, &bSpeedDown);

  bStepUp.attachPop(bStepUpPopCallback, &bStepUp);
  bStepDown.attachPop(bStepDownPopCallback, &bStepDown);

  bBlow1.attachPop(bBlow1PopCallback, &bBlow1);
  bBlow2.attachPop(bBlow2PopCallback, &bBlow2);

  btAuto.attachPop(btAutoPopCallback, &btAuto);
  bStop.attachPop(bStopPopCallback, &bStop);
  bManStop.attachPop(bStopPopCallback, &bManStop);
  bsettingStop.attachPop(bStopPopCallback, &bsettingStop);

  btManDir.attachPop(btManDirPopCallback, &btManDir);
  bManDoor.attachPop(bManDoorPopCallback, &bManDoor);
  bManBDoor.attachPop(bManBDoorPopCallback, &bManBDoor);
  bManBurn.attachPop(bManBurnPopCallback, &bManBurn);
  bManPush.attachPop(bManPushPopCallback, &bManPush);
  bManAsh.attachPop(bManAshPopCallback, &bManAsh);
  bManMainC.attachPop(bManMainCPopCallback, &bManMainC);
  hManSpeed.attachPop(hManSpeedPopCallback, &hManSpeed);
  hManStep.attachPop(hManStepPopCallback, &hManStep);
  btManIgn.attachPop(btManIgnPopCallback, &btManIgn);
  btManBurn.attachPop(btManBurnPopCallback, &btManBurn);
  btManWeigh.attachPop(btManWeighPopCallback, &btManWeigh);
  b0.attachPop(b0PopCallback, &b0);

  bman1.attachPop(bManSpeedDownPopCallback, &bman1);
  bman2.attachPop(bManSpeedUpPopCallback, &bman2);
  bman3.attachPop(bManStepUpPopCallback, &bman3);
  bman4.attachPop(bManStepDownPopCallback, &bman4);

  page3.attachPush(page3PushCallback, &page3);
  page3.attachPop(page3PopCallback, &page3);

  // minta sync ke CTRL dan bisa juga ping slave sensor
  delay(800);
  sendToNode(NODE_CTRL, "CMD_REQ_SYNC");
  // optional polling awal
  // sendToNode(NODE_TEMP, "REQ_TEMP");
  // sendToNode(NODE_WGHT, "REQ_WEIGHT");
  // sendToNode(NODE_LIDAR, "REQ_VOL");

  Serial.println("HMI MASTER RS485 MULTI-SLAVE READY");
}

// Polling sederhana (biar multi slave tetap tenang, no collision)
static uint32_t lastPollMs = 0;
static uint8_t pollIdx = 0;

void pollSlavesSimple() {
  uint32_t now = millis();
  if (now - lastPollMs < 250) return; // 4Hz total polling
  lastPollMs = now;

  // round robin
  switch (pollIdx) {
    case 0: sendToNode(NODE_TEMP, "REQ_TEMP"); break;
    case 1: sendToNode(NODE_WGHT, "REQ_WEIGHT"); break;
    case 2: sendToNode(NODE_LIDAR, "REQ_VOL"); break;
    default: break;
  }
  pollIdx = (pollIdx + 1) % 3;
}

void loop() {
  nexLoop(nex_listen_list);

  // optional: jika kamu perlu deteksi page sendme
  pollNextionSendme();

  // RX RS485
  ParseDataFromRS485();

  // polling sensor (opsional)
  pollSlavesSimple();

  // refresh UI
  updateHMI();
}
