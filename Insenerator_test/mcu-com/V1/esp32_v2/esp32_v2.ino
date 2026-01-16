/*
 * MCU-HMI (Nextion Controller) ESP32
 * -------------------------------------
 * Tugas: Menangani UI, Input User, dan Menampilkan Data
 * Hardware: Nextion Display (Serial2)
 * Komunikasi: Mengirim perintah ke CTRL, Menerima update status dari CTRL
 */
// #include <SoftwareSerial.h>
#include <Arduino.h>
#include <SPI.h>
#include <SD.h>
#include <Preferences.h>


Preferences prefs;


SPIClass sdSPI(VSPI);

bool sdOK = false;
uint32_t sessionId = 0;
String sessionBase;  // contoh: "/S000123"
File fEvents, fBatches, fTrend;

// Nextion scroll text (buat di editor Nextion):
// pageLog: ScrollText stLog


// === SD Pins (ubah sesuai wiring kamu) ===
static const int SD_CS   = 22;
static const int SD_SCK  = 18;
static const int SD_MISO = 19;
static const int SD_MOSI = 23;


// HardwareSerial Serial(1);

// [Nextion Serial Define]

#define NEXTION_RX 16
#define NEXTION_TX 17

#define nexSerial Serial2  // dipakai di library Nextion
#include <Nextion.h>

enum SystemState {
  IDLE = 0,
  WEIGHING,
  OPENING_MAIN_DOOR,
  DUMPING_IN,
  DUMPING_TRASH,
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
// Variabel ini nanti di-update via Serial dari MCU-CTRL
float currentTempC = 0.0;
float currentWeightKg = 0.0;
float currentVol = 0.0;
float solarVolumeUsed_L = 0.0;
float totalConsumedKg = 0.0;
float totalVolume = 0.0;
unsigned long totalBatchCount = 0;
float minBatchWeight = 5.0;  // Default, nanti disinkronkan
uint32_t burnerActiveTimeSec = 30;
float tempSetpoint = 1200.0;
float volSetpoint = 20.0;

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
int manualMotorIdx = 0; // motor yang akan digerakkan di manual

// Blower UI State
int blowLevel[3] = { 0, 0, 0 };

// ===== Remaining time state (mirror dari CTRL) =====
long remainingTimeSec = -1;           // -1 = belum ada data
bool remainingIsPreheat = false;      // true = tampil "PRE-HEAT"
unsigned long remainingLastRxMs = 0;  // kapan terakhir update diterima (opsional)

// ===== UI helper untuk remaining time =====
long lastRemainingTimeSent = -999999;
bool lastPreheatSent = false;

// Struktur Konfigurasi Motor (Mirror untuk Slider)
struct MotorConfig {
  long speed;
  long accel;
  long steps;
};
// Default sementara agar tidak error saat akses array
MotorConfig motorConfigs[6] = {
  { 1000, 200, 2000 }, { 1000, 200, 5000 }, { 1000, 200, 3000 }, { 800, 200, 1000 }, { 800, 200, 1000 }, { 800, 200, 1000 }
};

static const uint32_t MOTOR_MAX_SPD[6] = {
  10000,  // Door
  12000,  // Feed
  15000,  // Burner mover
  8000,   // Burn Door
  7000,   // Ash
  14000   // Main Conveyor
};

// minimal speed (biar persen kecil tetap jalan, opsional)
static const uint32_t MOTOR_MIN_SPD[6] = {
  300, 300, 300, 300, 300, 300
};

static inline uint8_t clampPctMan(long v) {
  return clampPct100(v);
}


static inline uint8_t clampPct100(long v) {
  if (v < 1) return 1;
  if (v > 100) return 100;
  return (uint8_t)v;
}

static inline uint8_t stepRealToPct(long stepsReal) {
  // stepsReal disimpan = pct*100
  long pct = stepsReal / 100L;
  return clampPct100(pct);
}

static inline uint8_t clampPct(int v) {
  if (v < 1) return 1;
  if (v > 100) return 100;
  return (uint8_t)v;
}

static inline uint32_t pctToSpeed(uint8_t motorId, uint8_t pct) {
  pct = clampPct(pct);
  uint32_t maxS = MOTOR_MAX_SPD[motorId];
  uint32_t minS = MOTOR_MIN_SPD[motorId];

  // linear: min + (pct/100)*(max-min)
  uint32_t span = (maxS > minS) ? (maxS - minS) : 0;
  return minS + (span * pct) / 100;
}

static inline uint8_t speedToPct(uint8_t motorId, uint32_t speed) {
  uint32_t maxS = MOTOR_MAX_SPD[motorId];
  uint32_t minS = MOTOR_MIN_SPD[motorId];   // kalau kamu pakai min=0, tetap aman

  if (maxS <= minS) return 100;
  if (speed <= minS) return 1;
  if (speed >= maxS) return 100;

  uint32_t span = maxS - minS;
  uint32_t pct = ((speed - minS) * 100UL) / span;
  return clampPct((int)pct);
}




// ================== NEXTION OBJECTS ==================
/* [Nextion UI Objects] */
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
NexButton bSpeedUp   = NexButton(2, 38, "b1"); // dulu: bSpeedUp
NexButton bSpeedDown = NexButton(2, 40, "b2"); // dulu: bSpeedDown

NexButton bStepUp    = NexButton(2, 37, "b4"); // dulu: bStepUp
NexButton bStepDown  = NexButton(2, 39, "b0"); // dulu: bStepDown

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


// pageBatch: ScrollText stBatch
NexPage page3 = NexPage(3, 0, "page3");      // <=== tambah ini

NexScrolltext stLog   = NexScrolltext(3, 5, "slt0");     // contoh page3 id=3, comp id=1
NexScrolltext stBatch = NexScrolltext(3, 6, "slt1");   // contoh page4 id=4, comp id=1



// --- LISTEN LIST ---
NexTouch *nex_listen_list[] = {
  &page0, &page1, &page2,&page3,
  &btAuto, &bStop, &b1,
  &nWeightSet, &nTimeSet,&hFuel,
  &bTempUp, &bTempDown, &bTimeUp, &bTimeDown, &bWeightUp, &bWeightDown,
  &bSelDoor, &bSelBurn, &bSelFeed, &bSelBDoor, &bSelMainC, &bSelAsh,
  &hSpeed, &hStep,&bStepUp,&hSpeed, &hStep,
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

// ===== ScrollText buffers (append dilakukan di MCU) =====
String logBuf0;
String logBuf1;

static const size_t LOG_MAX_CHARS   = 900;
static const size_t BATCH_MAX_CHARS = 900;

bool logPageActive = false;   // true kalau page3 aktif
bool logDirty = false;
bool batchDirty = false;

static inline void trimFrontToLimit(String &s, size_t maxChars) {
  if (s.length() <= maxChars) return;

  int cut = s.length() - (int)maxChars;
  int pos = s.indexOf('\r', cut);   // IMPORTANT: '\r' bukan "\\r"
  if (pos < 0) pos = cut;
  s.remove(0, pos + 1);
}

static inline void appendLine(String &buf, const String &line, size_t maxChars) {
  buf += line;
  buf += '\r'; // newline Nextion
  trimFrontToLimit(buf, maxChars);
}

void uiAppendLog(const String &s) {
  uiAppendLogBuffered(s);
  if (logPageActive) uiFlushIfActive();
}

void uiAppendLogBuffered(const String &s) {
  appendLine(logBuf0, s, LOG_MAX_CHARS);
  logDirty = true;
}


void uiAppendBatchBuffered(const String &s) {
  appendLine(logBuf1, s, BATCH_MAX_CHARS);
  batchDirty = true;
}

// Flush hanya jika page3 aktif
void uiFlushIfActive() {
  if (!logPageActive) return;

  if (logDirty) {
    stLog.setText(logBuf0.c_str());
    logDirty = false;
  }
  if (batchDirty) {
    stBatch.setText(logBuf1.c_str());
    batchDirty = false;
  }
}
void uiAppendBatchSummary(uint32_t batchId, float vol, float w, float fuel,
                          float tpeak, uint32_t durS, const String& result) {
  char buf[128];
  snprintf(buf, sizeof(buf),
           "#%lu  V=%.1f  W=%.1f  Fuel=%.1f  Tp=%.0f  %lus  %s",
           (unsigned long)batchId, vol, w, fuel, tpeak, (unsigned long)durS, result.c_str());

  uiAppendBatchBuffered(String(buf));
  uiFlushIfActive();   // optional
}

// ================== HELPER FUNCTIONS ==================

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
  sprintf(buf, "%d m3", (int)volSetpoint);
  nVolSet.setText(buf);
}



void updateMotorInfoTexts() {
  // ambil dari config (real), lalu tampilkan ke UI sebagai persen (speed) & persen (step)
  uint8_t spdPct  = speedToPct((uint8_t)selectedMotorIdx, (uint32_t)motorConfigs[selectedMotorIdx].speed);
  uint8_t stepPct = stepRealToPct(motorConfigs[selectedMotorIdx].steps);

  // Update Number + Slider sekaligus (sesuai permintaan kamu)
  nSpeed.setValue(spdPct);
  hSpeed.setValue(spdPct);

  // nStep: kamu minta dikalikan 100 dari nilai hStep (jadi tampil real steps)
  nStep.setValue((uint32_t)stepPct * 100UL);
  hStep.setValue(stepPct);
}

void applyMotorSpeedPct(uint8_t pct, bool sendToCTRL = true) {
  pct = clampPct100(pct);

  uint32_t realSpeed = pctToSpeed((uint8_t)selectedMotorIdx, pct);
  motorConfigs[selectedMotorIdx].speed = (long)realSpeed;
  motorSettingsDirty = true;

  // sync UI
  updateMotorInfoTexts();

  if (sendToCTRL) {
    Serial.print("M_SPD:");
    Serial.print(selectedMotorIdx);
    Serial.print(":");
    Serial.println(realSpeed);
  }
}

void applyMotorStepPct(uint8_t pct, bool sendToCTRL = true) {
  pct = clampPct100(pct);

  long realSteps = (long)pct * 100L;   // sesuai permintaan: step yang dipakai = pct*100
  motorConfigs[selectedMotorIdx].steps = realSteps;
  motorSettingsDirty = true;

  // sync UI
  updateMotorInfoTexts();

  if (sendToCTRL) {
    Serial.print("M_STP:");
    Serial.print(selectedMotorIdx);
    Serial.print(":");
    Serial.println(realSteps);
  }
}

void updateSliderFromConfig() {
  uint8_t pct = speedToPct((uint8_t)selectedMotorIdx, (uint32_t)motorConfigs[selectedMotorIdx].speed);
  hSpeed.setValue(pct);
  hStep.setValue(motorConfigs[selectedMotorIdx].steps/100);
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

void initSettingPageTexts() {
  showTempSet();
  showTimeSet();
  showWeightSet();

  selectedMotorIdx = 0;
  updateMotorSelectorUI();
  updateSliderFromConfig();
  updateMotorInfoTexts();

  for (int i = 0; i < 2; i++) {
    updateBlowText(i);
  }
}

// ================== HMI UPDATE ROUTINE ==================
void updateHMI() {
  // Dipanggil di loop utama untuk refresh Text Object jika ada data baru dari CTRL
  char buf[20];

  // --- Temp Dashboard ---
  if (fabs(currentTempC - lastTempSent) > 1) {
    dtostrf(currentTempC, 3, 0, buf);
    tTemp.setText(buf);
    lastTempSent = currentTempC;
    Serial.print("Current Temp: ");
    Serial.println(currentTempC);
  }

  // --- Temp Manual Page ---
  if (fabs(currentTempC - lastTempManSent) > 1) {
    dtostrf(currentTempC, 3, 0, buf);
    tManTemp.setText(buf);
    lastTempManSent = currentTempC;
  }

  // --- Weight Dashboard ---
  if (fabs(currentWeightKg - lastWeightSent) > 1) {
    dtostrf(currentWeightKg, 4, 1, buf);
    tWeight.setText(buf);
    tWeightCal.setText(buf);
    lastWeightSent = currentWeightKg;
    Serial.print("Current Weight: ");
    Serial.println(currentWeightKg);
  }

  // --- Weight Manual Page ---
  if (fabs(currentWeightKg - lastWeightManSent) > 1) {
    dtostrf(currentWeightKg, 4, 1, buf);
    tManWeight.setText(buf);
    lastWeightManSent = currentWeightKg;
  }

// ---  Volume Dashboard ---

  if (fabs(currentVol - lastVolSent) > 1) {
    dtostrf(currentVol, 4, 1, buf);
    tVolume.setText(buf);
    lastVolSent = currentVol;
    Serial.print("Current Volume: ");
    Serial.println(currentVol);
  }

  // ---  Total Volume Dashboard ---

  if (fabs(totalVolume - lastTotalVolSent) > 1) {
    dtostrf(totalVolume, 4, 1, buf);
    ttotVolume.setText(buf);
    lastTotalVolSent = totalVolume;
    Serial.print("Total Volume: ");
    Serial.println(totalVolume);
  }

  // --- Fuel ---
  if (fabs(solarVolumeUsed_L - lastFuelSent) > 1) {
    dtostrf(solarVolumeUsed_L, 4, 1, buf);
    tFuel.setText(buf);
    lastFuelSent = solarVolumeUsed_L;
    Serial.print("solarVolumeUsed_L: ");
    Serial.println(solarVolumeUsed_L);
  }

  // --- Total Konsumsi KG ---
  if (fabs(totalConsumedKg - lastTotalKgSent) > 1) {
    dtostrf(totalConsumedKg, 4, 1, buf);
    tTotWeight.setText(buf);
    lastTotalKgSent = totalConsumedKg;
  }
  if (remainingIsPreheat) {
    // Update hanya kalau berubah
    if (!lastPreheatSent) {
      tTime.setText("PRE-HEAT");
      lastPreheatSent = true;
    }
  } else {
    // kalau bukan preheat, tampil MM:SS dari remainingTimeSec
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
    } else {
      // optional: kalau belum pernah dapat data, kamu mau kosongin / tampil "--:--"
      // tTime.setText("--:--");
    }
  }
  // --- Fuel Level (slider + text) ---
  if ((int)currentFuelPct != lastFuelPctSent) {
    hFuel.setValue(currentFuelPct);

    char b[16];
    // opsi 1: tampil persen
    // snprintf(b, sizeof(b), "%u%%", currentFuelPct);

    // opsi 2: tampil liter sisa
    dtostrf(currentFuelLiter, 4, 1, b);
    tFuel.setText(b);

    lastFuelPctSent = (int)currentFuelPct;
  }


  // --- Total Batch ---
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
      case DUMPING_TRASH: tStat.setText("DUMP"); break;
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


// ================== CALLBACK IMPLEMENTATIONS ==================

void pollNextionSendme() {
  while (nexSerial.available()) {
    uint8_t b = (uint8_t)nexSerial.read();
    if (b == 0x66) { // sendme header
      while (!nexSerial.available()) {}
      uint8_t pid = (uint8_t)nexSerial.read();

      // buang 0xFF 0xFF 0xFF
      for (int i = 0; i < 3; i++) {
        while (!nexSerial.available()) {}
        (void)nexSerial.read();
      }

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

  // begitu masuk page3, paksa refresh
  logDirty = true;
  batchDirty = true;
  uiFlushIfActive();
}

void page3PopCallback(void *ptr) {
  logPageActive = false;
}


// --- DASHBOARD ---
void btAutoPopCallback(void *ptr) {
  uint32_t dual_state;
  btAuto.getValue(&dual_state);
  if (dual_state == 1) {
    Serial.println("CMD_AUTO_START");
  } else {
    Serial.println("CMD_AUTO_STOP");
  }
}

void bStopPopCallback(void *ptr) {
  Serial.println("CMD_EMERGENCY_STOP");
  // Serial.println("CMD_ONE_SHOT_AUTO");
}

void b0PopCallback(void *ptr) {
  Serial.println("CMD_ONE_SHOT_AUTO");
}

// --- MANUAL CONTROL ---
void applyManualSpeedPct(uint8_t pct) {
  pct = clampPctMan(pct);
  hManSpeed.setValue(pct);   // sync UI biar konsisten

  uint32_t realSpeed = pctToSpeed((uint8_t)manualMotorIdx, pct);
  Serial.print("SET_M_SPD:");
  Serial.println(realSpeed);
}

void applyManualStepPct(uint8_t pct) {
  pct = clampPctMan(pct);
  hManStep.setValue(pct);    // sync UI

  long realSteps = (long)pct * 100L;         // <==== manual step juga *100
  Serial.print("SET_M_STP:");
  Serial.println(realSteps);
}

void btManDirPopCallback(void *ptr) {
  uint32_t val;
  btManDir.getValue(&val);
  
  // val = 0 atau 1
  // Kirim ke CTRL untuk mengubah variabel 'isManualOpen'
  Serial.print("CMD_SET_MANUAL_DIR:");
  Serial.println(val);
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


// Helper Manual Move Wrapper
void reqManualMove(int motorID) {
  manualMotorIdx = motorID; 
  Serial.print("REQ_MOVE:");
  Serial.println(motorID);
}

void bManDoorPopCallback(void *ptr) {
  reqManualMove(0);
}  // Door
void bManPushPopCallback(void *ptr) {
  reqManualMove(1);
}  // Push
void bManBurnPopCallback(void *ptr) {
  reqManualMove(2);
}  // Burner
void bManBDoorPopCallback(void *ptr) {
  reqManualMove(3);
}  // BurnDoor
void bManAshPopCallback(void *ptr) {
  reqManualMove(4);
}  // Ash
void bManMainCPopCallback(void *ptr) {
  reqManualMove(5);
}  // MainConveyor

// --- MANUAL IGNITION (Pemantik) ---
void btManIgnPopCallback(void *ptr) {
  uint32_t val;
  btManIgn.getValue(&val);
  // Kirim: CMD_MAN_IGN:1 (ON) atau CMD_MAN_IGN:0 (OFF)
  Serial.print("CMD_MAN_IGN:");
  Serial.println(val);
}

void btManBurnPopCallback(void *ptr) {
  uint32_t val;
  btManBurn.getValue(&val);
  // Kirim: CMD_MAN_IGN:1 (ON) atau CMD_MAN_IGN:0 (OFF)
  Serial.print("CMD_MAN_BURN:");
  Serial.println(val);
}

// --- MANUAL FEEDER (Main Conveyor) ---
void btManFeedPopCallback(void *ptr) {
  uint32_t val;
  btManFeed.getValue(&val);
  // Kirim: CMD_MAN_FEED:1 atau 0
  Serial.print("CMD_MAN_FEED:");
  Serial.println(val);
}

// --- MANUAL WEIGHING (Motor/Relay Timbangan) ---
void btManWeighPopCallback(void *ptr) {
  uint32_t val;
  btManWeigh.getValue(&val);
  // Kirim: CMD_MAN_WEIGH:1 atau 0
  Serial.print("CMD_MAN_WEIGH:");
  Serial.println(val);
}

// --- SETTINGS ---
void bTempUpPopCallback(void *ptr) {
  tempSetpoint += 50.0;
  showTempSet();
  Serial.print("SET_TE:");
  Serial.println(tempSetpoint);
}
void bTempDownPopCallback(void *ptr) {
  tempSetpoint -= 50.0;
  if (tempSetpoint < 0) tempSetpoint = 0;
  showTempSet();
  Serial.print("SET_TE:");
  Serial.println(tempSetpoint);
}

void bTimeUpPopCallback(void *ptr) {
  burnerActiveTimeSec += 60;
  showTimeSet();
  Serial.print("SET_TI:");
  Serial.println(burnerActiveTimeSec);


}
void bTimeDownPopCallback(void *ptr) {
  if (burnerActiveTimeSec >= 60) burnerActiveTimeSec -= 60;
  showTimeSet();
  Serial.print("SET_TI:");
  Serial.println(burnerActiveTimeSec);
}

void bWeightUpPopCallback(void *ptr) {
  minBatchWeight += 1.0;
  showWeightSet();
  Serial.print("SET_W:");
  Serial.println(minBatchWeight);
}
void bWeightDownPopCallback(void *ptr) {
  if (minBatchWeight >= 1.0) minBatchWeight -= 1.0;
  showWeightSet();
  Serial.print("SET_W:");
  Serial.println(minBatchWeight);
}

void bSelDoorPopCallback(void *ptr) {
  selectMotor(0);
}
void bSelFeedPopCallback(void *ptr) {
  selectMotor(1);
}
void bSelBurnPopCallback(void *ptr) {
  selectMotor(2);
}
void bSelBDoorPopCallback(void *ptr) {
  selectMotor(3);
}
void bSelAshPopCallback(void *ptr) {
  selectMotor(4);
}
void bSelMainCPopCallback(void *ptr) {
  selectMotor(5);
}

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
  Serial.print("BLOW:");
  Serial.print(id);
  Serial.print(":");
  Serial.println(blowLevel[id]);
  updateBlowText(id);  // Update tampilan dulu biar terasa instan (Zero Latency)

  // Kirim perintah memaksa ke CTRL
  // Format: BLOW:<id>:<level> -> Contoh "BLOW:0:2"
  Serial.print("BLOW:");
  Serial.print(id);
  Serial.print(":");
  Serial.println(blowLevel[id]);
}

void bBlow1PopCallback(void *ptr) {
  updateBlowerState(0);
}
void bBlow2PopCallback(void *ptr) {
  updateBlowerState(1);
}

void bSavePopCallback(void *ptr) {
  // Kirim nilai setting terbaru ke CTRL sebelum save

  Serial.println("CMD_SAVE_SETTINGS");
  motorSettingsDirty = false;
}

void bZeroPopCallback(void *ptr) {
  Serial.println("CMD_TARE_SCALE");
}

void bSetPopCallback(void *ptr) {
  Serial.println("CMD_SET_SCALE");
}

void bWallPopCallback(void *ptr) {
  Serial.println("CMD_CAL_WALL");
}

void bSpeedPopCallback(void *ptr) {
  Serial.println("CMD_CAL_SPEED");
}


void nWeightSetPopCallback(void *ptr) {
  char buf[16] = { 0 };
  nWeightSet.getText(buf, sizeof(buf) - 1);
  minBatchWeight = atof(buf);
  showWeightSet();
}

void nTimeSetPopCallback(void *ptr) {
  char buf[16] = { 0 };
  nTimeSet.getText(buf, sizeof(buf) - 1);
  uint32_t menit = (uint32_t)atoi(buf);
  burnerActiveTimeSec = menit * 60;
  showTimeSet();
}

void nVolSetPopCallback(void *ptr) {
  char buf[16] = { 0 };
  nVolSet.getText(buf, sizeof(buf) - 1);
  uint32_t vol = (uint32_t)atoi(buf);
  volSetpoint = vol;
  showTimeSet();
}

String inputBuffer = "";

void ParseDataFromCTRL() {
  while (Serial.available() > 0) {
    char inChar = (char)Serial.read();
    
    if (inChar == '\n') {
      inputBuffer.trim(); // Hapus \r atau spasi
      
      // Proses Data (Pindahkan logika parsing ke sini)
      if (inputBuffer.length() > 0) {
        ProcessLine(inputBuffer);
      }
      
      inputBuffer = ""; // Reset buffer
    } else {
      inputBuffer += inChar;
    }
  }
}

void logBatchSummary(uint32_t batchId, float vol, float w, float fuel,
                     float tpeak, uint32_t durS, const String& result) {
  // UI
  uiAppendBatchSummary(batchId, vol, w, fuel, tpeak, durS, result);

  // SD
  logBatchSummarySD(batchId, vol, w, fuel, tpeak, durS, result);
}

// Buat fungsi terpisah agar rapi
void ProcessLine(String data) {
  if (data.startsWith("METRICS:")) {
 
      int c1 = data.indexOf(',');
      int c2 = data.indexOf(',', c1 + 1);
      int c3 = data.indexOf(',', c2 + 1);
      
      if (c1 > 0 && c2 > 0 && c3 > 0) {
        String tFuel = data.substring(8, c1); 
        String tVol = data.substring(c1 + 1, c2);
        String tWeight = data.substring(c2 + 1, c3);
        String tBatch = data.substring(c3 + 1);     

        solarVolumeUsed_L = tFuel.toFloat();
        totalVolume  = tVol.toFloat();
        totalConsumedKg  = tWeight.toFloat();
        totalBatchCount  = tBatch.toInt();
      }
    }
    // 1. Terima Data Telemetri Rutin (Format: DATA:TEMP,WEIGHT,STATE_ID)
    if (data.startsWith("DATA:")) {
      // Format: DATA:TEMP,WEIGHT,STATE,CUR_VOL
      int firstComma = data.indexOf(',');
      int secondComma = data.indexOf(',', firstComma + 1);
      int thirdComma = data.indexOf(',', secondComma + 1);

      if (firstComma > 0 && secondComma > 0 && thirdComma > 0) {
        String sTemp = data.substring(5, firstComma);
        String sWeight = data.substring(firstComma + 1, secondComma);
        String sState = data.substring(secondComma + 1, thirdComma);
        String sVol = data.substring(thirdComma + 1);

        currentTempC = sTemp.toFloat();
        currentWeightKg = sWeight.toFloat();
        
        // Update State
        int st = sState.toInt();
        if (st >= IDLE && st <= FAULT) {
          currentState = (SystemState)st;
        }

        // Update Volume Batch Saat Ini
        currentVol = sVol.toFloat();
      }
    }

    // 2. Terima Progress Bar (Format: PROG:50)
    if (data.startsWith("PROG:")) {
      int val = data.substring(5).toInt();
      lastProgValue = val;
      jStat.setValue(val);
    } else 
    if (data.startsWith("REM_TIME:")) {
      long val = data.substring(9).toInt();
      remainingTimeSec = val;
      remainingIsPreheat = false;
      remainingLastRxMs = millis();
    } 

    // PREHEAT (atau format lain sesuai CTRL-mu)
    if (data.startsWith("PREHEAT")) {
      remainingIsPreheat = true;
      remainingLastRxMs = millis();
    }
    // 3. Sinkronisasi Setting Awal
    if (data.startsWith("W_SET:")) {
      minBatchWeight = data.substring(6).toFloat();
      showWeightSet();
    } else if (data.startsWith("TE_SET:")) {  // temp setpoint
      tempSetpoint = data.substring(7).toFloat();
      showTempSet();
    } else if (data.startsWith("TI_SET:")) {  // time seconds
      burnerActiveTimeSec = data.substring(7).toInt();
      showTimeSet();
      
    } else if (data.startsWith("VB_SET:")) {  // m3 seconds
      volSetpoint = data.substring(7).toInt();
      showVolSet();
    }

    // 4. Sinkronisasi Posisi Slider Motor (M_SYNC:ID:SPD:STP)
    if (data.startsWith("M_SYNC:")) {
      // Gunakan nama variabel yang konsisten biar tidak bingung
      int firstColon = data.indexOf(':');
      int secondColon = data.indexOf(':', firstColon + 1);
      int thirdColon = data.indexOf(':', secondColon + 1);

      // Pastikan data valid (titik dua ditemukan semua)
      if (firstColon > 0 && secondColon > 0 && thirdColon > 0) {
        int id = data.substring(firstColon + 1, secondColon).toInt();
        long spd = data.substring(secondColon + 1, thirdColon).toInt();
        long stp = data.substring(thirdColon + 1).toInt();

        if (id >= 0 && id < 6) {
          motorConfigs[id].speed = spd;
          motorConfigs[id].steps = stp;
            if (id == selectedMotorIdx) {
              updateMotorInfoTexts();
            }
        }
      }
    }

    // 5. Sinkronisasi Status Blower (B_SYNC:ID:LVL)
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
    // LOG:<level>,<tag>,<msg>
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
    // BATCH_SUM:<id>,<vol>,<w>,<fuel>,<tpeak>,<durS>,<result>
    String x = data.substring(10);
    int p[6];
    p[0] = x.indexOf(',');
    for(int i=1;i<6;i++) p[i] = x.indexOf(',', p[i-1]+1);

    if (p[0]>0 && p[5]>0) {
      uint32_t bid = x.substring(0, p[0]).toInt();
      float vol = x.substring(p[0]+1, p[1]).toFloat();
      float w   = x.substring(p[1]+1, p[2]).toFloat();
      float fuel= x.substring(p[2]+1, p[3]).toFloat();
      float tp  = x.substring(p[3]+1, p[4]).toFloat();
      uint32_t dur = x.substring(p[4]+1, p[5]).toInt();
      String res = x.substring(p[5]+1);

      logBatchSummary(bid, vol, w, fuel, tp, dur, res);
    }
  }
  if (data.startsWith("FUEL:")) {
    // Format: FUEL:<pct>,<liter>
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




bool sdInit() {
  // Init SPI VSPI untuk SD
  sdSPI.begin(SD_SCK, SD_MISO, SD_MOSI, SD_CS);

  if (!SD.begin(SD_CS, sdSPI, 20000000)) {  // 20MHz, bisa turunkan kalau SD rewel
    sdOK = false;
    Serial.println("SD init failed!");
    return false;
  }

  sdOK = true;
  Serial.println("SD init OK");
  return true;
}

void sdStartSession(bool forceNew) {
  if (!sdOK) return;

  prefs.begin("hmi", false);

  if (forceNew) {
    sessionId = prefs.getUInt("sid", 0) + 1;
    prefs.putUInt("sid", sessionId);
  } else {
    // pakai yang terakhir kalau ada
    sessionId = prefs.getUInt("sid", 0);
    if (sessionId == 0) {
      sessionId = 1;
      prefs.putUInt("sid", sessionId);
    }
  }

  prefs.end();

  char dir[16];
  snprintf(dir, sizeof(dir), "/S%06lu", (unsigned long)sessionId);
  sessionBase = String(dir);

  if (!SD.exists(sessionBase)) {
    if (!SD.mkdir(sessionBase)) {
      Serial.println("SD mkdir session failed!");
      return;
    }
  }

  // Tutup file lama kalau ada
  sdCloseFiles();

  // Buka file append
  fEvents  = SD.open(sessionBase + "/events.csv", FILE_APPEND);
  fBatches = SD.open(sessionBase + "/batches.csv", FILE_APPEND);

  if (!fEvents || !fBatches) {
    Serial.println("SD open log files failed!");
    sdCloseFiles();
    return;
  }

  // Header jika file baru kosong
  if (fEvents.size() == 0) {
    fEvents.println("ms,level,tag,message");
    fEvents.flush();
  }
  if (fBatches.size() == 0) {
    fBatches.println("ms,batchId,vol,w,fuel,tpeak,durS,result");
    fBatches.flush();
  }

  Serial.print("SD session: ");
  Serial.println(sessionBase);
}

void sdCloseFiles() {
  if (fEvents)  { fEvents.flush();  fEvents.close(); }
  if (fBatches) { fBatches.flush(); fBatches.close(); }
  if (fTrend)   { fTrend.flush();   fTrend.close(); } // kalau kamu pakai nanti
}

void sdWriteLine(File &f, const String &line) {
  if (!sdOK) return;
  if (!f) return;

  f.println(line);

  // flush periodik biar aman (nggak tiap line juga boleh)
  static uint32_t lastFlushMs = 0;
  uint32_t now = millis();
  if (now - lastFlushMs > 1000) {   // flush tiap 1 detik
    f.flush();
    lastFlushMs = now;
  }
}

void logEventSD(const char* lvl, const char* tag, const String& msg) {
  if (!sdOK || !fEvents) return;

  // CSV aman: bungkus message pakai quotes, dan escape quotes di dalam msg
  String m = msg;
  m.replace("\"", "\"\"");

  String line;
  line.reserve(32 + m.length());
  line += String(millis());
  line += ",";
  line += lvl;
  line += ",";
  line += tag;
  line += ",\"";
  line += m;
  line += "\"";

  sdWriteLine(fEvents, line);
}

void logBatchSummarySD(uint32_t batchId, float vol, float w, float fuel,
                       float tpeak, uint32_t durS, const String& result) {
  if (!sdOK || !fBatches) return;

  String r = result;
  r.replace("\"", "\"\"");

  char buf[160];
  snprintf(buf, sizeof(buf),
           "%lu,%lu,%.3f,%.3f,%.3f,%.1f,%lu,\"%s\"",
           (unsigned long)millis(),
           (unsigned long)batchId,
           vol, w, fuel, tpeak,
           (unsigned long)durS,
           r.c_str());

  sdWriteLine(fBatches, String(buf));
}


void logEvent(const char* lvl, const char* tag, const String& msg) {
  // UI
  char buf[160];
  snprintf(buf, sizeof(buf), "[%s][%s] %s", lvl, tag, msg.c_str());
  uiAppendLogBuffered(String(buf));
  uiFlushIfActive();

  // SD
  logEventSD(lvl, tag, msg);
}




// ================== SETUP & LOOP ==================
void setup() {
  Serial.begin(9600, SERIAL_8N1, 16, 17);  // Debug serial & komunikasi ke CTRL
  nexSerial.begin(9600, SERIAL_8N1, 25, 26);
  nexInit();
  // enable line break di ScrollText (wajib dari editor juga)
  stLog.setFont(0);    // opsional, kalau perlu
  stBatch.setFont(0);  // opsional
  sdInit();
  sdStartSession(true);   // true = setiap boot bikin session baru
  // kalau mau lanjut session terakhir: sdStartSession(false);

  // Clear tampilan + buffer
  logBuf0 = "";
  logBuf1 = "";
  logDirty = true;
  batchDirty = true;

  delay(1500);  // Tunggu CTRL siap
  Serial.println("CMD_REQ_SYNC");
  // Attach Callbacks
  // page0.attachPop(page0PopCallback, &page0);
  // page1.attachPop(page1PopCallback, &page1);
  // page2.attachPop(page2PopCallback, &page2);
  // b1.attachPop(b1PopCallback, &b1);

  // Settings
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

  // Motor Selectors
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

  // Blower
  bBlow1.attachPop(bBlow1PopCallback, &bBlow1);
  bBlow2.attachPop(bBlow2PopCallback, &bBlow2);

  // Dashboard
  btAuto.attachPop(btAutoPopCallback, &btAuto);
  bStop.attachPop(bStopPopCallback, &bStop);
  bManStop.attachPop(bStopPopCallback, &bManStop);
  bsettingStop.attachPop(bStopPopCallback, &bsettingStop);

  // Manual
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
  btManFeed.attachPop(btManFeedPopCallback, &btManFeed);
  btManWeigh.attachPop(btManWeighPopCallback, &btManWeigh);
  b0.attachPop(b0PopCallback, &b0);


  bman1.attachPop(bManSpeedDownPopCallback, &bman1);
  bman2.attachPop(bManSpeedUpPopCallback, &bman2);
  bman3.attachPop(bManStepUpPopCallback, &bman3);
  bman4.attachPop(bManStepDownPopCallback, &bman4);

  page3.attachPush(page3PushCallback, &page3);
  page3.attachPop(page3PopCallback, &page3);
}

void loop() {

  nexLoop(nex_listen_list);

  ParseDataFromCTRL();

  updateHMI();
  uiFlushIfActive();   
  pollNextionSendme(); 

}