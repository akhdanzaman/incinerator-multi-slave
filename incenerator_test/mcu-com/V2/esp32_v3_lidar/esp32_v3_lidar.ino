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
#include <math.h>
struct LidarPoint {
  float distanceMm;
  float angleDeg;
  uint8_t quality;
  bool startBit;
};
// ==============================
// LIDAR / VOLUME BLOCK (REPLACED)
// ==============================
// Mengganti implementasi LIDAR sebelumnya (library RPLidar) dengan parser UART raw
// + strategi drain-buffer + deteksi sweep via wrap angle + auto background calibration.
// NOTE: Hanya blok ini yang berubah. Kode lain biarkan sama.

#define USE_LIDAR 0  // (LIDAR-related)

Preferences prefs;

SPIClass sdSPI(VSPI);

bool sdOK = false;
uint32_t sessionId = 0;
String sessionBase;  // contoh: "/S000123"
File fEvents, fBatches, fTrend;

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

// =========================
// CONFIG
// =========================
static const int LIDAR_RX = 12;
static const int LIDAR_TX = 13;
static const uint32_t LIDAR_BAUD = 460800;

static const int SPAN = 360;
static const float HALF_WALL_MM = 250.0f;
static const float BELT_SPEED_CMPS = 30.0f;
static const float HEIGHT_DEADBAND_MM = 10.0f;
static const int MIN_HITS_PER_SWEEP = 12;
static const float EMA_ALPHA = 0.25f;
static const float MIN_SLICE_AREA_CM2 = 2.0f;
static const float DEG2RAD_F = 0.01745329251f;

// Jumlah sweep untuk kalibrasi (misal 10Hz x 2 detik = 20 sweep)
static const int CALIBRATION_FRAMES = 20;

// =========================
// STORAGE & HANDLES
// =========================
static uint16_t scanA[SPAN];
static uint16_t scanB[SPAN];
static uint16_t *scanWrite = scanA;
static uint16_t *scanRead = scanB;

// Background Storage
static uint16_t bg[SPAN];
static uint32_t bgSum[SPAN];  // Buffer penjumlahan untuk rata-rata
static int bgFrameCount = 0;
static bool hasBg = false;

static portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
static TaskHandle_t taskLidarHandle = NULL;
static TaskHandle_t taskPrintHandle = NULL;

// Volume vars
static volatile float vBatch = 0.0f;
static volatile float vTotal = 0.0f;
static float sliceAreaEma = 0.0f;

// Debug vars
static volatile uint32_t dbg_rawBytes = 0;
static volatile uint32_t dbg_framesOK = 0;
static volatile uint32_t dbg_framesBad = 0;
static volatile uint32_t dbg_sweeps = 0;
static volatile uint32_t dbg_lastFrameMs = 0;
static volatile uint32_t dbg_lastSweepMs = 0;

// =========================
// RAW UART PARSER STRUCT
// =========================


// =========================
// FILTER / VOLUME PARAMS
// =========================
#define DEG_TO_RAD 0.01745329251f


// Volume model
static float conveyorSpeed_cmps = BELT_SPEED_CMPS;
static float volumeScaleFactor = 1.0f;
static bool speedCalibrated = true;   // fixed speed
static bool volumeCalibrated = true;  // scaleFactor default 1

static volatile float esp_currentBatchVolume = 0.0f;  // cm^3
static volatile float esp_totalVolume = 0.0f;         // cm^3

static TaskHandle_t taskLidarProcHandle = NULL;

// Untuk kirim ke Mega periodik
static uint32_t lastLidarTxMs = 0;

// =========================
// DEBUG (optional)
// =========================


static volatile uint16_t dbg_angMin_x10 = 9999;
static volatile uint16_t dbg_angMax_x10 = 0;
static volatile uint32_t dbg_startHits = 0;

/// =========================
// HELPERS
// =========================
static inline void clearBuf(uint16_t *buf) {
  memset(buf, 0, SPAN * sizeof(uint16_t));
}

static inline bool inBeltWindow(int deg, float distMm) {
  float th = (float)deg * DEG2RAD_F;
  float x = distMm * sinf(th);
  return (fabsf(x) <= HALF_WALL_MM);
}

static float computeSliceAreaCm2(const uint16_t *scan) {
  if (!hasBg) return 0.0f;
  float area_mm2 = 0.0f;
  int hits = 0;
  for (int deg = 0; deg < 360; deg++) {
    uint16_t b = bg[deg];
    uint16_t s = scan[deg];
    if (b == 0 || s == 0) continue;

    // Filter window
    float bgMm = (float)b;
    float scMm = (float)s;
    if (!inBeltWindow(deg, bgMm) && !inBeltWindow(deg, scMm)) continue;

    // Filter tinggi & noise
    float h = bgMm - scMm;
    if (h <= HEIGHT_DEADBAND_MM) continue;
    if (scMm < 80.0f) continue;  // Terlalu dekat lidar (noise internal)

    // Integrasi: Area = h * arc_length
    // arc_length approx = radius * d_theta
    // Kita gunakan radius scan (scMm) untuk arc width
    area_mm2 += (h * scMm * DEG2RAD_F);
    hits++;
  }
  if (hits < MIN_HITS_PER_SWEEP) return 0.0f;
  return area_mm2 / 100.0f;  // mm2 -> cm2
}
// =========================
// PARSER
// =========================
static inline bool headerLooksValid(uint8_t b0) {
  bool b1 = (b0 >> 1) & 0x01;
  bool b0v = b0 & 0x01;
  return (b1 == (!b0v));
}

static inline bool decode5Bytes(const uint8_t buf[5], LidarPoint &out) {
  out.quality = buf[0] >> 2;
  // Bit 0 = !S, Bit 1 = S. Start bit ada di bit 1.
  out.startBit = (buf[0] & 0x01) != 0;   // START FLAG itu bit0

  out.angleDeg = ((buf[1] >> 1) | (buf[2] << 7)) / 64.0f;
  out.distanceMm = (float)(buf[3] | (buf[4] << 8)) / 4.0f;

  if (out.angleDeg >= 360.0f) return false;
  return true;
}
// =========================
// LIDAR TASK (READ + PROCESS)
// =========================
// NOTE:
// - Nama fungsi taskLidarRead & taskLidarProcess dipertahankan agar setup() tidak berubah.
// - taskLidarRead sekarang menangani: parsing UART, deteksi sweep, swap buffer,
//   auto background calibration, dan integrasi volume.
// - taskLidarProcess dibuat idle (menunggu notify) supaya kompatibel tanpa beban.

void taskLidar(void *pv) {
  clearBuf(scanA);
  clearBuf(scanB);
  memset(bgSum, 0, SPAN * sizeof(uint32_t));

  Serial1.setRxBufferSize(8192);
  Serial1.begin(LIDAR_BAUD, SERIAL_8N1, LIDAR_RX, LIDAR_TX);

  // Tunggu Lidar Boot
  vTaskDelay(pdMS_TO_TICKS(1000));

  // Reset Command
  const uint8_t stopCmd[] = { 0xA5, 0x25 };
  const uint8_t startCmd[] = { 0xA5, 0x20 };
  Serial1.write(stopCmd, 2);
  vTaskDelay(pdMS_TO_TICKS(100));
  while (Serial1.available()) Serial1.read();
  Serial1.write(startCmd, 2);

  uint32_t lastVolMs = millis();
  uint32_t bootMs = millis();
  float prevAngle = -1.0f;

  dbg_lastFrameMs = millis();
  dbg_lastSweepMs = millis();

  uint8_t buf[5];

  for (;;) {
    // === 1. DRAIN BUFFER STRATEGY ===
    // Jika data < 5 byte, tidur sebentar biar CPU adem
    if (Serial1.available() < 5) {
      vTaskDelay(pdMS_TO_TICKS(1));
      continue;
    }

    // Selama data masih ada, proses terus tanpa delay!
    while (Serial1.available() >= 5) {
      uint8_t b0 = Serial1.peek();

      if (headerLooksValid(b0)) {
        Serial1.readBytes(buf, 5);
        dbg_rawBytes += 5;

        LidarPoint p;
        if (decode5Bytes(buf, p)) {
          dbg_framesOK++;
          dbg_lastFrameMs = millis();

          // === 2. DETEKSI SWEEP ===
          bool newSweep = p.startBit;
          if (prevAngle >= 0.0f) {
            if (prevAngle > 300.0f && p.angleDeg < 60.0f) newSweep = true;
          }
          prevAngle = p.angleDeg;

          // === 3. PROSES SWEEP BARU ===
          if (newSweep) {
            dbg_sweeps++;
            dbg_lastSweepMs = millis();

            // Swap Buffer
            taskENTER_CRITICAL(&mux);
            uint16_t *oldRead = scanRead;
            scanRead = scanWrite;
            scanWrite = oldRead;
            taskEXIT_CRITICAL(&mux);

            clearBuf(scanWrite);  // Reset buffer tulis

            // --- LOGIKA KALIBRASI & VOLUME (STATE MACHINE) ---

            // Jika belum punya BG dan sudah lewat waktu boot (3s)
            if (!hasBg) {
              // MODE KALIBRASI: Akumulasi data
              if (bgFrameCount < CALIBRATION_FRAMES) {
                // Ambil data dari scanRead yg baru saja di-swap
                for (int i = 0; i < SPAN; i++) {
                  if (scanRead[i] > 0) bgSum[i] += scanRead[i];
                }
                bgFrameCount++;
                if (bgFrameCount % 5 == 0) Serial.printf("[CAL] Frame %d/%d\n", bgFrameCount, CALIBRATION_FRAMES);
              } else {
                // FINALIZE KALIBRASI
                Serial.println("[CAL] Finalizing Background...");
                for (int i = 0; i < SPAN; i++) {
                  // Jika data terkumpul, rata-rata. Jika tidak (0), tetap 0.
                  // Kita bagi dengan CALIBRATION_FRAMES, tapi idealnya bagi dengan hit count per index.
                  // Untuk simplifikasi, anggap solid wall selalu kena hit.
                  bg[i] = (uint16_t)(bgSum[i] / CALIBRATION_FRAMES);
                }
                hasBg = true;
                Serial.println("[CAL] Background Done!");
              }
            }
            // MODE RUNNING
            else if (hasBg) {
              float areaRaw = computeSliceAreaCm2(scanRead);

              // EMA Filter
              sliceAreaEma = (EMA_ALPHA * areaRaw) + ((1.0f - EMA_ALPHA) * sliceAreaEma);

              uint32_t now = millis();
              uint32_t dtMs = now - lastVolMs;
              if (dtMs > 200) dtMs = 200;
              lastVolMs = now;

              if (sliceAreaEma > MIN_SLICE_AREA_CM2) {
                float dV = sliceAreaEma * BELT_SPEED_CMPS * (dtMs / 1000.0f);
                vTotal += dV;
                vBatch += dV;
                esp_currentBatchVolume = vBatch;
                esp_totalVolume = vTotal;
              }
            }
          }

          // === 4. TULIS DATA KE BUCKET (SCAN WRITE) ===
          if (p.distanceMm > 0) {
            int deg = (int)(p.angleDeg + 0.5f);
            if (deg >= 0 && deg < SPAN) {
              uint16_t mm = (uint16_t)p.distanceMm;
              // Filter: ambil jarak terdekat jika ada tumpang tindih sudut
              if (scanWrite[deg] == 0 || mm < scanWrite[deg]) {
                scanWrite[deg] = mm;
              }
            }
          }

        } else {
          dbg_framesBad++;  // Checksum/logic fail
        }
      } else {
        Serial1.read();  // Header salah, geser 1 byte
        dbg_rawBytes++;
        dbg_framesBad++;
      }
    }
  }
}




void sendLidarToMegaPeriodik() {
  uint32_t nowMs = millis();
  if (nowMs - lastLidarTxMs < 100) return;  // 10 Hz
  lastLidarTxMs = nowMs;

  float b = esp_currentBatchVolume;
  float t = esp_totalVolume;

  // Tetap pakai format kamu biar MCU-CTRL tinggal parse
  Serial.print("LIDAR:");
  Serial.print(b, 3);
  Serial.print(",");
  Serial.println(t, 3);
}

void sendLidarDebugToMegaPeriodik() {
  static uint32_t lastDbgMs = 0;
  uint32_t now = millis();
  if (now - lastDbgMs < 1000) return;  // 1 Hz biar nggak spam
  lastDbgMs = now;

  // snapshot ringan
  uint32_t rawB = dbg_rawBytes;
  uint32_t okF = dbg_framesOK;
  uint32_t badF = dbg_framesBad;
  uint32_t sw = dbg_sweeps;

  uint32_t ageFrame = now - dbg_lastFrameMs;
  uint32_t ageSweep = now - dbg_lastSweepMs;

  float area = sliceAreaEma;
  float bVol = esp_currentBatchVolume;
  float tVol = esp_totalVolume;

  // kirim ke Mega lewat link ESP->Mega (Serial pada ESP32)
  // Serial.print("DBG_LIDAR:");
  // Serial.print("rawB=");
  // Serial.print(rawB);
  // Serial.print(",ok=");
  // Serial.print(okF);
  // Serial.print(",bad=");
  // Serial.print(badF);
  // Serial.print(",sweep=");
  // Serial.print(sw);
  // Serial.print(",ageFms=");
  // Serial.print(ageFrame);
  // Serial.print(",ageSms=");
  // Serial.print(ageSweep);

  // Serial.print(",bgN=");
  // Serial.print(bgFrameCount);
  // Serial.print(",areaEma=");
  // Serial.print(area, 2);
  // Serial.print(",bVol=");
  // Serial.print(bVol, 3);
  // Serial.print(",tVol=");
  // Serial.print(tVol, 3);
  // Serial.println();
  // Serial.print(",aMin=");
  // Serial.print(dbg_angMin_x10 / 10.0f, 1);
  // Serial.print(",aMax=");
  // Serial.print(dbg_angMax_x10 / 10.0f, 1);
  // Serial.print(",sBit=");
  // Serial.print(dbg_startHits);
  dbg_angMin_x10 = 9999;
  dbg_angMax_x10 = 0;
  dbg_startHits = 0;
}


static bool lidarInitAndStart() {
  // Inisialisasi UART sudah dilakukan di taskLidarRead.
  // Fungsi ini dipanggil dari setup(), jadi kita pertahankan agar flow tidak berubah.
  return true;
}

// ================================
// END LIDAR / VOLUME BLOCK REPLACED
// ================================



// === SD Pins (ubah sesuai wiring kamu) ===
static const int SD_CS = 22;
static const int SD_SCK = 18;
static const int SD_MISO = 19;
static const int SD_MOSI = 23;


// HardwareSerial Serial(1);

// [Nextion Serial Define]


#define nexSerial Serial2  // dipakai di library Nextion
#include <Nextion.h>

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
int manualMotorIdx = 0;  // motor yang akan digerakkan di manual

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




static inline uint8_t clampPct100(long v) {
  if (v < 1) return 1;
  if (v > 100) return 100;
  return (uint8_t)v;
}
static inline uint8_t clampPctMan(long v) {
  return clampPct100(v);
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
  uint32_t minS = MOTOR_MIN_SPD[motorId];  // kalau kamu pakai min=0, tetap aman

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
NexButton bSpeedUp = NexButton(2, 38, "b1");    // dulu: bSpeedUp
NexButton bSpeedDown = NexButton(2, 40, "b2");  // dulu: bSpeedDown

NexButton bStepUp = NexButton(2, 37, "b4");    // dulu: bStepUp
NexButton bStepDown = NexButton(2, 39, "b0");  // dulu: bStepDown

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
NexPage page3 = NexPage(3, 0, "page3");  // <=== tambah ini

NexScrolltext stLog = NexScrolltext(3, 5, "slt0");    // contoh page3 id=3, comp id=1
NexScrolltext stBatch = NexScrolltext(3, 6, "slt1");  // contoh page4 id=4, comp id=1



// --- LISTEN LIST ---
NexTouch *nex_listen_list[] = {
  &page0, &page1, &page2, &page3,
  &btAuto, &bStop, &b1,
  &nWeightSet, &nTimeSet, &hFuel,
  &bTempUp, &bTempDown, &bTimeUp, &bTimeDown, &bWeightUp, &bWeightDown,
  &bSelDoor, &bSelBurn, &bSelFeed, &bSelBDoor, &bSelMainC, &bSelAsh,
  &hSpeed, &hStep, &bStepUp, &hSpeed, &hStep,
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

static const size_t LOG_MAX_CHARS = 900;
static const size_t BATCH_MAX_CHARS = 900;

bool logPageActive = false;  // true kalau page3 aktif
bool logDirty = false;
bool batchDirty = false;

static inline void trimFrontToLimit(String &s, size_t maxChars) {
  if (s.length() <= maxChars) return;

  int cut = s.length() - (int)maxChars;
  int pos = s.indexOf('\r', cut);  // IMPORTANT: '\r' bukan "\\r"
  if (pos < 0) pos = cut;
  s.remove(0, pos + 1);
}

static inline void appendLine(String &buf, const String &line, size_t maxChars) {
  buf += line;
  buf += '\r';  // newline Nextion
  trimFrontToLimit(buf, maxChars);
}
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
                          float tpeak, uint32_t durS, const String &result) {
  char buf[128];
  snprintf(buf, sizeof(buf),
           "#%lu  V=%.1f  W=%.1f  Fuel=%.1f  Tp=%.0f  %lus  %s",
           (unsigned long)batchId, vol, w, fuel, tpeak, (unsigned long)durS, result.c_str());

  uiAppendBatchBuffered(String(buf));
  uiFlushIfActive();  // optional
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
  sprintf(buf, "%d cm3", (int)volSetpoint);
  nVolSet.setText(buf);
}



void updateMotorInfoTexts() {
  // ambil dari config (real), lalu tampilkan ke UI sebagai persen (speed) & persen (step)
  uint8_t spdPct = speedToPct((uint8_t)selectedMotorIdx, (uint32_t)motorConfigs[selectedMotorIdx].speed);
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

  long realSteps = (long)pct * 100L;  // sesuai permintaan: step yang dipakai = pct*100
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
    if (b == 0x66) {  // sendme header
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
  hManSpeed.setValue(pct);  // sync UI biar konsisten

  uint32_t realSpeed = pctToSpeed((uint8_t)manualMotorIdx, pct);
  Serial.print("SET_M_SPD:");
  Serial.println(realSpeed);
}

void applyManualStepPct(uint8_t pct) {
  pct = clampPctMan(pct);
  hManStep.setValue(pct);  // sync UI

  long realSteps = (long)pct * 100L;  // <==== manual step juga *100
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
  Serial.println("TESTRELAY");
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
  logEvent("INFO", "LIDAR", "Calibrating background...");
  // lidarStartBackgroundCal(2500);
  logEvent("INFO", "LIDAR", "Background OK");
  // optional: tetap kasih tahu Mega biar dia log juga
  Serial.println("CMD_CAL_WALL");
}

void bSpeedPopCallback(void *ptr) {
  // Kalau kamu punya flow kalibrasi speed/volume yang rumit (obj 15x15x5),
  // implementasinya pindah ke ESP32 juga. Untuk sementara:
  logEvent("WARN", "LIDAR", "Speed cal not implemented on ESP32 yet");
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
  volSetpoint = (float)vol;

  showVolSet();  // ✅ bener

  // ✅ kirim ke CTRL supaya minBatchVolume update
  Serial.print("SET_VB:");
  Serial.println(volSetpoint, 0);
}

String inputBuffer = "";

void ParseDataFromCTRL() {
  while (Serial.available() > 0) {
    char inChar = (char)Serial.read();

    if (inChar == '\n') {
      inputBuffer.trim();  // Hapus \r atau spasi

      // Proses Data (Pindahkan logika parsing ke sini)
      if (inputBuffer.length() > 0) {
        ProcessLine(inputBuffer);
      }

      inputBuffer = "";  // Reset buffer
    } else {
      inputBuffer += inChar;
    }
  }
}

void logBatchSummary(uint32_t batchId, float vol, float w, float fuel,
                     float tpeak, uint32_t durS, const String &result) {
  // UI
  uiAppendBatchSummary(batchId, vol, w, fuel, tpeak, durS, result);

  // SD
  logBatchSummarySD(batchId, vol, w, fuel, tpeak, durS, result);
}

// Buat fungsi terpisah agar rapi
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
  if (data.startsWith("LIDAR_CFG:")) {
    String x = data.substring(10);
    int c1 = x.indexOf(',');
    int c2 = x.indexOf(',', c1 + 1);
    if (c1 > 0 && c2 > 0) {
      conveyorSpeed_cmps = x.substring(0, c1).toFloat();
      volumeScaleFactor = x.substring(c1 + 1, c2).toFloat();
      uint8_t flags = (uint8_t)x.substring(c2 + 1).toInt();

      speedCalibrated = (flags & 0x01) != 0;
      volumeCalibrated = (flags & 0x02) != 0;

      logEvent("INFO", "LIDAR", "CFG updated from CTRL");
    }
  }
  if (data == "CMD_LIDAR_RESET_BATCH") {
    esp_currentBatchVolume = 0.0f;
    logEvent("INFO", "LIDAR", "Batch volume reset by CTRL");
  }
  // 1. Terima Data Telemetri Rutin (Format: DATA:TEMP,WEIGHT,STATE_ID)
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


  // 2. Terima Progress Bar (Format: PROG:50)
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

  } else if (data.startsWith("VB_SET:")) {  // cm3
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
  fEvents = SD.open(sessionBase + "/events.csv", FILE_APPEND);
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
  if (fEvents) {
    fEvents.flush();
    fEvents.close();
  }
  if (fBatches) {
    fBatches.flush();
    fBatches.close();
  }
  if (fTrend) {
    fTrend.flush();
    fTrend.close();
  }  // kalau kamu pakai nanti
}

void sdWriteLine(File &f, const String &line) {
  if (!sdOK) return;
  if (!f) return;

  f.println(line);

  // flush periodik biar aman (nggak tiap line juga boleh)
  static uint32_t lastFlushMs = 0;
  uint32_t now = millis();
  if (now - lastFlushMs > 1000) {  // flush tiap 1 detik
    f.flush();
    lastFlushMs = now;
  }
}

void logEventSD(const char *lvl, const char *tag, const String &msg) {
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
                       float tpeak, uint32_t durS, const String &result) {
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


void logEvent(const char *lvl, const char *tag, const String &msg) {
  // UI
  char buf[160];
  snprintf(buf, sizeof(buf), "[%s][%s] %s", lvl, tag, msg.c_str());
  uiAppendLogBuffered(String(buf));
  uiFlushIfActive();

  // SD
  logEventSD(lvl, tag, msg);
}


void taskPrint(void *pv) {
  uint32_t last_ok = 0, last_bad = 0, last_sw = 0;

  for (;;) {
    vTaskDelay(pdMS_TO_TICKS(1000));
    uint32_t now = millis();

    Serial.printf(" EMA=%.2f cm2 | Vol=%.2f cm3 | FPS=%lu | Swp=%lu | FreeHeap=%u\n ",
                  sliceAreaEma, vTotal,
                  (dbg_framesOK - last_ok),
                  (dbg_sweeps - last_sw),
                  esp_get_free_heap_size());

    last_ok = dbg_framesOK;
    last_sw = dbg_sweeps;

    if (now - dbg_lastFrameMs > 2000) Serial.println("!! NO FRAMES RECEIVED !!");
  }
}


// ================== SETUP & LOOP ==================
void setup() {
  Serial.begin(9600, SERIAL_8N1, 16, 17);  // Debug serial & komunikasi ke CTRL
  nexSerial.begin(9600, SERIAL_8N1, 25, 26);
  nexInit();
  // enable line break di ScrollText (wajib dari editor juga)
  // stLog.setFont(0);    // opsional, kalau perlu
  // stBatch.setFont(0);  // opsional
  // sdInit();
  // sdStartSession(true);   // true = setiap boot bikin session baru
  // // kalau mau lanjut session terakhir: sdStartSession(false);

#if USE_LIDAR
  // ===== LIDAR START (UPDATED) =====
  (void)lidarInitAndStart();
  xTaskCreatePinnedToCore(taskPrint, "Print", 4096, NULL, 1, &taskPrintHandle, 0);

  // Task LIDAR read (core 0) dan process (core 1) biar UI/Nextion nggak kecekik.
  xTaskCreatePinnedToCore(taskLidar, "Lidar", 16384, NULL, 5, &taskLidarHandle, 1);
  // ===== END LIDAR START =====
#endif

  // Clear tampilan + buffer
  logBuf0 = "";
  logBuf1 = "";
  logDirty = true;
  batchDirty = true;

  delay(1500);  // Tunggu CTRL siap
  Serial.println("CMD_REQ_SYNC");

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
  btManBurn.attachPop(btManBurnPopCallback, &btManBurn);
  btManWeigh.attachPop(btManWeighPopCallback, &btManWeigh);
  b0.attachPop(b0PopCallback, &b0);


  bman1.attachPop(bManSpeedDownPopCallback, &bman1);
  bman2.attachPop(bManSpeedUpPopCallback, &bman2);
  bman3.attachPop(bManStepUpPopCallback, &bman3);
  bman4.attachPop(bManStepDownPopCallback, &bman4);

  page3.attachPush(page3PushCallback, &page3);
  page3.attachPop(page3PopCallback, &page3);
  // ===== TEST KONEKSI AWAL =====
  Serial.println("ESP32 BOOT OK");
  // Serial2.println("ESP32_BOOT");   // ini dikirim ke CTRL
}

void loop() {
  nexLoop(nex_listen_list);

  ParseDataFromCTRL();

  updateHMI();

#if USE_LIDAR
  sendLidarToMegaPeriodik();
  sendLidarDebugToMegaPeriodik();
#endif
}