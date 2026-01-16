#include <Arduino.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// =========================
// DATA STRUCT
// =========================
struct LidarPoint {
  float distanceMm;
  float angleDeg;
  uint8_t quality;
  bool startBit;
};

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
static uint16_t* scanWrite = scanA;
static uint16_t* scanRead  = scanB;

// Background Storage
static uint16_t bg[SPAN];
static uint32_t bgSum[SPAN]; // Buffer penjumlahan untuk rata-rata
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
// HELPERS
// =========================
static inline void clearBuf(uint16_t* buf) {
  memset(buf, 0, SPAN * sizeof(uint16_t));
}

static inline bool inBeltWindow(int deg, float distMm) {
  float th = (float)deg * DEG2RAD_F;
  float x = distMm * sinf(th);
  return (fabsf(x) <= HALF_WALL_MM);
}

static float computeSliceAreaCm2(const uint16_t* scan) {
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
    if (scMm < 80.0f) continue; // Terlalu dekat lidar (noise internal)

    // Integrasi: Area = h * arc_length
    // arc_length approx = radius * d_theta
    // Kita gunakan radius scan (scMm) untuk arc width
    area_mm2 += (h * scMm * DEG2RAD_F);
    hits++;
  }
  if (hits < MIN_HITS_PER_SWEEP) return 0.0f;
  return area_mm2 / 100.0f; // mm2 -> cm2
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
  bool s  = (buf[0] & 0x01) != 0;   // start
  bool ns = (buf[0] & 0x02) != 0;   // inverse start
  if (s == ns) return false;        // invalid

  if ((buf[1] & 0x01) == 0) return false; // angle check bit must be 1

  out.quality  = buf[0] >> 2;
  out.startBit = s;

  out.angleDeg = ((buf[1] >> 1) | (buf[2] << 7)) / 64.0f;
  if (out.angleDeg < 0.0f || out.angleDeg >= 360.0f) return false;

  out.distanceMm = (float)(buf[3] | (buf[4] << 8)) / 4.0f;
  if (out.distanceMm <= 0.0f || out.distanceMm > 16000.0f) return false;

  return true;
}

// =========================
// TASK: LIDAR (CORE 1)
// =========================
void taskLidar(void *pv) {
  clearBuf(scanA);
  clearBuf(scanB);
  memset(bgSum, 0, SPAN * sizeof(uint32_t));

  Serial1.setRxBufferSize(8192);
  Serial1.begin(LIDAR_BAUD, SERIAL_8N1, LIDAR_RX, LIDAR_TX);
  
  // Tunggu Lidar Boot
  vTaskDelay(pdMS_TO_TICKS(1000)); 

  // Reset Command
  const uint8_t stopCmd[] = {0xA5, 0x25};
  const uint8_t startCmd[] = {0xA5, 0x20};
  Serial1.write(stopCmd, 2);
  vTaskDelay(pdMS_TO_TICKS(100));
  while(Serial1.available()) Serial1.read();
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
          bool newSweep = false;
          if (prevAngle >= 0.0f) {
            if (prevAngle > 350.0f && p.angleDeg < 10.0f) newSweep = true;
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
            
            clearBuf(scanWrite); // Reset buffer tulis

            // --- LOGIKA KALIBRASI & VOLUME (STATE MACHINE) ---
            
            // Jika belum punya BG dan sudah lewat waktu boot (3s)
            if (!hasBg && (millis() - bootMs > 3000)) {
              // MODE KALIBRASI: Akumulasi data
              if (bgFrameCount < CALIBRATION_FRAMES) {
                // Ambil data dari scanRead yg baru saja di-swap
                for(int i=0; i<SPAN; i++) {
                   if(scanRead[i] > 0) bgSum[i] += scanRead[i];
                }
                bgFrameCount++;
                if (bgFrameCount % 5 == 0) Serial.printf("[CAL] Frame %d/%d\n", bgFrameCount, CALIBRATION_FRAMES);
              } 
              else {
                // FINALIZE KALIBRASI
                Serial.println("[CAL] Finalizing Background...");
                for(int i=0; i<SPAN; i++) {
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
          dbg_framesBad++; // Checksum/logic fail
        }
      } else {
        Serial1.read(); // Header salah, geser 1 byte
        dbg_rawBytes++;
        dbg_framesBad++;
      }
    }
  }
}

// =========================
// TASK: PRINT (CORE 0)
// =========================
void taskPrint(void *pv) {
  uint32_t last_ok = 0, last_bad = 0, last_sw = 0;

  for (;;) {
    vTaskDelay(pdMS_TO_TICKS(1000));
    uint32_t now = millis();
    
    Serial.printf("[PRT] T=%lu | EMA=%.2f cm2 | Vol=%.2f cm3 | FPS=%lu | Swp=%lu | FreeHeap=%u\n",
      now, sliceAreaEma, vTotal,
      (dbg_framesOK - last_ok),
      (dbg_sweeps - last_sw),
      esp_get_free_heap_size()
    );

    last_ok = dbg_framesOK;
    last_sw = dbg_sweeps;

    if (now - dbg_lastFrameMs > 2000) Serial.println("!! NO FRAMES RECEIVED !!");
  }
}

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("Starting FreeRTOS Lidar Optimized...");

  xTaskCreatePinnedToCore(taskPrint, "Print", 4096, NULL, 1, &taskPrintHandle, 0);
  xTaskCreatePinnedToCore(taskLidar, "Lidar", 16384, NULL, 5, &taskLidarHandle, 1);
}

void loop() {
  vTaskDelete(NULL); 
}