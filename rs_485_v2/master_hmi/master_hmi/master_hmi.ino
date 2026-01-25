/*
 * MCU MASTER (ESP32) - Nextion HMI + RS485 Multi-Slave (FreeRTOS)
 * -----------------------------------------------------------------
 * Goal:
 * - Menjadi master HMI seperti UI pada gambar (Dashboard / Manual / Settings)
 * - Mengirim perintah dan menerima telemetry dari 3 slave via RS485
 * - Protokol RS485 MENYESUAIKAN 3 kode slave kamu:
 *     - Slave mengirim ke master:   Nxx:<payload>\n
 *       ex: N02:TEMP:1234.5
 *           N02:LIMITS:0,1
 *           N02:LIM_HB:0,1
 *           N03:WEIGHT:12.34
 *           N03:LIMITS:1,0
 *     - Master mengirim ke slave:   TOxx:<payload>\n
 *       ex: TO02:REQ_ALL
 *           TO03:REQ_WEIGHT
 *           TO02:REQ_LIMITS
 *           TO03:TARE
 *
 * Catatan penting:
 * - Untuk SLAVE1 (MEGA) yang kamu kirim, kodenya belum menerima command RS485 (hanya USB Serial).
 *   Master ini mengasumsikan MEGA sudah dimodif sederhana untuk menerima frame TO01:<cmd>
 *   dan memanggil handleCommand(<cmd>) seperti handler USB-nya.
 *   Kalau belum, kamu harus tambah RX RS485 di MEGA agar bisa dikendalikan dari master.
 *
 * PERBAIKAN UTAMA (Nextion plug-n-play):
 * - Input Nextion tidak lagi pakai string EVT + newline.
 * - Sekarang pakai event default Nextion: Touch Event 0x65 (65 pp cc ee FF FF FF)
 * - Tidak perlu setting event script apa pun di Nextion.
 * - Kamu cukup isi DEFINE component id & page id sesuai desain HMI kamu.
 *
 * PERBAIKAN SETPOINT TEXT + TOMBOL UP/DOWN:
 * - nTempSet, nTimeSet, t2, t1 adalah OBJECT TEXT (bukan number).
 * - Nilai dibaca dari Nextion saat dibutuhkan via: get <obj>.txt  -> return 0x70 + string + FFF
 * - Tombol up/down untuk tiap setpoint:
 *     - Ambil nilai text (fallback ke g.* jika gagal)
 *     - Tambah/kurang sesuai step
 *     - Tulis balik ke text object (obj.txt="...")
 *     - Update g.* agar SAVE selalu kirim value terbaru
 */

#include <Arduino.h>

// ======================
// NEXTION (UART2)
// ======================
static HardwareSerial &nexSerial = Serial2;
static const int NEX_RX = 16;
static const int NEX_TX = 17;
static const uint32_t NEX_BAUD = 115200;

// Nextion command terminator
static inline void nexEnd() {
  nexSerial.write(0xFF);
  nexSerial.write(0xFF);
  nexSerial.write(0xFF);
}

static void nexCmd(const String &s) {
  nexSerial.print(s);
  nexEnd();
}

static void nexSetTxt(const char *obj, const String &val) {
  String cmd = String(obj) + ".txt=\"" + val + "\"";
  nexCmd(cmd);
}

static void nexSetNum(const char *obj, int v) {
  String cmd = String(obj) + ".val=" + String(v);
  nexCmd(cmd);
}

static void nexSetVis(const char *obj, bool vis) {
  String cmd = String("vis ") + obj + "," + (vis ? "1" : "0");
  nexCmd(cmd);
}

static void nexSetProgress(const char *obj, int pct0_100) {
  if (pct0_100 < 0) pct0_100 = 0;
  if (pct0_100 > 100) pct0_100 = 100;
  String cmd = String(obj) + ".val=" + String(pct0_100);
  nexCmd(cmd);
}

// ======================
// RS485 (UART1)
// ======================
static HardwareSerial &rsSer = Serial1;
static const int RS485_RX = 25;
static const int RS485_TX = 26;
static const int RS485_DE = 4;
static const uint32_t RS485_BAUD = 9600;

static inline void rsTxMode(bool en) {
  digitalWrite(RS485_DE, en ? HIGH : LOW);
}

static void rsSendLine(const String &line) {
  rsTxMode(true);
  rsSer.print(line);
  rsSer.print("\n");
  rsSer.flush();
  rsTxMode(false);
}

static inline void rsSendTo(uint8_t nodeId, const String &payload) {
  char hdr[8];
  snprintf(hdr, sizeof(hdr), "TO%02u:", (unsigned)nodeId);
  rsSendLine(String(hdr) + payload);
}

// ======================
// NODE IDS
// ======================
static const uint8_t NODE_MEGA_ACT = 1;
static const uint8_t NODE_TEMP     = 2;
static const uint8_t NODE_WEIGHT   = 3;

// ======================
// APP STATE
// ======================
enum UiPage : uint8_t { UI_DASH = 0, UI_MANUAL = 1, UI_SETTINGS = 2 };

struct SharedState {
  // sensors
  float tempC = NAN;
  float weightKg = NAN;
  uint8_t lim2_a = 0, lim2_b = 0; // dari slave2: LIMITS:x,y
  uint8_t lim3_a = 0, lim3_b = 0; // dari slave3: LIMITS:x,y

  // derived/metrics (placeholder, karena belum ada slave volume)
  float totalVolume = 0;
  uint32_t totalBatch = 0;
  float totalWeight = 0;

  // fuel (placeholder, idealnya dari MEGA ACT)
  uint8_t fuelPct = 0;

  // mode
  bool autoRunning = false;
  bool estop = false;

  // settings
  float setTemp = 1200.0f;
  uint32_t setTimeSec = 30;
  float setBatchKg = 5.0f;
  float setVolume = 0;
  int blowerAirLvl = 0;   // 0=off, 1=low, 2=high
  int blowerCoolLvl = 0;  // 0=off, 1=low, 2=high

  // manual params
  int manSpeed = 1000;
  int manSteps = 1000;
  bool manDirOpen = true;
} g;

static SemaphoreHandle_t gMutex;

// ======================
// QUEUES
// ======================
struct RsTxMsg { uint8_t node; char payload[120]; };
static QueueHandle_t qRsTx;

struct UiUpdate {
  char obj[24];
  char txt[64];
  int  isNum;
  int  num;
};
static QueueHandle_t qUi;

// ======================
// UTIL
// ======================
static inline bool isFinite(float x) { return isfinite(x); }

static bool parseFrame(const String &line, uint8_t &idOut, String &payloadOut) {
  // Expect:
  //  Nxx:payload
  if (line.length() < 5) return false;
  if (line[0] != 'N') return false;
  char d1 = line[1], d2 = line[2];
  if (d1 < '0' || d1 > '9' || d2 < '0' || d2 > '9') return false;
  if (line[3] != ':') return false;
  idOut = (uint8_t)((d1 - '0') * 10 + (d2 - '0'));
  payloadOut = line.substring(4);
  return true;
}

static void enqueueUiTxt(const char *obj, const String &val) {
  UiUpdate u{};
  strncpy(u.obj, obj, sizeof(u.obj) - 1);
  strncpy(u.txt, val.c_str(), sizeof(u.txt) - 1);
  u.isNum = 0;
  xQueueSend(qUi, &u, 0);
}

static void enqueueUiNum(const char *obj, int v) {
  UiUpdate u{};
  strncpy(u.obj, obj, sizeof(u.obj) - 1);
  u.isNum = 1;
  u.num = v;
  xQueueSend(qUi, &u, 0);
}

static void enqueueRs(uint8_t node, const String &payload) {
  RsTxMsg m{};
  m.node = node;
  strncpy(m.payload, payload.c_str(), sizeof(m.payload) - 1);
  xQueueSend(qRsTx, &m, 0);
}

// ======================
// HMI OBJECT NAMES (sesuai label di gambar)
// ======================
// DASH
static const char *OBJ_D_STAT   = "tStat";
static const char *OBJ_D_TIME   = "tTime";
static const char *OBJ_D_TEMP   = "tTemp";
static const char *OBJ_D_WEIGHT = "tWeight";
static const char *OBJ_D_VOL    = "tVolume";
static const char *OBJ_D_BATCH  = "tBatch";
static const char *OBJ_D_TW     = "tTotWeight";
static const char *OBJ_D_FUEL   = "tFuel";
static const char *OBJ_D_HFUEL  = "hFuel";   // progress bar
static const char *OBJ_D_AUTO   = "btAuto";  // start auto
static const char *OBJ_D_STOP   = "bStop";

// MANUAL
static const char *OBJ_M_DOOR   = "bManDoor";
static const char *OBJ_M_BDOOR  = "bManBDoor";
static const char *OBJ_M_ASH    = "bManAsh";
static const char *OBJ_M_BURNMV = "bManBum";   // burner mover
static const char *OBJ_M_CONV   = "bManCon";
static const char *OBJ_M_PUSH   = "bManPush";
static const char *OBJ_M_DIR    = "btManDir";
static const char *OBJ_M_RELAYB = "btManBum";  // relays burner (as per screenshot label overlap)
static const char *OBJ_M_RELAYI = "btManIgn";
static const char *OBJ_M_SPD    = "hManSpeed";
static const char *OBJ_M_STEP   = "hManStep";
static const char *OBJ_M_TEMP   = "tManTemp";
static const char *OBJ_M_WEIGHT = "tManWeight";
static const char *OBJ_M_BLOW_A = "b8";
static const char *OBJ_M_BLOW_C = "b9";
static const char *OBJ_M_STOP   = "b5";

// SETTINGS (NOTE: semua ini TEXT object di HMI kamu)
static const char *OBJ_S_TSET   = "nTempSet"; // TEXT
static const char *OBJ_S_TIME   = "nTimeSet"; // TEXT
static const char *OBJ_S_WB     = "t2";       // TEXT (berat/batch)
static const char *OBJ_S_VOL    = "t1";       // TEXT (volume)
static const char *OBJ_S_BLOW1T = "tBlow1txt";
static const char *OBJ_S_BLOW2T = "tBlow2txt";
static const char *OBJ_S_SAVE   = "bSave";
static const char *OBJ_S_RESET  = "bReset";
static const char *OBJ_S_STOP   = "bsettingStop";

// ======================
// STATUS TEXT HELPERS
// ======================
static String stateToText(bool autoRunning, bool estop) {
  if (estop) return "STOP";
  if (autoRunning) return "AUTO";
  return "IDLE";
}

static String fmt1(float v) {
  if (!isFinite(v)) return "IDLE";
  char b[24];
  snprintf(b, sizeof(b), "%.1f", (double)v);
  return String(b);
}

static String fmt2(float v) {
  if (!isFinite(v)) return "IDLE";
  char b[24];
  snprintf(b, sizeof(b), "%.2f", (double)v);
  return String(b);
}

// =====================================================
// NEXTION INPUT (DEFAULT TOUCH EVENT 0x65) - PLUG N PLAY
// =====================================================
// Nextion default touch event (return code):
//   0x65  pageId  compId  event  0xFF 0xFF 0xFF
// event: biasanya 0x00 = release, 0x01 = press
//
// Kamu cukup set page id & component id (di Nextion editor sudah ada fieldnya).
// Tidak perlu script event sama sekali.

// ---- isi sendiri sesuai HMI kamu ----
// PAGE IDs
#define NEX_PAGE_DASH      0   // <- isi
#define NEX_PAGE_MANUAL    1   // <- isi
#define NEX_PAGE_SETTINGS  2   // <- isi

// DASH component IDs
#define NEX_CID_btAuto       0   // <- isi
#define NEX_CID_bStop        0   // <- isi

// MANUAL component IDs
#define NEX_CID_bManDoor     0   // <- isi
#define NEX_CID_bManBDoor    0   // <- isi
#define NEX_CID_bManPush     0   // <- isi
#define NEX_CID_bManCon      0   // <- isi
#define NEX_CID_btManIgn     0   // <- isi
#define NEX_CID_btManBum     0   // <- isi
#define NEX_CID_hManSpeed    0   // <- isi
#define NEX_CID_hManStep     0   // <- isi
#define NEX_CID_b5Stop       0   // <- isi

// SETTINGS component IDs
#define NEX_CID_bSave         0   // <- isi
#define NEX_CID_bReset        0   // <- isi
#define NEX_CID_bsettingStop  0   // <- isi

// Setpoint buttons (UP/DOWN) (sesuai screenshot kamu)
#define NEX_CID_bTempUp       0   // <- isi (bTempUp)
#define NEX_CID_bTempDn       0   // <- isi (bTempD)
#define NEX_CID_bTimeUp       0   // <- isi (bTimeUp)
#define NEX_CID_bTimeDn       0   // <- isi (bTimeD)
#define NEX_CID_bBatchUp      0   // <- isi (b2)
#define NEX_CID_bBatchDn      0   // <- isi (b4)
#define NEX_CID_bVolUp        0   // <- isi (b1)
#define NEX_CID_bVolDn        0   // <- isi (b0)

// Blower buttons
#define NEX_CID_bBlow1        0   // <- isi (bBlow1)
#define NEX_CID_bBlow2        0   // <- isi (bBlow2)

// -----------------------------------------------------
// Default touch frame reader (robust sync)
// -----------------------------------------------------
static bool nexReadTouch(uint8_t &page, uint8_t &comp, uint8_t &evt) {
  // Frame: 65 pp cc ee FF FF FF
  static uint8_t buf[7];
  static uint8_t idx = 0;

  while (nexSerial.available()) {
    uint8_t b = (uint8_t)nexSerial.read();

    // sync to 0x65
    if (idx == 0) {
      if (b != 0x65) continue;
      buf[idx++] = b;
      continue;
    }

    buf[idx++] = b;

    if (idx == 7) {
      idx = 0;
      if (buf[0] == 0x65 && buf[4] == 0xFF && buf[5] == 0xFF && buf[6] == 0xFF) {
        page = buf[1];
        comp = buf[2];
        evt  = buf[3];
        return true;
      }
      // if invalid, continue scanning
    }
  }

  return false;
}

static void nexDrainNonTouchReturns(unsigned long maxMs) {
  unsigned long t0 = millis();
  while (nexSerial.available() && (millis() - t0) < maxMs) {
    (void)nexSerial.read();
  }
}

// =====================================================
// NEXTION GET TEXT (0x70) helpers (untuk object TEXT)
// =====================================================
static bool nexGetTxtBlocking(const char* objDotTxt, String &out, uint32_t timeoutMs = 140) {
  // bersihin input biar gak ketuker return lama
  while (nexSerial.available()) (void)nexSerial.read();

  // kirim query
  String cmd = String("get ") + objDotTxt; // ex: get nTempSet.txt
  nexCmd(cmd);

  // tunggu return 0x70 + <string bytes> + FFF
  out = "";
  unsigned long t0 = millis();
  bool started = false;

  while ((millis() - t0) < timeoutMs) {
    while (nexSerial.available()) {
      uint8_t b = (uint8_t)nexSerial.read();

      if (!started) {
        if (b != 0x70) continue;
        started = true;
        continue;
      }

      // after start: capture until FFF
      // We detect FFF by checking last 3 bytes after appending.
      out += (char)b;

      int n = out.length();
      if (n >= 3) {
        if ((uint8_t)out[n-1] == 0xFF && (uint8_t)out[n-2] == 0xFF && (uint8_t)out[n-3] == 0xFF) {
          out.remove(n-3); // drop terminator
          out.trim();
          return true;
        }
      }
    }
    vTaskDelay(pdMS_TO_TICKS(1));
  }

  return false;
}

static bool parseFloatLoose(const String &sIn, float &out) {
  String s = sIn;
  s.trim();
  if (!s.length()) return false;
  // allow commas as decimal separators, and remove non-numeric junk
  s.replace(",", ".");
  // Keep only digits, sign, dot
  String t; t.reserve(s.length());
  for (int i=0;i<(int)s.length();i++) {
    char c = s[i];
    if ((c>='0' && c<='9') || c=='-' || c=='+' || c=='.') t += c;
  }
  if (!t.length()) return false;
  out = t.toFloat();
  return isfinite(out);
}

static float clampf(float v, float lo, float hi) {
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

static void formatFloatNoTrailingZeros(float v, uint8_t decimals, char *out, size_t outSz) {
  // decimals: 0..3 typical
  char fmt[8];
  snprintf(fmt, sizeof(fmt), "%%.%uf", (unsigned)decimals);
  char buf[32];
  snprintf(buf, sizeof(buf), fmt, (double)v);

  // trim trailing zeros + dot
  String s(buf);
  if (decimals > 0) {
    while (s.endsWith("0")) s.remove(s.length()-1);
    if (s.endsWith(".")) s.remove(s.length()-1);
  }
  strncpy(out, s.c_str(), outSz-1);
  out[outSz-1] = 0;
}

// ============================================
// SETPOINT RULES (step + clamp)
// ============================================
static const float SP_TEMP_STEP  = 10.0f;
static const float SP_TEMP_MIN   = 0.0f;
static const float SP_TEMP_MAX   = 2000.0f;

static const float SP_BATCH_STEP = 0.1f;
static const float SP_BATCH_MIN  = 0.0f;
static const float SP_BATCH_MAX  = 999.0f;

static const float SP_VOL_STEP   = 0.1f;
static const float SP_VOL_MIN    = 0.0f;
static const float SP_VOL_MAX    = 99999.0f;

static const int   SP_TIME_STEP  = 1;
static const int   SP_TIME_MIN   = 1;
static const int   SP_TIME_MAX   = 86400;

// Helper: read a TEXT object as float, fallback to provided fallbackVal if fail.
static float readTextAsFloatOrFallback(const char *objName, float fallbackVal) {
  String txt;
  if (nexGetTxtBlocking((String(objName) + ".txt").c_str(), txt)) {
    float v;
    if (parseFloatLoose(txt, v)) return v;
  }
  return fallbackVal;
}

static void writeTextValue(const char *objName, float v, uint8_t decimals) {
  char b[32];
  formatFloatNoTrailingZeros(v, decimals, b, sizeof(b));
  nexSetTxt(objName, String(b));
}

static void writeTextValueInt(const char *objName, int v) {
  nexSetTxt(objName, String(v));
}

// ======================
// TASK: RS485 TX
// ======================
static void taskRsTx(void *pv) {
  RsTxMsg m;
  for (;;) {
    if (xQueueReceive(qRsTx, &m, portMAX_DELAY) == pdTRUE) {
      rsSendTo(m.node, String(m.payload));
    }
  }
}

// ======================
// TASK: RS485 RX
// ======================
static void taskRsRx(void *pv) {
  String buf;
  buf.reserve(256);

  for (;;) {
    while (rsSer.available()) {
      char c = (char)rsSer.read();
      if (c == '\r') continue;
      if (c == '\n') {
        buf.trim();
        if (buf.length()) {
          uint8_t id;
          String payload;
          if (parseFrame(buf, id, payload)) {
            // Update shared state
            if (xSemaphoreTake(gMutex, pdMS_TO_TICKS(20)) == pdTRUE) {
              if (id == NODE_TEMP) {
                if (payload.startsWith("TEMP:")) {
                  String v = payload.substring(5);
                  if (v == "OPEN" || v == "NaN") g.tempC = NAN;
                  else {
                    float t = v.toFloat();
                    g.tempC = isfinite(t) ? t : NAN;
                  }
                } else if (payload.startsWith("LIMITS:")) {
                  int c1 = payload.indexOf(',');
                  if (c1 > 7) {
                    g.lim2_a = (uint8_t)payload.substring(7, c1).toInt();
                    g.lim2_b = (uint8_t)payload.substring(c1 + 1).toInt();
                  }
                } else if (payload.startsWith("LIM_HB:")) {
                  int c1 = payload.indexOf(',');
                  if (c1 > 7) {
                    g.lim2_a = (uint8_t)payload.substring(7, c1).toInt();
                    g.lim2_b = (uint8_t)payload.substring(c1 + 1).toInt();
                  }
                }
              } else if (id == NODE_WEIGHT) {
                if (payload.startsWith("WEIGHT:")) {
                  String v = payload.substring(7);
                  if (v == "NaN") g.weightKg = NAN;
                  else {
                    float w = v.toFloat();
                    g.weightKg = isfinite(w) ? w : NAN;
                  }
                } else if (payload.startsWith("LIMITS:")) {
                  int c1 = payload.indexOf(',');
                  if (c1 > 7) {
                    g.lim3_a = (uint8_t)payload.substring(7, c1).toInt();
                    g.lim3_b = (uint8_t)payload.substring(c1 + 1).toInt();
                  }
                } else if (payload.startsWith("LIM_HB:")) {
                  int c1 = payload.indexOf(',');
                  if (c1 > 7) {
                    g.lim3_a = (uint8_t)payload.substring(7, c1).toInt();
                    g.lim3_b = (uint8_t)payload.substring(c1 + 1).toInt();
                  }
                }
              } else if (id == NODE_MEGA_ACT) {
                if (payload.startsWith("FUEL:")) {
                  int pct = payload.substring(5).toInt();
                  if (pct < 0) pct = 0; if (pct > 100) pct = 100;
                  g.fuelPct = (uint8_t)pct;
                } else if (payload.startsWith("AUTO:")) {
                  g.autoRunning = payload.substring(5).toInt() != 0;
                }
              }
              xSemaphoreGive(gMutex);
            }
          }
        }
        buf = "";
      } else {
        buf += c;
        if (buf.length() > 240) buf = "";
      }
    }
    vTaskDelay(pdMS_TO_TICKS(5));
  }
}

// ======================
// TASK: NEXTION RX (DEFAULT TOUCH)
// ======================
static void taskNexRx(void *pv) {
  for (;;) {
    uint8_t page = 0, comp = 0, evt = 0;

    while (nexReadTouch(page, comp, evt)) {
      const bool isRelease = (evt == 0x00);
      if (!isRelease) continue;

      // DASH
      if (page == NEX_PAGE_DASH && comp == NEX_CID_bStop) {
        if (xSemaphoreTake(gMutex, pdMS_TO_TICKS(20)) == pdTRUE) {
          g.estop = true;
          g.autoRunning = false;
          xSemaphoreGive(gMutex);
        }
        enqueueRs(NODE_MEGA_ACT, "CMD_EMERGENCY_STOP");
      }
      else if (page == NEX_PAGE_DASH && comp == NEX_CID_btAuto) {
        if (xSemaphoreTake(gMutex, pdMS_TO_TICKS(20)) == pdTRUE) {
          g.estop = false;
          g.autoRunning = true;
          xSemaphoreGive(gMutex);
        }
        enqueueRs(NODE_MEGA_ACT, "CMD_RUN_AUTO");
      }

      // MANUAL
      else if (page == NEX_PAGE_MANUAL && comp == NEX_CID_b5Stop) {
        if (xSemaphoreTake(gMutex, pdMS_TO_TICKS(20)) == pdTRUE) {
          g.estop = true;
          g.autoRunning = false;
          xSemaphoreGive(gMutex);
        }
        enqueueRs(NODE_MEGA_ACT, "CMD_EMERGENCY_STOP");
      }
      else if (page == NEX_PAGE_MANUAL && comp == NEX_CID_bManDoor) {
        bool open;
        if (xSemaphoreTake(gMutex, pdMS_TO_TICKS(20)) == pdTRUE) {
          g.manDirOpen = !g.manDirOpen;
          open = g.manDirOpen;
          xSemaphoreGive(gMutex);
        } else {
          open = true;
        }
        enqueueRs(NODE_MEGA_ACT, open ? "MAINDOOR_OPEN" : "MAINDOOR_CLOSE");
      }
      else if (page == NEX_PAGE_MANUAL && comp == NEX_CID_bManBDoor) {
        bool open;
        if (xSemaphoreTake(gMutex, pdMS_TO_TICKS(20)) == pdTRUE) {
          g.manDirOpen = !g.manDirOpen;
          open = g.manDirOpen;
          xSemaphoreGive(gMutex);
        } else {
          open = true;
        }
        enqueueRs(NODE_MEGA_ACT, open ? "BDOOR_OPEN" : "BDOOR_CLOSE");
      }
      else if (page == NEX_PAGE_MANUAL && comp == NEX_CID_bManPush) {
        bool maju;
        if (xSemaphoreTake(gMutex, pdMS_TO_TICKS(20)) == pdTRUE) {
          g.manDirOpen = !g.manDirOpen;
          maju = g.manDirOpen;
          xSemaphoreGive(gMutex);
        } else {
          maju = true;
        }
        enqueueRs(NODE_MEGA_ACT, maju ? "PUSHER_MAJU" : "PUSHER_MUNDUR");
      }
      else if (page == NEX_PAGE_MANUAL && comp == NEX_CID_bManCon) {
        static uint8_t convMode = 0; // 0 stop, 1 fwd, 2 rev
        convMode = (uint8_t)((convMode + 1) % 3);
        if (convMode == 0) enqueueRs(NODE_MEGA_ACT, "CONVEYOR_STOP");
        else if (convMode == 1) enqueueRs(NODE_MEGA_ACT, "CONVEYOR_FWD");
        else enqueueRs(NODE_MEGA_ACT, "CONVEYOR_REV");
      }
      else if (page == NEX_PAGE_MANUAL && comp == NEX_CID_btManIgn) {
        static uint8_t ign = 0;
        ign ^= 1;
        enqueueRs(NODE_MEGA_ACT, String("CMD_MAN_IGN:") + String((int)ign));
      }
      else if (page == NEX_PAGE_MANUAL && comp == NEX_CID_btManBum) {
        static uint8_t burn = 0;
        burn ^= 1;
        enqueueRs(NODE_MEGA_ACT, String("CMD_MAN_BURN:") + String((int)burn));
      }

      // SETTINGS
      else if (page == NEX_PAGE_SETTINGS && comp == NEX_CID_bsettingStop) {
        if (xSemaphoreTake(gMutex, pdMS_TO_TICKS(20)) == pdTRUE) {
          g.estop = true;
          g.autoRunning = false;
          xSemaphoreGive(gMutex);
        }
        enqueueRs(NODE_MEGA_ACT, "CMD_EMERGENCY_STOP");
      }

      // --- UP/DOWN Setpoints (TEXT objects) ---
      else if (page == NEX_PAGE_SETTINGS && (comp == NEX_CID_bTempUp || comp == NEX_CID_bTempDn)) {
        float cur;
        if (xSemaphoreTake(gMutex, pdMS_TO_TICKS(20)) == pdTRUE) { cur = g.setTemp; xSemaphoreGive(gMutex); }
        else cur = 1200.0f;

        float v = readTextAsFloatOrFallback(OBJ_S_TSET, cur);
        v += (comp == NEX_CID_bTempUp) ? SP_TEMP_STEP : -SP_TEMP_STEP;
        v = clampf(v, SP_TEMP_MIN, SP_TEMP_MAX);

        writeTextValue(OBJ_S_TSET, v, 0);

        if (xSemaphoreTake(gMutex, pdMS_TO_TICKS(20)) == pdTRUE) {
          g.setTemp = v;
          xSemaphoreGive(gMutex);
        }
      }
      else if (page == NEX_PAGE_SETTINGS && (comp == NEX_CID_bTimeUp || comp == NEX_CID_bTimeDn)) {
        uint32_t cur;
        if (xSemaphoreTake(gMutex, pdMS_TO_TICKS(20)) == pdTRUE) { cur = g.setTimeSec; xSemaphoreGive(gMutex); }
        else cur = 30;

        float f = readTextAsFloatOrFallback(OBJ_S_TIME, (float)cur);
        int v = (int)roundf(f);
        v += (comp == NEX_CID_bTimeUp) ? SP_TIME_STEP : -SP_TIME_STEP;
        if (v < SP_TIME_MIN) v = SP_TIME_MIN;
        if (v > SP_TIME_MAX) v = SP_TIME_MAX;

        writeTextValueInt(OBJ_S_TIME, v);

        if (xSemaphoreTake(gMutex, pdMS_TO_TICKS(20)) == pdTRUE) {
          g.setTimeSec = (uint32_t)v;
          xSemaphoreGive(gMutex);
        }
      }
      else if (page == NEX_PAGE_SETTINGS && (comp == NEX_CID_bBatchUp || comp == NEX_CID_bBatchDn)) {
        float cur;
        if (xSemaphoreTake(gMutex, pdMS_TO_TICKS(20)) == pdTRUE) { cur = g.setBatchKg; xSemaphoreGive(gMutex); }
        else cur = 5.0f;

        float v = readTextAsFloatOrFallback(OBJ_S_WB, cur);
        v += (comp == NEX_CID_bBatchUp) ? SP_BATCH_STEP : -SP_BATCH_STEP;
        v = clampf(v, SP_BATCH_MIN, SP_BATCH_MAX);

        writeTextValue(OBJ_S_WB, v, 2);

        if (xSemaphoreTake(gMutex, pdMS_TO_TICKS(20)) == pdTRUE) {
          g.setBatchKg = v;
          xSemaphoreGive(gMutex);
        }
      }
      else if (page == NEX_PAGE_SETTINGS && (comp == NEX_CID_bVolUp || comp == NEX_CID_bVolDn)) {
        float cur;
        if (xSemaphoreTake(gMutex, pdMS_TO_TICKS(20)) == pdTRUE) { cur = g.setVolume; xSemaphoreGive(gMutex); }
        else cur = 0.0f;

        float v = readTextAsFloatOrFallback(OBJ_S_VOL, cur);
        v += (comp == NEX_CID_bVolUp) ? SP_VOL_STEP : -SP_VOL_STEP;
        v = clampf(v, SP_VOL_MIN, SP_VOL_MAX);

        writeTextValue(OBJ_S_VOL, v, 2);

        if (xSemaphoreTake(gMutex, pdMS_TO_TICKS(20)) == pdTRUE) {
          g.setVolume = v;
          xSemaphoreGive(gMutex);
        }
      }

      else if (page == NEX_PAGE_SETTINGS && comp == NEX_CID_bSave) {
        // Pastikan g.* sinkron dengan layar sebelum kirim.
        // Karena semua setpoint adalah TEXT, kita tarik .txt di sini juga (safety net).
        float v;
        String txt;

        if (nexGetTxtBlocking((String(OBJ_S_TSET) + ".txt").c_str(), txt) && parseFloatLoose(txt, v)) {
          v = clampf(v, SP_TEMP_MIN, SP_TEMP_MAX);
          if (xSemaphoreTake(gMutex, pdMS_TO_TICKS(20)) == pdTRUE) { g.setTemp = v; xSemaphoreGive(gMutex); }
        }

        if (nexGetTxtBlocking((String(OBJ_S_TIME) + ".txt").c_str(), txt) && parseFloatLoose(txt, v)) {
          int iv = (int)roundf(v);
          if (iv < SP_TIME_MIN) iv = SP_TIME_MIN;
          if (iv > SP_TIME_MAX) iv = SP_TIME_MAX;
          if (xSemaphoreTake(gMutex, pdMS_TO_TICKS(20)) == pdTRUE) { g.setTimeSec = (uint32_t)iv; xSemaphoreGive(gMutex); }
        }

        if (nexGetTxtBlocking((String(OBJ_S_WB) + ".txt").c_str(), txt) && parseFloatLoose(txt, v)) {
          v = clampf(v, SP_BATCH_MIN, SP_BATCH_MAX);
          if (xSemaphoreTake(gMutex, pdMS_TO_TICKS(20)) == pdTRUE) { g.setBatchKg = v; xSemaphoreGive(gMutex); }
        }

        if (nexGetTxtBlocking((String(OBJ_S_VOL) + ".txt").c_str(), txt) && parseFloatLoose(txt, v)) {
          v = clampf(v, SP_VOL_MIN, SP_VOL_MAX);
          if (xSemaphoreTake(gMutex, pdMS_TO_TICKS(20)) == pdTRUE) { g.setVolume = v; xSemaphoreGive(gMutex); }
        }

        // baru kirim settings pakai g yang sudah sinkron
        float setTemp; uint32_t setTime; float setBatch; float setVol;
        int b1, b2;
        if (xSemaphoreTake(gMutex, pdMS_TO_TICKS(20)) == pdTRUE) {
          setTemp = g.setTemp;
          setTime = g.setTimeSec;
          setBatch = g.setBatchKg;
          setVol = g.setVolume;
          b1 = g.blowerAirLvl;
          b2 = g.blowerCoolLvl;
          xSemaphoreGive(gMutex);
        }

        enqueueRs(NODE_MEGA_ACT, String("SET_TE:") + String(setTemp, 1));
        enqueueRs(NODE_MEGA_ACT, String("SET_TI:") + String((unsigned)setTime));
        enqueueRs(NODE_MEGA_ACT, String("SET_W:")  + String(setBatch, 2));
        (void)setVol;
        enqueueRs(NODE_MEGA_ACT, String("BLOW:air:") + String(b1));
        enqueueRs(NODE_MEGA_ACT, String("BLOW:cool:") + String(b2));
      }
      else if (page == NEX_PAGE_SETTINGS && comp == NEX_CID_bReset) {
        if (xSemaphoreTake(gMutex, pdMS_TO_TICKS(20)) == pdTRUE) {
          g.setTemp = 1200.0f;
          g.setTimeSec = 30;
          g.setBatchKg = 5.0f;
          g.setVolume = 0;
          g.blowerAirLvl = 0;
          g.blowerCoolLvl = 0;
          xSemaphoreGive(gMutex);
        }
        // reset juga UI text
        writeTextValue(OBJ_S_TSET, 1200.0f, 0);
        writeTextValueInt(OBJ_S_TIME, 30);
        writeTextValue(OBJ_S_WB, 5.0f, 2);
        writeTextValue(OBJ_S_VOL, 0.0f, 2);
      }
      else if (page == NEX_PAGE_SETTINGS && comp == NEX_CID_bBlow1) {
        if (xSemaphoreTake(gMutex, pdMS_TO_TICKS(20)) == pdTRUE) {
          g.blowerAirLvl = (g.blowerAirLvl + 1) % 3;
          xSemaphoreGive(gMutex);
        }
      }
      else if (page == NEX_PAGE_SETTINGS && comp == NEX_CID_bBlow2) {
        if (xSemaphoreTake(gMutex, pdMS_TO_TICKS(20)) == pdTRUE) {
          g.blowerCoolLvl = (g.blowerCoolLvl + 1) % 3;
          xSemaphoreGive(gMutex);
        }
      }
    }

    if (nexSerial.available() > 128) {
      nexDrainNonTouchReturns(2);
    }

    vTaskDelay(pdMS_TO_TICKS(5));
  }
}

// ======================
// TASK: NEXTION UI FLUSH
// ======================
static void taskUiFlush(void *pv) {
  UiUpdate u;
  for (;;) {
    while (xQueueReceive(qUi, &u, 0) == pdTRUE) {
      if (u.isNum) nexSetNum(u.obj, u.num);
      else nexSetTxt(u.obj, String(u.txt));
      vTaskDelay(pdMS_TO_TICKS(5));
    }
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

// ======================
// TASK: POLLER (REQ_ALL)
// ======================
static void taskPoller(void *pv) {
  const TickType_t dt = pdMS_TO_TICKS(500);
  uint32_t lastSec = 0;
  for (;;) {
    enqueueRs(NODE_TEMP, "REQ_ALL");
    enqueueRs(NODE_WEIGHT, "REQ_ALL");

    uint32_t s = millis() / 1000;
    if (s != lastSec) {
      lastSec = s;
      uint32_t mm = (s / 60) % 60;
      uint32_t hh = (s / 3600);
      char b[16];
      snprintf(b, sizeof(b), "%02u:%02u", (unsigned)hh, (unsigned)mm);
      enqueueUiTxt(OBJ_D_TIME, String(b));
    }

    vTaskDelay(dt);
  }
}

// ======================
// TASK: UI RENDER (push shared state into Nextion)
// ======================
static void taskUiRender(void *pv) {
  const TickType_t dt = pdMS_TO_TICKS(250);
  for (;;) {
    SharedState s;
    if (xSemaphoreTake(gMutex, pdMS_TO_TICKS(30)) == pdTRUE) {
      s = g;
      xSemaphoreGive(gMutex);
    }

    // DASHBOARD
    enqueueUiTxt(OBJ_D_STAT, stateToText(s.autoRunning, s.estop));
    enqueueUiTxt(OBJ_D_TEMP, fmt1(s.tempC));
    enqueueUiTxt(OBJ_D_WEIGHT, fmt2(s.weightKg));

    // placeholders
    enqueueUiTxt(OBJ_D_VOL, fmt1(s.totalVolume));
    enqueueUiTxt(OBJ_D_BATCH, String(s.totalBatch));
    enqueueUiTxt(OBJ_D_TW, fmt2(s.totalWeight));

    enqueueUiTxt(OBJ_D_FUEL, String((int)s.fuelPct) + "%");
    enqueueUiNum(OBJ_D_HFUEL, (int)s.fuelPct);

    // MANUAL snapshot
    enqueueUiTxt(OBJ_M_TEMP, fmt1(s.tempC));
    enqueueUiTxt(OBJ_M_WEIGHT, fmt2(s.weightKg));

    // SETTINGS snapshot (karena object setpoint adalah TEXT, kita tulis TXT, bukan .val)
    // Ini hanya mirror dari g.* ke layar. Jika user klik UP/DOWN, g.* sudah di-update.
    {
      char b[32];
      formatFloatNoTrailingZeros(s.setTemp, 0, b, sizeof(b));
      enqueueUiTxt(OBJ_S_TSET, String(b));
    }
    enqueueUiTxt(OBJ_S_TIME, String((unsigned)s.setTimeSec));
    {
      char b[32];
      formatFloatNoTrailingZeros(s.setBatchKg, 2, b, sizeof(b));
      enqueueUiTxt(OBJ_S_WB, String(b));
    }
    {
      char b[32];
      formatFloatNoTrailingZeros(s.setVolume, 2, b, sizeof(b));
      enqueueUiTxt(OBJ_S_VOL, String(b));
    }

    auto lvlToTxt = [](int lvl) -> String {
      if (lvl <= 0) return "OFF";
      if (lvl == 1) return "LOW";
      return "HIGH";
    };
    enqueueUiTxt(OBJ_S_BLOW1T, lvlToTxt(s.blowerAirLvl));
    enqueueUiTxt(OBJ_S_BLOW2T, lvlToTxt(s.blowerCoolLvl));

    vTaskDelay(dt);
  }
}

// ======================
// BOOT INIT HMI (optional)
// ======================
static void hmiBootInit() {
  nexCmd("page 0");
  nexSetTxt(OBJ_D_STAT, "IDLE");
  nexSetTxt(OBJ_D_TEMP, "IDLE");
  nexSetTxt(OBJ_D_WEIGHT, "IDLE");
  nexSetTxt(OBJ_D_VOL, "0");
  nexSetTxt(OBJ_D_BATCH, "0");
  nexSetTxt(OBJ_D_TW, "0");
  nexSetTxt(OBJ_D_FUEL, "0%");
  nexSetProgress(OBJ_D_HFUEL, 0);

  // init setpoint text sesuai default g
  writeTextValue(OBJ_S_TSET, g.setTemp, 0);
  writeTextValueInt(OBJ_S_TIME, (int)g.setTimeSec);
  writeTextValue(OBJ_S_WB, g.setBatchKg, 2);
  writeTextValue(OBJ_S_VOL, g.setVolume, 2);
}

// ======================
// SETUP / LOOP
// ======================
void setup() {
  Serial.begin(115200);
  delay(200);

  // RS485 init
  pinMode(RS485_DE, OUTPUT);
  rsTxMode(false);
  rsSer.begin(RS485_BAUD, SERIAL_8N1, RS485_RX, RS485_TX);

  // Nextion init
  nexSerial.begin(NEX_BAUD, SERIAL_8N1, NEX_RX, NEX_TX);

  // sync primitives
  gMutex = xSemaphoreCreateMutex();
  qRsTx  = xQueueCreate(24, sizeof(RsTxMsg));
  qUi    = xQueueCreate(32, sizeof(UiUpdate));

  // initial shared state
  if (xSemaphoreTake(gMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
    g.tempC = NAN;
    g.weightKg = NAN;
    g.autoRunning = false;
    g.estop = false;
    g.fuelPct = 0;
    xSemaphoreGive(gMutex);
  }

  hmiBootInit();

  // tasks
  xTaskCreatePinnedToCore(taskRsTx,     "rsTx",     4096, nullptr, 3, nullptr, 1);
  xTaskCreatePinnedToCore(taskRsRx,     "rsRx",     4096, nullptr, 3, nullptr, 1);
  xTaskCreatePinnedToCore(taskNexRx,    "nexRx",    6144, nullptr, 2, nullptr, 0);
  xTaskCreatePinnedToCore(taskUiFlush,  "uiFlush",  4096, nullptr, 1, nullptr, 0);
  xTaskCreatePinnedToCore(taskPoller,   "poller",   4096, nullptr, 1, nullptr, 1);
  xTaskCreatePinnedToCore(taskUiRender, "uiRender", 4096, nullptr, 1, nullptr, 0);

  // initial poll
  enqueueRs(NODE_TEMP, "REQ_ALL");
  enqueueRs(NODE_WEIGHT, "REQ_ALL");

  Serial.println("[MASTER] READY");
}

void loop() {
  vTaskDelay(pdMS_TO_TICKS(1000));
}

/*
 * END.
 */
