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

// SETTINGS
static const char *OBJ_S_TSET   = "nTempSet";
static const char *OBJ_S_TIME   = "nTimeSet";
static const char *OBJ_S_WB     = "t2";     // setpoint berat/batch
static const char *OBJ_S_VOL    = "t1";     // setpoint volume
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
//
// NOTE:
// - Nextion juga bisa mengirim return lain (0x66, 0x70, 0x71, error, dsb).
//   Parser ini akan sinkronisasi dan hanya memicu saat frame 0x65 valid.

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
#define NEX_CID_bManCon      0   // <- isi (bisa cycle/segmented button)
#define NEX_CID_btManIgn     0   // <- isi
#define NEX_CID_btManBum     0   // <- isi
#define NEX_CID_hManSpeed    0   // <- isi (slider)
#define NEX_CID_hManStep     0   // <- isi (slider)
#define NEX_CID_b5Stop       0   // <- isi (stop di page manual)

// SETTINGS component IDs
#define NEX_CID_bSave         0   // <- isi
#define NEX_CID_bReset        0   // <- isi
#define NEX_CID_bsettingStop  0   // <- isi
#define NEX_CID_nTempSet      0   // <- isi (number)
#define NEX_CID_nTimeSet      0   // <- isi (number)
#define NEX_CID_t2Batch       0   // <- isi (text/number)
#define NEX_CID_t1Vol         0   // <- isi (text/number)
#define NEX_CID_bBlow1        0   // <- isi (button)
#define NEX_CID_bBlow2        0   // <- isi (button)

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

// OPTIONAL: baca numeric/text return jika kamu nanti mau tanpa script.
// Saat ini tidak dipakai (tetap ada, tidak mengganggu).
static void nexDrainNonTouchReturns(unsigned long maxMs) {
  unsigned long t0 = millis();
  while (nexSerial.available() && (millis() - t0) < maxMs) {
    (void)nexSerial.read();
  }
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
                // optional: parse MEGA telemetry kalau kamu tambahkan nanti, contoh:
                //  N01:STATE:IDLE
                //  N01:FUEL:75
                //  N01:METRICS:vol,batch,totalW
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

    // process all available touch frames
    while (nexReadTouch(page, comp, evt)) {
      // We act on RELEASE by default (lebih stabil, mencegah double-trigger)
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
        // button toggle state tidak bisa dibaca dari 0x65.
        // default: treat as toggle local, kirim sesuai local state.
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
        // Tanpa script, kita tidak tahu value multi-state dari Nextion.
        // Solusi plug-n-play: cycle state lokal: STOP -> FWD -> REV -> STOP ...
        static uint8_t convMode = 0; // 0 stop, 1 fwd, 2 rev
        convMode = (uint8_t)((convMode + 1) % 3);
        if (convMode == 0) enqueueRs(NODE_MEGA_ACT, "CONVEYOR_STOP");
        else if (convMode == 1) enqueueRs(NODE_MEGA_ACT, "CONVEYOR_FWD");
        else enqueueRs(NODE_MEGA_ACT, "CONVEYOR_REV");
      }
      else if (page == NEX_PAGE_MANUAL && comp == NEX_CID_btManIgn) {
        // toggle local
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
      else if (page == NEX_PAGE_SETTINGS && comp == NEX_CID_bSave) {
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
        (void)setVol; // volume placeholder
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
      }
      else if (page == NEX_PAGE_SETTINGS && comp == NEX_CID_bBlow1) {
        // cycle blower air lvl: 0->1->2->0
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

      // numeric widgets (nTempSet/nTimeSet/t2/t1/hManSpeed/hManStep)
      // Tanpa script, Nextion tidak otomatis push value ke MCU hanya lewat 0x65.
      // Plug-n-play approach: biarkan user edit, MCU tetap punya state internal.
      // Nilai tetap dikirim saat SAVE via g.* (yang bisa kamu update lewat cara lain bila mau).
    }

    // Optional: drain bytes lain agar tidak numpuk jika Nextion spam return codes
    // (tidak wajib, tapi aman)
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
    // request periodic snapshots
    enqueueRs(NODE_TEMP, "REQ_ALL");
    enqueueRs(NODE_WEIGHT, "REQ_ALL");

    // optional: ask MEGA for fuel/status if you implement it
    // enqueueRs(NODE_MEGA_ACT, "REQ_ALL");

    // update time text (simple uptime)
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

    // SETTINGS snapshot
    enqueueUiNum(OBJ_S_TSET, (int)roundf(s.setTemp));
    enqueueUiNum(OBJ_S_TIME, (int)s.setTimeSec);
    enqueueUiTxt(OBJ_S_WB, String(s.setBatchKg, 1));
    enqueueUiTxt(OBJ_S_VOL, String(s.setVolume, 1));

    // blower text (sekadar status)
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
  xTaskCreatePinnedToCore(taskNexRx,    "nexRx",    4096, nullptr, 2, nullptr, 0);
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
