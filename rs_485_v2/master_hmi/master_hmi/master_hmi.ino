/*
 * MCU MASTER (ESP32) - Nextion HMI (Library) + RS485 Multi-Slave (FreeRTOS)
 * ------------------------------------------------------------------------
 * MIGRATION GOALS (from "string driver" -> Nextion library, safer & tested):
 * - Pakai Nextion library object-based (NexText/NexNumber/NexProgressBar/NexButton/NexDSButton/NexSlider)
 * - Event input pakai attachPop() (release) seperti HMI lama
 * - Tidak menambah tombol baru: hanya mapping komponen yang sudah ada di Master baru
 * - Tetap mempertahankan: FreeRTOS tasks, queue UI/RS485, shared state, poller
 *
 * NOTE:
 * - Kamu HARUS sesuaikan PAGE ID + COMPONENT ID di bawah agar match dengan HMI kamu.
 * - Nextion library butuh nexLoop(nex_listen_list) dipanggil rutin.
 *   Di sini dipanggil di taskNexLoop (FreeRTOS), jadi loop() tetap kosong.
 */

#include <Arduino.h>

// ======================
// NEXTION (UART2)
// ======================
#include <Nextion.h>

static HardwareSerial &nexSerial = Serial2;
static const int NEX_RX = 16;
static const int NEX_TX = 17;
static const uint32_t NEX_BAUD = 9600;

// IMPORTANT: set this to use Serial2
#define nexSerial Serial2

// ======================
// RS485 (UART1)
// ======================
static HardwareSerial &rsSer = Serial1;
static const int RS485_RX = 25;
static const int RS485_TX = 26;
static const int RS485_DE = 4;
static const uint32_t RS485_BAUD = 9600;

static inline void rsTxMode(bool en) { digitalWrite(RS485_DE, en ? HIGH : LOW); }

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
struct SharedState {
  float tempC = NAN;
  float weightKg = NAN;
  uint8_t lim2_a = 0, lim2_b = 0;
  uint8_t lim3_a = 0, lim3_b = 0;

  float totalVolume = 0;
  uint32_t totalBatch = 0;
  float totalWeight = 0;

  uint8_t fuelPct = 0;

  bool autoRunning = false;
  bool estop = false;

  // settings
  float setTemp = 1200.0f;
  uint32_t setTimeSec = 30;
  float setBatchKg = 5.0f;
  float setVolume = 0;
  int blowerAirLvl = 0;
  int blowerCoolLvl = 0;

  // manual
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

// UI update queue (safer to not touch Nex objects from multiple tasks)
enum UiKind : uint8_t { UI_TXT=0, UI_NUM=1 };
struct UiUpdate {
  UiKind kind;
  uint16_t objId;    // internal id we map to Nex objects
  char txt[64];
  int32_t num;
};
static QueueHandle_t qUi;

// ======================
// UTIL
// ======================
static inline bool isFinite(float x) { return isfinite(x); }

static bool parseFrame(const String &line, uint8_t &idOut, String &payloadOut) {
  if (line.length() < 5) return false;
  if (line[0] != 'N') return false;
  char d1 = line[1], d2 = line[2];
  if (d1 < '0' || d1 > '9' || d2 < '0' || d2 > '9') return false;
  if (line[3] != ':') return false;
  idOut = (uint8_t)((d1 - '0') * 10 + (d2 - '0'));
  payloadOut = line.substring(4);
  return true;
}

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

static float clampf(float v, float lo, float hi) {
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

static bool parseFloatLoose(const String &sIn, float &out) {
  String s = sIn;
  s.trim();
  if (!s.length()) return false;
  s.replace(",", ".");
  String t; t.reserve(s.length());
  for (int i=0;i<(int)s.length();i++) {
    char c = s[i];
    if ((c>='0' && c<='9') || c=='-' || c=='+' || c=='.') t += c;
  }
  if (!t.length()) return false;
  out = t.toFloat();
  return isfinite(out);
}

static void formatFloatNoTrailingZeros(float v, uint8_t decimals, char *out, size_t outSz) {
  char fmt[8];
  snprintf(fmt, sizeof(fmt), "%%.%uf", (unsigned)decimals);
  char buf[32];
  snprintf(buf, sizeof(buf), fmt, (double)v);

  String s(buf);
  if (decimals > 0) {
    while (s.endsWith("0")) s.remove(s.length()-1);
    if (s.endsWith(".")) s.remove(s.length()-1);
  }
  strncpy(out, s.c_str(), outSz-1);
  out[outSz-1] = 0;
}

// ======================
// SETPOINT RULES
// ======================
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

// ======================
// NEXTION: PAGE/COMP IDs
// ======================
// Kamu WAJIB sesuaikan ini dengan Nextion editor (page, id).
// NOTE: Nextion lib pakai (pageId, componentId, "name") saat deklarasi.

// Pages
static const uint8_t PAGE_DASH     = 0;
static const uint8_t PAGE_MANUAL   = 1;
static const uint8_t PAGE_SETTINGS = 2;

// ----------------------
// DASH objects
// ----------------------
NexText tStat      = NexText(PAGE_DASH,     1,  "tStat");
NexText tTime      = NexText(PAGE_DASH,     2,  "tTime");
NexText tTemp      = NexText(PAGE_DASH,     3,  "tTemp");
NexText tWeight    = NexText(PAGE_DASH,     4,  "tWeight");
NexText tVolume    = NexText(PAGE_DASH,     5,  "tVolume");
NexText tBatch     = NexText(PAGE_DASH,     6,  "tBatch");
NexText tTotWeight = NexText(PAGE_DASH,     7,  "tTotWeight");
NexText tFuel      = NexText(PAGE_DASH,     8,  "tFuel");
NexProgressBar hFuel = NexProgressBar(PAGE_DASH, 9,  "hFuel");

NexButton btAuto   = NexButton(PAGE_DASH,   10, "btAuto");
NexButton bStop    = NexButton(PAGE_DASH,   11, "bStop");

// ----------------------
// MANUAL objects
// ----------------------
NexButton bManDoor  = NexButton(PAGE_MANUAL, 1,  "bManDoor");
NexButton bManBDoor = NexButton(PAGE_MANUAL, 2,  "bManBDoor");
NexButton bManCon   = NexButton(PAGE_MANUAL, 3,  "bManCon");
NexButton bManPush  = NexButton(PAGE_MANUAL, 4,  "bManPush");

NexDSButton btManDir  = NexDSButton(PAGE_MANUAL, 5, "btManDir");
NexDSButton btManIgn  = NexDSButton(PAGE_MANUAL, 6, "btManIgn");
NexDSButton btManBurn = NexDSButton(PAGE_MANUAL, 7, "btManBurn");

// Sliders (kalau memang di HMI kamu slider; kalau sebenarnya "hManSpeed" adalah slider)
NexSlider hManSpeed = NexSlider(PAGE_MANUAL, 8, "hManSpeed");
NexSlider hManStep  = NexSlider(PAGE_MANUAL, 9, "hManStep");

NexText tManTemp    = NexText(PAGE_MANUAL, 10, "tManTemp");
NexText tManWeight  = NexText(PAGE_MANUAL, 11, "tManWeight");

NexButton b5StopManual = NexButton(PAGE_MANUAL, 12, "b5");

// ----------------------
// SETTINGS objects
// NOTE: kamu bilang ini TEXT, jadi pakai NexText
// ----------------------
NexText nTempSet = NexText(PAGE_SETTINGS, 1, "nTempSet");
NexText nTimeSet = NexText(PAGE_SETTINGS, 2, "nTimeSet");
NexText t2Batch  = NexText(PAGE_SETTINGS, 3, "t2");
NexText t1Vol    = NexText(PAGE_SETTINGS, 4, "t1");

NexText tBlow1txt = NexText(PAGE_SETTINGS, 5, "tBlow1txt");
NexText tBlow2txt = NexText(PAGE_SETTINGS, 6, "tBlow2txt");

NexButton bSave        = NexButton(PAGE_SETTINGS, 7, "bSave");
NexButton bReset       = NexButton(PAGE_SETTINGS, 8, "bReset");
NexButton bsettingStop = NexButton(PAGE_SETTINGS, 9, "bsettingStop");

// UP/DOWN buttons for setpoints
NexButton bTempUp  = NexButton(PAGE_SETTINGS, 10, "bTempUp");
NexButton bTempDn  = NexButton(PAGE_SETTINGS, 11, "bTempD");
NexButton bTimeUp  = NexButton(PAGE_SETTINGS, 12, "bTimeUp");
NexButton bTimeDn  = NexButton(PAGE_SETTINGS, 13, "bTimeD");
NexButton bBatchUp = NexButton(PAGE_SETTINGS, 14, "b2");
NexButton bBatchDn = NexButton(PAGE_SETTINGS, 15, "b4");
NexButton bVolUp   = NexButton(PAGE_SETTINGS, 16, "b1");
NexButton bVolDn   = NexButton(PAGE_SETTINGS, 17, "b0");

NexButton bBlow1   = NexButton(PAGE_SETTINGS, 18, "bBlow1");
NexButton bBlow2   = NexButton(PAGE_SETTINGS, 19, "bBlow2");

// Listener list (Nextion library requirement)
NexTouch *nex_listen_list[] = {
  &btAuto, &bStop,
  &bManDoor, &bManBDoor, &bManCon, &bManPush,
  &btManDir, &btManIgn, &btManBurn,
  &hManSpeed, &hManStep,
  &b5StopManual,
  &bSave, &bReset, &bsettingStop,
  &bTempUp, &bTempDn, &bTimeUp, &bTimeDn,
  &bBatchUp, &bBatchDn, &bVolUp, &bVolDn,
  &bBlow1, &bBlow2,
  NULL
};

// ======================
// UI OBJ ID MAP (for UI queue)
// ======================
enum UiObjId : uint16_t {
  UI_TSTAT=1, UI_TTIME, UI_TTEMP, UI_TWEIGHT,
  UI_TVOL, UI_TBATCH, UI_TTOTW, UI_TFUEL,
  UI_HFUEL,

  UI_TMAN_TEMP, UI_TMAN_WEIGHT,
  UI_DSB_DIR,

  UI_ST_TSET, UI_ST_TIME, UI_ST_BATCH, UI_ST_VOL,
  UI_ST_BLOW1, UI_ST_BLOW2
};

static void uiEnqTxt(UiObjId id, const String &s) {
  UiUpdate u{};
  u.kind = UI_TXT;
  u.objId = (uint16_t)id;
  strncpy(u.txt, s.c_str(), sizeof(u.txt)-1);
  xQueueSend(qUi, &u, 0);
}

static void uiEnqNum(UiObjId id, int32_t v) {
  UiUpdate u{};
  u.kind = UI_NUM;
  u.objId = (uint16_t)id;
  u.num = v;
  xQueueSend(qUi, &u, 0);
}

static void enqueueRs(uint8_t node, const String &payload) {
  RsTxMsg m{};
  m.node = node;
  strncpy(m.payload, payload.c_str(), sizeof(m.payload)-1);
  xQueueSend(qRsTx, &m, 0);
}

// ======================
// NEXTION SAFE SETTERS (centralized)
// ======================
static void nxSetText(NexText &obj, const String &s) {
  obj.setText(s.c_str());
}

static void nxSetProgress(NexProgressBar &obj, int v0_100) {
  if (v0_100 < 0) v0_100 = 0;
  if (v0_100 > 100) v0_100 = 100;
  obj.setValue((uint32_t)v0_100);
}

static void nxSetDs(NexDSButton &obj, bool on) {
  obj.setValue(on ? 1 : 0);
}

// Reading TEXT using library call (safer than manual 0x70 parsing)
static bool nxGetText(NexText &obj, String &out) {
  char buf[64] = {0};
  uint16_t len = sizeof(buf);
  if (obj.getText(buf, len)) {
    out = String(buf);
    out.trim();
    return true;
  }
  return false;
}

static float readTextAsFloatOrFallback(NexText &obj, float fallbackVal) {
  String txt;
  if (nxGetText(obj, txt)) {
    float v;
    if (parseFloatLoose(txt, v)) return v;
  }
  return fallbackVal;
}

static void writeTextValue(NexText &obj, float v, uint8_t decimals) {
  char b[32];
  formatFloatNoTrailingZeros(v, decimals, b, sizeof(b));
  nxSetText(obj, String(b));
}

static void writeTextValueInt(NexText &obj, int v) {
  nxSetText(obj, String(v));
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
          uint8_t id; String payload;
          if (parseFrame(buf, id, payload)) {
            if (xSemaphoreTake(gMutex, pdMS_TO_TICKS(20)) == pdTRUE) {
              if (id == NODE_TEMP) {
                if (payload.startsWith("TEMP:")) {
                  String v = payload.substring(5);
                  if (v == "OPEN" || v == "NaN") g.tempC = NAN;
                  else { float t = v.toFloat(); g.tempC = isfinite(t) ? t : NAN; }
                } else if (payload.startsWith("LIMITS:") || payload.startsWith("LIM_HB:")) {
                  int p = payload.indexOf(':');
                  int c1 = payload.indexOf(',');
                  if (p >= 0 && c1 > p+1) {
                    g.lim2_a = (uint8_t)payload.substring(p+1, c1).toInt();
                    g.lim2_b = (uint8_t)payload.substring(c1+1).toInt();
                  }
                }
              } else if (id == NODE_WEIGHT) {
                if (payload.startsWith("WEIGHT:")) {
                  String v = payload.substring(7);
                  if (v == "NaN") g.weightKg = NAN;
                  else { float w = v.toFloat(); g.weightKg = isfinite(w) ? w : NAN; }
                } else if (payload.startsWith("LIMITS:") || payload.startsWith("LIM_HB:")) {
                  int p = payload.indexOf(':');
                  int c1 = payload.indexOf(',');
                  if (p >= 0 && c1 > p+1) {
                    g.lim3_a = (uint8_t)payload.substring(p+1, c1).toInt();
                    g.lim3_b = (uint8_t)payload.substring(c1+1).toInt();
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
// NEXTION EVENT CALLBACKS (HMI lama style)
// ======================

// DASH
static void cb_btAuto(void *ptr) {
  if (xSemaphoreTake(gMutex, pdMS_TO_TICKS(20)) == pdTRUE) {
    g.estop = false;
    g.autoRunning = true;
    xSemaphoreGive(gMutex);
  }
  enqueueRs(NODE_MEGA_ACT, "CMD_RUN_AUTO");
}

static void cb_bStop(void *ptr) {
  if (xSemaphoreTake(gMutex, pdMS_TO_TICKS(20)) == pdTRUE) {
    g.estop = true;
    g.autoRunning = false;
    xSemaphoreGive(gMutex);
  }
  enqueueRs(NODE_MEGA_ACT, "CMD_EMERGENCY_STOP");
}

// MANUAL
static void cb_btManDir(void *ptr) {
  bool dirOpen = true;
  if (xSemaphoreTake(gMutex, pdMS_TO_TICKS(20)) == pdTRUE) {
    g.manDirOpen = !g.manDirOpen;
    dirOpen = g.manDirOpen;
    xSemaphoreGive(gMutex);
  }
  uiEnqNum(UI_DSB_DIR, dirOpen ? 1 : 0);
  enqueueRs(NODE_MEGA_ACT, String("MAN_DIR:") + (dirOpen ? "OPEN" : "CLOSE"));
}

static void cb_b5StopManual(void *ptr) {
  cb_bStop(ptr);
}

static void cb_bManDoor(void *ptr) {
  bool dirOpen = true;
  if (xSemaphoreTake(gMutex, pdMS_TO_TICKS(20)) == pdTRUE) {
    dirOpen = g.manDirOpen;
    xSemaphoreGive(gMutex);
  }
  enqueueRs(NODE_MEGA_ACT, dirOpen ? "MAINDOOR_OPEN" : "MAINDOOR_CLOSE");
}

static void cb_bManBDoor(void *ptr) {
  bool dirOpen = true;
  if (xSemaphoreTake(gMutex, pdMS_TO_TICKS(20)) == pdTRUE) {
    dirOpen = g.manDirOpen;
    xSemaphoreGive(gMutex);
  }
  enqueueRs(NODE_MEGA_ACT, dirOpen ? "BDOOR_OPEN" : "BDOOR_CLOSE");
}

static void cb_bManPush(void *ptr) {
  bool dirOpen = true;
  if (xSemaphoreTake(gMutex, pdMS_TO_TICKS(20)) == pdTRUE) {
    dirOpen = g.manDirOpen;
    xSemaphoreGive(gMutex);
  }
  enqueueRs(NODE_MEGA_ACT, dirOpen ? "PUSHER_MAJU" : "PUSHER_MUNDUR");
}

static void cb_bManCon(void *ptr) {
  static uint8_t convMode = 0; // 0 stop, 1 fwd, 2 rev
  convMode = (uint8_t)((convMode + 1) % 3);
  if (convMode == 0) enqueueRs(NODE_MEGA_ACT, "CONVEYOR_STOP");
  else if (convMode == 1) enqueueRs(NODE_MEGA_ACT, "CONVEYOR_FWD");
  else enqueueRs(NODE_MEGA_ACT, "CONVEYOR_REV");
}

static void cb_btManIgn(void *ptr) {
  uint32_t v=0;
  btManIgn.getValue(&v);
  enqueueRs(NODE_MEGA_ACT, String("CMD_MAN_IGN:") + String((int)v));
}

static void cb_btManBurn(void *ptr) {
  uint32_t v=0;
  btManBurn.getValue(&v);
  enqueueRs(NODE_MEGA_ACT, String("CMD_MAN_BURN:") + String((int)v));
}

static void cb_hManSpeed(void *ptr) {
  uint32_t v=0;
  hManSpeed.getValue(&v);
  if (xSemaphoreTake(gMutex, pdMS_TO_TICKS(20)) == pdTRUE) {
    g.manSpeed = (int)v;
    xSemaphoreGive(gMutex);
  }
  enqueueRs(NODE_MEGA_ACT, String("MAN_SPEED:") + String((unsigned)v));
}

static void cb_hManStep(void *ptr) {
  uint32_t v=0;
  hManStep.getValue(&v);
  if (xSemaphoreTake(gMutex, pdMS_TO_TICKS(20)) == pdTRUE) {
    g.manSteps = (int)v;
    xSemaphoreGive(gMutex);
  }
  enqueueRs(NODE_MEGA_ACT, String("MAN_STEPS:") + String((unsigned)v));
}

// SETTINGS
static void cb_bsettingStop(void *ptr) {
  cb_bStop(ptr);
}

static void cb_bTempUp(void *ptr) {
  float cur = 1200.0f;
  if (xSemaphoreTake(gMutex, pdMS_TO_TICKS(20)) == pdTRUE) { cur = g.setTemp; xSemaphoreGive(gMutex); }

  float v = readTextAsFloatOrFallback(nTempSet, cur);
  v = clampf(v + SP_TEMP_STEP, SP_TEMP_MIN, SP_TEMP_MAX);
  writeTextValue(nTempSet, v, 0);

  if (xSemaphoreTake(gMutex, pdMS_TO_TICKS(20)) == pdTRUE) { g.setTemp = v; xSemaphoreGive(gMutex); }
}

static void cb_bTempDn(void *ptr) {
  float cur = 1200.0f;
  if (xSemaphoreTake(gMutex, pdMS_TO_TICKS(20)) == pdTRUE) { cur = g.setTemp; xSemaphoreGive(gMutex); }

  float v = readTextAsFloatOrFallback(nTempSet, cur);
  v = clampf(v - SP_TEMP_STEP, SP_TEMP_MIN, SP_TEMP_MAX);
  writeTextValue(nTempSet, v, 0);

  if (xSemaphoreTake(gMutex, pdMS_TO_TICKS(20)) == pdTRUE) { g.setTemp = v; xSemaphoreGive(gMutex); }
}

static void cb_bTimeUp(void *ptr) {
  uint32_t cur = 30;
  if (xSemaphoreTake(gMutex, pdMS_TO_TICKS(20)) == pdTRUE) { cur = g.setTimeSec; xSemaphoreGive(gMutex); }

  float f = readTextAsFloatOrFallback(nTimeSet, (float)cur);
  int v = (int)roundf(f);
  v += SP_TIME_STEP;
  if (v < SP_TIME_MIN) v = SP_TIME_MIN;
  if (v > SP_TIME_MAX) v = SP_TIME_MAX;

  writeTextValueInt(nTimeSet, v);

  if (xSemaphoreTake(gMutex, pdMS_TO_TICKS(20)) == pdTRUE) { g.setTimeSec = (uint32_t)v; xSemaphoreGive(gMutex); }
}

static void cb_bTimeDn(void *ptr) {
  uint32_t cur = 30;
  if (xSemaphoreTake(gMutex, pdMS_TO_TICKS(20)) == pdTRUE) { cur = g.setTimeSec; xSemaphoreGive(gMutex); }

  float f = readTextAsFloatOrFallback(nTimeSet, (float)cur);
  int v = (int)roundf(f);
  v -= SP_TIME_STEP;
  if (v < SP_TIME_MIN) v = SP_TIME_MIN;
  if (v > SP_TIME_MAX) v = SP_TIME_MAX;

  writeTextValueInt(nTimeSet, v);

  if (xSemaphoreTake(gMutex, pdMS_TO_TICKS(20)) == pdTRUE) { g.setTimeSec = (uint32_t)v; xSemaphoreGive(gMutex); }
}

static void cb_bBatchUp(void *ptr) {
  float cur = 5.0f;
  if (xSemaphoreTake(gMutex, pdMS_TO_TICKS(20)) == pdTRUE) { cur = g.setBatchKg; xSemaphoreGive(gMutex); }

  float v = readTextAsFloatOrFallback(t2Batch, cur);
  v = clampf(v + SP_BATCH_STEP, SP_BATCH_MIN, SP_BATCH_MAX);
  writeTextValue(t2Batch, v, 2);

  if (xSemaphoreTake(gMutex, pdMS_TO_TICKS(20)) == pdTRUE) { g.setBatchKg = v; xSemaphoreGive(gMutex); }
}

static void cb_bBatchDn(void *ptr) {
  float cur = 5.0f;
  if (xSemaphoreTake(gMutex, pdMS_TO_TICKS(20)) == pdTRUE) { cur = g.setBatchKg; xSemaphoreGive(gMutex); }

  float v = readTextAsFloatOrFallback(t2Batch, cur);
  v = clampf(v - SP_BATCH_STEP, SP_BATCH_MIN, SP_BATCH_MAX);
  writeTextValue(t2Batch, v, 2);

  if (xSemaphoreTake(gMutex, pdMS_TO_TICKS(20)) == pdTRUE) { g.setBatchKg = v; xSemaphoreGive(gMutex); }
}

static void cb_bVolUp(void *ptr) {
  float cur = 0.0f;
  if (xSemaphoreTake(gMutex, pdMS_TO_TICKS(20)) == pdTRUE) { cur = g.setVolume; xSemaphoreGive(gMutex); }

  float v = readTextAsFloatOrFallback(t1Vol, cur);
  v = clampf(v + SP_VOL_STEP, SP_VOL_MIN, SP_VOL_MAX);
  writeTextValue(t1Vol, v, 2);

  if (xSemaphoreTake(gMutex, pdMS_TO_TICKS(20)) == pdTRUE) { g.setVolume = v; xSemaphoreGive(gMutex); }
}

static void cb_bVolDn(void *ptr) {
  float cur = 0.0f;
  if (xSemaphoreTake(gMutex, pdMS_TO_TICKS(20)) == pdTRUE) { cur = g.setVolume; xSemaphoreGive(gMutex); }

  float v = readTextAsFloatOrFallback(t1Vol, cur);
  v = clampf(v - SP_VOL_STEP, SP_VOL_MIN, SP_VOL_MAX);
  writeTextValue(t1Vol, v, 2);

  if (xSemaphoreTake(gMutex, pdMS_TO_TICKS(20)) == pdTRUE) { g.setVolume = v; xSemaphoreGive(gMutex); }
}

static void cb_bBlow1(void *ptr) {
  int lvl;
  if (xSemaphoreTake(gMutex, pdMS_TO_TICKS(20)) == pdTRUE) {
    g.blowerAirLvl = (g.blowerAirLvl + 1) % 3;
    lvl = g.blowerAirLvl;
    xSemaphoreGive(gMutex);
  } else lvl = 0;

  // UI text will be refreshed by renderer
  enqueueRs(NODE_MEGA_ACT, String("BLOW:air:") + String(lvl));
}

static void cb_bBlow2(void *ptr) {
  int lvl;
  if (xSemaphoreTake(gMutex, pdMS_TO_TICKS(20)) == pdTRUE) {
    g.blowerCoolLvl = (g.blowerCoolLvl + 1) % 3;
    lvl = g.blowerCoolLvl;
    xSemaphoreGive(gMutex);
  } else lvl = 0;

  enqueueRs(NODE_MEGA_ACT, String("BLOW:cool:") + String(lvl));
}

static void cb_bSave(void *ptr) {
  // safety net: pull from screen text before send
  float fv;
  String txt;

  if (nxGetText(nTempSet, txt) && parseFloatLoose(txt, fv)) {
    fv = clampf(fv, SP_TEMP_MIN, SP_TEMP_MAX);
    if (xSemaphoreTake(gMutex, pdMS_TO_TICKS(20)) == pdTRUE) { g.setTemp = fv; xSemaphoreGive(gMutex); }
  }

  if (nxGetText(nTimeSet, txt) && parseFloatLoose(txt, fv)) {
    int iv = (int)roundf(fv);
    if (iv < SP_TIME_MIN) iv = SP_TIME_MIN;
    if (iv > SP_TIME_MAX) iv = SP_TIME_MAX;
    if (xSemaphoreTake(gMutex, pdMS_TO_TICKS(20)) == pdTRUE) { g.setTimeSec = (uint32_t)iv; xSemaphoreGive(gMutex); }
  }

  if (nxGetText(t2Batch, txt) && parseFloatLoose(txt, fv)) {
    fv = clampf(fv, SP_BATCH_MIN, SP_BATCH_MAX);
    if (xSemaphoreTake(gMutex, pdMS_TO_TICKS(20)) == pdTRUE) { g.setBatchKg = fv; xSemaphoreGive(gMutex); }
  }

  if (nxGetText(t1Vol, txt) && parseFloatLoose(txt, fv)) {
    fv = clampf(fv, SP_VOL_MIN, SP_VOL_MAX);
    if (xSemaphoreTake(gMutex, pdMS_TO_TICKS(20)) == pdTRUE) { g.setVolume = fv; xSemaphoreGive(gMutex); }
  }

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
  enqueueRs(NODE_MEGA_ACT, String("BLOW:air:")  + String(b1));
  enqueueRs(NODE_MEGA_ACT, String("BLOW:cool:") + String(b2));
}

static void cb_bReset(void *ptr) {
  if (xSemaphoreTake(gMutex, pdMS_TO_TICKS(20)) == pdTRUE) {
    g.setTemp = 1200.0f;
    g.setTimeSec = 30;
    g.setBatchKg = 5.0f;
    g.setVolume = 0;
    g.blowerAirLvl = 0;
    g.blowerCoolLvl = 0;
    xSemaphoreGive(gMutex);
  }

  writeTextValue(nTempSet, 1200.0f, 0);
  writeTextValueInt(nTimeSet, 30);
  writeTextValue(t2Batch, 5.0f, 2);
  writeTextValue(t1Vol, 0.0f, 2);
}

// ======================
// TASK: NEXTION LOOP (runs nexLoop safely)
// ======================
static void taskNexLoop(void *pv) {
  for (;;) {
    nexLoop(nex_listen_list);
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

// ======================
// TASK: UI FLUSH (single writer to Nextion objects)
// ======================
static void taskUiFlush(void *pv) {
  UiUpdate u;
  for (;;) {
    while (xQueueReceive(qUi, &u, 0) == pdTRUE) {
      if (u.kind == UI_TXT) {
        switch ((UiObjId)u.objId) {
          case UI_TSTAT:      nxSetText(tStat, String(u.txt)); break;
          case UI_TTIME:      nxSetText(tTime, String(u.txt)); break;
          case UI_TTEMP:      nxSetText(tTemp, String(u.txt)); break;
          case UI_TWEIGHT:    nxSetText(tWeight, String(u.txt)); break;
          case UI_TVOL:       nxSetText(tVolume, String(u.txt)); break;
          case UI_TBATCH:     nxSetText(tBatch, String(u.txt)); break;
          case UI_TTOTW:      nxSetText(tTotWeight, String(u.txt)); break;
          case UI_TFUEL:      nxSetText(tFuel, String(u.txt)); break;
          case UI_TMAN_TEMP:  nxSetText(tManTemp, String(u.txt)); break;
          case UI_TMAN_WEIGHT:nxSetText(tManWeight, String(u.txt)); break;
          case UI_ST_TSET:    nxSetText(nTempSet, String(u.txt)); break;
          case UI_ST_TIME:    nxSetText(nTimeSet, String(u.txt)); break;
          case UI_ST_BATCH:   nxSetText(t2Batch, String(u.txt)); break;
          case UI_ST_VOL:     nxSetText(t1Vol, String(u.txt)); break;
          case UI_ST_BLOW1:   nxSetText(tBlow1txt, String(u.txt)); break;
          case UI_ST_BLOW2:   nxSetText(tBlow2txt, String(u.txt)); break;
          default: break;
        }
      } else {
        switch ((UiObjId)u.objId) {
          case UI_HFUEL:    nxSetProgress(hFuel, (int)u.num); break;
          case UI_DSB_DIR:  nxSetDs(btManDir, u.num != 0); break;
          default: break;
        }
      }
      vTaskDelay(pdMS_TO_TICKS(2));
    }
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

// ======================
// TASK: POLLER
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
      uiEnqTxt(UI_TTIME, String(b));
    }

    vTaskDelay(dt);
  }
}

// ======================
// TASK: UI RENDER (push shared state into UI queue)
// ======================
static void taskUiRender(void *pv) {
  const TickType_t dt = pdMS_TO_TICKS(250);
  for (;;) {
    SharedState s;
    if (xSemaphoreTake(gMutex, pdMS_TO_TICKS(30)) == pdTRUE) {
      s = g;
      xSemaphoreGive(gMutex);
    }

    uiEnqTxt(UI_TSTAT, stateToText(s.autoRunning, s.estop));
    uiEnqTxt(UI_TTEMP, fmt1(s.tempC));
    uiEnqTxt(UI_TWEIGHT, fmt2(s.weightKg));

    uiEnqTxt(UI_TVOL, fmt1(s.totalVolume));
    uiEnqTxt(UI_TBATCH, String(s.totalBatch));
    uiEnqTxt(UI_TTOTW, fmt2(s.totalWeight));

    uiEnqTxt(UI_TFUEL, String((int)s.fuelPct) + "%");
    uiEnqNum(UI_HFUEL, (int)s.fuelPct);

    uiEnqTxt(UI_TMAN_TEMP, fmt1(s.tempC));
    uiEnqTxt(UI_TMAN_WEIGHT, fmt2(s.weightKg));

    // mirror settings (TEXT objects)
    {
      char b[32];
      formatFloatNoTrailingZeros(s.setTemp, 0, b, sizeof(b));
      uiEnqTxt(UI_ST_TSET, String(b));
    }
    uiEnqTxt(UI_ST_TIME, String((unsigned)s.setTimeSec));
    {
      char b[32];
      formatFloatNoTrailingZeros(s.setBatchKg, 2, b, sizeof(b));
      uiEnqTxt(UI_ST_BATCH, String(b));
    }
    {
      char b[32];
      formatFloatNoTrailingZeros(s.setVolume, 2, b, sizeof(b));
      uiEnqTxt(UI_ST_VOL, String(b));
    }

    auto lvlToTxt = [](int lvl) -> String {
      if (lvl <= 0) return "OFF";
      if (lvl == 1) return "LOW";
      return "HIGH";
    };
    uiEnqTxt(UI_ST_BLOW1, lvlToTxt(s.blowerAirLvl));
    uiEnqTxt(UI_ST_BLOW2, lvlToTxt(s.blowerCoolLvl));

    // mirror DSButton dir
    uiEnqNum(UI_DSB_DIR, s.manDirOpen ? 1 : 0);

    vTaskDelay(dt);
  }
}

// ======================
// HMI INIT (boot)
// ======================
static void hmiBootInit() {
  sendCommand("page 0");

  tStat.setText("IDLE");
  tTemp.setText("IDLE");
  tWeight.setText("IDLE");
  tVolume.setText("0");
  tBatch.setText("0");
  tTotWeight.setText("0");
  tFuel.setText("0%");
  hFuel.setValue(0);

  // init DSButton states
  btManDir.setValue(g.manDirOpen ? 1 : 0);

  // init setpoints
  writeTextValue(nTempSet, g.setTemp, 0);
  writeTextValueInt(nTimeSet, (int)g.setTimeSec);
  writeTextValue(t2Batch, g.setBatchKg, 2);
  writeTextValue(t1Vol, g.setVolume, 2);
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
  nexInit();

  // primitives
  gMutex = xSemaphoreCreateMutex();
  qRsTx  = xQueueCreate(24, sizeof(RsTxMsg));
  qUi    = xQueueCreate(48, sizeof(UiUpdate));

  if (xSemaphoreTake(gMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
    g.tempC = NAN;
    g.weightKg = NAN;
    g.autoRunning = false;
    g.estop = false;
    g.fuelPct = 0;
    xSemaphoreGive(gMutex);
  }

  // Attach callbacks (release = attachPop)
  btAuto.attachPop(cb_btAuto, &btAuto);
  bStop.attachPop(cb_bStop, &bStop);

  btManDir.attachPop(cb_btManDir, &btManDir);
  b5StopManual.attachPop(cb_b5StopManual, &b5StopManual);
  bManDoor.attachPop(cb_bManDoor, &bManDoor);
  bManBDoor.attachPop(cb_bManBDoor, &bManBDoor);
  bManPush.attachPop(cb_bManPush, &bManPush);
  bManCon.attachPop(cb_bManCon, &bManCon);

  btManIgn.attachPop(cb_btManIgn, &btManIgn);
  btManBurn.attachPop(cb_btManBurn, &btManBurn);

  hManSpeed.attachPop(cb_hManSpeed, &hManSpeed);
  hManStep.attachPop(cb_hManStep, &hManStep);

  bsettingStop.attachPop(cb_bsettingStop, &bsettingStop);
  bTempUp.attachPop(cb_bTempUp, &bTempUp);
  bTempDn.attachPop(cb_bTempDn, &bTempDn);
  bTimeUp.attachPop(cb_bTimeUp, &bTimeUp);
  bTimeDn.attachPop(cb_bTimeDn, &bTimeDn);
  bBatchUp.attachPop(cb_bBatchUp, &bBatchUp);
  bBatchDn.attachPop(cb_bBatchDn, &bBatchDn);
  bVolUp.attachPop(cb_bVolUp, &bVolUp);
  bVolDn.attachPop(cb_bVolDn, &bVolDn);
  bBlow1.attachPop(cb_bBlow1, &bBlow1);
  bBlow2.attachPop(cb_bBlow2, &bBlow2);

  bSave.attachPop(cb_bSave, &bSave);
  bReset.attachPop(cb_bReset, &bReset);

  hmiBootInit();

  // Tasks
  xTaskCreatePinnedToCore(taskRsTx,     "rsTx",     4096, nullptr, 3, nullptr, 1);
  xTaskCreatePinnedToCore(taskRsRx,     "rsRx",     4096, nullptr, 3, nullptr, 1);
  xTaskCreatePinnedToCore(taskNexLoop,  "nexLoop",  4096, nullptr, 2, nullptr, 0);
  xTaskCreatePinnedToCore(taskUiFlush,  "uiFlush",  4096, nullptr, 1, nullptr, 0);
  xTaskCreatePinnedToCore(taskPoller,   "poller",   4096, nullptr, 1, nullptr, 1);
  xTaskCreatePinnedToCore(taskUiRender, "uiRender", 4096, nullptr, 1, nullptr, 0);

  enqueueRs(NODE_TEMP, "REQ_ALL");
  enqueueRs(NODE_WEIGHT, "REQ_ALL");

  Serial.println("[MASTER] READY (Nextion library mode)");
}

void loop() {
  vTaskDelay(pdMS_TO_TICKS(1000));
}

/*
 * END.
 */
