#include <Arduino.h>
#include "HX711.h"

/// Program Slave RS485 + HX711 + 7-Segment (NODE 3)
/// Pola LIMIT dibuat identik dengan Slave 2:
/// - LIMITS: event on change (langsung kirim saat berubah)
/// - LIM_HB: heartbeat periodik (di sini 500ms)
/// - Event diberi rate-limit kecil agar tidak spam bila ada noise
///
/// Weight:
/// - WEIGHT periodic (default 500 ms)
/// - SIMULATE_HX711: generate nilai random jika HX711 belum ditancap

// =========================
// DEBUG / SIMULASI
// =========================
#define DEBUG_SERIAL   1   // 1=print debug ke Serial Monitor
#define SIMULATE_HX711 1   // 1=pakai random weight, 0=pakai HX711 asli

#if DEBUG_SERIAL
  #define DBG(x)   Serial.print(x)
  #define DBGLN(x) Serial.println(x)
#else
  #define DBG(x)
  #define DBGLN(x)
#endif

// =========================
// 7-SEGMENT PINS
// =========================
#define DATA_PIN  14
#define CLOCK_PIN 13
#define LATCH_PIN 12

// =========================
// NODE / RS485
// =========================
static const uint8_t NODE_ID = 3;
static const int PIN_RS485_DE = 4;
static const int RS485_RX = 25;
static const int RS485_TX = 26;

float weightValue = 0;

// =========================
// 7-SEG MAPPING
// =========================
static byte segDigit14[10] = {
  0b00101000, 0b01111110, 0b00110001, 0b00110100, 0b01100110,
  0b10100100, 0b10100000, 0b00111110, 0b00100000, 0b00100100
};

static byte segDigit2[10] = {
  0b00001000, 0b01011110, 0b00010001, 0b00010100, 0b01000110,
  0b10000100, 0b10000000, 0b00011110, 0b00000000, 0b00000100
};

static byte segDigit3[10] = {
  0b00101000, 0b11101011, 0b00110001, 0b10100001, 0b11100010,
  0b10100100, 0b00100100, 0b11101001, 0b00100000, 0b10100000
};

static inline void sendByte(byte b) {
  digitalWrite(LATCH_PIN, LOW);
  shiftOut(DATA_PIN, CLOCK_PIN, MSBFIRST, b);
  digitalWrite(LATCH_PIN, HIGH);
}

static void pesanError(){
  byte tandaMin = 0b11110111;
  byte hurufE   = 0b10100001;
  byte hurufR1  = 0b01111100;
  byte hurufR2  = 0b10101011;

  sendByte(~tandaMin);
  sendByte(~hurufE);
  sendByte(~hurufR1);
  sendByte(~hurufR2);
}

static void convertMassValueto7Segment(float massValue) {
  if (!isfinite(massValue) || massValue > 100.0f) {
    pesanError();
    return;
  }

  if (massValue < 0) massValue = 0;

  int v = (int)massValue;
  int d1 = (v / 1000) % 10;
  int d2 = (v / 100)  % 10;
  int d3 = (v / 10)   % 10;
  int d4 =  v % 10;

  const byte SEG_KOSONG = 0b11111111;

  byte o1 = (d1 == 0) ? SEG_KOSONG : segDigit14[d1];
  byte o2;
  if (d1 == 0 && d2 == 0) o2 = SEG_KOSONG;
  else o2 = segDigit14[d2];

  byte o3;
  if (d1 == 0 && d2 == 0 && d3 == 0) o3 = SEG_KOSONG;
  else o3 = segDigit3[d3];

  byte o4 = segDigit14[d4];

  sendByte(~o1);
  sendByte(~o2);
  sendByte(~o3);
  sendByte(~o4);
}

static void startupBlink() {
  byte allOn  = ~0b00000000;
  byte allOff = ~0b11111111;

  for (int i = 0; i < 3; i++) {
    sendByte(allOn); sendByte(allOn); sendByte(allOn); sendByte(allOn);
    delay(200);
    sendByte(allOff); sendByte(allOff); sendByte(allOff); sendByte(allOff);
    delay(200);
  }
}

// =========================
// RS485 SIMPLE
// =========================
struct RS485Simple {
  HardwareSerial *ser = nullptr;
  int pinDE = -1;
  uint32_t baud = 9600;

  void begin(HardwareSerial &s, uint32_t b, int rx, int tx, int dePin) {
    ser = &s;
    baud = b;
    pinDE = dePin;
    pinMode(pinDE, OUTPUT);
    digitalWrite(pinDE, LOW);
    ser->begin(baud, SERIAL_8N1, rx, tx);
  }

  void txMode(bool en) { digitalWrite(pinDE, en ? HIGH : LOW); }

  void sendLine(const String &line) {
    txMode(true);
    ser->print(line);
    ser->print("\n");
    ser->flush();
    txMode(false);
  }

  bool readLine(String &out) {
    while (ser->available()) {
      char c = (char)ser->read();
      if (c == '\r') continue;
      if (c == '\n') {
        out.trim();
        return out.length() > 0;
      }
      out += c;
      if (out.length() > 256) out = "";
    }
    return false;
  }
};

static RS485Simple bus;
static String rxLine;

static inline void sendToMaster(const String &payload) {
  char hdr[8];
  snprintf(hdr, sizeof(hdr), "N%02u:", (unsigned)NODE_ID);
  String msg = String(hdr) + payload;

#if DEBUG_SERIAL
  Serial.print("[RS485 TX] ");
  Serial.println(msg);
#endif

  bus.sendLine(msg);
}

static bool parseToFrame(const String &line, uint8_t &idOut, String &payloadOut) {
  // Expect: TOxx:payload
  if (line.length() < 6) return false;
  if (!(line[0] == 'T' && line[1] == 'O')) return false;
  char d1 = line[2], d2 = line[3];
  if (d1 < '0' || d1 > '9' || d2 < '0' || d2 > '9') return false;
  if (line[4] != ':') return false;
  idOut = (uint8_t)((d1 - '0') * 10 + (d2 - '0'));
  payloadOut = line.substring(5);
  return true;
}

// =========================
// HX711
// =========================
static const int HX711_DT  = 21;
static const int HX711_SCK = 22;

static HX711 scale;
static float scale_factor = 2123.157f;

// smoothing EMA
static float wEma = 0;
static bool  wInit = false;
static const float W_ALPHA = 0.18f;

// runtime online flag
static bool gHx711Online = !SIMULATE_HX711;

static float simulateWeightKg() {
  static bool init = false;
  static float w = 0;
  if (!init) {
    init = true;
    randomSeed(micros());
    w = (float)random(0, 2000) / 100.0f; // 0.00 - 20.00
  }
  w += (float)random(-20, 21) / 100.0f;  // -0.20 .. +0.20
  if (w < 0) w = 0;
  if (w > 99.99f) w = 99.99f;
  return w;
}

static float readWeightKg() {
#if SIMULATE_HX711
  // pakai simulasi agar sistem tetap hidup
  weightValue = simulateWeightKg();
  if (!wInit) { wInit = true; wEma = weightValue; }
  wEma = (W_ALPHA * weightValue) + ((1.0f - W_ALPHA) * wEma);
  return wEma;
#else
  if (!gHx711Online) return NAN;
  if (!scale.is_ready()) return NAN;

  weightValue = scale.get_units(1);
  if (!isfinite(weightValue)) return NAN;

  if (!wInit) { wInit = true; wEma = weightValue; }
  wEma = (W_ALPHA * weightValue) + ((1.0f - W_ALPHA) * wEma);

  if (wEma < 0 && wEma > -0.2f) wEma = 0;
  return wEma;
#endif
}

// =========================
// LIMIT SWITCH (DOOR & PUSH)
// =========================
// aktif LOW (INPUT_PULLUP)
static const int LIM_DOOR = 27;
static const int LIM_PUSH = 33;

static inline uint8_t limActive(int pin) { return (digitalRead(pin) == LOW) ? 1 : 0; }

static const uint32_t DEBOUNCE_MS = 25;
static uint8_t stDoor = 0, stPush = 0;
static uint8_t lastSentDoor = 255, lastSentPush = 255;
static uint8_t rdDoor = 0, rdPush = 0;
static uint32_t tDoor = 0, tPush = 0;

static void sampleLimitsDebounced() {
  uint32_t now = millis();
  uint8_t r;

  r = limActive(LIM_DOOR);
  if (r != rdDoor) { rdDoor = r; tDoor = now; }
  if ((now - tDoor) >= DEBOUNCE_MS) stDoor = rdDoor;

  r = limActive(LIM_PUSH);
  if (r != rdPush) { rdPush = r; tPush = now; }
  if ((now - tPush) >= DEBOUNCE_MS) stPush = rdPush;
}

// IDENTIK konsep dengan Slave 2
static const uint32_t LIMIT_HB_MS = 500;     // heartbeat 500ms
static uint32_t lastLimitHb = 0;

static const uint32_t LIMIT_EVENT_MIN_MS = 50; // anti spam event
static uint32_t lastLimitEvent = 0;

static void sendLimitHeartbeatAndEvent() {
  uint32_t now = millis();

  // EVENT: kirim cepat bila berubah (debounced) + rate limit
  if ((stDoor != lastSentDoor || stPush != lastSentPush) &&
      (now - lastLimitEvent >= LIMIT_EVENT_MIN_MS)) {

    lastLimitEvent = now;

#if DEBUG_SERIAL
    Serial.print("[LIMIT EVENT] Door="); Serial.print(stDoor);
    Serial.print(" Push="); Serial.println(stPush);
#endif

    // event payload
    sendToMaster(String("LIMITS:") + String((int)stDoor) + "," + String((int)stPush));

    // update last sent
    lastSentDoor = stDoor;
    lastSentPush = stPush;
  }

  // HEARTBEAT: kirim periodik
  if (now - lastLimitHb >= LIMIT_HB_MS) {
    lastLimitHb = now;

#if DEBUG_SERIAL
    Serial.print("[LIMIT HB] Door="); Serial.print(stDoor);
    Serial.print(" Push="); Serial.println(stPush);
#endif

    sendToMaster(String("LIM_HB:") + String((int)stDoor) + "," + String((int)stPush));
  }
}

// =========================
// WEIGHT periodic TX
// =========================
static const uint32_t WEIGHT_TX_MS = 500;
static uint32_t lastWeightTx = 0;

static void sendWeightPeriodic() {
  uint32_t now = millis();
  if (now - lastWeightTx < WEIGHT_TX_MS) return;
  lastWeightTx = now;

  float w = readWeightKg();
  if (isfinite(w)) {
#if DEBUG_SERIAL
    Serial.print("[WEIGHT] ");
    Serial.println(w, 2);
#endif
    sendToMaster(String("WEIGHT:") + String(w, 2));
  } else {
#if DEBUG_SERIAL
    Serial.println("[WEIGHT] NaN");
#endif
    sendToMaster("WEIGHT:NaN");
  }

  // display pakai nilai terakhir
  convertMassValueto7Segment(weightValue);
}

// =========================
// RX handler
// =========================
static void handlePayload(const String &payload) {
#if DEBUG_SERIAL
  Serial.print("[RS485 RX CMD] ");
  Serial.println(payload);
#endif

  if (payload == "REQ_WEIGHT") {
    float w = readWeightKg();
    if (isfinite(w)) sendToMaster(String("WEIGHT:") + String(w, 2));
    else sendToMaster("WEIGHT:NaN");
    return;
  }

  if (payload == "REQ_LIMITS") {
    sendToMaster(String("LIMITS:") + String((int)stDoor) + "," + String((int)stPush));
    lastSentDoor = stDoor;
    lastSentPush = stPush;
    return;
  }

  if (payload == "REQ_ALL") {
    float w = readWeightKg();
    if (isfinite(w)) sendToMaster(String("WEIGHT:") + String(w, 2));
    else sendToMaster("WEIGHT:NaN");

    sendToMaster(String("LIMITS:") + String((int)stDoor) + "," + String((int)stPush));
    lastSentDoor = stDoor;
    lastSentPush = stPush;
    return;
  }

  if (payload == "TARE") {
#if SIMULATE_HX711
    sendToMaster("LOG:INFO,SIM,TARE_IGNORED");
#else
    if (gHx711Online) {
      scale.tare(20);
      wInit = false;
      sendToMaster("LOG:INFO,HX711,TARED");
    } else {
      sendToMaster("LOG:WARN,HX711,NOT_READY");
    }
#endif
    return;
  }

  if (payload == "PING") {
    sendToMaster("PONG");
    return;
  }

  sendToMaster(String("LOG:WARN,UNK,") + payload);
}

static void processRx() {
  while (bus.readLine(rxLine)) {
    uint8_t id = 0;
    String payload;
    if (!parseToFrame(rxLine, id, payload)) { rxLine = ""; continue; }
    if (id != NODE_ID) { rxLine = ""; continue; }
    handlePayload(payload);
    rxLine = "";
  }
}

// =========================
// setup / loop
// =========================
void setup() {
  Serial.begin(115200);
  delay(200);
  DBGLN("[SLAVE3] Boot...");

  pinMode(DATA_PIN, OUTPUT);
  pinMode(CLOCK_PIN, OUTPUT);
  pinMode(LATCH_PIN, OUTPUT);

  startupBlink();

  bus.begin(Serial1, 9600, RS485_RX, RS485_TX, PIN_RS485_DE);
  DBGLN("[SLAVE3] RS485 started");

  // HX711 init
  scale.begin(HX711_DT, HX711_SCK);

#if SIMULATE_HX711
  gHx711Online = false;
  DBGLN("[SLAVE3] SIMULATE_HX711=1 -> random weight");
#else
  const uint32_t t0 = millis();
  while (!scale.is_ready() && (millis() - t0) < 1500) {
    delay(5);
  }
  if (!scale.is_ready()) {
    gHx711Online = false;
    DBGLN("[SLAVE3] HX711 NOT READY");
  } else {
    gHx711Online = true;
    scale.set_scale(scale_factor);
    scale.tare(20);
    DBGLN("[SLAVE3] HX711 init done");
  }
#endif

  // Limits
  pinMode(LIM_DOOR, INPUT_PULLUP);
  pinMode(LIM_PUSH, INPUT_PULLUP);
  sampleLimitsDebounced();
  delay(300);

  sendToMaster("LOG:INFO,BOOT,SLAVE3_READY");

  // initial snapshot
  sendToMaster(String("LIMITS:") + String((int)stDoor) + "," + String((int)stPush));
  lastSentDoor = stDoor;
  lastSentPush = stPush;

  float w = readWeightKg();
  if (isfinite(w)) sendToMaster(String("WEIGHT:") + String(w, 2));
  else sendToMaster("WEIGHT:NaN");
}

void loop() {
  processRx();

  // polling limit cepat (debounce)
  static uint32_t lastLimPoll = 0;
  uint32_t now = millis();
  if (now - lastLimPoll >= 5) {
    lastLimPoll = now;
    sampleLimitsDebounced();
  }

  // identik dengan Slave 2: event + heartbeat
  sendLimitHeartbeatAndEvent();

  // weight periodic
  sendWeightPeriodic();
}