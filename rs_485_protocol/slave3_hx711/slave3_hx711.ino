#include <Arduino.h>
#include "HX711.h"

// =========================
// NODE / RS485
// =========================
static const uint8_t NODE_ID = 2;
static const int PIN_RS485_DE = 4;
static const int RS485_RX = 16;
static const int RS485_TX = 17;

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
    ser->print('\n');
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

RS485Simple bus;
String rxLine;

// =========================
// HX711
// =========================
// HX711 pins (ubah sesuai wiring)
static const int HX711_DT  = 32;
static const int HX711_SCK = 33;

HX711 scale;
float scale_factor = 2123.157f;

// smoothing EMA
static float wEma = 0;
static bool  wInit = false;
static const float W_ALPHA = 0.18f;

// =========================
// LIMIT SWITCH (ubah sesuai wiring)
// =========================
// aktif LOW (INPUT_PULLUP)
static const int LIM_DOOR  = 25;
static const int LIM_PUSH  = 26;
static const int LIM_BURN  = 27;
static const int LIM_BDOOR = 14;

static inline uint8_t limActive(int pin) { return (digitalRead(pin) == LOW) ? 1 : 0; }

// debounce
static const uint32_t DEBOUNCE_MS = 25;
uint8_t stDoor=0, stPush=0, stBurn=0, stBDoor=0;
uint8_t lastSentDoor=255, lastSentPush=255, lastSentBurn=255, lastSentBDoor=255;

uint8_t rdDoor=0, rdPush=0, rdBurn=0, rdBDoor=0;
uint32_t tDoor=0, tPush=0, tBurn=0, tBDoor=0;

// =========================
// TX helpers
// =========================
static inline void sendToMaster(const String &payload) {
  char hdr[8];
  snprintf(hdr, sizeof(hdr), "N%02u:", (unsigned)NODE_ID);
  bus.sendLine(String(hdr) + payload);
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
// HX711 read
// =========================
float readWeightKg() {
  if (!scale.is_ready()) return NAN;

  float w = scale.get_units(1);
  if (!isfinite(w)) return NAN;

  if (!wInit) { wInit = true; wEma = w; }
  wEma = (W_ALPHA * w) + ((1.0f - W_ALPHA) * wEma);

  if (wEma < 0 && wEma > -0.2f) wEma = 0;
  return wEma;
}

static void sendWeightNow() {
  float w = readWeightKg();
  if (isfinite(w)) sendToMaster(String("WEIGHT:") + String(w, 2));
  else sendToMaster("WEIGHT:NaN");
}

// =========================
// LIMIT sampling
// =========================
static void sampleLimitsDebounced() {
  uint32_t now = millis();
  uint8_t r;

  r = limActive(LIM_DOOR);
  if (r != rdDoor) { rdDoor = r; tDoor = now; }
  if ((now - tDoor) >= DEBOUNCE_MS) stDoor = rdDoor;

  r = limActive(LIM_PUSH);
  if (r != rdPush) { rdPush = r; tPush = now; }
  if ((now - tPush) >= DEBOUNCE_MS) stPush = rdPush;

  r = limActive(LIM_BURN);
  if (r != rdBurn) { rdBurn = r; tBurn = now; }
  if ((now - tBurn) >= DEBOUNCE_MS) stBurn = rdBurn;

  r = limActive(LIM_BDOOR);
  if (r != rdBDoor) { rdBDoor = r; tBDoor = now; }
  if ((now - tBDoor) >= DEBOUNCE_MS) stBDoor = rdBDoor;
}

static void sendLimitsNow() {
  sendToMaster(String("LIMITS:") +
               String((int)stDoor) + "," +
               String((int)stPush) + "," +
               String((int)stBurn) + "," +
               String((int)stBDoor));
  lastSentDoor  = stDoor;
  lastSentPush  = stPush;
  lastSentBurn  = stBurn;
  lastSentBDoor = stBDoor;
}

static void sendLimitsOnChange() {
  if (stDoor!=lastSentDoor || stPush!=lastSentPush || stBurn!=lastSentBurn || stBDoor!=lastSentBDoor) {
    sendLimitsNow();
  }
}

// =========================
// RX payload handler
// =========================
static void handlePayload(const String &payload) {
  if (payload == "REQ_WEIGHT") {
    sendWeightNow();
    return;
  }
  if (payload == "TARE") {
    scale.tare(20);
    wInit = false; // reset EMA biar ga “nyangkut”
    sendToMaster("LOG:INFO,HX711,TARED");
    return;
  }
  if (payload == "REQ_LIMITS") {
    sendLimitsNow();
    return;
  }
  if (payload == "REQ_ALL") {
    sendWeightNow();
    sendLimitsNow();
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
    if (!parseToFrame(rxLine, id, payload)) { rxLine=""; continue; }
    if (id != NODE_ID) { rxLine=""; continue; }
    handlePayload(payload);
    rxLine="";
  }
}

// =========================
// setup / loop
// =========================
void setup() {
  Serial.begin(115200);
  bus.begin(Serial1, 9600, RS485_RX, RS485_TX, PIN_RS485_DE);

  // HX711
  scale.begin(HX711_DT, HX711_SCK);
  scale.set_scale(scale_factor);
  scale.tare(20);

  // Limits
  pinMode(LIM_DOOR,  INPUT_PULLUP);
  pinMode(LIM_PUSH,  INPUT_PULLUP);
  pinMode(LIM_BURN,  INPUT_PULLUP);
  pinMode(LIM_BDOOR, INPUT_PULLUP);

  sampleLimitsDebounced();

  sendToMaster("LOG:INFO,BOOT,SLAVE2_HX711_LIMIT_READY");
  sendLimitsNow();
  sendWeightNow();
}

void loop() {
  processRx();

  // Limit polling cepat + change detect
  static uint32_t lastLimPoll = 0;
  uint32_t now = millis();
  if (now - lastLimPoll >= 5) {
    lastLimPoll = now;
    sampleLimitsDebounced();
    sendLimitsOnChange();
  }

  // OPTIONAL: weight telemetry periodik (biar master ga perlu request terus)
  // Kalau kamu ga mau periodik, set interval besar atau comment block ini.
  static uint32_t lastWTx = 0;
  if (now - lastWTx >= 500) {
    lastWTx = now;
    sendWeightNow();
  }
}
