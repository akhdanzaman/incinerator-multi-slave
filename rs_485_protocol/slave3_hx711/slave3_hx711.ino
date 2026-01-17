#include <Arduino.h>
#include "HX711.h"
///TESSSTTT ARDAN
// =========================
// NODE / RS485
// =========================

#define DATA_PIN 14
#define CLOCK_PIN 13
#define LATCH_PIN 12

static const uint8_t NODE_ID = 3;
static const int PIN_RS485_DE = 4;
static const int RS485_RX = 25;
static const int RS485_TX = 26;

byte outX1, outX2, outY1, outY2;   // hasil konversi 7 Segment siap pakai
float weightValue = 0;

//Dibuat 3 fungsi Array untuk mapping tiap-tiap digit karena setiap digit pada seven segment punya maping yang berbeda-beda
byte segDigit14[10] = { //maping untuk digit 1 dan 4
  0b00101000, // 0
  0b01111110, // 1
  0b00110001, // 2  
  0b00110100, // 3
  0b01100110, // 4
  0b10100100, // 5
  0b10100000, // 6
  0b00111110, // 7
  0b00100000, // 8
  0b00100100  // 9
};

byte segDigit2[10] = { //maping untuk digit 2 (mirip seperti digit 1 dan 4, hanya saja segment dot-nya selalu menyala sebagai tanda koma)
  0b00001000, // 0
  0b01011110, // 1
  0b00010001, // 2  
  0b00010100, // 3
  0b01000110, // 4
  0b10000100, // 5
  0b10000000, // 6
  0b00011110, // 7
  0b00000000, // 8
  0b00000100  // 9
};

byte segDigit3[10] = { //maping untuk digit 3
  0b00101000, // 0
  0b11101011, // 1
  0b00110001, // 2  
  0b10100001, // 3
  0b11100010, // 4
  0b10100100, // 5
  0b00100100, // 6
  0b11101001, // 7
  0b00100000, // 8
  0b10100000  // 9
};

void sendByte(byte b) {
  digitalWrite(LATCH_PIN, LOW);
  shiftOut(DATA_PIN, CLOCK_PIN, MSBFIRST, b);
  digitalWrite(LATCH_PIN, HIGH);
}

void convertMassValueto7Segment(float massValue) {

  // --- ERROR CHECK ---
  // Jika suhu melebihi batas aman → tampilkan pesan error
  if (massValue > 100.0f) {
    pesanError();
    return;   // Stop agar tidak lanjut ke proses konversi normal
  }

  // Safety untuk nilai negatif
  if (massValue < 0) massValue = 0;

  // Ambil nilai integer saja untuk suhu
  int tempIntVal = (int)massValue;

  // Pecah digit ribuan → satuan
  int tX1 = (tempIntVal / 1000) % 10;  // ribuan
  int tX2 = (tempIntVal / 100)  % 10;  // ratusan
  int tX3 = (tempIntVal / 10)   % 10;  // puluhan
  int tX4 = tempIntVal % 10;           // satuan

  // Nilai 7-segment kosong
  const byte SEG_T_KOSONG = 0b11111111;

  // Output buffer untuk 4 digit suhu
  byte tOut1, tOut2, tOut3, tOut4;

  // Rules: leading zero = kosong
  tOut1 = (tX1 == 0) ? SEG_T_KOSONG : segDigit14[tX1];

  if (tX1 == 0 && tX2 == 0)
    tOut2 = SEG_T_KOSONG;
  else
    tOut2 = segDigit14[tX2];

  if (tX1 == 0 && tX2 == 0 && tX3 == 0)
    tOut3 = SEG_T_KOSONG;
  else
    tOut3 = segDigit3[tX3];

  // Digit satuan selalu tampil
  tOut4 = segDigit14[tX4];

  // Kirim ke driver seven segment
  sendByte(~tOut1);
  sendByte(~tOut2);
  sendByte(~tOut3);
  sendByte(~tOut4);
}

void pesanError(){
  byte tandaMin = 0b11110111; 
  byte hurufE = 0b10100001; 
  byte hurufR1= 0b01111100; 
  byte hurufR2= 0b10101011; 
  
  sendByte(~tandaMin); 
  sendByte(~hurufE); 
  sendByte(~hurufR1); 
  sendByte(~hurufR2); 
}

void startupBlink() {
  byte allOn = ~0b00000000;   // semua segmen ON (karena pakai common anoda jadi harus di-invert)
  byte allOff = ~0b11111111;  // semua segmen OFF

  for (int i = 0; i < 3; i++) {
    // semua digit ON
    sendByte(allOn);
    sendByte(allOn);
    sendByte(allOn);
    sendByte(allOn);
    delay(200);

    // semua digit OFF
    sendByte(allOff);
    sendByte(allOff);
    sendByte(allOff);
    sendByte(allOff);
    delay(200);
  }
}

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
static const int HX711_DT  = 21;
static const int HX711_SCK = 22;

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
static const int LIM_DOOR  = 27;
static const int LIM_PUSH  = 33;
//static const int LIM_BURN  = 27;
//static const int LIM_BDOOR = 14;

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

  weightValue = scale.get_units(1);
  if (!isfinite(weightValue)) return NAN;

  if (!wInit) { wInit = true; wEma = weightValue; }
  wEma = (W_ALPHA * weightValue) + ((1.0f - W_ALPHA) * wEma);

  if (wEma < 0 && wEma > -0.2f) wEma = 0;
  return wEma;
}

static void sendWeightNow() {
  weightValue = readWeightKg();
  if (isfinite(weightValue)) sendToMaster(String("WEIGHT:") + String(weightValue, 2));
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

//  r = limActive(LIM_BURN);
//  if (r != rdBurn) { rdBurn = r; tBurn = now; }
//  if ((now - tBurn) >= DEBOUNCE_MS) stBurn = rdBurn;
//
//  r = limActive(LIM_BDOOR);
//  if (r != rdBDoor) { rdBDoor = r; tBDoor = now; }
//  if ((now - tBDoor) >= DEBOUNCE_MS) stBDoor = rdBDoor;
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
  pinMode(DATA_PIN, OUTPUT);
  pinMode(CLOCK_PIN, OUTPUT);
  pinMode(LATCH_PIN, OUTPUT);

  startupBlink();
  Serial.begin(115200);
  bus.begin(Serial1, 9600, RS485_RX, RS485_TX, PIN_RS485_DE);

  // HX711
  scale.begin(HX711_DT, HX711_SCK);
  scale.set_scale(scale_factor);
  scale.tare(20);

  // Limits
  pinMode(LIM_DOOR,  INPUT_PULLUP);
  pinMode(LIM_PUSH,  INPUT_PULLUP);
//  pinMode(LIM_BURN,  INPUT_PULLUP);
//  pinMode(LIM_BDOOR, INPUT_PULLUP);

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
  convertMassValueto7Segment(weightValue);
  }
}
