// ESP32 Slave - MAX31856 Thermocouple + 7-Segment Display + RS485 (mirip HX711 slave)
//
// Catatan:
// - NODE_ID sebaiknya unik per node di jaringan RS485.
// - Display 7-seg diasumsikan pakai shift-register daisy-chain (contoh 74HC595) dengan DATA/CLOCK/LATCH.
// - Mapping segDigit14 / segDigit2 / segDigit3 disamakan persis dengan project HX711 agar konsisten.

#include <Arduino.h>
#include <SPI.h>
#include <MAX31856.h>

// =========================
// 7-SEG (SHIFT REGISTER)
// =========================
#define DATA_PIN  14
#define CLOCK_PIN 13
#define LATCH_PIN 12

byte outX1, outX2, outY1, outY2;

// Mapping segment (sama dengan HX711)
byte segDigit14[10] = {
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

byte segDigit2[10] = {
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

byte segDigit3[10] = {
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

static inline void send4Bytes(byte b1, byte b2, byte b3, byte b4) {
  digitalWrite(LATCH_PIN, LOW);
  shiftOut(DATA_PIN, CLOCK_PIN, MSBFIRST, b1);
  shiftOut(DATA_PIN, CLOCK_PIN, MSBFIRST, b2);
  shiftOut(DATA_PIN, CLOCK_PIN, MSBFIRST, b3);
  shiftOut(DATA_PIN, CLOCK_PIN, MSBFIRST, b4);
  digitalWrite(LATCH_PIN, HIGH);
}

static void pesanError() {
  byte tandaMin = 0b11110111;
  byte hurufE   = 0b10100001;
  byte hurufR1  = 0b01111100;
  byte hurufR2  = 0b10101011;
  send4Bytes(~tandaMin, ~hurufE, ~hurufR1, ~hurufR2);
}

static void startupBlink() {
  byte allOn  = ~0b00000000;
  byte allOff = ~0b11111111;

  for (int i = 0; i < 3; i++) {
    send4Bytes(allOn, allOn, allOn, allOn);
    delay(200);
    send4Bytes(allOff, allOff, allOff, allOff);
    delay(200);
  }
}

static void convertTempValueto7Segment(float tempValue) {
  if (!isfinite(tempValue) || tempValue > 1600.0f) {
    pesanError();
    return;
  }

  if (tempValue < 0) tempValue = 0;

  int tempIntVal = (int)tempValue;

  int tX1 = (tempIntVal / 1000) % 10;
  int tX2 = (tempIntVal / 100)  % 10;
  int tX3 = (tempIntVal / 10)   % 10;
  int tX4 = tempIntVal % 10;

  const byte SEG_T_KOSONG = 0b11111111;

  byte tOut1, tOut2, tOut3, tOut4;

  tOut1 = (tX1 == 0) ? SEG_T_KOSONG : segDigit14[tX1];

  if (tX1 == 0 && tX2 == 0) tOut2 = SEG_T_KOSONG;
  else tOut2 = segDigit14[tX2];

  if (tX1 == 0 && tX2 == 0 && tX3 == 0) tOut3 = SEG_T_KOSONG;
  else tOut3 = segDigit3[tX3];

  tOut4 = segDigit14[tX4];

  send4Bytes(~tOut1, ~tOut2, ~tOut3, ~tOut4);
}

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

static inline void sendToMaster(const String &payload) {
  char hdr[8];
  snprintf(hdr, sizeof(hdr), "N%02u:", (unsigned)NODE_ID);
  bus.sendLine(String(hdr) + payload);
}

static bool parseToFrame(const String &line, uint8_t &idOut, String &payloadOut) {
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
// MAX31856
// =========================
#define SCK 18
#define CS  5
#define SDI 23
#define SDO 19

#define CR0_INIT  (CR0_AUTOMATIC_CONVERSION + CR0_OPEN_CIRCUIT_FAULT_TYPE_K + CR0_NOISE_FILTER_50HZ)
#define CR1_INIT  (CR1_AVERAGE_2_SAMPLES + CR1_THERMOCOUPLE_TYPE_S)
#define MASK_INIT (~(MASK_VOLTAGE_UNDER_OVER_FAULT + MASK_THERMOCOUPLE_OPEN_FAULT))

MAX31856 *temperature;

// Flag fault OPEN (cara lama: library mengembalikan konstanta FAULT_OPEN)
static bool tcOpenFault = false;

static float tEma = 0;
static bool  tInit = false;
static const float T_ALPHA = 0.20f;

static float readTempC() {
  tcOpenFault = false;

  float t = temperature->readThermocouple(CELSIUS);

  // === DEBUG RAW SENSOR ===
  Serial.print("[DBG] RAW TC = ");
  Serial.println(t);
  // ========================

  if ((int)t == FAULT_OPEN) {
    tcOpenFault = true;

    Serial.println("[DBG] FAULT_OPEN detected");
    return NAN;
  }

  if (!isfinite(t) || t > 5000.0f) {
    Serial.println("[DBG] Invalid / out-of-range temperature");
    return NAN;
  }

  if (!tInit) { tInit = true; tEma = t; }
  tEma = (T_ALPHA * t) + ((1.0f - T_ALPHA) * tEma);

  if (tEma < 0 && tEma > -0.2f) tEma = 0;

  // === DEBUG FILTERED VALUE ===
  Serial.print("[DBG] EMA TC = ");
  Serial.println(tEma);
  // ============================

  return tEma;
}


static void sendTempNow() {
  float t = readTempC();
  if (isfinite(t)) sendToMaster(String("TEMP:") + String(t, 1));
  else sendToMaster("TEMP:NaN");
}

// =========================
// RX payload handler
// =========================
static void handlePayload(const String &payload) {
  if (payload == "GET" || payload == "REQ_TEMP") {
    sendTempNow();
    return;
  }
  if (payload == "REQ_ALL") {
    sendTempNow();
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
  pinMode(DATA_PIN, OUTPUT);
  pinMode(CLOCK_PIN, OUTPUT);
  pinMode(LATCH_PIN, OUTPUT);

  startupBlink();

  Serial.begin(115200);
  bus.begin(Serial1, 9600, RS485_RX, RS485_TX, PIN_RS485_DE);

  SPI.begin();
  temperature = new MAX31856(SDI, SDO, CS, SCK);
  temperature->writeRegister(REGISTER_CR0, CR0_INIT);
  temperature->writeRegister(REGISTER_CR1, CR1_INIT);
  temperature->writeRegister(REGISTER_MASK, MASK_INIT);
  delay(200);

  sendToMaster("LOG:INFO,BOOT,SLAVE_MAX31856_7SEG_READY");

  float t0 = readTempC();
  sendTempNow();
  convertTempValueto7Segment(t0);
}

void loop() {
  processRx();

  static uint32_t lastTx = 0;
  uint32_t now = millis();
  if (now - lastTx >= 500) {
    lastTx = now;

    float t = readTempC();

    static float lastShown = 0;
    if (isfinite(t)) {
      if (fabs(t - lastShown) < 50.0f) lastShown = t;
    }

    Serial.print("[DBG] tcOpenFault = ");
    Serial.println(tcOpenFault ? "YES" : "NO");
    
    Serial.print("[DBG] LastShown = ");
    Serial.println(lastShown);


    // kirim ke master
   if (tcOpenFault) {
      Serial.println("[TX] TEMP:OPEN");
      sendToMaster("TEMP:OPEN");
    
    } else if (isfinite(t)) {
      Serial.print("[TX] TEMP:");
      Serial.println(t, 1);
      sendToMaster(String("TEMP:") + String(t, 1));
    
    } else {
      Serial.println("[TX] TEMP:NaN");
      sendToMaster("TEMP:NaN");
    }

    // tampilkan ke 7-seg
    if (tcOpenFault) {
      pesanError();                 // ERROR hanya untuk OPEN
    } else {
      convertTempValueto7Segment(lastShown);
    }
  }
}
