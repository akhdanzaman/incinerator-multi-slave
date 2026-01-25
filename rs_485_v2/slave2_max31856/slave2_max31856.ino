#include <Arduino.h>
#include <SPI.h>
#include <MAX31856.h>

/// Program Slave RS485 + MAX31856 + 7-Segment (NODE 2)
/// Dibuat identik gaya dengan Slave 3 (HX711) agar mudah disandingkan:
///
/// LIMIT:
/// - LIMITS: event on change (langsung kirim saat berubah)
/// - LIM_HB: heartbeat periodik (500ms)
/// - Event diberi rate-limit kecil agar tidak spam bila ada noise
///
/// TEMP:
/// - TEMP periodic (default 500 ms)
/// - Jika thermocouple OPEN -> kirim TEMP:OPEN dan display error

// =========================
// DEBUG
// =========================
#define DEBUG_SERIAL 1  // 1=print debug ke Serial Monitor

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
static const uint8_t NODE_ID = 2;
static const int PIN_RS485_DE = 4;
static const int RS485_RX = 25;
static const int RS485_TX = 26;

// nilai terakhir (untuk display)
static float tempValue = 0.0f;

// =========================
// 7-SEG MAPPING (SAMA DENGAN HX711)
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

static inline void send4Bytes(byte b1, byte b2, byte b3, byte b4) {
  digitalWrite(LATCH_PIN, LOW);
  shiftOut(DATA_PIN, CLOCK_PIN, MSBFIRST, b1);
  shiftOut(DATA_PIN, CLOCK_PIN, MSBFIRST, b2);
  shiftOut(DATA_PIN, CLOCK_PIN, MSBFIRST, b3);
  shiftOut(DATA_PIN, CLOCK_PIN, MSBFIRST, b4);
  digitalWrite(LATCH_PIN, HIGH);
}

static void pesanError(){
  byte tandaMin = 0b11110111;
  byte hurufE   = 0b10100001;
  byte hurufR1  = 0b01111100;
  byte hurufR2  = 0b10101011;

  send4Bytes(~tandaMin, ~hurufE, ~hurufR1, ~hurufR2);
}

static void convertTempValueto7Segment(float tempVal) {
  if (!isfinite(tempVal) || tempVal > 1600.0f) {
    pesanError();
    return;
  }

  if (tempVal < 0) tempVal = 0;

  int v = (int)tempVal;
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

  send4Bytes(~o1, ~o2, ~o3, ~o4);
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

// =========================
// RS485 SIMPLE (IDENTIK)
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
// MAX31856
// =========================
#define SCK 18
#define CS  5
#define SDI 23
#define SDO 19

#define CR0_INIT  (CR0_AUTOMATIC_CONVERSION + CR0_OPEN_CIRCUIT_FAULT_TYPE_K + CR0_NOISE_FILTER_50HZ)
#define CR1_INIT  (CR1_AVERAGE_2_SAMPLES + CR1_THERMOCOUPLE_TYPE_S)
#define MASK_INIT (~(MASK_VOLTAGE_UNDER_OVER_FAULT + MASK_THERMOCOUPLE_OPEN_FAULT))

static MAX31856 *temperature;

// flag open fault
static bool tcOpenFault = false;

// smoothing EMA
static float tEma = 0;
static bool  tInit = false;
static const float T_ALPHA = 0.20f;

static float readTempC() {
  tcOpenFault = false;

  float t = temperature->readThermocouple(CELSIUS);

#if DEBUG_SERIAL
  Serial.print("[TEMP RAW] ");
  Serial.println(t);
#endif

  // library lama: FAULT_OPEN dikembalikan sebagai angka float
  if ((int)t == FAULT_OPEN) {
    tcOpenFault = true;
    return NAN;
  }

  if (!isfinite(t) || t > 5000.0f) {
    return NAN;
  }

  // simpan nilai terakhir untuk display (mirip weightValue)
  tempValue = t;

  if (!tInit) { tInit = true; tEma = t; }
  tEma = (T_ALPHA * t) + ((1.0f - T_ALPHA) * tEma);

  if (tEma < 0 && tEma > -0.2f) tEma = 0;

  return tEma;
}

// =========================
// LIMIT SWITCH (BDOOR & BURN)
// =========================
// aktif LOW (INPUT_PULLUP)
static const int LIM_BDOOR = 27;
static const int LIM_BURN  = 33;

static inline uint8_t limActive(int pin) { return (digitalRead(pin) == LOW) ? 1 : 0; }

static const uint32_t DEBOUNCE_MS = 25;
static uint8_t stDoor = 0, stBurn = 0;
static uint8_t lastSentDoor = 255, lastSentBurn = 255;
static uint8_t rdDoor = 0, rdBurn = 0;
static uint32_t tDoor = 0, tBurn = 0;

static void sampleLimitsDebounced() {
  uint32_t now = millis();
  uint8_t r;

  r = limActive(LIM_BDOOR);
  if (r != rdDoor) { rdDoor = r; tDoor = now; }
  if ((now - tDoor) >= DEBOUNCE_MS) stDoor = rdDoor;

  r = limActive(LIM_BURN);
  if (r != rdBurn) { rdBurn = r; tBurn = now; }
  if ((now - tBurn) >= DEBOUNCE_MS) stBurn = rdBurn;
}

// IDENTIK konsep dengan Slave 3
static const uint32_t LIMIT_HB_MS = 500;        // heartbeat 500ms
static uint32_t lastLimitHb = 0;

static const uint32_t LIMIT_EVENT_MIN_MS = 50;  // anti spam event
static uint32_t lastLimitEvent = 0;

static void sendLimitHeartbeatAndEvent() {
  uint32_t now = millis();

  // EVENT: kirim cepat bila berubah (debounced) + rate limit
  if ((stDoor != lastSentDoor || stBurn != lastSentBurn) &&
      (now - lastLimitEvent >= LIMIT_EVENT_MIN_MS)) {

    lastLimitEvent = now;

#if DEBUG_SERIAL
    Serial.print("[LIMIT EVENT] BDoor="); Serial.print(stDoor);
    Serial.print(" Burn="); Serial.println(stBurn);
#endif

    sendToMaster(String("LIMITS:") + String((int)stDoor) + "," + String((int)stBurn));

    lastSentDoor = stDoor;
    lastSentBurn = stBurn;
  }

  // HEARTBEAT: kirim periodik
  if (now - lastLimitHb >= LIMIT_HB_MS) {
    lastLimitHb = now;

#if DEBUG_SERIAL
    Serial.print("[LIMIT HB] BDoor="); Serial.print(stDoor);
    Serial.print(" Burn="); Serial.println(stBurn);
#endif

    sendToMaster(String("LIM_HB:") + String((int)stDoor) + "," + String((int)stBurn));
  }
}

// =========================
// TEMP periodic TX (IDENTIK WEIGHT)
// =========================
static const uint32_t TEMP_TX_MS = 500;
static uint32_t lastTempTx = 0;

static void sendTempPeriodic() {
  uint32_t now = millis();
  if (now - lastTempTx < TEMP_TX_MS) return;
  lastTempTx = now;

  float t = readTempC();

  if (tcOpenFault) {
#if DEBUG_SERIAL
    Serial.println("[TEMP] OPEN");
#endif
    sendToMaster("TEMP:OPEN");
    pesanError();
    return;
  }

  if (isfinite(t)) {
#if DEBUG_SERIAL
    Serial.print("[TEMP] ");
    Serial.println(t, 1);
#endif
    sendToMaster(String("TEMP:") + String(t, 1));
  } else {
#if DEBUG_SERIAL
    Serial.println("[TEMP] NaN");
#endif
    sendToMaster("TEMP:NaN");
  }

  // display pakai nilai terakhir
  convertTempValueto7Segment(tempValue);
}

// =========================
// RX handler (IDENTIK)
// =========================
static void handlePayload(const String &payload) {
#if DEBUG_SERIAL
  Serial.print("[RS485 RX CMD] ");
  Serial.println(payload);
#endif

  if (payload == "REQ_TEMP" || payload == "REQ_T" || payload == "GET") {
    float t = readTempC();
    if (tcOpenFault) sendToMaster("TEMP:OPEN");
    else if (isfinite(t)) sendToMaster(String("TEMP:") + String(t, 1));
    else sendToMaster("TEMP:NaN");
    return;
  }

  if (payload == "REQ_LIMITS") {
    sendToMaster(String("LIMITS:") + String((int)stDoor) + "," + String((int)stBurn));
    lastSentDoor = stDoor;
    lastSentBurn = stBurn;
    return;
  }

  if (payload == "REQ_ALL") {
    float t = readTempC();
    if (tcOpenFault) sendToMaster("TEMP:OPEN");
    else if (isfinite(t)) sendToMaster(String("TEMP:") + String(t, 1));
    else sendToMaster("TEMP:NaN");

    sendToMaster(String("LIMITS:") + String((int)stDoor) + "," + String((int)stBurn));
    lastSentDoor = stDoor;
    lastSentBurn = stBurn;
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
  pinMode(PIN_RS485_DE, OUTPUT);
  digitalWrite(PIN_RS485_DE, LOW);
  Serial.begin(115200);
  delay(200);
  DBGLN("[SLAVE2] Boot...");

  pinMode(DATA_PIN, OUTPUT);
  pinMode(CLOCK_PIN, OUTPUT);
  pinMode(LATCH_PIN, OUTPUT);

  startupBlink();

  bus.begin(Serial1, 9600, RS485_RX, RS485_TX, PIN_RS485_DE);
  DBGLN("[SLAVE2] RS485 started");

  // MAX31856 init
  SPI.begin();
  temperature = new MAX31856(SDI, SDO, CS, SCK);
  temperature->writeRegister(REGISTER_CR0, CR0_INIT);
  temperature->writeRegister(REGISTER_CR1, CR1_INIT);
  temperature->writeRegister(REGISTER_MASK, MASK_INIT);
  delay(200);

  // Limits
  pinMode(LIM_BDOOR, INPUT_PULLUP);
  pinMode(LIM_BURN, INPUT_PULLUP);
  sampleLimitsDebounced();

  delay(150);
  sendToMaster("LOG:INFO,BOOT,SLAVE2_MAX31856_READY");

  // initial snapshot
  sendToMaster(String("LIMITS:") + String((int)stDoor) + "," + String((int)stBurn));
  lastSentDoor = stDoor;
  lastSentBurn = stBurn;

  float t = readTempC();
  if (tcOpenFault) sendToMaster("TEMP:OPEN");
  else if (isfinite(t)) sendToMaster(String("TEMP:") + String(t, 1));
  else sendToMaster("TEMP:NaN");

  // initial display
  if (tcOpenFault) pesanError();
  else convertTempValueto7Segment(tempValue);
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

  // identik: event + heartbeat
  sendLimitHeartbeatAndEvent();

  // temp periodic
  sendTempPeriodic();
}