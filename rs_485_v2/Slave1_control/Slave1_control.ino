// =====================================================
// SIMULASI VERSI 1
// SLAVE1 (Arduino Mega)
// Fokus: penambahan variabel loadWeight (belum ada logika keputusan)
// + Tambahan: kontrol motor via USB Serial command + status print
// Commands:b
//   MAINDOOR_OPEN | MAINDOOR_CLOSE
//   PUSHER_MAJU   | PUSHER_MUNDUR
//   BDOOR_OPEN    | BDOOR_CLOSE
//   BURNER_MAJU   | BURNER_MUNDUR
//   CONVEYOR_FWD  | CONVEYOR_REV | CONVEYOR_STOP
// =====================================================

#include <Arduino.h>
#include <AccelStepper.h>

static const uint8_t NODE_ID = 1;


// =========================
// PINS
// =========================
#define STEP_PIN_DOOR   6
#define DIR_PIN_DOOR    7

#define STEP_PIN_PUSH   4
#define DIR_PIN_PUSH    5

#define STEP_PIN_BURN   26
#define DIR_PIN_BURN    27

#define STEP_PIN_BDOOR  28
#define DIR_PIN_BDOOR   29

#define STEP_PIN_ASH    30
#define DIR_PIN_ASH     31

#define STEP_PIN_MAINC  8
#define DIR_PIN_MAINC   9

#define RELAY_BLOWER   32
#define RELAY_BURNER   34
#define RELAY_IGNITION 33

#define PIN_FUEL_ADC   A0

// =========================
// LIMIT SWITCH
// =========================
#define LIMIT_PIN_PUSH_HOME 52
#define LIMIT_PIN_DOOR_HOME 45
#define LIMIT_PIN_BDOOR_HOME 46

enum ControlMode : uint8_t {
  MODE_MANUAL = 0,
  MODE_AUTO   = 1
};

ControlMode controlMode = MODE_MANUAL;

enum SystemState : uint8_t {
  IDLE,
  WEIGHING,
  OPENING_MAIN_DOOR,
  DUMPING_IN,
  DUMPING_OUT,
  CLOSING_MAIN_DOOR,
  OPENING_BURN_DOOR,
  BURNER_IN,
  IGNITING,
  PREHEAT,
  BURNING,
  BURNER_OUT,
  CLOSING_BURN_DOOR,
  FAULT
};
enum MotorID : uint8_t {
  MOTOR_DOOR = 0,
  MOTOR_PUSHER,
  MOTOR_BURNER,
  MOTOR_BDOOR,
  MOTOR_ASH,
  MOTOR_CONVEYOR
};

// ===== Relay helpers (asumsi ACTIVE LOW) =====
inline void burnerOn() { digitalWrite(RELAY_BURNER, LOW); }
inline void burnerOff() { digitalWrite(RELAY_BURNER, HIGH); }
inline void blowOn()  { digitalWrite(RELAY_BLOWER, LOW);  Serial.println("[DBG] blowOn()"); }
inline void blowOff() { digitalWrite(RELAY_BLOWER, HIGH); Serial.println("[DBG] blowOff()"); }
inline void ignOn() { digitalWrite(RELAY_IGNITION, LOW); }
inline void ignOff() { digitalWrite(RELAY_IGNITION, HIGH); }

static inline bool isDumpingState(SystemState s) {
  return (s == DUMPING_IN || s == DUMPING_OUT);
}

// =========================
// RS485
// =========================
#define RS485_BUS Serial1
static const uint32_t RS485_BAUD = 9600;
static const int RS485_DE_RE = 39;

static inline void rs485SetTx(bool en) {
  digitalWrite(RS485_DE_RE, en ? HIGH : LOW);
}

static void rs485SendRawLine(const String &line) {
  rs485SetTx(true);
  delayMicroseconds(200);

  RS485_BUS.print(line);
  RS485_BUS.write(10);   // ASCII 10 = newline '\n'

  RS485_BUS.flush();
  delayMicroseconds(200);
  rs485SetTx(false);
}


static inline void sendToNode(uint8_t nodeId, const String &payload) {
  char hdr[8];
  snprintf(hdr, sizeof(hdr), "TO%02u:", nodeId);
  rs485SendRawLine(String(hdr) + payload);
}

static const uint8_t NODE_TEMP   = 2;
static const uint8_t NODE_WEIGHT = 3;

// =========================
// STEPPERS
// =========================
AccelStepper stepperDoor(AccelStepper::DRIVER, STEP_PIN_DOOR, DIR_PIN_DOOR);
AccelStepper stepperPush(AccelStepper::DRIVER, STEP_PIN_PUSH, DIR_PIN_PUSH);
AccelStepper stepperAsh(AccelStepper::DRIVER, STEP_PIN_ASH, DIR_PIN_ASH);
AccelStepper stepperBurner(AccelStepper::DRIVER, STEP_PIN_BURN, DIR_PIN_BURN);
AccelStepper stepperBurnDoor(AccelStepper::DRIVER, STEP_PIN_BDOOR, DIR_PIN_BDOOR);
AccelStepper stepperMainConveyor(AccelStepper::DRIVER, STEP_PIN_MAINC, DIR_PIN_MAINC);

static inline bool anyStepperBusy() {
  if (stepperDoor.distanceToGo() != 0) return true;
  if (stepperPush.distanceToGo() != 0) return true;
  if (stepperAsh.distanceToGo() != 0) return true;
  if (stepperBurner.distanceToGo() != 0) return true;
  if (stepperBurnDoor.distanceToGo() != 0) return true;
  if (stepperMainConveyor.speed() != 0) return true;
  return false;
}

// =========================
// CONFIG & VARS
// =========================
struct MotorConfig { long speed; long accel; long steps; };

// Index: 0 Door, 1 Push, 2 Burner, 3 BurnDoor, 4 Ash, 5 Main Conveyor
MotorConfig motorConfigs[6] = {
  { 10000, 200, 6200 },
  { 20000, 200, 10500 },
  { 10000, 200, 11500 },
  {  2000, 200,  1400 },
  {   800, 200,  1000 },
  { 20000, 200,  1000 }
};


static inline long getOpenPos(uint8_t idx)  { return motorConfigs[idx].steps; }
static inline long getClosePos(uint8_t idx) { (void)idx; return 0; }

static inline long doorOpenPos()  { return 0; }
static inline long doorClosePos() { return motorConfigs[0].steps; }

static inline long bdoorOpenPos()  { return 0; }
static inline long bdoorClosePos() { return motorConfigs[3].steps; }

// System vars
SystemState currentState = IDLE;
bool autoModeEnabled = false;
bool oneShotAutoMode = false;

static const unsigned long ONE_SHOT_BURN_MS = 20000;
unsigned long oneShotBurnStartMs = 0;
bool oneShotBurnTimerActive = false;

// Manual flags
bool isManualFeederOn = false;
bool isManualWeighingOn = false;
bool isManualOpen = true;
long manualSteps = 1000;
long manualSpeed = 1000;

// Busy flags
bool motorDoorBusy = false;
bool motorPushBusy = false;
bool motorAshBusy = false;
bool motorBurnerBusy = false;
bool motorBurnDoorBusy = false;

// Burner logic
static const unsigned long IGNITION_delay_MS = 20000;
static const float OVERHEAT_TEMP = 1400.0f;

bool isPriming = false;
unsigned long ignitiondelayStart = 0;
unsigned long burnAccumulatedMs = 0;
unsigned long burnPhaseStartMs = 0;
unsigned long lastCountdownUpdate = 0;
long last_remaining = -1;

bool startBurningAfterClose = false;

long conveyorSpeed = 20000; // default speed untuk CONVEYOR_FWD/REV

// Batch/metrics
float loadWeight = 0;
float minLoadWeight = 3.0;
float minBatchWeight = 5.0f;
float solarVolumeUsed_L = 0;
float totalConsumedKg = 0;
unsigned long totalBatchCount = 0;
static const float LITERS_PER_HOUR_BURNER = 10.0f;

float tempSetpoint = 1200.0f;
uint32_t burnerActiveTimeSec = 30;

// Sensor mirrors (diinject)
float currentTempC = 0.0f;

// Fuel
int FUEL_ADC_EMPTY = 200;
int FUEL_ADC_FULL  = 850;
uint8_t fuelPct = 0;
float fuelLiterRemaining = 0.0f;
static const float TANK_CAPACITY_L = 20.0f;
float fuelEma = 0;
static const float FUEL_EMA_ALPHA = 0.15f;

// Helper: set mode + efek samping aman
void setMode(ControlMode m) {
  if (controlMode == m) return;
  controlMode = m;

  if (controlMode == MODE_MANUAL) {
    // Matikan auto supaya tidak melawan manual
    autoModeEnabled = false;
    oneShotAutoMode = false;

    // Optional: stop conveyor auto-run (manual boleh nyalakan lagi via command)
    stepperMainConveyor.setSpeed(0);

    Serial.println("[MODE] MANUAL");
  } else {
    Serial.println("[MODE] AUTO");
  }
}

// Kalau ada command manual, paksa pindah ke MANUAL
static inline void enterManualByCommand(const char *cmdName) {
  if (controlMode != MODE_MANUAL) {
    Serial.print("[MODE] Switch to MANUAL by cmd: ");
    Serial.println(cmdName);
  }
  setMode(MODE_MANUAL);
}

//===========AUTO NEED DROP===============
bool autoNeedDrop = false;           // setelah push: wajib turun dulu
uint8_t autoDropStableCnt = 0;
uint8_t autoWeighStableCnt = 0;

static const uint8_t AUTO_STABLE_NEED = 3;
static const float WEIGH_RESET_HYS = 0.5f;   // hysteresis (kg), tuning sesuai noise
//==============================================================

uint8_t readFuelPercent() {
  int raw = analogRead(PIN_FUEL_ADC);
  if (fuelEma <= 0) fuelEma = raw;
  fuelEma = (FUEL_EMA_ALPHA * raw) + ((1.0f - FUEL_EMA_ALPHA) * fuelEma);
  int v = (int)(fuelEma + 0.5f);
  long pct = map(v, FUEL_ADC_EMPTY, FUEL_ADC_FULL, 0, 100);
  if (pct < 0) pct = 0;
  if (pct > 100) pct = 100;
  return (uint8_t)pct;
}

// Telemetry timers
unsigned long lastSensorTxMs = 0;
unsigned long lastProgTxMs = 0;
unsigned long lastFuelTxMs = 0;
unsigned long lastMetricsTxMs = 0;



// =========================
// CORE HELPERS
// =========================
void applyMotorConfigsToSteppers() {
  stepperDoor.setMaxSpeed(motorConfigs[0].speed);
  stepperPush.setMaxSpeed(motorConfigs[1].speed);
  stepperBurner.setMaxSpeed(motorConfigs[2].speed);
  stepperBurnDoor.setMaxSpeed(motorConfigs[3].speed);
  stepperAsh.setMaxSpeed(motorConfigs[4].speed);
  stepperMainConveyor.setMaxSpeed(motorConfigs[5].speed);

  stepperDoor.setAcceleration(motorConfigs[0].accel);
  stepperPush.setAcceleration(motorConfigs[1].accel);
  stepperBurner.setAcceleration(motorConfigs[2].accel);
  stepperBurnDoor.setAcceleration(motorConfigs[3].accel);
  stepperAsh.setAcceleration(motorConfigs[4].accel);
  stepperMainConveyor.setAcceleration(motorConfigs[5].accel);
}

void setAutoMode(bool en) {
  if (autoModeEnabled == en) return;
  autoModeEnabled = en;
  if (!en) {
    stepperMainConveyor.setSpeed(0);
  }
}

void emergencyStop() {
  burnerOff();
  ignOff();
  blowOff();

  isPriming = false;
  burnAccumulatedMs = 0;
  burnPhaseStartMs = 0;

  isManualFeederOn = false;
  isManualWeighingOn = false;

  stepperDoor.stop();
  stepperPush.stop();
  stepperAsh.stop();
  stepperBurner.stop();
  stepperBurnDoor.stop();

  stepperMainConveyor.setSpeed(0);
}

// =========================
// BURN / PRIMING / CONVEYOR SERVICES
// =========================
void startBurningHold() {
  burnerOff();
  blowOff();

  burnAccumulatedMs = 0;
  burnPhaseStartMs = 0;
  lastCountdownUpdate = millis();

  currentState = BURNING;
  burnPhaseStartMs = millis();
}

void checkFeederConveyor() {
  // Kalau bukan AUTO, jangan ada otomatisasi conveyor
  if (controlMode != MODE_AUTO) return;

  if (isManualFeederOn) return; // kalau kamu masih pakai flag ini, tetap dihormati

  static bool conveyorRunning = false;
  bool shouldRun = (autoModeEnabled) &&
                   (currentState != FAULT) &&
                   (!isDumpingState(currentState)) &&
                   (loadWeight < minLoadWeight);

  if (shouldRun && !conveyorRunning) {
    stepperMainConveyor.setMaxSpeed(motorConfigs[5].speed);
    stepperMainConveyor.setSpeed(motorConfigs[5].speed);
    conveyorRunning = true;
  } else if (!shouldRun && conveyorRunning) {
    stepperMainConveyor.setSpeed(0);
    conveyorRunning = false;
  }
}


void checkStepperFlags() {
  if (motorDoorBusy && stepperDoor.distanceToGo() == 0) motorDoorBusy = false;
  if (motorPushBusy && stepperPush.distanceToGo() == 0) motorPushBusy = false;
  if (motorAshBusy && stepperAsh.distanceToGo() == 0) motorAshBusy = false;
  if (motorBurnerBusy && stepperBurner.distanceToGo() == 0) motorBurnerBusy = false;
  if (motorBurnDoorBusy && stepperBurnDoor.distanceToGo() == 0) motorBurnDoorBusy = false;
}

// =========================
// RUN ALL STEPPERS
// =========================
void runAllSteppers() {
  stepperDoor.run();
  stepperPush.run();
  stepperAsh.run();
  stepperBurner.run();
  stepperBurnDoor.run();
  stepperMainConveyor.runSpeed();
}

// =========================
// TELEMETRY (DATA/PROG/FUEL)
// =========================
void sendTelemetry() {
  unsigned long now = millis();

  if (now - lastSensorTxMs >= 500) {
    lastSensorTxMs = now;
    String line = String("DATA:") + String(currentTempC, 1) + "," + String(loadWeight, 2) + "," + String((int)currentState);
    (void)line;
  }

  if (now - lastProgTxMs >= 500) {
    lastProgTxMs = now;
  }

  if (now - lastFuelTxMs >= 500) {
    lastFuelTxMs = now;
    fuelPct = readFuelPercent();
    fuelLiterRemaining = (fuelPct / 100.0f) * TANK_CAPACITY_L;
  }
}

// =========================
// MAIN DOOR HOMING
// =========================
static inline bool doorHomePressed() {
  return digitalRead(LIMIT_PIN_DOOR_HOME) == LOW;
}



// =========================
// PUSHER HOMING
// =========================
static inline bool pushHomePressed() {
  return digitalRead(LIMIT_PIN_PUSH_HOME) == LOW;
}

void homePusher() {
  const int HOME_DIR = -1;
  const float SEEK_SPEED_1 = 600;
  const float SEEK_SPEED_2 = 400;
  const long  BACKOFF_STEPS = 500;

  const unsigned long HOMING_TIMEOUT_MS = 15000;
  const uint8_t PRESS_STABLE_COUNT = 8;
  const uint8_t RELEASE_STABLE_COUNT = 8;

  auto waitPressedStable = [&](float spd) {
    const unsigned long t0 = millis();
    uint8_t cnt = 0;
    stepperPush.setSpeed(spd);
    while (cnt < PRESS_STABLE_COUNT) {
      stepperPush.runSpeed();
      if (pushHomePressed()) cnt++; else cnt = 0;
      if (millis() - t0 > HOMING_TIMEOUT_MS) return false;
    }
    return true;
  };

  auto waitReleasedStable = [&](float spd) {
    const unsigned long t0 = millis();
    uint8_t cnt = 0;
    stepperPush.setSpeed(spd);
    while (cnt < RELEASE_STABLE_COUNT) {
      stepperPush.runSpeed();
      if (!pushHomePressed()) cnt++; else cnt = 0;
      if (millis() - t0 > HOMING_TIMEOUT_MS) return false;
    }
    return true;
  };

  stepperPush.setAcceleration(200);
  stepperPush.setMaxSpeed(20000);

  Serial.print("Homing pusher... limitRaw=");
  Serial.println(digitalRead(LIMIT_PIN_PUSH_HOME));

  if (pushHomePressed()) {
    if (!waitReleasedStable(-HOME_DIR * SEEK_SPEED_1)) {
      stepperPush.setSpeed(0);
      Serial.println("ERR: Homing timeout (release). Check switch/wiring");
      return;
    }
  }

  if (!waitPressedStable(HOME_DIR * SEEK_SPEED_1)) {
    stepperPush.setSpeed(0);
    Serial.println("ERR: Homing timeout (seek1). Flip HOME_DIR if direction wrong");
    return;
  }

  stepperPush.setCurrentPosition(0);
  stepperPush.moveTo(-HOME_DIR * BACKOFF_STEPS);
  while (stepperPush.distanceToGo() != 0) {
    stepperPush.run();
  }

  if (pushHomePressed()) {
    if (!waitReleasedStable(-HOME_DIR * SEEK_SPEED_2)) {
      stepperPush.setSpeed(0);
      Serial.println("ERR: Homing timeout (release2). Check switch/wiring");
      return;
    }
  }

  if (!waitPressedStable(HOME_DIR * SEEK_SPEED_2)) {
    stepperPush.setSpeed(0);
    Serial.println("ERR: Homing timeout (seek2). Flip HOME_DIR if direction wrong");
    return;
  }

  stepperPush.setCurrentPosition(0);
  stepperPush.moveTo(0);

  Serial.println("OK: Pusher homed (pos=0)");
}

// =========================
// BDOOR HOMING (identik: seek -> backoff -> seek)
// =========================
static inline bool bdoorHomePressed() {
  return digitalRead(LIMIT_PIN_BDOOR_HOME) == LOW;
}

void homeBDoor() {
  const int HOME_DIR = -1; // -1 atau +1 (balik jika arah salah)

  const float SEEK_SPEED_1 = 500;
  const float SEEK_SPEED_2 = 100;
  const long  BACKOFF_STEPS = 300;

  const unsigned long HOMING_TIMEOUT_MS = 30000;
  const uint8_t PRESS_STABLE_COUNT = 8;
  const uint8_t RELEASE_STABLE_COUNT = 8;

  auto waitPressedStable = [&](float spd) {
    const unsigned long t0 = millis();
    uint8_t cnt = 0;
    stepperBurnDoor.setSpeed(spd);
    while (cnt < PRESS_STABLE_COUNT) {
      stepperBurnDoor.runSpeed();
      if (bdoorHomePressed()) cnt++; else cnt = 0;
      if (millis() - t0 > HOMING_TIMEOUT_MS) return false;
    }
    return true;
  };

  auto waitReleasedStable = [&](float spd) {
    const unsigned long t0 = millis();
    uint8_t cnt = 0;
    stepperBurnDoor.setSpeed(spd);
    while (cnt < RELEASE_STABLE_COUNT) {
      stepperBurnDoor.runSpeed();
      if (!bdoorHomePressed()) cnt++; else cnt = 0;
      if (millis() - t0 > HOMING_TIMEOUT_MS) return false;
    }
    return true;
  };

  stepperBurnDoor.setAcceleration(200);
  stepperBurnDoor.setMaxSpeed(2000);

  Serial.print("Homing bdoor... limitRaw=");
  Serial.println(digitalRead(LIMIT_PIN_BDOOR_HOME));

  if (bdoorHomePressed()) {
    if (!waitReleasedStable(-HOME_DIR * SEEK_SPEED_1)) {
      stepperBurnDoor.setSpeed(0);
      Serial.println("ERR: BDoor homing timeout (release). Check switch/wiring");
      return;
    }
  }

  if (!waitPressedStable(HOME_DIR * SEEK_SPEED_1)) {
    stepperBurnDoor.setSpeed(0);
    Serial.println("ERR: BDoor homing timeout (seek1). Flip HOME_DIR if direction wrong");
    return;
  }

  stepperBurnDoor.setCurrentPosition(0);
  stepperBurnDoor.moveTo(-HOME_DIR * BACKOFF_STEPS);
  while (stepperBurnDoor.distanceToGo() != 0) {
    stepperBurnDoor.run();
  }

  if (bdoorHomePressed()) {
    if (!waitReleasedStable(-HOME_DIR * SEEK_SPEED_2)) {
      stepperBurnDoor.setSpeed(0);
      Serial.println("ERR: BDoor homing timeout (release2). Check switch/wiring");
      return;
    }
  }

  if (!waitPressedStable(HOME_DIR * SEEK_SPEED_2)) {
    stepperBurnDoor.setSpeed(0);
    Serial.println("ERR: BDoor homing timeout (seek2). Flip HOME_DIR if direction wrong");
    return;
  }

  stepperBurnDoor.setCurrentPosition(0);
  stepperBurnDoor.moveTo(0);
  Serial.println("OK: BDoor homed (pos=0)");
   // =========================
  // OPTION A: Setelah homing, tutup pintu otomatis
  // (sama persis seperti perintah MAINDOOR_CLOSE)
  // =========================
  cmdBDoorClose();
  while (stepperBurnDoor.distanceToGo() != 0) {
  stepperBurnDoor.run();
  }
  Serial.println("OK: Door closed after homing");
}

// =========================
// USB & RS485 BUFFER
// =========================
String rs485Buf;

// USB command line buffer
String usbLine;

// =========================
// USB COMMAND RX (newline-terminated)
// =========================
void processUsbCommands() {
  while (Serial.available()) {
    char c = (char)Serial.read();

    // ASCII 10 = '\n' (newline)
    if (c == 10) {
      usbLine.trim();
      if (usbLine.length() > 0) {
        handleCommand(usbLine);   // status print ada di handleCommand
      }
      usbLine = "";
    }
    // ASCII 13 = '\r' (carriage return) -> diabaikan
    else if (c != 13) {
      usbLine += c;
    }
  }
}


// =========================
// TEMP MONITOR
// =========================
String lastTempStr = "Not Connected";
bool gotAnyNode2 = false;
unsigned long lastNode2RxMs = 0;

// =========================
// WEIGHT MONITOR
// =========================
String loadWeightStr = "Not Connected";
bool gotAnyNode3 = false;
unsigned long lastNode3RxMs = 0;

static const unsigned long PRINT_INTERVAL = 5000;
unsigned long lastPrintMs = 0;

// =========================
// RS485 RX: parse TOxx:payload  atau  Nxx:payload
// contoh valid:
//   TO03:WEIGHT:12.3
//   N03:WEIGHT:12.3
// =========================
static bool parseToFrame(const String &line, uint8_t &idOut, String &payloadOut) {
  // minimal:
  //  TO00:x  -> 6 char (T O d d : x)
  //  N00:x   -> 5 char (N d d : x)
  if (line.length() < 5) return false;

  uint8_t start = 0;

  // Format TOxx:...
  if (line.length() >= 6 && line[0] == 'T' && line[1] == 'O') {
    char d1 = line[2], d2 = line[3];
    if (d1 < '0' || d1 > '9' || d2 < '0' || d2 > '9') return false;
    if (line[4] != ':') return false;

    idOut = (uint8_t)((d1 - '0') * 10 + (d2 - '0'));
    payloadOut = line.substring(5);
    return true;
  }

  // Format Nxx:...
  if (line[0] == 'N') {
    if (line.length() < 5) return false;
    char d1 = line[1], d2 = line[2];
    if (d1 < '0' || d1 > '9' || d2 < '0' || d2 > '9') return false;
    if (line[3] != ':') return false;

    idOut = (uint8_t)((d1 - '0') * 10 + (d2 - '0'));
    payloadOut = line.substring(4);
    return true;
  }

  return false;
}

static inline void sendToMaster1(const String &payload) {
  rs485SendRawLine(String("N01:") + payload);
}

static void processRs485CommandsForMe() {
  while (RS485_BUS.available()) {
    char c = (char)RS485_BUS.read();
    if (c == 10) { // '\n'
      rs485Buf.trim();
      if (rs485Buf.length() > 0) {
        uint8_t id;
        String payload;
        if (parseToFrame(rs485Buf, id, payload)) {
          // HANYA terima command yang ditujukan ke Mega ini
          if (rs485Buf.startsWith("TO") && id == NODE_ID) {
            handleCommand(payload);     // ini reuse handler USB kamu. good.
          }
          // kalau kamu masih mau monitor N02/N03, boleh tetap di sini, tapi pisahin bloknya.
        }
      }
      rs485Buf = "";
    } else if (c != 13) {
      rs485Buf += c;
      if (rs485Buf.length() > 200) rs485Buf = "";
    }
  }
}


void processRs485RxSensors() {
  while (RS485_BUS.available()) {
    char c = (char)RS485_BUS.read();

    // ASCII 10 = '\n' (newline)
    if (c == 10) {
      rs485Buf.trim();

      if (rs485Buf.length() > 0) {
        uint8_t nid;
        String payload;

        if (parseToFrame(rs485Buf, nid, payload)) {
          if (nid == NODE_TEMP && payload.startsWith("TEMP:")) {
            lastTempStr = payload;
            gotAnyNode2 = true;
            lastNode2RxMs = millis();
          }

          if (nid == NODE_WEIGHT && payload.startsWith("WEIGHT:")) {
            loadWeightStr = payload;

            String valStr = payload.substring(7);
            float w = valStr.toFloat();
            if (isfinite(w)) {
              loadWeight = w;
            }

            gotAnyNode3 = true;
            lastNode3RxMs = millis();
          }
        }
      }

      rs485Buf = "";
    }
    // ASCII 13 = '\r' (carriage return) -> diabaikan
    else if (c != 13) {
      rs485Buf += c;
      // optional safety biar buffer nggak ngembang kalau data kacau
      if (rs485Buf.length() > 200) rs485Buf = "";
    }
  }
}


static void processRs485Unified() {
  while (RS485_BUS.available()) {
    char c = (char)RS485_BUS.read();
    if (c == 10) { // '\n'
      rs485Buf.trim();
      if (rs485Buf.length()) {
        uint8_t id; String payload;
        if (parseToFrame(rs485Buf, id, payload)) {

          // 1) Command dari Master: TO01:<cmd>
          if (rs485Buf.startsWith("TO") && id == NODE_ID) {
            handleCommand(payload);
          }

          // 2) Telemetry dari node lain: N02 / N03
          if (rs485Buf.startsWith("N")) {
            if (id == NODE_TEMP && payload.startsWith("TEMP:")) {
              currentTempC = payload.substring(5).toFloat();
            }
            if (id == NODE_WEIGHT && payload.startsWith("WEIGHT:")) {
              loadWeight = payload.substring(7).toFloat();
            }
          }
        }
      }
      rs485Buf = "";
    } else if (c != 13) {
      rs485Buf += c;
      if (rs485Buf.length() > 220) rs485Buf = "";
    }
  }
}

// =========================
// PERIODIC MONITOR
// =========================
void periodicPrint() {
  unsigned long now = millis();
  if (now - lastPrintMs < PRINT_INTERVAL) return;
  lastPrintMs = now;

  Serial.print("[TEMP] ");
  Serial.println((gotAnyNode2 && now - lastNode2RxMs < PRINT_INTERVAL) ? lastTempStr : "Not Connected");

  Serial.print("[WEIGHT] ");
  Serial.println((gotAnyNode3 && now - lastNode3RxMs < PRINT_INTERVAL) ? loadWeightStr : "Not Connected");
}

// =========================
// DIRECT MOTOR COMMANDS (USB)
// =========================
static inline void cmdMainDoorOpen()  { stepperDoor.moveTo(doorOpenPos()); }
static inline void cmdMainDoorClose() { stepperDoor.moveTo(doorClosePos()); }

static inline void cmdPusherMaju()   { stepperPush.moveTo(getOpenPos(1)); }
static inline void cmdPusherMundur() { stepperPush.moveTo(getClosePos(1)); }

static inline void cmdBDoorOpen()  { stepperBurnDoor.moveTo(bdoorOpenPos()); }
static inline void cmdBDoorClose() { stepperBurnDoor.moveTo(bdoorClosePos()); }

static inline void cmdBurnerMaju()   { stepperBurner.moveTo(getOpenPos(2)); }
static inline void cmdBurnerMundur() { stepperBurner.moveTo(getClosePos(2)); }

// Conveyor is continuous (runSpeed), so control by setSpeed()
static inline void cmdConveyorFwd() {
  stepperMainConveyor.setMaxSpeed(motorConfigs[5].speed);
  stepperMainConveyor.setSpeed(abs(conveyorSpeed));
}
static inline void cmdConveyorRev() {
  stepperMainConveyor.setMaxSpeed(motorConfigs[5].speed);
  stepperMainConveyor.setSpeed(-abs(conveyorSpeed));
}
static inline void cmdConveyorStop() {
  stepperMainConveyor.setSpeed(0);
}

// =========================
// ONE-SHOT SIMULATION (RUN_ONE_SHOOT)
// Urutan:
//   CONVEYOR_FWD
//   MAINDOOR_OPEN (tunggu selesai)
//   PUSHER_MAJU (selama pusher bergerak -> conveyor STOP)
//   PUSHER_MUNDUR (selama pusher bergerak -> conveyor STOP)
//   (setelah pusher mundur selesai) CONVEYOR_FWD
//   MAINDOOR_CLOSE (tunggu selesai)
//   DONE (conveyor STOP)
// =========================
enum OneShotStep : uint8_t {
  OS_IDLE = 0,
  OS_WEIGHING,
  OS_CONV_START,
  OS_DOOR_OPEN,
  OS_PUSH_MAJU,
  OS_PUSH_MUNDUR,
  OS_CONV_RESUME,
  OS_DOOR_CLOSE,

  // tambahan
  OS_BDOOR_OPEN,
  OS_BURNER_ON,     // 20 detik non-blocking
  OS_BURNER_OFF,
  OS_BDOOR_CLOSE,

  OS_DONE
};


bool oneShotRunning = false;
OneShotStep oneShotStep = OS_IDLE;
bool prevManualFeederOn = false;

//=============================================================

enum AutoSeqStep : uint8_t {
  AS_IDLE = 0,
  AS_DOOR_OPEN,
  AS_WEIGHING,
  AS_PUSH_MAJU,
  AS_PUSH_MUNDUR,
  AS_DOOR_CLOSE
};

bool autoSeqRunning = false;
AutoSeqStep autoSeqStep = AS_IDLE;

static const uint8_t AUTO_REPEAT = 4;
uint8_t autoCycleCount = 0;
//======================================================================

static inline bool atTarget(AccelStepper &m, long target) {
  return (m.distanceToGo() == 0 && m.currentPosition() == target);
}

void startOneShot() {
  if (oneShotRunning) {
    Serial.println("[ONE_SHOT] Already running");
    return;
  }

  Serial.println("[ONE_SHOT] START");

  // supaya checkFeederConveyor() tidak mengubah conveyor saat one-shot
  prevManualFeederOn = isManualFeederOn;
  isManualFeederOn = true;

  oneShotRunning = true;
  oneShotStep = OS_WEIGHING;

  // kick pertama langsung jalan
  // cmdConveyorFwd(); saat weighing ini di comment dulu
  Serial.println("[ONE_SHOT] STEP: WEIGHING");
}

void cancelOneShot(const char *reason) {
  if (!oneShotRunning) return;
  Serial.print("[ONE_SHOT] CANCELED: ");
  Serial.println(reason);

  // safety
  burnerOff();
  blowOff();

  oneShotBurnTimerActive = false;
  oneShotBurnStartMs = 0;

  oneShotRunning = false;
  oneShotStep = OS_IDLE;
  isManualFeederOn = prevManualFeederOn;
}


void serviceOneShot() {
  if (!oneShotRunning) return;

  const long DOOR_OPEN_POS  = doorOpenPos();
  const long DOOR_CLOSE_POS = doorClosePos();

  const long BDOOR_OPEN_POS  = bdoorOpenPos();
  const long BDOOR_CLOSE_POS = bdoorClosePos();

  const long PUSH_IN_POS    = getOpenPos(1);
  const long PUSH_OUT_POS   = getClosePos(1);

  switch (oneShotStep) {

    case OS_WEIGHING: {
      if (handleWeighingOneShoot()) {
        cmdConveyorStop();  // optional
        Serial.println("[ONE_SHOT] WEIGH OK -> NEXT");
        oneShotStep = OS_CONV_START;
      }
    } break;

    case OS_CONV_START: {
      cmdMainDoorOpen();
      Serial.println("[ONE_SHOT] STEP: MAINDOOR_OPEN");
      oneShotStep = OS_DOOR_OPEN;
    } break;

    case OS_DOOR_OPEN: {
      if (atTarget(stepperDoor, DOOR_OPEN_POS)) {
        cmdConveyorStop();
        cmdPusherMaju();
        Serial.println("[ONE_SHOT] STEP: PUSHER_MAJU (CONVEYOR_STOP)");
        oneShotStep = OS_PUSH_MAJU;
      }
    } break;

    case OS_PUSH_MAJU: {
      if (stepperPush.distanceToGo() != 0) cmdConveyorStop();

      if (atTarget(stepperPush, PUSH_IN_POS)) {
        cmdConveyorStop();
        cmdPusherMundur();
        Serial.println("[ONE_SHOT] STEP: PUSHER_MUNDUR (CONVEYOR_STOP)");
        oneShotStep = OS_PUSH_MUNDUR;
      }
    } break;

    case OS_PUSH_MUNDUR: {
      if (stepperPush.distanceToGo() != 0) cmdConveyorStop();

      if (atTarget(stepperPush, PUSH_OUT_POS)) {
        cmdConveyorFwd();
        Serial.println("[ONE_SHOT] STEP: CONVEYOR_FWD (resume)");
        oneShotStep = OS_CONV_RESUME;
      }
    } break;

    case OS_CONV_RESUME: {
      cmdMainDoorClose();
      Serial.println("[ONE_SHOT] STEP: MAINDOOR_CLOSE");
      oneShotStep = OS_DOOR_CLOSE;
    } break;

    case OS_DOOR_CLOSE: {
      if (atTarget(stepperDoor, DOOR_CLOSE_POS)) {
        cmdBDoorOpen();
        Serial.println("[ONE_SHOT] STEP: BDOOR_OPEN");
        oneShotStep = OS_BDOOR_OPEN;
      }
    } break;

    case OS_BDOOR_OPEN: {
      if (atTarget(stepperBurnDoor, BDOOR_OPEN_POS)) {
        burnerOn();
        blowOn();
        oneShotBurnTimerActive = true;
        oneShotBurnStartMs = millis();

        Serial.println("[ONE_SHOT] STEP: BURNER_ON (20s) + BLOWER_ON");
        oneShotStep = OS_BURNER_ON;
      }
    } break;

    case OS_BURNER_ON: {
      if (!oneShotBurnTimerActive) {
        oneShotBurnTimerActive = true;
        oneShotBurnStartMs = millis();
        burnerOn();
      }

      if (millis() - oneShotBurnStartMs >= ONE_SHOT_BURN_MS) {
        burnerOff();
        oneShotBurnTimerActive = false;

        Serial.println("[ONE_SHOT] STEP: BURNER_OFF");
        oneShotStep = OS_BURNER_OFF;
      }
    } break;

    case OS_BURNER_OFF: {
      burnerOff();
      cmdBDoorClose();
      Serial.println("[ONE_SHOT] STEP: BDOOR_CLOSE");
      oneShotStep = OS_BDOOR_CLOSE;
    } break;

    case OS_BDOOR_CLOSE: {
      if (atTarget(stepperBurnDoor, BDOOR_CLOSE_POS)) {
        cmdConveyorStop();
        burnerOff();
        blowOff();
        Serial.println("[ONE_SHOT] DONE (CONVEYOR_STOP)");

        oneShotRunning = false;
        oneShotStep = OS_IDLE;
        isManualFeederOn = prevManualFeederOn;
      }
    } break;

    default:
      break;
  }
}

void startAutoSeq() {
  if (autoSeqRunning) {
    Serial.println("[AUTOSEQ] Already running");
    return;
  }
  if (oneShotRunning) {
    Serial.println("[AUTOSEQ] Blocked: oneShotRunning");
    return;
  }

  setMode(MODE_AUTO);
  setAutoMode(true);

  prevManualFeederOn = isManualFeederOn;
  isManualFeederOn = true;

  autoSeqRunning = true;
  autoSeqStep = AS_DOOR_OPEN;   // mulai dari buka pintu dulu
  autoCycleCount = 0;
  autoWeighStableCnt = 0;

  Serial.println("[AUTOSEQ] START (Door open once, then 4x: Weigh->PushIn->PushOut, door close once, loop)");
}

void stopAutoSeq(const char *reason) {
  if (!autoSeqRunning) return;

  Serial.print("[AUTOSEQ] STOP: ");
  Serial.println(reason);

  autoSeqRunning = false;
  autoSeqStep = AS_IDLE;
  autoCycleCount = 0;
  autoWeighStableCnt = 0;

  isManualFeederOn = prevManualFeederOn;

  cmdConveyorStop();
}

void serviceAutoSeq() {
  if (!autoSeqRunning) return;

  // berhenti kalau ada yang mengubah mode/auto flag
  if (controlMode != MODE_AUTO) { stopAutoSeq("mode != AUTO"); return; }
  if (!autoModeEnabled)         { stopAutoSeq("autoModeEnabled = false"); return; }
  if (currentState == FAULT)    { stopAutoSeq("FAULT"); return; }

  const long DOOR_OPEN_POS  = doorOpenPos();
  const long DOOR_CLOSE_POS = doorClosePos();
  const long PUSH_IN_POS    = getOpenPos(1);
  const long PUSH_OUT_POS   = getClosePos(1);

  switch (autoSeqStep) {

    case AS_DOOR_OPEN: {
      cmdMainDoorOpen();
      Serial.println("[AUTOSEQ] STEP: MAINDOOR_OPEN");
      autoSeqStep = AS_WEIGHING;
      autoWeighStableCnt = 0;
    } break;

    case AS_WEIGHING: {
      if (handleWeighingAutoNoResidualHack()) {
        cmdConveyorStop();
        cmdPusherMaju();

        Serial.print("[AUTOSEQ] WEIGH OK -> PUSH_MAJU (cycle ");
        Serial.print(autoCycleCount + 1);
        Serial.println(")");

        autoSeqStep = AS_PUSH_MAJU;
      }
    } break;

    case AS_PUSH_MAJU: {
      if (stepperPush.distanceToGo() != 0) cmdConveyorStop();

      if (atTarget(stepperPush, PUSH_IN_POS)) {
        cmdConveyorStop();
        cmdPusherMundur();
        Serial.println("[AUTOSEQ] STEP: PUSH_MUNDUR");
        autoSeqStep = AS_PUSH_MUNDUR;
      }
    } break;

    case AS_PUSH_MUNDUR: {
      if (stepperPush.distanceToGo() != 0) cmdConveyorStop();

      if (atTarget(stepperPush, PUSH_OUT_POS)) {

        autoNeedDrop = true;           // wajib lihat berat turun dulu
        autoDropStableCnt = 0;         // reset counter drop
        autoWeighStableCnt = 0;        // reset counter naik

        autoCycleCount++;

        if (autoCycleCount < AUTO_REPEAT) {
          // ulang siklus lagi: WEIGHING -> PUSH_MAJU -> PUSH_MUNDUR
          autoWeighStableCnt = 0;
          Serial.println("[AUTOSEQ] NEXT CYCLE -> WEIGHING");
          autoSeqStep = AS_WEIGHING;
        } else {
          // setelah 4x siklus, baru tutup pintu
          cmdMainDoorClose();
          Serial.println("[AUTOSEQ] 4x cycles done -> MAINDOOR_CLOSE");
          autoSeqStep = AS_DOOR_CLOSE;
        }
      }
    } break;

    case AS_DOOR_CLOSE: {
      if (atTarget(stepperDoor, DOOR_CLOSE_POS)) {
        cmdConveyorStop();
        Serial.println("[AUTOSEQ] DONE -> STOP + BACK TO MANUAL");

        stopAutoSeq("DONE");

        // balik manual
        setAutoMode(false);
        setMode(MODE_MANUAL);

        burnerOff();
        ignOff();

        // optional: state utama kembali idle
        currentState = IDLE;
      }
    } break;


    default:
      break;
  }
}


// =========================
// COMMAND HANDLER
// =========================
void handleCommand(String cmd) {
  cmd.trim();
  if (cmd.length() == 0) return;

  // Status print saat command diterima
  Serial.print("[CMD] ");
  Serial.println(cmd);

  // =========================
  // DIRECT COMMANDS (no colon)  -> MANUAL => auto OFF
  // =========================
  if (cmd == "MAINDOOR_OPEN") {
    enterManualByCommand("MAINDOOR_OPEN");
    Serial.println("[ACT] Main Door OPEN");
    cmdMainDoorOpen();
    return;
  }
  if (cmd == "MAINDOOR_CLOSE") {
    enterManualByCommand("MAINDOOR_CLOSE");
    Serial.println("[ACT] Main Door CLOSE");
    cmdMainDoorClose();
    return;
  }

  if (cmd == "PUSHER_MAJU") {
    enterManualByCommand("PUSHER_MAJU");
    Serial.println("[ACT] Pusher MAJU");
    cmdPusherMaju();
    return;
  }
  if (cmd == "PUSHER_MUNDUR") {
    enterManualByCommand("PUSHER_MUNDUR");
    Serial.println("[ACT] Pusher MUNDUR");
    cmdPusherMundur();
    return;
  }

  if (cmd == "BDOOR_OPEN") {
    enterManualByCommand("BDOOR_OPEN");
    Serial.println("[ACT] Burn Door OPEN");
    cmdBDoorOpen();
    return;
  }
  if (cmd == "BDOOR_CLOSE") {
    enterManualByCommand("BDOOR_CLOSE");
    Serial.println("[ACT] Burn Door CLOSE");
    cmdBDoorClose();
    return;
  }

  if (cmd == "BURNER_MAJU") {
    enterManualByCommand("BURNER_MAJU");
    Serial.println("[ACT] Burner MAJU");
    cmdBurnerMaju();
    return;
  }
  if (cmd == "BURNER_MUNDUR") {
    enterManualByCommand("BURNER_MUNDUR");
    Serial.println("[ACT] Burner MUNDUR");
    cmdBurnerMundur();
    return;
  }

  if (cmd == "CONVEYOR_FWD") {
    enterManualByCommand("CONVEYOR_FWD");
    Serial.println("[ACT] Conveyor FORWARD");
    cmdConveyorFwd();
    return;
  }
  if (cmd == "CONVEYOR_REV") {
    enterManualByCommand("CONVEYOR_REV");
    Serial.println("[ACT] Conveyor REVERSE");
    cmdConveyorRev();
    return;
  }
  if (cmd == "CONVEYOR_STOP" || cmd == "CONVERYOR_STOP") {
    enterManualByCommand("CONVEYOR_STOP");
    Serial.println("[ACT] Conveyor STOP");
    cmdConveyorStop();
    return;
  }

  // =========================
  // ONE-SHOT COMMAND (manual macro) -> MANUAL => auto OFF
  // =========================
  if (cmd == "RUN_ONE_SHOOT") {
    enterManualByCommand("RUN_ONE_SHOOT");
    Serial.println("[ACT] RUN_ONE_SHOOT");
    startOneShot();
    return;
  }
  if (cmd == "CMD_RUN_AUTO") {
  Serial.println("[ACT] CMD_RUN_AUTO");
  startAutoSeq();
  return;
  }

  // // =========================
  // // PARAMETRIC / LEGACY COMMANDS (header:val)
  // // =========================
  int colon = cmd.indexOf(':');
  String header = (colon >= 0) ? cmd.substring(0, colon) : cmd;
  String valStr = (colon >= 0) ? cmd.substring(colon + 1) : String("");

  // if (header == "CMD_SENSOR_SNAPSHOT") {
  //   // snapshot boleh tanpa memaksa MANUAL/AUTO (data injection)
  //   applySensorSnapshot(valStr);
  //   Serial.println("[ACT] Sensor snapshot applied");
  //   return;
  // }

  // ---- manual relays & manual flags -> MANUAL => auto OFF
  if (header == "CMD_MAN_IGN") {
    enterManualByCommand("CMD_MAN_IGN");
    int v = valStr.toInt();
    (v ? ignOn() : ignOff());
    Serial.print("[ACT] IGN=");
    Serial.println(v);
    return;
  }

  if (header == "CMD_MAN_BURN") {
    enterManualByCommand("CMD_MAN_BURN");
    int v = valStr.toInt();
    (v ? burnerOn() : burnerOff());
    Serial.print("[ACT] BURN=");
    Serial.println(v);
    return;
  }

  if (header == "CMD_MAN_FEED") {
    enterManualByCommand("CMD_MAN_FEED");
    int v = valStr.toInt();
    isManualFeederOn = (v != 0);
    if (!isManualFeederOn) stepperMainConveyor.setSpeed(0);
    Serial.print("[ACT] MAN_FEED=");
    Serial.println((int)isManualFeederOn);
    return;
  }

  if (header == "CMD_MAN_WEIGH") {
    enterManualByCommand("CMD_MAN_WEIGH");
    int v = valStr.toInt();
    isManualWeighingOn = (v != 0);
    Serial.print("[ACT] MAN_WEIGH=");
    Serial.println((int)isManualWeighingOn);
    return;
  }

  // ---- auto control -> AUTO
  if (header == "CMD_AUTO_START") {
    setMode(MODE_AUTO);
    setAutoMode(true);
    if (currentState == IDLE) currentState = WEIGHING;
    Serial.println("[AUTO] START");
    return;
  }

  if (header == "CMD_AUTO_STOP") {
    stopAutoSeq("CMD_AUTO_STOP");
    setAutoMode(false);
    autoModeEnabled = false;
    setMode(MODE_MANUAL);
    Serial.println("[AUTO] STOP");
    return;
  }


  if (header == "CMD_ONE_SHOT_AUTO") {
    // ini bagian AUTO sequence, jadi masuk MODE_AUTO
    setMode(MODE_AUTO);
    oneShotAutoMode = true;
    if (currentState == IDLE) currentState = OPENING_MAIN_DOOR;
    Serial.println("[AUTO] ONE_SHOT_AUTO");
    return;
  }

  // ---- safety -> MANUAL
  if (header == "CMD_EMERGENCY_STOP") {
    emergencyStop();
    currentState = FAULT;
    setMode(MODE_MANUAL);       // paksa manual setelah E-stop
    Serial.println("[ACT] EMERGENCY_STOP");
    return;
  }

  // ---- settings: kamu bisa pilih mau MANUAL atau tidak
  // Biasanya aman kalau setting boleh kapan saja tanpa pindah mode
  if (header == "M_SPD") {
    int c = valStr.indexOf(':');
    if (c > 0) {
      int id = valStr.substring(0, c).toInt();
      long spd = valStr.substring(c + 1).toInt();
      if (id >= 0 && id < 6) {
        motorConfigs[id].speed = spd;
        applyMotorConfigsToSteppers();
        Serial.print("[ACT] M_SPD id=");
        Serial.print(id);
        Serial.print(" spd=");
        Serial.println(spd);
        return;
      }
    }
    Serial.println("[WARN] Bad M_SPD format");
    return;
  }

  if (header == "M_STP") {
    int c = valStr.indexOf(':');
    if (c > 0) {
      int id = valStr.substring(0, c).toInt();
      long stp = valStr.substring(c + 1).toInt();
      if (id >= 0 && id < 6) {
        motorConfigs[id].steps = stp;
        Serial.print("[ACT] M_STP id=");
        Serial.print(id);
        Serial.print(" stp=");
        Serial.println(stp);
        return;
      }
    }
    Serial.println("[WARN] Bad M_STP format");
    return;
  }

  if (header == "SET_W") {
    minBatchWeight = valStr.toFloat();
    if (!isfinite(minBatchWeight) || minBatchWeight < 0) minBatchWeight = 0;
    Serial.print("[ACT] SET_W=");
    Serial.println(minBatchWeight, 2);
    return;
  }

  if (header == "SET_TE") {
    tempSetpoint = valStr.toFloat();
    if (!isfinite(tempSetpoint) || tempSetpoint < 0) tempSetpoint = 0;
    Serial.print("[ACT] SET_TE=");
    Serial.println(tempSetpoint, 1);
    return;
  }

  if (header == "SET_TI") {
    burnerActiveTimeSec = (uint32_t)valStr.toInt();
    if (burnerActiveTimeSec == 0) burnerActiveTimeSec = 1;
    Serial.print("[ACT] SET_TI=");
    Serial.println(burnerActiveTimeSec);
    return;
  }

  if (header == "BLOW") {
    enterManualByCommand("BLOW");
    int c = valStr.indexOf(':');
    int lvl = (c > 0) ? valStr.substring(c + 1).toInt() : valStr.toInt();
    if (lvl > 0) blowOn(); else blowOff();
    Serial.print("[ACT] BLOW lvl=");
    Serial.println(lvl);
    return;
  }


  Serial.println("[WARN] Unknown command");
}

void runStateMachine() {
  if (controlMode != MODE_AUTO) return;  // KUNCI: state machine hanya AUTO
  if (!autoModeEnabled) return;          // auto harus aktif
  if (currentState == FAULT) return;

  switch (currentState) {
    case IDLE:               handleIdle(); break;
    case WEIGHING:           handleWeighing(); break;
    case OPENING_MAIN_DOOR:  handleOpeningMainDoor(); break;
    case DUMPING_IN:         handleDumpingIn(); break;
    case DUMPING_OUT:        handleDumpingOut(); break;
    case CLOSING_MAIN_DOOR:  handleClosingMainDoor(); break;
    case OPENING_BURN_DOOR:  handleOpeningBurnDoor(); break;
    case BURNER_IN:          handleBurnerIn(); break;
    case IGNITING:           handleIgniting(); break;
    case BURNER_OUT:         handleBurnerOut(); break;
    case CLOSING_BURN_DOOR:  handleClosingBurnDoor(); break;

    // yang lain (PREHEAT/BURNING dsb) nanti bisa kamu lengkapi
    default:
      break;
  }
}

// =========================
// STATE HANDLERS (AUTO)
// =========================
void handleIdle() {
  // Pastikan door di posisi close saat idle
  long closePos = doorClosePos();
  if (stepperDoor.targetPosition() != closePos) stepperDoor.moveTo(closePos);
}

static inline bool handleWeighingOneShoot() {
  cmdConveyorFwd();

  static uint8_t stableCnt = 0;
  const uint8_t STABLE_NEED = 3; // contoh: harus valid 3x

  if (loadWeight >= minLoadWeight) {
    if (stableCnt < STABLE_NEED) stableCnt++;
  } else {
    stableCnt = 0;
  }

  return (stableCnt >= STABLE_NEED);
}

void handleWeighing() {
  // contoh sederhana: blower ON saat timbang
  blowOn();

  // kalau berat sudah cukup, lanjut buka pintu utama
  if (loadWeight >= minLoadWeight) {
    currentState = OPENING_MAIN_DOOR;
  }
}

static inline bool handleWeighingAutoNoResidualHack() {
  cmdConveyorFwd();

  // 1) setelah push, kita WAJIB lihat berat turun dulu (di bawah reset threshold)
  float resetTh = (minLoadWeight - WEIGH_RESET_HYS);
  if (resetTh < 0) resetTh = 0;

  if (autoNeedDrop) {
    if (loadWeight < resetTh) {
      if (autoDropStableCnt < AUTO_STABLE_NEED) autoDropStableCnt++;
    } else {
      autoDropStableCnt = 0;
    }

    if (autoDropStableCnt >= AUTO_STABLE_NEED) {
      autoNeedDrop = false;           // sudah “turun”, sekarang boleh nunggu naik
      autoWeighStableCnt = 0;
    }
    return false;
  }

  // 2) sekarang baru nunggu naik sampai minLoadWeight (stabil)
  if (loadWeight >= minLoadWeight) {
    if (autoWeighStableCnt < AUTO_STABLE_NEED) autoWeighStableCnt++;
  } else {
    autoWeighStableCnt = 0;
  }

  return (autoWeighStableCnt >= AUTO_STABLE_NEED);
}



void handleOpeningMainDoor() {
  long targetPos = doorOpenPos();
  blowOff();

  if (stepperDoor.targetPosition() != targetPos) stepperDoor.moveTo(targetPos);

  if (stepperDoor.distanceToGo() == 0 && stepperDoor.currentPosition() == targetPos) {
    currentState = DUMPING_IN;
  }
}

void handleDumpingIn() {
  long targetPos = getOpenPos(1);

  if (stepperPush.targetPosition() != targetPos) stepperPush.moveTo(targetPos);

  if (stepperPush.distanceToGo() == 0 && stepperPush.currentPosition() == targetPos) {
    currentState = DUMPING_OUT;
    loadWeight = 0; // simulasi habis dump
  }
}

void handleDumpingOut() {
  long targetPos = getClosePos(1);

  if (stepperPush.targetPosition() != targetPos) stepperPush.moveTo(targetPos);

  if (stepperPush.distanceToGo() == 0 && stepperPush.currentPosition() == targetPos) {
    currentState = CLOSING_MAIN_DOOR;
  }
}

void handleClosingMainDoor() {
  long targetPos = doorClosePos();

  if (stepperDoor.targetPosition() != targetPos) stepperDoor.moveTo(targetPos);

  if (stepperDoor.distanceToGo() == 0 && stepperDoor.currentPosition() == targetPos) {
    // jika masih panas, skip pemanas, tutup burn door lalu burning
    if (currentTempC >= tempSetpoint) {
      burnerOff();
      blowOff();
      stepperBurner.moveTo(getClosePos(2));
      stepperBurnDoor.moveTo(getClosePos(3));
      startBurningAfterClose = true;
      currentState = CLOSING_BURN_DOOR;
    } else {
      currentState = OPENING_BURN_DOOR;
    }
  }

  blowOn();
}

void handleOpeningBurnDoor() {
  long targetPos = bdoorOpenPos();

  if (stepperBurnDoor.targetPosition() != targetPos) stepperBurnDoor.moveTo(targetPos);

  if (stepperBurnDoor.distanceToGo() == 0 && stepperBurnDoor.currentPosition() == targetPos) {
    currentState = BURNER_IN;
  }
}

void handleBurnerIn() {
  long targetPos = getOpenPos(2);

  if (stepperBurner.targetPosition() != targetPos) stepperBurner.moveTo(targetPos);

  if (stepperBurner.distanceToGo() == 0 && stepperBurner.currentPosition() == targetPos) {
    currentState = IGNITING;
  }
}

void handleIgniting() {
  burnerOn();

  if (!isPriming) {
    isPriming = true;
    ignitiondelayStart = millis();
    burnAccumulatedMs = 0;
    burnPhaseStartMs = 0;
  }

  // (opsional) setelah delay, kamu bisa lanjut state berikutnya
  // misal: currentState = BURNER_OUT; dst
}

void handleBurnerOut() {
  burnerOff();

  long targetPos = getClosePos(2);
  if (stepperBurner.targetPosition() != targetPos) stepperBurner.moveTo(targetPos);

  if (stepperBurner.distanceToGo() == 0 && stepperBurner.currentPosition() == targetPos) {
    currentState = CLOSING_BURN_DOOR;
  }
}

void handleClosingBurnDoor() {
  long targetPos = bdoorClosePos();

  if (stepperBurnDoor.targetPosition() != targetPos) stepperBurnDoor.moveTo(targetPos);

  bool doorClosed = (stepperBurnDoor.distanceToGo() == 0 && stepperBurnDoor.currentPosition() == targetPos);
  bool burnerOutOK = (stepperBurner.distanceToGo() == 0 && stepperBurner.currentPosition() == getClosePos(2));

  if (!doorClosed) return;

  if (startBurningAfterClose) {
    if (burnerOutOK) {
      startBurningAfterClose = false;
      startBurningHold();     // masuk BURNING
    }
    return;
  }

  // kalau AUTO masih aktif, balik timbang lagi
  if (autoModeEnabled) currentState = WEIGHING;
  else currentState = IDLE;
}


// =========================
// SETUP
// =========================
void setup() {
  Serial.begin(115200);

  pinMode(LIMIT_PIN_PUSH_HOME, INPUT_PULLUP);
  pinMode(LIMIT_PIN_DOOR_HOME, INPUT_PULLUP);
  pinMode(LIMIT_PIN_BDOOR_HOME, INPUT_PULLUP);

  pinMode(RELAY_IGNITION, OUTPUT);
  pinMode(RELAY_BLOWER, OUTPUT);
  pinMode(RELAY_BURNER, OUTPUT);

  burnerOff();
  ignOff();
  blowOff();

  pinMode(RS485_DE_RE, OUTPUT);
  rs485SetTx(false);
  RS485_BUS.begin(RS485_BAUD);

  // Direction inversions (match your earlier behavior)
  stepperBurner.setPinsInverted(true, false, true);
  stepperMainConveyor.setPinsInverted(true, false, true);

  applyMotorConfigsToSteppers();

  // Homing pusher saat startup
  homePusher();

  // Homing main door saat startup
  homeDoor();

  homeBDoor();

  Serial.println("=== SLAVE1 READY (USB COMMAND ENABLED) ===");
  Serial.println("Commands:");
  Serial.println("  MAINDOOR_OPEN | MAINDOOR_CLOSE");
  Serial.println("  PUSHER_MAJU   | PUSHER_MUNDUR");
  Serial.println("  BDOOR_OPEN    | BDOOR_CLOSE");
  Serial.println("  BURNER_MAJU   | BURNER_MUNDUR");
  Serial.println("  CONVEYOR_FWD  | CONVEYOR_REV | CONVEYOR_STOP");
  Serial.println("  (also accepts typo: CONVERYOR_STOP)");
}

void homeDoor() {
  const int HOME_DIR = -1;
  const float SEEK_SPEED_1 = 500;
  const float SEEK_SPEED_2 = 100;
  const long  BACKOFF_STEPS = 300;

  const unsigned long HOMING_TIMEOUT_MS = 30000;
  const uint8_t PRESS_STABLE_COUNT = 8;
  const uint8_t RELEASE_STABLE_COUNT = 8;

  auto waitPressedStable = [&](float spd) {
    const unsigned long t0 = millis();
    uint8_t cnt = 0;
    stepperDoor.setSpeed(spd);
    while (cnt < PRESS_STABLE_COUNT) {
      stepperDoor.runSpeed();
      if (doorHomePressed()) cnt++; else cnt = 0;
      if (millis() - t0 > HOMING_TIMEOUT_MS) return false;
    }
    return true;
  };

  auto waitReleasedStable = [&](float spd) {
    const unsigned long t0 = millis();
    uint8_t cnt = 0;
    stepperDoor.setSpeed(spd);
    while (cnt < RELEASE_STABLE_COUNT) {
      stepperDoor.runSpeed();
      if (!doorHomePressed()) cnt++; else cnt = 0;
      if (millis() - t0 > HOMING_TIMEOUT_MS) return false;
    }
    return true;
  };

  stepperDoor.setAcceleration(200);
  stepperDoor.setMaxSpeed(10000);

  Serial.print("Homing door... limitRaw=");
  Serial.println(digitalRead(LIMIT_PIN_DOOR_HOME));

  if (doorHomePressed()) {
    if (!waitReleasedStable(-HOME_DIR * SEEK_SPEED_1)) {
      stepperDoor.setSpeed(0);
      Serial.println("ERR: Door homing timeout (release). Check switch/wiring");
      return;
    }
  }

  if (!waitPressedStable(HOME_DIR * SEEK_SPEED_1)) {
    stepperDoor.setSpeed(0);
    Serial.println("ERR: Door homing timeout (seek1). Flip HOME_DIR if direction wrong");
    return;
  }

  stepperDoor.setCurrentPosition(0);
  stepperDoor.moveTo(-HOME_DIR * BACKOFF_STEPS);
  while (stepperDoor.distanceToGo() != 0) {
    stepperDoor.run();
  }

  if (doorHomePressed()) {
    if (!waitReleasedStable(-HOME_DIR * SEEK_SPEED_2)) {
      stepperDoor.setSpeed(0);
      Serial.println("ERR: Door homing timeout (release2). Check switch/wiring");
      return;
    }
  }

  if (!waitPressedStable(HOME_DIR * SEEK_SPEED_2)) {
    stepperDoor.setSpeed(0);
    Serial.println("ERR: Door homing timeout (seek2). Flip HOME_DIR if direction wrong");
    return;
  }

  stepperDoor.setCurrentPosition(0);
  stepperDoor.moveTo(0);

  Serial.println("OK: Door homed (pos=0)");
    // =========================
  // OPTION A: Setelah homing, tutup pintu otomatis
  // (sama persis seperti perintah MAINDOOR_CLOSE)
  // =========================
  cmdMainDoorClose();

  while (stepperDoor.distanceToGo() != 0) {
  stepperDoor.run();
  }

  Serial.println("OK: Door closed after homing");
}

// =========================
// LOOP
// =========================
void loop() {
  /// Realtime motor running
  runAllSteppers();

  // one-shot service (non-blocking)
  serviceOneShot();  
  // rs485SetTx(false); 
  serviceAutoSeq();


  // slow tick services
  static unsigned long lastSlow = 0;
  unsigned long now = millis();
  if (now - lastSlow >= 50) {
    lastSlow = now;
    checkStepperFlags();
    checkFeederConveyor(); // akan kita kunci oleh MODE juga (lihat bawah)
    runStateMachine();     // AKTIFKAN
  }

  // USB commands (Serial Monitor)
  processUsbCommands();

  // RS485 sensors + periodic print
  processRs485Unified();

  periodicPrint();
}
