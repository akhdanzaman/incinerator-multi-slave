/*
 * SLAVE1 - MEGA (Actuator Controller) | RS485 Simple ASCII
 * -------------------------------------------------------
 * Goal: Menyamakan fungsi handling state, relay, stepper, burn-cycle, conveyor, manual control,
 *       dan telemetry dengan referensi MCU-CTRL, tapi transportnya via RS485 simple driver.
 *
 * Frame RX dari Master (diterima slave):
 *   TO<id>:<payload>\n
 *   contoh: TO01:CMD_AUTO_START
 *
 * Frame TX dari Slave (kirim ke Master):
 *   N<id>:<payload>\n
 *   contoh: N01:DATA:....
 *
 * Catatan:
 * - Sensor (temp/weight/volume) diasumsikan di-inject / diset dari luar (master atau slave sensor).
 *   Jadi di sini ada handler CMD_SENSOR_SNAPSHOT (opsional) untuk update currentTempC/currentWeightKg/currentBatchVolume/totalVolume.
 * - Tidak ada CRC/retry/collision avoidance. Pakai polling master.
 */

#include <Arduino.h>
#include <AccelStepper.h>
#include <EEPROM.h>
#include <math.h>

// =========================
// RS485 SIMPLE DRIVER
// =========================
struct RS485Simple {
  HardwareSerial *ser;
  int pinDE;
  uint32_t baud;

  void begin(HardwareSerial &s, uint32_t b, int rx, int tx, int dePin) {
    ser = &s;
    baud = b;
    pinDE = dePin;
    pinMode(pinDE, OUTPUT);
    digitalWrite(pinDE, LOW); // RX
    ser->begin(baud, SERIAL_8N1, rx, tx);
  }

  // Mega overload (UART pins fixed)
  void beginMega(HardwareSerial &s, uint32_t b, int dePin) {
    ser = &s;
    baud = b;
    pinDE = dePin;
    pinMode(pinDE, OUTPUT);
    digitalWrite(pinDE, LOW);
    ser->begin(baud);
  }

  void txMode(bool en) { digitalWrite(pinDE, en ? HIGH : LOW); }

  void sendLine(const String &line) {
    txMode(true);
    ser->print(line);
    ser->print('\n');
    ser->flush();
    txMode(false);
  }

  // baca 1 line non-blocking, return true kalau dapat \n
  bool readLine(String &out) {
    while (ser->available()) {
      char c = (char)ser->read();
      if (c == '\r') continue;
      if (c == '\n') {
        out.trim();
        return out.length() > 0;
      }
      out += c;
      if (out.length() > 256) out = ""; // safety
    }
    return false;
  }
};

// =========================
// NODE / BUS CONFIG
// =========================
static const uint8_t NODE_ID = 1; // SLAVE1 address
static const int PIN_RS485_DE = 2; // adjust

RS485Simple bus;

static inline void sendToMaster(const String &payload) {
  char hdr[8];
  snprintf(hdr, sizeof(hdr), "N%02u:", (unsigned)NODE_ID);
  bus.sendLine(String(hdr) + payload);
}

// =========================
// STATE MACHINE
// =========================
enum SystemState {
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

static inline bool isDumpingState(SystemState s) {
  return (s == DUMPING_IN || s == DUMPING_OUT);
}

// =========================
// LIMIT SWITCH MIRRORS (injected from Master / Slave2)
// =========================
volatile uint8_t limDoor0  = 0;
volatile uint8_t limPush0  = 0;
volatile uint8_t limBurn0  = 0;
volatile uint8_t limBDoor0 = 0;


// =========================
// HARDWARE PINS (sesuaikan wiring kamu)
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

static const int ENABLE_PIN = 22; // Active LOW enable

#define RELAY_BLOWER   32
#define RELAY_BURNER   34
#define RELAY_IGNITION 33

#define PIN_FUEL_ADC   A0

// ===== Relay helpers (asumsi ACTIVE LOW) =====
inline void burnerOn() { digitalWrite(RELAY_BURNER, LOW); }
inline void burnerOff() { digitalWrite(RELAY_BURNER, HIGH); }
inline void blowOn() { digitalWrite(RELAY_BLOWER, LOW); }
inline void blowOff() { digitalWrite(RELAY_BLOWER, HIGH); }
inline void ignOn() { digitalWrite(RELAY_IGNITION, LOW); }
inline void ignOff() { digitalWrite(RELAY_IGNITION, HIGH); }

// =========================
// STEPPERS
// =========================
AccelStepper stepperDoor(AccelStepper::DRIVER, STEP_PIN_DOOR, DIR_PIN_DOOR);
AccelStepper stepperPush(AccelStepper::DRIVER, STEP_PIN_PUSH, DIR_PIN_PUSH);
AccelStepper stepperAsh(AccelStepper::DRIVER, STEP_PIN_ASH, DIR_PIN_ASH);
AccelStepper stepperBurner(AccelStepper::DRIVER, STEP_PIN_BURN, DIR_PIN_BURN);
AccelStepper stepperBurnDoor(AccelStepper::DRIVER, STEP_PIN_BDOOR, DIR_PIN_BDOOR);
AccelStepper stepperMainConveyor(AccelStepper::DRIVER, STEP_PIN_MAINC, DIR_PIN_MAINC);

static inline void setSteppersEnabled(bool en) {
  digitalWrite(ENABLE_PIN, en ? LOW : HIGH);
}

static inline bool anyStepperBusy() {
  if (stepperDoor.distanceToGo() != 0) return true;
  if (stepperPush.distanceToGo() != 0) return true;
  if (stepperAsh.distanceToGo() != 0) return true;
  if (stepperBurner.distanceToGo() != 0) return true;
  if (stepperBurnDoor.distanceToGo() != 0) return true;
  if (stepperMainConveyor.speed() != 0) return true;
  return false;
}

static inline void refreshEnablePin(SystemState currentState) {
  if (currentState == FAULT) {
    setSteppersEnabled(false);
    return;
  }
  setSteppersEnabled(anyStepperBusy());
}

// =========================
// CONFIG & VARS
// =========================
struct MotorConfig { long speed; long accel; long steps; };

// Index: 0 Door, 1 Push, 2 Burner, 3 BurnDoor, 4 Ash, 5 Main Conveyor
MotorConfig motorConfigs[6] = {
  { 10000, 200, 6000 },
  { 20000, 200, 10500 },
  { 10000, 200, 11500 },
  {  2000, 200,  1600 },
  {   800, 200,  1000 },
  { 20000, 200,  1000 }
};

static inline long getOpenPos(uint8_t idx)  { return motorConfigs[idx].steps; }
static inline long getClosePos(uint8_t idx) { (void)idx; return 0; }

// POS constants (computed from configs)
static inline long POS_DOOR_OPEN()   { return getOpenPos(0); }
static inline long POS_DOOR_CLOSE()  { return 0; }
static inline long POS_PUSH_IN()     { return getOpenPos(1); }
static inline long POS_PUSH_OUT()    { return 0; }
static inline long POS_BURNER_IN()   { return getOpenPos(2); }
static inline long POS_BURNER_OUT()  { return 0; }
static inline long POS_BDOOR_OPEN()  { return getOpenPos(3); }
static inline long POS_BDOOR_CLOSE() { return 0; }

// System vars
SystemState currentState = IDLE;
bool autoModeEnabled = false;
bool oneShotAutoMode = false;

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
static const float REHEAT_DROP_C = 400.0f;
static const unsigned long IGNITION_delay_MS = 20000;
static const float OVERHEAT_TEMP = 1400.0f;

bool isPriming = false;
unsigned long ignitiondelayStart = 0;
unsigned long burnAccumulatedMs = 0;
unsigned long burnPhaseStartMs = 0;
unsigned long lastCountdownUpdate = 0;
long last_remaining = -1;

bool startBurningAfterClose = false;

// Batch/metrics
float minBatchWeight = 5.0f;
float minBatchVolume = 100000.0f;
double currentBatchVolume = 0;
float totalVolume = 0;
float solarVolumeUsed_L = 0;
float totalConsumedKg = 0;
unsigned long totalBatchCount = 0;
static const float LITERS_PER_HOUR_BURNER = 10.0f;

float tempSetpoint = 1200.0f;
uint32_t burnerActiveTimeSec = 30;

// Sensor mirrors (diinject)
float currentTempC = 0.0f;
float currentWeightKg = 0.0f;

// Fuel
int FUEL_ADC_EMPTY = 200;
int FUEL_ADC_FULL  = 850;
uint8_t fuelPct = 0;
float fuelLiterRemaining = 0.0f;
static const float TANK_CAPACITY_L = 20.0f;
float fuelEma = 0;
static const float FUEL_EMA_ALPHA = 0.15f;

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
  setSteppersEnabled(false);
}

void computeMetrics() {
  float literPerSec = LITERS_PER_HOUR_BURNER / 3600.0f;
  float fuelPerBatch = literPerSec * (float)burnerActiveTimeSec;
  solarVolumeUsed_L += fuelPerBatch;

  if (currentWeightKg > 0.1f) totalConsumedKg += currentWeightKg;
  totalBatchCount++;

  // kirim metrics ringkas
  sendToMaster(String("METRICS:") + String(solarVolumeUsed_L, 2) + "," + String(totalConsumedKg, 2) + "," + String(totalBatchCount));

  // kirim batch summary ringkas
  sendToMaster(String("BATCH_SUM:") + String(totalBatchCount) + "," + String((float)currentBatchVolume, 3) + "," + String(currentWeightKg, 1) + "," + String(solarVolumeUsed_L, 2) + "," + String(currentTempC, 0) + "," + String(burnerActiveTimeSec) + ",OK");
}

static void applyLimitHoming() {
  // Door
  if (limDoor0) {
    if (stepperDoor.currentPosition() != 0) stepperDoor.setCurrentPosition(0);
    // kalau lagi coba “melewati” limit ke arah negatif, hentikan
    if (stepperDoor.distanceToGo() != 0 && stepperDoor.targetPosition() < stepperDoor.currentPosition()) {
      stepperDoor.stop();
      stepperDoor.moveTo(0);
    }
  }

  // Push
  if (limPush0) {
    if (stepperPush.currentPosition() != 0) stepperPush.setCurrentPosition(0);
    if (stepperPush.distanceToGo() != 0 && stepperPush.targetPosition() < stepperPush.currentPosition()) {
      stepperPush.stop();
      stepperPush.moveTo(0);
    }
  }

  // Burner
  if (limBurn0) {
    if (stepperBurner.currentPosition() != 0) stepperBurner.setCurrentPosition(0);
    if (stepperBurner.distanceToGo() != 0 && stepperBurner.targetPosition() < stepperBurner.currentPosition()) {
      stepperBurner.stop();
      stepperBurner.moveTo(0);
    }
  }

  // BurnDoor
  if (limBDoor0) {
    if (stepperBurnDoor.currentPosition() != 0) stepperBurnDoor.setCurrentPosition(0);
    if (stepperBurnDoor.distanceToGo() != 0 && stepperBurnDoor.targetPosition() < stepperBurnDoor.currentPosition()) {
      stepperBurnDoor.stop();
      stepperBurnDoor.moveTo(0);
    }
  }
}

static void applyLimitsSnapshot(const String &valStr) {
  // format: d,p,b,bd (0/1)
  int c1 = valStr.indexOf(',');
  int c2 = valStr.indexOf(',', c1 + 1);
  int c3 = valStr.indexOf(',', c2 + 1);
  if (c1 < 0 || c2 < 0 || c3 < 0) return;

  int d  = valStr.substring(0, c1).toInt();
  int p  = valStr.substring(c1 + 1, c2).toInt();
  int b  = valStr.substring(c2 + 1, c3).toInt();
  int bd = valStr.substring(c3 + 1).toInt();

  limDoor0  = (d  != 0);
  limPush0  = (p  != 0);
  limBurn0  = (b  != 0);
  limBDoor0 = (bd != 0);

  // langsung apply biar posisi 0 “nempel”
  applyLimitHoming();
}


// =========================
// PROGRESS (full-ish, match reference)
// =========================
int calculateProgress() {
  uint8_t val = 0;

  switch (currentState) {
    case WEIGHING:
      if (minBatchVolume > 0) {
        float ratio = (float)currentBatchVolume / minBatchVolume;
        if (ratio < 0) ratio = 0;
        if (ratio > 1) ratio = 1;
        val = (uint8_t)(ratio * 100.0f);
      } else {
        val = 0;
      }
      break;

    case OPENING_MAIN_DOOR: {
      long pos = stepperDoor.currentPosition();
      long a = POS_DOOR_CLOSE();
      long b = POS_DOOR_OPEN();
      if (b <= a) { val = 0; break; }
      if (pos < a) pos = a;
      if (pos > b) pos = b;
      val = (uint8_t)((pos - a) * 100L / (b - a));
      break;
    }

    case CLOSING_MAIN_DOOR: {
      long pos = stepperDoor.currentPosition();
      long a = POS_DOOR_CLOSE();
      long b = POS_DOOR_OPEN();
      if (b <= a) { val = 0; break; }
      if (pos < a) pos = a;
      if (pos > b) pos = b;
      val = (uint8_t)((b - pos) * 100L / (b - a));
      break;
    }

    case DUMPING_IN: {
      long pos = stepperPush.currentPosition();
      long a = POS_PUSH_OUT();
      long b = POS_PUSH_IN();
      if (b <= a) { val = 0; break; }
      if (pos < a) pos = a;
      if (pos > b) pos = b;
      val = (uint8_t)((pos - a) * 100L / (b - a));
      break;
    }

    case DUMPING_OUT: {
      long pos = stepperPush.currentPosition();
      long a = POS_PUSH_OUT();
      long b = POS_PUSH_IN();
      if (b <= a) { val = 0; break; }
      if (pos < a) pos = a;
      if (pos > b) pos = b;
      val = (uint8_t)((b - pos) * 100L / (b - a));
      break;
    }

    case OPENING_BURN_DOOR: {
      long pos = stepperBurnDoor.currentPosition();
      long a = POS_BDOOR_CLOSE();
      long b = POS_BDOOR_OPEN();
      if (b <= a) { val = 0; break; }
      if (pos < a) pos = a;
      if (pos > b) pos = b;
      val = (uint8_t)((pos - a) * 100L / (b - a));
      break;
    }

    case CLOSING_BURN_DOOR: {
      long pos = stepperBurnDoor.currentPosition();
      long a = POS_BDOOR_CLOSE();
      long b = POS_BDOOR_OPEN();
      if (b <= a) { val = 0; break; }
      if (pos < a) pos = a;
      if (pos > b) pos = b;
      val = (uint8_t)((b - pos) * 100L / (b - a));
      break;
    }

    case BURNER_IN: {
      long pos = stepperBurner.currentPosition();
      long a = POS_BURNER_OUT();
      long b = POS_BURNER_IN();
      if (b <= a) { val = 0; break; }
      if (pos < a) pos = a;
      if (pos > b) pos = b;
      val = (uint8_t)((pos - a) * 100L / (b - a));
      break;
    }

    case BURNER_OUT: {
      long pos = stepperBurner.currentPosition();
      long a = POS_BURNER_OUT();
      long b = POS_BURNER_IN();
      if (b <= a) { val = 0; break; }
      if (pos < a) pos = a;
      if (pos > b) pos = b;
      val = (uint8_t)((b - pos) * 100L / (b - a));
      break;
    }

    case IGNITING:
      if (isPriming) {
        unsigned long elapsed = millis() - ignitiondelayStart;
        if (elapsed > IGNITION_delay_MS) elapsed = IGNITION_delay_MS;
        val = (uint8_t)(elapsed * 100UL / IGNITION_delay_MS);
      } else {
        val = 0;
      }
      break;

    case BURNING: {
      unsigned long durationMS = burnerActiveTimeSec * 1000UL;
      unsigned long t = millis();
      unsigned long elapsed = burnAccumulatedMs + ((burnPhaseStartMs ? (t - burnPhaseStartMs) : 0));
      if (elapsed > durationMS) elapsed = durationMS;
      if (durationMS > 0) val = (uint8_t)(elapsed * 100UL / durationMS);
      else val = 0;
      break;
    }

    default:
      val = 0;
      break;
  }

  return (int)val;
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

  sendToMaster("BURNING_START");
}

void checkIgnitionPriming() {
  if (!isPriming) return;

  unsigned long now = millis();
  if (now - ignitiondelayStart >= IGNITION_delay_MS) {
    blowOff();
    isPriming = false;
    currentState = PREHEAT;
  } else {
    long remaining = (long)((IGNITION_delay_MS - (now - ignitiondelayStart)) / 1000UL);
    if (remaining != last_remaining) {
      last_remaining = remaining;
      sendToMaster(String("REM_TIME:") + String(remaining));
    }
  }
}

void serviceBurnCycle() {
  unsigned long t = millis();
  unsigned long durationMs = burnerActiveTimeSec * 1000UL;

  // PREHEAT: burner ON sampai setpoint
  if (currentState == PREHEAT) {
    burnerOn();
    sendToMaster("PREHEAT");

    if (currentTempC >= tempSetpoint) {
      burnerOff();
      startBurningAfterClose = true;
      currentState = BURNER_OUT;
    }
    return;
  }

  // BURNING hold
  if (currentState == BURNING) {
    burnerOff();

    if (currentTempC < (tempSetpoint - REHEAT_DROP_C)) {
      if (burnPhaseStartMs != 0) {
        burnAccumulatedMs += (t - burnPhaseStartMs);
        burnPhaseStartMs = 0;
      }
      startBurningAfterClose = false;
      currentState = OPENING_BURN_DOOR;
      return;
    }

    if (burnPhaseStartMs == 0) burnPhaseStartMs = t;

    unsigned long elapsed = burnAccumulatedMs + (t - burnPhaseStartMs);
    if (elapsed >= durationMs) {
      burnAccumulatedMs = 0;
      burnPhaseStartMs = 0;
      startBurningAfterClose = false;
      currentState = BURNER_OUT;
      return;
    }

    if (t - lastCountdownUpdate >= 1000) {
      lastCountdownUpdate = t;
      long remainingSeconds = (long)((durationMs - elapsed) / 1000UL);
      if (remainingSeconds < 0) remainingSeconds = 0;
      sendToMaster(String("REM_TIME:") + String(remainingSeconds));
    }
    return;
  }
}

void checkFeederConveyor() {
  if (isManualFeederOn) return;

  static bool conveyorRunning = false;
  bool shouldRun = (autoModeEnabled) && (currentState != FAULT) && (!isDumpingState(currentState)) && (currentBatchVolume < minBatchVolume);

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
// STATE HANDLERS (match reference behavior)
// =========================
void handleIdle() {
  long closePos = getClosePos(0);
  if (stepperDoor.targetPosition() != closePos) stepperDoor.moveTo(closePos);
}

void handleWeighing() {
  blowOn();
  if (currentBatchVolume >= minBatchVolume) {
    currentState = OPENING_MAIN_DOOR;
  }
}

void handleOpeningMainDoor() {
  long targetPos = getOpenPos(0);
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
    computeMetrics();
    currentState = DUMPING_OUT;
    currentBatchVolume = 0;
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
  long targetPos = getClosePos(0);

  if (stepperDoor.targetPosition() != targetPos) stepperDoor.moveTo(targetPos);
  if (stepperDoor.distanceToGo() == 0 && stepperDoor.currentPosition() == targetPos) {
    // skip pemanas jika masih panas
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
  long targetPos = getOpenPos(3);

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
    sendToMaster("IGNITING");
  }
}

void handlePreheat() {
  // dikontrol serviceBurnCycle()
}

void handleBurning() {
  // dikontrol serviceBurnCycle()
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
  long targetPos = getClosePos(3);

  if (stepperBurnDoor.targetPosition() != targetPos) stepperBurnDoor.moveTo(targetPos);

  bool doorClosed = (stepperBurnDoor.distanceToGo() == 0 && stepperBurnDoor.currentPosition() == targetPos);
  bool burnerOutOK = (stepperBurner.distanceToGo() == 0 && stepperBurner.currentPosition() == getClosePos(2));

  if (doorClosed) {
    if (startBurningAfterClose) {
      if (burnerOutOK) {
        startBurningAfterClose = false;
        startBurningHold();
      }
    } else {
      if (oneShotAutoMode) {
        currentState = IDLE;
        oneShotAutoMode = false;
      } else if (autoModeEnabled) {
        currentState = WEIGHING;
      } else {
        currentState = IDLE;
      }
    }
  }
}

void handleFault() {
  emergencyStop();
  autoModeEnabled = false;
  currentState = IDLE;
}

void runStateMachine() {
  // safety basic
  if (currentTempC >= OVERHEAT_TEMP) {
    currentState = FAULT;
  }

  switch (currentState) {
    case IDLE: handleIdle(); break;
    case WEIGHING: handleWeighing(); break;
    case OPENING_MAIN_DOOR: handleOpeningMainDoor(); break;
    case DUMPING_IN: handleDumpingIn(); break;
    case DUMPING_OUT: handleDumpingOut(); break;
    case CLOSING_MAIN_DOOR: handleClosingMainDoor(); break;
    case OPENING_BURN_DOOR: handleOpeningBurnDoor(); break;
    case BURNER_IN: handleBurnerIn(); break;
    case IGNITING: handleIgniting(); break;
    case PREHEAT: handlePreheat(); break;
    case BURNING: handleBurning(); break;
    case BURNER_OUT: handleBurnerOut(); break;
    case CLOSING_BURN_DOOR: handleClosingBurnDoor(); break;
    case FAULT: handleFault(); break;
  }
}

// =========================
// RUN ALL STEPPERS
// =========================
void runAllSteppers() {
  if (currentState == FAULT) {
    refreshEnablePin(currentState);
    return;
  }

  applyLimitHoming();

  stepperDoor.run();
  stepperPush.run();
  stepperAsh.run();
  stepperBurner.run();
  stepperBurnDoor.run();
  stepperMainConveyor.runSpeed();

  refreshEnablePin(currentState);
}


// =========================
// MANUAL MOVE
// =========================
void runManualStepper(AccelStepper &motor) {
  if (currentState != IDLE) return;

  motor.setMaxSpeed(manualSpeed);
  motor.setAcceleration(2000);

  long delta = isManualOpen ? manualSteps : -manualSteps;
  long tgt = motor.currentPosition() + delta;
  motor.moveTo(tgt);
}

void executeManualMove(int id) {
  switch (id) {
    case 0: runManualStepper(stepperDoor); break;
    case 1: runManualStepper(stepperPush); break;
    case 2: runManualStepper(stepperBurner); break;
    case 3: runManualStepper(stepperBurnDoor); break;
    case 4: runManualStepper(stepperAsh); break;
    case 5:
      // conveyor: treat as stepper but better as runSpeed; keep consistent with reference
      runManualStepper(stepperMainConveyor);
      break;
  }
}

// =========================
// TELEMETRY (DATA/PROG/FUEL)
// =========================
void sendTelemetry() {
  unsigned long now = millis();

  if (now - lastSensorTxMs >= 500) {
    lastSensorTxMs = now;
    // DATA:TEMP,WEIGHT,STATE,BATCHVOL,TOTALVOL
    String line = String("DATA:") + String(currentTempC, 1) + "," + String(currentWeightKg, 2) + "," + String((int)currentState) + "," + String((float)currentBatchVolume, 3) + "," + String(totalVolume, 3);
    sendToMaster(line);
  }

  if (now - lastProgTxMs >= 500) {
    lastProgTxMs = now;
    sendToMaster(String("PROG:") + String(calculateProgress()));
  }

  if (now - lastFuelTxMs >= 500) {
    lastFuelTxMs = now;
    fuelPct = readFuelPercent();
    fuelLiterRemaining = (fuelPct / 100.0f) * TANK_CAPACITY_L;
    sendToMaster(String("FUEL:") + String((int)fuelPct) + "," + String(fuelLiterRemaining, 1));
  }
}

// =========================
// RS485 RX: parse TOxx:payload
// =========================
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

// optional snapshot: CMD_SENSOR_SNAPSHOT:T,W,BATCHVOL,TOTALVOL
static void applySensorSnapshot(const String &valStr) {
  // format: temp,weight,batchVol,totalVol
  int c1 = valStr.indexOf(',');
  int c2 = valStr.indexOf(',', c1 + 1);
  int c3 = valStr.indexOf(',', c2 + 1);
  if (c1 < 0 || c2 < 0 || c3 < 0) return;

  float t = valStr.substring(0, c1).toFloat();
  float w = valStr.substring(c1 + 1, c2).toFloat();
  double bv = valStr.substring(c2 + 1, c3).toFloat();
  float tv = valStr.substring(c3 + 1).toFloat();

  if (isfinite(t)) currentTempC = t;
  if (isfinite(w)) currentWeightKg = w;
  if (isfinite((float)bv) && bv >= 0) currentBatchVolume = bv;
  if (isfinite(tv) && tv >= 0) totalVolume = tv;
}

// =========================
// COMMAND HANDLER (match reference names)
// =========================
void handleCommand(String cmd) {
  cmd.trim();
  if (cmd.length() == 0) return;

  // split header:val
  int colon = cmd.indexOf(':');
  String header = (colon >= 0) ? cmd.substring(0, colon) : cmd;
  String valStr = (colon >= 0) ? cmd.substring(colon + 1) : String("");
  
  // ---- limits snapshot ----
  if (header == "CMD_LIMITS") {
    applyLimitsSnapshot(valStr);
    return;
  }

  // ---- sensor snapshot ----
  if (header == "CMD_SENSOR_SNAPSHOT") {
    applySensorSnapshot(valStr);
    return;
  }

  // ---- manual relays ----
  if (header == "CMD_MAN_IGN") {
    int v = valStr.toInt();
    (v ? ignOn() : ignOff());
    sendToMaster("LOG:INFO,MAN,IGN=" + String(v));
    return;
  }

  if (header == "CMD_MAN_BURN") {
    int v = valStr.toInt();
    (v ? burnerOn() : burnerOff());
    sendToMaster("LOG:INFO,MAN,BURN=" + String(v));
    return;
  }

  if (header == "CMD_MAN_FEED") {
    int v = valStr.toInt();
    isManualFeederOn = (v != 0);
    if (!isManualFeederOn) stepperMainConveyor.setSpeed(0);
    return;
  }

  if (header == "CMD_MAN_WEIGH") {
    int v = valStr.toInt();
    isManualWeighingOn = (v != 0);
    return;
  }

  if (header == "CMD_SET_MANUAL_DIR") {
    int v = valStr.toInt();
    isManualOpen = (v != 0);
    return;
  }

  if (header == "SET_M_SPD") {
    manualSpeed = valStr.toInt();
    return;
  }

  if (header == "SET_M_STP") {
    manualSteps = valStr.toInt();
    return;
  }

  // ---- sync request ----
  if (header == "CMD_REQ_SYNC") {
    sendToMaster(String("W_SET:") + String(minBatchWeight, 1));
    sendToMaster(String("TE_SET:") + String(tempSetpoint, 0));
    sendToMaster(String("TI_SET:") + String(burnerActiveTimeSec));
    sendToMaster(String("VB_SET:") + String(minBatchVolume, 0));

    for (int i = 0; i < 6; i++) {
      sendToMaster(String("M_SYNC:") + String(i) + ":" + String(motorConfigs[i].speed) + ":" + String(motorConfigs[i].steps));
    }
    return;
  }

  // ---- auto control ----
  if (header == "CMD_AUTO_START") {
    setAutoMode(true);
    if (currentState == IDLE) currentState = WEIGHING;
    return;
  }

  if (header == "CMD_AUTO_STOP") {
    setAutoMode(false);
    autoModeEnabled = false;
    // tetap biarkan state apa adanya (lebih aman)
    return;
  }

  if (header == "CMD_ONE_SHOT_AUTO") {
    oneShotAutoMode = true;
    if (currentState == IDLE) currentState = OPENING_MAIN_DOOR;
    return;
  }

  if (header == "CMD_EMERGENCY_STOP") {
    emergencyStop();
    currentState = FAULT;
    return;
  }

  // ---- settings ----
  if (header == "CMD_SAVE_SETTINGS") {
    // kamu bisa isi EEPROM save beneran kalau mau. Untuk sekarang: cukup ACK.
    sendToMaster("LOG:INFO,EEPROM,SAVED");
    return;
  }

  if (header == "REQ_MOVE") {
    int id = valStr.toInt();
    executeManualMove(id);
    return;
  }

  // motor configs
  if (header == "M_SPD") {
    // format: M_SPD:<id>:<speed>  -> karena split awal cuma ambil header+valStr
    int c = valStr.indexOf(':');
    if (c > 0) {
      int id = valStr.substring(0, c).toInt();
      long spd = valStr.substring(c + 1).toInt();
      if (id >= 0 && id < 6) {
        motorConfigs[id].speed = spd;
        applyMotorConfigsToSteppers();
      }
    }
    return;
  }

  if (header == "M_STP") {
    int c = valStr.indexOf(':');
    if (c > 0) {
      int id = valStr.substring(0, c).toInt();
      long stp = valStr.substring(c + 1).toInt();
      if (id >= 0 && id < 6) {
        motorConfigs[id].steps = stp;
      }
    }
    return;
  }

  if (header == "SET_VB") {
    minBatchVolume = valStr.toFloat();
    if (!isfinite(minBatchVolume) || minBatchVolume <= 0) minBatchVolume = 1000.0f;
    return;
  }

  if (header == "SET_W") {
    minBatchWeight = valStr.toFloat();
    if (!isfinite(minBatchWeight) || minBatchWeight < 0) minBatchWeight = 0;
    return;
  }

  if (header == "SET_TE") {
    tempSetpoint = valStr.toFloat();
    if (!isfinite(tempSetpoint) || tempSetpoint < 0) tempSetpoint = 0;
    return;
  }

  if (header == "SET_TI") {
    burnerActiveTimeSec = (uint32_t)valStr.toInt();
    if (burnerActiveTimeSec == 0) burnerActiveTimeSec = 1;
    return;
  }

  if (header == "BLOW") {
    // format BLOW:<id>:<lvl> , tapi di slave1 kita cuma punya 1 blower relay -> on/off saja
    // treat any lvl>0 as ON
    int c = valStr.indexOf(':');
    int lvl = (c > 0) ? valStr.substring(c + 1).toInt() : valStr.toInt();
    if (lvl > 0) blowOn(); else blowOff();
    return;
  }

  // unknown
  sendToMaster(String("LOG:WARN,UNK,") + cmd);
}

// =========================
// BUS RX
// =========================
String rxLine;
String rxAccum;

void processBusRx() {
  while (bus.readLine(rxAccum)) {
    String line = rxAccum;
    rxAccum = "";

    uint8_t id = 0;
    String payload;
    if (!parseToFrame(line, id, payload)) {
      // noise
      continue;
    }
    if (id != NODE_ID) {
      // not for us
      continue;
    }
    handleCommand(payload);
  }
}

// =========================
// SETUP / LOOP
// =========================
void setup() {
  Serial.begin(115200);

  // RS485 on Serial1 (Mega)
  bus.beginMega(Serial1, 9600, PIN_RS485_DE);

  pinMode(RELAY_BURNER, OUTPUT);
  pinMode(RELAY_IGNITION, OUTPUT);
  pinMode(RELAY_BLOWER, OUTPUT);
  burnerOff(); ignOff(); blowOff();

  pinMode(ENABLE_PIN, OUTPUT);
  setSteppersEnabled(false);

  // Direction inversions (match your earlier behavior)
  stepperDoor.setPinsInverted(true, false, false);
  stepperBurner.setPinsInverted(true, false, true);
  stepperMainConveyor.setPinsInverted(true, false, true);

  applyMotorConfigsToSteppers();

  sendToMaster("LOG:INFO,BOOT,SLAVE1_READY");
}

void loop() {
  // Realtime motor running
  runAllSteppers();

  // slow tick services
  static unsigned long lastSlow = 0;
  unsigned long now = millis();
  if (now - lastSlow >= 50) {
    lastSlow = now;
    checkIgnitionPriming();
    checkStepperFlags();
    serviceBurnCycle();
    checkFeederConveyor();
    runStateMachine();
  }

  // RS485 RX
  processBusRx();

  // Telemetry TX
  sendTelemetry();
}
