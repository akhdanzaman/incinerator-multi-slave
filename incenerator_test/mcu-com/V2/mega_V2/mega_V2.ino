/*
 * MCU-CTRL (Realtime Machine Controller) ARDUINO MEGA
 * -------------------------------------
 * Tugas: Mengontrol Hardware (Stepper, Sensor, Relay, Lidar)
 * Menjalankan State Machine & Logic Keselamatan
 * Komunikasi: Menerima perintah dari HMI, Mengirim status ke HMI
 *
 * Perubahan utama (sesuai request kamu):
 * - Fix logika & pemetaan stepper (index motorConfigs vs POS_* salah, dan gerak jadi konsisten)
 * - Stepper enable dikelola otomatis (enable ON saat ada motor jalan)
 * - Handler state tidak lagi mengandalkan run() di dalam handler (runAllSteppers yang ngurus)
 * - Fungsi DIMMER dihapus (include RBDdimmer + objek + ramp/update)
 *
 * Catatan: Aku TIDAK menghapus fungsi lain selain yang terkait dimmer.
 */


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

// ================== INCLUDES ==================
#include <Arduino.h>
#include <AccelStepper.h>
#include "HX711.h"
#include <SPI.h>
#include <MAX31856.h>
#include <EEPROM.h>
#include <math.h>
#include <avr/io.h>

// ================== DEFINES & CONFIG ==================
#define USE_MAX31856 1  // 1 = Hardware, 0 = Simulasi
#define USE_HX711 1
#define USE_STEPPER 1  // 0 = Simulasi Gerak, 1 = Real Stepper
#define USE_LIDAR 0

// Konfigurasi Fisik
#define DEG_TO_RAD 0.01745329251
#define MIN_HEIGHT_DIFF 15.0        // mm
#define WALL_HEIGHT_THRESHOLD 40.0  // mm
#define OBJ_TIMEOUT_MS 500
#define CAL_OBJ_LENGTH_CM 15.0
#define CAL_OBJ_TARGET_HEIGHT 50.0  // mm
#define CAL_HEIGHT_TOLERANCE 10.0   // mm
#define CAL_ANGLE_WINDOW 3
#define CAL_OBJ_VOLUME_CM3 (15.0 * 15.0 * 5.0)

// EEPROM Addresses
#define EEPROM_ADDR_WEIGHT_SET 0
#define EEPROM_ADDR_TIME_SET 4
#define EEPROM_ADDR_TEMP_SET 8
#define EEPROM_ADDR_SCALE 12
#define EEPROM_ADDR_MOTOR_CFG 20
#define EEPROM_ADDR_LIDAR_SPEED_CMPS 100  // float (4 byte)
#define EEPROM_ADDR_LIDAR_VOL_SCALE 104   // float (4 byte)
#define EEPROM_ADDR_LIDAR_FLAGS 108       // uint8_t (1 byte)
#define EEPROM_ADDR_VOL_BATCH_SET 112

// MAX31856 Config
#define CR0_INIT (CR0_AUTOMATIC_CONVERSION + CR0_OPEN_CIRCUIT_FAULT_TYPE_K + CR0_NOISE_FILTER_50HZ)
#define CR1_INIT (CR1_AVERAGE_2_SAMPLES + CR1_THERMOCOUPLE_TYPE_S)
#define MASK_INIT (~(MASK_VOLTAGE_UNDER_OVER_FAULT + MASK_THERMOCOUPLE_OPEN_FAULT))

// ================== HX711 NON-BLOCKING FILTER ==================
// Tujuan: update weight halus tanpa get_units(n) yang bikin loop ketahan.

// tuning (silakan adjust)
static const float W_CLAMP_NEG_KG = 0.20f;  // noise minus kecil dianggap 0
static const float W_JUMP_KG = 1.50f;       // loncatan > ini dianggap spike (ignore)
static const float W_EMA_ALPHA = 0.18f;     // 0.1 halus banget, 0.3 lebih responsif
static const float W_DEADBAND_KG = 0.05f;   // perubahan kecil gak ditampilkan

static float wbuf[3] = { 0, 0, 0 };
static uint8_t wbi = 0;
static bool wbufFull = false;

static bool wEmaInit = false;
static float wEma = 0.0f;
static float wDisp = 0.0f;

static inline float median3(float a, float b, float c) {
  if (a > b) {
    float t = a;
    a = b;
    b = t;
  }
  if (b > c) {
    float t = b;
    b = c;
    c = t;
  }
  if (a > b) {
    float t = a;
    a = b;
    b = t;
  }
  return b;
}


// ================== PIN CONFIGURATION ==================
// Sensor Pins
const int HX711_DT = 41;
const int HX711_SCK = 40;
const int MAX31856_CS = 10;
const int MAX31856_SCK = 13;
const int MAX31856_SDI = 11;
const int MAX31856_SDO = 12;

// 74HC595 (7-Segment)
#define DATA_PIN 23
#define CLOCK_PIN 25
#define LATCH_PIN 24

// 74HC595 (7-Segment) Display 2 (TOTAL WEIGHT)
#define DATA_PIN_TW 47
#define CLOCK_PIN_TW 49
#define LATCH_PIN_TW 48

// Stepper Pins
#define STEP_PIN_DOOR 6
#define DIR_PIN_DOOR 7

#define STEP_PIN_PUSH 4
#define DIR_PIN_PUSH 5

#define STEP_PIN_BURN 26
#define DIR_PIN_BURN 27

#define STEP_PIN_BDOOR 28
#define DIR_PIN_BDOOR 29

// NOTE: Pastikan pin ini TIDAK bentrok dengan 74HC595.
// Kamu sebelumnya pakai 24/25 buat ASH stepper padahal itu LATCH/CLOCK.
// Aku ganti default ke pin aman. Kalau di wiring kamu beda, sesuaikan.
#define STEP_PIN_ASH 30
#define DIR_PIN_ASH 31

#define STEP_PIN_MAINC 8
#define DIR_PIN_MAINC 9

const int ENABLE_PIN = 22;  // Active LOW enable (LOW=enable, HIGH=disable)

// Blower & Relay Pins
const int PIN_FUEL_ADC = A0;

#define RELAY_BLOWER 32
#define RELAY_BURNER 34
#define RELAY_IGNITION 33

// ===== Relay helpers (asumsi ACTIVE LOW: LOW=ON, HIGH=OFF) =====
inline void burnerOn() {
  digitalWrite(RELAY_BURNER, LOW);
}
inline void burnerOff() {
  digitalWrite(RELAY_BURNER, HIGH);
}
inline void blowOn() {
  digitalWrite(RELAY_BLOWER, LOW);
}
inline void blowOff() {
  digitalWrite(RELAY_BLOWER, HIGH);
}
inline void ignOn() {
  digitalWrite(RELAY_IGNITION, LOW);
}
inline void ignOff() {
  digitalWrite(RELAY_IGNITION, HIGH);
}

const float REHEAT_DROP_C = 400.0f;

// Burn cycle timer (timer hanya dihitung saat burner OFF / state BURNING)
unsigned long burnAccumulatedMs = 0;
unsigned long burnPhaseStartMs = 0;
unsigned long volumeScaleFactor = 1.0f;

// ================== GLOBAL VARIABLES (LOGIC) ==================
// ===== NON-BLOCKING BACKGROUND CAL =====
bool bgCalActive = false;
unsigned long bgCalStartMs = 0;

// --- Motor Config Struct ---
struct MotorConfig {
  long speed;
  long accel;
  long steps;
};

// Default Configs (Akan ditimpa loadSettings)
// Index yang dipakai di seluruh kode:
// 0: Door, 1: Push, 2: Burner, 3: BurnDoor, 4: Ash, 5: Main Conveyor
MotorConfig motorConfigs[6] = {
  { 10000, 200, 6000 },   // 0: Door      (POS_DOOR_OPEN)
  { 20000, 200, 10500 },  // 1: Push      (POS_PUSH_IN)
  { 10000, 200, 11500 },  // 2: Burner    (POS_BURNER_IN)
  { 2000, 200, 1600 },    // 3: BurnDoor  (POS_BDOOR_OPEN)
  { 800, 200, 1000 },     // 4: Ash       (sementara)
  { 20000, 200, 1000 }    // 5: MAIN CONVEYOR (sementara)
};
MotorConfig motorConfigsBase[6];

long getOpenPos(uint8_t idx) {
  return motorConfigs[idx].steps;
}
long getClosePos(uint8_t idx) {
  (void)idx;
  return 0;
}

unsigned long lastProgressSent = 0;
unsigned long lastRemTimeSent = 0;
unsigned long lastLidarRxMs = 0;
static bool speedCalibrated = true;
static bool volumeCalibrated = true;

// Noise Filter
#define HEIGHT_DEADBAND_MM 25.0
#define MIN_HITS_PER_SWEEP 8
#define MIN_SLICE_AREA_CM2 6.0
#define EMA_ALPHA 0.25f
#define EMPTY_HOLD_MS 3000

float sliceAreaEma = 0;
unsigned long emptySince = 0;
volatile int dbg_hits = 0;
volatile float dbg_area_raw = 0;
volatile float dbg_area_ema = 0;
int hits = 0;

float backgroundDist[360];
float currentScan[360];
bool hasBackground = false;
float mountingHeight = 0;
int rightWallAngle = -1;
int leftWallAngle = -1;
float conveyorSpeed = 0.05;  // cm/s
float totalVolume = 0;
unsigned long lastScanTime = 0;
unsigned long lastVolumePrintTime = 0;

bool objMeasurementStarted = false;
unsigned long objStartTime = 0;
unsigned long lastObjectSeenTime = 0;

long last_remaining = -1;

// --- STATE MACHINE & SYSTEM VARS ---
bool autoModeEnabled = false;
SystemState currentState = IDLE;

static inline bool isDumpingState(SystemState s) {
  return (s == DUMPING_IN || s == DUMPING_OUT);
}

int lastState = -1;

// Flag Manual Tambahan
bool isManualFeederOn = false;
bool isManualWeighingOn = false;

// Motor Busy Flags
bool motorDoorBusy = false;
bool motorPushBusy = false;
bool motorAshBusy = false;
bool motorBurnerBusy = false;
bool motorBurnDoorBusy = false;
bool feederMotorOn = false;
bool weighingMotorOn = false;

// Motor Positions Constants (FIXED: mapping index ke motorConfigs yang benar)
const long POS_DOOR_OPEN = motorConfigs[0].steps;
const long POS_DOOR_CLOSE = 0;
const long POS_PUSH_IN = motorConfigs[1].steps;
const long POS_PUSH_OUT = 0;
const long POS_BURNER_IN = motorConfigs[2].steps;
const long POS_BURNER_OUT = 0;
const long POS_BDOOR_OPEN = motorConfigs[3].steps;
const long POS_BDOOR_CLOSE = 0;

// Burner & Logic
bool burnerActive = false;
bool isPriming = false;
bool burnTimerStarted = false;
bool preheat = false;
unsigned long burnerStartTime = 0;
unsigned long lastCountdownUpdate = 0;
unsigned long ignitiondelayStart = 0;
unsigned long now;
const unsigned long IGNITION_delay_MS = 20000;

float minBatchVolume = 100000.0;
double currentBatchVolume = 0;

// Metrics
float currentTempC = 0.0;
float currentWeightKg = 0.0;
unsigned long lastSensorMillis = 0;
const unsigned long SENSOR_INTERVAL = 500;

float minBatchWeight = 5.0;
uint32_t burnerActiveTimeSec = 15;
const unsigned long DUMP_DURATION_MS = 10000;
unsigned long actionStartTime = 0;

float solarVolumeUsed_L = 0.0;
float totalConsumedKg = 0.0;
unsigned long totalBatchCount = 0;
const float LITERS_PER_HOUR_BURNER = 10.0;

const float OVERHEAT_TEMP = 1400.0;
float scale_factor = 2123.157;
long scale_zero = 0;

// Manual Control Helper
bool isManualOpen = true;
long manualSteps = 1000;
long manualSpeed = 1000;
bool oneShotAutoMode = false;

// 7-Segment Helpers
float massValue = 0;
byte outX1, outX2, outY1, outY2;
float tempSetpoint = 1200.0;

const float CAL_KNOWN_WEIGHT_KG = 5.0f;

// ===== Fuel Level Sensor =====
unsigned long lastFuelSentMs = 0;
int FUEL_ADC_EMPTY = 200;
int FUEL_ADC_FULL = 850;

uint8_t fuelPct = 0;
float fuelLiterRemaining = 0;
const float TANK_CAPACITY_L = 20.0;

float fuelEma = 0;
const float FUEL_EMA_ALPHA = 0.15f;

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


// Mapping 7-Seg Arrays
byte segDigit14[10] = { 0b00101000, 0b01111110, 0b00110001, 0b00110100, 0b01100110, 0b10100100, 0b10100000, 0b00111110, 0b00100000, 0b00100100 };
byte segDigit2[10] = { 0b00001000, 0b01011110, 0b00010001, 0b00010100, 0b01000110, 0b10000100, 0b10000000, 0b00011110, 0b00000000, 0b00000100 };
byte segDigit3[10] = { 0b00101000, 0b11101011, 0b00110001, 0b10100001, 0b11100010, 0b10100100, 0b00100100, 0b11101001, 0b00100000, 0b10100000 };

// ================== HARDWARE OBJECTS ==================
HX711 scale;
AccelStepper stepperDoor(AccelStepper::DRIVER, STEP_PIN_DOOR, DIR_PIN_DOOR);
AccelStepper stepperPush(AccelStepper::DRIVER, STEP_PIN_PUSH, DIR_PIN_PUSH);
AccelStepper stepperAsh(AccelStepper::DRIVER, STEP_PIN_ASH, DIR_PIN_ASH);
AccelStepper stepperBurner(AccelStepper::DRIVER, STEP_PIN_BURN, DIR_PIN_BURN);
AccelStepper stepperBurnDoor(AccelStepper::DRIVER, STEP_PIN_BDOOR, DIR_PIN_BDOOR);
AccelStepper stepperMainConveyor(AccelStepper::DRIVER, STEP_PIN_MAINC, DIR_PIN_MAINC);

MAX31856* temperature;


static void updateWeightFilterNonBlocking() {
#if USE_HX711
  // hanya baca kalau data siap, jadi gak “nungguin” HX711
  if (!scale.is_ready()) return;

  // 1 sample saja (paling aman buat realtime)
  float w = scale.get_units(1);

  if (!isfinite(w)) return;

  // clamp noise minus kecil jadi 0 (biar gak kedip -0.1)
  if (w < 0.0f && w > -W_CLAMP_NEG_KG) w = 0.0f;

  // simpan ke buffer 3 sampel
  wbuf[wbi++] = w;
  if (wbi >= 3) {
    wbi = 0;
    wbufFull = true;
  }

  float wMed = wbufFull ? median3(wbuf[0], wbuf[1], wbuf[2]) : w;

  // init EMA
  if (!wEmaInit) {
    wEmaInit = true;
    wEma = wMed;
    wDisp = wMed;
    currentWeightKg = wMed;
    return;
  }

  // reject spike: kalau loncatan gede, abaikan sampel ini
  if (fabs(wMed - wEma) > W_JUMP_KG) {
    // tetap pakai nilai display lama
    currentWeightKg = wDisp;
    return;
  }

  // EMA smoothing
  wEma = (W_EMA_ALPHA * wMed) + ((1.0f - W_EMA_ALPHA) * wEma);

  // deadband untuk tampilan: biar stabil
  if (fabs(wEma - wDisp) >= W_DEADBAND_KG) {
    wDisp = wEma;
  }

  currentWeightKg = wDisp;
#endif
}


// ================== STEPPER HELPERS (FIXED) ==================

// Kalau ada driver yang arah kebalik, set di sini (sekali, bukan tiap handler)
const bool INV_DOOR_DIR = false;
const bool INV_PUSH_DIR = false;
const bool INV_ASH_DIR = false;
const bool INV_BURN_DIR = false;
const bool INV_BDOOR_DIR = true;  // kamu sebelumnya pakai setPinsInverted(false,true,false)
const bool INV_MAINC_DIR = false;

static inline void setSteppersEnabled(bool en) {
  // enable pin active LOW
  digitalWrite(ENABLE_PIN, en ? LOW : HIGH);
}

static inline bool anyStepperBusy() {
  // distanceToGo() valid untuk motor posisi; untuk conveyor kita pakai speed()
  if (stepperDoor.distanceToGo() != 0) return true;
  if (stepperPush.distanceToGo() != 0) return true;
  if (stepperAsh.distanceToGo() != 0) return true;
  if (stepperBurner.distanceToGo() != 0) return true;
  if (stepperBurnDoor.distanceToGo() != 0) return true;

  // conveyor: runSpeed dengan speed != 0 berarti jalan
  if (stepperMainConveyor.speed() != 0) return true;

  return false;
}

static inline void refreshEnablePin() {
  // Disable saat FAULT atau tidak ada motor jalan, biar driver adem
  if (currentState == FAULT) {
    setSteppersEnabled(false);
    return;
  }
  setSteppersEnabled(anyStepperBusy());
}

// ================== helper hx =====

bool calibrateScaleWithKnownWeight(float knownKg = CAL_KNOWN_WEIGHT_KG) {
#if !USE_HX711
  return false;
#else
  if (knownKg <= 0.0f) return false;

  long raw = scale.get_value(1);
  if (labs(raw) < 10000) {
    return false;
  }

  float newScale = (float)raw / knownKg;

  if (!isfinite(newScale) || newScale < 1.0f || newScale > 10000000.0f) {
    return false;
  }

  scale_factor = newScale;
  scale.set_scale(scale_factor);

  float checkKg = scale.get_units(10);
  if (fabs(checkKg - knownKg) > 0.5f) {
    return false;
  }

  return true;
#endif
}

void setAutoMode(bool en) {
  if (autoModeEnabled == en) return;
  autoModeEnabled = en;

  if (!en) {
    // auto off: matikan conveyor
    stepperMainConveyor.setSpeed(0);
  }
}

// ================== UTILITIES ==================
void emergencyStop() {
  burnerOff();
  burnerActive = false;
  burnTimerStarted = false;
  feederMotorOn = false;
  weighingMotorOn = false;

  setSteppersEnabled(false);

  stepperDoor.stop();
  stepperPush.stop();
  stepperAsh.stop();
  stepperBurner.stop();
  stepperBurnDoor.stop();

  stepperMainConveyor.setSpeed(0);
}

// --- 74HC595 Display ---
void sendByte(byte b) {
  digitalWrite(LATCH_PIN, LOW);
  shiftOut(DATA_PIN, CLOCK_PIN, MSBFIRST, b);
  digitalWrite(LATCH_PIN, HIGH);
}

void send4BytesTW(byte b1, byte b2, byte b3, byte b4) {
  digitalWrite(LATCH_PIN_TW, LOW);
  shiftOut(DATA_PIN_TW, CLOCK_PIN_TW, MSBFIRST, b1);
  shiftOut(DATA_PIN_TW, CLOCK_PIN_TW, MSBFIRST, b2);
  shiftOut(DATA_PIN_TW, CLOCK_PIN_TW, MSBFIRST, b3);
  shiftOut(DATA_PIN_TW, CLOCK_PIN_TW, MSBFIRST, b4);
  digitalWrite(LATCH_PIN_TW, HIGH);
}
void send4Bytes(byte b1, byte b2, byte b3, byte b4) {
  digitalWrite(LATCH_PIN, LOW);
  shiftOut(DATA_PIN, CLOCK_PIN, MSBFIRST, b1);
  shiftOut(DATA_PIN, CLOCK_PIN, MSBFIRST, b2);
  shiftOut(DATA_PIN, CLOCK_PIN, MSBFIRST, b3);
  shiftOut(DATA_PIN, CLOCK_PIN, MSBFIRST, b4);
  digitalWrite(LATCH_PIN, HIGH);
}

void pesanError() {
  byte tandaMin = 0b11110111;
  byte hurufE = 0b10100001;
  byte hurufR1 = 0b01111100;
  byte hurufR2 = 0b10101011;

  // Dulu: sendByte 4x (latch 4x)
  // Sekarang: latch sekali
  send4Bytes(~tandaMin, ~hurufE, ~hurufR1, ~hurufR2);
}

void sendByteTW(byte b) {
  digitalWrite(LATCH_PIN_TW, LOW);
  shiftOut(DATA_PIN_TW, CLOCK_PIN_TW, MSBFIRST, b);
  digitalWrite(LATCH_PIN_TW, HIGH);
}

void pesanErrorTW() {
  byte tandaMin = 0b11110111;
  byte hurufE = 0b10100001;
  byte hurufR1 = 0b01111100;
  byte hurufR2 = 0b10101011;

  send4BytesTW(~tandaMin, ~hurufE, ~hurufR1, ~hurufR2);
}

void convertTotalWeightTo7Segment(float valKg) {
  if (!isfinite(valKg) || valKg > 999.99f) {
    pesanErrorTW();
    return;
  }
  if (valKg < 0) valKg = 0;

  int nilaiInt = (int)valKg;                       // xxx
  int nilaiDec = (int)((valKg - nilaiInt) * 100);  // .yy

  int X1 = (nilaiInt / 10) % 10;  // puluhan
  int X2 = nilaiInt % 10;         // satuan
  int Y1 = (nilaiDec / 10) % 10;  // desimal 1
  int Y2 = nilaiDec % 10;         // desimal 2

  byte out1 = (nilaiInt < 10) ? 0b11111111 : segDigit14[X1];
  byte out2 = segDigit2[X2];
  byte out3 = segDigit3[Y1];
  byte out4 = segDigit14[Y2];

  send4BytesTW(~out1, ~out2, ~out3, ~out4);
}


void convertTemperatureValueto7Segment(float tempValue) {
  if (tempValue > 1500.0f) {
    pesanError();
    return;
  }
  if (tempValue < 0) tempValue = 0;

  int tempIntVal = (int)tempValue;
  int tX1 = (tempIntVal / 1000) % 10;
  int tX2 = (tempIntVal / 100) % 10;
  int tX3 = (tempIntVal / 10) % 10;
  int tX4 = tempIntVal % 10;

  const byte SEG_T_KOSONG = 0b11111111;

  byte tOut1 = (tX1 == 0) ? SEG_T_KOSONG : segDigit14[tX1];
  byte tOut2 = (tX1 == 0 && tX2 == 0) ? SEG_T_KOSONG : segDigit14[tX2];
  byte tOut3 = (tX1 == 0 && tX2 == 0 && tX3 == 0) ? SEG_T_KOSONG : segDigit3[tX3];
  byte tOut4 = segDigit14[tX4];

  send4Bytes(~tOut1, ~tOut2, ~tOut3, ~tOut4);
}
void convertMassValueto7Segment(float val) {
  if (val > 250.0f) {
    pesanError();
    return;
  }
  if (val < 0) val = 0;

  int nilaiInt = (int)val;
  int nilaiDec = (int)((val - nilaiInt) * 100);

  int X1 = (nilaiInt / 10) % 10;
  int X2 = nilaiInt % 10;
  int Y1 = (nilaiDec / 10) % 10;
  int Y2 = nilaiDec % 10;

  outX1 = (nilaiInt < 10) ? 0b11111111 : segDigit14[X1];
  outX2 = segDigit2[X2];
  outY1 = segDigit3[Y1];
  outY2 = segDigit14[Y2];

  send4Bytes(~outX1, ~outX2, ~outY1, ~outY2);
}
// ================== LOGIC BLOCKS ==================
void readSensors() {
  if (currentState == FAULT) return;

#if USE_MAX31856
float t = temperature->readThermocouple(CELSIUS); if (t < 5000.0f) currentTempC = t;
#else
  // Simulasi suhu
  if (currentState == BURNING) {
    if (currentTempC < tempSetpoint) currentTempC += 50;
  } else {
    if (currentTempC > 20) currentTempC -= 50;
  }
  if (currentTempC < 0) currentTempC = 0;
#endif

#if USE_HX711
  updateWeightFilterNonBlocking();
#else
  if (currentState == WEIGHING) currentWeightKg += 1;
#endif
}

void tareScale() {
  if (currentState != FAULT) {
#if USE_HX711
    scale.tare(20);
    scale_zero = scale.get_units(1);
#endif
    currentWeightKg = 0;
    Serial1.println("CMD_LIDAR_RESET_BATCH");
  }
}

void checkIgnitionPriming() {
  if (!isPriming) return;

  if (millis() - ignitiondelayStart >= IGNITION_delay_MS) {
    blowOff();
    isPriming = false;

    // setelah priming selesai, masuk PREHEAT (burner tetap ON sampai setpoint)
    currentState = PREHEAT;
  } else {
    long remaining = (IGNITION_delay_MS - (millis() - ignitiondelayStart)) / 1000;
    if (remaining != last_remaining && now - lastProgressSent > 500) {
      last_remaining = remaining;
      Serial1.print("REM_TIME:");
      Serial1.println(remaining);
      lastRemTimeSent = now;
    }
  }
}


void checkFeederConveyor() {
  if (isManualFeederOn) return;

  static bool conveyorRunning = false;
#if USE_LIDAR
  if (millis() - lastLidarRxMs > 1000) {
    currentState = FAULT;
    stepperMainConveyor.setSpeed(0);
    conveyorRunning = false;
    return;
  }
#endif

  bool shouldRun =
    (autoModeEnabled) && (currentState != FAULT) && (!isDumpingState(currentState)) && (currentBatchVolume < minBatchVolume);

  if (shouldRun && !conveyorRunning) {
    stepperMainConveyor.setMaxSpeed(motorConfigs[5].speed);
    stepperMainConveyor.setSpeed(motorConfigs[5].speed);
    conveyorRunning = true;
  } else if (!shouldRun && conveyorRunning) {
    stepperMainConveyor.setSpeed(0);
    conveyorRunning = false;
  }
}

bool startBurningAfterClose = false;  // TRUE hanya saat pintu ditutup untuk MULAI burning

void startBurningHold() {
  burnerOff();

  blowOff();

  // reset timer burning untuk batch/hold baru
  burnAccumulatedMs = 0;
  burnPhaseStartMs = 0;
  lastCountdownUpdate = millis();

  currentState = BURNING;
  burnPhaseStartMs = millis();  // timer mulai tepat saat masuk BURNING

  Serial1.println("BURNING_START");
}

void serviceBurnCycle() {
  unsigned long t = millis();
  unsigned long durationMs = burnerActiveTimeSec * 1000UL;

  // ================= PREHEAT =================
  // burner ON sampai setpoint tercapai
  if (currentState == PREHEAT) {
    burnerOn();
    Serial1.println("PREHEAT");

    if (currentTempC >= tempSetpoint) {
      burnerOff();

      // setelah panas cukup: keluarkan burner, lalu tutup pintu, lalu baru masuk BURNING
      startBurningAfterClose = true;
      currentState = BURNER_OUT;
    }
    return;
  }

  // ================= BURNING (HOLD) =================
  if (currentState == BURNING) {
    burnerOff();

    // drop terlalu jauh -> reheat: buka pintu lalu proses pemanas lagi
    if (currentTempC < (tempSetpoint - REHEAT_DROP_C)) {
      if (burnPhaseStartMs != 0) {
        burnAccumulatedMs += (t - burnPhaseStartMs);
        burnPhaseStartMs = 0;
      }

      // reheat sequence
      startBurningAfterClose = false;
      currentState = OPENING_BURN_DOOR;
      return;
    }

    if (burnPhaseStartMs == 0) burnPhaseStartMs = t;

    unsigned long elapsed = burnAccumulatedMs + (t - burnPhaseStartMs);
    if (elapsed >= durationMs) {
      // burning selesai -> sekarang boleh keluarkan burner lagi (cycle selesai)
      burnAccumulatedMs = 0;
      burnPhaseStartMs = 0;

      startBurningAfterClose = false;  // penting: jangan start burning lagi setelah pintu nutup
      currentState = BURNER_OUT;
      return;
    }

    if (t - lastCountdownUpdate >= 1000) {
      lastCountdownUpdate = t;
      long remainingSeconds = (long)((durationMs - elapsed) / 1000UL);
      if (remainingSeconds < 0) remainingSeconds = 0;
      Serial1.print("REM_TIME:");
      Serial1.println(remainingSeconds);
      lastRemTimeSent = now;
    }
    return;
  }
}


void checkStepperFlags() {
  if (motorDoorBusy && stepperDoor.distanceToGo() == 0) motorDoorBusy = false;
  if (motorPushBusy && stepperPush.distanceToGo() == 0) motorPushBusy = false;
  if (motorAshBusy && stepperAsh.distanceToGo() == 0) motorAshBusy = false;
  if (motorBurnerBusy && stepperBurner.distanceToGo() == 0) motorBurnerBusy = false;
  if (motorBurnDoorBusy && stepperBurnDoor.distanceToGo() == 0) motorBurnDoorBusy = false;
}

// ================== EEPROM FUNCTIONS ==================
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

void loadSettings() {
  EEPROM.get(EEPROM_ADDR_WEIGHT_SET, minBatchWeight);
  EEPROM.get(EEPROM_ADDR_TIME_SET, burnerActiveTimeSec);
  EEPROM.get(EEPROM_ADDR_TEMP_SET, tempSetpoint);
  EEPROM.get(EEPROM_ADDR_SCALE, scale_factor);

  if (!isfinite(scale_factor) || scale_factor < 1.0f || scale_factor > 10000000.0f) {
    scale_factor = 397.0f;
  }

  for (int i = 0; i < 6; i++) {
    EEPROM.get(EEPROM_ADDR_MOTOR_CFG + (i * sizeof(MotorConfig)), motorConfigs[i]);
    if (motorConfigs[i].speed < 0 || motorConfigs[i].speed > 50000) {
      motorConfigs[i].speed = 1000;
      motorConfigs[i].accel = 200;
    }
  }

  applyMotorConfigsToSteppers();

  EEPROM.get(EEPROM_ADDR_LIDAR_SPEED_CMPS, conveyorSpeed);
  EEPROM.get(EEPROM_ADDR_LIDAR_VOL_SCALE, volumeScaleFactor);
  EEPROM.get(EEPROM_ADDR_VOL_BATCH_SET, minBatchVolume);
  if (!isfinite(minBatchVolume) || minBatchVolume <= 0.0f) {
    minBatchVolume = 20.0;
  }

  uint8_t flags = 0;
  EEPROM.get(EEPROM_ADDR_LIDAR_FLAGS, flags);
  speedCalibrated = (flags & 0x01);
  volumeCalibrated = (flags & 0x02);

  if (!isfinite(conveyorSpeed) || conveyorSpeed <= 0.0f || conveyorSpeed > 500.0f) {
    conveyorSpeed = 0.05f;
    speedCalibrated = false;
  }
  if (!isfinite(volumeScaleFactor) || volumeScaleFactor <= 0.0f || volumeScaleFactor > 1000.0f) {
    volumeScaleFactor = 1.0f;
    volumeCalibrated = false;
  }
}

void saveSettings() {
  EEPROM.put(EEPROM_ADDR_WEIGHT_SET, minBatchWeight);
  EEPROM.put(EEPROM_ADDR_TIME_SET, burnerActiveTimeSec);
  EEPROM.put(EEPROM_ADDR_TEMP_SET, tempSetpoint);
  for (int i = 0; i < 6; i++) {
    EEPROM.put(EEPROM_ADDR_MOTOR_CFG + (i * sizeof(MotorConfig)), motorConfigs[i]);
  }
  EEPROM.put(EEPROM_ADDR_SCALE, scale_factor);
  EEPROM.put(EEPROM_ADDR_LIDAR_SPEED_CMPS, conveyorSpeed);
  EEPROM.put(EEPROM_ADDR_LIDAR_VOL_SCALE, volumeScaleFactor);
  EEPROM.put(EEPROM_ADDR_VOL_BATCH_SET, minBatchVolume);

  uint8_t flags = 0;
  if (speedCalibrated) flags |= 0x01;
  if (volumeCalibrated) flags |= 0x02;
  EEPROM.put(EEPROM_ADDR_LIDAR_FLAGS, flags);

  Serial.println("SETTINGS SAVED");
}

void computeMetrics() {
  float literPerSec = LITERS_PER_HOUR_BURNER / 3600.0;
  float fuelPerBatch = literPerSec * burnerActiveTimeSec;
  solarVolumeUsed_L += fuelPerBatch;

  if (currentWeightKg > 0.1) {
    totalConsumedKg += currentWeightKg;
  }

  totalBatchCount++;

  Serial1.print("METRICS:");
  Serial1.print(solarVolumeUsed_L);
  Serial1.print(",");
  Serial1.print(totalConsumedKg);
  Serial1.print(",");
  Serial1.println(totalBatchCount);

  Serial1.print("BATCH_SUM:");
  Serial1.print(totalBatchCount);
  Serial1.print(",");
  Serial1.print(currentBatchVolume, 3);
  Serial1.print(",");
  Serial1.print(currentWeightKg, 1);
  Serial1.print(",");
  Serial1.print(solarVolumeUsed_L, 2);
  Serial1.print(",");
  Serial1.print(currentTempC, 0);
  Serial1.print(",");
  Serial1.print(burnerActiveTimeSec);
  Serial1.println(",OK");
}

// ================== STATE MACHINE HANDLERS ==================

void handleIdle() {
  // FIX: handler jangan pakai run() langsung. cukup set target.
  long closePos = getClosePos(0);
  // blowOff();
#if USE_STEPPER
  if (stepperDoor.targetPosition() != closePos) {
    stepperDoor.moveTo(closePos);
  }
#else
  stepperDoor.setCurrentPosition(closePos);
#endif
}

void handleWeighing() {
  blowOn();
  if (currentBatchVolume >= minBatchVolume) {
    currentState = OPENING_MAIN_DOOR;
  }
}

void handleOpeningMainDoor() {
  // Serial1.println("LOG:INFO,DOOR,Opening Main Door");
  long targetPos = getOpenPos(0);
  blowOff();

#if !USE_STEPPER
  if (stepperDoor.targetPosition() != targetPos) {
    stepperDoor.moveTo(targetPos);
  }
  if (stepperDoor.distanceToGo() == 0 && stepperDoor.currentPosition() == targetPos) {
    currentState = DUMPING_IN;
  }
#else
  stepperDoor.setCurrentPosition(targetPos);
  currentState = DUMPING_IN;
#endif
}

void handleDumpingIn() {
  long targetPos = getOpenPos(1);

#if !USE_STEPPER
  if (stepperPush.targetPosition() != targetPos) {
    stepperPush.moveTo(targetPos);
  }
  if (stepperPush.distanceToGo() == 0 && stepperPush.currentPosition() == targetPos) {
    actionStartTime = millis();
    computeMetrics();
    currentState = DUMPING_OUT;
    currentBatchVolume = 0;
  }
#else
  stepperPush.setCurrentPosition(targetPos);
  actionStartTime = millis();
  computeMetrics();
  currentState = DUMPING_OUT;
#endif
}

void handleDumpingOut() {
  long targetPos = getClosePos(1);

#if !USE_STEPPER
  if (stepperPush.targetPosition() != targetPos) {
    stepperPush.moveTo(targetPos);
  }
  if (stepperPush.distanceToGo() == 0 && stepperPush.currentPosition() == targetPos) {
    currentState = CLOSING_MAIN_DOOR;
  }
#else
  stepperPush.setCurrentPosition(targetPos);
  currentState = CLOSING_MAIN_DOOR;
#endif
}

void handleClosingMainDoor() {
  long targetPos = getClosePos(0);

#if !USE_STEPPER
  if (stepperDoor.targetPosition() != targetPos) {
    stepperDoor.moveTo(targetPos);
  }
  if (stepperDoor.distanceToGo() == 0 && stepperDoor.currentPosition() == targetPos) {
    tareScale();

    // ===== SKIP PEMANAS JIKA SUHU MASIH DI ATAS/SETPPOINT =====
    if (currentTempC >= tempSetpoint) {
      // pastikan semua pemanas mati
      burnerOff();
      blowOff();

      // pastikan burner & burn door tertutup (biar aman kalau posisi nyangkut)
      stepperBurner.moveTo(getClosePos(2));
      stepperBurnDoor.moveTo(getClosePos(3));

      startBurningAfterClose = true;
      currentState = CLOSING_BURN_DOOR;  // kita paksa state ini untuk "menutup & start burning"
    } else {
      currentState = OPENING_BURN_DOOR;
    }
  }
#else
  stepperDoor.setCurrentPosition(targetPos);
  tareScale();

  if (currentTempC >= tempSetpoint) {
    burnerOff();
    blowOff();
    stepperBurner.setCurrentPosition(getClosePos(2));
    stepperBurnDoor.setCurrentPosition(getClosePos(3));
    startBurningAfterClose = true;
    currentState = CLOSING_BURN_DOOR;
  } else {
    currentState = OPENING_BURN_DOOR;
  }
#endif
  blowOn();
}


void handleOpeningBurnDoor() {
  long targetPos = getOpenPos(3);

#if USE_STEPPER
  if (stepperBurnDoor.targetPosition() != targetPos) {
    stepperBurnDoor.moveTo(targetPos);
  }
  if (stepperBurnDoor.distanceToGo() == 0 && stepperBurnDoor.currentPosition() == targetPos) {
    currentState = BURNER_IN;
  }
#else
  stepperBurnDoor.setCurrentPosition(targetPos);
  currentState = BURNER_IN;
#endif
}

void handleBurnerIn() {
  long targetPos = getOpenPos(2);

#if USE_STEPPER
  if (stepperBurner.targetPosition() != targetPos) {
    stepperBurner.moveTo(targetPos);
  }
  if (stepperBurner.distanceToGo() == 0 && stepperBurner.currentPosition() == targetPos) {
    currentState = IGNITING;
  }
#else
  stepperBurner.setCurrentPosition(targetPos);
  currentState = IGNITING;
#endif
}

void handleIgniting() {
  burnerOn();
  if (!isPriming) {



    isPriming = true;
    ignitiondelayStart = millis();

    burnAccumulatedMs = 0;
    burnPhaseStartMs = 0;

    Serial1.println("IGNITING");
  }
}

void handleBurning() {
  // dikontrol oleh serviceBurnCycle()
}

void handlePreheat() {
  // dikontrol di serviceBurnCycle(), handler cukup kosong
}


void handleBurnerOut() {
  burnerOff();
  long targetPos = getClosePos(2);

#if USE_STEPPER
  if (stepperBurner.targetPosition() != targetPos) {
    stepperBurner.moveTo(targetPos);
  }
  if (stepperBurner.distanceToGo() == 0 && stepperBurner.currentPosition() == targetPos) {
    currentState = CLOSING_BURN_DOOR;
  }
#else
  stepperBurner.setCurrentPosition(targetPos);
  currentState = CLOSING_BURN_DOOR;
#endif
}


void handleClosingBurnDoor() {
  long targetPos = getClosePos(3);

#if USE_STEPPER
  if (stepperBurnDoor.targetPosition() != targetPos) {
    stepperBurnDoor.moveTo(targetPos);
  }

  // kalau kamu skip pemanas, pastikan burner juga sudah OUT sebelum mulai burning
  bool doorClosed = (stepperBurnDoor.distanceToGo() == 0 && stepperBurnDoor.currentPosition() == targetPos);
  bool burnerOutOK = (stepperBurner.distanceToGo() == 0 && stepperBurner.currentPosition() == getClosePos(2));

  if (doorClosed) {
    if (startBurningAfterClose) {
      if (burnerOutOK) {
        startBurningAfterClose = false;
        startBurningHold();  // << timer burning mulai DI SINI
      }
    } else {
      // penutupan pintu setelah burning selesai: balik normal
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
#else
  stepperBurnDoor.setCurrentPosition(targetPos);

  if (startBurningAfterClose) {
    startBurningAfterClose = false;
    startBurningHold();
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
#endif
}


void handleFault() {
  emergencyStop();
  autoModeEnabled = false;
  currentState = IDLE;
}

void runStateMachine() {
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
    case BURNING: handleBurning(); break;
    case BURNER_OUT: handleBurnerOut(); break;
    case CLOSING_BURN_DOOR: handleClosingBurnDoor(); break;
    case FAULT: handleFault(); break;
    case PREHEAT: handlePreheat(); break;
  }
}

// ================== HELPER GERAK ==================
void runAllSteppers() {
  if (currentState == FAULT) {
    refreshEnablePin();
    return;
  }

#if USE_STEPPER
  stepperDoor.run();
  stepperPush.run();
  stepperAsh.run();
  stepperBurner.run();
  stepperBurnDoor.run();

  if (isManualFeederOn) {
    stepperMainConveyor.setSpeed(motorConfigs[5].speed);
    stepperMainConveyor.runSpeed();
  } else {
    stepperMainConveyor.runSpeed();
  }
#else
  // mode simulasi: tidak ada run
#endif

  // update enable pin terakhir, supaya akurat setelah run()
  refreshEnablePin();
}

// ================== SERIAL PROTOCOL ==================
void processIncomingSerial() {
  static char buf[64];
  static uint8_t idx = 0;

  while (Serial1.available()) {
    char c = Serial1.read();
    if (c == '\n') {
      buf[idx] = '\0';

      if (strncmp(buf, "CMD_", 4) == 0 || strncmp(buf, "SET_", 4) == 0 || strncmp(buf, "REQ_", 4) == 0 || strncmp(buf, "BLOW", 4) == 0 || strncmp(buf, "M_", 2) == 0 || strncmp(buf, "LIDAR", 5) == 0) {
        handleCommand(buf);
      } else {
        Serial.print(F("dbg: "));
        Serial.println(buf);
      }

      idx = 0;
    } else if (idx < sizeof(buf) - 1) {
      buf[idx++] = c;
    }
  }
}

void handleCommand(char* cmd) {
  char* header = strtok(cmd, ":\r\n");
  if (header == NULL) return;
  char* valStr = strtok(NULL, ":\r\n");

  // ===== Terima volume dari ESP32 (LIDAR pindah) =====
  if (strcmp(header, "LIDAR") == 0) {
    lastLidarRxMs = millis();
    if (valStr) {
      char* comma = strchr(valStr, ',');
      float batchV = 0;
      float totalV = totalVolume;

      if (comma) {
        *comma = '\0';
        batchV = atof(valStr);
        totalV = atof(comma + 1);
      } else {
        batchV = atof(valStr);
      }

      if (isfinite(batchV) && batchV >= 0) currentBatchVolume = batchV;
      if (isfinite(totalV) && totalV >= 0) totalVolume = totalV;
    }
    return;
  }

  // --- KALIBRASI BERAT 5KG ---
  if (strcmp(header, "CMD_SET_SCALE") == 0) {
    Serial.println("Calibrating Weight 5kg...");
    performWeightCalibration();
    return;
  }

  // --- KALIBRASI TARE (NOL) ---
  if (strcmp(header, "CMD_TARE_SCALE") == 0) {
    tareScale();
    Serial.println("Scale Tared");
    return;
  }

  if (strcmp(header, "CMD_MAN_IGN") == 0) {
    if (valStr != NULL) {
      int v = atoi(valStr);
      digitalWrite(RELAY_IGNITION, v ? LOW : HIGH);
      burnerActive = (v != 0);
      Serial1.println("CMD_MAN_IGN");
    }
    return;
  }

  if (strcmp(header, "CMD_MAN_BURN") == 0) {
    if (valStr != NULL) {
      int v = atoi(valStr);
      digitalWrite(RELAY_BURNER, v ? LOW : HIGH);
      burnerActive = (v != 0);
      Serial.print("MANBURN");
    }
    return;
  }

  if (strcmp(header, "CMD_MAN_FEED") == 0) {
    if (valStr != NULL) {
      int v = atoi(valStr);
      isManualFeederOn = (v != 0);
      if (!isManualFeederOn) {
        stepperMainConveyor.setSpeed(0);
      }
    }
    return;
  }

  if (strcmp(header, "CMD_MAN_WEIGH") == 0) {
    if (valStr != NULL) {
      int v = atoi(valStr);
      isManualWeighingOn = (v != 0);
      weighingMotorOn = (v != 0);
    }
    return;
  }

  if (strcmp(header, "CMD_SET_MANUAL_DIR") == 0) {
    if (valStr != NULL) {
      int v = atoi(valStr);
      isManualOpen = (v != 0);
    }
    return;
  }

  if (strcmp(header, "SET_M_SPD") == 0) {
    if (valStr != NULL) {
      manualSpeed = atol(valStr);
      Serial.println("Manual Speed Updated");
    }
    return;
  }

  if (strcmp(header, "SET_M_STP") == 0) {
    if (valStr != NULL) {
      manualSteps = atol(valStr);
      Serial.println("Manual Steps Updated");
    }
    return;
  }

  if (strcmp(header, "CMD_REQ_SYNC") == 0) {
    Serial1.print("W_SET:");
    Serial1.println(minBatchWeight);
    Serial1.print("TE_SET:");
    Serial1.println(tempSetpoint);
    Serial1.print("TI_SET:");
    Serial1.println(burnerActiveTimeSec);
    Serial1.print("VB_SET:");
    Serial1.println(minBatchVolume);
    Serial1.print("LIDAR_CFG:");
    Serial1.print(conveyorSpeed, 6);
    Serial1.print(",");
    Serial1.print(volumeScaleFactor, 6);
    Serial1.print(",");
    uint8_t flags = 0;
    if (speedCalibrated) flags |= 0x01;
    if (volumeCalibrated) flags |= 0x02;
    Serial1.println(flags);

    for (int i = 0; i < 6; i++) {
      Serial1.print("M_SYNC:");
      Serial1.print(i);
      Serial1.print(":");
      Serial1.print(motorConfigs[i].speed);
      Serial1.print(":");
      Serial1.println(motorConfigs[i].steps);
    }
    return;
  }

  if (strcmp(header, "CMD_AUTO_START") == 0) {
    Serial.println("Auto Mode");
    setAutoMode(true);
    currentState = (currentState == IDLE) ? WEIGHING : currentState;
    return;
  }

  if (strcmp(header, "CMD_AUTO_STOP") == 0) {
    setAutoMode(true);
    autoModeEnabled = true;
    currentState = (currentState == IDLE) ? WEIGHING : currentState;

    return;
  }

  if (strcmp(header, "CMD_ONE_SHOT_AUTO") == 0) {
    Serial.println("CMD_ONE_SHOT_AUTO");
    oneShotAutoMode = true;
    if (currentState == IDLE) currentState = OPENING_MAIN_DOOR;
    return;
  }

  if (strcmp(header, "CMD_EMERGENCY_STOP") == 0) {
    Serial.println("CMD_EMERGENCY_STOP");
    emergencyStop();
    currentState = FAULT;
    return;
  }

  if (strcmp(header, "CMD_DISCARD_SETTINGS") == 0) {
    Serial.println("CMD_DISCARD_SETTINGS");
    return;
  }

  if (strcmp(header, "CMD_SAVE_SETTINGS") == 0) {
    Serial.println("CMD_SAVE_SETTINGS");
    saveSettings();
    return;
  }

  if (strcmp(header, "REQ_MOVE") == 0) {
    if (valStr != NULL) {
      int id = atoi(valStr);
      executeManualMove(id);
      Serial.println("REQ_MOVE");
    }
    return;
  }

  if (strcmp(header, "M_SPD") == 0) {
    char* spdStr = strtok(NULL, ":\r\n");
    if (valStr != NULL && spdStr != NULL) {
      int id = atoi(valStr);
      long val = atol(spdStr);
      if (id >= 0 && id < 6) {
        motorConfigs[id].speed = val;
        applyMotorConfigsToSteppers();
        saveSettings();
      }
    }
    return;
  }

  if (strcmp(header, "M_STP") == 0) {
    char* stpStr = strtok(NULL, ":\r\n");
    if (valStr != NULL && stpStr != NULL) {
      int id = atoi(valStr);
      long val = atol(stpStr);
      if (id >= 0 && id < 6) {
        motorConfigs[id].steps = val;
        saveSettings();
      }
    }
    return;
  }

  if (valStr == NULL) return;

  if (strcmp(header, "SET_VB") == 0) {
    minBatchVolume = atof(valStr);
    Serial.println("CMD_SAVE_SETTINGS");
    saveSettings();
  }

  if (strcmp(header, "SET_W") == 0) {
    minBatchWeight = atof(valStr);
    Serial.println(minBatchWeight);
  } else if (strcmp(header, "SET_TE") == 0) {
    tempSetpoint = atof(valStr);
    Serial.println(tempSetpoint);
  } else if (strcmp(header, "SET_TI") == 0) {
    burnerActiveTimeSec = (uint32_t)atol(valStr);
    Serial.println(burnerActiveTimeSec);
  }
}

// Helper untuk Manual Move dari ID
void executeManualMove(int id) {
  switch (id) {
    case 0: runManualStepper(stepperDoor); break;
    case 1: runManualStepper(stepperPush); break;
    case 2: runManualStepper(stepperBurner); break;
    case 3: runManualStepper(stepperBurnDoor); break;
    case 4: runManualStepper(stepperAsh); break;
    case 5: runManualStepper(stepperMainConveyor); break;
  }
}

void runManualStepper(AccelStepper& motor) {
  if (currentState != IDLE) return;

  motor.setMaxSpeed(manualSpeed);
  motor.setAcceleration(2000);

  // move() itu RELATIF. biar gak nyangkut karena targetPosition lama,
  // aku set moveTo(current + delta) eksplisit.
  long delta = isManualOpen ? manualSteps : -manualSteps;
  long tgt = motor.currentPosition() + delta;
  motor.moveTo(tgt);

  Serial.print("MOVING MOT TO: ");
  Serial.println(tgt);
}

void performWeightCalibration() {
#if USE_HX711
  delay(5000);
  long rawValue = scale.get_value(100);
  float newScaleFactor = (float)rawValue / CAL_KNOWN_WEIGHT_KG;
  if (newScaleFactor == 0) newScaleFactor = 1.0;
  scale_factor = newScaleFactor;
  scale.set_scale(scale_factor);
  saveSettings();
  Serial1.print("MSG:CAL_WEIGHT_OK, Factor:");
  Serial1.println(scale_factor);
#else
  Serial1.println("MSG:HX711_DISABLED");
#endif
}

void checkfuelPercent() {
  if (now - lastFuelSentMs > 500) {
    fuelPct = readFuelPercent();
    fuelLiterRemaining = (fuelPct / 100.0f) * TANK_CAPACITY_L;

    Serial1.print("FUEL:");
    Serial1.print((int)fuelPct);
    Serial1.print(",");
    Serial1.println(fuelLiterRemaining, 1);

    lastFuelSentMs = now;
  }
}

void printResetCause() {
  uint8_t f = MCUSR;
  MCUSR = 0;
  Serial.print("MCUSR=");
  Serial.println(f, BIN);
}

// ================== SETUP & LOOP ==================
void setup() {
  Serial.begin(9600);
  Serial1.begin(9600, SERIAL_8N1);

  pinMode(RELAY_BURNER, OUTPUT);
  pinMode(RELAY_IGNITION, OUTPUT);
  pinMode(RELAY_BLOWER, OUTPUT);

  blowOff();
  ignOff();
  burnerOff();

  pinMode(ENABLE_PIN, OUTPUT);
  setSteppersEnabled(false);

  // Display 1
  pinMode(DATA_PIN, OUTPUT);
  pinMode(CLOCK_PIN, OUTPUT);
  pinMode(LATCH_PIN, OUTPUT);

  // Display 2 (Total Weight)
  pinMode(DATA_PIN_TW, OUTPUT);
  pinMode(CLOCK_PIN_TW, OUTPUT);
  pinMode(LATCH_PIN_TW, OUTPUT);


  printResetCause();

  // Invert arah sekali saja
  stepperDoor.setPinsInverted(true, false, false);
  stepperBurner.setPinsInverted(true, false, true);
  stepperMainConveyor.setPinsInverted(true, false, true);


  // loadSettings();  // kalau kamu mau pakai EEPROM beneran, aktifkan
  applyMotorConfigsToSteppers();

  if (isnan(minBatchWeight) || burnerActiveTimeSec == 0 || burnerActiveTimeSec > 86400) {
    minBatchWeight = 5.0;
    burnerActiveTimeSec = 30;
    tempSetpoint = 300;
    saveSettings();
  }

#if USE_MAX31856
  SPI.begin();
  temperature = new MAX31856(MAX31856_SDI, MAX31856_SDO, MAX31856_CS, MAX31856_SCK);
  temperature->writeRegister(REGISTER_CR0, CR0_INIT);
#else
  currentTempC = 8.0;
#endif

#if USE_HX711
  scale.begin(HX711_DT, HX711_SCK);
  scale.set_scale(scale_factor);
  tareScale();
#endif

  Serial.println("reset");
}

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

    case OPENING_MAIN_DOOR:
      {
        long pos = stepperDoor.currentPosition();
        if (pos < POS_DOOR_CLOSE) pos = POS_DOOR_CLOSE;
        if (pos > POS_DOOR_OPEN) pos = POS_DOOR_OPEN;
        val = (uint8_t)((pos - POS_DOOR_CLOSE) * 100L / (POS_DOOR_OPEN - POS_DOOR_CLOSE));
        break;
      }

    case CLOSING_MAIN_DOOR:
      {
        long pos = stepperDoor.currentPosition();
        if (pos < POS_DOOR_CLOSE) pos = POS_DOOR_CLOSE;
        if (pos > POS_DOOR_OPEN) pos = POS_DOOR_OPEN;
        val = (uint8_t)((POS_DOOR_OPEN - pos) * 100L / (POS_DOOR_OPEN - POS_DOOR_CLOSE));
        break;
      }

    case DUMPING_IN:
      {
        long pos = stepperPush.currentPosition();
        if (pos < POS_PUSH_OUT) pos = POS_PUSH_OUT;
        if (pos > POS_PUSH_IN) pos = POS_PUSH_IN;
        val = (uint8_t)((pos - POS_PUSH_OUT) * 100L / (POS_PUSH_IN - POS_PUSH_OUT));
        break;
      }

    case DUMPING_OUT:
      {
        long pos = stepperPush.currentPosition();
        if (pos < POS_PUSH_OUT) pos = POS_PUSH_OUT;
        if (pos > POS_PUSH_IN) pos = POS_PUSH_IN;
        val = (uint8_t)((POS_PUSH_IN - pos) * 100L / (POS_PUSH_IN - POS_PUSH_OUT));
        break;
      }

    case OPENING_BURN_DOOR:
      {
        long pos = stepperBurnDoor.currentPosition();
        if (pos < POS_BDOOR_CLOSE) pos = POS_BDOOR_CLOSE;
        if (pos > POS_BDOOR_OPEN) pos = POS_BDOOR_OPEN;
        val = (uint8_t)((pos - POS_BDOOR_CLOSE) * 100L / (POS_BDOOR_OPEN - POS_BDOOR_CLOSE));
        break;
      }

    case CLOSING_BURN_DOOR:
      {
        long pos = stepperBurnDoor.currentPosition();
        if (pos < POS_BDOOR_CLOSE) pos = POS_BDOOR_CLOSE;
        if (pos > POS_BDOOR_OPEN) pos = POS_BDOOR_OPEN;
        val = (uint8_t)((POS_BDOOR_OPEN - pos) * 100L / (POS_BDOOR_OPEN - POS_BDOOR_CLOSE));
        break;
      }

    case BURNER_IN:
      {
        long pos = stepperBurner.currentPosition();
        if (pos < POS_BURNER_OUT) pos = POS_BURNER_OUT;
        if (pos > POS_BURNER_IN) pos = POS_BURNER_IN;
        val = (uint8_t)((pos - POS_BURNER_OUT) * 100L / (POS_BURNER_IN - POS_BURNER_OUT));
        break;
      }

    case BURNER_OUT:
      {
        long pos = stepperBurner.currentPosition();
        if (pos < POS_BURNER_OUT) pos = POS_BURNER_OUT;
        if (pos > POS_BURNER_IN) pos = POS_BURNER_IN;
        val = (uint8_t)((POS_BURNER_IN - pos) * 100L / (POS_BURNER_IN - POS_BURNER_OUT));
        break;
      }

    case IGNITING:
      if (isPriming) {
        // burnerOn();
        unsigned long elapsed = millis() - ignitiondelayStart;
        if (elapsed > IGNITION_delay_MS) elapsed = IGNITION_delay_MS;
        val = (uint8_t)(elapsed * 100UL / IGNITION_delay_MS);
      } else {
        val = 0;
      }
      break;

    case BURNING:
      {
        unsigned long durationMS = burnerActiveTimeSec * 1000UL;
        unsigned long t = millis();
        unsigned long elapsed = burnAccumulatedMs + ((burnPhaseStartMs ? (t - burnPhaseStartMs) : 0));
        if (elapsed > durationMS) elapsed = durationMS;
        val = (uint8_t)(elapsed * 100UL / durationMS);
        break;
      }

    default:
      val = 0;
      break;
  }

  return val;
}

void lidarSimulation() {
  static unsigned long lastVisUpdate = 0;
  if (millis() - lastVisUpdate > 100) {
    lastVisUpdate = millis();
    for (int i = 0; i < 360; i++) {
      float simulatedDistance = 0.0;
      if (i >= 30 && i <= 60) {
        simulatedDistance = 100 + (rand() % 20);
      } else if (i >= 120 && i <= 150) {
        simulatedDistance = 200 + (rand() % 20);
      } else {
        simulatedDistance = 0;
      }
      currentScan[i] = simulatedDistance;
    }
  }

  if (currentState == WEIGHING && autoModeEnabled) {
    static unsigned long lastVolSimTime = 0;
    if (millis() - lastVolSimTime > 100) {
      lastVolSimTime = millis();

      float targetTimeSec = 10.0;
      float ticks = targetTimeSec * 10.0;
      float increment = minBatchVolume / ticks;
      float noise = (random(10, 50) / 1000.0f);
      double volInc = increment + noise;
      currentBatchVolume += volInc;
    }
  }
}

void loop() {
  now = millis();

  // 1. Hardware Control (Realtime)
  runAllSteppers();

  static uint32_t lastSlowTick = 0;
  if (now - lastSlowTick > 50) {
    lastSlowTick = now;
    checkIgnitionPriming();
    checkStepperFlags();
    serviceBurnCycle();
    checkFeederConveyor();
    checkfuelPercent();
    updateWeightFilterNonBlocking();
  }

  lidarSimulation();
  runStateMachine();
  processIncomingSerial();

  // 5. Update Berkala ke HMI
  if (now - lastSensorMillis >= SENSOR_INTERVAL) {
    lastSensorMillis = now;
    readSensors();

    if (currentState == WEIGHING) {
      convertMassValueto7Segment(currentWeightKg);
    } else {
      convertTemperatureValueto7Segment(currentTempC);
    }
    convertTotalWeightTo7Segment(totalConsumedKg);

    Serial1.print("DATA:");
    Serial1.print(currentTempC);
    Serial1.print(",");
    Serial1.print(currentWeightKg);
    Serial1.print(",");
    Serial1.print(currentState);
    Serial1.print(",");
    Serial1.print(currentBatchVolume);
    Serial1.print(",");
    Serial1.println(totalVolume);
  }

  if (now - lastProgressSent > 500) {
    int prog = calculateProgress();
    Serial1.print("PROG:");
    Serial1.println(prog);
    lastProgressSent = now;
  }
}
