/*
 * MCU-CTRL (Realtime Machine Controller) ARDUINO MEGA
 * -------------------------------------
 * Tugas: Mengontrol Hardware (Stepper, Sensor, Relay, Lidar)
 * Menjalankan State Machine & Logic Keselamatan
 * Komunikasi: Menerima perintah dari HMI, Mengirim status ke HMI
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
#include <RBDdimmer.h>
#include <math.h>
// #include <SoftwareSerial.h>
// HardwareSerial Serial(3); 
// ================== DEFINES & CONFIG ==================
#define USE_MAX31856    0    // 1 = Hardware, 0 = Simulasi
#define USE_HX711       0
#define USE_STEPPER     0    // 0 = Simulasi Gerak, 1 = Real Stepper
#define USE_LIDAR       0
#define USE_DIMMER      0

// Konfigurasi Fisik
#define DEG_TO_RAD 0.01745329251
#define MIN_HEIGHT_DIFF        15.0   // mm
#define WALL_HEIGHT_THRESHOLD  40.0   // mm
#define OBJ_TIMEOUT_MS         500
#define CAL_OBJ_LENGTH_CM      15.0
#define CAL_OBJ_TARGET_HEIGHT  50.0   // mm
#define CAL_HEIGHT_TOLERANCE   10.0   // mm
#define CAL_ANGLE_WINDOW       3
#define CAL_OBJ_VOLUME_CM3   (15.0 * 15.0 * 5.0) 

// EEPROM Addresses
#define EEPROM_ADDR_WEIGHT_SET  0
#define EEPROM_ADDR_TIME_SET    4
#define EEPROM_ADDR_TEMP_SET    8  
#define EEPROM_ADDR_SCALE       12
#define EEPROM_ADDR_MOTOR_CFG   20 
#define EEPROM_ADDR_BLOWER_LEVEL 92 
#define EEPROM_ADDR_LIDAR_SPEED_CMPS   100   // float (4 byte)
#define EEPROM_ADDR_LIDAR_VOL_SCALE    104   // float (4 byte)
#define EEPROM_ADDR_LIDAR_FLAGS        108   // uint8_t (1 byte)
#define EEPROM_ADDR_VOL_BATCH_SET       112

// MAX31856 Config
#define CR0_INIT  (CR0_AUTOMATIC_CONVERSION + CR0_OPEN_CIRCUIT_FAULT_TYPE_K + CR0_NOISE_FILTER_50HZ)
#define CR1_INIT  (CR1_AVERAGE_2_SAMPLES + CR1_THERMOCOUPLE_TYPE_S)
#define MASK_INIT (~(MASK_VOLTAGE_UNDER_OVER_FAULT + MASK_THERMOCOUPLE_OPEN_FAULT))

// ================== PIN CONFIGURATION ==================
// Sensor Pins
const int HX711_DT      = 38;
const int HX711_SCK     = 39;
const int MAX31856_CS   = 10;
const int MAX31856_SCK  = 13;
const int MAX31856_SDI  = 11;
const int MAX31856_SDO  = 12;



// 74HC595 (7-Segment)
#define DATA_PIN   30
#define CLOCK_PIN  31
#define LATCH_PIN  32

// Stepper Pins
#define STEP_PIN_DOOR 26
#define DIR_PIN_DOOR 27
#define STEP_PIN_PUSH 34
#define DIR_PIN_PUSH  35
#define STEP_PIN_ASH  36
#define DIR_PIN_ASH   37
#define STEP_PIN_MAINC 4
#define DIR_PIN_MAINC 5

const int STEP_PIN_BURN   = 28; 
const int DIR_PIN_BURN    = 29;
const int STEP_PIN_BDOOR  = 33; 
const int DIR_PIN_BDOOR   = 25;
const int ENABLE_PIN    = 99;

// Blower & Relay Pins
const int PIN_BLOWER_1 = 44;
const int PIN_BLOWER_2 = 45;
#define DIMMER_ZERO_CROSS_PIN 2

const int PIN_FUEL_ADC = A0;   // ganti sesuai pin analog kamu



const int RELAY_BURNER         = 9;
const int RELAY_IGNITION       = 98;

// ===== Relay helpers (asumsi ACTIVE LOW: LOW=ON, HIGH=OFF) =====
inline void burnerOn()   { digitalWrite(RELAY_BURNER, LOW);  }
inline void burnerOff()  { digitalWrite(RELAY_BURNER, HIGH); }
inline void ignOn()      { digitalWrite(RELAY_IGNITION, HIGH); }
inline void ignOff()     { digitalWrite(RELAY_IGNITION, LOW); }

const float REHEAT_DROP_C = 400.0f;

// Burn cycle timer (timer hanya dihitung saat burner OFF / state BURNING)
unsigned long burnAccumulatedMs = 0;   // total waktu BURNING (OFF) yang sudah terkumpul
unsigned long burnPhaseStartMs  = 0;   // start timestamp untuk segmen BURNING saat ini

// ================== GLOBAL VARIABLES (LOGIC) ==================
// ===== NON-BLOCKING BACKGROUND CAL =====
bool bgCalActive = false;
unsigned long bgCalStartMs = 0;

float bgSum[360];
uint16_t bgCnt[360];
// --- Motor Config Struct ---
struct MotorConfig {
  long speed;
  long accel;
  long steps;
};

// Default Configs (Akan ditimpa loadSettings)
MotorConfig motorConfigs[6] = {
  { 1000, 200, 2000 },  // 0: Door      (POS_DOOR_OPEN)
  { 1000, 200, 5000 },  // 1: Push      (POS_PUSH_IN)
  { 1000, 200, 3000 },  // 2: Burner    (POS_BURNER_IN)
  { 800, 200, 1000 },   // 3: BurnDoor  (POS_BDOOR_OPEN)
  { 800, 200, 1000 },   // 4: Ash       (sementara)
  { 800, 200, 1000 }    // 5: MAIN CONVEYOR       (sementara)
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

// --- LIDAR Variables ---
float volumeScaleFactor = 1.0;
double volumeAccumDuringCal = 0;
bool volumeCalibrated = false;
double calAreaTime_ms = 0; 

// Noise Filter
#define HEIGHT_DEADBAND_MM        25.0 
#define MIN_HITS_PER_SWEEP        8      
#define MIN_SLICE_AREA_CM2        6.0    
#define EMA_ALPHA                0.25f 
#define EMPTY_HOLD_MS            3000   

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
int leftWallAngle  = -1;
float conveyorSpeed = 0.05;   // cm/s
double totalVolume  = 0;      
unsigned long lastScanTime = 0;
unsigned long lastVolumePrintTime = 0;
bool speedCalibrated = false;
bool calibrateSpeedMode = false;
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

// Motor Positions Constants
const long POS_DOOR_OPEN      = 1200;
const long POS_DOOR_CLOSE     = 0;
const long POS_PUSH_IN        = 7100;
const long POS_PUSH_OUT       = 0; 
const long POS_BURNER_IN      = 3000;
const long POS_BURNER_OUT     = 0;    
const long POS_BDOOR_OPEN     = 1000; 
const long POS_BDOOR_CLOSE    = 0;    

// Burner & Logic
bool burnerActive = false;
bool isPriming    = false;
bool burnTimerStarted = false;   
unsigned long burnerStartTime     = 0;
unsigned long lastCountdownUpdate = 0;
unsigned long ignitiondelayStart = 0;
unsigned long now;
const unsigned long IGNITION_delay_MS = 10000;

float minBatchVolume = 20.0;    // Batas volume per batch (misal 20 Liter/cm3)
double currentBatchVolume = 0;

// Metrics
float currentTempC = 0.0;
float currentWeightKg = 0.0;
unsigned long lastSensorMillis = 0;
const unsigned long SENSOR_INTERVAL = 500; // Frekuensi update sensor & kirim ke HMI

float minBatchWeight = 5.0;
uint32_t burnerActiveTimeSec = 30;
const unsigned long DUMP_DURATION_MS = 10000;
unsigned long actionStartTime = 0;

float solarVolumeUsed_L = 0.0;
float totalConsumedKg = 0.0;
unsigned long totalBatchCount = 0;
const float LITERS_PER_HOUR_BURNER = 10.0;

const float OVERHEAT_TEMP = 1400.0;
float scale_factor = 397.0;
long scale_zero = 0;

// Manual Control Helper
bool isManualOpen = true; 
long manualSteps = 1000;  
long manualSpeed = 1000;  
bool oneShotAutoMode = false;

// Blower
int blowLevel[3] = {0, 0, 0}; 
int blowTarget[3]  = {0, 0, 0};   
int blowCurrent[3] = {0, 0, 0}; 
unsigned long lastBlowerFade = 0;
const unsigned long BLOWER_FADE_INTERVAL = 50; // Dipercepat

// 7-Segment Helpers
float massValue = 0;
byte outX1, outX2, outY1, outY2;
float tempSetpoint = 1200.0;

const float CAL_KNOWN_WEIGHT_KG = 5.0f;

// ===== Fuel Level Sensor =====
unsigned long lastFuelSentMs = 0;
// Kalibrasi ADC (isi setelah kamu ukur)
int FUEL_ADC_EMPTY = 200;      // contoh nilai analog saat tangki kosong
int FUEL_ADC_FULL  = 850;      // contoh nilai analog saat tangki full

uint8_t fuelPct = 0;           // 0-100 untuk HMI
float fuelLiterRemaining = 0;  // opsional
const float TANK_CAPACITY_L = 20.0; // opsional: kapasitas tangki (liter)

// smoothing biar tampilan stabil
float fuelEma = 0;
const float FUEL_EMA_ALPHA = 0.15f;

uint8_t readFuelPercent() {
  int raw = analogRead(PIN_FUEL_ADC);

  // EMA smoothing
  if (fuelEma <= 0) fuelEma = raw;
  fuelEma = (FUEL_EMA_ALPHA * raw) + ((1.0f - FUEL_EMA_ALPHA) * fuelEma);

  int v = (int)(fuelEma + 0.5f);

  // map ke 0..100 (pakai constrain)
  long pct = map(v, FUEL_ADC_EMPTY, FUEL_ADC_FULL, 0, 100);
  if (pct < 0) pct = 0;
  if (pct > 100) pct = 100;
  return (uint8_t)pct;
}


// Mapping 7-Seg Arrays (Disimpan di CTRL untuk hardware 74HC595)
byte segDigit14[10] = { 0b00101000, 0b01111110, 0b00110001, 0b00110100, 0b01100110, 0b10100100, 0b10100000, 0b00111110, 0b00100000, 0b00100100 };
byte segDigit2[10]  = { 0b00001000, 0b01011110, 0b00010001, 0b00010100, 0b01000110, 0b10000100, 0b10000000, 0b00011110, 0b00000000, 0b00000100 };
byte segDigit3[10]  = { 0b00101000, 0b11101011, 0b00110001, 0b10100001, 0b11100010, 0b10100100, 0b00100100, 0b11101001, 0b00100000, 0b10100000 };

// ================== HARDWARE OBJECTS ==================
HX711 scale;
AccelStepper stepperDoor(AccelStepper::DRIVER, STEP_PIN_DOOR, DIR_PIN_DOOR);
AccelStepper stepperPush(AccelStepper::DRIVER, STEP_PIN_PUSH, DIR_PIN_PUSH);
AccelStepper stepperAsh(AccelStepper::DRIVER, STEP_PIN_ASH, DIR_PIN_ASH);
AccelStepper stepperBurner(AccelStepper::DRIVER, STEP_PIN_BURN, DIR_PIN_BURN);
AccelStepper stepperBurnDoor(AccelStepper::DRIVER, STEP_PIN_BDOOR, DIR_PIN_BDOOR);
AccelStepper stepperMainConveyor(AccelStepper::DRIVER, STEP_PIN_MAINC, DIR_PIN_MAINC);

MAX31856 *temperature;
dimmerLamp blower1(PIN_BLOWER_1);    
dimmerLamp blower2(PIN_BLOWER_2);    


// ====helper hx =====

// Return true kalau sukses
bool calibrateScaleWithKnownWeight(float knownKg = CAL_KNOWN_WEIGHT_KG) {
#if !USE_HX711
  return false;
#else
  if (knownKg <= 0.0f) return false;

  long raw = scale.get_value(1); 

  if (labs(raw) < 10000) {          // threshold bisa kamu tuning
    return false;
  }

  // Rumus HX711 (Bogde): units = (read - offset) / scale
  // => scale = (read - offset) / units
  float newScale = (float)raw / knownKg;

  // Guard: hindari scale factor yang tidak masuk akal
  if (!isfinite(newScale) || newScale < 1.0f || newScale > 10000000.0f) {
    return false;
  }

  scale_factor = newScale;
  scale.set_scale(scale_factor);

  // Optional: verifikasi cepat harus ~5kg
  float checkKg = scale.get_units(10);
  if (fabs(checkKg - knownKg) > 0.5f) {   // toleransi 0.5kg (silakan ketatkan)
    // tetap bisa kamu anggap sukses, tapi aku bikin fail supaya ketahuan
    return false;
  }

  return true;
#endif
}

// ================== LIDAR FUNCTIONS ==================
float getHeightNearZeroDeg() {
  float maxH = 0;
  for (int i = 360 - CAL_ANGLE_WINDOW; i < 360; i++) {
    int idx = i % 360;
    if (backgroundDist[idx] > 0 && currentScan[idx] > 0) {
      float h = backgroundDist[idx] - currentScan[idx];
      if (h > maxH) maxH = h;
    }
  }
  for (int i = 0; i <= CAL_ANGLE_WINDOW; i++) {
    if (backgroundDist[i] > 0 && currentScan[i] > 0) {
      float h = backgroundDist[i] - currentScan[i];
      if (h > maxH) maxH = h;
    }
  }
  return maxH;
}

bool isValidCalibrationHeight(float h) {
  return (h >= CAL_OBJ_TARGET_HEIGHT - CAL_HEIGHT_TOLERANCE &&
          h <= CAL_OBJ_TARGET_HEIGHT + CAL_HEIGHT_TOLERANCE);
}

void setAutoMode(bool en) {
  if (autoModeEnabled == en) return;
  autoModeEnabled = en;

  if (en) {
    // saat auto start: blower langsung mengikuti level EEPROM
    for (int i = 0; i < 2; i++) {
      blowTarget[i]  = levelToPower(blowLevel[i]);
      blowCurrent[i] = blowTarget[i];   // langsung ON (kalau kamu mau fade, hapus baris ini)
    }
  } else {
    // auto off: matikan blower + conveyor
    for (int i = 0; i < 2; i++) {
      blowTarget[i] = 0;
    }
    stepperMainConveyor.setSpeed(0);
  }
}

void processOneSweep() {
  if (!hasBackground) return;
  unsigned long now = millis();
  unsigned long dt = now - lastScanTime;
  lastScanTime = now;
  if (dt > 200) dt = 100;

  // Hitung Luas Irisan + Noise Filter
  float sliceAreaCM2_raw = 0;
  int hitsLocal = 0;

  for (int i = 0; i < 360; i++) {
    if (rightWallAngle >= 0 && leftWallAngle >= 0) {
      bool inConveyor = (i <= rightWallAngle) || (i >= leftWallAngle);
      if (!inConveyor) continue;
    }
    if (backgroundDist[i] <= 0 || currentScan[i] <= 0) continue;
    float heightDiff = backgroundDist[i] - currentScan[i];
    if (heightDiff <= HEIGHT_DEADBAND_MM) continue;   
    if (currentScan[i] < 80) continue;               
    float arcWidth = currentScan[i] * DEG_TO_RAD;    
    float areaElement = heightDiff * arcWidth;       
    sliceAreaCM2_raw += areaElement;
    hitsLocal++;
  }

  sliceAreaCM2_raw /= 100.0f; // mm^2 -> cm^2
  if (hitsLocal < MIN_HITS_PER_SWEEP) sliceAreaCM2_raw = 0;

  // EMA smoothing
  sliceAreaEma = (EMA_ALPHA * sliceAreaCM2_raw) + ((1.0f - EMA_ALPHA) * sliceAreaEma);

  // Anti creep
  if (sliceAreaEma < (MIN_SLICE_AREA_CM2 * 0.3f)) {
    if (emptySince == 0) emptySince = millis();
    if (millis() - emptySince > EMPTY_HOLD_MS) sliceAreaEma = 0;
  } else {
    emptySince = 0;
  }
  float sliceAreaCM2 = sliceAreaEma;

  // Mode Kalibrasi Speed
  float h0 = getHeightNearZeroDeg();
  bool validCal = isValidCalibrationHeight(h0);

  if (calibrateSpeedMode) {
    if (validCal) {
      if (!objMeasurementStarted) {
        objMeasurementStarted = true;
        objStartTime = millis();
        lastObjectSeenTime = objStartTime;
        calAreaTime_ms = 0;
        sliceAreaEma = 0; emptySince = 0;
      }
      lastObjectSeenTime = millis();
      if (sliceAreaCM2_raw > MIN_SLICE_AREA_CM2) {
        calAreaTime_ms += (double)sliceAreaCM2_raw * (double)dt; 
      }
    } else {
      if (objMeasurementStarted && (millis() - lastObjectSeenTime > OBJ_TIMEOUT_MS)) {
        float dur_s = (lastObjectSeenTime - objStartTime) / 1000.0f;
        if (dur_s < 0.05f) dur_s = 0.05f;
        conveyorSpeed = CAL_OBJ_LENGTH_CM / dur_s; 
        speedCalibrated = true;
        double lidarVol = (double)conveyorSpeed * (calAreaTime_ms / 1000.0);
        if (lidarVol > 50.0) { 
          volumeScaleFactor = (float)(CAL_OBJ_VOLUME_CM3 / lidarVol);
          volumeCalibrated = true;
        } else {
          volumeScaleFactor = 1.0f; volumeCalibrated = false;
        }
        calibrateSpeedMode = false;
        objMeasurementStarted = false;
        saveSettings();
        Serial1.print("LIDAR_CAL_OK:");
        Serial1.print(conveyorSpeed, 3);
        Serial1.print(",");
        Serial1.println(volumeScaleFactor, 6);
      }
    }
    for (int i = 0; i < 360; i++) currentScan[i] = 0;
    return;
  }

// Volume Normal
  if (!speedCalibrated || !volumeCalibrated) {
    for (int i = 0; i < 360; i++) currentScan[i] = 0;
    return;
  }
  
  if (sliceAreaCM2 > MIN_SLICE_AREA_CM2) {
    double dV = (double)sliceAreaCM2 * (double)conveyorSpeed * ((double)dt / 1000.0);
    double volScaled = dV * (double)volumeScaleFactor;
    
    totalVolume += volScaled;         // Akumulasi Total (Statistik)
    currentBatchVolume += volScaled;  // Akumulasi Batch (Logic Trigger)
  }
  for (int i = 0; i < 360; i++) currentScan[i] = 0;
}

void startBackgroundCalibration() {
  memset(bgSum, 0, sizeof(bgSum));
  memset(bgCnt, 0, sizeof(bgCnt));

  bgCalActive = true;
  bgCalStartMs = millis();

  hasBackground = false;
  rightWallAngle = -1;
  leftWallAngle  = -1;

  for (int i = 0; i < 360; i++) currentScan[i] = 0;
}

void serviceBackgroundCalibration() {
  if (!bgCalActive) return;

  const uint16_t MAX_FRAMES_PER_CALL = 60;
  uint16_t frames = 0;

  while (Serial2.available() >= 5 && frames < MAX_FRAMES_PER_CALL) {
    frames++;

    int b0 = Serial2.read();
    bool s1 = (b0 >> 1) & 1;
    bool s0 = b0 & 1;

    if (s1 == !s0) {
      if (Serial2.available() < 4) break;

      byte b1 = Serial2.read();
      byte b2 = Serial2.read();
      byte b3 = Serial2.read();
      byte b4 = Serial2.read();

      float angle = ((b1 >> 1) | (b2 << 7)) / 64.0f;
      float dist  = (b3 | (b4 << 8)) / 4.0f;
      int idx = (int)angle;

      if (dist > 0 && idx >= 0 && idx < 360) {
        bgSum[idx] += dist;
        bgCnt[idx] += 1;
      }
    } else {
      // resync ringan
      if (Serial2.available() >= 4) { Serial2.read(); Serial2.read(); Serial2.read(); Serial2.read(); }
      else break;
    }
  }

  if (millis() - bgCalStartMs < 3000) return;

  for (int i = 0; i < 360; i++) {
    backgroundDist[i] = (bgCnt[i] > 0) ? (bgSum[i] / (float)bgCnt[i]) : 0.0f;
  }

 
  float hsum = 0; int hc = 0;
  int ref[] = {358,359,0,1,2};
  for (int i=0;i<5;i++) if (backgroundDist[ref[i]]>0) {hsum+=backgroundDist[ref[i]];hc++;}
  mountingHeight = hc ? hsum/hc : 0;
  hasBackground = true;

  // Auto detect walls
  rightWallAngle = -1; leftWallAngle = -1;
  for (int i = 0; i < 80; i++) {
    float y = backgroundDist[i] * cos(i * DEG_TO_RAD);
    if ((mountingHeight - y) > WALL_HEIGHT_THRESHOLD) { rightWallAngle = i; break; }
  }
  for (int i = 360; i > 280; i--) {
    int idx = (i==360)?0:i;
    float y = backgroundDist[idx] * cos((360-idx) * DEG_TO_RAD);
    if ((mountingHeight - y) > WALL_HEIGHT_THRESHOLD) { leftWallAngle = idx; break; }
  }
}

void resetLidar() {
  byte stopCmd[]  = {0xA5, 0x25};
  Serial2.write(stopCmd, 2);
  //delay(100);
  byte startCmd[] = {0xA5, 0x20};
  Serial2.write(startCmd, 2);
}

// ================== UTILITIES ==================
void emergencyStop() {
  burnerOff();
  burnerActive = false;
  burnTimerStarted = false;
  feederMotorOn = false;
  weighingMotorOn = false;
  
  digitalWrite(ENABLE_PIN, HIGH); // Disable stepper
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

void pesanError() {
  byte tandaMin = 0b11110111; 
  byte hurufE   = 0b10100001; 
  byte hurufR1  = 0b01111100; 
  byte hurufR2  = 0b10101011; 
  sendByte(~tandaMin); sendByte(~hurufE); sendByte(~hurufR1); sendByte(~hurufR2); 
}

void convertTemperatureValueto7Segment(float tempValue) {
  if (tempValue > 1500.0f) { pesanError(); return; }
  if (tempValue < 0) tempValue = 0;
  int tempIntVal = (int)tempValue;
  int tX1 = (tempIntVal / 1000) % 10;
  int tX2 = (tempIntVal / 100)  % 10;
  int tX3 = (tempIntVal / 10)   % 10;
  int tX4 = tempIntVal % 10;
  const byte SEG_T_KOSONG = 0b11111111;
  byte tOut1 = (tX1 == 0) ? SEG_T_KOSONG : segDigit14[tX1];
  byte tOut2 = (tX1 == 0 && tX2 == 0) ? SEG_T_KOSONG : segDigit14[tX2];
  byte tOut3 = (tX1 == 0 && tX2 == 0 && tX3 == 0) ? SEG_T_KOSONG : segDigit3[tX3];
  byte tOut4 = segDigit14[tX4];
  sendByte(~tOut1); sendByte(~tOut2); sendByte(~tOut3); sendByte(~tOut4);
}

void convertMassValueto7Segment(float val) {
  if (val > 250.0f) { pesanError(); return; }
  if (val < 0) val = 0;
  int nilaiInt = (int)val;
  int nilaiDec = (int)((val - nilaiInt) * 100);
  int X1 = (nilaiInt / 10) % 10;
  int X2 = nilaiInt % 10;
  int Y1 = (nilaiDec / 10) % 10;
  int Y2 =  nilaiDec % 10;
  outX1 = (nilaiInt < 10) ? 0b11111111 : segDigit14[X1];
  outX2 = segDigit2[X2];
  outY1 = segDigit3[Y1];
  outY2 = segDigit14[Y2];
  sendByte(~outX1); sendByte(~outX2); sendByte(~outY1); sendByte(~outY2);
}

// ================== LOGIC BLOCKS ==================
void readSensors() {
  if (currentState == FAULT) return;

#if USE_MAX31856
  currentTempC = temperature->readThermocouple(CELSIUS);
#else
  // Simulasi suhu
  if (currentState == BURNING) {
    if (currentTempC < tempSetpoint) currentTempC += 50;
    // Serial.println(currentTempC);
  } else {
    if (currentTempC > 20) currentTempC -= 50;
  }
  if (currentTempC < 0) currentTempC = 0;
#endif

#if USE_HX711
  if (scale.is_ready()) {currentWeightKg = scale.get_units(1);}
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
  currentBatchVolume = 0;
  currentWeightKg = 0;
  }
}

void checkIgnitionPriming() {
  if (!isPriming) return;

  if (millis() - ignitiondelayStart >= IGNITION_delay_MS) {
    ignOff();
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

  // Rule: auto mode ON, bukan fault, bukan dumping, dan feeder belum mencapai target volume
  bool shouldRun =
      (autoModeEnabled) &&
      (currentState != FAULT) &&
      (!isDumpingState(currentState)) &&
      (currentBatchVolume < minBatchVolume);

  if (shouldRun && !conveyorRunning) {
    digitalWrite(ENABLE_PIN, LOW);
    stepperMainConveyor.setMaxSpeed(motorConfigs[5].speed);
    stepperMainConveyor.setSpeed(motorConfigs[5].speed);
    conveyorRunning = true;
  } 
  else if (!shouldRun && conveyorRunning) {
    stepperMainConveyor.setSpeed(0);
    conveyorRunning = false;
  }
}

void startBurningHold() {
  burnerOff();                 // saat BURNING, burner wajib OFF
  currentState = BURNING;

  // mulai segmen timer BURNING (burner OFF)
  if (burnPhaseStartMs == 0) burnPhaseStartMs = millis();

  // opsional: reset countdown tick biar rapi
  lastCountdownUpdate = millis();

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
      burnPhaseStartMs  = 0;

      startBurningAfterClose = false; // penting: jangan start burning lagi setelah pintu nutup
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


int levelToPower(int level) {
  switch (level) {
    case 0: return 40;  // OFF/Idle power
    case 1: return 60;  
    case 2: return 80;  
    case 3: return 95;  
  }
  return 0;
}

void updateBlowerDimmer() {
  if (millis() - lastBlowerFade < BLOWER_FADE_INTERVAL) return;
  lastBlowerFade = millis();

  // Loop hanya untuk 2 Blower (Index 0 dan 1)
  for (int i = 0; i < 2; i++) { 
    // Logic Fade Smooth (Tetap sama)
    if (blowCurrent[i] < blowTarget[i]) {
      blowCurrent[i] += 5;
      if (blowCurrent[i] > blowTarget[i]) blowCurrent[i] = blowTarget[i];
    } 
    else if (blowCurrent[i] > blowTarget[i]) {
      blowCurrent[i] -= 5;
      if (blowCurrent[i] < blowTarget[i]) blowCurrent[i] = blowTarget[i];
    }
    
    blowCurrent[i] = constrain(blowCurrent[i], 0, 95);

    // Apply ke Hardware (Hapus case 2)
#if USE_DIMMER
    switch (i) {
      case 0: blower1.setPower(blowCurrent[i]); break; // Air Blower
      case 1: blower2.setPower(blowCurrent[i]); break; // Cooler Blower
    }
#endif
  }
}

void checkStepperFlags() {
  // Reset busy flags jika stepper sampai tujuan
  if (motorDoorBusy      && stepperDoor.distanceToGo()   == 0) motorDoorBusy       = false;
  if (motorPushBusy      && stepperPush.distanceToGo()   == 0) motorPushBusy       = false;
  if (motorAshBusy       && stepperAsh.distanceToGo()    == 0) motorAshBusy        = false;
  if (motorBurnerBusy    && stepperBurner.distanceToGo() == 0) motorBurnerBusy     = false;
  if (motorBurnDoorBusy  && stepperBurnDoor.distanceToGo()== 0) motorBurnDoorBusy = false;
}

// ================== EEPROM FUNCTIONS ==================
void applyMotorConfigsToSteppers() {
  stepperDoor.setMaxSpeed(motorConfigs[0].speed);
  stepperPush.setMaxSpeed(motorConfigs[1].speed);
  stepperBurner.setMaxSpeed(motorConfigs[2].speed);
  stepperBurnDoor.setMaxSpeed(motorConfigs[3].speed);
  stepperAsh.setMaxSpeed(motorConfigs[4].speed);
  stepperMainConveyor.setMaxSpeed(motorConfigs[5].speed);
}

void loadSettings() {
  EEPROM.get(EEPROM_ADDR_WEIGHT_SET, minBatchWeight);
  EEPROM.get(EEPROM_ADDR_TIME_SET, burnerActiveTimeSec);
  EEPROM.get(EEPROM_ADDR_TEMP_SET, tempSetpoint);
  EEPROM.get(EEPROM_ADDR_SCALE, scale_factor);
  
  if (!isfinite(scale_factor) || scale_factor < 1.0f || scale_factor > 10000000.0f) {
    scale_factor = 397.0f; // fallback default kamu
  }  
  for (int i = 0; i < 6; i++) {
    EEPROM.get(EEPROM_ADDR_MOTOR_CFG + (i * sizeof(MotorConfig)), motorConfigs[i]);
    // Sanity check
    if (motorConfigs[i].speed < 0 || motorConfigs[i].speed > 50000) {
      motorConfigs[i].speed = 1000;
      motorConfigs[i].accel = 200;
    }
  }
  for (int i = 0; i < 3; i++) {
    uint8_t lvl;
    EEPROM.get(EEPROM_ADDR_BLOWER_LEVEL + i, lvl);
    if (lvl > 3) lvl = 0;
    blowLevel[i]   = lvl;
    blowTarget[i]  = levelToPower(lvl);
    blowCurrent[i] = blowTarget[i];
  }
  applyMotorConfigsToSteppers();
  EEPROM.get(EEPROM_ADDR_LIDAR_SPEED_CMPS, conveyorSpeed);
  EEPROM.get(EEPROM_ADDR_LIDAR_VOL_SCALE, volumeScaleFactor);
  EEPROM.get(EEPROM_ADDR_VOL_BATCH_SET, minBatchVolume);
  if (!isfinite(minBatchVolume) || minBatchVolume <= 0.0f) {
    minBatchVolume = 20.0; // Default jika eeprom kosong
  }

  uint8_t flags = 0;
  EEPROM.get(EEPROM_ADDR_LIDAR_FLAGS, flags);
  speedCalibrated  = (flags & 0x01);
  volumeCalibrated = (flags & 0x02);

  // sanity check biar gak ke-load nilai sampah
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
  for (int i = 0; i < 3; i++) {
    uint8_t lvl = (uint8_t)constrain(blowLevel[i], 0, 3);
    EEPROM.put(EEPROM_ADDR_BLOWER_LEVEL + i, lvl);
  }
  EEPROM.put(EEPROM_ADDR_SCALE, scale_factor);
  EEPROM.put(EEPROM_ADDR_LIDAR_SPEED_CMPS, conveyorSpeed);
  EEPROM.put(EEPROM_ADDR_LIDAR_VOL_SCALE, volumeScaleFactor);
  EEPROM.put(EEPROM_ADDR_VOL_BATCH_SET, minBatchVolume);

  // simpan flags (bit0=speedCal, bit1=volCal)
  uint8_t flags = 0;
  if (speedCalibrated)  flags |= 0x01;
  if (volumeCalibrated) flags |= 0x02;
  EEPROM.put(EEPROM_ADDR_LIDAR_FLAGS, flags);
  Serial.println("SETTINGS SAVED");

}

void computeMetrics() {
  float literPerSec = LITERS_PER_HOUR_BURNER / 3600.0;
  float fuelPerBatch = literPerSec * burnerActiveTimeSec;
  solarVolumeUsed_L += fuelPerBatch; // Gunakan += agar terakumulasi (bukan = )

  totalVolume += currentBatchVolume;

  if (currentWeightKg > 0.1) {
    totalConsumedKg += currentWeightKg;
  }
  
  totalBatchCount++;

  Serial1.print("METRICS:");
  Serial1.print(solarVolumeUsed_L); Serial1.print(",");
  Serial1.print(totalVolume);       Serial1.print(",");
  Serial1.print(totalConsumedKg);   Serial1.print(",");
  Serial1.println(totalBatchCount);
  Serial1.print("BATCH_SUM:");
  Serial1.print(totalBatchCount);
  Serial1.print(",");
  Serial1.print(currentBatchVolume,1);
  Serial1.print(",");
  Serial1.print(currentWeightKg,1);
  Serial1.print(",");
  Serial1.print(solarVolumeUsed_L,2); // atau fuel per batch
  Serial1.print(",");
  Serial1.print(currentTempC,0); // atau peak temp kalau kamu track
  Serial1.print(",");
  Serial1.print(burnerActiveTimeSec);
  Serial1.println(",OK");
}

// ================== STATE MACHINE HANDLERS ==================

void handleIdle() {
  long closePos = getClosePos(0); // Index 0: Door
  ignOff();
  if (stepperDoor.currentPosition() != closePos) {
    digitalWrite(ENABLE_PIN, LOW);
    stepperDoor.moveTo(closePos); 
    stepperDoor.run(); // Pastikan ada run() jika ini dipanggil di loop tanpa runAllSteppers
  } else {
    digitalWrite(ENABLE_PIN, HIGH);
  }
}

void handleWeighing() {
  // Serial.println("handleWeighing");
  ignOn();
  if (currentBatchVolume >= minBatchVolume) {
    currentState = OPENING_MAIN_DOOR;

  }
}

void handleOpeningMainDoor() {
  Serial1.println("LOG:INFO,DOOR,Opening Main Door");
  long targetPos = getOpenPos(0); // Index 0: Door
  ignOff();
#if USE_STEPPER
  digitalWrite(ENABLE_PIN, LOW);
  stepperDoor.moveTo(targetPos);
  
  if (stepperDoor.distanceToGo() == 0 && stepperDoor.currentPosition() == targetPos) {
    currentState = DUMPING_IN;
  }
#else
  // SIMULASI: Langsung set posisi ke target (agar konsisten dengan settingan)
  stepperDoor.setCurrentPosition(targetPos);
  currentState = DUMPING_IN;
#endif
}

void handleDumpingIn() {
  long targetPos = getOpenPos(1); // Index 1: Push

#if USE_STEPPER
  stepperPush.moveTo(targetPos);
  
  if (stepperPush.distanceToGo() == 0 && stepperPush.currentPosition() == targetPos) {
    actionStartTime = millis();
    computeMetrics();
    currentState = DUMPING_OUT;
  }
#else
  stepperPush.setCurrentPosition(targetPos);
  actionStartTime = millis();
  computeMetrics();
  currentState = DUMPING_OUT;
#endif

}

void handleDumpingOut() {
  long targetPos = getClosePos(1); // Index 1: Push (Posisi Tutup/Balik)

#if USE_STEPPER
  stepperPush.moveTo(targetPos);
  
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

#if USE_STEPPER
  if (stepperDoor.targetPosition() != targetPos) {
    stepperDoor.moveTo(targetPos);
  }
  if (stepperDoor.distanceToGo() == 0 && stepperDoor.currentPosition() == targetPos) {
    tareScale();

    // ===== SKIP PEMANAS JIKA SUHU MASIH DI ATAS/SETPPOINT =====
    if (currentTempC >= tempSetpoint) {
      // pastikan semua pemanas mati
      burnerOff();
      ignOff();

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
    ignOff();
    stepperBurner.setCurrentPosition(getClosePos(2));
    stepperBurnDoor.setCurrentPosition(getClosePos(3));
    startBurningAfterClose = true;
    currentState = CLOSING_BURN_DOOR;
  } else {
    currentState = OPENING_BURN_DOOR;
  }
#endif
}



void handleOpeningBurnDoor() {
  long targetPos = getOpenPos(3);

#if USE_STEPPER
  stepperBurnDoor.setMaxSpeed(10000);
  stepperBurnDoor.setAcceleration(200);
  stepperBurnDoor.setPinsInverted(false, true, false);
  stepperBurnDoor.moveTo(targetPos);
  Serial.println(targetPos);
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
  stepperBurner.setAcceleration(200);
  stepperBurner.moveTo(targetPos);
  if (stepperBurner.distanceToGo() == 0 && stepperBurner.currentPosition() == targetPos) {
    currentState = IGNITING;
  }
#else
  stepperBurner.setCurrentPosition(targetPos);
  currentState = IGNITING;
#endif
}

void handleIgniting() {
  if (!isPriming) {
    burnerOn();     // PREHEAT butuh burner ON
    ignOn();        // spark ON

    isPriming = true;
    ignitiondelayStart = millis();

    // reset burn timer untuk siklus baru
    burnAccumulatedMs = 0;
    burnPhaseStartMs  = 0;

    Serial1.println("IGNITING");
  }
}

void handlePreheat() {
  // tunggu sampai suhu >= setpoint -> dipindah oleh kontrol burn cycle
}

void handleBurning() {

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
  bool doorClosed  = (stepperBurnDoor.distanceToGo() == 0 && stepperBurnDoor.currentPosition() == targetPos);
  bool burnerOutOK = (stepperBurner.distanceToGo() == 0 && stepperBurner.currentPosition() == getClosePos(2));

  if (doorClosed) {
    if (startBurningAfterClose) {
      if (burnerOutOK) {
        startBurningAfterClose = false;
        startBurningHold();   // << timer burning mulai DI SINI
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
}

void runStateMachine() {
  switch (currentState) {
    case IDLE:              handleIdle(); break;
    case WEIGHING:          handleWeighing(); break;
    case OPENING_MAIN_DOOR: handleOpeningMainDoor(); break;
    case DUMPING_IN:        handleDumpingIn(); break;
    case DUMPING_OUT:       handleDumpingOut(); break;
    case CLOSING_MAIN_DOOR: handleClosingMainDoor(); break;
    case OPENING_BURN_DOOR: handleOpeningBurnDoor(); break;
    case BURNER_IN:         handleBurnerIn(); break;
    case IGNITING:          handleIgniting(); break;
    case PREHEAT:           handlePreheat(); break;
    case BURNING:           handleBurning(); break;
    case BURNER_OUT:        handleBurnerOut(); break;
    case CLOSING_BURN_DOOR: handleClosingBurnDoor(); break;
    case FAULT:             handleFault(); break;
    case PREHEAT: handlePreheat(); break;

  }
}

// ================== HELPER GERAK ==================
void runAllSteppers() {
  if (currentState != FAULT) {
    stepperDoor.run();
    stepperPush.run();
    stepperAsh.run();
    stepperBurner.run();
    stepperBurnDoor.run();
    bool autoRun = (stepperMainConveyor.speed() != 0 && !isManualFeederOn);
    
    if (isManualFeederOn) {
      // Manual Mode: Set speed konstan (ambil dari config motor index 5)
      stepperMainConveyor.setSpeed(motorConfigs[5].speed);
      stepperMainConveyor.runSpeed();
    } 
    else {
      // Auto Mode atau Idle (speed diatur oleh checkFeederConveyor)
      stepperMainConveyor.runSpeed();
    }
  }
}

void processIncomingSerial() {
  static char buf[64];
  static uint8_t idx = 0;

  while (Serial1.available()) {
    char c = Serial1.read();
    if (c == '\n') {
      buf[idx] = '\0';

      // Cek apakah perintah valid
      if (strncmp(buf, "CMD_", 4) == 0 || 
          strncmp(buf, "SET_", 4) == 0 || 
          strncmp(buf, "REQ_", 4) == 0 || 
          strncmp(buf, "BLOW", 4) == 0 || 
          strncmp(buf, "M_", 2)   == 0) {
        handleCommand(buf);  // Proses perintah yang valid
        // Serial.println("found");
      } else {
        // Log atau abaikan data debug
        Serial.print(F("Pesan Debug Ditemukan: "));
        Serial.println(buf);
      }
      
      idx = 0;
    } else if (idx < sizeof(buf)-1) {
      buf[idx++] = c;
    }
  }
}

void handleCommand(char* cmd) {


  char* header = strtok(cmd, ":\r\n");
  if (header == NULL) return;
  char* valStr = strtok(NULL, ":\r\n");


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

  // --- KALIBRASI BACKGROUND LIDAR ---
  if (strcmp(header, "CMD_CAL_WALL") == 0) {
    startBackgroundCalibration(); 
    Serial.println("Calibrating Wall...");
    return;
  }

  // --- KALIBRASI VOLUME/SPEED (Start Scan Benda 15x15x5) ---
  if (strcmp(header, "CMD_CAL_SPEED") == 0) {
    calibrateSpeedMode = true; 
    objMeasurementStarted = false; 
    Serial.println("Waiting for object 15x15x5...");
    return;
  }

  if (strcmp(header, "CMD_MAN_IGN") == 0) {
    if (valStr != NULL) {
      int v = atoi(valStr);
      // Logic Relay: Active LOW (sesuai setup awal: HIGH = Mati)
      // Jika v=1 (ON) -> tulis LOW. Jika v=0 (OFF) -> tulis HIGH.
      digitalWrite(RELAY_IGNITION, v ? LOW : HIGH);
      
      // Jika Ignition ON, set burnerActive agar logic lain tau
      burnerActive = (v != 0); 
      Serial.println("CMD_MAN_IGN");
    }
    return;
  }
    if (strcmp(header, "CMD_MAN_BURN") == 0) {
    if (valStr != NULL) {
      int v = atoi(valStr);
      // Logic Relay: Active LOW (sesuai setup awal: HIGH = Mati)
      // Jika v=1 (ON) -> tulis LOW. Jika v=0 (OFF) -> tulis HIGH.
      digitalWrite(RELAY_BURNER, v ? LOW : HIGH);
      
      // Jika Ignition ON, set burnerActive agar logic lain tau
      burnerActive = (v != 0); 
    }
    return;
  }

  if (strcmp(header, "CMD_MAN_FEED") == 0) {
    if (valStr != NULL) {
      int v = atoi(valStr);
      isManualFeederOn = (v != 0);
      
      // Jika dimatikan, pastikan speed langsung 0
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
      weighingMotorOn = (v != 0); // Update flag global lama jika ada
      
      // [OPSIONAL] Jika ada PIN RELAY khusus untuk weighing, kontrol disini:
      // digitalWrite(RELAY_WEIGHING, v ? HIGH : LOW);
    }
    return;
  }

  if (strcmp(header, "CMD_SET_MANUAL_DIR") == 0) {
    if (valStr != NULL) {
      int v = atoi(valStr);
      // Jika v != 0 (artinya 1), maka true (Open/CW). Jika 0, maka false (Close/CCW)
      isManualOpen = (v != 0); 
      // Serial.println(isManualOpen ? "DIR: OPEN" : "DIR: CLOSE"); // Debug output
    }
    return;
  }
  // Set Manual Speed (Temporary)
  if (strcmp(header, "SET_M_SPD") == 0) {
    if (valStr != NULL) {
      manualSpeed = atol(valStr); 
      Serial.println("Manual Speed Updated"); 
    }
    return;
  }

  // Set Manual Steps/Distance (Temporary)
  if (strcmp(header, "SET_M_STP") == 0) {
    if (valStr != NULL) {
      manualSteps = atol(valStr);
      Serial.println("Manual Steps Updated");
    }
    return;
  }

  if (strcmp(header, "CMD_REQ_SYNC") == 0) {
    Serial.print("W_SET:"); Serial.println(minBatchWeight);
    Serial.print("TE_SET:"); Serial.println(tempSetpoint);
    Serial.print("TI_SET:"); Serial.println(burnerActiveTimeSec);
    Serial.print("VB_SET:"); Serial.println(minBatchVolume);


    for (int i = 0; i < 6; i++) {
      Serial.print("M_SYNC:"); Serial.print(i); Serial.print(":");
      Serial.print(motorConfigs[i].speed); Serial.print(":");
      Serial.println(motorConfigs[i].steps);
    }
    for (int i = 0; i < 3; i++) {
      Serial.print("B_SYNC:"); Serial.print(i); Serial.print(":");
      Serial.println(blowLevel[i]);
    }
    for (int i = 0; i < 3; i++) {
      Serial.print("B_SYNC:"); Serial.print(i); Serial.print(":");
      Serial.println(blowLevel[i]);
    }
    return;
  }


  if (strcmp(header, "CMD_AUTO_START") == 0) {
    Serial.println("Auto Mode"); 
    setAutoMode(true); 
    if (currentState==IDLE) currentState=WEIGHING; 
      return; 
  }
  if (strcmp(header, "CMD_AUTO_STOP") == 0)  {
    setAutoMode(false); 
    autoModeEnabled = false; return; }
  if (strcmp(header, "CMD_ONE_SHOT_AUTO") == 0) {Serial.println("CMD_ONE_SHOT_AUTO");  oneShotAutoMode = true; if (currentState==IDLE) currentState=OPENING_MAIN_DOOR; return; }
  if (strcmp(header, "CMD_EMERGENCY_STOP") == 0) {Serial.println("CMD_EMERGENCY_STOP");  emergencyStop(); currentState = FAULT; return; }
  if (strcmp(header, "CMD_TARE_SCALE") == 0) {Serial.println("CMD_TARE_SCALE");  tareScale(); return; }
  if (strcmp(header, "CMD_CAL_WALL") == 0) {Serial.println("CMD_CAL_WALL");  startBackgroundCalibration(); return; }
  if (strcmp(header, "CMD_CAL_SPEED") == 0) {Serial.println("CMD_CAL_SPEED");  calibrateSpeedMode = true; objMeasurementStarted = false; return; }
  if (strcmp(header, "CMD_DISCARD_SETTINGS") == 0) {Serial.println("CMD_DISCARD_SETTINGS");  loadSettings(); return; }
  if (strcmp(header, "CMD_SAVE_SETTINGS") == 0) {Serial.println("CMD_SAVE_SETTINGS");  saveSettings(); }




  if (strcmp(header, "REQ_MOVE") == 0) {
    if (valStr != NULL) {
      int id = atoi(valStr);
      executeManualMove(id);
      Serial.println("REQ_MOVE");
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

  if (strcmp(header, "BLOW") == 0) {
    char* lvlStr = strtok(NULL, ":\r\n");
    if (valStr != NULL && lvlStr != NULL) {
      int id = atoi(valStr);
      int lvl = atoi(lvlStr);
      
      // Ubah kondisi menjadi id < 2 (karena cuma ada index 0 dan 1)
      if (id >= 0 && id < 2) { 
        blowLevel[id] = lvl;
        blowTarget[id] = levelToPower(lvl);
        saveSettings();
      }
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
  }
  else if (strcmp(header, "SET_TE") == 0) {
    tempSetpoint = atof(valStr);
    Serial.println(tempSetpoint);
  }
  else if (strcmp(header, "SET_TI") == 0) {
    burnerActiveTimeSec = (uint32_t)atol(valStr);
    Serial.println(burnerActiveTimeSec);
  }  


}


// Helper untuk Manual Move dari ID
void executeManualMove(int id) {
   switch(id) {
     case 0: runManualStepper(stepperDoor); break;
     case 1: runManualStepper(stepperPush); break;
     case 2: runManualStepper(stepperBurner); break;     // Perbaikan ;
     case 3: runManualStepper(stepperBurnDoor); break;   // Perbaikan ;
     case 4: runManualStepper(stepperAsh); break;        // Perbaikan ;
     case 5: runManualStepper(stepperMainConveyor); break; // Perbaikan ;
   }
}

void runManualStepper(AccelStepper &motor) {
  if (currentState != IDLE) return;
  
  // Pastikan pin enable aktif
  digitalWrite(ENABLE_PIN, LOW); 
  
  motor.setMaxSpeed(manualSpeed);
  motor.setAcceleration(200);
  
  long target = isManualOpen ? manualSteps : -manualSteps;
  motor.move(target);
  
  // Debug
  Serial.print("MOVING MOT: "); Serial.println(target);
}

// Panggil fungsi ini saat command diterima
void performWeightCalibration() {
#if USE_HX711
  // 1. Baca nilai raw rata-rata
  long rawValue = scale.get_value(20); // Ambil rata-rata 20 sample
  
  // 2. Hitung Scale Factor: (Raw - ZeroOffset) / KnownWeight
  // Karena kita asumsikan sudah di-Tare sebelumnya (Offset sudah 0 relatif terhadap tare), 
  // maka rumusnya: Raw / KnownWeight
  
  float newScaleFactor = (float)rawValue / CAL_KNOWN_WEIGHT_KG;

  // 3. Validasi agar tidak error div by zero atau nilai ngawur
  if (newScaleFactor == 0) newScaleFactor = 1.0;

  // 4. Terapkan
  scale_factor = newScaleFactor;
  scale.set_scale(scale_factor);
  
  // 5. Simpan ke EEPROM
  saveSettings();

  // 6. Feedback ke Serial (bisa dibaca HMI jika perlu)
  Serial1.print("MSG:CAL_WEIGHT_OK, Factor:");
  Serial1.println(scale_factor);
#else
  Serial1.println("MSG:HX711_DISABLED");
#endif
}

void serviceLidar() {
  const uint16_t MAX_FRAMES_PER_CALL = 120; // batasi biar nggak ngunci loop
  uint16_t frames = 0;
#if USE_LIDAR
  while (Serial2.available() >= 5 && frames < MAX_FRAMES_PER_CALL) {
    frames++;
    int b0 = Serial2.read();
    bool s1 = (b0 >> 1) & 1;
    bool s0 = b0 & 1;
    if (s1 != !s0) { /* resync */ continue; }
    if (Serial2.available() < 4) break;
    byte b1=Serial2.read(), b2=Serial2.read(), b3=Serial2.read(), b4=Serial2.read();
    float angle = ((b1 >> 1) | (b2 << 7)) / 64.0f;
    float dist  = (b3 | (b4 << 8)) / 4.0f;
    int idx = (int)angle;
    if (dist > 0 && idx>=0 && idx<360) currentScan[idx] = dist;

    static int lastAng = 0;
    if (lastAng > 340 && idx < 20) processOneSweep();
    lastAng = idx;
  }

#else
  // ==================== SIMULATION LOGIC ====================
  
  // 1. Simulasi Visual (Isi array dengan noise agar grafik HMI bergerak)
  // Kita update visual agak jarang biar gak berat
  static unsigned long lastVisUpdate = 0;
  if (millis() - lastVisUpdate > 100) { 
    lastVisUpdate = millis();
    for (int i = 0; i < 360; i++) {
      float simulatedDistance = 0.0;
      // Buat gundukan simulasi di area tertentu
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

  // 2. Simulasi Kenaikan Volume (PENTING UNTUK STATE MACHINE)
  // Logic: Jika sedang WEIGHING dan Conveyor Jalan (Auto Mode), volume naik.
  if (currentState == WEIGHING && autoModeEnabled) {
    static unsigned long lastVolSimTime = 0;
    // Update volume setiap 100ms
    if (millis() - lastVolSimTime > 100) {
      lastVolSimTime = millis();
      
      // Tentukan kecepatan pengisian simulasi
      // Misal: Kita ingin batch penuh (mencapai minBatchVolume) dalam 10 detik (10000ms)
      // Rate = Target / (10000ms / 100ms per tick) = Target / 100
      float targetTimeSec = 10.0; 
      float ticks = targetTimeSec * 10.0; // karena update tiap 0.1 detik
      float increment = minBatchVolume / ticks; 

      // Tambah sedikit random noise biar natural (0.01 - 0.05 liter)
      float noise = (random(10, 50) / 1000.0f);
      
      double volInc = increment + noise;
      
      currentBatchVolume += volInc;
      

      // Debugging opsional (lihat di Serial Monitor)
      // Serial.print("Sim Vol:"); Serial.println(currentBatchVolume);
    }
  }
  
  // Catatan: Di mode simulasi, kita TIDAK memanggil processOneSweep()
  // karena fungsi itu butuh kalibrasi speed & scale yang rumit.
  // Kita langsung "bypass" mengisi variabel currentBatchVolume.

#endif

}

void checkfuelPercent(){
  if (now - lastFuelSentMs > 500) {
  fuelPct = readFuelPercent();
  fuelLiterRemaining = (fuelPct / 100.0f) * TANK_CAPACITY_L; // opsional

  Serial1.print("FUEL:");
  Serial1.print((int)fuelPct);
  Serial1.print(",");
  Serial1.println(fuelLiterRemaining, 1); // 1 digit desimal

  lastFuelSentMs = now;
}
}

// ================== SETUP & LOOP ==================
void setup() {
  Serial.begin(115200);     // Debug & Komunikasi ke HMI MCU

  Serial1.begin(115200, SERIAL_8N1);
  Serial.println("Aaa");

#if USE_DIMMER
  blower1.begin(NORMAL_MODE, ON);
  blower2.begin(NORMAL_MODE, ON);
#endif


#if USE_LIDAR
  Serial2.begin(460800);    // Lidar UART (Khusus)
  Serial2.setTimeout(1);
  for (int i = 0; i < 360; i++) { backgroundDist[i] = 0; currentScan[i] = 0; }
  resetLidar(); // Send stop/start cmd
#endif 
  
  loadSettings();
  if (isnan(minBatchWeight) || burnerActiveTimeSec == 0 || burnerActiveTimeSec > 86400) {
    minBatchWeight = 5.0;
    burnerActiveTimeSec = 30; 
    tempSetpoint = 1200;
    saveSettings();
  }

  pinMode(RELAY_BURNER, OUTPUT);
  pinMode(RELAY_IGNITION, OUTPUT);
  ignOff(); // Active Low assumption based on code

  pinMode(PIN_BLOWER_1, OUTPUT);
  pinMode(PIN_BLOWER_2, OUTPUT);

  pinMode(ENABLE_PIN, OUTPUT);
  digitalWrite(ENABLE_PIN, HIGH);

  pinMode(DATA_PIN, OUTPUT);
  pinMode(CLOCK_PIN, OUTPUT);
  pinMode(LATCH_PIN, OUTPUT);

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
}



int calculateProgress() {
  uint8_t val = 0;   // 0–100

  switch (currentState) {
    // 1) WEIGHING: SEKARANG BERDASARKAN VOLUME
    case WEIGHING: {
      if (minBatchVolume > 0) {
        float ratio = (float)currentBatchVolume / minBatchVolume;
        if (ratio < 0) ratio = 0;
        if (ratio > 1) ratio = 1;
        val = (uint8_t)(ratio * 100.0f);
      } else {
        val = 0;
      }
      break;
    }

    // 2) OPENING / CLOSING MAIN DOOR: berdasarkan posisi stepperDoor
    case OPENING_MAIN_DOOR: {
      long pos = stepperDoor.currentPosition();
      if (pos < POS_DOOR_CLOSE) pos = POS_DOOR_CLOSE;
      if (pos > POS_DOOR_OPEN)  pos = POS_DOOR_OPEN;
      val = (uint8_t)((pos - POS_DOOR_CLOSE) * 100L / (POS_DOOR_OPEN - POS_DOOR_CLOSE));
      break;
    }

    case CLOSING_MAIN_DOOR: {
      long pos = stepperDoor.currentPosition();
      if (pos < POS_DOOR_CLOSE) pos = POS_DOOR_CLOSE;
      if (pos > POS_DOOR_OPEN)  pos = POS_DOOR_OPEN;
      // kebalikan: dari open ke close
      val = (uint8_t)((POS_DOOR_OPEN - pos) * 100L / (POS_DOOR_OPEN - POS_DOOR_CLOSE));
      break;
    }

    // 3) DUMPING_IN & DUMPING_OUT: posisi stepperPush
    case DUMPING_IN: {
      long pos = stepperPush.currentPosition();
      if (pos < POS_PUSH_OUT) pos = POS_PUSH_OUT;
      if (pos > POS_PUSH_IN)  pos = POS_PUSH_IN;
      val = (uint8_t)((pos - POS_PUSH_OUT) * 100L / (POS_PUSH_IN - POS_PUSH_OUT));
      break;
    }

    case DUMPING_OUT: {
      long pos = stepperPush.currentPosition();
      if (pos < POS_PUSH_OUT) pos = POS_PUSH_OUT;
      if (pos > POS_PUSH_IN)  pos = POS_PUSH_IN;
      val = (uint8_t)((POS_PUSH_IN - pos) * 100L / (POS_PUSH_IN - POS_PUSH_OUT));
      break;
    }



    // 5) BURN DOOR & BURNER IN/OUT: posisi stepperBurnDoor & stepperBurner
    case OPENING_BURN_DOOR: {
      long pos = stepperBurnDoor.currentPosition();
      if (pos < POS_BDOOR_CLOSE) pos = POS_BDOOR_CLOSE;
      if (pos > POS_BDOOR_OPEN)  pos = POS_BDOOR_OPEN;
      val = (uint8_t)((pos - POS_BDOOR_CLOSE) * 100L / (POS_BDOOR_OPEN - POS_BDOOR_CLOSE));
      break;
    }

    case CLOSING_BURN_DOOR: {
      long pos = stepperBurnDoor.currentPosition();
      if (pos < POS_BDOOR_CLOSE) pos = POS_BDOOR_CLOSE;
      if (pos > POS_BDOOR_OPEN)  pos = POS_BDOOR_OPEN;
      val = (uint8_t)((POS_BDOOR_OPEN - pos) * 100L / (POS_BDOOR_OPEN - POS_BDOOR_CLOSE));
      break;
    }

    case BURNER_IN: {
      long pos = stepperBurner.currentPosition();
      if (pos < POS_BURNER_OUT) pos = POS_BURNER_OUT;
      if (pos > POS_BURNER_IN)  pos = POS_BURNER_IN;
      val = (uint8_t)((pos - POS_BURNER_OUT) * 100L / (POS_BURNER_IN - POS_BURNER_OUT));
      break;
    }

    case BURNER_OUT: {
      long pos = stepperBurner.currentPosition();
      if (pos < POS_BURNER_OUT) pos = POS_BURNER_OUT;
      if (pos > POS_BURNER_IN)  pos = POS_BURNER_IN;
      val = (uint8_t)((POS_BURNER_IN - pos) * 100L / (POS_BURNER_IN - POS_BURNER_OUT));
      break;
    }

    // 6) IGNITING: berdasarkan waktu priming (IGNITION_delay_MS)
    case IGNITING: {
      if (isPriming) {
        unsigned long elapsed = millis() - ignitiondelayStart;
        if (elapsed > IGNITION_delay_MS) elapsed = IGNITION_delay_MS;
        val = (uint8_t)(elapsed * 100UL / IGNITION_delay_MS);
      } else {
        val = 0;
      }
      break;
    }

    case PREHEAT: {
      float r = (tempSetpoint > 0) ? (currentTempC / tempSetpoint) : 0;
      if (r < 0) r = 0;
      if (r > 1) r = 1;
      val = (uint8_t)(r * 100.0f);
      break;
    }

    case BURNING: {
      unsigned long durationMS = burnerActiveTimeSec * 1000UL;
      unsigned long t = millis();
      unsigned long elapsed = burnAccumulatedMs + ((burnPhaseStartMs ? (t - burnPhaseStartMs) : 0));
      if (elapsed > durationMS) elapsed = durationMS;
      val = (uint8_t)(elapsed * 100UL / durationMS);
      break;
    }

    // default untuk IDLE & FAULT
    default:
      val = 0;
      break;
  }

  // jStat.setValue(val);
  return val;
}

void loop() {
  now = millis();

  // 1. Hardware Control (Realtime)
  runAllSteppers();

  static uint32_t lastSlowTick = 0;
  if (now - lastSlowTick > 50) {
    serviceBackgroundCalibration();
    checkIgnitionPriming();
    checkStepperFlags();
    serviceBurnCycle();
    updateBlowerDimmer();
    checkFeederConveyor();
    checkfuelPercent();
  }
  serviceLidar();
  runStateMachine();
  processIncomingSerial();

  // 5. Update Berkala ke HMI
  if (now - lastSensorMillis >= SENSOR_INTERVAL) {
    lastSensorMillis = now;
    readSensors();
    
    // Update Hardware 7-Segment (Lokal)
    if (currentState == WEIGHING) {
      convertMassValueto7Segment(currentWeightKg);
    } else {
      convertTemperatureValueto7Segment(currentTempC);
    }

    // Kirim Data ke HMI
    Serial1.print("DATA:"); 
    Serial1.print(currentTempC); Serial1.print(",");
    Serial1.print(currentWeightKg); Serial1.print(",");
    Serial1.print(currentState); Serial1.print(",");
    Serial1.println(currentBatchVolume);
  }
  
  // Kirim Progress Bar
  if (now - lastProgressSent > 500) {
      int prog = calculateProgress();
      Serial1.print("PROG:"); Serial1.println(prog); 
      lastProgressSent = now;
  }
}