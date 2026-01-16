// *EDIT Insinerator IoT v5 - Firmware untuk Arduino Mega (Konsep Batch)
// Refactored: Logging Removed & Loop Cleaned

#define SIMULATION 1    // 1 = tanpa hardware, 0 = dengan hardware

/* ====== INCLUDES ====== */
#include <Arduino.h>
#include <AccelStepper.h>
#include "HX711.h"
#include <SPI.h>
#include <MAX31856.h>
#include <Nextion.h>
#include <EEPROM.h>
#define nexSerial Serial2   // tambahkan ini
// ---------- Config ----------
//HardwareSerial &PCSerial   = Serial;   // To your PC (Serial Monitor)
//HardwareSerial &NextionSer = Serial2;  // To Nextion (TX2/RX2)


/* ====== EEPROM ADDRESS ====== */
#define EEPROM_ADDR_WEIGHT_SET  0
#define EEPROM_ADDR_TIME_SET    4
#define EEPROM_ADDR_TEMP_SET    8   // [BARU]
#define EEPROM_ADDR_SCALE       12
#define EEPROM_ADDR_MOTOR_CFG   20  // [BARU] Untuk simpan speed motor

/* ====== PIN CONFIG ====== */
// Sensor Pins
const int HX711_DT      = 38;
const int HX711_SCK     = 49;
const int MAX31856_CS   = 53;
const int MAX31856_SCK  = 52;
const int MAX31856_SDI  = 51;
const int MAX31856_SDO  = 50;

// Stepper Pins
// Stepper Pins (GANTI BAGIAN INI)
#define STEP_PIN_DOOR 2
#define DIR_PIN_DOOR 3
#define STEP_PIN_PUSH 4
#define DIR_PIN_PUSH 5
//const int STEP_PIN_PUSH = 4;  const int DIR_PIN_PUSH  = 5;
const int ENABLE_PIN    = 6;

// [PENTING] Ash Tray DIPINDAH ke 30/31 agar pin 44-46 kosong untuk Blower
const int STEP_PIN_ASH    = 30; const int DIR_PIN_ASH     = 31;
const int STEP_PIN_BURN   = 22; const int DIR_PIN_BURN    = 23;
const int STEP_PIN_BDOOR  = 24; const int DIR_PIN_BDOOR   = 25;

// [BARU] Blower Pins (PWM Hardware)
const int PIN_BLOWER_1 = 44;
const int PIN_BLOWER_2 = 45;
const int PIN_BLOWER_3 = 46;

// AC Motor / Relay Pins
const int RELAY_FEEDER_MOTOR   = 8;
const int RELAY_WEIGHING_MOTOR = 7;
const int RELAY_BURNER         = 9;
const int RELAY_IGNITION       = 10;

struct MotorConfig {
  long speed;
  long accel;
};

// Default Configs (Akan ditimpa EEPROM nanti)
MotorConfig motorConfigs[5] = {
  {1000, 200}, // 0: Door
  {1000, 200}, // 1: Push
  {1000, 200}, // 2: Burner
  {800, 200},  // 3: BurnDoor
  {800, 200}   // 4: Ash
};

// Variabel selektor untuk layar Pengaturan
int selectedMotorIdx = 0;
float tempSetpoint = 1200.0; 

/* ====== CONFIG MAX31856 ====== */
#define CR0_INIT  (CR0_AUTOMATIC_CONVERSION + CR0_OPEN_CIRCUIT_FAULT_TYPE_K + CR0_NOISE_FILTER_50HZ)
#define CR1_INIT  (CR1_AVERAGE_2_SAMPLES + CR1_THERMOCOUPLE_TYPE_S)
#define MASK_INIT (~(MASK_VOLTAGE_UNDER_OVER_FAULT + MASK_THERMOCOUPLE_OPEN_FAULT))

/* ====== HARDWARE OBJECTS ====== */
HX711 scale;
AccelStepper stepperDoor(AccelStepper::DRIVER, STEP_PIN_DOOR, DIR_PIN_DOOR);
AccelStepper stepperPush(AccelStepper::DRIVER, STEP_PIN_PUSH, DIR_PIN_PUSH);
AccelStepper stepperAsh(AccelStepper::DRIVER, STEP_PIN_ASH, DIR_PIN_ASH);
AccelStepper stepperBurner(AccelStepper::DRIVER, STEP_PIN_BURN, DIR_PIN_BURN);
AccelStepper stepperBurnDoor(AccelStepper::DRIVER, STEP_PIN_BDOOR, DIR_PIN_BDOOR);
MAX31856 *temperature;

/* ====== VARIABLES ====== */
// Motor Control
long speed = 1000;
long accel = 200;
bool motorDoorBusy = false;
bool motorPushBusy = false;
bool motorAshBusy = false;
bool motorBurnerBusy = false;
bool motorBurnDoorBusy = false;
bool feederMotorOn = false;
bool weighingMotorOn = false;

const long POS_DOOR_OPEN      = 2000;
const long POS_DOOR_CLOSE     = 0;

const long POS_PUSH_IN        = 5000; // Posisi Feeder Masuk
const long POS_PUSH_OUT       = 0;    // Posisi Feeder Standby (Luar)

const long POS_BURNER_IN      = 3000; // Burner masuk ke ruang bakar
const long POS_BURNER_OUT     = 0;    // Burner standby diluar

const long POS_BDOOR_OPEN     = 1000; // Pintu Burner Buka
const long POS_BDOOR_CLOSE    = 0;    // Pintu Burner Tutup

// Burner & Ignition Logic
bool burnerActive = false;
bool isPriming    = false;
bool burnTimerStarted = false;   

unsigned long burnerStartTime     = 0;
unsigned long lastCountdownUpdate = 0;
unsigned long ignitionDelayStart  = 0;
unsigned long now;
const unsigned long IGNITION_DELAY_MS = 10000;

// System State
bool autoModeEnabled = false;
enum SystemState {
  IDLE,
  WEIGHING,
  OPENING_MAIN_DOOR,
  DUMPING_IN,
  DUMPING_TRASH,
  DUMPING_OUT,
  CLOSING_MAIN_DOOR,

  OPENING_BURN_DOOR,
  BURNER_IN,
  IGNITING,
  BURNING,
  BURNER_OUT,
  CLOSING_BURN_DOOR,

  FAULT
};
SystemState currentState = IDLE;

int lastState = -1;

// Metrics & Sensor Data
float currentTempC = 0.0;
float currentWeightKg = 0.0;
unsigned long lastSensorMillis = 0;
const unsigned long SENSOR_INTERVAL = 2500;

float minBatchWeight = 5.0;
uint32_t burnerActiveTimeSec = 30;
const unsigned long DUMP_DURATION_MS = 10000;
unsigned long actionStartTime = 0;

float solarVolumeUsed_L = 0.0;
float totalConsumedKg = 0.0;
unsigned long totalBatchCount = 0;
const float LITERS_PER_HOUR_BURNER = 10.0;
float sfc_L_per_kg = 0.0;

const float OVERHEAT_TEMP = 1400.0;
float scale_factor = -7050.0;
long scale_zero = 0;

long remaining = 0;
long last_remaining = -1;

// Logic Helper Manual
bool isManualOpen = true; // Status tombol OPEN/CLOSE (True=Open)
long manualSteps = 1000;  // Nilai default langkah stepper
long manualSpeed = 1000;  // Nilai default kecepatan

int blowLevel[3] = {0, 0, 0}; // Simpan status tiap blower
long target = 0;

/* ====== NEXTION OBJECTS ====== */
// --- PAGE 0: DASHBOARD ---
NexText tStat       = NexText(0, 2, "tStat");    // Status Operasi
NexText tTemp       = NexText(0, 3, "tTemp");    // Suhu Ruang
NexText tWeight     = NexText(0, 4, "tWeight");  // Berat Muatan
NexText tTime       = NexText(0, 5, "tTime");    // Waktu Bakar (Realtime)
NexText tFuel       = NexText(0, 6, "tFuel");    // Est. BBM
NexText tBatch      = NexText(0, 7, "tBatch");   // Total Batch
NexText tTotWeight  = NexText(0, 8, "tTotWeight");// Total Weight
NexDSButton btAuto  = NexDSButton(0, 9, "btAuto");
//NexButton bStart    = NexButton(0, 9, "bStart");
NexButton bStop     = NexButton(0, 13, "bStop");
NexProgressBar jStat = NexProgressBar(0, 16, "jStat");


// --- PAGE 1: KONTROL MANUAL ====== */

NexDSButton btManDir   = NexDSButton(1, 2, "btManDir"); // 1=Close, 0=Open (Tergantung desainmu)

NexButton bManDoor     = NexButton(1, 7, "bManDoor");   // Pintu Utama
NexButton bManBDoor    = NexButton(1, 9, "bManBDoor");  // Pintu Burner
NexButton bManBurn     = NexButton(1, 8, "bManBurn");   // Burner Mover
NexButton bManPush     = NexButton(1, 10, "bManPush");   // Feeder / Push
NexButton bManAsh      = NexButton(1, 11, "bManAsh");    // Ash Tray

NexSlider hManSpeed    = NexSlider(1, 12, "hManSpeed");  // Slider Speed
NexSlider hManStep     = NexSlider(1, 13, "hManStep");   // Slider Steps (Jarak)

NexDSButton btManBlow  = NexDSButton(1, 3, "btManBlow");  // Blower
NexDSButton btManIgn   = NexDSButton(1, 4, "btManIgn");   // Igniter
NexDSButton btManFeed  = NexDSButton(1, 5, "btManFeed");  // Feeder Motor
NexDSButton btManWeigh = NexDSButton(1, 6, "btManWeigh"); // Weighing Motor

NexText tManTemp       = NexText(1, 15, "tManTemp");
NexText tManWeight     = NexText(1, 16, "tManWeight");
NexButton bManStop     = NexButton(1, 14, "bManStop");
NexButton b0     = NexButton(1, 19, "b0");


/// --- PAGE 2: PENGATURAN [BARU] ---
// Display Angka
NexText nTempSet    = NexText(2, 26, "nTempSet");
NexText nTimeSet    = NexText(2, 25, "nTimeSet");
NexText nWeightSet  = NexText(2, 24, "nWeightSet");

// Tombol Plus Minus (Incremental)
NexButton bTempUp     = NexButton(2, 8, "bTempUp");
NexButton bTempDown   = NexButton(2, 9, "bTempDown");
NexButton bTimeUp     = NexButton(2, 7, "bTimeUp");
NexButton bTimeDown   = NexButton(2, 6, "bTimeDown");
NexButton bWeightUp   = NexButton(2, 5, "bWeightUp");
NexButton bWeightDown = NexButton(2, 4, "bWeightDown");

// Tombol Pilih Motor (Radio Button Logic)
NexButton bSelDoor    = NexButton(2, 14, "bSelDoor");
NexButton bSelBurn    = NexButton(2, 12, "bSelBurn");
NexButton bSelFeed    = NexButton(2, 13, "bSelFeed");
NexButton bSelBDoor   = NexButton(2, 15, "bSelBDoor");

// Slider Tuning
NexSlider hSpeed      = NexSlider(2, 11, "hSpeed");
NexSlider hStep       = NexSlider(2, 10, "hStep");

// Tombol Blower (Low/Med/High/Off Logic)
NexButton bBlow1      = NexButton(2, 22, "bBlow1");
NexButton bBlow2      = NexButton(2, 21, "bBlow2");
NexButton bBlow3      = NexButton(2, 20, "bBlow3");

// Tombol Save & Zero
NexButton bSave       = NexButton(2, 19, "bSave");
NexButton bZero       = NexButton(2, 16, "bZero");
NexText tWeightCal     = NexText(0, 23, "tWeightCal");  // Berat Muatan


NexTouch *nex_listen_list[] = {
  // --- Page 0: Dashboard ---
  &btAuto,  &bStop,
//&bStart,
  // Settings Page
  &nWeightSet, &nTimeSet, // (Ini objek sisa dr kode lama, boleh dihapus kalau tidak dipakai)
  &bTempUp, &bTempDown, &bTimeUp, &bTimeDown, &bWeightUp, &bWeightDown,
  &bSelDoor, &bSelBurn, &bSelFeed, &bSelBDoor,
  &hSpeed, &hStep,
  &bBlow1, &bBlow2, &bBlow3,
  &bSave, &bZero,

  // Manual Page
  &btManDir, 
  &bManDoor, &bManBDoor, &bManBurn, &bManPush, &bManAsh,
  &hManSpeed, &hManStep, &bManStop,
  &btManBlow, &btManIgn, &btManFeed, &btManWeigh, &b0,

  // Tombol Selector Motor
  &bSelDoor, &bSelBurn, &bSelFeed, &bSelBDoor,

  // Slider Tuning
  &hSpeed, &hStep,

  // Blower & Tombol Fungsi
  &bBlow1, &bBlow2, &bBlow3,
  &bSave, &bZero,

  NULL 
};

/* ====== UTILITIES ====== */

void emergencyStop() {
  // Matikan semua aktuator
  digitalWrite(RELAY_BURNER, LOW);
  burnerActive = false;
  burnTimerStarted = false;

  // Matikan motor AC
  digitalWrite(RELAY_FEEDER_MOTOR, LOW);
  feederMotorOn = false;
  digitalWrite(RELAY_WEIGHING_MOTOR, LOW);
  weighingMotorOn = false;

  // Hentikan dan nonaktifkan stepper
  digitalWrite(ENABLE_PIN, HIGH); // Disable stepper
  stepperDoor.stop();
  stepperPush.stop();
  stepperAsh.stop();
  stepperBurner.stop();
  stepperBurnDoor.stop();
}

void tareScale() {
  if (currentState != FAULT) {
#if !SIMULATION
    scale.tare(20);
    scale_zero = scale.get_units(10);
#else
    return;
#endif
  }
}

//===================== Selector motor & slider ==============
void updateSliderFromConfig() {
  // Sinkronkan posisi slider dengan nilai tersimpan saat ganti motor
  hSpeed.setValue(motorConfigs[selectedMotorIdx].speed);
}

// Saat tombol "Pintu Utama", "Burner Mover" dll ditekan
void bSelDoorPopCallback(void *ptr) {
  selectedMotorIdx = 0;
  updateSliderFromConfig();
}

void b0PopCallback(void *ptr) {
  runManualStepper1(stepperPush, 1200);
  runManualStepper1(stepperDoor, 7100);
  runManualStepper1(stepperDoor, -7100);
  runManualStepper1(stepperPush, -1200);
}
void bSelFeedPopCallback(void *ptr) {
  selectedMotorIdx = 1;
  updateSliderFromConfig();
}
void bSelBurnPopCallback(void *ptr) {
  selectedMotorIdx = 2;
  updateSliderFromConfig();
}
void bSelBDoorPopCallback(void *ptr) {
  selectedMotorIdx = 3;
  updateSliderFromConfig();
}

// Saat Slider digeser
void hSpeedPopCallback(void *ptr) {
  uint32_t val;
  hSpeed.getValue(&val);
  motorConfigs[selectedMotorIdx].speed = val; // Update variabel konfigurasi

  // Update langsung ke motor yang sesuai agar terasa efeknya
  if (selectedMotorIdx == 0) stepperDoor.setMaxSpeed(val);
  else if (selectedMotorIdx == 1) stepperPush.setMaxSpeed(val);
  else if (selectedMotorIdx == 2) stepperBurner.setMaxSpeed(val);
  else if (selectedMotorIdx == 3) stepperBurnDoor.setMaxSpeed(val);
  else if (selectedMotorIdx == 4) stepperAsh.setMaxSpeed(val);
}

//====================== Save settings for page 2 ======================

void loadSettings() {
  EEPROM.get(EEPROM_ADDR_WEIGHT_SET, minBatchWeight);
  EEPROM.get(EEPROM_ADDR_TIME_SET, burnerActiveTimeSec);
  EEPROM.get(EEPROM_ADDR_TEMP_SET, tempSetpoint);

  // Load Settingan Motor satu per satu
  for (int i = 0; i < 5; i++) {
    EEPROM.get(EEPROM_ADDR_MOTOR_CFG + (i * sizeof(MotorConfig)), motorConfigs[i]);
    if (motorConfigs[i].speed < 100) motorConfigs[i].speed = 1000; // Safety
  }
  // Apply ke Hardware
//  stepperDoor.setMaxSpeed(motorConfigs[0].speed);
  stepperPush.setMaxSpeed(motorConfigs[1].speed);
  stepperBurner.setMaxSpeed(motorConfigs[2].speed);
  stepperBurnDoor.setMaxSpeed(motorConfigs[3].speed);
  stepperAsh.setMaxSpeed(motorConfigs[4].speed);
}

void saveSettings() {
  EEPROM.put(EEPROM_ADDR_WEIGHT_SET, minBatchWeight);
  EEPROM.put(EEPROM_ADDR_TIME_SET, burnerActiveTimeSec);
  EEPROM.put(EEPROM_ADDR_TEMP_SET, tempSetpoint);

  for (int i = 0; i < 5; i++) {
    EEPROM.put(EEPROM_ADDR_MOTOR_CFG + (i * sizeof(MotorConfig)), motorConfigs[i]);
  }
}


void bSavePopCallback(void *ptr) {
  saveSettings(); // Panggil fungsi save EEPROM
}

//====================== Blower ======================


void setBlowerPWM(int pin, int level) {
  int pwmVal = 0;
  if (level == 1) pwmVal = 85;       // Low (~33%)
  else if (level == 2) pwmVal = 170; // Med (~66%)
  else if (level == 3) pwmVal = 255; // High (100%)
  analogWrite(pin, pwmVal);
}

void updateBlowerState(int id) {
  // Logic: Tekan tombol -> Level naik -> Mentok balik ke 0 (Off)
  blowLevel[id]++;
  if (blowLevel[id] > 3) blowLevel[id] = 0;

  int pin = (id == 0) ? PIN_BLOWER_1 : (id == 1) ? PIN_BLOWER_2 : PIN_BLOWER_3;
  setBlowerPWM(pin, blowLevel[id]);
}

void bBlow1PopCallback(void *ptr) { updateBlowerState(0); }
void bBlow2PopCallback(void *ptr) { updateBlowerState(1); }
void bBlow3PopCallback(void *ptr) { updateBlowerState(2); }


//====================== Actuating Function ======================

void motorTurnCW(AccelStepper &motor, bool &busyFlag, long distance, long maxSpeedVal, long accelVal) {
  if (busyFlag) return;
  busyFlag = true;

  motor.setPinsInverted(false, false, false);
  motor.setMaxSpeed(maxSpeedVal);
  motor.setAcceleration(accelVal);
  motor.move(distance);
}

void motorTurnCCW(AccelStepper &motor, bool &busyFlag, long distance, long maxSpeedVal, long accelVal) {
  if (busyFlag) return;
  busyFlag = true;

  motor.setPinsInverted(true, false, false);
  motor.setMaxSpeed(maxSpeedVal);
  motor.setAcceleration(accelVal);
  motor.move(distance);
}


/* ====== NEXTION CALLBACKS ====== */

// --- 1. Dashboard & General ---
void btAutoPopCallback(void *ptr) {
  uint32_t dual_state;
  btAuto.getValue(&dual_state);
  if (dual_state == 1) {
    //Serial.println("AUTO START");
    autoModeEnabled = true;
    if (currentState == IDLE) {
      currentState = WEIGHING;
      tareScale();
    }
  } else {
    autoModeEnabled = false;
  }
}

//void bStartPopCallback(void *ptr) {
//  if (currentState == IDLE && autoModeEnabled) {
//      currentState = WEIGHING;
//      tareScale();
//  }
//}

void bStopPopCallback(void *ptr) {
  currentState = IDLE;
  emergencyStop();
}

// --- 2. Manual Control ---
void btManDirPopCallback(void *ptr) {
  uint32_t val;
  btManDir.getValue(&val);
  isManualOpen = (val == 0); 
}

void hManSpeedPopCallback(void *ptr) {
  uint32_t val;
  hManSpeed.getValue(&val);
  manualSpeed = map(val, 0, 100, 0, 10000); 
}

void hManStepPopCallback(void *ptr) {
  uint32_t val;
  hManStep.getValue(&val);
  manualSteps = map(val, 0, 100, 0, 20000); 
}

// Fungsi Helper Gerak Manual
void runManualStepper(AccelStepper &motor) {
  if (currentState != IDLE) return; 
  digitalWrite(ENABLE_PIN, LOW); 
  motor.setMaxSpeed(manualSpeed);
  Serial.println(manualSpeed);
  motor.setAcceleration(200);
  
  long target = isManualOpen ? manualSteps : -manualSteps;
//  motor.run();
  motor.move(target);
  Serial.println(target); 
}

void runManualStepper1(AccelStepper &motor, long target) {
  if (currentState != IDLE) return; 
  digitalWrite(ENABLE_PIN, LOW); 
  motor.setMaxSpeed(manualSpeed);
  Serial.println(manualSpeed);
  motor.setAcceleration(200);
  
//  long target = isManualOpen ? manualSteps : -manualSteps;
//  motor.run();
  motor.move(target);
  motor.runToPosition(); 
  Serial.println(target); 
}


//void runManualStepper(AccelStepper &motor) {
////  if (currentState != IDLE) return; 
////  digitalWrite(ENABLE_PIN, LOW); 
////  stepperDoor.setMaxSpeed(1000);
////  Serial.println(manualSpeed);
////  stepperDoor.setAcceleration(200);
////  long target = isManualOpen ? manualSteps : -manualSteps;
//  stepperDoor.move(1000);
////  Serial.println(target); 
//}

// --- CALLBACK TOMBOL MOTOR ---
void bManDoorPopCallback(void *ptr)  { runManualStepper(stepperDoor);Serial.println("Mandoor"); }
void bManBDoorPopCallback(void *ptr) { runManualStepper(stepperBurnDoor); }
void bManBurnPopCallback(void *ptr)  { runManualStepper(stepperBurner); }
void bManPushPopCallback(void *ptr)  { runManualStepper(stepperPush); Serial.println("Pusher"); }
void bManAshPopCallback(void *ptr)   { runManualStepper(stepperAsh); }

// --- CALLBACK RELAY (ON/OFF) ---
void btManBlowPopCallback(void *ptr) {
  uint32_t val; btManBlow.getValue(&val);
  if(val == 1) { 
    setBlowerPWM(PIN_BLOWER_1, 3); setBlowerPWM(PIN_BLOWER_2, 3); setBlowerPWM(PIN_BLOWER_3, 3);
  } else { 
    setBlowerPWM(PIN_BLOWER_1, 0); setBlowerPWM(PIN_BLOWER_2, 0); setBlowerPWM(PIN_BLOWER_3, 0);
  }
}

void btManIgnPopCallback(void *ptr) {
  uint32_t val; btManIgn.getValue(&val);
  digitalWrite(RELAY_IGNITION, (val==1) ? LOW : HIGH); // Cek active low/high relaymu
}

void btManFeedPopCallback(void *ptr) {
  uint32_t val; btManFeed.getValue(&val);
  digitalWrite(RELAY_FEEDER_MOTOR, (val==1) ? HIGH : LOW);
}

void btManWeighPopCallback(void *ptr) {
  uint32_t val; btManWeigh.getValue(&val);
  digitalWrite(RELAY_WEIGHING_MOTOR, (val==1) ? HIGH : LOW);
}

// --- 3. Settings Page ---
void showTempSet() {
  char buf[16];
  sprintf(buf, "%d C", (int)tempSetpoint);
  nTempSet.setText(buf);
}

void showWeightSet() {
  char buf[16];
  sprintf(buf, "%d kg", (int)minBatchWeight);
  nWeightSet.setText(buf);
}

void showTimeSet() {
  char buf[16];
  // burnerActiveTimeSec disimpan dalam detik → tampilkan menit
  uint32_t menit = burnerActiveTimeSec / 60;
  sprintf(buf, "%lu min", (unsigned long)menit);   // "15 min"
  nTimeSet.setText(buf);
}



void bTempUpPopCallback(void *ptr) {
  tempSetpoint += 50.0;
  showTempSet();
}
void bTempDownPopCallback(void *ptr) {
  tempSetpoint -= 50.0;
  if (tempSetpoint < 0) tempSetpoint = 0;
  showTempSet();
}

void bTimeUpPopCallback(void *ptr) {
  burnerActiveTimeSec += 300; // +5 menit
  showTimeSet();
}
void bTimeDownPopCallback(void *ptr) {
  if (burnerActiveTimeSec >= 300) burnerActiveTimeSec -= 300;
  showTimeSet();
}


void bWeightUpPopCallback(void *ptr) {
  minBatchWeight += 1.0;
  showWeightSet();
}
void bWeightDownPopCallback(void *ptr) {
  if (minBatchWeight >= 1.0) minBatchWeight -= 1.0;
  showWeightSet();
}




void bZeroPopCallback(void *ptr) {
  if (currentState == IDLE || currentState == WEIGHING) {
    tareScale();
  }
}



void nWeightSetPopCallback(void *ptr) {
  char buf[16] = {0};
  nWeightSet.getText(buf, sizeof(buf)-1);
  // atof akan berhenti saat ketemu huruf (k, g, spasi), jadi "5 kg" masih aman
  minBatchWeight = atof(buf);
  EEPROM.put(EEPROM_ADDR_WEIGHT_SET, minBatchWeight);
  showWeightSet();   // refresh supaya formatnya rapi "x kg"
}


void nTimeSetPopCallback(void *ptr) {
  char buf[16] = {0};
  nTimeSet.getText(buf, sizeof(buf)-1);
  uint32_t menit = (uint32_t)atoi(buf);   // baca angka awal
  burnerActiveTimeSec = menit * 60;       // simpan dalam detik
  EEPROM.put(EEPROM_ADDR_TIME_SET, burnerActiveTimeSec);
  showTimeSet();
}


// ====== Callback Control Manual =====

// --- Buka Pintu ---
void bOpenDoorPopCallback(void *ptr) {
  if (currentState != IDLE) return;
  digitalWrite(ENABLE_PIN, LOW);
  motorTurnCCW(stepperDoor, motorDoorBusy, 5000, 1000, 350);
  // tDoorState.setText("Door State: OPEN");
}

// --- Tutup Pintu ---
void bCloseDoorPopCallback(void *ptr) {
  if (currentState != IDLE || autoModeEnabled == true) return;
  motorTurnCW(stepperDoor, motorDoorBusy, 5000, 2000, 350);
  // tDoorState.setText("Door State: CLOSED");
}

// --- Push mendorong sampah ---
void bTurnOnPushPopCallback(void *ptr) {
  if (currentState != IDLE || autoModeEnabled == true) return;
  motorTurnCCW(stepperPush, motorPushBusy, 5000, 2000, 350);
  // tPushState.setText("Push State: ON");
}

// --- Push Stop mendorong sampah ---
void bTurnOffPushPopCallback(void *ptr) {
  if (currentState != IDLE || autoModeEnabled == true) return;
  motorTurnCW(stepperPush, motorPushBusy, 5000, 2000, 350);
  // tPushState.setText("Push State: OFF");
}

// --- Conveyor ON ---
void bTurnOnConveyPopCallback(void *ptr) {
  if (currentState != IDLE || autoModeEnabled == true) return;
  // tConveyState.setText("Conveyor State: ON");
  digitalWrite(RELAY_WEIGHING_MOTOR, HIGH);
}

// --- Conveyor OFF ---
void bTurnOffConveyPopCallback(void *ptr) {
  if (currentState != IDLE || autoModeEnabled == true) return;
  // tConveyState.setText("Conveyor State: OFF");
  digitalWrite(RELAY_WEIGHING_MOTOR, LOW);
}

//// --- Set Burner Timer Up ---
//void bTimeUpPopCallback(void *ptr) {
//  if (currentState != IDLE || autoModeEnabled == true) return;
//  burnerDuration += 1000;
//  char buffer[10];
//  sprintf(buffer, "%ld", burnerDuration / 1000);
//  tBurningTime.setText(buffer);
//}
//
//// --- Set Burner Timer Down ---
//void bTimeDownPopCallback(void *ptr) {
//  if (currentState != IDLE || autoModeEnabled == true) return;
//  if (burnerDuration > 1000) burnerDuration -= 1000;
//  char buffer[10];
//  sprintf(buffer, "%ld", burnerDuration / 1000);
//  tBurningTime.setText(buffer);
//}

// --- Turn On Burner ---
void bTurnOnBurnerPopCallback(void *ptr) {
  if (currentState != IDLE || autoModeEnabled == true) return;
  digitalWrite(RELAY_IGNITION, LOW);

  if (burnerActive) return;

  isPriming = true;
  ignitionDelayStart = millis();

  burnerActive = true;
  burnerStartTime = millis();

  digitalWrite(RELAY_BURNER, HIGH);

  // tBurnerState.setText("Burner State: ON");
}

void bStopEmergencyPopCallback(void *ptr) {
  // ----- STOP DOOR -----
//  stepperDoor.stop();
//  stepperDoor.setCurrentPosition(stepperDoor.currentPosition());
//  stepperDoor.move(0);
  motorDoorBusy = false;
  // tDoorState.setText("Door State: STOP");

  // ----- STOP Push -----
  stepperPush.stop();
  stepperPush.setCurrentPosition(stepperPush.currentPosition());
  stepperPush.move(0);
  motorPushBusy = false;
  // tPushState.setText("Push State: STOP");

  // ----- STOP BURNER -----
  burnerActive = false;
  burnerStartTime = 0;
  digitalWrite(RELAY_BURNER, LOW);
  digitalWrite(RELAY_IGNITION, HIGH);
  // tBurnerState.setText("Burner State: OFF");

  // ----- STOP CONVEYOR -----
  digitalWrite(RELAY_WEIGHING_MOTOR, LOW);
  // tConveyState.setText("Conveyor State: OFF");
}


/* ====== SETUP ====== */
void setup() {
  Serial.begin(9600);
  Serial2.begin(9600);
//  Serial2.begin(9600, SERIAL_8N1, 16, 17);  delay(100);

  // Init EEPROM & Load
//  EEPROM.begin();
  loadSettings();
//  stepperDoor.setMaxSpeed(1000);
////  Serial.println(manualSpeed);
//  stepperDoor.setAcceleration(200);// put your setup code here, to run once:

  showTempSet();
  showTimeSet();
  showWeightSet();
  if (isnan(minBatchWeight) || burnerActiveTimeSec == 0 || burnerActiveTimeSec > 86400) {
    minBatchWeight = 5.0;
    burnerActiveTimeSec = 1; 
    tempSetpoint = 1200;
    saveSettings();
}

  // Validasi Awal
  if (isnan(minBatchWeight)) minBatchWeight = 5.0;
  if (burnerActiveTimeSec == 0) burnerActiveTimeSec = 30;

#if !SIMULATION
  // Pins Init
  pinMode(RELAY_FEEDER_MOTOR, OUTPUT);
  pinMode(RELAY_WEIGHING_MOTOR, OUTPUT);
  pinMode(RELAY_BURNER, OUTPUT);
  pinMode(RELAY_IGNITION, OUTPUT);
  digitalWrite(RELAY_IGNITION, HIGH);

  pinMode(PIN_BLOWER_1, OUTPUT);
  pinMode(PIN_BLOWER_2, OUTPUT);
  pinMode(PIN_BLOWER_3, OUTPUT);

  pinMode(ENABLE_PIN, OUTPUT);
  digitalWrite(ENABLE_PIN, HIGH);




  SPI.begin();
  temperature = new MAX31856(MAX31856_SDI, MAX31856_SDO, MAX31856_CS, MAX31856_SCK);
  temperature->writeRegister(REGISTER_CR0, CR0_INIT);

  scale.begin(HX711_DT, HX711_SCK);
  scale.set_scale(scale_factor);
  tareScale();
#else
  currentTempC = 25.0;
  //Serial.println("STARTING SYSTEM");
#endif

  nexInit();

  bTempUp.attachPop(bTempUpPopCallback, &bTempUp);
  bTempDown.attachPop(bTempDownPopCallback, &bTempDown);

  // 1. Dashboard Callbacks
  btAuto.attachPop(btAutoPopCallback, &btAuto);
  bStop.attachPop(bStopPopCallback, &bStop);
  bManStop.attachPop(bStopPopCallback, &bManStop);

  nWeightSet.attachPop(nWeightSetPopCallback, &nWeightSet);
  nTimeSet.attachPop(nTimeSetPopCallback, &nTimeSet); 

  // 2. Manual Control Callbacks
  btManDir.attachPop(btManDirPopCallback, &btManDir);
  bManDoor.attachPop(bManDoorPopCallback, &bManDoor);
  bManBDoor.attachPop(bManBDoorPopCallback, &bManBDoor);
  bManBurn.attachPop(bManBurnPopCallback, &bManBurn);
  bManPush.attachPop(bManPushPopCallback, &bManPush);
  bManAsh.attachPop(bManAshPopCallback, &bManAsh);
  
  hManSpeed.attachPop(hManSpeedPopCallback, &hManSpeed);
  hManStep.attachPop(hManStepPopCallback, &hManStep);
  
  btManBlow.attachPop(btManBlowPopCallback, &btManBlow);
  btManIgn.attachPop(btManIgnPopCallback, &btManIgn);
  btManFeed.attachPop(btManFeedPopCallback, &btManFeed);
  btManWeigh.attachPop(btManWeighPopCallback, &btManWeigh);
  b0.attachPop(b0PopCallback, &b0);


  // 3. Settings Page Callbacks
  bTempUp.attachPop(bTempUpPopCallback, &bTempUp);
  bTempDown.attachPop(bTempDownPopCallback, &bTempDown);
  bTimeUp.attachPop(bTimeUpPopCallback, &bTimeUp);
  bTimeDown.attachPop(bTimeDownPopCallback, &bTimeDown);
  bWeightUp.attachPop(bWeightUpPopCallback, &bWeightUp);
  bWeightDown.attachPop(bWeightDownPopCallback, &bWeightDown);

  bSelDoor.attachPop(bSelDoorPopCallback, &bSelDoor);
  bSelBurn.attachPop(bSelBurnPopCallback, &bSelBurn);
  bSelFeed.attachPop(bSelFeedPopCallback, &bSelFeed);
  bSelBDoor.attachPop(bSelBDoorPopCallback, &bSelBDoor);

  hSpeed.attachPop(hSpeedPopCallback, &hSpeed);

  bBlow1.attachPop(bBlow1PopCallback, &bBlow1);
  bBlow2.attachPop(bBlow2PopCallback, &bBlow2);
  bBlow3.attachPop(bBlow3PopCallback, &bBlow3);

  bSave.attachPop(bSavePopCallback, &bSave);
  bZero.attachPop(bZeroPopCallback, &bZero);

//  emergencyStop();
}



/* ====== HELPER FUNCTIONS FOR LOOP ====== */

void runAllSteppers() {
  if (currentState != FAULT) {
    stepperDoor.run();
    stepperPush.run();
    stepperAsh.run();
    stepperBurner.run();
    stepperBurnDoor.run();
  }
}

void checkIgnitionPriming() {
  if (!isPriming) return;

  if (millis() - ignitionDelayStart >= IGNITION_DELAY_MS) {
    digitalWrite(RELAY_IGNITION, HIGH);
    isPriming = false;
    burnerActive = true;

    // Timer BELUM mulai sampai suhu capai setpoint
    burnerStartTime   = 0;
    burnTimerStarted  = false;

    currentState = BURNING;

  } else {
    long remaining = (IGNITION_DELAY_MS - (millis() - ignitionDelayStart)) / 1000;
    if (remaining != last_remaining) {
      last_remaining = remaining;
    }
  }
}

void checkStepperFlags() {
//  if (motorDoorBusy      && stepperDoor.distanceToGo()   == 0) motorDoorBusy      = false;
  if (motorPushBusy      && stepperPush.distanceToGo()   == 0) motorPushBusy      = false;
  if (motorAshBusy       && stepperAsh.distanceToGo()    == 0) motorAshBusy       = false;
  if (motorBurnerBusy    && stepperBurner.distanceToGo() == 0) motorBurnerBusy    = false;
  if (motorBurnDoorBusy  && stepperBurnDoor.distanceToGo()== 0) motorBurnDoorBusy = false;
}

void checkBurnerTimer() {
  // Timer hanya relevan kalau burner ON dan di state BURNING
  if (!burnerActive || currentState != BURNING) return;

  unsigned long currentMillis = millis();

  // 1) Mulai timer HANYA setelah suhu mencapai setpoint
  if (!burnTimerStarted) {
    if (currentTempC >= tempSetpoint) {
      burnerStartTime  = currentMillis;
      burnTimerStarted = true;
    } else {
      // Belum mencapai setpoint -> tampilkan PREHEAT / WAIT
      tTime.setText("PRE HEAT");
      return;  // belum mulai ngitung durasi
    }
  }

  // 2) Kalau sudah mulai, jalankan hitungan mundur normal
  unsigned long durationMS = burnerActiveTimeSec * 1000UL;

  if (currentMillis - burnerStartTime >= durationMS) {
    burnerActive = false;
    burnTimerStarted = false;
    digitalWrite(RELAY_BURNER, LOW);
  }
  else if (currentMillis - lastCountdownUpdate >= 1000) {
    lastCountdownUpdate = currentMillis;

    unsigned long elapsed         = currentMillis - burnerStartTime;
    long remainingSeconds         = (durationMS - elapsed) / 1000;
    if (remainingSeconds < 0) remainingSeconds = 0;

    char timeBuf[10];
    sprintf(timeBuf, "%02ld:%02ld", remainingSeconds/60, remainingSeconds%60);
    tTime.setText(timeBuf);
  }
}



void checkPeriodicUpdates() {
  if (now - lastSensorMillis >= SENSOR_INTERVAL) {
    lastSensorMillis = now;
    readSensors();
    updateHMI();
  }
}

void runStateMachine() {
  switch (currentState) {
    case IDLE:              handleIdle(); break;
    case WEIGHING:          handleWeighing(); break;
    case OPENING_MAIN_DOOR: handleOpeningMainDoor(); break;
    case DUMPING_IN:        handleDumpingIn(); break;
    case DUMPING_TRASH:     handleDumpingTrash(); break;
    case DUMPING_OUT:       handleDumpingOut(); break;
    case CLOSING_MAIN_DOOR: handleClosingMainDoor(); break;
    case OPENING_BURN_DOOR: handleOpeningBurnDoor(); break;
    case BURNER_IN:         handleBurnerIn(); break;
    case IGNITING:          handleIgniting(); break;
    case BURNING:           handleBurning(); break;
    case BURNER_OUT:        handleBurnerOut(); break;
    case CLOSING_BURN_DOOR: handleClosingBurnDoor(); break;
    case FAULT:             handleFault(); break;
  }
}

/* ====== STATE HANDLERS ====== */

void handleIdle() {
  //  emergencyStop();
//  Serial.println("aaa");
  if (stepperDoor.currentPosition() != POS_DOOR_CLOSE) {
    digitalWrite(ENABLE_PIN, LOW);
    stepperDoor.moveTo(POS_DOOR_CLOSE);
  } else {
    digitalWrite(ENABLE_PIN, HIGH);
  }
}

void handleWeighing() {
  digitalWrite(ENABLE_PIN, HIGH);

  digitalWrite(RELAY_FEEDER_MOTOR, HIGH); feederMotorOn = true;
  digitalWrite(RELAY_WEIGHING_MOTOR, LOW); weighingMotorOn = false;

  // SIMULATION: berat sudah disimulasikan di readSensors(), jadi ini sudah OK
  if (currentWeightKg >= minBatchWeight) {
    digitalWrite(RELAY_FEEDER_MOTOR, LOW); feederMotorOn = false;
    currentState = OPENING_MAIN_DOOR;
  }
}

void handleOpeningMainDoor() {
#if SIMULATION
  // Anggap pintu langsung kebuka
  stepperDoor.setCurrentPosition(POS_DOOR_OPEN);
  currentState = DUMPING_IN;
#else
  digitalWrite(ENABLE_PIN, LOW);
  stepperDoor.moveTo(POS_DOOR_OPEN);
  if (stepperDoor.distanceToGo() == 0 && stepperDoor.currentPosition() == POS_DOOR_OPEN) {
    currentState = DUMPING_IN;
  }
#endif
}

void handleDumpingIn() {
#if SIMULATION
  // Anggap push sudah masuk
  stepperPush.setCurrentPosition(POS_PUSH_IN);
  actionStartTime = millis();
  currentState = DUMPING_TRASH;
#else
  stepperPush.moveTo(POS_PUSH_IN);
  if (stepperPush.distanceToGo() == 0 && stepperPush.currentPosition() == POS_PUSH_IN) {
    actionStartTime = millis();
    currentState = DUMPING_TRASH;
  }
#endif
}


void handleDumpingTrash() {
  digitalWrite(RELAY_WEIGHING_MOTOR, HIGH);
  weighingMotorOn = true;

  if (millis() - actionStartTime >= DUMP_DURATION_MS) {
    digitalWrite(RELAY_WEIGHING_MOTOR, LOW);
    weighingMotorOn = false;
    currentWeightKg = 0;
    currentState = DUMPING_OUT;
  }
}

void handleDumpingOut() {
#if SIMULATION
  stepperPush.setCurrentPosition(POS_PUSH_OUT);
  currentState = CLOSING_MAIN_DOOR;
#else
  stepperPush.moveTo(POS_PUSH_OUT);
  if (stepperPush.distanceToGo() == 0 && stepperPush.currentPosition() == POS_PUSH_OUT) {
    currentState = CLOSING_MAIN_DOOR;
  }
#endif
}


void handleClosingMainDoor() {
#if SIMULATION
  stepperDoor.setCurrentPosition(POS_DOOR_CLOSE);
  tareScale();
  currentState = OPENING_BURN_DOOR;
#else
  stepperDoor.moveTo(POS_DOOR_CLOSE);
  if (stepperDoor.distanceToGo() == 0 && stepperDoor.currentPosition() == POS_DOOR_CLOSE) {
    tareScale();
    currentState = OPENING_BURN_DOOR;
  }
#endif
}

// 6. Buka Pintu Burner
void handleOpeningBurnDoor() {
#if SIMULATION
  stepperBurnDoor.setCurrentPosition(POS_BDOOR_OPEN);
  currentState = BURNER_IN;
#else
  stepperBurnDoor.moveTo(POS_BDOOR_OPEN);
  if (stepperBurnDoor.distanceToGo() == 0 && stepperBurnDoor.currentPosition() == POS_BDOOR_OPEN) {
    currentState = BURNER_IN;
  }
#endif
}


void handleBurnerIn() {
#if SIMULATION
  stepperBurner.setCurrentPosition(POS_BURNER_IN);
  currentState = IGNITING;
#else
  stepperBurner.moveTo(POS_BURNER_IN);
  if (stepperBurner.distanceToGo() == 0 && stepperBurner.currentPosition() == POS_BURNER_IN) {
    currentState = IGNITING;
  }
#endif
}

void handleIgniting() {
  // Ignition Sequence
  if (!isPriming) {
    digitalWrite(RELAY_IGNITION, LOW); // Trigger Spark
    isPriming = true;
    ignitionDelayStart = millis();

    digitalWrite(RELAY_BURNER, HIGH);  // Fuel ON
    burnerActive = true;

    totalBatchCount++;
    computeMetrics();
  }
}


void handleBurning() {
  // Kalau burner sudah dimatikan oleh checkBurnerTimer -> lanjut
  if (!burnerActive) {
    burnTimerStarted = false;  // reset flag untuk siklus berikutnya
    currentState = BURNER_OUT;
  }
}

void handleBurnerOut() {
#if SIMULATION
  stepperBurner.setCurrentPosition(POS_BURNER_OUT);
  currentState = CLOSING_BURN_DOOR;
#else
  stepperBurner.moveTo(POS_BURNER_OUT);
  if (stepperBurner.distanceToGo() == 0 && stepperBurner.currentPosition() == POS_BURNER_OUT) {
    currentState = CLOSING_BURN_DOOR;
  }
#endif
}


void handleClosingBurnDoor() {
#if SIMULATION
  stepperBurnDoor.setCurrentPosition(POS_BDOOR_CLOSE);
  if (autoModeEnabled) {
    currentState = WEIGHING;
  } else {
    currentState = IDLE;
  }
#else
  stepperBurnDoor.moveTo(POS_BDOOR_CLOSE);
  if (stepperBurnDoor.distanceToGo() == 0 && stepperBurnDoor.currentPosition() == POS_BDOOR_CLOSE) {
    if (autoModeEnabled) {
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

//void checkBurnerTimer() {
//  if (!burnerActive || currentState != BURNING) return;
//
//  unsigned long durationMS = burnerActiveTimeSec * 1000UL;
//  if (millis() - burnerStartTime >= durationMS) {
//    burnerActive = false;
//    digitalWrite(RELAY_BURNER, LOW);
//  }
//}

void readSensors() {
  if (currentState == FAULT) return;

#if SIMULATION
  if (currentState == BURNING) {
    // Naik sampai setpoint saja
    if (currentTempC < tempSetpoint) {
      currentTempC += 50;
      if (currentTempC > tempSetpoint) currentTempC = tempSetpoint;
    }
    // Kalau mau simulasi overshoot dikit, boleh tambahkan logika lain di sini
  } else {
    // Di luar BURNING, suhu turun pelan
    if (currentTempC > 25) {          // suhu ambient
      currentTempC -= 50;
      if (currentTempC < 25) currentTempC = 25;
    }
  }

  if (currentState == WEIGHING) currentWeightKg += 1;
#else
  currentTempC = temperature->readThermocouple(CELSIUS);
  if (scale.is_ready()) currentWeightKg = scale.get_units(5);
#endif

  // Overheat Check
  if (currentTempC >= tempSetpoint) { // Pakai nilai dari setting
    // Optional: Masuk mode FAULT atau Pause Burner
    // currentState = FAULT;
  }
}

void updateStateProgressBar() {
  uint8_t val = 0;   // 0–100

  switch (currentState) {
    // 1) WEIGHING: berdasarkan berat
    case WEIGHING: {
      if (minBatchWeight > 0) {
        float ratio = currentWeightKg / minBatchWeight;
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

    // 4) DUMPING_TRASH: berdasarkan waktu DUMP_DURATION_MS
    case DUMPING_TRASH: {
      unsigned long elapsed = millis() - actionStartTime;
      if (elapsed > DUMP_DURATION_MS) elapsed = DUMP_DURATION_MS;
      val = (uint8_t)(elapsed * 100UL / DUMP_DURATION_MS);
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

    // 6) IGNITING: berdasarkan waktu priming (IGNITION_DELAY_MS)
    case IGNITING: {
      if (isPriming) {
        unsigned long elapsed = millis() - ignitionDelayStart;
        if (elapsed > IGNITION_DELAY_MS) elapsed = IGNITION_DELAY_MS;
        val = (uint8_t)(elapsed * 100UL / IGNITION_DELAY_MS);
      } else {
        val = 0;
      }
      break;
    }

    // 7) BURNING: berdasarkan waktu burnerActiveTimeSec
    case BURNING: {
      unsigned long durationMS = burnerActiveTimeSec * 1000UL;
      unsigned long elapsed    = millis() - burnerStartTime;
      if (elapsed > durationMS) elapsed = durationMS;
      val = (uint8_t)(elapsed * 100UL / durationMS);
      break;
    }

    // default untuk IDLE & FAULT
    default:
      val = 0;
      break;
  }

  jStat.setValue(val);
}

void computeMetrics() {
  // Fuel Calculation: (Liter/Hour / 3600) * SecondsBurned * TotalBatches
  float literPerSec = LITERS_PER_HOUR_BURNER / 3600.0;
  float fuelPerBatch = literPerSec * burnerActiveTimeSec;
  solarVolumeUsed_L = totalBatchCount * fuelPerBatch;

  totalConsumedKg = totalBatchCount * minBatchWeight;
}

void updateHMI() {
  char buf[20]; // [FIX] Deklarasi dipindah ke paling atas

  // State String Mapping
//  const char* stateStr = "IDLE";
//  if (currentState == WEIGHING) stateStr = "WEIGH";
//  else if (currentState == BURNING) stateStr = "BURN";
//  else if (currentState == FAULT) stateStr = "FAULT";

//  if (currentState != lastState) {
//    tStat.setText(stateStr);
//    lastState = currentState;
//  }

  // Update Realtime Burn Timer (MM:SS)
  if (currentState == BURNING) {
     if (!burnTimerStarted) {
        // Masih naik suhu, timer belum mulai
        tTime.setText("PREHEAT");   // atau "PREH"
     } else {
        long elapsed = (millis() - burnerStartTime) / 1000;
        long remain  = burnerActiveTimeSec - elapsed;
        if (remain < 0) remain = 0;
        
        char timeBuf[10];
        sprintf(timeBuf, "%02ld:%02ld", remain/60, remain%60);
        tTime.setText(timeBuf);
     }
  } else {
     tTime.setText("--:--");
  }


  // Update Sensors to Dashboard
  dtostrf(currentTempC, 3, 0, buf);
  tTemp.setText(buf); // Suhu bulat

  dtostrf(currentWeightKg, 4, 1, buf);
  tWeight.setText(buf);

  // Update BBM
  computeMetrics(); // Refresh metric
  dtostrf(solarVolumeUsed_L, 4, 1, buf);
  tFuel.setText(buf);
  //Serial.println("BBM OK");

  // Update Totals
  snprintf(buf, sizeof(buf), "%lu", (unsigned long)totalBatchCount);
  tBatch.setText(buf);
  //Serial.println("TOTAL BATCH OK");
  
  // Update Sensor Manual Page
  dtostrf(totalConsumedKg, 4, 1, buf);
  tTotWeight.setText(buf);
  //Serial.println("TOTAL WEIGHT OK");
  
  dtostrf(currentTempC, 3, 0, buf); 
  tManTemp.setText(buf);    
  //Serial.println("CURRENT TEMP OK");
   
  dtostrf(currentWeightKg, 4, 1, buf);
  tManWeight.setText(buf);
  if (currentState != lastState) {
    switch (currentState) {
      case IDLE:          tStat.setText("IDLE");       break;
      case WEIGHING:      tStat.setText("WEIGHING");   break;
      case OPENING_MAIN_DOOR: tStat.setText("D.O"); break;
      case DUMPING_IN:     tStat.setText("F.IN");    break;
      case DUMPING_TRASH: tStat.setText("DUMP");    break;
      case DUMPING_OUT:    tStat.setText("F. OUT");   break;
      case CLOSING_MAIN_DOOR: tStat.setText("DOOR CLOSE"); break;
      
      case OPENING_BURN_DOOR: tStat.setText("B-DOOR OPEN"); break;
      case BURNER_IN:     tStat.setText("BURNER IN");  break;
      case IGNITING:      tStat.setText("IGNITING");   break;
      case BURNING:       tStat.setText("BURNING");    break;
      case BURNER_OUT:    tStat.setText("BURNER OUT"); break;
      case CLOSING_BURN_DOOR: tStat.setText("B-DOOR CLOSE"); break;
      
      case FAULT:         tStat.setText("FAULT");      break;
    }
    lastState = currentState;
  }
  updateStateProgressBar();

}


/* ====== MAIN (Refactored) ====== */
void loop() {
  now = millis();
  nexLoop(nex_listen_list);

  // Hardware Run
  if (currentState != FAULT) {
  stepperDoor.run();
    stepperPush.run();
    stepperAsh.run();
    stepperBurner.run();
    stepperBurnDoor.run();
  }

  // Logic Blocks
  checkIgnitionPriming();
  checkStepperFlags();
  checkBurnerTimer();

  // Periodic Update
  if (now - lastSensorMillis >= SENSOR_INTERVAL) {
    lastSensorMillis = now;
    readSensors();
    updateHMI();
  }

  runStateMachine();
}
/* ====== END OF FILE ====== */
