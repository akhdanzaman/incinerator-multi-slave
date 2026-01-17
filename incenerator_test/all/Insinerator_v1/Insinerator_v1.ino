// *EDIT Insinerator IoT v5 - Firmware untuk Arduino Mega (Konsep Batch)

#define SIMULATION 1    // 1 = tanpa hardware, 0 = dengan hardware

/* ====== INCLUDES ====== */
#include <Arduino.h>
#include <AccelStepper.h>
#include "HX711.h"
#include <SPI.h>
#include <MAX31856.h> 
#include <Nextion.h>
#include <EEPROM.h>

// ---------- Config ----------
HardwareSerial &PCSerial   = Serial;   // To your PC (Serial Monitor)
HardwareSerial &NextionSer = Serial2;  // To Nextion (TX2/RX2)

/* ====== EEPROM ADDRESS ====== */
#define EEPROM_ADDR_WEIGHT 0
#define EEPROM_ADDR_TIME   4
#define EEPROM_ADDR_SCALE  8

/* ====== PIN CONFIG ====== */
// Sensor Pins
const int HX711_DT      = 38;
const int HX711_SCK     = 49;
const int MAX31856_CS   = 53;
const int MAX31856_SCK  = 52;
const int MAX31856_SDI  = 51;
const int MAX31856_SDO  = 50;

// Stepper Pins
const int STEP_PIN_DOOR = 2; 
const int DIR_PIN_DOOR  = 3; 
const int STEP_PIN_PUSH = 4; 
const int DIR_PIN_PUSH  = 5; 
const int ENABLE_PIN    = 4; 

const int STEP_PIN_ASH    = 11; 
const int DIR_PIN_ASH     = 12; 

const int STEP_PIN_BURN   = 22; 
const int DIR_PIN_BURN    = 23;

const int STEP_PIN_BDOOR  = 24;
const int DIR_PIN_BDOOR   = 25;


// AC Motor / Relay Pins
const int RELAY_FEEDER_MOTOR   = 8; 
const int RELAY_WEIGHING_MOTOR = 7; 
const int RELAY_BURNER         = 9;
const int RELAY_IGNITION       = 10;

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
bool burnerOn     = false;

unsigned long burnerDuration      = 0;         
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
  DUMPING_IN,        // [BARU] Memajukan conveyor
  DUMPING_TRASH,    // [BARU] Menyalakan conveyor
  DUMPING_OUT,       // [BARU] Memundurkan conveyor
  CLOSING_MAIN_DOOR,
  
  OPENING_BURN_DOOR,// [BARU] Buka pintu burner
  BURNER_IN,        // [BARU] Masukkan burner
  IGNITING, 
  BURNING, 
  BURNER_OUT,       // [BARU] Keluarkan burner
  CLOSING_BURN_DOOR,// [BARU] Tutup pintu burner
  
  FAULT 
};
SystemState currentState = IDLE;

int lastState = -1;

// Metrics & Sensor Data
float currentTempC = 0.0;
float currentWeightKg = 0.0;
unsigned long lastSensorMillis = 0;
const unsigned long SENSOR_INTERVAL = 500;

float minBatchWeight = 5.0;
uint32_t burnerOnTimeSec = 30;
const unsigned long DUMP_DURATION_MS = 10000;
unsigned long actionStartTime = 0;

float solarVolumeUsed_L = 0.0;
float totalConsumedKg = 0.0;
unsigned long totalBatchCount = 0;
const float LITERS_PER_HOUR_BURNER = 10.0;
float sfc_L_per_kg = 0.0;

const float OVERHEAT_TEMP = 1200.0;
float scale_factor = -7050.0;
long scale_zero = 0;

long remaining = 0;
long last_remaining = -1;


/* ====== NEXTION OBJECTS ====== */
// Home Page
NexText tHomeTemp         = NexText(0, 2, "tTemp");
NexText tHomeWeight       = NexText(0, 3, "tWeight");
NexText tHomeBurnerStatus = NexText(0, 4, "tBurner");
NexText tHomeState        = NexText(0, 5, "tHomeState");
NexDSButton btAuto        = NexDSButton(0, 7, "btAuto");
NexButton bToManual       = NexButton(0, 10, "bToManual"); 

// Page X 
NexText tAlarmList        = NexText(6, 2, "tAlarmList"); // Asumsi Page 6
NexText tSFC              = NexText(3, 2, "tSFC");       // Asumsi Page 3
NexText tConsumed         = NexText(3, 3, "tConsumed");

// Page X 
NexButton bStart          = NexButton(4, 2, "bStart"); 
NexButton bStop           = NexButton(4, 3, "bStop");
NexButton bTare           = NexButton(5, 5, "bTare"); 
NexNumber nBatchWeightSet = NexNumber(5, 6, "nBatch");
NexNumber nBurnTimeSet    = NexNumber(5, 7, "nBurnT");

// Page 5 (Manual Control)
NexButton bOpenDoor       = NexButton(5, 3, "bOpenDoor");
NexButton bCloseDoor      = NexButton(5, 4, "bCloseDoor");
NexButton bTurnOnConvey   = NexButton(5, 5, "bTurnOnConvey");
NexButton bTurnOffConvey  = NexButton(5, 6, "bTurnOffConvey");
NexButton bTurnOnPush     = NexButton(5, 15, "bTurnOnPush");
NexButton bTurnOffPush    = NexButton(5, 16, "bTurnOffPush");
NexButton bTurnOnBurner   = NexButton(5, 7, "bTurnOnBurner"); 
NexButton bStopEmergency  = NexButton(5, 19, "bStopEmergency"); 
NexButton bTimeUp         = NexButton(5, 11, "bTimeUp");
NexButton bTimeDown       = NexButton(5, 10, "bTimeDown");
NexButton bBackToDashboard= NexButton(5, 10, "bToDashboard");

NexText tDoorState        = NexText(5, 12, "tDoorState"); 
NexText tConveyState      = NexText(5, 13, "tConveyState"); 
NexText tPushState        = NexText(5, 17, "tPushState"); 
NexText tBurnerState      = NexText(5, 14, "tBurnerState"); 
NexText tBurningTime      = NexText(5, 9, "tBurningTime");

// Page 7 (Log View)
NexDSButton btLog         = NexDSButton(7, 3, "btLog"); 

NexTouch *nex_listen_list[] = {
  &btAuto, &bToManual, &bBackToDashboard,
  &bStart, &bStop, 
  &bTare, &nBatchWeightSet, &nBurnTimeSet,
  &bOpenDoor, &bCloseDoor, &bTurnOnConvey, &bTurnOffConvey, 
  &bTurnOnPush, &bTurnOffPush, &bTurnOnBurner, &bStopEmergency, 
  &bTimeUp, &bTimeDown,
  &btLog,
  NULL
};

/* ====== ALARM LOGGING & UTILITIES ====== */
void alarmLog(const String &msg) {
  Serial.print("[ALARM] ");
  Serial.println(msg);
  tAlarmList.setText(msg.c_str());
}

void emergencyStop() {
  // Matikan semua aktuator
  digitalWrite(RELAY_BURNER, LOW); 
  burnerOn = false;
  
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
  Serial.println("Taring scale...");
  if (currentState != FAULT) {
  #if !SIMULATION
    scale.tare(20);
    scale_zero = scale.get_units(10);
  #else
    Serial.println("skip tarescale");
    return;
  #endif
  }
  Serial.println("Tare complete.");
}

// ---------- Logger class ----------
class DualLogger : public Print {
public:
  DualLogger(HardwareSerial &pc, HardwareSerial &hmi)
    : _pc(pc), _hmi(hmi) {}

  void begin(unsigned long baudPC, unsigned long baudHMI) {
    _pc.begin(baudPC);
    _hmi.begin(baudHMI);
  }

  // Fungsi untuk Mengaktifkan/Mematikan Log ke Nextion
  void enableNextionLog(bool state) {
    _nextionEnabled = state;
    if (!state) {
        // Bersihkan buffer saat dimatikan
        _logBuffer = ""; 
        _lineBuffer = "";
    }
  }

  virtual size_t write(uint8_t b) override {
    // 1. SELALU kirim ke PC (Serial Monitor)
    _pc.write(b);

    // 2. Cek apakah Logger Nextion Aktif?
    if (!_nextionEnabled) return 1; 

    // --- Logika String (Hanya jalan jika Enabled) ---
    if (b == '\n') {
      sendLineToNextion(_lineBuffer);
      _lineBuffer = "";
    } else if (b >= 32 && b <= 126) { // Filter karakter aneh
       if (_lineBuffer.length() < 200) { 
         _lineBuffer += (char)b;
       }
    }
    return 1;
  }

  using Print::write;

private:
  HardwareSerial &_pc;
  HardwareSerial &_hmi;
  String _lineBuffer;
  String _logBuffer;
  
  bool _nextionEnabled = false; // Default OFF

  void sendLineToNextion(const String &line) {
    if (line.length() == 0) return;
    _logBuffer += line + "\r\n";
    
    const uint16_t maxLen = 600; 
    if (_logBuffer.length() > maxLen) {
      _logBuffer.remove(0, _logBuffer.length() - maxLen);
    }

    _hmi.print("log.txt=\"");
    for (unsigned int i = 0; i < _logBuffer.length(); i++) {
      char c = _logBuffer[i];
      if (c == '\"') _hmi.print("\\\""); 
      else _hmi.print(c);
    }
    _hmi.print("\"");
    endNextionCmd();
  }

  void endNextionCmd() {
    _hmi.write(0xFF);
    _hmi.write(0xFF);
    _hmi.write(0xFF);
  }
};

// ---------- Create global logger ----------
DualLogger Logger(PCSerial, NextionSer);
#define Serial Logger   // ⚠️ must be after creating Logger


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

void btLogPopCallback(void *ptr) {
    uint32_t state;
    btLog.getValue(&state);

    if (state == 1) {
        Logger.enableNextionLog(true);
        Serial.println("--- DEBUG LOG: ON ---"); 
    } else {
        Serial.println("--- DEBUG LOG: OFF ---");
        Logger.enableNextionLog(false);
    }
}

void btAutoPopCallback(void *ptr) {
  uint32_t dual_state;
  btAuto.getValue(&dual_state); // Baca status tombol (0 atau 1)

  if (dual_state == 1) {
    // --- JIKA TOMBOL DI-ON-KAN ---
    Serial.println("AUTO MODE: ENABLED");
    autoModeEnabled = true;

    // Jika sistem sedang diam (IDLE), langsung pancing untuk mulai
    if (currentState == IDLE) {
       Serial.println("Starting Sequence...");
       currentState = WEIGHING;
       tareScale();
    }
  } else {
    // --- JIKA TOMBOL DI-OFF-KAN ---
    Serial.println("AUTO MODE: DISABLED (Finishing cycle or Stopping)");
    autoModeEnabled = false;
  }
}

void bStopPopCallback(void *ptr) {
  Serial.println("State -> IDLE (Stop)");
  currentState = IDLE;
  emergencyStop();
}

void bTarePopCallback(void *ptr) {
  if (currentState == IDLE || currentState == WEIGHING) {
    tareScale();
    alarmLog("Scale Tared");
  }
}

void nBatchWeightSetPopCallback(void *ptr) {
  uint32_t number_val;
  nBatchWeightSet.getValue(&number_val);
  minBatchWeight = (float)number_val;
  EEPROM.put(EEPROM_ADDR_WEIGHT, minBatchWeight);
  Serial.print("New Batch Weight Saved: "); Serial.println(minBatchWeight);
}

void nBurnTimeSetPopCallback(void *ptr) {
  uint32_t number_val;
  nBurnTimeSet.getValue(&number_val);
  burnerOnTimeSec = number_val;
  EEPROM.put(EEPROM_ADDR_TIME, burnerOnTimeSec);
  Serial.print("New Burner Time Saved: "); Serial.println(burnerOnTimeSec);
}

// ====== Callback Control Manual =====

// --- Buka Pintu ---
void bOpenDoorPopCallback(void *ptr) {
  if (currentState != IDLE) {
      Serial.println("Access Denied: System is running!");
      return; 
  }
  Serial.println("Manual: Open Door");
  digitalWrite(ENABLE_PIN, LOW); 
  motorTurnCCW(stepperDoor, motorDoorBusy, 5000, 1000, 350);
  tDoorState.setText("Door State: OPEN");
}

// --- Tutup Pintu ---
void bCloseDoorPopCallback(void *ptr) {
  if (currentState != IDLE || autoModeEnabled == true) {
      Serial.println("Access Denied: Auto Mode is ON!");
      return; 
  }
  Serial.println("Close Door Button Pressed!");
  motorTurnCW(stepperDoor, motorDoorBusy, 5000, 2000, 350);
  tDoorState.setText("Door State: CLOSED");
}

// --- Push mendorong sampah ---
void bTurnOnPushPopCallback(void *ptr) {
  if (currentState != IDLE || autoModeEnabled == true) {
      Serial.println("Access Denied: Auto Mode is ON!");
      return; 
  }
  Serial.println("Turn ON Push Button Pressed!");
  motorTurnCCW(stepperPush, motorPushBusy, 5000, 2000, 350);
  tPushState.setText("Push State: ON");
}

// --- Push Stop mendorong sampah ---
void bTurnOffPushPopCallback(void *ptr) {
  if (currentState != IDLE || autoModeEnabled == true) {
      Serial.println("Access Denied: Auto Mode is ON!");
      return; 
  }
  Serial.println("Turn OFF Push Button Pressed!");
  motorTurnCW(stepperPush, motorPushBusy, 5000, 2000, 350);
  tPushState.setText("Push State: OFF");
}

// --- Conveyor ON ---
void bTurnOnConveyPopCallback(void *ptr) {
  if (currentState != IDLE || autoModeEnabled == true) {
      Serial.println("Access Denied: Auto Mode is ON!");
      return; 
  }
  Serial.println("Turn On Conveyor button pressed!");
  tConveyState.setText("Conveyor State: ON");
  digitalWrite(RELAY_WEIGHING_MOTOR, HIGH);
}

// --- Conveyor OFF ---
void bTurnOffConveyPopCallback(void *ptr) {
  if (currentState != IDLE || autoModeEnabled == true) {
      Serial.println("Access Denied: Auto Mode is ON!");
      return; 
  }
  Serial.println("Turn Off Conveyor button pressed!");
  tConveyState.setText("Conveyor State: OFF");
  digitalWrite(RELAY_WEIGHING_MOTOR, LOW);
}

// --- Set Burner Timer Up ---
void bTimeUpPopCallback(void *ptr) {
  if (currentState != IDLE || autoModeEnabled == true) {
      Serial.println("Access Denied: Auto Mode is ON!");
      return; 
  }
  burnerDuration += 1000;
  char buffer[10];
  sprintf(buffer, "%ld", burnerDuration/1000);
  tBurningTime.setText(buffer);
  Serial.println("Burning Time: "); Serial.println(burnerDuration/1000);
}

// --- Set Burner Timer Down ---
void bTimeDownPopCallback(void *ptr) {
  if (currentState != IDLE || autoModeEnabled == true) {
      Serial.println("Access Denied: Auto Mode is ON!");
      return; 
  }
  if (burnerDuration > 1000) burnerDuration -= 1000;
  char buffer[10];
  sprintf(buffer, "%ld", burnerDuration/1000);
  tBurningTime.setText(buffer);
  Serial.println("Burning Time: "); Serial.println(burnerDuration/1000);
}

// --- Turn On Burner ---
void bTurnOnBurnerPopCallback(void *ptr) {
  if (currentState != IDLE || autoModeEnabled == true) {
      Serial.println("Access Denied: Auto Mode is ON!");
      return; 
  }
  digitalWrite(RELAY_IGNITION, LOW);

  if (burnerActive) return;   // Jika burner sedang aktif, abaikan

  isPriming = true;
  ignitionDelayStart = millis(); // Catat waktu mulai priming

  burnerActive = true;
  burnerStartTime = millis();

  digitalWrite(RELAY_BURNER, HIGH);
  
  Serial.print("Burner ON for ");
  Serial.print(burnerDuration/1000);
  Serial.println(" seconds");
  tBurnerState.setText("Burner State: ON");
}

void bStopEmergencyPopCallback(void *ptr) {
  Serial.println("EMERGENCY STOP!");

  // ----- STOP DOOR -----
  stepperDoor.stop();
  stepperDoor.setCurrentPosition(stepperDoor.currentPosition());
  stepperDoor.move(0);              
  motorDoorBusy = false;
  tDoorState.setText("Door State: STOP");

  // ----- STOP Push -----
  stepperPush.stop();
  stepperPush.setCurrentPosition(stepperPush.currentPosition());
  stepperPush.move(0);
  motorPushBusy = false;
  tPushState.setText("Push State: STOP");

  // ----- STOP BURNER -----
  burnerActive = false;
  burnerStartTime = 0;
  digitalWrite(RELAY_BURNER, LOW);
  digitalWrite(RELAY_IGNITION, HIGH);
  tBurnerState.setText("Burner State: OFF");

  // ----- STOP CONVEYOR -----
  digitalWrite(RELAY_WEIGHING_MOTOR, LOW);
  tConveyState.setText("Conveyor State: OFF");

  Serial.println("ALL SYSTEMS STOPPED SAFELY!");
}


/* ====== SETUP ====== */
void setup() {
  Logger.begin(9600, 9600);
  delay(100);

  // --- EEPROM Init ---
  EEPROM.begin(512);
  EEPROM.get(EEPROM_ADDR_WEIGHT, minBatchWeight);
  EEPROM.get(EEPROM_ADDR_TIME, burnerOnTimeSec);
  EEPROM.get(EEPROM_ADDR_SCALE, scale_factor);
  
  if (isnan(minBatchWeight) || minBatchWeight < 0 || minBatchWeight > 1000) {
    minBatchWeight = 5.0;
    EEPROM.put(EEPROM_ADDR_WEIGHT, minBatchWeight);
  }
  if (burnerOnTimeSec > 86400 || burnerOnTimeSec == 0) {
    burnerOnTimeSec = 30;
    EEPROM.put(EEPROM_ADDR_TIME, burnerOnTimeSec);
  }
  if (isnan(scale_factor) || scale_factor == 0.0) {
    scale_factor = -7050.0;
    EEPROM.put(EEPROM_ADDR_SCALE, scale_factor);
  }
  Serial.print("Loaded Batch Weight: "); Serial.println(minBatchWeight);
  Serial.print("Loaded Burner Time: "); Serial.println(burnerOnTimeSec);

#if !SIMULATION
  SPI.begin();  

  // [WHIZOO] Inisialisasi MAX31856
  temperature = new MAX31856(MAX31856_SDI, MAX31856_SDO, MAX31856_CS, MAX31856_SCK);
  temperature->writeRegister(REGISTER_CR0, CR0_INIT);
  temperature->writeRegister(REGISTER_CR1, CR1_INIT);
  temperature->writeRegister(REGISTER_MASK, MASK_INIT);
  delay(250);
  
  float t = temperature->readThermocouple(CELSIUS);
  if ((int)t == NO_MAX31856 || (int)t == FAULT_VOLTAGE) {
     Serial.println("FATAL: Could not find MAX31856. Halting...");
     alarmLog("MAX31856 Error");
     currentState = FAULT;
  }
  if (currentState != FAULT) {
     Serial.println("MAX31856 Initialized");
  }

  // Loadcell
  scale.begin(HX711_DT, HX711_SCK);
  scale.set_scale(scale_factor);
  tareScale();
  
  // Stepper Init
  pinMode(ENABLE_PIN, OUTPUT);
  digitalWrite(ENABLE_PIN, HIGH);
// 1. Main Door
  stepperDoor.setMaxSpeed(1000); stepperDoor.setAcceleration(500);
  stepperDoor.setCurrentPosition(0); 

  // 2. Push / Feeder
  stepperPush.setMaxSpeed(1000); stepperPush.setAcceleration(500);
  stepperPush.setCurrentPosition(0); 

  // 3. Ash (Manual)
  stepperAsh.setMaxSpeed(800); stepperAsh.setAcceleration(400);
  stepperAsh.setCurrentPosition(0);

  // 4. Burner Mover
  stepperBurner.setMaxSpeed(1000); stepperBurner.setAcceleration(500);
  stepperBurner.setCurrentPosition(0);

  // 5. Burner Door
  stepperBurnDoor.setMaxSpeed(800); stepperBurnDoor.setAcceleration(400);
  stepperBurnDoor.setCurrentPosition(0);

  // Relay Init
  pinMode(RELAY_FEEDER_MOTOR, OUTPUT);
  pinMode(RELAY_WEIGHING_MOTOR, OUTPUT);
  pinMode(RELAY_BURNER, OUTPUT);
  
#else
  Serial.println("[SIM] MAX31856 & HX711 dilewati (no hardware).");
  currentTempC = 200.0;      // nilai dummy
  currentWeightKg = 10.0;     // dummy
#endif
  
  emergencyStop(); // Matikan semua

  // Nextion
  Serial2.begin(9600, SERIAL_8N1, 16, 17);
  Serial.println("done init");

  nexInit();
  
  // Callbacks
  btAuto.attachPop(btAutoPopCallback, &btAuto);
  bStop.attachPop(bStopPopCallback, &bStop);
  bTare.attachPop(bTarePopCallback, &bTare);
  nBatchWeightSet.attachPop(nBatchWeightSetPopCallback, &nBatchWeightSet);
  nBurnTimeSet.attachPop(nBurnTimeSetPopCallback, &nBurnTimeSet);

  // Callback Control Manual
  bOpenDoor.attachPop(bOpenDoorPopCallback, &bOpenDoor);
  bCloseDoor.attachPop(bCloseDoorPopCallback, &bCloseDoor);
  bTurnOnConvey.attachPop(bTurnOnConveyPopCallback, &bTurnOnConvey);
  bTurnOffConvey.attachPop(bTurnOffConveyPopCallback, &bTurnOffConvey);
  bTurnOnPush.attachPop(bTurnOnPushPopCallback, &bTurnOnPush);
  bTurnOffPush.attachPop(bTurnOffPushPopCallback, &bTurnOffPush);
  bTurnOnBurner.attachPop(bTurnOnBurnerPopCallback, &bTurnOnBurner);
  bStopEmergency.attachPop(bStopEmergencyPopCallback, &bStopEmergency);
  bTimeUp.attachPop(bTimeUpPopCallback, &bTimeUp);
  bTimeDown.attachPop(bTimeDownPopCallback, &bTimeDown);

  btLog.attachPop(btLogPopCallback, &btLog);

  if (currentState != FAULT) {
    Serial.println("System initialized. State: IDLE");
    currentState = IDLE;
  }
  Serial.println("System Booting...");
}

/* ====== MAIN LOOP ====== */
void loop() {
  now = millis(); 

  nexLoop(nex_listen_list);
  if (currentState != FAULT) {
        stepperDoor.run();
        stepperPush.run();
        stepperAsh.run();
        stepperBurner.run();
        stepperBurnDoor.run();
    }

  // --- A. Ignition Priming ---
  if (isPriming) {
        if (millis() - ignitionDelayStart >= IGNITION_DELAY_MS) {
            digitalWrite(RELAY_IGNITION, HIGH);
            isPriming = false; 
            tBurnerState.setText("Ignition State: Off");
            Serial.println("Ignition Triggered! Priming complete.");
        } else {
             // Opsional: Tampilkan countdown di Serial atau Nextion saat priming
            long remaining = (IGNITION_DELAY_MS - (millis() - ignitionDelayStart)) / 1000;
            
            if (remaining != last_remaining) {    
                Serial.print("Priming countdown: ");
                Serial.println(remaining);
            
                last_remaining = remaining;   // update nilai terakhir
            }
        }
    }

  if (motorDoorBusy && stepperDoor.distanceToGo() == 0) {
    motorDoorBusy = false;
  }
  
  if (motorPushBusy && stepperPush.distanceToGo() == 0) {
    motorPushBusy = false;
  }  

  // --- B. Timer Burner (Manual) ---
  if (burnerActive && currentState == BURNING) {
      unsigned long currentMillis = millis();
      
      // Cek apakah waktu habis
      if (currentMillis - burnerStartTime >= burnerDuration) {
          burnerActive = false;
          digitalWrite(RELAY_BURNER, LOW);
          Serial.println("Burner OFF (Manual Timer finished)");
          tBurnerState.setText("Burner State: OFF");
      } 
      // Update Countdown di Layar (Setiap 1 detik)
      else if (currentMillis - lastCountdownUpdate >= 1000) {
          lastCountdownUpdate = currentMillis;
          
          // Hitung sisa waktu
          unsigned long elapsed = currentMillis - burnerStartTime;
          long remaining = (burnerDuration - elapsed) / 1000;
          if (remaining < 0) remaining = 0;
          
          // Update ke Nextion
          char buffer[30];
          sprintf(buffer, "Burner ON (%ld s)", remaining);
          tBurnerState.setText(buffer);
          
          Serial.print("Burner countdown: ");
          Serial.println(remaining);
      }
  }
  
  // --- C. Sensor & HMI Update ---
  if (now - lastSensorMillis >= SENSOR_INTERVAL) {
    lastSensorMillis = now;
    readSensors();
    updateHMI();
  }

  // --- D. State Machine ---
  switch (currentState) {
    case IDLE:              handleIdle(); break;
    case WEIGHING:          handleWeighing(); break;
    case OPENING_MAIN_DOOR: handleOpeningMainDoor(); break;
    
    case DUMPING_IN:         handleDumpingIn(); break;      // [BARU]
    case DUMPING_TRASH:     handleDumpingTrash(); break;  // [BARU]
    case DUMPING_OUT:        handleDumpingOut(); break;     // [BARU]
    
    case CLOSING_MAIN_DOOR: handleClosingMainDoor(); break;

    case OPENING_BURN_DOOR: handleOpeningBurnDoor(); break; // [BARU]
    case BURNER_IN:         handleBurnerIn(); break;        // [BARU]
    
    case IGNITING:          handleIgniting(); break;
    case BURNING:           handleBurning(); break;
    
    case BURNER_OUT:        handleBurnerOut(); break;       // [BARU]
    case CLOSING_BURN_DOOR: handleClosingBurnDoor(); break; // [BARU]
    
    case FAULT:             handleFault(); break;
  }
}

/* ====== STATE HANDLERS ====== */

void handleIdle() {
  emergencyStop(); // Ensure everything safe
  
  // Door closing logic if redundant check needed
  if (stepperDoor.currentPosition() != POS_DOOR_CLOSE) {
    digitalWrite(ENABLE_PIN, LOW); 
    stepperDoor.moveTo(POS_DOOR_CLOSE);
  } else {
    digitalWrite(ENABLE_PIN, HIGH); 
  }
}

void handleWeighing() {
  digitalWrite(ENABLE_PIN, HIGH); // Disable Steppers
  
  // Nyalakan Conveyor Luar (Feeder)
  digitalWrite(RELAY_FEEDER_MOTOR, HIGH); feederMotorOn = true;
  digitalWrite(RELAY_WEIGHING_MOTOR, LOW); weighingMotorOn = false;

  if (currentWeightKg >= minBatchWeight) {
    Serial.println("Batch weight reached.");
    digitalWrite(RELAY_FEEDER_MOTOR, LOW); feederMotorOn = false;
    currentState = OPENING_MAIN_DOOR;
  }
}

void handleOpeningMainDoor() {
  digitalWrite(ENABLE_PIN, LOW); 
  stepperDoor.moveTo(POS_DOOR_OPEN);
  if (stepperDoor.distanceToGo() == 0 && stepperDoor.currentPosition() == POS_DOOR_OPEN) {
    Serial.println("Door Open. Moving Feeder IN.");
    currentState = DUMPING_IN;
  }
}

void handleDumpingIn() {
    stepperPush.moveTo(POS_PUSH_IN);
    if (stepperPush.distanceToGo() == 0 && stepperPush.currentPosition() == POS_PUSH_IN) {
        Serial.println("Feeder In Position. Dumping Trash...");
        actionStartTime = millis(); // Catat waktu mulai dumping
        currentState = DUMPING_TRASH;
    }
}

void handleDumpingTrash() {
    digitalWrite(RELAY_WEIGHING_MOTOR, HIGH);
    weighingMotorOn = true;

    // Tunggu sampai waktu dump habis
    if (millis() - actionStartTime >= DUMP_DURATION_MS) {
        digitalWrite(RELAY_WEIGHING_MOTOR, LOW);
        weighingMotorOn = false;
        Serial.println("Trash Dumped. Retracting Feeder...");
        currentState = DUMPING_OUT;
    }
}

void handleDumpingOut() {
    stepperPush.moveTo(POS_PUSH_OUT);
    if (stepperPush.distanceToGo() == 0 && stepperPush.currentPosition() == POS_PUSH_OUT) {
        Serial.println("Feeder Retracted. Closing Main Door...");
        currentState = CLOSING_MAIN_DOOR;
    }
}

void handleClosingMainDoor() {
  stepperDoor.moveTo(POS_DOOR_CLOSE);
  if (stepperDoor.distanceToGo() == 0 && stepperDoor.currentPosition() == POS_DOOR_CLOSE) {
    Serial.println("Main Door Closed. Opening Burner Door...");
    tareScale(); 
    currentState = OPENING_BURN_DOOR;
  }
}

// 6. Buka Pintu Burner
void handleOpeningBurnDoor() {
    stepperBurnDoor.moveTo(POS_BDOOR_OPEN);
    if (stepperBurnDoor.distanceToGo() == 0 && stepperBurnDoor.currentPosition() == POS_BDOOR_OPEN) {
        Serial.println("Burner Door Open. Inserting Burner...");
        currentState = BURNER_IN;
    }
}

void handleBurnerIn() {
    stepperBurner.moveTo(POS_BURNER_IN);
    if (stepperBurner.distanceToGo() == 0 && stepperBurner.currentPosition() == POS_BURNER_IN) {
        Serial.println("Burner In Position. Igniting...");
        currentState = IGNITING;
    }
}

void handleIgniting() {
  // Ignition Sequence
  digitalWrite(RELAY_IGNITION, LOW); // Trigger Spark
  isPriming = true;
  ignitionDelayStart = millis(); 
  
  burnerActive = true;
  burnerStartTime = millis();
  digitalWrite(RELAY_BURNER, HIGH);  // Fuel ON
  burnerOn = true;
  
  totalBatchCount++;
  computeMetrics(); 
  Serial.println("Ignition Sequence Started. Moving to BURNING.");
  currentState = BURNING;
}


void handleBurning() {
  // Logika mematikan burner ada di loop() utama (Timer)
  if (!burnerOn) {
    Serial.println("Burn Complete. Retracting Burner...");
    currentState = BURNER_OUT;
  }
}

void handleBurnerOut() {
    stepperBurner.moveTo(POS_BURNER_OUT);
    if (stepperBurner.distanceToGo() == 0 && stepperBurner.currentPosition() == POS_BURNER_OUT) {
        Serial.println("Burner Retracted. Closing Burner Door...");
        currentState = CLOSING_BURN_DOOR;
    }
}

void handleClosingBurnDoor() {
    stepperBurnDoor.moveTo(POS_BDOOR_CLOSE);
    if (stepperBurnDoor.distanceToGo() == 0 && stepperBurnDoor.currentPosition() == POS_BDOOR_CLOSE) {
        Serial.println("Burner Door Closed. Cycle Finished.");
        
        // Loop Logic
        if (autoModeEnabled) {
            Serial.println("Auto Mode ON: Next Batch.");
            currentState = WEIGHING; 
        } else {
            Serial.println("Auto Mode OFF: Idle.");
            currentState = IDLE;
        }
    }
}


void handleFault() {
  emergencyStop();
}

/* ====== READ SENSORS ====== */
void readSensors() {
  if (currentState == FAULT) return;

#if SIMULATION
  // Mode simulasi: isi nilai boongan saja
  currentTempC += 1.0;
  if (currentTempC > 900) currentTempC = 200;

  currentWeightKg += 0.5;
  if (currentWeightKg > minBatchWeight + 5) currentWeightKg = 0;

  return;
#endif

  // 1. Baca temperature (MAX31856)
  float tempReading = temperature->readThermocouple(CELSIUS);
  String faultMsg = "TC Fault: ";

  switch ((int) tempReading) {
    case FAULT_OPEN:
      faultMsg += "Open";
      alarmLog(faultMsg);
      currentState = FAULT;
      return;
    case FAULT_VOLTAGE:
      faultMsg += "Voltage (Short)";
      alarmLog(faultMsg);
      currentState = FAULT;
      return;
    case NO_MAX31856:
      faultMsg += "Not Connected";
      alarmLog(faultMsg);
      currentState = FAULT;
      return;
    default:
      currentTempC = tempReading;
      break;
  }

  // 2. Baca Loadcell
  if (scale.is_ready()) {
    if (currentState == WEIGHING || currentState == IDLE) {
      currentWeightKg = scale.get_units(5);
    }
  } else {
    alarmLog("Loadcell Not Ready");
  }

  // Cek Overheat
  if (currentTempC >= OVERHEAT_TEMP) {
    alarmLog("EMERGENCY: Overheat");
    currentState = FAULT;
    return;
  }
}

/* ====== COMPUTE METRICS ====== */
void computeMetrics() {
  if (totalBatchCount == 0 || minBatchWeight == 0) {
    sfc_L_per_kg = 0;
    totalConsumedKg = 0;
    solarVolumeUsed_L = 0;
    return;
  }
  float solarPerBatch = (LITERS_PER_HOUR_BURNER / 3600.0) * (float)burnerOnTimeSec;
  solarVolumeUsed_L = (float)totalBatchCount * solarPerBatch;
  totalConsumedKg = (float)totalBatchCount * minBatchWeight;
  sfc_L_per_kg = solarPerBatch / minBatchWeight;
}

/* ====== HMI UPDATE ====== */
void updateHMI() {
//  char buf[32];
//  dtostrf(currentTempC, 6, 1, buf);
//  tHomeTemp.setText(buf);
//  dtostrf(currentWeightKg, 6, 2, buf);
//  tHomeWeight.setText(buf);
//  if (burnerOn) tHomeBurnerStatus.setText("ON");
//  else tHomeBurnerStatus.setText("OFF");
//
//  dtostrf(sfc_L_per_kg, 5, 3, buf);
//  tSFC.setText(buf);
//  dtostrf(totalConsumedKg, 6, 2, buf);
//  tConsumed.setText(buf);

  // --- UPDATE STATE HANYA KALAU BERUBAH ---
//  if (currentState != lastState) {
//    switch (currentState) {
//      case IDLE:          tHomeState.setText("IDLE");       break;
//      case WEIGHING:      tHomeState.setText("WEIGHING");   break;
//      case OPENING_MAIN_DOOR: tHomeState.setText("DOOR OPEN"); break;
//      case DUMPING_IN:     tHomeState.setText("FEED IN");    break;
//      case DUMPING_TRASH: tHomeState.setText("DUMPING");    break;
//      case DUMPING_OUT:    tHomeState.setText("FEED OUT");   break;
//      case CLOSING_MAIN_DOOR: tHomeState.setText("DOOR CLOSE"); break;
//      
//      case OPENING_BURN_DOOR: tHomeState.setText("B-DOOR OPEN"); break;
//      case BURNER_IN:     tHomeState.setText("BURNER IN");  break;
//      case IGNITING:      tHomeState.setText("IGNITING");   break;
//      case BURNING:       tHomeState.setText("BURNING");    break;
//      case BURNER_OUT:    tHomeState.setText("BURNER OUT"); break;
//      case CLOSING_BURN_DOOR: tHomeState.setText("B-DOOR CLOSE"); break;
//      
//      case FAULT:         tHomeState.setText("FAULT");      break;
//    }
//    lastState = currentState;
//  }
}

/* ====== END OF FILE ====== */
