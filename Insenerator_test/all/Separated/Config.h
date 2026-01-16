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
