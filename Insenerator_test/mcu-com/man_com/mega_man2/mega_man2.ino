/*
 * MEGA - STEPPERS + SERIAL STATE CONTROL + HMI AUTO CYCLE + HMI TELEMETRY
 * ----------------------------------------------------------------------
 * Tambahan utama:
 * - Limit switch per stepper sebagai titik 0 (homing)
 * - Homing non-blocking, bisa per-axis: HOME:<idx>
 *
 * Command baru:
 *   HOME            -> home semua axis
 *   HOME:<idx>      -> home 1 axis saja
 *                     idx mapping:
 *                       0=DOOR, 1=PUSH, 2=BURNER, 3=BDOOR, 4=ASH, 5=MAINC
 *
 * Catatan wiring limit switch:
 * - Default pakai INPUT_PULLUP (switch ke GND saat aktif)
 * - Aktif = LOW
 */

#include <Arduino.h>
#include <AccelStepper.h>

enum HomeStage { HS_IDLE, HS_SEEK, HS_BACKOFF, HS_RESEEK, HS_DONE, HS_FAIL };

struct HomingJob {
  AccelStepper* m;
  int lsPin;
  HomeStage st;
  long seekSpeed;
  long backoffSteps;
  long reseekSpeed;
  bool dirTowardSwitch;
  unsigned long startedMs;
  unsigned long timeoutMs;
};

// ================== STATE MACHINE ==================
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
  BURNING,
  BURNER_OUT,
  CLOSING_BURN_DOOR
};

SystemState state = IDLE;
bool autoRun = false;
bool stateCompleted = false;

bool autoCycleActive = false;

// ================== PIN CONFIG ==================
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

const int ENABLE_PIN = 22; // active LOW enable

// ===== LIMIT SWITCH PINS (ubah sesuai wiring kamu) =====
#define LS_DOOR_PIN   32
#define LS_PUSH_PIN   11
#define LS_BURN_PIN   34
#define LS_BDOOR_PIN  35
#define LS_ASH_PIN    36
#define LS_MAINC_PIN  37

// ================== STEPPERS ==================
AccelStepper stepperDoor(AccelStepper::DRIVER, STEP_PIN_DOOR, DIR_PIN_DOOR);
AccelStepper stepperPush(AccelStepper::DRIVER, STEP_PIN_PUSH, DIR_PIN_PUSH);
AccelStepper stepperAsh(AccelStepper::DRIVER, STEP_PIN_ASH, DIR_PIN_ASH);
AccelStepper stepperBurner(AccelStepper::DRIVER, STEP_PIN_BURN, DIR_PIN_BURN);
AccelStepper stepperBurnDoor(AccelStepper::DRIVER, STEP_PIN_BDOOR, DIR_PIN_BDOOR);
AccelStepper stepperMainConveyor(AccelStepper::DRIVER, STEP_PIN_MAINC, DIR_PIN_MAINC);

HomingJob homeJobs[6] = {
  { &stepperDoor,         LS_DOOR_PIN,  HS_IDLE,  800,  600,  300, true,  0, 12000 },
  { &stepperPush,         LS_PUSH_PIN,  HS_IDLE,  800,  600,  300, true,  0, 120000 },
  { &stepperBurner,       LS_BURN_PIN,  HS_IDLE,  800,  600,  300, true,  0, 12000 },
  { &stepperBurnDoor,     LS_BDOOR_PIN, HS_IDLE,  800,  600,  300, true,  0, 12000 },
  { &stepperAsh,          LS_ASH_PIN,   HS_IDLE,  800,  600,  300, true,  0, 12000 },
  { &stepperMainConveyor, LS_MAINC_PIN, HS_IDLE,  800,  600,  300, true,  0, 12000 }
};
// ================== POSITIONS (steps) ==================
const long POS_DOOR_OPEN   = 5800;
const long POS_DOOR_CLOSE  = 0;
const long POS_PUSH_IN     = 10500;
const long POS_PUSH_OUT    = 0;
const long POS_BURNER_IN   = 11500;
const long POS_BURNER_OUT  = 0;
const long POS_BDOOR_OPEN  = 1600;
const long POS_BDOOR_CLOSE = 0;

// ================== MOTOR TUNING ==================
struct MotorCfg { long speed; long accel; };
MotorCfg motor[6] = {
  {4000, 200},
  {2000, 300},
  {2000, 800},
  {2000, 600},
  {2000, 200},
  {2000, 200}
};

static inline void enableSteppers(bool en){
  digitalWrite(ENABLE_PIN, en ? LOW : HIGH);
}

void applyMotorCfg(){
  stepperDoor.setMaxSpeed(motor[0].speed);          stepperDoor.setAcceleration(motor[0].accel);
  stepperPush.setMaxSpeed(motor[1].speed);          stepperPush.setAcceleration(motor[1].accel);
  stepperBurner.setMaxSpeed(motor[2].speed);        stepperBurner.setAcceleration(motor[2].accel);
  stepperBurnDoor.setMaxSpeed(motor[3].speed);      stepperBurnDoor.setAcceleration(motor[3].accel);
  stepperAsh.setMaxSpeed(motor[4].speed);           stepperAsh.setAcceleration(motor[4].accel);
  stepperMainConveyor.setMaxSpeed(motor[5].speed);  stepperMainConveyor.setAcceleration(motor[5].accel);
}

// ================== TIMERS ==================
const unsigned long WEIGHING_MS  = 3000;
const unsigned long IGNITING_MS  = 3000;
const unsigned long BURNING_MS   = 5000;
unsigned long stateStartMs = 0;

// ================== SIMULATED TELEMETRY (HMI expects this) ==================
static float currentTempC     = 25.0f;
static float currentWeightKg  = 0.0f;
static float currentBatchVol  = 0.0f;
static float totalVolume      = 0.0f;

static unsigned long lastSensorMs   = 0;
static unsigned long lastProgMs     = 0;
static unsigned long lastRemTimeMs  = 0;

const unsigned long SENSOR_INTERVAL_MS = 500;
const unsigned long PROG_INTERVAL_MS   = 500;
const unsigned long REM_INTERVAL_MS    = 500;

// ================== STATE NAMES ==================
const char* stateName(SystemState s){
  switch(s){
    case IDLE: return "IDLE";
    case WEIGHING: return "WEIGHING";
    case OPENING_MAIN_DOOR: return "OPENING_MAIN_DOOR";
    case DUMPING_IN: return "DUMPING_IN";
    case DUMPING_OUT: return "DUMPING_OUT";
    case CLOSING_MAIN_DOOR: return "CLOSING_MAIN_DOOR";
    case OPENING_BURN_DOOR: return "OPENING_BURN_DOOR";
    case BURNER_IN: return "BURNER_IN";
    case IGNITING: return "IGNITING";
    case BURNING: return "BURNING";
    case BURNER_OUT: return "BURNER_OUT";
    case CLOSING_BURN_DOOR: return "CLOSING_BURN_DOOR";
  }
  return "?";
}

static inline bool moveDone(AccelStepper &m, long target){
  return (m.distanceToGo() == 0 && m.currentPosition() == target);
}

SystemState nextState(SystemState s){
  switch(s){
    case IDLE: return WEIGHING;
    case WEIGHING: return OPENING_MAIN_DOOR;
    case OPENING_MAIN_DOOR: return DUMPING_IN;
    case DUMPING_IN: return DUMPING_OUT;
    case DUMPING_OUT: return CLOSING_MAIN_DOOR;
    case CLOSING_MAIN_DOOR: return OPENING_BURN_DOOR;
    case OPENING_BURN_DOOR: return BURNER_IN;
    case BURNER_IN: return IGNITING;
    case IGNITING: return BURNING;
    case BURNING: return BURNER_OUT;
    case BURNER_OUT: return CLOSING_BURN_DOOR;
    case CLOSING_BURN_DOOR: return WEIGHING;
  }
  return IDLE;
}

static inline void logBoth(const String& s){
  Serial.println(s);
  Serial1.println(s);
}

// ================== LIMIT SWITCH / HOMING ==================
static const bool LS_ACTIVE_LOW = true;

static inline bool lsActive(int pin){
  int v = digitalRead(pin);
  return LS_ACTIVE_LOW ? (v == LOW) : (v == HIGH);
}

enum AxisIdx { AX_DOOR=0, AX_PUSH=1, AX_BURNER=2, AX_BDOOR=3, AX_ASH=4, AX_MAINC=5 };


bool homingActive = false;

static inline const char* axisName(int idx){
  switch(idx){
    case AX_DOOR: return "DOOR";
    case AX_PUSH: return "PUSH";
    case AX_BURNER: return "BURNER";
    case AX_BDOOR: return "BDOOR";
    case AX_ASH: return "ASH";
    case AX_MAINC: return "MAINC";
  }
  return "?";
}

static inline void homeJobReset(HomingJob &j){
  j.st = HS_IDLE;
  j.startedMs = 0;
  j.m->stop();
  j.m->setSpeed(0);
}

void startHomingAxis(int idx){
  if(idx < 0 || idx > 5) return;

  enableSteppers(true);

  HomingJob &j = homeJobs[idx];
  j.st = HS_SEEK;
  j.startedMs = millis();

  j.m->stop();
  j.m->setSpeed(j.dirTowardSwitch ? -abs(j.seekSpeed) : abs(j.seekSpeed));

  homingActive = true;

  String s = "HOME: start axis ";
  s += String(idx);
  s += " (";
  s += axisName(idx);
  s += ")";
  logBoth(s);
}

void startHomingAll(){
  enableSteppers(true);
  homingActive = true;
  for(int i=0;i<6;i++){
    homeJobs[i].st = HS_SEEK;
    homeJobs[i].startedMs = millis();
    homeJobs[i].m->stop();
    homeJobs[i].m->setSpeed(homeJobs[i].dirTowardSwitch ? -abs(homeJobs[i].seekSpeed) : abs(homeJobs[i].seekSpeed));
  }
  logBoth("HOME: start all axes");
}

bool anyHomingInProgress(){
  for(int i=0;i<6;i++){
    if(homeJobs[i].st == HS_SEEK || homeJobs[i].st == HS_BACKOFF || homeJobs[i].st == HS_RESEEK) return true;
  }
  return false;
}

bool anyHomingFailed(){
  for(int i=0;i<6;i++){
    if(homeJobs[i].st == HS_FAIL) return true;
  }
  return false;
}

bool allHomingDone(){
  for(int i=0;i<6;i++){
    if(homeJobs[i].st != HS_DONE && homeJobs[i].st != HS_IDLE) return false;
  }
  return true;
}

void serviceHomingAll(){
  if(!homingActive) return;

  unsigned long now = millis();

  for(int i=0;i<6;i++){
    HomingJob &j = homeJobs[i];
    AccelStepper &m = *j.m;

    if(j.st == HS_IDLE || j.st == HS_DONE || j.st == HS_FAIL) continue;

    // if(now - j.startedMs > j.timeoutMs){
    //   j.st = HS_FAIL;
    //   continue;
    // }

    switch(j.st){
      case HS_SEEK: {
        if(lsActive(j.lsPin)){
          m.setSpeed(0);
          m.stop();
          j.st = HS_BACKOFF;
          long backDir = j.dirTowardSwitch ? +abs(j.backoffSteps) : -abs(j.backoffSteps);
          m.move(backDir);
        } else {
          m.runSpeed();
        }
      } break;

      case HS_BACKOFF: {
        m.run();
        if(m.distanceToGo() == 0){
          j.st = HS_RESEEK;
          m.setSpeed(j.dirTowardSwitch ? -abs(j.reseekSpeed) : abs(j.reseekSpeed));
        }
      } break;

      case HS_RESEEK: {
        if(lsActive(j.lsPin)){
          m.setSpeed(0);
          m.stop();
          m.setCurrentPosition(0);
          j.st = HS_DONE;
        } else {
          m.runSpeed();
        }
      } break;

      default: break;
    }
  }

  if(!anyHomingInProgress()){
    homingActive = false;
    if(anyHomingFailed()){
      logBoth("HOME: FAIL (timeout or wiring/dir issue)");
    } else {
      logBoth("HOME: DONE");
    }
  }
}

// ================== HMI TELEMETRY HELPERS ==================
static inline uint8_t clampU8(int v){
  if(v < 0) return 0;
  if(v > 100) return 100;
  return (uint8_t)v;
}

int calculateProgress(){
  switch(state){
    case WEIGHING: {
      unsigned long el = millis() - stateStartMs;
      if(el > WEIGHING_MS) el = WEIGHING_MS;
      return (int)(el * 100UL / WEIGHING_MS);
    }

    case OPENING_MAIN_DOOR: {
      long pos = stepperDoor.currentPosition();
      if (pos < POS_DOOR_CLOSE) pos = POS_DOOR_CLOSE;
      if (pos > POS_DOOR_OPEN)  pos = POS_DOOR_OPEN;
      return (int)((pos - POS_DOOR_CLOSE) * 100L / (POS_DOOR_OPEN - POS_DOOR_CLOSE));
    }

    case CLOSING_MAIN_DOOR: {
      long pos = stepperDoor.currentPosition();
      if (pos < POS_DOOR_CLOSE) pos = POS_DOOR_CLOSE;
      if (pos > POS_DOOR_OPEN)  pos = POS_DOOR_OPEN;
      return (int)((POS_DOOR_OPEN - pos) * 100L / (POS_DOOR_OPEN - POS_DOOR_CLOSE));
    }

    case DUMPING_IN: {
      long pos = stepperPush.currentPosition();
      if (pos < POS_PUSH_OUT) pos = POS_PUSH_OUT;
      if (pos > POS_PUSH_IN)  pos = POS_PUSH_IN;
      return (int)((pos - POS_PUSH_OUT) * 100L / (POS_PUSH_IN - POS_PUSH_OUT));
    }

    case DUMPING_OUT: {
      long pos = stepperPush.currentPosition();
      if (pos < POS_PUSH_OUT) pos = POS_PUSH_OUT;
      if (pos > POS_PUSH_IN)  pos = POS_PUSH_IN;
      return (int)((POS_PUSH_IN - pos) * 100L / (POS_PUSH_IN - POS_PUSH_OUT));
    }

    case OPENING_BURN_DOOR: {
      long pos = stepperBurnDoor.currentPosition();
      if (pos < POS_BDOOR_CLOSE) pos = POS_BDOOR_CLOSE;
      if (pos > POS_BDOOR_OPEN)  pos = POS_BDOOR_OPEN;
      return (int)((pos - POS_BDOOR_CLOSE) * 100L / (POS_BDOOR_OPEN - POS_BDOOR_CLOSE));
    }

    case CLOSING_BURN_DOOR: {
      long pos = stepperBurnDoor.currentPosition();
      if (pos < POS_BDOOR_CLOSE) pos = POS_BDOOR_CLOSE;
      if (pos > POS_BDOOR_OPEN)  pos = POS_BDOOR_OPEN;
      return (int)((POS_BDOOR_OPEN - pos) * 100L / (POS_BDOOR_OPEN - POS_BDOOR_CLOSE));
    }

    case BURNER_IN: {
      long pos = stepperBurner.currentPosition();
      if (pos < POS_BURNER_OUT) pos = POS_BURNER_OUT;
      if (pos > POS_BURNER_IN)  pos = POS_BURNER_IN;
      return (int)((pos - POS_BURNER_OUT) * 100L / (POS_BURNER_IN - POS_BURNER_OUT));
    }

    case BURNER_OUT: {
      long pos = stepperBurner.currentPosition();
      if (pos < POS_BURNER_OUT) pos = POS_BURNER_OUT;
      if (pos > POS_BURNER_IN)  pos = POS_BURNER_IN;
      return (int)((POS_BURNER_IN - pos) * 100L / (POS_BURNER_IN - POS_BURNER_OUT));
    }

    case IGNITING: {
      unsigned long el = millis() - stateStartMs;
      if(el > IGNITING_MS) el = IGNITING_MS;
      return (int)(el * 100UL / IGNITING_MS);
    }

    case BURNING: {
      unsigned long el = millis() - stateStartMs;
      if(el > BURNING_MS) el = BURNING_MS;
      return (int)(el * 100UL / (BURNING_MS ? BURNING_MS : 1));
    }

    default:
      return 0;
  }
}

long calculateRemainingSec(){
  unsigned long now = millis();
  switch(state){
    case WEIGHING: {
      if(now - stateStartMs >= WEIGHING_MS) return 0;
      return (long)((WEIGHING_MS - (now - stateStartMs)) / 1000UL);
    }
    case IGNITING: {
      if(now - stateStartMs >= IGNITING_MS) return 0;
      return (long)((IGNITING_MS - (now - stateStartMs)) / 1000UL);
    }
    case BURNING: {
      if(now - stateStartMs >= BURNING_MS) return 0;
      return (long)((BURNING_MS - (now - stateStartMs)) / 1000UL);
    }
    default:
      return -1;
  }
}

void sendTelemetryToHMI(){
  unsigned long now = millis();

  if(state == BURNING){
    if(currentTempC < 1200.0f) currentTempC += 20.0f;
  } else {
    if(currentTempC > 25.0f) currentTempC -= 10.0f;
  }
  if(currentTempC < 0) currentTempC = 0;

  if(state == WEIGHING){
    currentWeightKg += 0.5f;
    if(currentWeightKg > 50.0f) currentWeightKg = 50.0f;
    currentBatchVol += 2.0f;
  }

  if(state == DUMPING_IN && stateCompleted){
    totalVolume += currentBatchVol;
    currentBatchVol = 0.0f;
    currentWeightKg = 0.0f;
  }

  if(now - lastSensorMs >= SENSOR_INTERVAL_MS){
    lastSensorMs = now;

    Serial1.print("DATA:");
    Serial1.print(currentTempC, 1);
    Serial1.print(",");
    Serial1.print(currentWeightKg, 1);
    Serial1.print(",");
    Serial1.print((int)state);
    Serial1.print(",");
    Serial1.print(currentBatchVol, 3);
    Serial1.print(",");
    Serial1.println(totalVolume, 3);
  }

  if(now - lastProgMs >= PROG_INTERVAL_MS){
    lastProgMs = now;
    int p = calculateProgress();
    Serial1.print("PROG:");
    Serial1.println(clampU8(p));
  }

  if(now - lastRemTimeMs >= REM_INTERVAL_MS){
    lastRemTimeMs = now;
    long rem = calculateRemainingSec();
    if(rem >= 0){
      Serial1.print("REM_TIME:");
      Serial1.println(rem);
    }
  }
}

// ================== Emergency stop helper ==================
void emergencyStop(){
  autoRun = false;
  autoCycleActive = false;

  homingActive = false;
  for(int i=0;i<6;i++) homeJobReset(homeJobs[i]);

  stepperDoor.stop(); stepperPush.stop(); stepperAsh.stop();
  stepperBurner.stop(); stepperBurnDoor.stop();
  stepperMainConveyor.stop();
  stepperMainConveyor.setSpeed(0);

  enableSteppers(false);

  stateCompleted = true;
  state = IDLE;

  logBoth("FAULT/ESTOP: drivers disabled, state=IDLE");
}

void enterState(SystemState s){
  state = s;
  stateStartMs = millis();
  stateCompleted = false;

  String msg = "ENTER ";
  msg += stateName(state);
  msg += " (";
  msg += String((int)state);
  msg += ")";
  logBoth(msg);

  switch(state){
    case IDLE:
      stateCompleted = true;
      stepperMainConveyor.setSpeed(0);
      break;

    case WEIGHING:
      stepperMainConveyor.setSpeed(motor[5].speed);
      break;

    case OPENING_MAIN_DOOR:
      stepperDoor.moveTo(POS_DOOR_OPEN);
      break;

    case DUMPING_IN:
      stepperPush.moveTo(POS_PUSH_IN);
      break;

    case DUMPING_OUT:
      stepperPush.moveTo(POS_PUSH_OUT);
      break;

    case CLOSING_MAIN_DOOR:
      stepperDoor.moveTo(POS_DOOR_CLOSE);
      break;

    case OPENING_BURN_DOOR:
      stepperBurnDoor.moveTo(POS_BDOOR_OPEN);
      break;

    case BURNER_IN:
      stepperBurner.moveTo(POS_BURNER_IN);
      break;

    case IGNITING:
      break;

    case BURNING:
      break;

    case BURNER_OUT:
      stepperBurner.moveTo(POS_BURNER_OUT);
      break;

    case CLOSING_BURN_DOOR:
      stepperBurnDoor.moveTo(POS_BDOOR_CLOSE);
      break;
  }
}

void serviceStateCompletion(){
  unsigned long now = millis();

  switch(state){
    case IDLE:
      stateCompleted = true;
      break;

    case WEIGHING:
      if(now - stateStartMs >= WEIGHING_MS){
        stepperMainConveyor.setSpeed(0);
        stateCompleted = true;
      }
      break;

    case OPENING_MAIN_DOOR:
      if(moveDone(stepperDoor, POS_DOOR_OPEN)) stateCompleted = true;
      break;

    case DUMPING_IN:
      if(moveDone(stepperPush, POS_PUSH_IN)) stateCompleted = true;
      break;

    case DUMPING_OUT:
      if(moveDone(stepperPush, POS_PUSH_OUT)) stateCompleted = true;
      break;

    case CLOSING_MAIN_DOOR:
      if(moveDone(stepperDoor, POS_DOOR_CLOSE)) stateCompleted = true;
      break;

    case OPENING_BURN_DOOR:
      if(moveDone(stepperBurnDoor, POS_BDOOR_OPEN)) stateCompleted = true;
      break;

    case BURNER_IN:
      if(moveDone(stepperBurner, POS_BURNER_IN)) stateCompleted = true;
      break;

    case IGNITING:
      if(now - stateStartMs >= IGNITING_MS) stateCompleted = true;
      break;

    case BURNING:
      if(now - stateStartMs >= BURNING_MS) stateCompleted = true;
      break;

    case BURNER_OUT:
      if(moveDone(stepperBurner, POS_BURNER_OUT)) stateCompleted = true;
      break;

    case CLOSING_BURN_DOOR:
      if(moveDone(stepperBurnDoor, POS_BDOOR_CLOSE)) stateCompleted = true;
      break;
  }
}

// ================== SERIAL COMMANDS ==================
String lineUSB;
String lineHMI;

SystemState parseStateByName(const String& name){
  for(int i=0;i<= (int)CLOSING_BURN_DOOR; i++){
    if(name.equalsIgnoreCase(stateName((SystemState)i))) return (SystemState)i;
  }
  return IDLE;
}

void printHelp(){
  Serial.println("Commands:");
  Serial.println("  HELP");
  Serial.println("  NEXT                 -> advance to next state (only if current done)");
  Serial.println("  RUN:0 or RUN:1       -> auto advance when done (infinite loop)");
  Serial.println("  S:<num>              -> jump to state by enum number");
  Serial.println("  STATE:<name>         -> jump by name (e.g. STATE:DUMPING_IN)");
  Serial.println("  POS?                 -> print current positions");
  Serial.println("  HOME                 -> homing semua axis");
  Serial.println("  HOME:<idx>            -> homing 1 axis (0..5). 0=DOOR 1=PUSH 2=BURNER 3=BDOOR 4=ASH 5=MAINC");
  Serial.println("HMI Commands (Serial1):");
  Serial.println("  CMD_AUTO_START       -> run 1 full cycle then stop (IDLE) (HOME all dulu)");
  Serial.println("  CMD_AUTO_STOP        -> stop auto cycle, go IDLE");
  Serial.println("  CMD_EMERGENCY_STOP   -> stop + disable drivers");
}

static inline int parseIntSafe(const String& s, int defV){
  if(s.length() == 0) return defV;
  bool neg = false;
  int i = 0;
  if(s[0] == '-') { neg = true; i = 1; }
  long v = 0;
  bool any = false;
  for(; i < (int)s.length(); i++){
    char c = s[i];
    if(c < '0' || c > '9') return defV;
    any = true;
    v = (v * 10) + (c - '0');
    if(v > 999999) break;
  }
  if(!any) return defV;
  return neg ? (int)(-v) : (int)v;
}

void handleCommand(String cmd){
  cmd.trim();
  if(cmd.length() == 0) return;

  if(cmd.equalsIgnoreCase("CMD_AUTO_START")){
    if(!homingActive && !anyHomingInProgress()) startHomingAll();
    autoCycleActive = true;
    autoRun = true;
    logBoth("AUTO: one-shot cycle ARMED (will start after HOME done)");
    return;
  }

  if(cmd.equalsIgnoreCase("CMD_AUTO_STOP")){
    autoRun = false;
    autoCycleActive = false;
    stepperMainConveyor.setSpeed(0);
    enterState(IDLE);
    logBoth("AUTO: STOP -> IDLE");
    return;
  }

  if(cmd.equalsIgnoreCase("CMD_EMERGENCY_STOP")){
    emergencyStop();
    return;
  }

  if(cmd.equalsIgnoreCase("HELP")){
    printHelp();
    return;
  }

  if(cmd.equalsIgnoreCase("HOME")){
    startHomingAll();
    return;
  }

  if(cmd.startsWith("HOME:")){
    String arg = cmd.substring(5);
    arg.trim();
    int idx = parseIntSafe(arg, -1);
    if(idx < 0 || idx > 5){
      Serial.println("ERR: HOME:<idx> must be 0..5");
      return;
    }
    startHomingAxis(idx);
    return;
  }

  if(cmd.equalsIgnoreCase("POS?")){
    Serial.print("Door="); Serial.print(stepperDoor.currentPosition());
    Serial.print(" Push="); Serial.print(stepperPush.currentPosition());
    Serial.print(" Burn="); Serial.print(stepperBurner.currentPosition());
    Serial.print(" BDoor="); Serial.print(stepperBurnDoor.currentPosition());
    Serial.print(" Ash="); Serial.print(stepperAsh.currentPosition());
    Serial.print(" MainC="); Serial.println(stepperMainConveyor.currentPosition());
    return;
  }

  if(cmd.equalsIgnoreCase("NEXT")){
    if(!stateCompleted){
      Serial.println("ERR: current state not completed yet");
      return;
    }
    enterState(nextState(state));
    return;
  }

  if(cmd.startsWith("RUN:")){
    int v = cmd.substring(4).toInt();
    autoRun = (v != 0);
    autoCycleActive = false;
    Serial.print("autoRun=");
    Serial.println(autoRun ? "1" : "0");
    return;
  }

  if(cmd.startsWith("S:")){
    int s = cmd.substring(2).toInt();
    if(s < 0 || s > (int)CLOSING_BURN_DOOR){
      Serial.println("ERR: state out of range");
      return;
    }
    enterState((SystemState)s);
    return;
  }

  if(cmd.startsWith("STATE:")){
    String nm = cmd.substring(6);
    nm.trim();
    SystemState s = parseStateByName(nm);
    if(!nm.equalsIgnoreCase("IDLE") && s == IDLE){
      Serial.println("WARN: unknown name, going to IDLE");
    }
    enterState(s);
    return;
  }

  Serial.println("ERR: unknown command (type HELP)");
}

static inline void consumeLineFrom(Stream &port, String &buf){
  while(port.available()){
    char c = (char)port.read();
    if(c == '\n'){
      handleCommand(buf);
      buf = "";
    } else if(c != '\r'){
      buf += c;
      if(buf.length() > 120) buf = "";
    }
  }
}

void processSerial(){
  consumeLineFrom(Serial, lineUSB);
  consumeLineFrom(Serial1, lineHMI);
}

// ================== LOOP HELPERS ==================
void runAllSteppers(){
  stepperDoor.run();
  stepperPush.run();
  stepperAsh.run();
  stepperBurner.run();
  stepperBurnDoor.run();
  stepperMainConveyor.run();
}

// ================== SETUP / LOOP ==================
void setup(){
  Serial.begin(9600);
  Serial1.begin(9600);

  pinMode(ENABLE_PIN, OUTPUT);
  enableSteppers(true);

  pinMode(LS_DOOR_PIN,  INPUT_PULLUP);
  pinMode(LS_PUSH_PIN,  INPUT_PULLUP);
  pinMode(LS_BURN_PIN,  INPUT_PULLUP);
  pinMode(LS_BDOOR_PIN, INPUT_PULLUP);
  pinMode(LS_ASH_PIN,   INPUT_PULLUP);
  pinMode(LS_MAINC_PIN, INPUT_PULLUP);

  applyMotorCfg();

  stepperDoor.setPinsInverted(true, false, false);

  stepperDoor.setCurrentPosition(0);
  stepperPush.setCurrentPosition(0);
  stepperBurner.setCurrentPosition(0);
  stepperBurnDoor.setCurrentPosition(0);
  stepperAsh.setCurrentPosition(0);
  stepperMainConveyor.setCurrentPosition(0);

  Serial.println("READY. Type HELP.");
  Serial1.println("READY(HMI). Send CMD_AUTO_START.");
  enterState(IDLE);
}

void loop(){
  processSerial();

  serviceHomingAll();

  if(!homingActive){
    if(state == WEIGHING){
      stepperMainConveyor.runSpeed();
    } else {
      stepperMainConveyor.setSpeed(0);
    }

    runAllSteppers();
    serviceStateCompletion();

    sendTelemetryToHMI();

    if(autoRun && autoCycleActive && state == IDLE && !anyHomingInProgress() && !anyHomingFailed()){
      enterState(WEIGHING);
    }

    if(autoRun && stateCompleted){
      if(autoCycleActive && state == CLOSING_BURN_DOOR){
        autoRun = false;
        autoCycleActive = false;
        enterState(IDLE);
        logBoth("AUTO: one-shot cycle DONE -> IDLE");
      } else {
        enterState(nextState(state));
      }
    }

  } else {
    sendTelemetryToHMI();
  }
}

/*
 * END
 */
