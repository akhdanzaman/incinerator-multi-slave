/*
 * CTRL - Arduino Mega (Manual Control Only)
 * - Reads commands from Serial (USB) AND Serial1 (ESP32 HMI)
 * - Drives 6 steppers (AccelStepper DRIVER mode)
 * - Controls 2 relays (burner + ignition) active LOW
 *
 * Commands (from Serial or Serial1):
 *   ESTOP
 *   MOVE
 *   SEL:0..5
 *   DIR:0/1
 *   SPD:<steps_per_second>      (nilai asli)
 *   STP:<steps>                 (nilai asli)
 *   FEED:0/1
 *   BURN:0/1
 *   IGN:0/1
 *
 * Debug from ESP32 (Serial1):
 *   DBG:anything here...
 *   -> will be printed to Serial Monitor as [ESP32] anything here...
 */

#include <Arduino.h>
#include <AccelStepper.h>

// ======= PIN MAP (EDIT THIS) =======
#define ENABLE_PIN 99  // TODO: ganti ke pin Mega yang valid. active LOW enable, HIGH = disable

#define RELAY_BURNER   34
#define RELAY_IGNITION 33   // active LOW

// Stepper pins (DRIVER)
#define STEP_PIN_DOOR   6
#define DIR_PIN_DOOR    7

#define STEP_PIN_PUSH   4
#define DIR_PIN_PUSH    5

#define STEP_PIN_BURNER 26
#define DIR_PIN_BURNER  27

#define STEP_PIN_BDOOR  28
#define DIR_PIN_BDOOR   29

#define STEP_PIN_ASH    3699   // TODO: ganti ke pin Mega yang valid
#define DIR_PIN_ASH     3799   // TODO: ganti ke pin Mega yang valid

#define STEP_PIN_MAINC  8
#define DIR_PIN_MAINC   9

// ======= STEPPERS =======
AccelStepper stDoor(AccelStepper::DRIVER, STEP_PIN_DOOR, DIR_PIN_DOOR);
AccelStepper stPush(AccelStepper::DRIVER, STEP_PIN_PUSH, DIR_PIN_PUSH);
AccelStepper stBurner(AccelStepper::DRIVER, STEP_PIN_BURNER, DIR_PIN_BURNER);
AccelStepper stBDoor(AccelStepper::DRIVER, STEP_PIN_BDOOR, DIR_PIN_BDOOR);
AccelStepper stAsh(AccelStepper::DRIVER, STEP_PIN_ASH, DIR_PIN_ASH);
AccelStepper stMainC(AccelStepper::DRIVER, STEP_PIN_MAINC, DIR_PIN_MAINC);

AccelStepper* motors[6] = { &stDoor, &stPush, &stBurner, &stBDoor, &stAsh, &stMainC };

// ======= LIMITS (nilai asli) =======
// Biar kamu ga “SPD:999999” lalu heran kenapa driver panas kayak hubungan.
static const uint32_t MOTOR_MAX_SPD[6] = { 10000, 20000, 15000,  8000,  7000, 14000 }; // steps/s
static const uint32_t MOTOR_MIN_SPD[6] = {  0,      0,     0,     0,     0,     0   }; // allow 0

static const long MOTOR_MAX_STP[6]     = { 2300,  32000,  3000,  1000,  1000,  3000 }; // steps (abs)

static uint8_t selectedMotor = 0;
static bool dirOpen = true;      // true=+ , false=-
static uint32_t spdAbs = 1000;   // steps/s (nilai asli)
static long stpAbs = 200;        // steps (nilai asli)
static bool feedContinuous = false;

inline void burnerOn()  { digitalWrite(RELAY_BURNER, LOW); }
inline void burnerOff() { digitalWrite(RELAY_BURNER, HIGH); }
inline void ignOn()     { digitalWrite(RELAY_IGNITION, LOW); }
inline void ignOff()    { digitalWrite(RELAY_IGNITION, HIGH); }

static inline uint32_t clampU32(uint32_t v, uint32_t lo, uint32_t hi) {
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

static inline long clampAbsSteps(uint8_t motorId, long steps) {
  long maxSt = MOTOR_MAX_STP[motorId];
  if (maxSt < 0) maxSt = -maxSt;
  if (steps < -maxSt) return -maxSt;
  if (steps >  maxSt) return  maxSt;
  return steps;
}

void emergencyStop() {
  burnerOff();
  ignOff();
  feedContinuous = false;

  for (int i = 0; i < 6; i++) motors[i]->stop();

  digitalWrite(ENABLE_PIN, HIGH); // disable driver
  stMainC.setSpeed(0);
}

void applySelectedMotorConfig() {
  AccelStepper* m = motors[selectedMotor];

  uint32_t maxS = MOTOR_MAX_SPD[selectedMotor];
  uint32_t minS = MOTOR_MIN_SPD[selectedMotor];
  uint32_t realSpd = clampU32(spdAbs, minS, maxS);

  digitalWrite(ENABLE_PIN, LOW);
  m->setMaxSpeed(realSpd);
  m->setAcceleration(100);
}

void doMoveOnce(Stream& out) {
  AccelStepper* m = motors[selectedMotor];
  applySelectedMotorConfig();

  long dist = clampAbsSteps(selectedMotor, stpAbs);
  if (!dirOpen) dist = -labs(dist); // pastikan negatif saat DIR=0
  else          dist =  labs(dist);

  m->move(dist);

  out.print("OK:MOVE motor=");
  out.print(selectedMotor);
  out.print(" spd=");
  out.print((unsigned long)clampU32(spdAbs, MOTOR_MIN_SPD[selectedMotor], MOTOR_MAX_SPD[selectedMotor]));
  out.print(" stp=");
  out.println(dist);
}

void setFeedContinuous(bool en, Stream& out) {
  feedContinuous = en;

  if (feedContinuous) {
    selectedMotor = 5; // main conveyor
    uint32_t maxS = MOTOR_MAX_SPD[5];
    uint32_t minS = MOTOR_MIN_SPD[5];
    uint32_t s = clampU32(spdAbs, minS, maxS);

    digitalWrite(ENABLE_PIN, LOW);
    stMainC.setMaxSpeed(s);
    stMainC.setSpeed(dirOpen ? (long)s : -(long)s);
  } else {
    stMainC.setSpeed(0);
  }

  out.print("OK:FEED=");
  out.println(feedContinuous ? 1 : 0);
}

// ======= COMMAND HANDLER =======
void handleLine(char* s, Stream& out, bool fromEsp32) {
  // trim end
  int n = strlen(s);
  while (n > 0 && (s[n-1] == '\r' || s[n-1] == ' ' || s[n-1] == '\t')) { s[n-1] = 0; n--; }
  if (n == 0) return;

  // Optional: debug line from ESP32
  if (fromEsp32 && strncmp(s, "DBG:", 4) == 0) {
    Serial.print("[ESP32] ");
    Serial.println(s + 4);
    return;
  }

  if (strcmp(s, "ESTOP") == 0) {
    emergencyStop();
    out.println("OK:ESTOP");
    return;
  }

  if (strcmp(s, "MOVE") == 0) {
    doMoveOnce(out);
    return;
  }

  // KEY:VAL
  char* colon = strchr(s, ':');
  if (!colon) { out.println("ERR:BAD_FORMAT"); return; }

  *colon = 0;
  const char* key = s;
  const char* val = colon + 1;

  if (strcmp(key, "SEL") == 0) {
    int v = atoi(val);
    if (v < 0 || v > 5) { out.println("ERR:SEL_RANGE"); return; }
    selectedMotor = (uint8_t)v;
    out.print("OK:SEL="); out.println(selectedMotor);
    return;
  }

  if (strcmp(key, "DIR") == 0) {
    int v = atoi(val);
    dirOpen = (v != 0);
    if (feedContinuous) setFeedContinuous(true, out); // update arah realtime
    out.print("OK:DIR="); out.println(dirOpen ? 1 : 0);
    return;
  }

  if (strcmp(key, "SPD") == 0) {
    long v = atol(val);
    if (v < 0) v = -v;
    spdAbs = (uint32_t)v;

    // kalau lagi FEED, update realtime
    if (feedContinuous) setFeedContinuous(true, out);

    out.print("OK:SPD=");
    out.println((unsigned long)spdAbs);
    return;
  }

  if (strcmp(key, "STP") == 0) {
    long v = atol(val);
    if (v < 0) v = -v;     // STP disimpan sebagai magnitudo, arah dari DIR
    stpAbs = v;

    out.print("OK:STP=");
    out.println((long)stpAbs);
    return;
  }

  if (strcmp(key, "FEED") == 0) {
    int v = atoi(val);
    setFeedContinuous(v != 0, out);
    return;
  }

  if (strcmp(key, "BURN") == 0) {
    int v = atoi(val);
    (v != 0) ? burnerOn() : burnerOff();
    out.print("OK:BURN="); out.println(v ? 1 : 0);
    return;
  }

  if (strcmp(key, "IGN") == 0) {
    int v = atoi(val);
    (v != 0) ? ignOn() : ignOff();
    out.print("OK:IGN="); out.println(v ? 1 : 0);
    return;
  }

  out.println("ERR:UNKNOWN_CMD");
}

// ======= LINE PARSER (for any Stream) =======
struct LineParser {
  char buf[80];
  uint8_t idx = 0;
  bool fromEsp32 = false;

  void poll(Stream& in, Stream& out) {
    while (in.available()) {
      char c = (char)in.read();

      if (c == '\n') {
        buf[idx] = 0;
        handleLine(buf, out, fromEsp32);
        idx = 0;
      } else {
        if (idx < sizeof(buf) - 1) buf[idx++] = c;
        else {
          idx = 0;
          out.println("ERR:LINE_TOO_LONG");
        }
      }
    }
  }
};

LineParser parserUSB;
LineParser parserESP;

void setup() {
  Serial.begin(9600);
  Serial1.begin(9600, SERIAL_8N1);

  parserUSB.fromEsp32 = false;
  parserESP.fromEsp32 = true;

  pinMode(ENABLE_PIN, OUTPUT);
  digitalWrite(ENABLE_PIN, HIGH); // disable

  pinMode(RELAY_BURNER, OUTPUT);
  pinMode(RELAY_IGNITION, OUTPUT);
  burnerOff();
  ignOff();

  for (int i = 0; i < 6; i++) {
    motors[i]->setMaxSpeed(1000);
    motors[i]->setAcceleration(200);
  }

  Serial.println("CTRL Manual-only ready (Serial + Serial1).");
  Serial1.println("OK:BOOT");
}

void loop() {
  // Command dari Serial Monitor
  parserUSB.poll(Serial, Serial);

  // Command dari ESP32
  parserESP.poll(Serial1, Serial1);

  // Run motor (hindari run() vs runSpeed tabrakan untuk motor 5)
  for (int i = 0; i < 6; i++) {
    if (i == 5 && feedContinuous) stMainC.runSpeed();
    else motors[i]->run();
  }
}
