/*
 * HMI - ESP32 (Nextion Manual Page1 Only)
 * Serial (UART) -> Mega Serial3
 * Serial2 -> Nextion
 */

#include <Arduino.h>                                                                   

#include <Nextion.h>
// ======= NEXTION OBJECTS (Page1 only) =======
NexPage page1 = NexPage(1, 0, "page1");

NexDSButton btManDir  = NexDSButton(1, 2, "btManDir");
NexButton   bManDoor  = NexButton(1, 4, "bManDoor");
NexButton   bManBDoor = NexButton(1, 5, "bManBDoor");
NexButton   bManBurn  = NexButton(1, 6, "bManBurn");
NexButton   bManPush  = NexButton(1, 20, "bManFeed");   // (nama kamu bManFeed)
NexButton   bManAsh   = NexButton(1, 7, "bManAsh");
NexDSButton bManMainC = NexDSButton(1, 18, "bManCon");

NexSlider hManSpeed = NexSlider(1, 8, "hManSpeed");
NexSlider hManStep  = NexSlider(1, 9, "hManStep");

NexDSButton btManBurnRelay = NexDSButton(1, 19, "btManBurn");
NexDSButton btManIgn       = NexDSButton(1, 3, "btManIgn");
NexDSButton btManFeed      = NexDSButton(1, 99, "btManFeed"); // continuous conveyor
NexButton   bStop          = NexButton(1, 10, "b5");

// Listen list
NexTouch *nex_listen_list[] = {
  &page1,
  &btManDir,
  &bManDoor, &bManBDoor, &bManBurn, &bManPush, &bManAsh, &bManMainC,
  &hManSpeed, &hManStep,
  &btManBurnRelay, &btManIgn, &btManFeed,
  &bStop,
  NULL
};

// ======= STATE =======
static uint8_t selectedMotor = 0; // 0..5
static uint8_t spdPct = 50;       // 1..100
static uint8_t stpPct = 20;       // 1..100

static inline uint8_t clampPct(int v){
  if (v < 1) return 1;
  if (v > 100) return 100;
  return (uint8_t)v;
}

void ctrlSendLine(const char* s) {
  Serial.print(s);
  Serial.print('\n');
}

void ctrlSendKV(const char* k, int v) {
  Serial.print(k);
  Serial.print(':');
  Serial.print(v);
  Serial.print('\n');
}

void sendSelectedMotor(uint8_t id) {
  selectedMotor = id;
  ctrlSendKV("SEL", selectedMotor);
}

void sendSpeedPct(uint8_t pct) {
  spdPct = clampPct(pct);
  ctrlSendKV("SPD", spdPct);
}

void sendStepPct(uint8_t pct) {
  stpPct = clampPct(pct);
  ctrlSendKV("STP", stpPct);
}

void doMove() {
  ctrlSendLine("MOVE");
}

// ======= CALLBACKS =======
void btManDirPopCallback(void *ptr) {
  uint32_t v;
  btManDir.getValue(&v);     // 0/1
  ctrlSendKV("DIR", (int)v);
}

void hManSpeedPopCallback(void *ptr) {
  uint32_t v;
  if (!hManSpeed.getValue(&v)) return;
  sendSpeedPct((uint8_t)v);
}

void hManStepPopCallback(void *ptr) {
  uint32_t v;
  if (!hManStep.getValue(&v)) return;
  sendStepPct((uint8_t)v);
}

void bStopPopCallback(void *ptr) {
  ctrlSendLine("ESTOP");
}

// motor buttons: select + move once
void bManDoorPopCallback(void *ptr)  { sendSelectedMotor(0); doMove(); }
void bManPushPopCallback(void *ptr)  { sendSelectedMotor(1); doMove(); }
void bManBurnPopCallback(void *ptr)  { sendSelectedMotor(2); doMove(); }
void bManBDoorPopCallback(void *ptr) { sendSelectedMotor(3); doMove(); }
void bManAshPopCallback(void *ptr)   { sendSelectedMotor(4); doMove(); }

// main conveyor button (DSButton) = select motor 5 + move once (optional)
void bManMainCPopCallback(void *ptr) {
  // kalau kamu mau tombol ini jadi "move once"
  sendSelectedMotor(5);
  doMove();
}

// relay toggles
void btManIgnPopCallback(void *ptr) {
  uint32_t v; btManIgn.getValue(&v);
  ctrlSendKV("IGN", (int)v);
}
void btManBurnRelayPopCallback(void *ptr) {
  uint32_t v; btManBurnRelay.getValue(&v);
  ctrlSendKV("BURN", (int)v);
}

// feeder continuous toggle (runSpeed)
void btManFeedPopCallback(void *ptr) {
  uint32_t v; btManFeed.getValue(&v);
  // Pastikan motor=5 biar konsisten
  sendSelectedMotor(5);
  ctrlSendKV("FEED", (int)v);
}

void setup() {
  
  Serial.begin(9600, SERIAL_8N1, 16, 17);  // Debug serial & komunikasi ke CTRL
  nexSerial.begin(9600, SERIAL_8N1, 25, 26);
  nexInit();

  // Attach
  btManDir.attachPop(btManDirPopCallback, &btManDir);

  hManSpeed.attachPop(hManSpeedPopCallback, &hManSpeed);
  hManStep.attachPop(hManStepPopCallback, &hManStep);

  bManDoor.attachPop(bManDoorPopCallback, &bManDoor);
  bManPush.attachPop(bManPushPopCallback, &bManPush);
  bManBurn.attachPop(bManBurnPopCallback, &bManBurn);
  bManBDoor.attachPop(bManBDoorPopCallback, &bManBDoor);
  bManAsh.attachPop(bManAshPopCallback, &bManAsh);
  bManMainC.attachPop(bManMainCPopCallback, &bManMainC);

  btManIgn.attachPop(btManIgnPopCallback, &btManIgn);
  btManBurnRelay.attachPop(btManBurnRelayPopCallback, &btManBurnRelay);
  btManFeed.attachPop(btManFeedPopCallback, &btManFeed);

  bStop.attachPop(bStopPopCallback, &bStop);

  delay(300);
  // sync initial values to CTRL
  ctrlSendKV("SEL", selectedMotor);
  ctrlSendKV("SPD", spdPct);
  ctrlSendKV("STP", stpPct);

  // initial dir from button state
  uint32_t dv = 1;
  btManDir.getValue(&dv);
  ctrlSendKV("DIR", (int)dv);
}

void loop() {
  nexLoop(nex_listen_list);
  // kalau mau baca ACK dari Mega, kamu bisa tambahin parser kecil di sini.
}
