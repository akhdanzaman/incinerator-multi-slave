

// =========================
// hmi_nextion.h
// =========================
#pragma once
#include <Arduino.h>
#include <Nextion.h>
#include "pins.h"
#include "telemetry_store.h"
#include "tasks.h"

class HMI {
public:
  void begin();
  void loop();
  void render(const TelemetryStore& tlm);
  void setEventQueue(QueueHandle_t q) { qEvt_ = q; }
  void bindTelemetry(TelemetryStore* t) { tlm_ = t; }

private:
  QueueHandle_t qEvt_ = NULL;
  TelemetryStore* tlm_ = nullptr;

  // Nextion serial alias
  HardwareSerial& nexSer_ = Serial2;

  // ===== Pages =====
  NexPage page0 = NexPage(0, 0, "page0");
  NexPage page1 = NexPage(1, 0, "page1");
  NexPage page2 = NexPage(2, 0, "page2");
  NexPage page3 = NexPage(3, 0, "page3");

  // --- PAGE 0: DASHBOARD ---
  NexText tStat = NexText(0, 2, "tStat");
  NexText tTemp = NexText(0, 3, "tTemp");
  NexText tWeight = NexText(0, 4, "tWeight");
  NexText tTime = NexText(0, 5, "tTime");
  NexText tFuel = NexText(0, 6, "tFuel");
  NexSlider hFuel = NexSlider(0, 17, "hFuel");
  NexText tBatch = NexText(0, 7, "tBatch");
  NexText tTotWeight = NexText(0, 8, "tTotWeight");
  NexDSButton btAuto = NexDSButton(0, 9, "btAuto");
  NexButton bStop = NexButton(0, 13, "bStop");
  NexProgressBar jStat = NexProgressBar(0, 16, "jStat");
  NexButton b1 = NexButton(0, 15, "b1");
  NexText tWeightCal = NexText(0, 18, "tWeightCal");
  NexText tTempSetDash = NexText(0, 11, "t8");
  NexText tVolume = NexText(0, 12, "t9");
  NexText ttotVolume = NexText(0, 10, "tVolume");

  // --- PAGE 1: MANUAL ---
  NexDSButton btManDir = NexDSButton(1, 2, "btManDir");
  NexButton bManDoor = NexButton(1, 4, "bManDoor");
  NexButton bManBDoor = NexButton(1, 5, "bManBDoor");
  NexButton bManBurn = NexButton(1, 6, "bManBurn");
  NexButton bManPush = NexButton(1, 20, "bManFeed");
  NexButton bManAsh = NexButton(1, 7, "bManAsh");
  NexDSButton bManMainC = NexDSButton(1, 18, "bManCon");

  NexSlider hManSpeed = NexSlider(1, 8, "hManSpeed");
  NexSlider hManStep = NexSlider(1, 9, "hManStep");
  NexButton bman1 = NexButton(1, 21, "b4");
  NexButton bman2 = NexButton(1, 22, "b0");
  NexButton bman3 = NexButton(1, 23, "b4");
  NexButton bman4 = NexButton(1, 24, "b0");

  NexDSButton btManBurn = NexDSButton(1, 19, "btManBurn");
  NexDSButton btManIgn = NexDSButton(1, 3, "btManIgn");
  NexDSButton btManFeed = NexDSButton(1, 99, "btManFeed");
  NexDSButton btManWeigh = NexDSButton(1, 98, "btManWeigh");

  NexText tManTemp = NexText(1, 11, "tManTemp");
  NexText tManWeight = NexText(1, 12, "tManWeight");
  NexButton bManStop = NexButton(1, 10, "b5");
  NexButton b0 = NexButton(1, 15, "b0");

  // --- PAGE 2: SETTINGS ---
  NexText nTempSet = NexText(2, 21, "nTempSet");
  NexText nTimeSet = NexText(2, 20, "nTimeSet");
  NexText nWeightSet = NexText(2, 19, "nWeightSet");
  NexText nVolSet = NexText(2, 99, "nVOlLLWeightSet");

  NexButton bTempUp = NexButton(2, 8, "bTempUp");
  NexButton bTempDown = NexButton(2, 9, "bTempDown");
  NexButton bTimeUp = NexButton(2, 7, "bTimeUp");
  NexButton bTimeDown = NexButton(2, 6, "bTimeDown");
  NexButton bWeightUp = NexButton(2, 5, "bWeightUp");
  NexButton bWeightDown = NexButton(2, 4, "bWeightDown");

  NexDSButton bSelDoor = NexDSButton(2, 27, "bSelDoor");
  NexDSButton bSelBurn = NexDSButton(2, 29, "bSelBurn");
  NexDSButton bSelFeed = NexDSButton(2, 28, "bSelFeed");
  NexDSButton bSelBDoor = NexDSButton(2, 30, "bSelBDoor");
  NexDSButton bSelAsh = NexDSButton(2, 33, "bSelAsh");
  NexDSButton bSelMainC = NexDSButton(2, 32, "bSelMai");

  NexSlider hSpeed = NexSlider(2, 11, "hSpeed");
  NexSlider hStep = NexSlider(2, 10, "hStep");
  NexButton bSpeedUp = NexButton(2, 38, "b1");
  NexButton bSpeedDown = NexButton(2, 40, "b2");
  NexButton bStepUp = NexButton(2, 37, "b4");
  NexButton bStepDown = NexButton(2, 39, "b0");

  NexNumber nSpeed = NexNumber(2, 42, "nSpeed");
  NexNumber nStep = NexNumber(2, 41, "nStep");

  NexButton bSave = NexButton(2, 15, "bSave");
  NexButton bZero = NexButton(2, 12, "bZero");
  NexButton bSet = NexButton(2, 31, "bSet");
  NexButton bWall = NexButton(2, 35, "bWall");
  NexButton bSpeed = NexButton(2, 36, "bSpeed");

  // Listen list
  NexTouch* listen_[72];

  // Cache (biar Nextion nggak spam)
  float lastTemp_ = -999;
  float lastWeight_ = -999;
  uint32_t lastBatch_ = 0xFFFFFFFF;
  uint32_t lastVol_ = 0xFFFFFFFF;
  uint32_t lastTotalVol_ = 0xFFFFFFFF;
  uint8_t lastFuelPct_ = 0xFF;
  int32_t lastRemain_ = -999999;
  bool lastPreheat_ = false;
  uint8_t lastProg_ = 0xFF;
  uint8_t lastState_ = 0xFF;

  uint8_t selectedMotor_ = 0;

  void attachCallbacks();
  void sendEvt(const MasterEvent& e);

  void showSetpoints();
  void showState(uint8_t st);

  static HMI* self_;

  // callbacks
  static void cb_btAuto(void*);
  static void cb_bStop(void*);
  static void cb_b0_oneShot(void*);
  static void cb_bZeroTare(void*);
  static void cb_bSetScale(void*);
  static void cb_bWall(void*);
  static void cb_bSpeed(void*);
  static void cb_bTempUp(void*);
  static void cb_bTempDown(void*);
  static void cb_bTimeUp(void*);
  static void cb_bTimeDown(void*);
  static void cb_bWeightUp(void*);
  static void cb_bWeightDown(void*);
  static void cb_bSelDoor(void*);
  static void cb_bSelFeed(void*);
  static void cb_bSelBurn(void*);
  static void cb_bSelBDoor(void*);
  static void cb_bSelAsh(void*);
  static void cb_bSelMainC(void*);
  static void cb_hSpeed(void*);
  static void cb_hStep(void*);
  static void cb_bSpeedUp(void*);
  static void cb_bSpeedDown(void*);
  static void cb_bStepUp(void*);
  static void cb_bStepDown(void*);

  static void cb_btManDir(void*);
  static void cb_bManDoor(void*);
  static void cb_bManPush(void*);
  static void cb_bManBurn(void*);
  static void cb_bManBDoor(void*);
  static void cb_bManAsh(void*);
  static void cb_bManMainC(void*);
  static void cb_btManIgn(void*);
  static void cb_btManBurn(void*);
  static void cb_btManFeed(void*);
  static void cb_btManWeigh(void*);

  static void cb_hManSpeed(void*);
  static void cb_hManStep(void*);
  static void cb_bman1(void*);
  static void cb_bman2(void*);
  static void cb_bman3(void*);
  static void cb_bman4(void*);
};
