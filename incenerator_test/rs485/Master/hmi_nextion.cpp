
// =========================
// hmi_nextion.cpp
// =========================
#include "hmi_nextion.h"

HMI* HMI::self_ = nullptr;

static inline uint8_t clampPct100(int v) {
  if (v < 1) return 1;
  if (v > 100) return 100;
  return (uint8_t)v;
}

void HMI::begin() {
  self_ = this;
  nexSer_.begin(NEX_BAUD, SERIAL_8N1, PIN_NEX_RX, PIN_NEX_TX);
  nexInit();

  // Listen list
  int i = 0;
  listen_[i++] = &page0; listen_[i++] = &page1; listen_[i++] = &page2; listen_[i++] = &page3;
  listen_[i++] = &btAuto; listen_[i++] = &bStop; listen_[i++] = &b0;

  listen_[i++] = &bTempUp; listen_[i++] = &bTempDown;
  listen_[i++] = &bTimeUp; listen_[i++] = &bTimeDown;
  listen_[i++] = &bWeightUp; listen_[i++] = &bWeightDown;

  listen_[i++] = &bSelDoor; listen_[i++] = &bSelFeed; listen_[i++] = &bSelBurn;
  listen_[i++] = &bSelBDoor; listen_[i++] = &bSelAsh; listen_[i++] = &bSelMainC;

  listen_[i++] = &hSpeed; listen_[i++] = &hStep;
  listen_[i++] = &bSpeedUp; listen_[i++] = &bSpeedDown;
  listen_[i++] = &bStepUp; listen_[i++] = &bStepDown;

  listen_[i++] = &bSave; listen_[i++] = &bZero; listen_[i++] = &bSet;
  listen_[i++] = &bWall; listen_[i++] = &bSpeed;

  listen_[i++] = &btManDir;
  listen_[i++] = &bManDoor; listen_[i++] = &bManPush; listen_[i++] = &bManBurn;
  listen_[i++] = &bManBDoor; listen_[i++] = &bManAsh; listen_[i++] = &bManMainC;

  listen_[i++] = &hManSpeed; listen_[i++] = &hManStep;
  listen_[i++] = &bman1; listen_[i++] = &bman2; listen_[i++] = &bman3; listen_[i++] = &bman4;

  listen_[i++] = &btManIgn; listen_[i++] = &btManBurn;
  listen_[i++] = &btManFeed; listen_[i++] = &btManWeigh;

  listen_[i++] = nullptr;

  attachCallbacks();
}

void HMI::sendEvt(const MasterEvent& e) {
  if (!qEvt_) return;
  (void)xQueueSend(qEvt_, &e, 0);
}

void HMI::loop() {
  nexLoop(listen_);
}

void HMI::showState(uint8_t st) {
  switch (st) {
    case ST_IDLE: tStat.setText("IDLE"); break;
    case ST_WEIGHING: tStat.setText("WEIGHING"); break;
    case ST_OPENING_MAIN_DOOR: tStat.setText("D.O"); break;
    case ST_DUMPING_IN: tStat.setText("F.IN"); break;
    case ST_DUMPING_OUT: tStat.setText("F.OUT"); break;
    case ST_CLOSING_MAIN_DOOR: tStat.setText("DOOR CLOSE"); break;
    case ST_OPENING_BURN_DOOR: tStat.setText("B-DOOR OPEN"); break;
    case ST_BURNER_IN: tStat.setText("BURNER IN"); break;
    case ST_IGNITING: tStat.setText("IGNITING"); break;
    case ST_BURNING: tStat.setText("BURNING"); break;
    case ST_BURNER_OUT: tStat.setText("BURNER OUT"); break;
    case ST_CLOSING_BURN_DOOR: tStat.setText("B-DOOR CLOSE"); break;
    case ST_PREHEAT: tStat.setText("PRE-HEAT"); break;
    case ST_FAULT: tStat.setText("FAULT"); break;
    default: tStat.setText("?"); break;
  }
}

void HMI::showSetpoints() {
  if (!tlm_) return;
  char buf[24];
  snprintf(buf, sizeof(buf), "%d C", (int)tlm_->tempSetpoint);
  nTempSet.setText(buf);
  tTempSetDash.setText(buf);

  snprintf(buf, sizeof(buf), "%lu min", (unsigned long)(tlm_->burnerActiveTimeSec / 60));
  nTimeSet.setText(buf);

  snprintf(buf, sizeof(buf), "%d kg", (int)tlm_->minBatchWeight);
  nWeightSet.setText(buf);

  snprintf(buf, sizeof(buf), "%lu cm3", (unsigned long)tlm_->volSetpoint);
  nVolSet.setText(buf);
}

void HMI::render(const TelemetryStore& tlm) {
  // Pakai telemetry Mega sebagai sumber utama UI
  const TelemMega& m = tlm.mega;

  // Temp
  float t = (float)m.tempC_x1;
  if (fabsf(t - lastTemp_) > 1.0f) {
    char b[16];
    snprintf(b, sizeof(b), "%d", (int)m.tempC_x1);
    tTemp.setText(b);
    tManTemp.setText(b);
    lastTemp_ = t;
  }

  // Weight
  float wKg = m.weight_g / 1000.0f;
  if (fabsf(wKg - lastWeight_) > 0.05f) {
    char b[16];
    dtostrf(wKg, 4, 1, b);
    tWeight.setText(b);
    tWeightCal.setText(b);
    tManWeight.setText(b);
    lastWeight_ = wKg;
  }

  // Volume
  if (m.batchVol_cm3 != lastVol_) {
    char b[16];
    snprintf(b, sizeof(b), "%lu", (unsigned long)m.batchVol_cm3);
    tVolume.setText(b);
    lastVol_ = m.batchVol_cm3;
  }
  if (m.totalVol_cm3 != lastTotalVol_) {
    char b[16];
    snprintf(b, sizeof(b), "%lu", (unsigned long)m.totalVol_cm3);
    ttotVolume.setText(b);
    lastTotalVol_ = m.totalVol_cm3;
  }

  // Fuel
  if (m.fuel_pct != lastFuelPct_) {
    hFuel.setValue(m.fuel_pct);
    char b[16];
    float liter = m.fuel_l_x10 / 10.0f;
    dtostrf(liter, 4, 1, b);
    tFuel.setText(b);
    lastFuelPct_ = m.fuel_pct;
  }

  // Batch count
  if (m.batchCount != lastBatch_) {
    char b[16];
    snprintf(b, sizeof(b), "%lu", (unsigned long)m.batchCount);
    tBatch.setText(b);
    lastBatch_ = m.batchCount;
  }

  // Remaining time
  if (m.remainingIsPreheat) {
    if (!lastPreheat_) {
      tTime.setText("PRE-HEAT");
      lastPreheat_ = true;
    }
  } else {
    if (m.remaining_s >= 0 && (m.remaining_s != lastRemain_ || lastPreheat_)) {
      char tb[10];
      long mm = m.remaining_s / 60;
      long ss = m.remaining_s % 60;
      snprintf(tb, sizeof(tb), "%02ld:%02ld", mm, ss);
      tTime.setText(tb);
      lastRemain_ = m.remaining_s;
      lastPreheat_ = false;
    }
  }

  // Progress
  if (m.progress_pct != lastProg_) {
    jStat.setValue(m.progress_pct);
    lastProg_ = m.progress_pct;
  }

  // State
  if (m.state != lastState_) {
    showState(m.state);
    lastState_ = m.state;
  }

  // Setpoints (master-owned)
  showSetpoints();
}

// ===== Callbacks: semua generate MasterEvent =====
void HMI::attachCallbacks() {
  btAuto.attachPop(cb_btAuto, &btAuto);
  bStop.attachPop(cb_bStop, &bStop);
  bManStop.attachPop(cb_bStop, &bManStop);
  b0.attachPop(cb_b0_oneShot, &b0);

  bZero.attachPop(cb_bZeroTare, &bZero);
  bSet.attachPop(cb_bSetScale, &bSet);
  bWall.attachPop(cb_bWall, &bWall);
  bSpeed.attachPop(cb_bSpeed, &bSpeed);

  bTempUp.attachPop(cb_bTempUp, &bTempUp);
  bTempDown.attachPop(cb_bTempDown, &bTempDown);
  bTimeUp.attachPop(cb_bTimeUp, &bTimeUp);
  bTimeDown.attachPop(cb_bTimeDown, &bTimeDown);
  bWeightUp.attachPop(cb_bWeightUp, &bWeightUp);
  bWeightDown.attachPop(cb_bWeightDown, &bWeightDown);

  bSelDoor.attachPop(cb_bSelDoor, &bSelDoor);
  bSelFeed.attachPop(cb_bSelFeed, &bSelFeed);
  bSelBurn.attachPop(cb_bSelBurn, &bSelBurn);
  bSelBDoor.attachPop(cb_bSelBDoor, &bSelBDoor);
  bSelAsh.attachPop(cb_bSelAsh, &bSelAsh);
  bSelMainC.attachPop(cb_bSelMainC, &bSelMainC);

  hSpeed.attachPop(cb_hSpeed, &hSpeed);
  hStep.attachPop(cb_hStep, &hStep);
  bSpeedUp.attachPop(cb_bSpeedUp, &bSpeedUp);
  bSpeedDown.attachPop(cb_bSpeedDown, &bSpeedDown);
  bStepUp.attachPop(cb_bStepUp, &bStepUp);
  bStepDown.attachPop(cb_bStepDown, &bStepDown);

  btManDir.attachPop(cb_btManDir, &btManDir);
  bManDoor.attachPop(cb_bManDoor, &bManDoor);
  bManPush.attachPop(cb_bManPush, &bManPush);
  bManBurn.attachPop(cb_bManBurn, &bManBurn);
  bManBDoor.attachPop(cb_bManBDoor, &bManBDoor);
  bManAsh.attachPop(cb_bManAsh, &bManAsh);
  bManMainC.attachPop(cb_bManMainC, &bManMainC);

  btManIgn.attachPop(cb_btManIgn, &btManIgn);
  btManBurn.attachPop(cb_btManBurn, &btManBurn);
  btManFeed.attachPop(cb_btManFeed, &btManFeed);
  btManWeigh.attachPop(cb_btManWeigh, &btManWeigh);

  hManSpeed.attachPop(cb_hManSpeed, &hManSpeed);
  hManStep.attachPop(cb_hManStep, &hManStep);
  bman1.attachPop(cb_bman1, &bman1);
  bman2.attachPop(cb_bman2, &bman2);
  bman3.attachPop(cb_bman3, &bman3);
  bman4.attachPop(cb_bman4, &bman4);
}

void HMI::cb_btAuto(void*){
  uint32_t v=0; self_->btAuto.getValue(&v);
  MasterEvent e{EVT_UI_AUTO_TOGGLE,0,0,0,(int32_t)v};
  self_->sendEvt(e);
}
void HMI::cb_bStop(void*){ MasterEvent e{EVT_UI_ESTOP,0,0,0,0}; self_->sendEvt(e);} 
void HMI::cb_b0_oneShot(void*){ MasterEvent e{EVT_UI_ONE_SHOT,0,0,0,0}; self_->sendEvt(e);} 
void HMI::cb_bZeroTare(void*){ MasterEvent e{EVT_UI_TARE,0,0,0,0}; self_->sendEvt(e);} 
void HMI::cb_bSetScale(void*){ MasterEvent e{EVT_UI_CAL_SCALE,0,0,0,0}; self_->sendEvt(e);} 
void HMI::cb_bWall(void*){ MasterEvent e{EVT_UI_CAL_WALL,0,0,0,0}; self_->sendEvt(e);} 
void HMI::cb_bSpeed(void*){ MasterEvent e{EVT_UI_CAL_SPEED,0,0,0,0}; self_->sendEvt(e);} 

void HMI::cb_bTempUp(void*){ if(self_->tlm_){ self_->tlm_->tempSetpoint += 50; } MasterEvent e{EVT_UI_SETPOINTS_CHANGED,0,0,0,0}; self_->sendEvt(e);} 
void HMI::cb_bTempDown(void*){ if(self_->tlm_){ self_->tlm_->tempSetpoint -= 50; if(self_->tlm_->tempSetpoint < 0) self_->tlm_->tempSetpoint=0; } MasterEvent e{EVT_UI_SETPOINTS_CHANGED,0,0,0,0}; self_->sendEvt(e);} 
void HMI::cb_bTimeUp(void*){ if(self_->tlm_){ self_->tlm_->burnerActiveTimeSec += 60; } MasterEvent e{EVT_UI_SETPOINTS_CHANGED,0,0,0,0}; self_->sendEvt(e);} 
void HMI::cb_bTimeDown(void*){ if(self_->tlm_){ if(self_->tlm_->burnerActiveTimeSec >= 60) self_->tlm_->burnerActiveTimeSec -= 60; } MasterEvent e{EVT_UI_SETPOINTS_CHANGED,0,0,0,0}; self_->sendEvt(e);} 
void HMI::cb_bWeightUp(void*){ if(self_->tlm_){ self_->tlm_->minBatchWeight += 1.0f; } MasterEvent e{EVT_UI_SETPOINTS_CHANGED,0,0,0,0}; self_->sendEvt(e);} 
void HMI::cb_bWeightDown(void*){ if(self_->tlm_){ if(self_->tlm_->minBatchWeight >= 1.0f) self_->tlm_->minBatchWeight -= 1.0f; } MasterEvent e{EVT_UI_SETPOINTS_CHANGED,0,0,0,0}; self_->sendEvt(e);} 

static void selMotor(HMI* h, uint8_t id){ h->selectedMotor_=id; }
void HMI::cb_bSelDoor(void*){ selMotor(self_,0);} 
void HMI::cb_bSelFeed(void*){ selMotor(self_,1);} 
void HMI::cb_bSelBurn(void*){ selMotor(self_,2);} 
void HMI::cb_bSelBDoor(void*){ selMotor(self_,3);} 
void HMI::cb_bSelAsh(void*){ selMotor(self_,4);} 
void HMI::cb_bSelMainC(void*){ selMotor(self_,5);} 

void HMI::cb_hSpeed(void*){ uint32_t v=1; self_->hSpeed.getValue(&v); v=clampPct100((int)v); self_->nSpeed.setValue(v); MasterEvent e{EVT_UI_MOTOR_SPEED_CHANGED,self_->selectedMotor_,0,0,(int32_t)v}; self_->sendEvt(e);} 
void HMI::cb_hStep(void*){ uint32_t v=1; self_->hStep.getValue(&v); v=clampPct100((int)v); self_->nStep.setValue(v*100UL); MasterEvent e{EVT_UI_MOTOR_STEP_CHANGED,self_->selectedMotor_,0,0,(int32_t)v}; self_->sendEvt(e);} 

void HMI::cb_bSpeedUp(void*){ uint32_t v=1; self_->hSpeed.getValue(&v); v=clampPct100((int)v+1); self_->hSpeed.setValue(v); self_->nSpeed.setValue(v); MasterEvent e{EVT_UI_MOTOR_SPEED_CHANGED,self_->selectedMotor_,0,0,(int32_t)v}; self_->sendEvt(e);} 
void HMI::cb_bSpeedDown(void*){ uint32_t v=1; self_->hSpeed.getValue(&v); v=clampPct100((int)v-1); self_->hSpeed.setValue(v); self_->nSpeed.setValue(v); MasterEvent e{EVT_UI_MOTOR_SPEED_CHANGED,self_->selectedMotor_,0,0,(int32_t)v}; self_->sendEvt(e);} 
void HMI::cb_bStepUp(void*){ uint32_t v=1; self_->hStep.getValue(&v); v=clampPct100((int)v+1); self_->hStep.setValue(v); self_->nStep.setValue(v*100UL); MasterEvent e{EVT_UI_MOTOR_STEP_CHANGED,self_->selectedMotor_,0,0,(int32_t)v}; self_->sendEvt(e);} 
void HMI::cb_bStepDown(void*){ uint32_t v=1; self_->hStep.getValue(&v); v=clampPct100((int)v-1); self_->hStep.setValue(v); self_->nStep.setValue(v*100UL); MasterEvent e{EVT_UI_MOTOR_STEP_CHANGED,self_->selectedMotor_,0,0,(int32_t)v}; self_->sendEvt(e);} 

void HMI::cb_btManDir(void*){ uint32_t v=0; self_->btManDir.getValue(&v); MasterEvent e{EVT_UI_MANUAL_DIR_CHANGED,(uint8_t)v,0,0,0}; self_->sendEvt(e);} 

static void reqMove(HMI* h, uint8_t id){ MasterEvent e{EVT_UI_MANUAL_MOVE,id,0,0,0}; h->sendEvt(e);} 
void HMI::cb_bManDoor(void*){ reqMove(self_,0);} 
void HMI::cb_bManPush(void*){ reqMove(self_,1);} 
void HMI::cb_bManBurn(void*){ reqMove(self_,2);} 
void HMI::cb_bManBDoor(void*){ reqMove(self_,3);} 
void HMI::cb_bManAsh(void*){ reqMove(self_,4);} 
void HMI::cb_bManMainC(void*){ reqMove(self_,5);} 

void HMI::cb_btManIgn(void*){ uint32_t v=0; self_->btManIgn.getValue(&v); MasterEvent e{EVT_UI_MANUAL_IGN,(uint8_t)v,0,0,0}; self_->sendEvt(e);} 
void HMI::cb_btManBurn(void*){ uint32_t v=0; self_->btManBurn.getValue(&v); MasterEvent e{EVT_UI_MANUAL_BURN,(uint8_t)v,0,0,0}; self_->sendEvt(e);} 
void HMI::cb_btManFeed(void*){ uint32_t v=0; self_->btManFeed.getValue(&v); MasterEvent e{EVT_UI_MANUAL_FEED,(uint8_t)v,0,0,0}; self_->sendEvt(e);} 
void HMI::cb_btManWeigh(void*){ uint32_t v=0; self_->btManWeigh.getValue(&v); MasterEvent e{EVT_UI_MANUAL_WEIGH,(uint8_t)v,0,0,0}; self_->sendEvt(e);} 

// Manual speed/step slider: di HMI lama kamu pakai applyManualSpeedPct/StepPct.
// Di sini kita kirim event “delta” pakai bman buttons biar nggak banyak logic di HMI.
void HMI::cb_hManSpeed(void*){ uint32_t v=1; self_->hManSpeed.getValue(&v); MasterEvent e{EVT_UI_MOTOR_SPEED_CHANGED,0xFE,0,0,(int32_t)clampPct100((int)v)}; self_->sendEvt(e);} 
void HMI::cb_hManStep(void*){ uint32_t v=1; self_->hManStep.getValue(&v); MasterEvent e{EVT_UI_MOTOR_STEP_CHANGED,0xFE,0,0,(int32_t)clampPct100((int)v)}; self_->sendEvt(e);} 

void HMI::cb_bman1(void*){ MasterEvent e{EVT_UI_MOTOR_SPEED_CHANGED,0xFC,0,0,-1}; self_->sendEvt(e);} // speed--
void HMI::cb_bman2(void*){ MasterEvent e{EVT_UI_MOTOR_SPEED_CHANGED,0xFC,0,0, 1}; self_->sendEvt(e);} // speed++
void HMI::cb_bman3(void*){ MasterEvent e{EVT_UI_MOTOR_STEP_CHANGED, 0xFB,0,0, 1}; self_->sendEvt(e);} // step++
void HMI::cb_bman4(void*){ MasterEvent e{EVT_UI_MOTOR_STEP_CHANGED, 0xFB,0,0,-1}; self_->sendEvt(e);} // step--
