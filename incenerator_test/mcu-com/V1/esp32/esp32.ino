/*
 * MCU-HMI (Nextion Controller)
 * -------------------------------------
 * Tugas: Menangani UI, Input User, dan Menampilkan Data
 * Hardware: Nextion Display (Serial2)
 * Komunikasi: Mengirim perintah ke CTRL, Menerima update status dari CTRL
 */
// #include <SoftwareSerial.h>
#include <Arduino.h>

// HardwareSerial Serial(1);

// [Nextion Serial Define]

#define NEXTION_RX 16
#define NEXTION_TX 17

#define nexSerial Serial2  // dipakai di library Nextion
#include <Nextion.h>

enum SystemState {
  IDLE = 0,
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
// ================== VARIABEL GLOBAL (Mirror dari CTRL untuk Display) ==================
// Variabel ini nanti di-update via Serial dari MCU-CTRL
float currentTempC = 0.0;
float currentWeightKg = 0.0;
float currentVol = 0.0;
float solarVolumeUsed_L = 0.0;
float totalConsumedKg = 0.0;
float totalVolume = 0.0;
unsigned long totalBatchCount = 0;
float minBatchWeight = 5.0;  // Default, nanti disinkronkan
uint32_t burnerActiveTimeSec = 30;
float tempSetpoint = 1200.0;
float volSetpoint = 20.0;

SystemState currentState = IDLE;
SystemState lastState = IDLE;

// Variabel UI Helper
float lastTempSent = -999;
float lastVolSent = -999;
float lastTotalVolSent = -999;

float lastTempManSent = -999;
float lastWeightSent = -999;
float lastWeightManSent = -999;
float lastFuelSent = -999;
float lastTotalKgSent = -999;
unsigned long lastBatchSent = 999999;

// Variabel Setting UI
int selectedMotorIdx = 0;
uint8_t currentPage = 0;
bool motorSettingsDirty = false;

// Blower UI State
int blowLevel[3] = { 0, 0, 0 };

// ===== Remaining time state (mirror dari CTRL) =====
long remainingTimeSec = -1;           // -1 = belum ada data
bool remainingIsPreheat = false;      // true = tampil "PRE-HEAT"
unsigned long remainingLastRxMs = 0;  // kapan terakhir update diterima (opsional)

// ===== UI helper untuk remaining time =====
long lastRemainingTimeSent = -999999;
bool lastPreheatSent = false;

// Struktur Konfigurasi Motor (Mirror untuk Slider)
struct MotorConfig {
  long speed;
  long accel;
  long steps;
};
// Default sementara agar tidak error saat akses array
MotorConfig motorConfigs[6] = {
  { 1000, 200, 2000 }, { 1000, 200, 5000 }, { 1000, 200, 3000 }, { 800, 200, 1000 }, { 800, 200, 1000 }, { 800, 200, 1000 }
};

// ================== NEXTION OBJECTS ==================
/* [Nextion UI Objects] */
NexPage page0 = NexPage(0, 0, "page0");
NexPage page1 = NexPage(1, 0, "page1");
NexPage page2 = NexPage(2, 0, "page2");

// --- PAGE 0: DASHBOARD ---
NexText tStat = NexText(0, 2, "tStat");
NexText tTemp = NexText(0, 3, "tTemp");
NexText tWeight = NexText(0, 4, "tWeight");
NexText tTime = NexText(0, 5, "tTime");
NexText tFuel = NexText(0, 6, "tFuel");
NexText tBatch = NexText(0, 7, "tBatch");
NexText tTotWeight = NexText(0, 8, "tTotWeight");
NexDSButton btAuto = NexDSButton(0, 9, "btAuto");
NexButton bStop = NexButton(0, 13, "bStop");
NexProgressBar jStat = NexProgressBar(0, 16, "jStat");
NexButton b1 = NexButton(0, 15, "b1");
NexText tWeightCal = NexText(0, 18, "tWeightCal");
NexButton bsettingStop = NexButton(2, 13, "bsettingStop");
NexText tTempSetDash = NexText(0, 11, "t8");
NexText tVolume = NexText(0, 12, "t9");
NexText ttotVolume = NexText(0, 10, "tVolume");


// --- PAGE 1: KONTROL MANUAL ---
NexDSButton btManDir = NexDSButton(1, 2, "btManDir");
NexButton bManDoor = NexButton(1, 4, "bManDoor");
NexButton bManBDoor = NexButton(1, 5, "bManBDoor");
NexButton bManBurn = NexButton(1, 6, "bManBurn");
NexButton bManPush = NexButton(1, 20, "bManFeed");
NexButton bManAsh = NexButton(1, 7, "bManAsh");
NexDSButton bManMainC = NexDSButton(1, 18, "bManCon");

NexSlider hManSpeed = NexSlider(1, 8, "hManSpeed");
NexSlider hManStep = NexSlider(1, 9, "hManStep");

NexDSButton btManBurn = NexDSButton(1, 19, "btManBurn");
NexDSButton btManIgn = NexDSButton(1, 3, "btManIgn");
NexDSButton btManFeed = NexDSButton(1, 99, "btManFeed");
NexDSButton btManWeigh = NexDSButton(1, 98, "btManWeigh");

NexText tManTemp = NexText(1, 11, "tManTemp");
NexText tManWeight = NexText(1, 12, "tManWeight");
NexButton bManStop = NexButton(1, 10, "b5");
NexButton b0 = NexButton(1, 15, "b0");

// --- PAGE 2: PENGATURAN ---
NexText nTempSet = NexText(2, 21, "nTempSet");
NexText nTimeSet = NexText(2, 20, "nTimeSet");
NexText nWeightSet = NexText(2, 19, "nWeightSet");

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

NexText tSpeed = NexText(2, 22, "tSpeed");
NexText tStep = NexText(2, 23, "tStep");

NexButton bBlow1 = NexButton(2, 17, "bBlow1");
NexButton bBlow2 = NexButton(2, 16, "bBlow2");

NexText tBlow1 = NexText(2, 25, "tBlow1");
NexText tBlow2 = NexText(2, 26, "tBlow2");

NexButton bSave = NexButton(2, 15, "bSave");
NexButton bZero = NexButton(2, 12, "bZero");

NexButton bSet = NexButton(2, 31, "bSet");
NexButton bWall = NexButton(2, 35, "bWall");
NexButton bSpeed = NexButton(2, 36, "bSpeed");




// --- LISTEN LIST ---
NexTouch *nex_listen_list[] = {
  &page0, &page1, &page2,
  &btAuto, &bStop, &b1,
  &nWeightSet, &nTimeSet,
  &bTempUp, &bTempDown, &bTimeUp, &bTimeDown, &bWeightUp, &bWeightDown,
  &bSelDoor, &bSelBurn, &bSelFeed, &bSelBDoor, &bSelMainC, &bSelAsh,
  &hSpeed, &hStep,
  &bBlow1, &bBlow2, &bsettingStop,
  &bSave, &bZero, &bWall, &bSpeed, &bSet,
  &btManDir, &bManMainC,
  &bManDoor, &bManBDoor, &bManBurn, &bManPush, &bManAsh,
  &hManSpeed, &hManStep, &bManStop,
  &btManBurn, &btManIgn, &btManFeed, &btManWeigh, &b0,
  NULL
};

// ================== HELPER FUNCTIONS ==================

const char *levelToText(int level) {
  switch (level) {
    case 0: return "OFF";
    case 1: return "LOW";
    case 2: return "MED";
    case 3: return "HIGH";
  }
  return "OFF";
}

void updateBlowText(int id) {
  const char *txt = levelToText(blowLevel[id]);
  switch (id) {
    case 0: tBlow1.setText(txt); break;
    case 1: tBlow2.setText(txt); break;
  }
}

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
  uint32_t menit = burnerActiveTimeSec / 60;
  sprintf(buf, "%lu min", (unsigned long)menit);
  nTimeSet.setText(buf);
}
void showVolSet() {
  char buf[16];
  sprintf(buf, "%d m3", (int)volSetpoint);
  // nVol.setText(buf);
}

void updateMotorInfoTexts() {
  char buf[16];
  // Ambil dari variabel lokal HMI (yang nanti disinkronkan dengan CTRL)
  sprintf(buf, "%ld", motorConfigs[selectedMotorIdx].speed);
  // tSpeed.setValue(buf);
  sprintf(buf, "%ld", motorConfigs[selectedMotorIdx].steps);
  // tStep.setValue(buf);
}

void updateSliderFromConfig() {
  hSpeed.setValue(motorConfigs[selectedMotorIdx].speed);
  hStep.setValue(motorConfigs[selectedMotorIdx].steps);
}

void updateMotorSelectorUI() {
  bSelDoor.setValue(selectedMotorIdx == 0 ? 1 : 0);
  bSelFeed.setValue(selectedMotorIdx == 1 ? 1 : 0);
  bSelBurn.setValue(selectedMotorIdx == 2 ? 1 : 0);
  bSelBDoor.setValue(selectedMotorIdx == 3 ? 1 : 0);
  bSelAsh.setValue(selectedMotorIdx == 4 ? 1 : 0);
  bSelMainC.setValue(selectedMotorIdx == 5 ? 1 : 0);
}

void selectMotor(uint8_t idx) {
  selectedMotorIdx = idx;
  updateMotorSelectorUI();
  updateSliderFromConfig();
  updateMotorInfoTexts();
}

void initSettingPageTexts() {
  showTempSet();
  showTimeSet();
  showWeightSet();

  selectedMotorIdx = 0;
  updateMotorSelectorUI();
  updateSliderFromConfig();
  updateMotorInfoTexts();

  for (int i = 0; i < 2; i++) {
    updateBlowText(i);
  }
}

// ================== HMI UPDATE ROUTINE ==================
void updateHMI() {
  // Dipanggil di loop utama untuk refresh Text Object jika ada data baru dari CTRL
  char buf[20];

  // --- Temp Dashboard ---
  if (fabs(currentTempC - lastTempSent) > 0.5) {
    dtostrf(currentTempC, 3, 0, buf);
    tTemp.setText(buf);
    lastTempSent = currentTempC;
    Serial.print("Current Temp: ");
    Serial.println(currentTempC);
  }

  // --- Temp Manual Page ---
  if (fabs(currentTempC - lastTempManSent) > 0.5) {
    dtostrf(currentTempC, 3, 0, buf);
    tManTemp.setText(buf);
    lastTempManSent = currentTempC;
  }

  // --- Weight Dashboard ---
  if (fabs(currentWeightKg - lastWeightSent) > 0.5) {
    dtostrf(currentWeightKg, 4, 1, buf);
    tWeight.setText(buf);
    tWeightCal.setText(buf);
    lastWeightSent = currentWeightKg;
    Serial.print("Current Weight: ");
    Serial.println(currentWeightKg);
  }

  // --- Weight Manual Page ---
  if (fabs(currentWeightKg - lastWeightManSent) > 0.5) {
    dtostrf(currentWeightKg, 4, 1, buf);
    tManWeight.setText(buf);
    lastWeightManSent = currentWeightKg;
  }

// ---  Volume Dashboard ---

  if (fabs(currentVol - lastVolSent) > 0.5) {
    dtostrf(currentVol, 4, 1, buf);
    tVolume.setText(buf);
    lastVolSent = currentVol;
    Serial.print("Current Volume: ");
    Serial.println(currentVol);
  }

  // ---  Total Volume Dashboard ---

  if (fabs(totalVolume - lastTotalVolSent) > 0.5) {
    dtostrf(totalVolume, 4, 1, buf);
    ttotVolume.setText(buf);
    lastTotalVolSent = totalVolume;
    Serial.print("Total Volume: ");
    Serial.println(totalVolume);
  }

  // --- Fuel ---
  if (fabs(solarVolumeUsed_L - lastFuelSent) > 0.5) {
    dtostrf(solarVolumeUsed_L, 4, 1, buf);
    tFuel.setText(buf);
    lastFuelSent = solarVolumeUsed_L;
    Serial.print("solarVolumeUsed_L: ");
    Serial.println(solarVolumeUsed_L);
  }

  // --- Total Konsumsi KG ---
  if (fabs(totalConsumedKg - lastTotalKgSent) > 0.5) {
    dtostrf(totalConsumedKg, 4, 1, buf);
    tTotWeight.setText(buf);
    lastTotalKgSent = totalConsumedKg;
  }
  if (remainingIsPreheat) {
    // Update hanya kalau berubah
    if (!lastPreheatSent) {
      tTime.setText("PRE-HEAT");
      lastPreheatSent = true;
    }
  } else {
    // kalau bukan preheat, tampil MM:SS dari remainingTimeSec
    if (remainingTimeSec >= 0) {
      if (lastPreheatSent || remainingTimeSec != lastRemainingTimeSent) {
        char timeBuf[10];
        long mm = remainingTimeSec / 60;
        long ss = remainingTimeSec % 60;
        sprintf(timeBuf, "%02ld:%02ld", mm, ss);
        tTime.setText(timeBuf);

        lastRemainingTimeSent = remainingTimeSec;
        lastPreheatSent = false;
      }
    } else {
      // optional: kalau belum pernah dapat data, kamu mau kosongin / tampil "--:--"
      // tTime.setText("--:--");
    }
  }

  // --- Total Batch ---
  if (totalBatchCount != lastBatchSent) {
    snprintf(buf, sizeof(buf), "%lu", totalBatchCount);
    tBatch.setText(buf);
    lastBatchSent = totalBatchCount;
    Serial.print("totalBatchCount: ");
    Serial.println(totalBatchCount);
  }
  if (currentState != lastState) {
    switch (currentState) {
      case IDLE: tStat.setText("IDLE"); break;
      case WEIGHING: tStat.setText("WEIGHING"); break;
      case OPENING_MAIN_DOOR: tStat.setText("D.O"); break;
      case DUMPING_IN: tStat.setText("F.IN"); break;
      case DUMPING_TRASH: tStat.setText("DUMP"); break;
      case DUMPING_OUT: tStat.setText("F. OUT"); break;
      case CLOSING_MAIN_DOOR: tStat.setText("DOOR CLOSE"); break;

      case OPENING_BURN_DOOR: tStat.setText("B-DOOR OPEN"); break;
      case BURNER_IN: tStat.setText("BURNER IN"); break;
      case IGNITING: tStat.setText("IGNITING"); break;
      case BURNING: tStat.setText("BURNING"); break;
      case BURNER_OUT: tStat.setText("BURNER OUT"); break;
      case CLOSING_BURN_DOOR: tStat.setText("B-DOOR CLOSE"); break;

      case FAULT: tStat.setText("FAULT"); break;
    }
    lastState = currentState;
  }
}


// ================== CALLBACK IMPLEMENTATIONS ==================


// --- DASHBOARD ---
void btAutoPopCallback(void *ptr) {
  uint32_t dual_state;
  btAuto.getValue(&dual_state);
  if (dual_state == 1) {
    Serial.println("CMD_AUTO_START");
  } else {
    Serial.println("CMD_AUTO_STOP");
  }
}

void bStopPopCallback(void *ptr) {
  Serial.println("CMD_EMERGENCY_STOP");
  // Serial.println("CMD_ONE_SHOT_AUTO");
}

void b0PopCallback(void *ptr) {
  Serial.println("CMD_ONE_SHOT_AUTO");
}

// --- MANUAL CONTROL ---
void btManDirPopCallback(void *ptr) {
  uint32_t val;
  btManDir.getValue(&val);
  
  // val = 0 atau 1
  // Kirim ke CTRL untuk mengubah variabel 'isManualOpen'
  Serial.print("CMD_SET_MANUAL_DIR:");
  Serial.println(val);
}

void hManSpeedPopCallback(void *ptr) {
  uint32_t val;
  hManSpeed.getValue(&val);

  long realSpeed = map(val, 0, 100, 100, 5000); 
  
  Serial.print("SET_M_SPD:");
  Serial.println(realSpeed);
}

// --- MANUAL STEPS SLIDER ---
void hManStepPopCallback(void *ptr) {
  uint32_t val;
  hManStep.getValue(&val);
  
  long realSteps = map(val, 0, 100, 100, 10000); 
  
  Serial.print("SET_M_STP:");
  Serial.println(realSteps);
}

// Helper Manual Move Wrapper
void reqManualMove(int motorID) {
  Serial.print("REQ_MOVE:");
  Serial.println(motorID);
}

void bManDoorPopCallback(void *ptr) {
  reqManualMove(0);
}  // Door
void bManPushPopCallback(void *ptr) {
  reqManualMove(1);
}  // Push
void bManBurnPopCallback(void *ptr) {
  reqManualMove(2);
}  // Burner
void bManBDoorPopCallback(void *ptr) {
  reqManualMove(3);
}  // BurnDoor
void bManAshPopCallback(void *ptr) {
  reqManualMove(4);
}  // Ash
void bManMainCPopCallback(void *ptr) {
  reqManualMove(5);
}  // MainConveyor

// --- MANUAL IGNITION (Pemantik) ---
void btManIgnPopCallback(void *ptr) {
  uint32_t val;
  btManIgn.getValue(&val);
  // Kirim: CMD_MAN_IGN:1 (ON) atau CMD_MAN_IGN:0 (OFF)
  Serial.print("CMD_MAN_IGN:");
  Serial.println(val);
}

void btManBurnPopCallback(void *ptr) {
  uint32_t val;
  btManBurn.getValue(&val);
  // Kirim: CMD_MAN_IGN:1 (ON) atau CMD_MAN_IGN:0 (OFF)
  Serial.print("CMD_MAN_BURN:");
  Serial.println(val);
}

// --- MANUAL FEEDER (Main Conveyor) ---
void btManFeedPopCallback(void *ptr) {
  uint32_t val;
  btManFeed.getValue(&val);
  // Kirim: CMD_MAN_FEED:1 atau 0
  Serial.print("CMD_MAN_FEED:");
  Serial.println(val);
}

// --- MANUAL WEIGHING (Motor/Relay Timbangan) ---
void btManWeighPopCallback(void *ptr) {
  uint32_t val;
  btManWeigh.getValue(&val);
  // Kirim: CMD_MAN_WEIGH:1 atau 0
  Serial.print("CMD_MAN_WEIGH:");
  Serial.println(val);
}

// --- SETTINGS ---
void bTempUpPopCallback(void *ptr) {
  tempSetpoint += 50.0;
  showTempSet();
  Serial.print("SET_TE:");
  Serial.println(tempSetpoint);
}
void bTempDownPopCallback(void *ptr) {
  tempSetpoint -= 50.0;
  if (tempSetpoint < 0) tempSetpoint = 0;
  showTempSet();
  Serial.print("SET_TE:");
  Serial.println(tempSetpoint);
}

void bTimeUpPopCallback(void *ptr) {
  burnerActiveTimeSec += 60;
  showTimeSet();
  Serial.print("SET_TI:");
  Serial.println(burnerActiveTimeSec);


}
void bTimeDownPopCallback(void *ptr) {
  if (burnerActiveTimeSec >= 60) burnerActiveTimeSec -= 60;
  showTimeSet();
  Serial.print("SET_TI:");
  Serial.println(burnerActiveTimeSec);
}

void bWeightUpPopCallback(void *ptr) {
  minBatchWeight += 1.0;
  showWeightSet();
  Serial.print("SET_W:");
  Serial.println(minBatchWeight);
}
void bWeightDownPopCallback(void *ptr) {
  if (minBatchWeight >= 1.0) minBatchWeight -= 1.0;
  showWeightSet();
  Serial.print("SET_W:");
  Serial.println(minBatchWeight);
}

void bSelDoorPopCallback(void *ptr) {
  selectMotor(0);
}
void bSelFeedPopCallback(void *ptr) {
  selectMotor(1);
}
void bSelBurnPopCallback(void *ptr) {
  selectMotor(2);
}
void bSelBDoorPopCallback(void *ptr) {
  selectMotor(3);
}
void bSelAshPopCallback(void *ptr) {
  selectMotor(4);
}
void bSelMainCPopCallback(void *ptr) {
  selectMotor(5);
}

void hSpeedPopCallback(void *ptr) {
  uint32_t val;
  hSpeed.getValue(&val);
  long speed = map(val, 0, 100, 0, 10000);
  motorConfigs[selectedMotorIdx].speed = speed;
  updateMotorInfoTexts();
  Serial.print("M_SPD:");
  Serial.print(selectedMotorIdx);
  Serial.print(":");
  Serial.println(speed);
}

void hStepPopCallback(void *ptr) {
  uint32_t val;
  hStep.getValue(&val);
  long steps = map(val, 0, 100, 0, 10000);  // Sesuaikan mapping
  motorConfigs[selectedMotorIdx].steps = steps;
  motorSettingsDirty = true;
  updateMotorInfoTexts();
  Serial.print("M_STP:");
  Serial.print(selectedMotorIdx);
  Serial.print(":");
  Serial.println(steps);
}

void updateBlowerState(int id) {
  blowLevel[id]++;
  if (blowLevel[id] > 3) blowLevel[id] = 0;
  Serial.print("BLOW:");
  Serial.print(id);
  Serial.print(":");
  Serial.println(blowLevel[id]);
  updateBlowText(id);  // Update tampilan dulu biar terasa instan (Zero Latency)

  // Kirim perintah memaksa ke CTRL
  // Format: BLOW:<id>:<level> -> Contoh "BLOW:0:2"
  Serial.print("BLOW:");
  Serial.print(id);
  Serial.print(":");
  Serial.println(blowLevel[id]);
}

void bBlow1PopCallback(void *ptr) {
  updateBlowerState(0);
}
void bBlow2PopCallback(void *ptr) {
  updateBlowerState(1);
}

void bSavePopCallback(void *ptr) {
  // Kirim nilai setting terbaru ke CTRL sebelum save

  Serial.println("CMD_SAVE_SETTINGS");
  motorSettingsDirty = false;
}

void bZeroPopCallback(void *ptr) {
  Serial.println("CMD_TARE_SCALE");
}

void bSetPopCallback(void *ptr) {
  Serial.println("CMD_SET_SCALE");
}

void bWallPopCallback(void *ptr) {
  Serial.println("CMD_CAL_WALL");
}

void bSpeedPopCallback(void *ptr) {
  Serial.println("CMD_CAL_SPEED");
}


void nWeightSetPopCallback(void *ptr) {
  char buf[16] = { 0 };
  nWeightSet.getText(buf, sizeof(buf) - 1);
  minBatchWeight = atof(buf);
  showWeightSet();
}

void nTimeSetPopCallback(void *ptr) {
  char buf[16] = { 0 };
  nTimeSet.getText(buf, sizeof(buf) - 1);
  uint32_t menit = (uint32_t)atoi(buf);
  burnerActiveTimeSec = menit * 60;
  showTimeSet();
}

String inputBuffer = "";

void ParseDataFromCTRL() {
  while (Serial.available() > 0) {
    char inChar = (char)Serial.read();
    
    if (inChar == '\n') {
      inputBuffer.trim(); // Hapus \r atau spasi
      
      // Proses Data (Pindahkan logika parsing ke sini)
      if (inputBuffer.length() > 0) {
        ProcessLine(inputBuffer);
      }
      
      inputBuffer = ""; // Reset buffer
    } else {
      inputBuffer += inChar;
    }
  }
}

// Buat fungsi terpisah agar rapi
void ProcessLine(String data) {
  if (data.startsWith("METRICS:")) {
 
      int c1 = data.indexOf(',');
      int c2 = data.indexOf(',', c1 + 1);
      int c3 = data.indexOf(',', c2 + 1);
      
      if (c1 > 0 && c2 > 0 && c3 > 0) {
        String tFuel = data.substring(8, c1); 
        String tVol = data.substring(c1 + 1, c2);
        String tWeight = data.substring(c2 + 1, c3);
        String tBatch = data.substring(c3 + 1);     

        solarVolumeUsed_L = tFuel.toFloat();
        totalVolume  = tVol.toFloat();
        totalConsumedKg  = tWeight.toFloat();
        totalBatchCount  = tBatch.toInt();
      }
    }
    // 1. Terima Data Telemetri Rutin (Format: DATA:TEMP,WEIGHT,STATE_ID)
    if (data.startsWith("DATA:")) {
      // Format: DATA:TEMP,WEIGHT,STATE,CUR_VOL
      int firstComma = data.indexOf(',');
      int secondComma = data.indexOf(',', firstComma + 1);
      int thirdComma = data.indexOf(',', secondComma + 1);

      if (firstComma > 0 && secondComma > 0 && thirdComma > 0) {
        String sTemp = data.substring(5, firstComma);
        String sWeight = data.substring(firstComma + 1, secondComma);
        String sState = data.substring(secondComma + 1, thirdComma);
        String sVol = data.substring(thirdComma + 1);

        currentTempC = sTemp.toFloat();
        currentWeightKg = sWeight.toFloat();
        
        // Update State
        int st = sState.toInt();
        if (st >= IDLE && st <= FAULT) {
          currentState = (SystemState)st;
        }

        // Update Volume Batch Saat Ini
        currentVol = sVol.toFloat();
      }
    }

    // 2. Terima Progress Bar (Format: PROG:50)
    if (data.startsWith("PROG:")) {
      int val = data.substring(5).toInt();
      jStat.setValue(val);
    } else 
    if (data.startsWith("REM_TIME:")) {
      long val = data.substring(9).toInt();
      remainingTimeSec = val;
      remainingIsPreheat = false;
      remainingLastRxMs = millis();
    } 

    // PREHEAT (atau format lain sesuai CTRL-mu)
    if (data.startsWith("PREHEAT")) {
      remainingIsPreheat = true;
      remainingLastRxMs = millis();
    }
    // 3. Sinkronisasi Setting Awal
    if (data.startsWith("W_SET:")) {
      minBatchWeight = data.substring(6).toFloat();
      showWeightSet();
    } else if (data.startsWith("TE_SET:")) {  // temp setpoint
      tempSetpoint = data.substring(7).toFloat();
      showTempSet();
    } else if (data.startsWith("TI_SET:")) {  // time seconds
      burnerActiveTimeSec = data.substring(7).toInt();
      showTimeSet();
      
    } else if (data.startsWith("VB_SET:")) {  // m3 seconds
      volSetpoint = data.substring(7).toInt();
      showVolSet();

    // 4. Sinkronisasi Posisi Slider Motor (M_SYNC:ID:SPD:STP)
    if (data.startsWith("M_SYNC:")) {
      // Gunakan nama variabel yang konsisten biar tidak bingung
      int firstColon = data.indexOf(':');
      int secondColon = data.indexOf(':', firstColon + 1);
      int thirdColon = data.indexOf(':', secondColon + 1);

      // Pastikan data valid (titik dua ditemukan semua)
      if (firstColon > 0 && secondColon > 0 && thirdColon > 0) {
        int id = data.substring(firstColon + 1, secondColon).toInt();
        long spd = data.substring(secondColon + 1, thirdColon).toInt();
        long stp = data.substring(thirdColon + 1).toInt();

        if (id >= 0 && id < 6) {
          motorConfigs[id].speed = spd;
          motorConfigs[id].steps = stp;
        }
      }
    }

    // 5. Sinkronisasi Status Blower (B_SYNC:ID:LVL)
    if (data.startsWith("B_SYNC:")) {
      int first = data.indexOf(':');
      int second = data.indexOf(':', first + 1);
      int id = data.substring(first + 1, second).toInt();
      int lvl = data.substring(second + 1).toInt();

      if (id >= 0 && id < 3) {
        blowLevel[id] = lvl;
        updateBlowText(id);
      }
    }
  }
}
// ================== SETUP & LOOP ==================
void setup() {
  Serial.begin(9600, SERIAL_8N1, 16, 17);  // Debug serial & komunikasi ke CTRL
  nexSerial.begin(9600, SERIAL_8N1, 25, 26);
  nexInit();
  delay(1500);  // Tunggu CTRL siap
  Serial.println("CMD_REQ_SYNC");
  // Attach Callbacks
  // page0.attachPop(page0PopCallback, &page0);
  // page1.attachPop(page1PopCallback, &page1);
  // page2.attachPop(page2PopCallback, &page2);
  // b1.attachPop(b1PopCallback, &b1);

  // Settings
  bTempUp.attachPop(bTempUpPopCallback, &bTempUp);
  bTempDown.attachPop(bTempDownPopCallback, &bTempDown);
  bTimeUp.attachPop(bTimeUpPopCallback, &bTimeUp);
  bTimeDown.attachPop(bTimeDownPopCallback, &bTimeDown);
  bWeightUp.attachPop(bWeightUpPopCallback, &bWeightUp);
  bWeightDown.attachPop(bWeightDownPopCallback, &bWeightDown);
  bSave.attachPop(bSavePopCallback, &bSave);
  bZero.attachPop(bZeroPopCallback, &bZero);
  bSet.attachPop(bSetPopCallback, &bSet);
  bWall.attachPop(bWallPopCallback, &bWall);
  bSpeed.attachPop(bSpeedPopCallback, &bSpeed);
  nWeightSet.attachPop(nWeightSetPopCallback, &nWeightSet);
  nTimeSet.attachPop(nTimeSetPopCallback, &nTimeSet);

  // Motor Selectors
  bSelDoor.attachPop(bSelDoorPopCallback, &bSelDoor);
  bSelBurn.attachPop(bSelBurnPopCallback, &bSelBurn);
  bSelFeed.attachPop(bSelFeedPopCallback, &bSelFeed);
  bSelBDoor.attachPop(bSelBDoorPopCallback, &bSelBDoor);
  bSelAsh.attachPop(bSelAshPopCallback, &bSelAsh);
  bSelMainC.attachPop(bSelMainCPopCallback, &bSelMainC);

  hSpeed.attachPop(hSpeedPopCallback, &hSpeed);
  hStep.attachPop(hStepPopCallback, &hStep);

  // Blower
  bBlow1.attachPop(bBlow1PopCallback, &bBlow1);
  bBlow2.attachPop(bBlow2PopCallback, &bBlow2);

  // Dashboard
  btAuto.attachPop(btAutoPopCallback, &btAuto);
  bStop.attachPop(bStopPopCallback, &bStop);
  bManStop.attachPop(bStopPopCallback, &bManStop);
  bsettingStop.attachPop(bStopPopCallback, &bsettingStop);

  // Manual
  btManDir.attachPop(btManDirPopCallback, &btManDir);
  bManDoor.attachPop(bManDoorPopCallback, &bManDoor);
  bManBDoor.attachPop(bManBDoorPopCallback, &bManBDoor);
  bManBurn.attachPop(bManBurnPopCallback, &bManBurn);
  bManPush.attachPop(bManPushPopCallback, &bManPush);
  bManAsh.attachPop(bManAshPopCallback, &bManAsh);
  bManMainC.attachPop(bManMainCPopCallback, &bManMainC);
  hManSpeed.attachPop(hManSpeedPopCallback, &hManSpeed);
  hManStep.attachPop(hManStepPopCallback, &hManStep);
  btManIgn.attachPop(btManIgnPopCallback, &btManIgn);
  btManFeed.attachPop(btManFeedPopCallback, &btManFeed);
  btManWeigh.attachPop(btManWeighPopCallback, &btManWeigh);
  b0.attachPop(b0PopCallback, &b0);
}

void loop() {

  nexLoop(nex_listen_list);
  // Serial.println("aaa");

  ParseDataFromCTRL();

  updateHMI();  // Refresh tampilan dengan data variabel terbaru
}