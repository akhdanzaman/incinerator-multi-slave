#include <Arduino.h>

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
}
