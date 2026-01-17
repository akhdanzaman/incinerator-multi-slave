/*
ESP1 MASTER (ESP32) = HMI + High-level State + RS485 Bus Controller
===============================================================
Kamu minta:
- Master = ESP1: Nextion HMI + state machine + kontrol semua slave via RS485
- Slave1 = Arduino Mega: aktuator stepper/relay + safety interlock (authoritative)
- Slave2 = ESP2: MAX31865 + 3 limit switch
- Slave3 = ESP3: HX711 + 1 limit switch
- Slave4 = ESP4: RPLidar C1 (olah volume di slave, kirim volume telemetry)

Dan kamu juga minta:
- Pakai library RS485Bus yang kamu kasih (preamble A5 5A + CRC16 Modbus)
- Hindari blocking: pecah jadi FreeRTOS tasks
- Reuse object Nextion dari kode lamamu

Catatan penting (biar nggak kejadian “kok nggak nyambung?”):
1) Di library RS485Bus kamu, frame TYPE sudah dibedakan (CMD/REQ/RSP/EVT/TELEM).
   Payload pertama biasanya [cmdId] atau [reqId].
2) Karena bus half-duplex, *master harus polling* (round-robin) agar tidak tabrakan.
   Slave boleh kirim EVT/TELEM periodik, tapi sebaiknya tetap “polite”: kirim saat diminta.

Project ini adalah skeleton modular. Kamu tinggal copy per file.

FILE TREE
---------
/ESP1_MASTER
  ESP1_MASTER.ino
  pins.h
  config.h
  node_ids.h
  proto_app.h
  proto_app.cpp
  telemetry_store.h
  telemetry_store.cpp
  hmi_nextion.h
  hmi_nextion.cpp
  master_sm.h
  master_sm.cpp
  tasks.h
  tasks.cpp

Dependencies:
- Nextion library (ITEAD)
- FreeRTOS sudah built-in di ESP32 Arduino
- RS485Bus.h / RS485Codec.h / RS485Config.h (library milikmu)
*/

// =========================
// ESP1_MASTER.ino
// =========================
#include <Arduino.h>

#include "pins.h"
#include "config.h"
#include "node_ids.h"

#include "RS485Bus.h"
#include "RS485Config.h"
#include "RS485Codec.h"

#include "telemetry_store.h"
#include "hmi_nextion.h"
#include "tasks.h"
#include "master_sm.h"

// RS485 bus on Serial1 (kamu bisa ganti ke Serial2 kalau bentrok)
HardwareSerial& RS485Serial = Serial1;
RS485Bus gBus(RS485Serial, PIN_RS485_DE, true);

TelemetryStore gTlm;
HMI gHmi;
MasterSM gSM;

void setup() {
#if MASTER_DEBUG
  Serial.begin(115200);
  delay(200);
  Serial.println("ESP1 MASTER BOOT");
#endif

  // RS485 UART
  RS485Serial.begin(RS485_BAUD, SERIAL_8N1, PIN_RS485_RX, PIN_RS485_TX);
  gBus.begin(RS485_BAUD);
  gBus.setNodeId(NODE_MASTER);
  gBus.setCallback(onRs485Frame, nullptr);



  // queues
  qEvt = xQueueCreate(Q_EVT_LEN, sizeof(MasterEvent));
  qTx  = xQueueCreate(Q_TX_LEN, sizeof(TxJob));

  // HMI
  gHmi.setEventQueue(qEvt);
  gHmi.bindTelemetry(&gTlm);
  gHmi.begin();

  // SM
  gSM.begin();

  // Tasks
  xTaskCreatePinnedToCore(task_rs485_poll, "rs485", 4096, nullptr, 5, nullptr, 1);
  xTaskCreatePinnedToCore(task_poll_scheduler, "poll", 4096, nullptr, 3, nullptr, 1);
  xTaskCreatePinnedToCore(task_ui_loop, "ui", 4096, nullptr, 2, nullptr, 0);
  xTaskCreatePinnedToCore(task_ui_render, "render", 4096, nullptr, 1, nullptr, 0);
  xTaskCreatePinnedToCore(task_state_machine, "sm", 4096, nullptr, 4, nullptr, 1);

#if MASTER_DEBUG
  Serial.println("Tasks started");
#endif
}

void loop() {
  // sengaja kosong. Semua di FreeRTOS tasks.
  vTaskDelay(pdMS_TO_TICKS(1000));
}

/*
====================================================
BUG/NOTE LIST (biar kamu nggak keburu marah):
====================================================
1) Beberapa command di AppProto masih placeholder karena RS485Config.h kamu belum punya cmdId spesifik
   untuk: ONE_SHOT, TARE, CAL_SCALE, CAL_WALL, CAL_SPEED, MANUAL_DIR, SAVE_SETTINGS, dll.
   Solusi paling bersih: tambahin enum Rs485CmdId baru di RS485Config.h.
   Kalau kamu mau, nanti aku rapihin daftar cmdId final + payload format supaya slave-side rapi.

2) Telemetry parsing di atas mengandalkan slave mengirim TELEM frame dengan payload:
      [telemId][struct bytes...]
   Jadi di masing-masing slave, kamu tinggal:
      uint8_t pl[1+sizeof(TelemMega)]; pl[0]=TLM_MEGA; memcpy(&pl[1], &tlm, sizeof(tlm));
      bus.sendTelem(NODE_MASTER, pl, sizeof(pl));

3) Queue qTx belum dipakai (karena kita langsung send lewat bus). Kalau kamu nanti mau rate-limit / retry,
   pindahin semua sendCmd/sendReq ke task khusus TX (biar ada backoff kalau bus sibuk).

4) onRs485Frame didefinisikan di tasks.cpp. Karena Arduino build kadang rese soal forward declare,
   kalau IDE kamu rewel, pindahin onRs485Frame ke file .ino atau bikin header tasks_internal.h.
*/
