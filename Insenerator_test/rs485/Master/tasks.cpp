
// =========================
// tasks.cpp
// =========================
#include "tasks.h"
#include "config.h"
#include "pins.h"
#include "node_ids.h"
#include "proto_app.h"

QueueHandle_t qEvt = NULL;
QueueHandle_t qTx  = NULL;

// Shared singletons (defined in .ino)
extern RS485Bus gBus;
extern TelemetryStore gTlm;



void task_rs485_poll(void*) {
  for (;;) {
    gBus.poll();
    vTaskDelay(pdMS_TO_TICKS(TASK_RS485_POLL_MS));
  }
}

void task_poll_scheduler(void*) {
  // Round-robin polling
  const uint8_t slaves[] = { NODE_SLAVE1_MEGA_ACT, NODE_SLAVE2_RTD_LIM, NODE_SLAVE3_HX_LIM, NODE_SLAVE4_LIDAR };
  uint8_t idx = 0;

  for (;;) {
    uint8_t dst = slaves[idx++ % (sizeof(slaves) / sizeof(slaves[0]))];
    // Mintalah telemetry snapshot
    (void)AppProto::sendReq_TelemSnapshot(gBus, dst);
    vTaskDelay(pdMS_TO_TICKS(TASK_POLL_SCHED_MS));
  }
}

// UI loop task dibuat terpisah biar nexLoop tidak mengganggu bus
extern class HMI gHmi;
void task_ui_loop(void*) {
  for (;;) {
    gHmi.loop();
    vTaskDelay(pdMS_TO_TICKS(TASK_UI_LOOP_MS));
  }
}

void task_ui_render(void*) {
  for (;;) {
    gHmi.render(gTlm);
    vTaskDelay(pdMS_TO_TICKS(TASK_UI_RENDER_MS));
  }
}

// State machine task: consume UI events + periodic tick
#include "master_sm.h"
extern MasterSM gSM;

void task_state_machine(void*) {
  uint32_t lastTick = millis();
  for (;;) {
    // Process UI events
    MasterEvent e;
    while (xQueueReceive(qEvt, &e, 0) == pdTRUE) {
      // UI event routing
      if (e.type != EVT_RS485_FRAME) {
        gSM.onUiEvent(e, gTlm, gBus);
      }
    }

    // Periodic tick
    uint32_t now = millis();
    if ((uint32_t)(now - lastTick) >= TASK_SM_TICK_MS) {
      lastTick = now;
      gSM.tick(gTlm, gBus);
    }

    vTaskDelay(pdMS_TO_TICKS(2));
  }
}