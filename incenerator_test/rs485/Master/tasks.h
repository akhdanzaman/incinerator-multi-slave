
// =========================
// tasks.h
// =========================
#pragma once
#include <Arduino.h>
#include "RS485Bus.h"
#include "telemetry_store.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

// Push decoded incoming frame as event
static bool onRs485Frame(uint8_t src, uint8_t dst, uint8_t type, uint8_t seq,
                         const uint8_t* payload, uint8_t len, void* user) {
  (void)dst; (void)seq;
  if (!payload || len == 0) return true;

  MasterEvent e{};
  e.type = EVT_RS485_FRAME;
  e.a = src;
  e.b = type;
  e.c = len;
  // v unused
  (void)xQueueSend(qEvt, &e, 0);

  // Parsing dilakukan langsung di callback biar telemetry update real-time.
  // Aman karena ringan.
  if (type == RS485_MSG_TELEM && len >= 1) {
    uint8_t tid = payload[0];
    const uint8_t* p = &payload[1];
    uint8_t plen = (uint8_t)(len - 1);

    uint32_t now = millis();
    if (tid == TLM_MEGA && plen >= sizeof(TelemMega)) {
      memcpy(&gTlm.mega, p, sizeof(TelemMega));
      gTlm.lastRxMegaMs = now;
    } else if (tid == TLM_RTD && plen >= sizeof(TelemRtd)) {
      memcpy(&gTlm.rtd, p, sizeof(TelemRtd));
      gTlm.lastRxRtdMs = now;
    } else if (tid == TLM_HX && plen >= sizeof(TelemHx)) {
      memcpy(&gTlm.hx, p, sizeof(TelemHx));
      gTlm.lastRxHxMs = now;
    } else if (tid == TLM_LIDAR && plen >= sizeof(TelemLidar)) {
      memcpy(&gTlm.lidar, p, sizeof(TelemLidar));
      gTlm.lastRxLidarMs = now;
    }
  }
  if (type == RS485_MSG_RSP && len >= 2) {
    uint8_t st  = payload[0];   // RSP_OK / ERR / BUSY
    uint8_t rid = payload[1];   // echo reqId
    const uint8_t* p = &payload[2];
    uint8_t plen = len - 2;

    if (st == RSP_OK) {
      uint32_t now = millis();
      if (rid == APP_REQ_TELEM_MEGA && plen >= sizeof(TelemMega)) { ... }
      ...
    }
  }

  return true;
}

// Internal events from HMI
enum MasterEventType : uint8_t {
  EVT_UI_AUTO_TOGGLE = 1,
  EVT_UI_ESTOP,
  EVT_UI_ONE_SHOT,
  EVT_UI_TARE,
  EVT_UI_CAL_SCALE,
  EVT_UI_CAL_WALL,
  EVT_UI_CAL_SPEED,

  EVT_UI_SETPOINTS_CHANGED,
  EVT_UI_MOTOR_SPEED_CHANGED,
  EVT_UI_MOTOR_STEP_CHANGED,

  EVT_UI_MANUAL_DIR_CHANGED,
  EVT_UI_MANUAL_MOVE,
  EVT_UI_MANUAL_IGN,
  EVT_UI_MANUAL_BURN,
  EVT_UI_MANUAL_FEED,
  EVT_UI_MANUAL_WEIGH,

  EVT_RS485_FRAME
};

#pragma pack(push,1)
struct MasterEvent {
  uint8_t type;
  uint8_t a;
  uint8_t b;
  uint8_t c;
  int32_t v;
};
#pragma pack(pop)

#pragma pack(push,1)
struct TxJob {
  uint8_t dst;
  uint8_t type; // Rs485MsgType
  uint8_t len;
  uint8_t payload[RS485BUS_MAX_PAYLOAD];
};
#pragma pack(pop)

extern QueueHandle_t qEvt;
extern QueueHandle_t qTx;

// Task entry points
void task_rs485_poll(void*);
void task_poll_scheduler(void*);
void task_state_machine(void*);
void task_ui_loop(void*);
void task_ui_render(void*);