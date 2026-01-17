
// =========================
// master_sm.cpp
// =========================
#include "master_sm.h"
#include "node_ids.h"
#include "config.h"

void MasterSM::begin() {
  lastSetPushMs_ = 0;
}

bool MasterSM::telemetryStale(const TelemetryStore& tlm) {
  uint32_t now = millis();
  bool mega = (now - tlm.lastRxMegaMs) > TELEMETRY_STALE_MS;
  bool rtd  = (now - tlm.lastRxRtdMs)  > TELEMETRY_STALE_MS;
  bool hx   = (now - tlm.lastRxHxMs)   > TELEMETRY_STALE_MS;
  // lidar tidak kritikal buat safety, tapi boleh dipakai
  return (mega || rtd || hx);
}

void MasterSM::pushSetpoints(TelemetryStore& tlm, RS485Bus& bus) {
  AppProto::sendCmd_Setpoints(bus, NODE_SLAVE1_MEGA_ACT,
                             (int16_t)tlm.tempSetpoint,
                             tlm.burnerActiveTimeSec,
                             tlm.minBatchWeight,
                             tlm.volSetpoint);
}

void MasterSM::onUiEvent(const MasterEvent& e, TelemetryStore& tlm, RS485Bus& bus) {
  switch (e.type) {
    case EVT_UI_AUTO_TOGGLE: {
      tlm.autoEnabled = (e.v != 0);
      AppProto::sendCmd_SetAuto(bus, NODE_SLAVE1_MEGA_ACT, tlm.autoEnabled);
    } break;

    case EVT_UI_ESTOP:
      AppProto::sendCmd_EStop(bus, NODE_SLAVE1_MEGA_ACT);
      break;

    case EVT_UI_ONE_SHOT:
      AppProto::sendCmd_OneShot(bus, NODE_SLAVE1_MEGA_ACT);
      break;

    case EVT_UI_TARE:
      AppProto::sendCmd_Tare(bus, NODE_SLAVE1_MEGA_ACT);
      break;

    case EVT_UI_CAL_SCALE:
      AppProto::sendCmd_CalScale(bus, NODE_SLAVE1_MEGA_ACT);
      break;

    case EVT_UI_CAL_WALL:
      AppProto::sendCmd_CalWall(bus, NODE_SLAVE1_MEGA_ACT);
      break;

    case EVT_UI_CAL_SPEED:
      AppProto::sendCmd_CalSpeed(bus, NODE_SLAVE1_MEGA_ACT);
      break;

    case EVT_UI_SETPOINTS_CHANGED:
      pushSetpoints(tlm, bus);
      break;

    case EVT_UI_MOTOR_SPEED_CHANGED:
      // e.a=motorId, e.v=speedPct (1..100) -> mapping speed real biar konsisten dengan HMI lama
      // Di master ini kita kirim “real speed” ke Mega. Mega yang punya mapping detail.
      // Kalau kamu mau mapping sama persis, nanti kita pindahin tabel MOTOR_MAX/MIN ke master.
      AppProto::sendCmd_MotorSpeed(bus, NODE_SLAVE1_MEGA_ACT, e.a, (uint32_t)e.v);
      break;

    case EVT_UI_MOTOR_STEP_CHANGED:
      AppProto::sendCmd_MotorSteps(bus, NODE_SLAVE1_MEGA_ACT, e.a, (int32_t)(e.v * 100));
      break;

    case EVT_UI_MANUAL_DIR_CHANGED:
      tlm.manualDirOpen = (uint8_t)e.a;
      AppProto::sendCmd_ManualDir(bus, NODE_SLAVE1_MEGA_ACT, tlm.manualDirOpen);
      break;

    case EVT_UI_MANUAL_MOVE:
      tlm.manualMotorIdx = e.a;
      AppProto::sendCmd_ReqMove(bus, NODE_SLAVE1_MEGA_ACT, tlm.manualMotorIdx);
      break;

    case EVT_UI_MANUAL_IGN:
      AppProto::sendCmd_ManIgn(bus, NODE_SLAVE1_MEGA_ACT, (uint8_t)e.a);
      break;

    case EVT_UI_MANUAL_BURN:
      AppProto::sendCmd_ManBurn(bus, NODE_SLAVE1_MEGA_ACT, (uint8_t)e.a);
      break;

    case EVT_UI_MANUAL_FEED:
      AppProto::sendCmd_ManFeed(bus, NODE_SLAVE1_MEGA_ACT, (uint8_t)e.a);
      break;

    case EVT_UI_MANUAL_WEIGH:
      AppProto::sendCmd_ManWeigh(bus, NODE_SLAVE1_MEGA_ACT, (uint8_t)e.a);
      break;

    default:
      break;
  }
}

void MasterSM::tick(TelemetryStore& tlm, RS485Bus& bus) {
  uint32_t now = millis();

  // Safety: kalau telemetry kritikal stale, minta Mega safe-stop.
  if (telemetryStale(tlm)) {
    AppProto::sendCmd_EStop(bus, NODE_SLAVE1_MEGA_ACT);
  }

  // Periodic setpoint push (anti lost-frame)
  if (now - lastSetPushMs_ > SETPOINT_PUSH_MS) {
    lastSetPushMs_ = now;
    pushSetpoints(tlm, bus);
  }
}