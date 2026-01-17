

// =========================
// master_sm.h
// =========================
#pragma once
#include <Arduino.h>
#include "telemetry_store.h"
#include "tasks.h"
#include "proto_app.h"

class MasterSM {
public:
  void begin();
  void tick(TelemetryStore& tlm, RS485Bus& bus);

  void onUiEvent(const MasterEvent& e, TelemetryStore& tlm, RS485Bus& bus);

private:
  uint32_t lastSetPushMs_ = 0;

  void pushSetpoints(TelemetryStore& tlm, RS485Bus& bus);
  bool telemetryStale(const TelemetryStore& tlm);
};
