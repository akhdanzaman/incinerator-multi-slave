
// =========================
// telemetry_store.h
// =========================
#pragma once
#include <Arduino.h>
#include "proto_app.h"

struct TelemetryStore {
  TelemMega  mega{};
  TelemRtd   rtd{};
  TelemHx    hx{};
  TelemLidar lidar{};

  uint32_t lastRxMegaMs = 0;
  uint32_t lastRxRtdMs  = 0;
  uint32_t lastRxHxMs   = 0;
  uint32_t lastRxLidarMs= 0;

  // Master-owned setpoints (mirror UI)
  float tempSetpoint = 1200.0f;
  uint32_t burnerActiveTimeSec = 30;
  float minBatchWeight = 5.0f;
  float volSetpoint = 20000.0f;

  // Manual
  uint8_t manualMotorIdx = 0;
  uint8_t manualDirOpen = 1;
  uint32_t manualSpeed = 1000;
  int32_t manualSteps = 1000;

  bool autoEnabled = false;
};