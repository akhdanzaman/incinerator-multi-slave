

// =========================
// proto_app.h
// =========================
#pragma once
#include <Arduino.h>
#include "RS485Bus.h"
#include "RS485Codec.h"
#include "node_ids.h"

// Kita bikin “App Protocol” di atas RS485Bus frame.
// Pola:
// - CMD: payload[0]=cmdId, sisanya data
// - REQ: payload[0]=reqId
// - RSP: payload[0]=RSP_OK/ERR/BUSY, payload[1]=echo req/cmd id, sisanya data
// - TELEM: payload[0]=telemId, sisanya struct
// - EVT: payload[0]=evtId, sisanya struct

// Tambahan ID aplikasi (di luar library RS485Config.h)
enum AppTelemId : uint8_t {
  TLM_MEGA   = 0x21,
  TLM_RTD    = 0x22,
  TLM_HX     = 0x23,
  TLM_LIDAR  = 0x24,
};

enum AppEvtId : uint8_t {
  EVT_FAULT      = 0x31,
  EVT_LIMIT_TRIP = 0x32,
  EVT_OVERTEMP   = 0x33,
};

// System states (mirror dari Mega enum lamamu)
enum SystemState : uint8_t {
  ST_IDLE = 0,
  ST_WEIGHING,
  ST_OPENING_MAIN_DOOR,
  ST_DUMPING_IN,
  ST_DUMPING_OUT,
  ST_CLOSING_MAIN_DOOR,
  ST_OPENING_BURN_DOOR,
  ST_BURNER_IN,
  ST_IGNITING,
  ST_BURNING,
  ST_BURNER_OUT,
  ST_CLOSING_BURN_DOOR,
  ST_FAULT,
  ST_PREHEAT
};

#pragma pack(push, 1)

// --- Telemetry structs (HARUS muat <= 95 byte setelah header telemId) ---
struct TelemMega {
  uint32_t ms;
  uint8_t  state;           // SystemState
  int16_t  tempC_x1;        // C
  int32_t  weight_g;
  uint32_t batchVol_cm3;
  uint32_t totalVol_cm3;

  uint8_t  fuel_pct;
  uint16_t fuel_l_x10;
  uint32_t totalConsumed_g;
  uint32_t batchCount;

  int32_t  remaining_s;     // -1 if none
  uint8_t  remainingIsPreheat;
  uint8_t  progress_pct;
  uint16_t faultCode;
};

struct TelemRtd {
  uint32_t ms;
  int16_t  tempC_x1;
  uint16_t limitMask; // 3 limit bits
  uint16_t faultCode;
};

struct TelemHx {
  uint32_t ms;
  int32_t  weight_g;
  uint16_t limitMask; // 1 limit bit
  uint16_t faultCode;
};

struct TelemLidar {
  uint32_t ms;
  uint32_t batchVol_cm3;
  uint32_t totalVol_cm3;
  uint16_t status;
  uint16_t faultCode;
};

// --- Events (opsional) ---
struct EvtFault {
  uint32_t ms;
  uint16_t faultCode;
  uint16_t detail;
  uint8_t  source;
};

#pragma pack(pop)

// Helpers encode payload to bus
namespace AppProto {

// CMD helpers
bool sendCmd_SetAuto(RS485Bus& bus, uint8_t dst, bool on);
bool sendCmd_EStop(RS485Bus& bus, uint8_t dst);
bool sendCmd_OneShot(RS485Bus& bus, uint8_t dst);

bool sendCmd_Setpoints(RS485Bus& bus, uint8_t dst, int16_t tempC, uint32_t burnerTime_s,
                       float minW_kg, float minV_cm3);

bool sendCmd_Tare(RS485Bus& bus, uint8_t dst);
bool sendCmd_CalScale(RS485Bus& bus, uint8_t dst);
bool sendCmd_CalWall(RS485Bus& bus, uint8_t dst);
bool sendCmd_CalSpeed(RS485Bus& bus, uint8_t dst);

bool sendCmd_ManualDir(RS485Bus& bus, uint8_t dst, uint8_t open);
bool sendCmd_ReqMove(RS485Bus& bus, uint8_t dst, uint8_t motorId);
bool sendCmd_SetManSpeed(RS485Bus& bus, uint8_t dst, uint32_t speed);
bool sendCmd_SetManStep(RS485Bus& bus, uint8_t dst, int32_t steps);

bool sendCmd_ManIgn(RS485Bus& bus, uint8_t dst, uint8_t on);
bool sendCmd_ManBurn(RS485Bus& bus, uint8_t dst, uint8_t on);
bool sendCmd_ManFeed(RS485Bus& bus, uint8_t dst, uint8_t on);
bool sendCmd_ManWeigh(RS485Bus& bus, uint8_t dst, uint8_t on);

bool sendCmd_MotorSpeed(RS485Bus& bus, uint8_t dst, uint8_t motorId, uint32_t speed);
bool sendCmd_MotorSteps(RS485Bus& bus, uint8_t dst, uint8_t motorId, int32_t steps);
bool sendCmd_SaveSettings(RS485Bus& bus, uint8_t dst);

// REQ helpers
bool sendReq_Ping(RS485Bus& bus, uint8_t dst);
bool sendReq_Status(RS485Bus& bus, uint8_t dst);
bool sendReq_TelemSnapshot(RS485Bus& bus, uint8_t dst);

} // namespace AppProto
