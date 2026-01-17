
// =========================
// proto_app.cpp
// =========================
#include "proto_app.h"

namespace {
  static inline void put_u32(uint8_t* p, uint32_t v) { RS485Codec::put_u32_le(p, v); }
  static inline void put_u16(uint8_t* p, uint16_t v) { RS485Codec::put_u16_le(p, v); }
  static inline void put_f32(uint8_t* p, float f) { RS485Codec::put_f32_le(p, f); }
}

namespace AppProto {

bool sendCmd_SetAuto(RS485Bus& bus, uint8_t dst, bool on) {
  uint8_t pl[2] = { CMD_SET_AUTOMODE, (uint8_t)(on ? 1 : 0) };
  return bus.sendCmd(dst, pl, sizeof(pl));
}

bool sendCmd_EStop(RS485Bus& bus, uint8_t dst) {
  uint8_t pl[1] = { CMD_EMERGENCY_STOP };
  return bus.sendCmd(dst, pl, sizeof(pl));
}

bool sendCmd_OneShot(RS485Bus& bus, uint8_t dst) {
  // Kamu belum punya cmdId khusus ONE_SHOT di library, jadi kita pakai CMD_RESET_BATCH sebagai placeholder.
  // Nanti di Mega kamu bisa ganti mapping cmdId-nya.
  uint8_t pl[1] = { CMD_RESET_BATCH };
  return bus.sendCmd(dst, pl, sizeof(pl));
}

bool sendCmd_Setpoints(RS485Bus& bus, uint8_t dst, int16_t tempC, uint32_t burnerTime_s,
                       float minW_kg, float minV_cm3) {
  // mengikuti komentar lib: [cmd, temp(2), time(4), minW(4 float), minV(4 float)]
  uint8_t pl[1 + 2 + 4 + 4 + 4];
  pl[0] = CMD_SETPOINTS;
  put_u16(&pl[1], (uint16_t)tempC);
  put_u32(&pl[3], burnerTime_s);
  put_f32(&pl[7], minW_kg);
  put_f32(&pl[11], minV_cm3);
  return bus.sendCmd(dst, pl, sizeof(pl));
}

bool sendCmd_Tare(RS485Bus& bus, uint8_t dst) {
  uint8_t pl[1] = { CMD_TARE_SCALE };
  return bus.sendCmd(dst, pl, 1);
}

bool sendCmd_CalScale(RS485Bus& bus, uint8_t dst) {
  uint8_t pl[2] = { CMD_RESET_BATCH, 0xA1 };
  return bus.sendCmd(dst, pl, sizeof(pl));
}

bool sendCmd_CalWall(RS485Bus& bus, uint8_t dst) {
  uint8_t pl[2] = { CMD_RESET_BATCH, 0xA2 };
  return bus.sendCmd(dst, pl, sizeof(pl));
}

bool sendCmd_CalSpeed(RS485Bus& bus, uint8_t dst) {
  uint8_t pl[2] = { CMD_RESET_BATCH, 0xA3 };
  return bus.sendCmd(dst, pl, sizeof(pl));
}

bool sendCmd_ManualDir(RS485Bus& bus, uint8_t dst, uint8_t open) {
  uint8_t pl[2] = { CMD_SET_AUTOMODE, (uint8_t)(open ? 2 : 3) };
  // NOTE: placeholder. Idealnya kamu tambah cmdId baru: CMD_SET_MANUAL_DIR.
  return bus.sendCmd(dst, pl, sizeof(pl));
}

bool sendCmd_ReqMove(RS485Bus& bus, uint8_t dst, uint8_t motorId) {
  // pakai CMD_MOVE_MOTOR tapi steps=0 untuk request gerak sesuai config motorId (interpret di slave)
  uint8_t pl[1 + 1 + 1 + 4 + 4];
  pl[0] = CMD_MOVE_MOTOR;
  pl[1] = motorId;
  pl[2] = 0; // dir: slave pakai manualDirOpen
  put_u32(&pl[3], 0);
  put_u32(&pl[7], 0);
  return bus.sendCmd(dst, pl, sizeof(pl));
}

bool sendCmd_SetManSpeed(RS485Bus& bus, uint8_t dst, uint32_t speed) {
  uint8_t pl[1 + 1 + 1 + 4 + 4];
  pl[0] = CMD_MOVE_MOTOR;
  pl[1] = 0xFE; // special: set manual speed
  pl[2] = 0;
  put_u32(&pl[3], 0);
  put_u32(&pl[7], speed);
  return bus.sendCmd(dst, pl, sizeof(pl));
}

bool sendCmd_SetManStep(RS485Bus& bus, uint8_t dst, int32_t steps) {
  uint8_t pl[1 + 1 + 1 + 4 + 4];
  pl[0] = CMD_MOVE_MOTOR;
  pl[1] = 0xFD; // special: set manual steps
  pl[2] = 0;
  put_u32(&pl[3], (uint32_t)steps);
  put_u32(&pl[7], 0);
  return bus.sendCmd(dst, pl, sizeof(pl));
}

bool sendCmd_ManIgn(RS485Bus& bus, uint8_t dst, uint8_t on) {
  uint8_t pl[3] = { CMD_RELAY_SET, 0x01, (uint8_t)(on ? 1 : 0) };
  return bus.sendCmd(dst, pl, sizeof(pl));
}

bool sendCmd_ManBurn(RS485Bus& bus, uint8_t dst, uint8_t on) {
  uint8_t pl[3] = { CMD_RELAY_SET, 0x02, (uint8_t)(on ? 1 : 0) };
  return bus.sendCmd(dst, pl, sizeof(pl));
}

bool sendCmd_ManFeed(RS485Bus& bus, uint8_t dst, uint8_t on) {
  uint8_t pl[3] = { CMD_RELAY_SET, 0x03, (uint8_t)(on ? 1 : 0) };
  return bus.sendCmd(dst, pl, sizeof(pl));
}

bool sendCmd_ManWeigh(RS485Bus& bus, uint8_t dst, uint8_t on) {
  uint8_t pl[3] = { CMD_RELAY_SET, 0x04, (uint8_t)(on ? 1 : 0) };
  return bus.sendCmd(dst, pl, sizeof(pl));
}

bool sendCmd_MotorSpeed(RS485Bus& bus, uint8_t dst, uint8_t motorId, uint32_t speed) {
  // Use CMD_MOVE_MOTOR as config channel (placeholder). Better: tambah cmdId baru.
  uint8_t pl[1 + 1 + 1 + 4 + 4];
  pl[0] = CMD_MOVE_MOTOR;
  pl[1] = motorId;
  pl[2] = 0x7E; // marker "set speed"
  put_u32(&pl[3], 0);
  put_u32(&pl[7], speed);
  return bus.sendCmd(dst, pl, sizeof(pl));
}

bool sendCmd_MotorSteps(RS485Bus& bus, uint8_t dst, uint8_t motorId, int32_t steps) {
  uint8_t pl[1 + 1 + 1 + 4 + 4];
  pl[0] = CMD_MOVE_MOTOR;
  pl[1] = motorId;
  pl[2] = 0x7F; // marker "set steps"
  put_u32(&pl[3], (uint32_t)steps);
  put_u32(&pl[7], 0);
  return bus.sendCmd(dst, pl, sizeof(pl));
}

bool sendCmd_SaveSettings(RS485Bus& bus, uint8_t dst) {
  uint8_t pl[2] = { CMD_NOP, 0x55 }; // placeholder "save"
  return bus.sendCmd(dst, pl, sizeof(pl));
}

bool sendReq_Ping(RS485Bus& bus, uint8_t dst) {
  uint8_t pl[1] = { REQ_PING };
  return bus.sendReq(dst, pl, sizeof(pl));
}

bool sendReq_Status(RS485Bus& bus, uint8_t dst) {
  uint8_t pl[1] = { REQ_STATUS };
  return bus.sendReq(dst, pl, sizeof(pl));
}

bool sendReq_TelemSnapshot(RS485Bus& bus, uint8_t dst) {
  uint8_t pl[1] = { REQ_TELEM_SNAPSHOT };
  return bus.sendReq(dst, pl, sizeof(pl));
}

} // namespace AppProto
