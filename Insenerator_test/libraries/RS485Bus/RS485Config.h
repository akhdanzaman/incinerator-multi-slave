
// =========================
// File: RS485Config.h
// =========================
#pragma once

#include <Arduino.h>

// ---- Frame format ----
// [0]  PRE1 = 0xA5
// [1]  PRE2 = 0x5A
// [2]  VER  = 0x01
// [3]  SRC  (node id)
// [4]  DST  (node id)  (0xFF=broadcast)
// [5]  TYPE (message type)
// [6]  SEQ  (sequence)
// [7]  LEN  (payload length, 0..RS485BUS_MAX_PAYLOAD)
// [8..] PAYLOAD
// [..] CRC16 (LSB,MSB)  computed from VER..PAYLOAD (bytes [2..(7+LEN)])

static const uint8_t RS485BUS_PREAMBLE_1 = 0xA5;
static const uint8_t RS485BUS_PREAMBLE_2 = 0x5A;
static const uint8_t RS485BUS_VERSION    = 0x01;

// Max payload: keep it kecil biar deterministik + RAM aman di AVR
#ifndef RS485BUS_MAX_PAYLOAD
#define RS485BUS_MAX_PAYLOAD 96
#endif

// Timeout parser (ms) saat frame nyangkut di tengah
#ifndef RS485BUS_RX_TIMEOUT_MS
#define RS485BUS_RX_TIMEOUT_MS 40
#endif

// Optional: drop frame jika broadcast + replyRequested
#ifndef RS485BUS_ALLOW_REPLY_TO_BROADCAST
#define RS485BUS_ALLOW_REPLY_TO_BROADCAST 0
#endif

// Node ids
static const uint8_t RS485BUS_BROADCAST_ID = 0xFF;

// Message types (kamu bisa tambah sesuai kebutuhan)
enum Rs485MsgType : uint8_t {
  RS485_MSG_CMD   = 0x01,  // Master -> Slave: command
  RS485_MSG_REQ   = 0x02,  // Master -> Slave: request data
  RS485_MSG_RSP   = 0x03,  // Slave  -> Master: response
  RS485_MSG_EVT   = 0x04,  // Slave  -> Master: event / alarm
  RS485_MSG_TELEM = 0x05,  // periodic telemetry
};

// Common command IDs (payload[0])
enum Rs485CmdId : uint8_t {
  CMD_EMERGENCY_STOP   = 0x01,
  CMD_SET_AUTOMODE     = 0x02,

  CMD_REQ_SYNC         = 0x10,
  CMD_ONE_SHOT_AUTO    = 0x11,

  CMD_SET_SETPOINTS    = 0x12, // payload: Telem/Setpoint struct
  CMD_TARE_SCALE       = 0x13,
  CMD_SET_SCALE        = 0x14,

  CMD_CAL_WALL         = 0x15,
  CMD_CAL_SPEED        = 0x16,

  CMD_SET_MANUAL_DIR   = 0x17, // payload: u8 dir
  CMD_REQ_MOVE         = 0x18, // payload: u8 motorId
  CMD_SET_M_SPD        = 0x19, // payload: u32 speed
  CMD_SET_M_STP        = 0x1A, // payload: i32 steps

  CMD_MOTOR_SPD        = 0x1B, // payload: u8 motorId + u32 speed
  CMD_MOTOR_STP        = 0x1C, // payload: u8 motorId + i32 steps

  CMD_SAVE_SETTINGS    = 0x1D,

  CMD_MAN_IGN          = 0x1E, // payload: u8 on/off
  CMD_MAN_BURN         = 0x1F,
  CMD_MAN_FEED         = 0x20,
  CMD_MAN_WEIGH        = 0x21
};

// Common request IDs (payload[0])
enum Rs485ReqId : uint8_t {
  REQ_PING           = 0x10, // payload: [req]
  REQ_STATUS         = 0x11, // payload: [req]
  REQ_TELEM_SNAPSHOT = 0x30 // payload: [telemId]
  
};

// Common response status (payload[0]) for RSP
enum Rs485RspStatus : uint8_t {
  RSP_OK  = 0x00,
  RSP_ERR = 0x01,
  RSP_BUSY= 0x02,
};

enum Rs485TelemId : uint8_t {
  TLM_MEGA  = 0x40,
  TLM_RTD   = 0x41,
  TLM_HX    = 0x42,
  TLM_LIDAR = 0x43
};
