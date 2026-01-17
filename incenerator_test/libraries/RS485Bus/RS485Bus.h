
// =========================
// File: RS485Bus.h
// =========================
#pragma once

#include <Arduino.h>
#include "RS485Config.h"

class RS485Bus {
public:
  // Callback: dipanggil saat frame valid diterima.
  // Return true kalau frame sudah diproses.
  typedef bool (*RxCallback)(uint8_t src, uint8_t dst, uint8_t type, uint8_t seq,
                             const uint8_t* payload, uint8_t len, void* user);

  RS485Bus(Stream& serial, int dePin, bool deActiveHigh = true);

  void begin(uint32_t baud);

  void setNodeId(uint8_t id) { _nodeId = id; }
  uint8_t nodeId() const { return _nodeId; }

  void setCallback(RxCallback cb, void* user = nullptr) {
    _cb = cb;
    _cbUser = user;
  }

  // Non-blocking poll. Panggil sesering mungkin.
  void poll();

  // Kirim frame generic
  bool send(uint8_t dst, uint8_t type, const uint8_t* payload, uint8_t len, uint8_t* outSeq = nullptr);

  // Helper: kirim CMD / REQ / RSP
  bool sendCmd(uint8_t dst, const uint8_t* payload, uint8_t len, uint8_t* outSeq = nullptr) {
    return send(dst, RS485_MSG_CMD, payload, len, outSeq);
  }
  bool sendReq(uint8_t dst, const uint8_t* payload, uint8_t len, uint8_t* outSeq = nullptr) {
    return send(dst, RS485_MSG_REQ, payload, len, outSeq);
  }
  bool sendRsp(uint8_t dst, uint8_t seq, const uint8_t* payload, uint8_t len);
  bool sendEvt(uint8_t dst, const uint8_t* payload, uint8_t len, uint8_t* outSeq = nullptr) {
    return send(dst, RS485_MSG_EVT, payload, len, outSeq);
  }
  bool sendTelem(uint8_t dst, const uint8_t* payload, uint8_t len, uint8_t* outSeq = nullptr) {
    return send(dst, RS485_MSG_TELEM, payload, len, outSeq);
  }

  // Utility: CRC16 (Modbus/IBM) untuk interoperabilitas
  static uint16_t crc16_modbus(const uint8_t* data, size_t len);

  // Optional: stats
  struct Stats {
    uint32_t rx_ok = 0;
    uint32_t rx_crc_fail = 0;
    uint32_t rx_len_fail = 0;
    uint32_t rx_drop = 0;
    uint32_t tx_ok = 0;
    uint32_t tx_fail = 0;
  };
  const Stats& stats() const { return _stats; }

private:
  Stream& _s;
  int _dePin;
  bool _deActiveHigh;

  uint8_t _nodeId = 0x01;
  uint8_t _seq = 0;

  RxCallback _cb = nullptr;
  void* _cbUser = nullptr;

  // RX parser state
  enum RxState : uint8_t {
    RX_WAIT_P1,
    RX_WAIT_P2,
    RX_WAIT_VER,
    RX_WAIT_SRC,
    RX_WAIT_DST,
    RX_WAIT_TYPE,
    RX_WAIT_SEQ,
    RX_WAIT_LEN,
    RX_WAIT_PAYLOAD,
    RX_WAIT_CRC_L,
    RX_WAIT_CRC_H,
  };

  RxState _rxState = RX_WAIT_P1;
  uint32_t _rxLastByteMs = 0;

  // current frame fields
  uint8_t _rxVer = 0;
  uint8_t _rxSrc = 0;
  uint8_t _rxDst = 0;
  uint8_t _rxType = 0;
  uint8_t _rxSeq = 0;
  uint8_t _rxLen = 0;
  uint8_t _rxPayload[RS485BUS_MAX_PAYLOAD];
  uint8_t _rxIdx = 0;
  uint16_t _rxCrc = 0;

  Stats _stats;

  void _setTx(bool on);
  void _resetRx();
  void _handleRxTimeout(uint32_t now);
};
