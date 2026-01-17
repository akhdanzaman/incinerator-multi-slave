
// =========================
// File: RS485Bus.cpp
// =========================
#include "RS485Bus.h"

RS485Bus::RS485Bus(Stream& serial, int dePin, bool deActiveHigh)
  : _s(serial), _dePin(dePin), _deActiveHigh(deActiveHigh) {}

void RS485Bus::begin(uint32_t /*baud*/) {
  pinMode(_dePin, OUTPUT);
  _setTx(false);
  _resetRx();
}

void RS485Bus::_setTx(bool on) {
  // RS485 transceiver: DE (driver enable). Banyak modul mengikat RE ke DE.
  // on=true => transmit
  // on=false => receive
  if (_dePin < 0) return;
  uint8_t v = (on ? 1 : 0);
  if (!_deActiveHigh) v = !v;
  digitalWrite(_dePin, v);
}

void RS485Bus::_resetRx() {
  _rxState = RX_WAIT_P1;
  _rxIdx = 0;
  _rxLen = 0;
  _rxCrc = 0;
  _rxLastByteMs = millis();
}

void RS485Bus::_handleRxTimeout(uint32_t now) {
  if ((uint32_t)(now - _rxLastByteMs) > (uint32_t)RS485BUS_RX_TIMEOUT_MS) {
    // timeout: drop partial frame
    if (_rxState != RX_WAIT_P1) {
      _stats.rx_drop++;
    }
    _resetRx();
  }
}

uint16_t RS485Bus::crc16_modbus(const uint8_t* data, size_t len) {
  // CRC-16/Modbus (poly 0xA001, init 0xFFFF)
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < len; i++) {
    crc ^= data[i];
    for (uint8_t b = 0; b < 8; b++) {
      if (crc & 0x0001) crc = (crc >> 1) ^ 0xA001;
      else crc >>= 1;
    }
  }
  return crc;
}

bool RS485Bus::send(uint8_t dst, uint8_t type, const uint8_t* payload, uint8_t len, uint8_t* outSeq) {
  if (len > RS485BUS_MAX_PAYLOAD) {
    _stats.tx_fail++;
    return false;
  }

#if !RS485BUS_ALLOW_REPLY_TO_BROADCAST
  // just a guard rail: biasanya broadcast tidak perlu ack
  // (library ini tidak implement ack, tapi guard ini menghindari pola jelek)
#endif

  uint8_t seq = ++_seq;
  if (outSeq) *outSeq = seq;

  // Build frame in stack buffer (small)
  // total = 2 pre + 6 header(VER..SEQ) + LEN + payload + 2 CRC
  const uint16_t total = (uint16_t)(2 + 6 + 1 + len + 2);
  uint8_t buf[2 + 6 + 1 + RS485BUS_MAX_PAYLOAD + 2];

  uint16_t i = 0;
  buf[i++] = RS485BUS_PREAMBLE_1;
  buf[i++] = RS485BUS_PREAMBLE_2;
  buf[i++] = RS485BUS_VERSION;
  buf[i++] = _nodeId;
  buf[i++] = dst;
  buf[i++] = type;
  buf[i++] = seq;
  buf[i++] = len;
  for (uint8_t k = 0; k < len; k++) buf[i++] = payload[k];

  // CRC from VER..PAYLOAD
  uint16_t crc = crc16_modbus(&buf[2], (size_t)(6 + 1 + len));
  buf[i++] = (uint8_t)(crc & 0xFF);
  buf[i++] = (uint8_t)((crc >> 8) & 0xFF);

  // TX
  _setTx(true);
  // kecil: tulis sekaligus
  size_t w = _s.write(buf, total);
  _s.flush();
  _setTx(false);

  if (w != total) {
    _stats.tx_fail++;
    return false;
  }

  _stats.tx_ok++;
  return true;
}

bool RS485Bus::sendRsp(uint8_t dst, uint8_t seq, const uint8_t* payload, uint8_t len) {
  if (len > RS485BUS_MAX_PAYLOAD) {
    _stats.tx_fail++;
    return false;
  }

  // Build frame with same seq (reply)
  const uint16_t total = (uint16_t)(2 + 6 + 1 + len + 2);
  uint8_t buf[2 + 6 + 1 + RS485BUS_MAX_PAYLOAD + 2];

  uint16_t i = 0;
  buf[i++] = RS485BUS_PREAMBLE_1;
  buf[i++] = RS485BUS_PREAMBLE_2;
  buf[i++] = RS485BUS_VERSION;
  buf[i++] = _nodeId;
  buf[i++] = dst;
  buf[i++] = RS485_MSG_RSP;
  buf[i++] = seq;
  buf[i++] = len;
  for (uint8_t k = 0; k < len; k++) buf[i++] = payload[k];

  uint16_t crc = crc16_modbus(&buf[2], (size_t)(6 + 1 + len));
  buf[i++] = (uint8_t)(crc & 0xFF);
  buf[i++] = (uint8_t)((crc >> 8) & 0xFF);

  _setTx(true);
  size_t w = _s.write(buf, total);
  _s.flush();
  _setTx(false);

  if (w != total) {
    _stats.tx_fail++;
    return false;
  }

  _stats.tx_ok++;
  return true;
}

void RS485Bus::poll() {
  uint32_t now = millis();
  _handleRxTimeout(now);

  while (_s.available() > 0) {
    int r = _s.read();
    if (r < 0) break;
    uint8_t b = (uint8_t)r;
    _rxLastByteMs = now;

    switch (_rxState) {
      case RX_WAIT_P1:
        if (b == RS485BUS_PREAMBLE_1) _rxState = RX_WAIT_P2;
        break;

      case RX_WAIT_P2:
        if (b == RS485BUS_PREAMBLE_2) _rxState = RX_WAIT_VER;
        else _rxState = RX_WAIT_P1;
        break;

      case RX_WAIT_VER:
        _rxVer = b;
        if (_rxVer != RS485BUS_VERSION) {
          _stats.rx_drop++;
          _resetRx();
        } else {
          _rxState = RX_WAIT_SRC;
        }
        break;

      case RX_WAIT_SRC:
        _rxSrc = b;
        _rxState = RX_WAIT_DST;
        break;

      case RX_WAIT_DST:
        _rxDst = b;
        _rxState = RX_WAIT_TYPE;
        break;

      case RX_WAIT_TYPE:
        _rxType = b;
        _rxState = RX_WAIT_SEQ;
        break;

      case RX_WAIT_SEQ:
        _rxSeq = b;
        _rxState = RX_WAIT_LEN;
        break;

      case RX_WAIT_LEN:
        _rxLen = b;
        if (_rxLen > RS485BUS_MAX_PAYLOAD) {
          _stats.rx_len_fail++;
          _resetRx();
        } else if (_rxLen == 0) {
          _rxIdx = 0;
          _rxState = RX_WAIT_CRC_L;
        } else {
          _rxIdx = 0;
          _rxState = RX_WAIT_PAYLOAD;
        }
        break;

      case RX_WAIT_PAYLOAD:
        _rxPayload[_rxIdx++] = b;
        if (_rxIdx >= _rxLen) {
          _rxState = RX_WAIT_CRC_L;
        }
        break;

      case RX_WAIT_CRC_L:
        _rxCrc = (uint16_t)b;
        _rxState = RX_WAIT_CRC_H;
        break;

      case RX_WAIT_CRC_H: {
        _rxCrc |= (uint16_t)b << 8;

        // Validate dest: accept if dst = me or broadcast
        bool dstOk = (_rxDst == _nodeId) || (_rxDst == RS485BUS_BROADCAST_ID);
        if (!dstOk) {
          _stats.rx_drop++;
          _resetRx();
          break;
        }

        // Compute CRC
        // Build scratch header [VER..LEN] + payload
        uint8_t hdr[7];
        hdr[0] = _rxVer;
        hdr[1] = _rxSrc;
        hdr[2] = _rxDst;
        hdr[3] = _rxType;
        hdr[4] = _rxSeq;
        hdr[5] = _rxLen;
        // NOTE: Di sender, CRC dihitung atas 6(header: VER,SRC,DST,TYPE,SEQ) + LEN + payload
        // Itu total 7 + payload

        uint16_t crc = 0xFFFF;
        // feed hdr
        crc = crc16_modbus(hdr, 6); // VER..SEQ (6 bytes)
        // feed LEN
        crc = crc16_modbus((const uint8_t*)&hdr[5], 1) ^ 0; // placeholder, kita akan hitung ulang proper di bawah

        // Cara aman: hitung sekali dari buffer linear
        // linear = [VER,SRC,DST,TYPE,SEQ,LEN,PAYLOAD]
        uint8_t lin[6 + 1 + RS485BUS_MAX_PAYLOAD];
        lin[0] = _rxVer;
        lin[1] = _rxSrc;
        lin[2] = _rxDst;
        lin[3] = _rxType;
        lin[4] = _rxSeq;
        lin[5] = _rxLen;
        for (uint8_t k = 0; k < _rxLen; k++) lin[6 + k] = _rxPayload[k];
        crc = crc16_modbus(lin, (size_t)(6 + 1 + _rxLen));

        if (crc != _rxCrc) {
          _stats.rx_crc_fail++;
          _resetRx();
          break;
        }

        _stats.rx_ok++;

        // Callback
        if (_cb) {
          (void)_cb(_rxSrc, _rxDst, _rxType, _rxSeq, _rxPayload, _rxLen, _cbUser);
        }

        _resetRx();
        break;
      }

      default:
        _resetRx();
        break;
    }
  }
}

