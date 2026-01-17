
// =========================
// File: RS485Codec.h  (helper payload encode/decode)
// =========================
#pragma once

#include <Arduino.h>

namespace RS485Codec {

static inline void put_u16_le(uint8_t* p, uint16_t v) {
  p[0] = (uint8_t)(v & 0xFF);
  p[1] = (uint8_t)((v >> 8) & 0xFF);
}

static inline void put_u32_le(uint8_t* p, uint32_t v) {
  p[0] = (uint8_t)(v & 0xFF);
  p[1] = (uint8_t)((v >> 8) & 0xFF);
  p[2] = (uint8_t)((v >> 16) & 0xFF);
  p[3] = (uint8_t)((v >> 24) & 0xFF);
}

static inline uint16_t get_u16_le(const uint8_t* p) {
  return (uint16_t)p[0] | ((uint16_t)p[1] << 8);
}

static inline uint32_t get_u32_le(const uint8_t* p) {
  return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

static inline void put_f32_le(uint8_t* p, float f) {
  union { float f; uint32_t u; } u;
  u.f = f;
  put_u32_le(p, u.u);
}

static inline float get_f32_le(const uint8_t* p) {
  union { float f; uint32_t u; } u;
  u.u = get_u32_le(p);
  return u.f;
}
}