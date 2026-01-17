// =========================
// pins.h
// =========================
#pragma once
#include <Arduino.h>

// Nextion UART (sesuai sketch lamamu)
static const int PIN_NEX_RX = 25;
static const int PIN_NEX_TX = 26;

// RS485 UART (pakai UART2 biar aman dari Nextion)
// ESP32 punya Serial1/Serial2 tergantung core; di Arduino core:
// - Serial2 default pins bisa beda, tapi kita set explicit.
static const int PIN_RS485_RX = 16;
static const int PIN_RS485_TX = 17;
static const int PIN_RS485_DE = 4;   // DE+RE (tied) pin
