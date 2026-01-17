// =========================
// config.h
// =========================
#pragma once
#include <Arduino.h>

#define MASTER_DEBUG 1

static const uint32_t RS485_BAUD = 115200;
static const uint32_t NEX_BAUD   = 9600;

// Task periods
static const uint32_t TASK_RS485_POLL_MS   = 2;   // poll cepat, ringan
static const uint32_t TASK_POLL_SCHED_MS   = 25;  // round-robin request cadence
static const uint32_t TASK_UI_LOOP_MS      = 5;   // nexLoop
static const uint32_t TASK_UI_RENDER_MS    = 60;  // render ke Nextion
static const uint32_t TASK_SM_TICK_MS      = 20;  // state machine tick

static const uint32_t TELEMETRY_STALE_MS   = 1200;
static const uint32_t SETPOINT_PUSH_MS     = 400;

// Queue sizes
static const uint16_t Q_EVT_LEN = 32;
static const uint16_t Q_TX_LEN  = 32;