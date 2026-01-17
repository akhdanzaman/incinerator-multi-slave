// =========================
// node_ids.h
// =========================
#pragma once
#include <Arduino.h>

// Node IDs untuk RS485Bus (SRC/DST)
static const uint8_t NODE_MASTER = 0x01;
static const uint8_t NODE_SLAVE1_MEGA_ACT = 0x02;
static const uint8_t NODE_SLAVE2_RTD_LIM  = 0x03;
static const uint8_t NODE_SLAVE3_HX_LIM   = 0x04;
static const uint8_t NODE_SLAVE4_LIDAR    = 0x05;