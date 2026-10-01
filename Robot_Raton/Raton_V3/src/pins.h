#pragma once
#include <Arduino.h>

// =============================================================
//  UMouse S3 — pins.h
//  ÚNICA fuente del pinout (ESP32-S3 YD, verificado contra el esquemático).
//  Notas de verificación al final del archivo.
// =============================================================

// ── Motores DRV8871 (MR = derecho, ML = izquierdo; CA/CB = encoder) ──
static const uint8_t MR_IN2 = 14;
static const uint8_t MR_IN1 = 13;
static const uint8_t MR_CA  = 12;
static const uint8_t MR_CB  = 11;

// ¡OJO! El esquemático rotula ML_IN1=GPIO5 y ML_IN2=GPIO6 (al revés).
// Se usa el mapeo escrito + sketch de pruebas S3 (ML_IN1=6, ML_IN2=5).
// Si en la prueba de motores el izquierdo gira al revés: intercambiar estas dos líneas.
static const uint8_t ML_IN2 = 5;
static const uint8_t ML_IN1 = 6;
static const uint8_t ML_CA  = 7;
static const uint8_t ML_CB  = 15;

// Alias con los nombres del código original
#define ENC_L_A  ML_CA
#define ENC_L_B  ML_CB
#define ENC_R_A  MR_CA
#define ENC_R_B  MR_CB

// ── Batería ──────────────────────────────────────────────────
static const uint8_t VBAT_PIN = 4;        // ADC1_CH3

// ── Sensores IR (IR = emisor IR383 vía 2N2222, FT = fototransistor PT1302) ──
static const uint8_t PIN_IR_FR = 9;   static const uint8_t PIN_FT_FR = 10;
static const uint8_t PIN_IR_R  = 40;  static const uint8_t PIN_FT_R  = 2;
static const uint8_t PIN_IR_FL = 17;  static const uint8_t PIN_FT_FL = 8;
static const uint8_t PIN_IR_L  = 39;  static const uint8_t PIN_FT_L  = 1;

// ── LEDs de debug ────────────────────────────────────────────
static const uint8_t PIN_LED_RED   = 21;
static const uint8_t PIN_LED_BLUE  = 47;
// El texto de la especificación decía 18, pero GPIO18 es BNO_INT (esquemático:
// WLED = I/O16, INT = I/O18; el sketch de pruebas también usa 16).
static const uint8_t PIN_LED_WHITE = 16;

// ── ARGB integrado (WS2812 de la YD-ESP32-S3) ────────────────
// Placa YD-ESP32-S3: WS2812 en GPIO48. En el esquemático I/O48 está sin
// conexión (X), así que no choca con nada. GPIO38 es del BNO (NO es el ARGB).
static const uint8_t PIN_ARGB = 48;

// ── I2C compartido OLED + BNO085 ─────────────────────────────
static const uint8_t I2C_SDA = 41;
static const uint8_t I2C_SCL = 42;

// ── BNO085 ───────────────────────────────────────────────────
static const uint8_t PIN_BNO_INT = 18;    // salida del BNO -> entrada ESP32 (INPUT, sin pull-up)
static const uint8_t PIN_BNO_PWR = 38;    // gate IRLZ44N (HIGH = BNO encendido). Con
                                          // BNO_USE_PWR_GATE=0 es el pin RST del BNO.
// ── Botón ────────────────────────────────────────────────────
#define BOOT_PIN 0                        // el mismo del proyecto original
