#pragma once
#include <Wire.h>
#include <Adafruit_SSD1306.h>
#include "settings.h"
#include "pins.h"
#include "status.h"

// =============================================================
//  UMouse S3 — sensors.h
//  IR diferencial (4 sensores: FL, L, R, FR), VBAT, OLED
//  Misma técnica del original: OFF/ON alternado con emisor por LEDC.
// =============================================================

static const float VBAT_DIV = 4.0f;       // LECTURA_REAL = ADC * 4 (igual que el original)

// ── LEDC IR ──────────────────────────────────────────────────
// 4 emisores IR + 4 pines de motor = 8 canales = TODOS los del ESP32-S3.
static const int IR_NSAMPLES = 12;
static const int IR_ON_US    = 250;
static const int IR_OFF_US   = 250;
static const int IR_LED_DUTY = 180;

// ── OLED ─────────────────────────────────────────────────────
#define OLED_W     128
#define OLED_H     64
#define OLED_ADDR  0x3C

extern Adafruit_SSD1306 display;
extern bool g_oledOK;

// ── ESTADO IR ────────────────────────────────────────────────
int irFL = 0, irL = 0, irR = 0, irFR = 0;

// ── VBAT ─────────────────────────────────────────────────────
inline float readVBAT_V(){
  return (analogReadMilliVolts(VBAT_PIN) * VBAT_DIV) / 1000.0f;
}

inline int calcDutyMax(float vbat){
  if(vbat < 1.0f) return 30;
  float d = 255.0f * (V_MOTOR_LIMIT / vbat);
  return (int)constrain(d, 0.0f, 255.0f);
}

float readVBAT_filtered(){
  uint32_t acc = 0;
  for(int i = 0; i < 16; i++){ acc += analogReadMilliVolts(VBAT_PIN); delay(2); }
  return ((float)(acc/16) * VBAT_DIV) / 1000.0f;
}

// ── IR ───────────────────────────────────────────────────────
int readDiff_mV(uint8_t ledPin, uint8_t adcPin){
  uint32_t offAcc = 0, onAcc = 0;
  for(int i = 0; i < IR_NSAMPLES; i++){
    ledcWrite(ledPin, 0);           delayMicroseconds(IR_OFF_US);
    offAcc += analogReadMilliVolts(adcPin);
    ledcWrite(ledPin, IR_LED_DUTY); delayMicroseconds(IR_ON_US);
    onAcc  += analogReadMilliVolts(adcPin);
    ledcWrite(ledPin, 0);           delayMicroseconds(IR_OFF_US);
  }
  int diff = (int)(offAcc/IR_NSAMPLES) - (int)(onAcc/IR_NSAMPLES);
#if IR_USE_ABS_DIFF
  if(diff < 0) diff = -diff;
#else
  if(diff < 0) diff = 0;
#endif
  return diff;
}

inline void readIRFront(){
  irFL = readDiff_mV(PIN_IR_FL, PIN_FT_FL);
  irFR = readDiff_mV(PIN_IR_FR, PIN_FT_FR);
}
void readIR(){                        // todos (páginas OLED)
  readIRFront();
  irL = readDiff_mV(PIN_IR_L, PIN_FT_L);
  irR = readDiff_mV(PIN_IR_R, PIN_FT_R);
}

inline bool wallFrontFromReadings(){
#if IR_FRONT_REQUIRE_BOTH
  return irFL >= IR_WALL_THR_FL && irFR >= IR_WALL_THR_FR;
#else
  return irFL >= IR_WALL_THR_FL || irFR >= IR_WALL_THR_FR;
#endif
}

// Igual que el original: cada consulta mide justo antes de decidir.
inline bool hasWallLeft()  { irL = readDiff_mV(PIN_IR_L, PIN_FT_L); return irL >= IR_WALL_THR_L; }
inline bool hasWallRight() { irR = readDiff_mV(PIN_IR_R, PIN_FT_R); return irR >= IR_WALL_THR_R; }
inline bool hasWallFront() { readIRFront(); return wallFrontFromReadings(); }

// ── OLED ─────────────────────────────────────────────────────
void oledHeader(const char* t){
  if(!g_oledOK) return;
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0,0);
  display.println(t);
}

void oledShow(const char* l1, const char* l2=""){
  if(!g_oledOK) return;
  oledHeader(l1);
  display.setCursor(0,12);
  display.println(l2);
  display.display();
}

// ── INIT ─────────────────────────────────────────────────────
void sensorsInit(){
  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);
  analogSetPinAttenuation(VBAT_PIN,  ADC_11db);
  analogSetPinAttenuation(PIN_FT_FL, ADC_11db);
  analogSetPinAttenuation(PIN_FT_L,  ADC_11db);
  analogSetPinAttenuation(PIN_FT_R,  ADC_11db);
  analogSetPinAttenuation(PIN_FT_FR, ADC_11db);

  ledcAttach(PIN_IR_FL, 20000, 8); ledcWrite(PIN_IR_FL, 0);
  ledcAttach(PIN_IR_L,  20000, 8); ledcWrite(PIN_IR_L,  0);
  ledcAttach(PIN_IR_R,  20000, 8); ledcWrite(PIN_IR_R,  0);
  ledcAttach(PIN_IR_FR, 20000, 8); ledcWrite(PIN_IR_FR, 0);
}
