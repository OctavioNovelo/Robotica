#pragma once
#include <Wire.h>
#include <Adafruit_SSD1306.h>
#include "settings.h"

// =============================================================
//  UMouse — sensors.h
//  IR diferencial, VBAT, OLED
// =============================================================

// ── PINES IR ─────────────────────────────────────────────────
#define LED_HW_R  13
#define LED_HW_C  14
#define LED_HW_L  26
#define FT_HW_R   35
#define FT_HW_C   34
#define FT_HW_L   39

#define VBAT_PIN  33
static const float VBAT_DIV = 4.0f;

// SWAP L/R por montaje físico
static const bool SWAP_LR = true;
inline uint8_t LED_L(){ return SWAP_LR ? LED_HW_R : LED_HW_L; }
inline uint8_t LED_R(){ return SWAP_LR ? LED_HW_L : LED_HW_R; }
inline uint8_t LED_C(){ return LED_HW_C; }
inline uint8_t FT_L() { return SWAP_LR ? FT_HW_R  : FT_HW_L; }
inline uint8_t FT_R() { return SWAP_LR ? FT_HW_L  : FT_HW_R; }
inline uint8_t FT_C() { return FT_HW_C; }

// ── LEDC IR ──────────────────────────────────────────────────
#define CH_IR_L  0
#define CH_IR_C  1
#define CH_IR_R  2
static const int IR_NSAMPLES = 12;
static const int IR_ON_US    = 250;
static const int IR_OFF_US   = 250;

// ── OLED ─────────────────────────────────────────────────────
#define OLED_W     128
#define OLED_H     64
#define OLED_ADDR  0x3C
#define I2C_SDA    21
#define I2C_SCL    22

extern Adafruit_SSD1306 display;

// ── ESTADO IR ────────────────────────────────────────────────
int irL = 0, irC = 0, irR = 0;

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
    ledcWrite(ledPin, 0);   delayMicroseconds(IR_OFF_US);
    offAcc += analogReadMilliVolts(adcPin);
    ledcWrite(ledPin, 180); delayMicroseconds(IR_ON_US);
    onAcc  += analogReadMilliVolts(adcPin);
    ledcWrite(ledPin, 0);   delayMicroseconds(IR_OFF_US);
  }
  int diff = (int)(offAcc/IR_NSAMPLES) - (int)(onAcc/IR_NSAMPLES);
  return (diff < 0) ? 0 : diff;
}

void readIR(){
  irL = readDiff_mV(LED_L(), FT_L());
  irC = readDiff_mV(LED_C(), FT_C());
  irR = readDiff_mV(LED_R(), FT_R());
}

inline bool hasWallLeft()  { readIR(); return irL >= IR_WALL_THR_L; }
inline bool hasWallRight() { readIR(); return irR >= IR_WALL_THR_R; }
inline bool hasWallFront() { readIR(); return irC >= IR_WALL_THR_C; }

// ── OLED ─────────────────────────────────────────────────────
void oledHeader(const char* t){
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0,0);
  display.println(t);
}

void oledShow(const char* l1, const char* l2=""){
  oledHeader(l1);
  display.setCursor(0,12);
  display.println(l2);
  display.display();
}

// ── INIT ─────────────────────────────────────────────────────
void sensorsInit(){
  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);
  analogSetPinAttenuation(VBAT_PIN, ADC_11db);
  analogSetPinAttenuation(FT_HW_L, ADC_11db);
  analogSetPinAttenuation(FT_HW_C, ADC_11db);
  analogSetPinAttenuation(FT_HW_R, ADC_11db);

  ledcAttachChannel(LED_HW_L, 20000, 8, CH_IR_L); ledcWrite(LED_HW_L, 0);
  ledcAttachChannel(LED_HW_C, 20000, 8, CH_IR_C); ledcWrite(LED_HW_C, 0);
  ledcAttachChannel(LED_HW_R, 20000, 8, CH_IR_R); ledcWrite(LED_HW_R, 0);
}
