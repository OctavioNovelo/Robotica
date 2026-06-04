#pragma once
#include <Wire.h>
#include <Adafruit_SSD1306.h>
#include "settings.h"

// =============================================================
//  UMouse — sensors.h (Versión ESP32-S3 + Raw Telemetry)
// =============================================================

// ── PINES IR 4 PARES (ESP32-S3) ───────────────────────────────
#define LED_HW_L   39 
#define FT_HW_L    1  

#define LED_HW_R   40 
#define FT_HW_R    2  

#define LED_HW_FL  17 
#define FT_HW_FL   8  

#define LED_HW_FR  9  
#define FT_HW_FR   10 

// ── BATERÍA ──────────────────────────────────────────────────
#define VBAT_PIN  4 
static const float VBAT_DIV = 4.316f; 

// ── LEDC IR ──────────────────────────────────────────────────
static const int IR_NSAMPLES = 10;
static const int IR_ON_US    = 250;
static const int IR_OFF_US   = 250;

// ── OLED ─────────────────────────────────────────────────────
#define OLED_W     128
#define OLED_H     64
#define OLED_ADDR  0x3C

extern Adafruit_SSD1306 display;

// ── ESTADO IR (Procesado y Crudo) ────────────────────────────
int irL = 0, irC = 0, irR = 0;
int irFL = 0, irFR = 0;

// Variables para guardar el RAW OFF/ON
int offL = 0, onL = 0;
int offR = 0, onR = 0;
int offFL = 0, onFL = 0;
int offFR = 0, onFR = 0;

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

// ── IR LÓGICA CRUDA ──────────────────────────────────────────
void readDiff_mV_Raw(uint8_t ledPin, uint8_t adcPin, int &outOff, int &outOn, int &outDiff){
  uint32_t offAcc = 0, onAcc = 0; 
  for(int i = 0; i < IR_NSAMPLES; i++){
    ledcWrite(ledPin, 0);   delayMicroseconds(IR_OFF_US);
    offAcc += analogReadMilliVolts(adcPin);
    ledcWrite(ledPin, 180); delayMicroseconds(IR_ON_US);
    onAcc  += analogReadMilliVolts(adcPin);
    ledcWrite(ledPin, 0);   delayMicroseconds(IR_OFF_US);
  }
  outOff = offAcc / IR_NSAMPLES;
  outOn  = onAcc / IR_NSAMPLES;
  int diff = outOff - outOn; 
  outDiff = (diff < 0) ? 0 : diff;
}

void readIR(){
  readDiff_mV_Raw(LED_HW_L,  FT_HW_L,  offL,  onL,  irL);
  readDiff_mV_Raw(LED_HW_R,  FT_HW_R,  offR,  onR,  irR);
  readDiff_mV_Raw(LED_HW_FL, FT_HW_FL, offFL, onFL, irFL);
  readDiff_mV_Raw(LED_HW_FR, FT_HW_FR, offFR, onFR, irFR);
  
  // Fusión central para Floodfill
  irC = (irFL > irFR) ? irFL : irFR;
}

inline bool hasWallLeft()  { readIR(); return irL >= IR_WALL_THR_L; }
inline bool hasWallRight() { readIR(); return irR >= IR_WALL_THR_R; }
inline bool hasWallFront() { readIR(); return (irFL >= IR_WALL_THR_FL || irFR >= IR_WALL_THR_FR); }

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
  analogSetPinAttenuation(FT_HW_R, ADC_11db);
  analogSetPinAttenuation(FT_HW_FL, ADC_11db);
  analogSetPinAttenuation(FT_HW_FR, ADC_11db);

  ledcAttach(LED_HW_L, 20000, 8);  ledcWrite(LED_HW_L, 0);
  ledcAttach(LED_HW_R, 20000, 8);  ledcWrite(LED_HW_R, 0);
  ledcAttach(LED_HW_FL, 20000, 8); ledcWrite(LED_HW_FL, 0);
  ledcAttach(LED_HW_FR, 20000, 8); ledcWrite(LED_HW_FR, 0);
}
