#pragma once
#include <Arduino.h>
#if __has_include(<esp_arduino_version.h>)
  #include <esp_arduino_version.h>
#endif
#include "pins.h"

// =============================================================
//  UMouse S3 — status.h
//  ARGB integrado (GPIO48) + LEDs de debug. Sin LEDC: los 8 canales
//  del ESP32-S3 ya están usados (4 motores + 4 emisores IR).
// =============================================================

#define ARGB_BRIGHT_DIV  2      // atenúa (los WS2812 de la placa deslumbran)

enum StatusId : uint8_t {
  ST_OFF = 0, ST_BOOT, ST_BNO_INIT, ST_BNO_OK,
  ST_MODE1, ST_MODE2, ST_MODE3,
  ST_STRAIGHT, ST_TURN, ST_WALL,
  ST_MAP_LOADED, ST_FLOOD, ST_ERROR, ST_NVS_CLEARED, ST_GOAL
};

static uint32_t g_argbLast = 0xFFFFFFFFu;

// Escribe el ARGB solo si el color cambió (rgbLedWrite tarda ~1 ms).
inline void setStatusColor(uint8_t r, uint8_t g, uint8_t b){
  uint32_t c = ((uint32_t)r << 16) | ((uint32_t)g << 8) | b;
  if(c == g_argbLast) return;
  g_argbLast = c;
  r /= ARGB_BRIGHT_DIV; g /= ARGB_BRIGHT_DIV; b /= ARGB_BRIGHT_DIV;
#if defined(ESP_ARDUINO_VERSION) && defined(ESP_ARDUINO_VERSION_VAL)
  #if ESP_ARDUINO_VERSION >= ESP_ARDUINO_VERSION_VAL(3,0,7)
    rgbLedWrite(PIN_ARGB, r, g, b);     // core >= 3.0.7 (verificado en el repo del core)
  #else
    neopixelWrite(PIN_ARGB, r, g, b);   // core 3.0.0-3.0.6
  #endif
#else
  neopixelWrite(PIN_ARGB, r, g, b);
#endif
}

inline void ledsDebug(bool red, bool blue, bool white){
  digitalWrite(PIN_LED_RED,   red   ? HIGH : LOW);
  digitalWrite(PIN_LED_BLUE,  blue  ? HIGH : LOW);
  digitalWrite(PIN_LED_WHITE, white ? HIGH : LOW);
}

inline void statusInit(){
  pinMode(PIN_LED_RED,   OUTPUT);
  pinMode(PIN_LED_BLUE,  OUTPUT);
  pinMode(PIN_LED_WHITE, OUTPUT);
  ledsDebug(false,false,false);
  setStatusColor(0,0,0);
}

// Un estado -> un color en el ARGB (+ LEDs de debug)
//   ARGB:  blanco=boot  amarillo=BNO init / floodfill  verde=BNO OK / recto
//          azul=NAV1  magenta=NAV2  cian=NAV3  naranja=giro  violeta=pared
//          verde-agua=mapa cargado  rojo=error  blanco fuerte=NVS borrada
//   LEDs:  blanco=boot/BNO/pared  azul=movimiento  rojo=error  todos=objetivo
inline void setStatus(StatusId s){
  switch(s){
    case ST_BOOT:       setStatusColor(80,80,80);  ledsDebug(false,false,true);  break;
    case ST_BNO_INIT:   setStatusColor(90,70,0);   ledsDebug(false,false,true);  break;
    case ST_BNO_OK:     setStatusColor(0,90,0);    ledsDebug(false,false,false); break;
    case ST_MODE1:      setStatusColor(0,0,110);   ledsDebug(false,false,false); break;
    case ST_MODE2:      setStatusColor(90,0,90);   ledsDebug(false,false,false); break;
    case ST_MODE3:      setStatusColor(0,90,90);   ledsDebug(false,false,false); break;
    case ST_STRAIGHT:   setStatusColor(0,110,0);   ledsDebug(false,true,false);  break;
    case ST_TURN:       setStatusColor(110,40,0);  ledsDebug(false,true,false);  break;
    case ST_WALL:       setStatusColor(60,0,110);  ledsDebug(false,false,true);  break;
    case ST_MAP_LOADED: setStatusColor(0,80,50);   ledsDebug(false,false,false); break;
    case ST_FLOOD:      setStatusColor(90,70,0);   ledsDebug(false,false,false); break;
    case ST_ERROR:      setStatusColor(120,0,0);   ledsDebug(true,false,false);  break;
    case ST_NVS_CLEARED:setStatusColor(120,120,120);ledsDebug(false,false,true); break;
    case ST_GOAL:       setStatusColor(0,120,40);  ledsDebug(true,true,true);    break;
    default:            setStatusColor(0,0,0);     ledsDebug(false,false,false); break;
  }
}
