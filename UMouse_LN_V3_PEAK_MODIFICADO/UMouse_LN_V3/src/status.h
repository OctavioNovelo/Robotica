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

// ── LEDs FÍSICOS = PAREDES DETECTADAS (exclusivo) ────────────
// Azul (GPIO47) = pared al FRENTE, Blanco (GPIO16) = pared a la IZQUIERDA,
// Rojo (GPIO21) = pared a la DERECHA. Independientes; reflejan la última detección
// IR vigente (ver g_irSnap en sensors.h), nunca el mapa NVS ni el estado general.
// setStatus() NO los toca. digitalWrite: sin canales LEDC.
static uint8_t g_wallLeds = 0xFF;       // bits: 1 = frente, 2 = izq, 4 = der (0xFF = desconocido)
inline void wallLedsSet(bool front, bool left, bool right){
  uint8_t m = (front ? 1 : 0) | (left ? 2 : 0) | (right ? 4 : 0);
  if(m == g_wallLeds) return;
  g_wallLeds = m;
  digitalWrite(PIN_LED_BLUE,  front ? HIGH : LOW);
  digitalWrite(PIN_LED_WHITE, left  ? HIGH : LOW);
  digitalWrite(PIN_LED_RED,   right ? HIGH : LOW);
}
inline void wallLedsOff(){ wallLedsSet(false, false, false); }

inline void statusInit(){
  pinMode(PIN_LED_RED,   OUTPUT);
  pinMode(PIN_LED_BLUE,  OUTPUT);
  pinMode(PIN_LED_WHITE, OUTPUT);
  wallLedsOff();
  setStatusColor(0,0,0);
}

// Un estado -> un color en el ARGB integrado (solo el ARGB: los LEDs físicos son de paredes)
//   blanco=boot  amarillo=BNO init  rosa=floodfill  verde=BNO OK / recto
//   azul=NAV1  magenta=NAV2  cian=NAV3  naranja=giro  violeta=pared
//   verde-agua=mapa cargado  rojo=error  blanco fuerte=NVS borrada
//   verde-azulado=meta (parpadea en la pantalla final, ver startRun)
inline void setStatus(StatusId s){
  switch(s){
    case ST_BOOT:       setStatusColor(80,80,80);  break;
    case ST_BNO_INIT:   setStatusColor(90,70,0);  break;
    case ST_BNO_OK:     setStatusColor(0,90,0); break;
    case ST_MODE1:      setStatusColor(0,0,110); break;
    case ST_MODE2:      setStatusColor(90,0,90); break;
    case ST_MODE3:      setStatusColor(0,90,90); break;
    case ST_STRAIGHT:   setStatusColor(0,110,0);  break;
    case ST_TURN:       setStatusColor(110,40,0);  break;
    case ST_WALL:       setStatusColor(60,0,110);  break;
    case ST_MAP_LOADED: setStatusColor(0,80,50); break;
    case ST_FLOOD:      setStatusColor(110,0,50);  break;   // antes (90,70,0) = ST_BNO_INIT
    case ST_ERROR:      setStatusColor(120,0,0);  break;
    case ST_NVS_CLEARED:setStatusColor(120,120,120); break;
    case ST_GOAL:       setStatusColor(0,120,40);    break;
    default:            setStatusColor(0,0,0); break;
  }
}
