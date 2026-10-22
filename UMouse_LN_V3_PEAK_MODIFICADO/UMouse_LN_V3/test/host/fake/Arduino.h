// Arduino.h simulado para pruebas en PC (solo lo que usan status.h / pins.h).
#pragma once
#include <cstdint>
#include <cstring>
#define HIGH 1
#define LOW 0
#define OUTPUT 1
extern int  g_pinLevel[64];
extern int  g_pinWrites[64];
extern uint8_t g_rgb[3];
inline void pinMode(uint8_t, int){}
inline void digitalWrite(uint8_t p, int v){ g_pinLevel[p] = v; g_pinWrites[p]++; }
inline void neopixelWrite(uint8_t, uint8_t r, uint8_t g, uint8_t b){ g_rgb[0]=r; g_rgb[1]=g; g_rgb[2]=b; }
