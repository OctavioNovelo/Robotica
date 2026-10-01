#pragma once
#include <Arduino.h>
#include <Wire.h>
#include <math.h>
#include <Adafruit_BNO08x.h>
#include "settings.h"
#include "pins.h"

// =============================================================
//  UMouse S3 — bno.h   BNO085 (Adafruit_BNO08x 1.2.x, I2C)
//
//  Alimentación/boot según Resumen_BNO085_Octavio.pdf:
//    GPIO38 = BNO_PWR (gate IRLZ44N, corte de GND). LOW=apagado, HIGH=encendido.
//    El constructor usa -1: la librería NO toca ningún pin de reset.
//    Sin tráfico I2C mientras el BNO está apagado.
// =============================================================

static const uint8_t BNO_ADDR_A = 0x4A;
static const uint8_t BNO_ADDR_B = 0x4B;
static const uint8_t BNO_MAX_EVENTS = 8;

static Adafruit_BNO08x bno08x(-1);
static sh2_SensorValue_t bnoEvent;

// ── Ángulos ──────────────────────────────────────────────────
inline float wrap360(float a){            // [0, 360)
  a = fmodf(a, 360.0f);
  if(a < 0.0f) a += 360.0f;
  if(a >= 360.0f) a -= 360.0f;
  return a;
}
inline float wrap180(float a){            // [-180, +180)   359° -> 0° = +1°, no -359°
  a = fmodf(a + 180.0f, 360.0f);
  if(a < 0.0f) a += 360.0f;
  return a - 180.0f;
}

// ── Estado ───────────────────────────────────────────────────
enum BnoState : uint8_t { BNO_ST_OFF = 0, BNO_ST_INIT, BNO_ST_OK, BNO_ST_ERROR };

struct BnoData {
  BnoState state   = BNO_ST_OFF;
  uint8_t  addr    = 0;
  float yawRaw = 0, pitch = 0, roll = 0;     // grados (yaw antihorario +)
  float yawRef = 0;                          // yaw raw que corresponde a heading 0
  float gyroZ  = 0;                          // rad/s (antihorario +)
  float linAx = 0, linAy = 0, linAz = 0;     // m/s^2
  uint8_t  accuracy   = 0;                   // 0-3 (bits 1-0 de status del reporte)
  uint32_t lastYawMs  = 0;
  uint16_t resets     = 0;                   // resets inesperados detectados
  uint8_t  recoveries = 0;                   // power-cycles en runtime
  bool     needRebase = false;               // el BNO se reinició: yaw ya no es válido
  const char* err     = "";
};
static BnoData g_bno;

// ── Encendido / apagado ──────────────────────────────────────
inline void bnoBootPowerCycle(){
#if BNO_USE_PWR_GATE
  pinMode(PIN_BNO_PWR, OUTPUT);
  digitalWrite(PIN_BNO_PWR, LOW);       // 1) BNO apagado
  pinMode(PIN_BNO_INT, INPUT);          // 2) INT como entrada (sin pull-up: BNO sin GND)
  delay(BNO_OFF_MS);                    // 3) apagado real
  digitalWrite(PIN_BNO_PWR, HIGH);      // 4) encender (conecta GND)
  delay(BNO_ON_SETTLE_MS);              // 5) arranque limpio
#else
  pinMode(PIN_BNO_INT, INPUT);
  pinMode(PIN_BNO_PWR, OUTPUT);         // modo RST directo (esquemático anterior)
  digitalWrite(PIN_BNO_PWR, HIGH); delay(10);
  digitalWrite(PIN_BNO_PWR, LOW);  delay(20);
  digitalWrite(PIN_BNO_PWR, HIGH);
  delay(BNO_ON_SETTLE_MS);
#endif
}

inline void i2cBegin(){                 // pasos 6 del PDF: I2C DESPUÉS de energizar
  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(400000);
}

// ── Reportes ─────────────────────────────────────────────────
inline bool bnoSetReports(){
  bool ok = true;
  // Game rotation vector: sin magnetómetro (mejor cerca de motores/imanes)
  ok &= bno08x.enableReport(SH2_GAME_ROTATION_VECTOR,  BNO_REPORT_US);
  ok &= bno08x.enableReport(SH2_GYROSCOPE_CALIBRATED,  BNO_REPORT_US);
  ok &= bno08x.enableReport(SH2_LINEAR_ACCELERATION,   BNO_ACCEL_US);
  return ok;
}

inline void quatToEulerDeg(float qr, float qi, float qj, float qk,
                           float &yaw, float &pitch, float &roll){
  float sqi = qi*qi, sqj = qj*qj, sqk = qk*qk;
  float r = atan2f(2.0f*(qr*qi + qj*qk), 1.0f - 2.0f*(sqi + sqj));
  float t = 2.0f*(qr*qj - qk*qi);
  if(t >  1.0f) t =  1.0f;
  if(t < -1.0f) t = -1.0f;
  float p = asinf(t);
  float y = atan2f(2.0f*(qr*qk + qi*qj), 1.0f - 2.0f*(sqj + sqk));
  yaw   = y * 180.0f / PI;
  pitch = p * 180.0f / PI;
  roll  = r * 180.0f / PI;
}

// ── Init ─────────────────────────────────────────────────────
// Llamar DESPUÉS de bnoBootPowerCycle() + i2cBegin().
inline bool bnoInit(){
  g_bno.state = BNO_ST_INIT;
  g_bno.err   = "";
  bool found = false;
  for(uint8_t t = 0; t < BNO_INIT_TRIES && !found; t++){
    if(bno08x.begin_I2C(BNO_ADDR_A, &Wire))      { found = true; g_bno.addr = BNO_ADDR_A; }
    else if(bno08x.begin_I2C(BNO_ADDR_B, &Wire)) { found = true; g_bno.addr = BNO_ADDR_B; }
    else delay(100);
  }
  if(!found){ g_bno.state = BNO_ST_ERROR; g_bno.err = "sin I2C"; return false; }
  if(!bnoSetReports()){ g_bno.state = BNO_ST_ERROR; g_bno.err = "reportes"; return false; }
  bno08x.wasReset();                    // descarta el flag del reset propio de begin_I2C
  g_bno.lastYawMs  = 0;
  g_bno.needRebase = false;
  g_bno.state = BNO_ST_OK;
  return true;
}

// ── Lectura ──────────────────────────────────────────────────
inline void bnoPoll(uint8_t maxEvents){
  if(g_bno.state != BNO_ST_OK) return;
  if(bno08x.wasReset()){                // reset inesperado: yaw vuelve a 0
    g_bno.resets++;
    g_bno.needRebase = true;
    if(!bnoSetReports()){ g_bno.state = BNO_ST_ERROR; g_bno.err = "reportes"; return; }
  }
  for(uint8_t i = 0; i < maxEvents; i++){
    if(!bno08x.getSensorEvent(&bnoEvent)) break;
    switch(bnoEvent.sensorId){
      case SH2_GAME_ROTATION_VECTOR:
        quatToEulerDeg(bnoEvent.un.gameRotationVector.real,
                       bnoEvent.un.gameRotationVector.i,
                       bnoEvent.un.gameRotationVector.j,
                       bnoEvent.un.gameRotationVector.k,
                       g_bno.yawRaw, g_bno.pitch, g_bno.roll);
        g_bno.accuracy  = bnoEvent.status & 0x03;
        g_bno.lastYawMs = millis();
        break;
      case SH2_GYROSCOPE_CALIBRATED:
        g_bno.gyroZ = bnoEvent.un.gyroscope.z;
        break;
      case SH2_LINEAR_ACCELERATION:
        g_bno.linAx = bnoEvent.un.linearAcceleration.x;
        g_bno.linAy = bnoEvent.un.linearAcceleration.y;
        g_bno.linAz = bnoEvent.un.linearAcceleration.z;
        break;
      default: break;
    }
  }
}
inline void bnoUpdate(){ bnoPoll(BNO_MAX_EVENTS); }
inline void bnoDrain() { bnoPoll(64); }

inline bool bnoFresh(){
  return g_bno.state == BNO_ST_OK && g_bno.lastYawMs != 0 &&
         (millis() - g_bno.lastYawMs) <= BNO_STALE_MS;
}

// Espera una muestra de yaw recibida a partir de ahora.
inline bool bnoWaitFresh(uint32_t maxMs){
  uint32_t t0 = millis();
  do {
    if(g_bno.state != BNO_ST_OK) return false;
    bnoPoll(BNO_MAX_EVENTS);
    if(g_bno.lastYawMs != 0 && g_bno.lastYawMs >= t0) return true;
    delay(2);
  } while(millis() - t0 < maxMs);
  return false;
}

inline const char* bnoStatusStr(){
  switch(g_bno.state){
    case BNO_ST_OFF:   return "OFF";
    case BNO_ST_INIT:  return "INIT";
    case BNO_ST_ERROR: return "ERR";
    default:           return bnoFresh() ? "OK" : "STALE";
  }
}

// ── Heading (compás: N=0° = dirección de arranque, horario = +) ──
inline float bnoHeading(){ return wrap360(BNO_HEADING_SIGN * (g_bno.yawRaw - g_bno.yawRef)); }
inline float bnoYawRel() { return wrap180(BNO_HEADING_SIGN * (g_bno.yawRaw - g_bno.yawRef)); }
inline float bnoRateDegS(){ return BNO_HEADING_SIGN * g_bno.gyroZ * (180.0f / PI); }   // + = giro a la derecha

// Define que el yaw actual equivale a `headingDeg`
inline void bnoRebaseTo(float headingDeg){ g_bno.yawRef = g_bno.yawRaw - BNO_HEADING_SIGN * headingDeg; }
inline void bnoZeroHeading(){ bnoRebaseTo(0.0f); }

// ── Recuperación en runtime (robot DETENIDO; bloquea ~1 s) ───
// Power-cycle real. Deja el I2C libre mientras el BNO está apagado.
inline bool bnoRecover(){
  if(g_bno.recoveries >= BNO_MAX_RECOVERIES){ g_bno.state = BNO_ST_ERROR; g_bno.err = "sin recup."; return false; }
  g_bno.recoveries++;
  g_bno.state = BNO_ST_INIT;
  Wire.end();
  bnoBootPowerCycle();
  i2cBegin();
  if(!bnoInit()) return false;
  g_bno.needRebase = true;               // yaw reinició en 0: hay que re-referenciar
  return true;
}
