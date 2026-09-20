#pragma once
#include <Wire.h>
#include "settings.h"
#include "sensors.h"
#include "motion.h"

// =============================================================
//  UMouse — diagnostics.h
//
//  Utilidades de diagnóstico portadas de UMouse_S3_Test_BootMotor.ino:
//  escaneo de bus I2C y secuencia de prueba de motores con lectura
//  de RPM por encoder. Pensado para usarse desde páginas de la OLED
//  (ver PAGE_MOTOR / PAGE_BNO en main.cpp), no forma parte de la
//  navegación real del floodfill.
// =============================================================

// ── ESTADO DE PRUEBA DE MOTOR ────────────────────────────────
int   g_motorTestPercent = 50;     // % del límite seguro actual (g_dutyMax) a usar en la prueba
int   g_motorCmdL = 0, g_motorCmdR = 0;
const char *g_motorState = "COAST";
float g_lastRpmL = 0.0f, g_lastRpmR = 0.0f;
long  g_lastMotorTicksL = 0, g_lastMotorTicksR = 0;

static const uint32_t MOTOR_TEST_MS    = 800;
static const uint32_t MOTOR_BRAKE_MS   = 180;
static const uint32_t MOTOR_COAST_MS   = 250;

// ── ESCANEO I2C ───────────────────────────────────────────────
// Recorre las 127 direcciones I2C posibles. Si printSerial=true,
// además de contar imprime cada dirección encontrada por Serial
// (útil para confirmar OLED en 0x3C y BNO085 en 0x4A/0x4B).
uint8_t scanI2C(bool printSerial){
  uint8_t count = 0;
  if(printSerial) Serial.println("\nI2C scan:");

  for(uint8_t addr = 1; addr < 127; addr++){
    Wire.beginTransmission(addr);
    uint8_t err = Wire.endTransmission();
    if(err == 0){
      count++;
      if(printSerial) Serial.printf("  encontrado: 0x%02X\n", addr);
    }
  }

  if(printSerial) Serial.printf("Total I2C: %u dispositivo(s)\n\n", count);
  return count;
}

// ── RPM POR ENCODER ───────────────────────────────────────────
float ticksToRPM(long absTicks, uint32_t durationMs){
  if(durationMs == 0 || ENC_TICKS_PER_REV <= 0.0f) return 0.0f;
  float rev = (float)absTicks / ENC_TICKS_PER_REV;
  return rev * 60000.0f / (float)durationMs;
}

// PWM (0-g_dutyMax) a usar en la prueba manual de motores, según el
// % elegido con g_motorTestPercent (ajustable, por ejemplo, con +/-
// desde una página de diagnóstico).
int calcMotorTestPWM(){
  int pwm = (g_dutyMax * g_motorTestPercent + 50) / 100;
  return ci(pwm, MOTOR_KICK_MIN, g_dutyMax);
}

// Prototipo — lo define main.cpp para refrescar el OLED en vivo
// durante la prueba (declarado aquí para no crear dependencia circular).
void drawMotorTestStage(const char *stage, int pwm, uint32_t elapsedMs, uint32_t totalMs);

// Corre ambos motores a (leftPWM,rightPWM) durante ms milisegundos,
// mide ticks/RPM por encoder, y frena+coastea al final (a menos que
// brakeAtEnd=false). Actualiza g_motorState/g_motorCmdL/R para que la
// página de diagnóstico pueda mostrar el estado en vivo.
void motorTimedTest(const char *label, int leftPWM, int rightPWM, uint32_t ms, bool brakeAtEnd = true){
  resetEncoders();
  g_motorState = label;
  uint32_t t0 = millis();

  while(millis() - t0 < ms){
    motorSetBoth(leftPWM, rightPWM);
    g_motorCmdL = leftPWM;
    g_motorCmdR = rightPWM;
    drawMotorTestStage(label, max(abs(leftPWM), abs(rightPWM)), millis()-t0, ms);
    delay(5);
  }

  long l,lA,r,rA;
  getEncAll(l,lA,r,rA);
  g_lastMotorTicksL = lA;
  g_lastMotorTicksR = rA;
  g_lastRpmL = ticksToRPM(lA, ms);
  g_lastRpmR = ticksToRPM(rA, ms);

  if(brakeAtEnd){
    motorBrakeAll(); delay(MOTOR_BRAKE_MS);
    motorCoastAll(); delay(MOTOR_COAST_MS);
  } else {
    motorCoastAll();
  }
  g_motorCmdL = 0; g_motorCmdR = 0; g_motorState = "COAST";
}

// Secuencia completa de prueba: coast, adelante, freno+coast, reversa,
// freno+coast, freno, coast final. Pensada para dispararse con un
// hold de BOOT en una página de diagnóstico de motores.
void motorFullSequenceTest(){
  int pwm = calcMotorTestPWM();

  drawMotorTestStage("INICIANDO", pwm, 0, 0);
  delay(350);

  motorCoastAll(); delay(MOTOR_COAST_MS);
  motorTimedTest("ADELANTE", +pwm, +pwm, MOTOR_TEST_MS, true);
  motorTimedTest("REVERSA",  -pwm, -pwm, MOTOR_TEST_MS, true);

  motorBrakeAll(); delay(MOTOR_BRAKE_MS);
  motorCoastAll(); delay(MOTOR_COAST_MS);
}
