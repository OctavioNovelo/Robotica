/*
  UMouse_S3_Test_BootMotor.ino
  -------------------------------------------------------------
  Sketch de pruebas para robot raton UMouse con ESP32-S3.

  Objetivo:
  - Probar pines nuevos de motores DRV8871.
  - Probar movimiento de motores: adelante, reversa, freno y coast desde pantalla usando BOOT.
  - Probar encoders por interrupcion.
  - Probar bateria con divisor resistivo.
  - Probar 4 sensores IR caseros por lectura diferencial.
  - Probar OLED I2C y BNO085/BNO08x por I2C.
  - NO requiere monitor Serial para las pruebas principales.
  - NO ejecuta floodfill ni navegacion automatica.

  Librerias necesarias en Arduino IDE:
  - Adafruit SSD1306
  - Adafruit GFX Library
  - Adafruit BNO08x
  - Adafruit BusIO

  Recomendado:
  - Placa: ESP32S3 Dev Module / tu placa ESP32-S3 correspondiente
  - Arduino-ESP32 core 3.x
*/

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_BNO08x.h>
#include <math.h>

#if __has_include(<esp_arduino_version.h>)
  #include <esp_arduino_version.h>
#endif
#ifndef ESP_ARDUINO_VERSION_MAJOR
  #define ESP_ARDUINO_VERSION_MAJOR 2
#endif

#include "UMouse_S3_Test_types.h"

// =============================================================
//  1) MAPA DE PINES NUEVO - ESP32-S3
// =============================================================

// ---- Motores DRV8871 ----
// MR = motor derecho, ML = motor izquierdo
// CA/CB = encoder canal A/B
static const uint8_t PIN_MR_IN2 = 14;
static const uint8_t PIN_MR_IN1 = 13;
static const uint8_t PIN_MR_CA  = 12;
static const uint8_t PIN_MR_CB  = 11;

static const uint8_t PIN_ML_IN2 = 5;
static const uint8_t PIN_ML_IN1 = 6;
static const uint8_t PIN_ML_CA  = 7;
static const uint8_t PIN_ML_CB  = 15;

// ---- Bateria ----
static const uint8_t PIN_VBAT_ADC = 4;

// Divisor resistivo asumido:
// +Bateria -> 3.272k -> ADC -> 987 ohm -> GND
// Si lo tienes invertido, NO lo conectes asi a 12.6 V porque sube demasiado el voltaje del ADC.
static const float VBAT_R_TOP_OHM    = 3272.0f;
static const float VBAT_R_BOTTOM_OHM = 987.0f;
static const float VBAT_DIV_RATIO    = (VBAT_R_TOP_OHM + VBAT_R_BOTTOM_OHM) / VBAT_R_BOTTOM_OHM;

// ---- Sensores IR caseros ----
// IR = LED emisor IR383, FT = fototransistor PT1302
static const uint8_t PIN_FT_FR = 10;
static const uint8_t PIN_IR_FR = 9;

static const uint8_t PIN_FT_R  = 2;
static const uint8_t PIN_IR_R  = 40;

static const uint8_t PIN_FT_FL = 8;
static const uint8_t PIN_IR_FL = 17;

static const uint8_t PIN_FT_L  = 1;
static const uint8_t PIN_IR_L  = 39;

// ---- LEDs debug ----
static const uint8_t PIN_LED_ROJO   = 21;
static const uint8_t PIN_LED_AZUL   = 47;
static const uint8_t PIN_LED_BLANCO = 16;

// ---- BNO085 / BNO08x ----
static const uint8_t PIN_I2C_SDA = 41;
static const uint8_t PIN_I2C_SCL = 42;
static const uint8_t PIN_BNO_INT = 18;
static const uint8_t PIN_BNO_RST = 38;

// ---- Boton BOOT de la placa ----
// No aparece en tu lista porque normalmente ya viene integrado en la placa.
static const uint8_t PIN_BOOT = 0;

// =============================================================
//  2) AJUSTES GENERALES DE PRUEBA
// =============================================================

// OLED asumida en el mismo bus I2C que el BNO085.
static const uint8_t OLED_ADDR = 0x3C;
static const uint8_t OLED_W    = 128;
static const uint8_t OLED_H    = 64;

// Direcciones comunes del BNO085/BNO08x por I2C.
static const uint8_t BNO_ADDR_A = 0x4A;
static const uint8_t BNO_ADDR_B = 0x4B;

// PWM. En ESP32-S3 hay 8 canales LEDC; este codigo usa 8 exactos:
// 4 pines de motor + 4 emisores IR.
static const uint32_t PWM_FREQ_HZ = 20000;
static const uint8_t  PWM_BITS    = 8;
static const uint8_t  PWM_MAX     = 255;

// Canales LEDC usados cuando se compila con Arduino-ESP32 core 2.x.
// En core 3.x ledcAttach trabaja directo con el pin, pero los dejamos definidos para compatibilidad.
static const uint8_t CH_ML_IN1 = 0;
static const uint8_t CH_ML_IN2 = 1;
static const uint8_t CH_MR_IN1 = 2;
static const uint8_t CH_MR_IN2 = 3;
static const uint8_t CH_IR_FL  = 4;
static const uint8_t CH_IR_L   = 5;
static const uint8_t CH_IR_R   = 6;
static const uint8_t CH_IR_FR  = 7;

// Bateria/motores.
// Motores de 6 V alimentados desde bateria 3S: por seguridad el PWM queda limitado
// siempre a un voltaje efectivo aproximado de 6 V y ademas a 50% del PWM total.
// Para exprimirlos en el futuro, cambia MOTOR_ALLOW_OVERDRIVE a true bajo tu propio criterio.
static const float V_MOTOR_LIMIT = 6.0f;       // voltaje efectivo maximo normal para motores
static const bool  MOTOR_ALLOW_OVERDRIVE = false;
static const float MOTOR_HARD_DUTY_FRACTION = 0.50f;  // mitad del PWM total como limite duro
static const int   MOTOR_SAFE_FALLBACK_MAX = 64;      // limite si la lectura de bateria falla
static const int   MOTOR_KICK_MIN = 30;               // minimo para romper estatica, escala 0-255
static const int   MOTOR_TEST_PERCENT_DEFAULT = 50;   // 50% del limite seguro ~= prueba suave, aprox 250 rpm
static const uint32_t MOTOR_TEST_MS = 800;
static const uint32_t MOTOR_BRAKE_MS = 180;
static const uint32_t MOTOR_COAST_MS = 250;

// Encoder para estimar RPM durante las pruebas de motor.
// Conserva estos valores iguales a los usados en el codigo de navegacion.
static const float ENC_PPR_MOTOR  = 7.0f;
static const float ENC_GEAR_RATIO = 30.0f;
static const float ENC_EDGES      = 2.0f;
static const float ENC_TICKS_PER_REV = ENC_PPR_MOTOR * ENC_GEAR_RATIO * ENC_EDGES;

static const float ENC_TRIM_R     = 1.00f;
static const int   ENC_SIGN_L     = +1;
static const int   ENC_SIGN_R     = -1;

// Control diferencial simple, conservando valores base anteriores.
static const float KP_ENC      = 0.30f;
static const float KI_ENC      = 0.13f;
static const float KI_ENC_MAX  = 7.0f;

// Pruebas de movimiento por encoder.
static const int TEST_DRIVE_TICKS = 250;
static const int TEST_TURN_TICKS  = 255;

// IR diferencial.
static const int IR_NSAMPLES = 10;
static const int IR_ON_US    = 250;
static const int IR_OFF_US   = 250;
static const int IR_LED_DUTY = 180;

// Umbrales iniciales. Ajustalos con lo que veas en la pantalla/Serial.
static const int IR_THR_FL = 80;
static const int IR_THR_L  = 70;
static const int IR_THR_R  = 80;
static const int IR_THR_FR = 80;

// true = usa abs(OFF - ON). Recomendado al inicio porque no importa si tu PT1302 sube o baja voltaje con luz.
// false = usa OFF - ON como en tu codigo anterior.
static const bool IR_USE_ABS_DIFF = true;

// BNO085 report rate.
static const uint32_t BNO_REPORT_INTERVAL_US = 10000; // 100 Hz

// Esta version esta pensada para usarse desde la pantalla y el boton BOOT.
// Si algun dia quieres volver a controlar por monitor Serial, cambia esto a true.
static const bool ENABLE_SERIAL_COMMANDS = false;

// =============================================================
//  3) OBJETOS GLOBALES
// =============================================================

Adafruit_SSD1306 display(OLED_W, OLED_H, &Wire, -1);
Adafruit_BNO08x bno08x(PIN_BNO_RST);
sh2_SensorValue_t bnoEvent;

bool g_oledOK = false;
bool g_bnoOK  = false;
uint8_t g_bnoAddr = 0;

// Encoders.
volatile long encL = 0;
volatile long encR = 0;
volatile long encL_abs = 0;
volatile long encR_abs = 0;

// PWM maximo calculado por bateria y estado de pruebas de motor.
int g_dutyMax = MOTOR_SAFE_FALLBACK_MAX;
int g_motorTestPercent = MOTOR_TEST_PERCENT_DEFAULT;
int g_motorCmdL = 0;
int g_motorCmdR = 0;
const char *g_motorState = "COAST";
float g_lastRpmL = 0.0f;
float g_lastRpmR = 0.0f;
long g_lastMotorTicksL = 0;
long g_lastMotorTicksR = 0;

// BNO datos actuales.
float g_yawDeg = 0.0f;
float g_pitchDeg = 0.0f;
float g_rollDeg = 0.0f;
float g_gyroX = 0.0f, g_gyroY = 0.0f, g_gyroZ = 0.0f;
float g_linAx = 0.0f, g_linAy = 0.0f, g_linAz = 0.0f;
uint32_t g_lastBNOms = 0;

// Pagina actual de la OLED. Debe estar declarada antes de las funciones de motor
// porque esas funciones la consultan mientras ejecutan la prueba.
Page g_page = PAGE_STATUS;

// Prototipos importantes para que Arduino IDE no reordene mal funciones usadas antes de declararse.
void drawMotorTestStage(const char *stage, int pwm, uint32_t elapsedMs, uint32_t totalMs);
void drawMotorTestResult();

// =============================================================
//  4) UTILIDADES
// =============================================================

int clampInt(int v, int lo, int hi) {
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

float clampFloat(float v, float lo, float hi) {
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

void ledsOff() {
  digitalWrite(PIN_LED_ROJO, LOW);
  digitalWrite(PIN_LED_AZUL, LOW);
  digitalWrite(PIN_LED_BLANCO, LOW);
}

void blinkLed(uint8_t pin, uint8_t times, uint16_t ms) {
  for (uint8_t i = 0; i < times; i++) {
    digitalWrite(pin, HIGH);
    delay(ms);
    digitalWrite(pin, LOW);
    delay(ms);
  }
}

bool bootDown() {
  return digitalRead(PIN_BOOT) == LOW;
}

bool bootShortPress() {
  static bool last = false;
  static uint32_t tDown = 0;
  bool now = bootDown();
  bool fired = false;

  if (now && !last) tDown = millis();
  if (!now && last) {
    uint32_t dt = millis() - tDown;
    if (dt > 30 && dt < 650) fired = true;
  }

  last = now;
  return fired;
}

bool bootLongPress(uint32_t ms) {
  static bool last = false;
  static uint32_t tDown = 0;
  bool now = bootDown();

  if (now && !last) tDown = millis();
  last = now;

  return now && (millis() - tDown >= ms);
}

// =============================================================
//  5) OLED
// =============================================================

void oledHeader(const char *title) {
  if (!g_oledOK) return;
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println(title);
  display.drawLine(0, 9, 127, 9, SSD1306_WHITE);
}

void oledShow(const char *l1, const char *l2 = "") {
  if (!g_oledOK) return;
  oledHeader(l1);
  display.setCursor(0, 14);
  display.println(l2);
  display.display();
}

// =============================================================
//  6) BATERIA
// =============================================================

float readVBAT_V() {
  uint32_t mv = analogReadMilliVolts(PIN_VBAT_ADC);
  return ((float)mv * VBAT_DIV_RATIO) / 1000.0f;
}

float readVBAT_filtered() {
  uint32_t acc = 0;
  const int n = 20;
  for (int i = 0; i < n; i++) {
    acc += analogReadMilliVolts(PIN_VBAT_ADC);
    delay(2);
  }
  return ((float)(acc / n) * VBAT_DIV_RATIO) / 1000.0f;
}

int calcDutyMax(float vbat) {
  int hardLimit = MOTOR_ALLOW_OVERDRIVE
                    ? PWM_MAX
                    : (int)((float)PWM_MAX * MOTOR_HARD_DUTY_FRACTION + 0.5f);

  // Si el ADC de bateria falla, usar un limite conservador.
  if (vbat < 1.0f) return clampInt(MOTOR_SAFE_FALLBACK_MAX, MOTOR_KICK_MIN, hardLimit);

  // Limite por voltaje: duty ~= Vmotor / Vbat.
  int byVoltage = (int)((float)PWM_MAX * (V_MOTOR_LIMIT / vbat) + 0.5f);
  int limited = min(byVoltage, hardLimit);

  return clampInt(limited, MOTOR_KICK_MIN, PWM_MAX);
}

int calcMotorTestPWM() {
  int pwm = (g_dutyMax * g_motorTestPercent + 50) / 100;
  return clampInt(pwm, MOTOR_KICK_MIN, g_dutyMax);
}

float ticksToRPM(long absTicks, uint32_t durationMs) {
  if (durationMs == 0 || ENC_TICKS_PER_REV <= 0.0f) return 0.0f;
  float rev = (float)absTicks / ENC_TICKS_PER_REV;
  return rev * 60000.0f / (float)durationMs;
}

// =============================================================
//  7) IR DIFERENCIAL
// =============================================================

IRReading irFL = {"FL", PIN_IR_FL, PIN_FT_FL, IR_THR_FL, 0, 0, 0, 0, false};
IRReading irL  = {"L",  PIN_IR_L,  PIN_FT_L,  IR_THR_L,  0, 0, 0, 0, false};
IRReading irR  = {"R",  PIN_IR_R,  PIN_FT_R,  IR_THR_R,  0, 0, 0, 0, false};
IRReading irFR = {"FR", PIN_IR_FR, PIN_FT_FR, IR_THR_FR, 0, 0, 0, 0, false};

void irLedOffAll() {
  pwmWritePin(PIN_IR_FL, 0);
  pwmWritePin(PIN_IR_L,  0);
  pwmWritePin(PIN_IR_R,  0);
  pwmWritePin(PIN_IR_FR, 0);
}

void readOneIR(IRReading &s) {
  uint32_t offAcc = 0;
  uint32_t onAcc  = 0;

  for (int i = 0; i < IR_NSAMPLES; i++) {
    pwmWritePin(s.ledPin, 0);
    delayMicroseconds(IR_OFF_US);
    offAcc += analogReadMilliVolts(s.ftPin);

    pwmWritePin(s.ledPin, IR_LED_DUTY);
    delayMicroseconds(IR_ON_US);
    onAcc += analogReadMilliVolts(s.ftPin);

    pwmWritePin(s.ledPin, 0);
    delayMicroseconds(IR_OFF_US);
  }

  s.offmV = (int)(offAcc / IR_NSAMPLES);
  s.onmV  = (int)(onAcc  / IR_NSAMPLES);
  s.diffSigned = s.offmV - s.onmV;
  s.diffUsed = IR_USE_ABS_DIFF ? abs(s.diffSigned) : s.diffSigned;
  if (s.diffUsed < 0) s.diffUsed = 0;
  s.wall = s.diffUsed >= s.threshold;
}

void readAllIR() {
  readOneIR(irFL);
  readOneIR(irL);
  readOneIR(irR);
  readOneIR(irFR);
}

bool hasWallFront() {
  readAllIR();
  return irFL.wall || irFR.wall;
}

bool hasWallLeft() {
  readAllIR();
  return irL.wall;
}

bool hasWallRight() {
  readAllIR();
  return irR.wall;
}

// =============================================================
//  8) ENCODERS
// =============================================================

void IRAM_ATTR isrEncL_A() {
  bool a = digitalRead(PIN_ML_CA);
  bool b = digitalRead(PIN_ML_CB);
  encL += (long)((a == b ? +1 : -1) * ENC_SIGN_L);
  encL_abs++;
}

void IRAM_ATTR isrEncR_A() {
  bool a = digitalRead(PIN_MR_CA);
  bool b = digitalRead(PIN_MR_CB);
  encR += (long)((a == b ? +1 : -1) * ENC_SIGN_R);
  encR_abs++;
}

void resetEncoders() {
  noInterrupts();
  encL = 0;
  encR = 0;
  encL_abs = 0;
  encR_abs = 0;
  interrupts();
}

void getEncAll(long &l, long &lA, long &r, long &rA) {
  noInterrupts();
  l = encL;
  lA = encL_abs;
  r = encR;
  rA = encR_abs;
  interrupts();
}

// =============================================================
//  9) MOTORES DRV8871
// =============================================================

void motorCoastAll() {
  pwmWritePin(PIN_ML_IN1, 0);
  pwmWritePin(PIN_ML_IN2, 0);
  pwmWritePin(PIN_MR_IN1, 0);
  pwmWritePin(PIN_MR_IN2, 0);
  g_motorCmdL = 0;
  g_motorCmdR = 0;
  g_motorState = "COAST";
}

void motorBrakeAll() {
  // IN1=HIGH e IN2=HIGH en DRV8871 = brake. No hay voltaje diferencial de avance en el motor.
  pwmWritePin(PIN_ML_IN1, PWM_MAX);
  pwmWritePin(PIN_ML_IN2, PWM_MAX);
  pwmWritePin(PIN_MR_IN1, PWM_MAX);
  pwmWritePin(PIN_MR_IN2, PWM_MAX);
  g_motorCmdL = 0;
  g_motorCmdR = 0;
  g_motorState = "BRAKE";
}

// Modo ya usado en tu codigo anterior:
// un pin queda alto y el otro conmuta entre brake y avance/reversa.
// El duty efectivo de movimiento es pwm/255, limitado por g_dutyMax.
void modeA(uint8_t in1, uint8_t in2, uint8_t pwm) {
  pwm = constrain(pwm, 0, PWM_MAX);
  pwmWritePin(in1, PWM_MAX);
  pwmWritePin(in2, PWM_MAX - pwm);
}

void modeB(uint8_t in1, uint8_t in2, uint8_t pwm) {
  pwm = constrain(pwm, 0, PWM_MAX);
  pwmWritePin(in2, PWM_MAX);
  pwmWritePin(in1, PWM_MAX - pwm);
}

void motorSetRaw(uint8_t in1, uint8_t in2, int speed, bool forwardIsModeA) {
  speed = clampInt(speed, -g_dutyMax, +g_dutyMax);

  if (speed == 0) {
    pwmWritePin(in1, 0);
    pwmWritePin(in2, 0);
    return;
  }

  int s = abs(speed);
  int kick = clampInt(MOTOR_KICK_MIN, 0, g_dutyMax);
  if (s < kick) s = kick;
  uint8_t pwm = (uint8_t)clampInt(s, 0, g_dutyMax);

  if (speed > 0) {
    if (forwardIsModeA) modeA(in1, in2, pwm);
    else                modeB(in1, in2, pwm);
  } else {
    if (forwardIsModeA) modeB(in1, in2, pwm);
    else                modeA(in1, in2, pwm);
  }
}

// Segun tu codigo anterior:
// Motor izquierdo adelante = modo B
// Motor derecho adelante = modo A
void motorL_set(int speed) {
  motorSetRaw(PIN_ML_IN1, PIN_ML_IN2, speed, false);
}

void motorR_set(int speed) {
  motorSetRaw(PIN_MR_IN1, PIN_MR_IN2, speed, true);
}

void motorSetBoth(int left, int right) {
  g_motorCmdL = clampInt(left,  -g_dutyMax, +g_dutyMax);
  g_motorCmdR = clampInt(right, -g_dutyMax, +g_dutyMax);
  motorL_set(left);
  motorR_set(right);
}

void motorHoldCoast(uint32_t ms) {
  motorCoastAll();
  uint32_t t0 = millis();
  while (millis() - t0 < ms) {
    updateBNO();
    if (g_page == PAGE_MOTOR) drawMotorTestStage("COAST", 0, millis() - t0, ms);
    delay(5);
  }
}

void motorHoldBrake(uint32_t ms) {
  motorBrakeAll();
  uint32_t t0 = millis();
  while (millis() - t0 < ms) {
    updateBNO();
    if (g_page == PAGE_MOTOR) drawMotorTestStage("FRENO", 0, millis() - t0, ms);
    delay(5);
  }
}

void motorTimedTest(const char *label, int leftPWM, int rightPWM, uint32_t ms, bool brakeAtEnd = true) {
  resetEncoders();
  g_motorState = label;
  uint32_t t0 = millis();

  Serial.printf("\n[MOTOR] %s | Lcmd=%d Rcmd=%d | dutyMax=%d test=%d%% | %lu ms\n",
                label, leftPWM, rightPWM, g_dutyMax, g_motorTestPercent, (unsigned long)ms);

  while (millis() - t0 < ms) {
    motorSetBoth(leftPWM, rightPWM);
    updateBNO();
    if (g_page == PAGE_MOTOR) drawMotorTestStage(label, max(abs(leftPWM), abs(rightPWM)), millis() - t0, ms);
    delay(5);
  }

  long l, lA, r, rA;
  getEncAll(l, lA, r, rA);
  g_lastMotorTicksL = lA;
  g_lastMotorTicksR = rA;
  g_lastRpmL = ticksToRPM(lA, ms);
  g_lastRpmR = ticksToRPM(rA, ms);

  Serial.printf("[MOTOR] fin %s | L=%ld L_abs=%ld %.1f rpm | R=%ld R_abs=%ld %.1f rpm\n",
                label, l, lA, g_lastRpmL, r, rA, g_lastRpmR);

  if (brakeAtEnd) {
    motorHoldBrake(MOTOR_BRAKE_MS);
    motorHoldCoast(MOTOR_COAST_MS);
  } else {
    motorCoastAll();
  }
}

void motorFullSequenceTest() {
  int pwm = calcMotorTestPWM();

  Serial.println("\n========== SECUENCIA COMPLETA DE MOTORES ==========");
  Serial.printf("VBAT=%.2f V | dutyMax=%d | testPWM=%d | test=%d%% del limite seguro\n",
                readVBAT_V(), g_dutyMax, pwm, g_motorTestPercent);

  drawMotorTestStage("INICIANDO", pwm, 0, 0);
  delay(350);

  motorHoldCoast(MOTOR_COAST_MS);
  Serial.println("Paso 1: COAST");

  motorTimedTest("ADELANTE", +pwm, +pwm, MOTOR_TEST_MS, true);
  Serial.println("Paso 2: ADELANTE + FRENO + COAST");

  motorTimedTest("REVERSA", -pwm, -pwm, MOTOR_TEST_MS, true);
  Serial.println("Paso 3: REVERSA + FRENO + COAST");

  motorHoldBrake(MOTOR_BRAKE_MS);
  Serial.println("Paso 4: FRENO");

  motorHoldCoast(MOTOR_COAST_MS);
  Serial.println("Paso 5: COAST final");
  Serial.println("==================================================\n");

  motorCoastAll();
  drawMotorTestResult();
  delay(1200);
}

void driveEncoderTicks(int targetTicks, int basePWM) {
  resetEncoders();
  uint32_t tStart = millis();
  uint32_t tCtrl = 0;
  float encI = 0.0f;

  while (true) {
    long l, lA, r, rA;
    getEncAll(l, lA, r, rA);
    long avg = (lA + rA) / 2;

    if (avg >= targetTicks) break;
    if (millis() - tStart > 4000) break;

    if (millis() - tCtrl >= 10) {
      tCtrl = millis();
      float err = (float)lA - ((float)rA * ENC_TRIM_R);
      if (fabsf(err) < 3.0f) err = 0.0f;

      encI += KI_ENC * err;
      encI = clampFloat(encI, -KI_ENC_MAX, +KI_ENC_MAX);

      float u = KP_ENC * err + encI;
      u = clampFloat(u, -(float)(basePWM / 3), +(float)(basePWM / 3));

      motorSetBoth(basePWM - (int)u, basePWM + (int)u);
    }

    delay(1);
  }

  motorBrakeAll();
  delay(80);
  motorCoastAll();

  long l, lA, r, rA;
  getEncAll(l, lA, r, rA);
  Serial.printf("DRIVE ticks target=%d | L_abs=%ld R_abs=%ld\n", targetTicks, lA, rA);
}

void turnEncoderTicks(int targetTicks, int leftPWM, int rightPWM) {
  resetEncoders();
  uint32_t tStart = millis();

  while (true) {
    long l, lA, r, rA;
    getEncAll(l, lA, r, rA);
    long avg = (lA + rA) / 2;

    if (avg >= targetTicks) break;
    if (millis() - tStart > 3000) break;

    motorSetBoth(leftPWM, rightPWM);
    delay(5);
  }

  motorBrakeAll();
  delay(80);
  motorCoastAll();

  long l, lA, r, rA;
  getEncAll(l, lA, r, rA);
  Serial.printf("TURN ticks target=%d | L_abs=%ld R_abs=%ld\n", targetTicks, lA, rA);
}

// =============================================================
//  10) BNO085 / BNO08x
// =============================================================

void quaternionToEulerDeg(float qr, float qi, float qj, float qk,
                          float &yawDeg, float &pitchDeg, float &rollDeg) {
  // i=x, j=y, k=z, real=w
  float sqi = qi * qi;
  float sqj = qj * qj;
  float sqk = qk * qk;

  float roll = atan2f(2.0f * (qr * qi + qj * qk), 1.0f - 2.0f * (sqi + sqj));

  float t2 = 2.0f * (qr * qj - qk * qi);
  t2 = clampFloat(t2, -1.0f, 1.0f);
  float pitch = asinf(t2);

  float yaw = atan2f(2.0f * (qr * qk + qi * qj), 1.0f - 2.0f * (sqj + sqk));

  yawDeg   = yaw   * 180.0f / PI;
  pitchDeg = pitch * 180.0f / PI;
  rollDeg  = roll  * 180.0f / PI;
}

bool setBNOReports() {
  if (!g_bnoOK) return false;

  bool ok = true;

  // GAME_ROTATION_VECTOR: orientacion sin magnetometro. Suele ser mejor cerca de motores.
  ok &= bno08x.enableReport(SH2_GAME_ROTATION_VECTOR, BNO_REPORT_INTERVAL_US);
  ok &= bno08x.enableReport(SH2_GYROSCOPE_CALIBRATED, BNO_REPORT_INTERVAL_US);
  ok &= bno08x.enableReport(SH2_LINEAR_ACCELERATION, BNO_REPORT_INTERVAL_US);

  return ok;
}

void updateBNO() {
  if (!g_bnoOK) return;

  if (bno08x.wasReset()) {
    Serial.println("BNO085 reset detectado, reactivando reportes...");
    setBNOReports();
  }

  // Leer varios eventos pendientes sin quedarse atrapado.
  for (uint8_t i = 0; i < 8; i++) {
    if (!bno08x.getSensorEvent(&bnoEvent)) return;

    g_lastBNOms = millis();

    switch (bnoEvent.sensorId) {
      case SH2_GAME_ROTATION_VECTOR:
        quaternionToEulerDeg(
          bnoEvent.un.gameRotationVector.real,
          bnoEvent.un.gameRotationVector.i,
          bnoEvent.un.gameRotationVector.j,
          bnoEvent.un.gameRotationVector.k,
          g_yawDeg,
          g_pitchDeg,
          g_rollDeg
        );
        break;

      case SH2_GYROSCOPE_CALIBRATED:
        g_gyroX = bnoEvent.un.gyroscope.x;
        g_gyroY = bnoEvent.un.gyroscope.y;
        g_gyroZ = bnoEvent.un.gyroscope.z;
        break;

      case SH2_LINEAR_ACCELERATION:
        g_linAx = bnoEvent.un.linearAcceleration.x;
        g_linAy = bnoEvent.un.linearAcceleration.y;
        g_linAz = bnoEvent.un.linearAcceleration.z;
        break;

      default:
        break;
    }
  }
}

// =============================================================
//  11) I2C SCAN
// =============================================================

uint8_t scanI2C(bool printResults) {
  uint8_t count = 0;

  if (printResults) Serial.println("\nI2C scan:");

  for (uint8_t addr = 1; addr < 127; addr++) {
    Wire.beginTransmission(addr);
    uint8_t err = Wire.endTransmission();

    if (err == 0) {
      count++;
      if (printResults) Serial.printf("  encontrado: 0x%02X\n", addr);
    }
  }

  if (printResults) Serial.printf("Total I2C: %u dispositivo(s)\n\n", count);
  return count;
}

// =============================================================
//  12) PAGINAS DE PANTALLA
// =============================================================

void drawStatusPage() {
  float vbat = readVBAT_V();
  g_dutyMax = calcDutyMax(vbat);
  if (g_dutyMax < MOTOR_KICK_MIN) g_dutyMax = MOTOR_KICK_MIN;

  oledHeader("STATUS");
  if (!g_oledOK) return;

  display.setCursor(0, 12);
  display.print("VBAT: "); display.print(vbat, 2); display.print(" V");

  display.setCursor(0, 22);
  display.print("DutyMax: "); display.print(g_dutyMax);

  display.setCursor(0, 32);
  display.print("OLED: "); display.print(g_oledOK ? "OK" : "NO");
  display.print(" BNO: "); display.print(g_bnoOK ? "OK" : "NO");

  display.setCursor(0, 42);
  display.print("BNO addr: ");
  if (g_bnoOK) display.printf("0x%02X", g_bnoAddr);
  else display.print("--");

  display.setCursor(0, 54);
  display.print("BOOT: pagina");
  display.display();
}

void drawIRDiffPage() {
  readAllIR();

  oledHeader("IR DIFF mV");
  if (!g_oledOK) return;

  display.setCursor(0, 12);
  display.print("FL:"); display.print(irFL.diffUsed);
  display.print("  FR:"); display.print(irFR.diffUsed);

  display.setCursor(0, 24);
  display.print("L :"); display.print(irL.diffUsed);
  display.print("  R :"); display.print(irR.diffUsed);

  display.setCursor(0, 38);
  display.print("Sign FL:"); display.print(irFL.diffSigned);
  display.print(" FR:"); display.print(irFR.diffSigned);

  display.setCursor(0, 52);
  display.print(IR_USE_ABS_DIFF ? "modo abs(OFF-ON)" : "modo OFF-ON");
  display.display();
}

void drawIRRawPage() {
  readAllIR();

  oledHeader("IR RAW OFF/ON");
  if (!g_oledOK) return;

  display.setCursor(0, 12);
  display.print("FL "); display.print(irFL.offmV); display.print("/"); display.print(irFL.onmV);
  display.setCursor(0, 22);
  display.print("FR "); display.print(irFR.offmV); display.print("/"); display.print(irFR.onmV);
  display.setCursor(0, 34);
  display.print("L  "); display.print(irL.offmV); display.print("/"); display.print(irL.onmV);
  display.setCursor(0, 44);
  display.print("R  "); display.print(irR.offmV); display.print("/"); display.print(irR.onmV);
  display.display();
}

void drawWallsPage() {
  readAllIR();

  oledHeader("PAREDES IR");
  if (!g_oledOK) return;

  display.setCursor(0, 12);
  display.print("FL:"); display.print(irFL.wall ? "Y" : "N");
  display.print(" FR:"); display.print(irFR.wall ? "Y" : "N");
  display.print(" F:"); display.print((irFL.wall || irFR.wall) ? "Y" : "N");

  display.setCursor(0, 24);
  display.print("L:"); display.print(irL.wall ? "Y" : "N");
  display.print(" R:"); display.print(irR.wall ? "Y" : "N");

  display.setCursor(0, 38);
  display.print("Thr FL/FR:"); display.print(IR_THR_FL); display.print("/"); display.print(IR_THR_FR);

  display.setCursor(0, 52);
  display.print("Thr L/R:"); display.print(IR_THR_L); display.print("/"); display.print(IR_THR_R);
  display.display();
}

void drawEncoderPage() {
  long l, lA, r, rA;
  getEncAll(l, lA, r, rA);

  oledHeader("ENCODERS");
  if (!g_oledOK) return;

  display.setCursor(0, 12);
  display.print("L: "); display.print(l);
  display.print(" abs:"); display.print(lA);

  display.setCursor(0, 26);
  display.print("R: "); display.print(r);
  display.print(" abs:"); display.print(rA);

  display.setCursor(0, 42);
  display.print("Mant BOOT: reset");

  display.setCursor(0, 54);
  display.print("Gira ruedas manual");
  display.display();
}

void drawBNOPage() {
  updateBNO();

  oledHeader("BNO085");
  if (!g_oledOK) return;

  display.setCursor(0, 12);
  display.print("Yaw:"); display.print(g_yawDeg, 1);
  display.print(" P:"); display.print(g_pitchDeg, 1);

  display.setCursor(0, 24);
  display.print("Roll:"); display.print(g_rollDeg, 1);

  display.setCursor(0, 36);
  display.print("Gz:"); display.print(g_gyroZ, 2);
  display.print(" rad/s");

  display.setCursor(0, 48);
  display.print("Ax:"); display.print(g_linAx, 1);
  display.print(" Ay:"); display.print(g_linAy, 1);

  display.display();
}

void drawMotorPage() {
  float vbat = readVBAT_V();
  int testPWM = calcMotorTestPWM();

  oledHeader("MOTORES");
  if (!g_oledOK) return;

  display.setCursor(0, 12);
  display.print("VB:"); display.print(vbat, 1);
  display.print(" Max:"); display.print(g_dutyMax);

  display.setCursor(0, 22);
  display.print("Test:"); display.print(testPWM);
  display.print(" "); display.print(g_motorTestPercent); display.print("%");

  display.setCursor(0, 32);
  display.print("Modo:"); display.print(g_motorState);

  display.setCursor(0, 42);
  display.print("Cmd L:"); display.print(g_motorCmdL);
  display.print(" R:"); display.print(g_motorCmdR);

  display.setCursor(0, 52);
  display.print("Mant BOOT: secuencia");

  display.display();
}

void drawMotorTestStage(const char *stage, int pwm, uint32_t elapsedMs, uint32_t totalMs) {
  if (!g_oledOK) return;

  long l, lA, r, rA;
  getEncAll(l, lA, r, rA);

  oledHeader("TEST MOTORES");
  display.setCursor(0, 12);
  display.print(stage);

  display.setCursor(0, 24);
  display.print("PWM:"); display.print(pwm);
  display.print(" Max:"); display.print(g_dutyMax);

  display.setCursor(0, 36);
  display.print("L:"); display.print(lA);
  display.print(" R:"); display.print(rA);

  display.setCursor(0, 48);
  if (totalMs > 0) {
    uint8_t pct = (uint8_t)clampInt((int)((elapsedMs * 100UL) / totalMs), 0, 100);
    display.print("Progreso: "); display.print(pct); display.print("%");
  } else {
    display.print("Mantente listo");
  }

  display.display();
}

void drawMotorTestResult() {
  if (!g_oledOK) return;

  oledHeader("RESULTADO MOTOR");
  display.setCursor(0, 12);
  display.print("Ticks L:"); display.print(g_lastMotorTicksL);

  display.setCursor(0, 24);
  display.print("Ticks R:"); display.print(g_lastMotorTicksR);

  display.setCursor(0, 36);
  display.print("RPM L:"); display.print(g_lastRpmL, 0);
  display.print(" R:"); display.print(g_lastRpmR, 0);

  display.setCursor(0, 52);
  display.print("BOOT corto: salir");
  display.display();
}

void drawCurrentPage() {
  switch (g_page) {
    case PAGE_STATUS:   drawStatusPage();  break;
    case PAGE_IR_DIFF:  drawIRDiffPage();  break;
    case PAGE_IR_RAW:   drawIRRawPage();   break;
    case PAGE_WALLS:    drawWallsPage();   break;
    case PAGE_ENCODERS: drawEncoderPage(); break;
    case PAGE_BNO:      drawBNOPage();     break;
    case PAGE_MOTOR:    drawMotorPage();   break;
    default:            drawStatusPage();  break;
  }
}

// =============================================================
//  13) SERIAL DEBUG
// =============================================================

void printHelp() {
  Serial.println();
  Serial.println("================ UMouse ESP32-S3 Test ================");
  Serial.println("h  -> ayuda");
  Serial.println("p  -> siguiente pagina OLED");
  Serial.println("i  -> escanear I2C");
  Serial.println("v  -> imprimir telemetria completa");
  Serial.println("z  -> reset encoders");
  Serial.println("s/c-> coast/stop motores");
  Serial.println("k  -> brake/freno sostenido");
  Serial.println("x  -> brake breve y luego coast");
  Serial.println("m  -> secuencia completa: coast, adelante, freno, atras, freno, coast");
  Serial.println("l  -> motor izquierdo adelante prueba corta");
  Serial.println("L  -> motor izquierdo reversa prueba corta");
  Serial.println("r  -> motor derecho adelante prueba corta");
  Serial.println("R  -> motor derecho reversa prueba corta");
  Serial.println("f  -> ambos adelante prueba corta");
  Serial.println("b  -> ambos reversa prueba corta");
  Serial.println("+/- -> subir/bajar porcentaje de prueba de motor");
  Serial.println("g  -> avanzar por encoders TEST_DRIVE_TICKS");
  Serial.println("a  -> giro izquierda por encoders TEST_TURN_TICKS");
  Serial.println("d  -> giro derecha por encoders TEST_TURN_TICKS");
  Serial.println("1  -> encender/apagar LED rojo debug");
  Serial.println("2  -> encender/apagar LED azul debug");
  Serial.println("3  -> encender/apagar LED blanco debug");
  Serial.println("=======================================================");
  Serial.println();
}

void printTelemetry() {
  updateBNO();
  readAllIR();

  long l, lA, r, rA;
  getEncAll(l, lA, r, rA);
  float vbat = readVBAT_V();

  Serial.println("\n--- TELEMETRIA ---");
  Serial.printf("VBAT: %.2f V | dutyMax: %d\n", vbat, g_dutyMax);
  Serial.printf("IR diff used mV | FL:%d L:%d R:%d FR:%d\n", irFL.diffUsed, irL.diffUsed, irR.diffUsed, irFR.diffUsed);
  Serial.printf("IR signed mV    | FL:%d L:%d R:%d FR:%d\n", irFL.diffSigned, irL.diffSigned, irR.diffSigned, irFR.diffSigned);
  Serial.printf("IR raw off/on   | FL:%d/%d L:%d/%d R:%d/%d FR:%d/%d\n",
                irFL.offmV, irFL.onmV, irL.offmV, irL.onmV, irR.offmV, irR.onmV, irFR.offmV, irFR.onmV);
  Serial.printf("WALLS           | FL:%d L:%d R:%d FR:%d FRONT:%d\n",
                irFL.wall, irL.wall, irR.wall, irFR.wall, (irFL.wall || irFR.wall));
  Serial.printf("ENC             | L:%ld L_abs:%ld | R:%ld R_abs:%ld\n", l, lA, r, rA);
  Serial.printf("MOTOR           | state:%s dutyMax:%d testPWM:%d percent:%d cmdL:%d cmdR:%d lastRPM L:%.1f R:%.1f\n",
                g_motorState, g_dutyMax, calcMotorTestPWM(), g_motorTestPercent,
                g_motorCmdL, g_motorCmdR, g_lastRpmL, g_lastRpmR);
  Serial.printf("BNO             | ok:%d addr:0x%02X yaw:%.1f pitch:%.1f roll:%.1f gyroZ:%.3f linA:%.2f %.2f %.2f age:%lu ms\n",
                g_bnoOK, g_bnoAddr, g_yawDeg, g_pitchDeg, g_rollDeg, g_gyroZ,
                g_linAx, g_linAy, g_linAz, millis() - g_lastBNOms);
}

void handleSerial() {
  if (!Serial.available()) return;

  char c = Serial.read();

  switch (c) {
    case '\n':
    case '\r':
      break;

    case 'h':
      if (ENABLE_SERIAL_COMMANDS) printHelp();
      break;

    case 'p':
      g_page = (Page)((g_page + 1) % PAGE_COUNT);
      drawCurrentPage();
      break;

    case 'i':
      scanI2C(true);
      break;

    case 'v':
      printTelemetry();
      break;

    case 'z':
      resetEncoders();
      Serial.println("Encoders reseteados");
      oledShow("Encoders", "reseteados");
      break;

    case 's':
    case 'c':
      motorCoastAll();
      Serial.println("Motores en COAST/STOP");
      break;

    case 'k':
      motorBrakeAll();
      Serial.println("Motores en BRAKE sostenido. Usa s/c para volver a coast.");
      break;

    case 'x':
      motorHoldBrake(MOTOR_BRAKE_MS);
      motorCoastAll();
      Serial.println("Brake breve aplicado y luego COAST");
      break;

    case 'm':
      motorFullSequenceTest();
      break;

    case '+':
      g_motorTestPercent = clampInt(g_motorTestPercent + 5, 10, 100);
      Serial.printf("Motor test percent = %d%% | testPWM=%d | dutyMax=%d\n",
                    g_motorTestPercent, calcMotorTestPWM(), g_dutyMax);
      break;

    case '-':
      g_motorTestPercent = clampInt(g_motorTestPercent - 5, 10, 100);
      Serial.printf("Motor test percent = %d%% | testPWM=%d | dutyMax=%d\n",
                    g_motorTestPercent, calcMotorTestPWM(), g_dutyMax);
      break;

    case 'l': {
      int pwm = calcMotorTestPWM();
      Serial.println("ML adelante");
      motorTimedTest("ML FWD", +pwm, 0, MOTOR_TEST_MS);
      break;
    }

    case 'L': {
      int pwm = calcMotorTestPWM();
      Serial.println("ML reversa");
      motorTimedTest("ML REV", -pwm, 0, MOTOR_TEST_MS);
      break;
    }

    case 'r': {
      int pwm = calcMotorTestPWM();
      Serial.println("MR adelante");
      motorTimedTest("MR FWD", 0, +pwm, MOTOR_TEST_MS);
      break;
    }

    case 'R': {
      int pwm = calcMotorTestPWM();
      Serial.println("MR reversa");
      motorTimedTest("MR REV", 0, -pwm, MOTOR_TEST_MS);
      break;
    }

    case 'f': {
      int pwm = calcMotorTestPWM();
      Serial.println("Ambos adelante");
      motorTimedTest("FWD", +pwm, +pwm, MOTOR_TEST_MS);
      break;
    }

    case 'b': {
      int pwm = calcMotorTestPWM();
      Serial.println("Ambos reversa");
      motorTimedTest("REV", -pwm, -pwm, MOTOR_TEST_MS);
      break;
    }

    case 'g': {
      int pwm = calcMotorTestPWM();
      Serial.println("Avance por encoder");
      driveEncoderTicks(TEST_DRIVE_TICKS, pwm);
      break;
    }

    case 'a': {
      int pwm = calcMotorTestPWM();
      Serial.println("Giro izquierda por encoder");
      turnEncoderTicks(TEST_TURN_TICKS, -pwm, +pwm);
      break;
    }

    case 'd': {
      int pwm = calcMotorTestPWM();
      Serial.println("Giro derecha por encoder");
      turnEncoderTicks(TEST_TURN_TICKS, +pwm, -pwm);
      break;
    }

    case '1':
      digitalWrite(PIN_LED_ROJO, !digitalRead(PIN_LED_ROJO));
      break;

    case '2':
      digitalWrite(PIN_LED_AZUL, !digitalRead(PIN_LED_AZUL));
      break;

    case '3':
      digitalWrite(PIN_LED_BLANCO, !digitalRead(PIN_LED_BLANCO));
      break;

    default:
      Serial.print("Comando no reconocido: ");
      Serial.println(c);
      Serial.println("Escribe h para ayuda.");
      break;
  }
}

// =============================================================
//  14) INIT HARDWARE
// =============================================================

void initDebugLeds() {
  pinMode(PIN_LED_ROJO, OUTPUT);
  pinMode(PIN_LED_AZUL, OUTPUT);
  pinMode(PIN_LED_BLANCO, OUTPUT);
  ledsOff();
}

void initADC() {
  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);
  analogSetPinAttenuation(PIN_VBAT_ADC, ADC_11db);
  analogSetPinAttenuation(PIN_FT_FL, ADC_11db);
  analogSetPinAttenuation(PIN_FT_L,  ADC_11db);
  analogSetPinAttenuation(PIN_FT_R,  ADC_11db);
  analogSetPinAttenuation(PIN_FT_FR, ADC_11db);
}

uint8_t pwmChannelForPin(uint8_t pin) {
  if (pin == PIN_ML_IN1) return CH_ML_IN1;
  if (pin == PIN_ML_IN2) return CH_ML_IN2;
  if (pin == PIN_MR_IN1) return CH_MR_IN1;
  if (pin == PIN_MR_IN2) return CH_MR_IN2;
  if (pin == PIN_IR_FL)  return CH_IR_FL;
  if (pin == PIN_IR_L)   return CH_IR_L;
  if (pin == PIN_IR_R)   return CH_IR_R;
  if (pin == PIN_IR_FR)  return CH_IR_FR;
  return 0;
}

void pwmWritePin(uint8_t pin, uint32_t duty) {
  duty = constrain(duty, 0, PWM_MAX);

#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcWrite(pin, duty);
#else
  ledcWrite(pwmChannelForPin(pin), duty);
#endif
}

bool attachPWM(uint8_t pin, const char *name, uint8_t channel) {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  bool ok = ledcAttach(pin, PWM_FREQ_HZ, PWM_BITS);
#else
  ledcSetup(channel, PWM_FREQ_HZ, PWM_BITS);
  ledcAttachPin(pin, channel);
  bool ok = true;
#endif

  Serial.printf("PWM %-8s GPIO%u CH%u -> %s\n", name, pin, channel, ok ? "OK" : "ERROR");
  if (ok) pwmWritePin(pin, 0);
  return ok;
}

void initPWM() {
  // 4 motores
  attachPWM(PIN_ML_IN1, "ML_IN1", CH_ML_IN1);
  attachPWM(PIN_ML_IN2, "ML_IN2", CH_ML_IN2);
  attachPWM(PIN_MR_IN1, "MR_IN1", CH_MR_IN1);
  attachPWM(PIN_MR_IN2, "MR_IN2", CH_MR_IN2);

  // 4 emisores IR
  attachPWM(PIN_IR_FL, "IR_FL", CH_IR_FL);
  attachPWM(PIN_IR_L,  "IR_L",  CH_IR_L);
  attachPWM(PIN_IR_R,  "IR_R",  CH_IR_R);
  attachPWM(PIN_IR_FR, "IR_FR", CH_IR_FR);

  motorCoastAll();
  irLedOffAll();
}

void initEncoders() {
  pinMode(PIN_ML_CA, INPUT);
  pinMode(PIN_ML_CB, INPUT);
  pinMode(PIN_MR_CA, INPUT);
  pinMode(PIN_MR_CB, INPUT);

  attachInterrupt(digitalPinToInterrupt(PIN_ML_CA), isrEncL_A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_MR_CA), isrEncR_A, CHANGE);

  resetEncoders();
}

void initI2CAndOLED() {
  Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);
  Wire.setClock(400000);

  g_oledOK = display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR);
  if (g_oledOK) {
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    display.println("UMouse S3 Test");
    display.println("OLED OK");
    display.display();
  }

  Serial.printf("OLED 0x%02X -> %s\n", OLED_ADDR, g_oledOK ? "OK" : "NO ENCONTRADA");
}

void initBNO() {
  pinMode(PIN_BNO_INT, INPUT_PULLUP);

  g_bnoOK = false;
  g_bnoAddr = 0;

  Serial.println("Iniciando BNO085/BNO08x...");

  if (bno08x.begin_I2C(BNO_ADDR_A, &Wire)) {
    g_bnoOK = true;
    g_bnoAddr = BNO_ADDR_A;
  } else if (bno08x.begin_I2C(BNO_ADDR_B, &Wire)) {
    g_bnoOK = true;
    g_bnoAddr = BNO_ADDR_B;
  }

  if (!g_bnoOK) {
    Serial.println("BNO085 NO encontrado. Revisa SDA/SCL, 3.3V, GND, RST y direccion 0x4A/0x4B.");
    return;
  }

  bool reportsOK = setBNOReports();
  Serial.printf("BNO085 encontrado en 0x%02X | reportes: %s\n", g_bnoAddr, reportsOK ? "OK" : "ERROR");

  for (int n = 0; n < bno08x.prodIds.numEntries; n++) {
    Serial.printf("BNO Part %ld Ver %u.%u.%u Build %lu\n",
                  bno08x.prodIds.entry[n].swPartNumber,
                  bno08x.prodIds.entry[n].swVersionMajor,
                  bno08x.prodIds.entry[n].swVersionMinor,
                  bno08x.prodIds.entry[n].swVersionPatch,
                  bno08x.prodIds.entry[n].swBuildNumber);
  }
}

// =============================================================
//  15) SETUP / LOOP
// =============================================================

void setup() {
  Serial.begin(115200);
  delay(800);

  Serial.println();
  Serial.println("Arrancando UMouse ESP32-S3 Test...");

  pinMode(PIN_BOOT, INPUT_PULLUP);
  initDebugLeds();
  digitalWrite(PIN_LED_BLANCO, HIGH);

  initI2CAndOLED();
  oledShow("UMouse S3", "iniciando...");

  initADC();
  initPWM();
  initEncoders();

  scanI2C(true);
  initBNO();

  float vbat = readVBAT_filtered();
  g_dutyMax = calcDutyMax(vbat);
  if (g_dutyMax < MOTOR_KICK_MIN) g_dutyMax = MOTOR_KICK_MIN;

  Serial.printf("VBAT inicial: %.2f V | DutyMax: %d | DivRatio: %.4f\n", vbat, g_dutyMax, VBAT_DIV_RATIO);
  if (ENABLE_SERIAL_COMMANDS) printHelp();

  digitalWrite(PIN_LED_BLANCO, LOW);
  blinkLed(PIN_LED_AZUL, 2, 80);

  oledShow("Listo", "BOOT cambia pag");
  delay(500);
}

void loop() {
  if (ENABLE_SERIAL_COMMANDS) handleSerial();
  updateBNO();

  if (bootShortPress()) {
    g_page = (Page)((g_page + 1) % PAGE_COUNT);
    drawCurrentPage();
  }

  if (bootLongPress(1200)) {
    while (bootDown()) {
      updateBNO();
      if (g_page == PAGE_MOTOR) drawMotorTestStage("Suelta BOOT", calcMotorTestPWM(), 0, 0);
      delay(10);
    }

    if (g_page == PAGE_MOTOR) {
      motorFullSequenceTest();
    } else if (g_page == PAGE_ENCODERS) {
      resetEncoders();
      oledShow("Encoders", "reseteados");
      Serial.println("BOOT largo: encoders reseteados");
      delay(300);
    }
  }

  static uint32_t tUI = 0;
  if (millis() - tUI >= 150) {
    tUI = millis();
    drawCurrentPage();
  }

  static uint32_t tBattery = 0;
  if (millis() - tBattery >= 500) {
    tBattery = millis();
    float vbat = readVBAT_V();
    g_dutyMax = calcDutyMax(vbat);
    if (g_dutyMax < MOTOR_KICK_MIN) g_dutyMax = MOTOR_KICK_MIN;
  }
}
