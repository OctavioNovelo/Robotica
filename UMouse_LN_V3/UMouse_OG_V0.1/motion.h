#pragma once
#include "settings.h"
#include "sensors.h"

// =============================================================
//  UMouse — motion.h  (final)
//
//  DRV8871: ambos pines PWM
//    Motor izquierdo adelante = MODO B
//    Motor derecho   adelante = MODO A
//
//  Avance: encoder diferencial puro
//  Sin IR para avance, sin BMI160, sin preshift de retroceso
//  Detector de atasco por velocidad de ticks (no por conteo)
//  Giros: PWM normalizado por VBAT
// =============================================================

// ── PINES MOTORES ────────────────────────────────────────────
#define ML_IN1  19
#define ML_IN2  25
#define MR_IN1  18
#define MR_IN2  23

// ── PINES ENCODER ────────────────────────────────────────────
#define ENC_L_A  4
#define ENC_L_B  16
#define ENC_R_A  17
#define ENC_R_B  5

// ── ENCODERS ─────────────────────────────────────────────────
volatile long encL = 0, encL_abs = 0;
volatile long encR = 0, encR_abs = 0;

// Flag: true cuando el último movimiento fue wall alignment o atasco
// El próximo FWD debe descontar ENC_PRESHIFT porque ya está en posición
bool g_skipPreshift = false;

IRAM_ATTR void isrEncL_A(){
  bool a = digitalRead(ENC_L_A), b = digitalRead(ENC_L_B);
  encL += (long)((a==b?+1:-1) * ENC_SIGN_L);
  encL_abs++;
}
IRAM_ATTR void isrEncR_A(){
  bool a = digitalRead(ENC_R_A), b = digitalRead(ENC_R_B);
  encR += (long)((a==b?+1:-1) * ENC_SIGN_R);
  encR_abs++;
}

void resetEncoders(){
  noInterrupts();
  encL = encL_abs = encR = encR_abs = 0;
  interrupts();
}

void getEncAll(long &l, long &lA, long &r, long &rA){
  noInterrupts();
  l=encL; lA=encL_abs; r=encR; rA=encR_abs;
  interrupts();
}

// ── ESTADO MOTORES ───────────────────────────────────────────
int g_dutyMax = 60;

inline int   ci(int v,int lo,int hi)      { return v<lo?lo:v>hi?hi:v; }
inline float cf(float v,float lo,float hi){ return v<lo?lo:v>hi?hi:v; }

// ── DRV8871 ──────────────────────────────────────────────────
void motorCoastAll(){
  ledcWrite(ML_IN1,0); ledcWrite(ML_IN2,0);
  ledcWrite(MR_IN1,0); ledcWrite(MR_IN2,0);
}
void motorBrakeAll(){
  ledcWrite(ML_IN1,255); ledcWrite(ML_IN2,255);
  ledcWrite(MR_IN1,255); ledcWrite(MR_IN2,255);
}

void _modeA(uint8_t in1,uint8_t in2,uint8_t pwm){
  ledcWrite(in1,255); ledcWrite(in2,255-constrain(pwm,0,100));
}
void _modeB(uint8_t in1,uint8_t in2,uint8_t pwm){
  ledcWrite(in2,255); ledcWrite(in1,255-constrain(pwm,0,100));
}

void _motorSet(uint8_t in1,uint8_t in2,int speed,bool fwdIsA){
  speed = ci(speed,-g_dutyMax,+g_dutyMax);
  if(speed==0){ ledcWrite(in1,0); ledcWrite(in2,0); return; }
  int k=ci(MOTOR_KICK_MIN,0,g_dutyMax);
  int s=abs(speed); if(s<k) s=k;
  uint8_t pwm=(uint8_t)s;
  if(speed>0){ if(fwdIsA) _modeA(in1,in2,pwm); else _modeB(in1,in2,pwm); }
  else        { if(fwdIsA) _modeB(in1,in2,pwm); else _modeA(in1,in2,pwm); }
}

void motorL_set(int s){ _motorSet(ML_IN1,ML_IN2,s,false); }
void motorR_set(int s){ _motorSet(MR_IN1,MR_IN2,s,true);  }
void motorSetBoth(int l,int r){ motorL_set(l); motorR_set(r); }

// ── INIT ─────────────────────────────────────────────────────
void motionInit(){
  ledcAttach(ML_IN1,20000,8); ledcAttach(ML_IN2,20000,8);
  ledcAttach(MR_IN1,20000,8); ledcAttach(MR_IN2,20000,8);
  motorCoastAll();

  pinMode(ENC_L_A,INPUT); pinMode(ENC_L_B,INPUT);
  pinMode(ENC_R_A,INPUT); pinMode(ENC_R_B,INPUT);
  attachInterrupt(digitalPinToInterrupt(ENC_L_A), isrEncL_A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_R_A), isrEncR_A, CHANGE);
}

// ── PWM NORMALIZADO POR VBAT ─────────────────────────────────
int pwmForVolts(float targetV, float vbat){
  if(vbat<1.0f) return MOTOR_KICK_MIN;
  float d=(targetV/vbat)*100.0f;
  return ci((int)(d+0.5f), MOTOR_KICK_MIN, g_dutyMax);
}

// ── AVANCE CON CONTROL DIFERENCIAL ───────────────────────────
// Detector de atasco por VELOCIDAD de ticks, no por conteo.
// Con PWM alto las ruedas pueden patinan y seguir contando ticks
// aunque el robot no avance — este detector lo detecta.
//
// MIN_TICKS_PER_MS: tasa mínima esperada a FWD_PWM=45, batería 12.6V
// ~566 ticks en ~1750ms = ~0.32 t/ms. Umbral en 0.20 da margen seguro.
// Si dispara en condiciones normales: bajar a 0.15
// Si no detecta patinamiento: subir a 0.25
static const float MIN_TICKS_PER_MS = 0.20f;

static void _driveEncoder(int ticks, int basePWM){
  resetEncoders();
  const uint32_t TIMEOUT = 5000;
  uint32_t tStart  = millis();
  uint32_t tCtrl   = 0;
  uint32_t tVel    = millis();   // timer para chequeo de velocidad
  long     velRef  = 0;          // referencia de ticks para velocidad
  float    encI    = 0.0f;

  // Espera breve antes de activar detector — ignora arranque lento
  bool detectorActivo = false;

  while(true){
    long l,lA,r,rA;
    getEncAll(l,lA,r,rA);
    long avg = (lA+rA)/2;

    if(avg >= (long)ticks) break;
    if(millis()-tStart > TIMEOUT) break;

    // Activar detector solo después de los primeros 300ms
    // (el arranque desde cero siempre es lento)
    if(!detectorActivo && millis()-tStart > 300) detectorActivo=true;

    // Chequeo de velocidad cada 200ms
    if(detectorActivo && millis()-tVel >= 200){
      long avgNow = (lA+rA)/2;
      float rate  = (float)(avgNow - velRef) / 200.0f;  // ticks/ms
      velRef      = avgNow;
      tVel        = millis();

      if(rate < MIN_TICKS_PER_MS){
        // Tasa muy baja → ruedas patinando contra pared
        oledShow("ATASCO","retrocediendo");
        motorBrakeAll(); delay(40); motorCoastAll();
        // Reemplaza el while por tiempo por este basado en ticks:
        resetEncoders();
        uint32_t t0 = millis();
        while(true){
          long l,lA,r,rA; getEncAll(l,lA,r,rA);
          long avg = (lA+rA)/2;
          if(avg >= 75) break;          // 1ms = 0.2333333, 75ms = 17.5mm,  160ms =  37.5mm
          if(millis()-t0 > 1000) break;  // timeout de seguridad
          int revL = -basePWM;
          int revR = -(int)((float)basePWM * ENC_TRIM_R);
          motorL_set(revL);
          motorR_set(revR);
          delay(5);
        }
        g_skipPreshift = true;  // ← retrocedió al centro, preshift ya consumido
        motorBrakeAll(); delay(60); motorCoastAll();
        break;
      }
    }

    // Control diferencial cada 10ms
    if(millis()-tCtrl >= 10){
      tCtrl=millis();
      getEncAll(l,lA,r,rA);

      float err = (float)lA - (float)rA * ENC_TRIM_R;
      if(fabsf(err)<3.0f) err=0.0f;

      encI += KI_ENC * err;
      encI  = cf(encI,-KI_ENC_MAX,+KI_ENC_MAX);

      float u = KP_ENC*err + encI;
      u = cf(u,-(float)(basePWM/3),+(float)(basePWM/3));

      motorSetBoth(basePWM-(int)u, basePWM+(int)u);
    }
  }
  motorBrakeAll(); delay(60); motorCoastAll(); delay(60);
}

// ── GIRO ─────────────────────────────────────────────────────
static void _turnEncoder(int ticks, int sL, int sR){
  resetEncoders();
  const uint32_t TIMEOUT=3000;
  uint32_t tStart=millis(), tCtrl=0;

  while(true){
    long l,lA,r,rA; getEncAll(l,lA,r,rA);
    if((lA+rA)/2 >= (long)ticks) break;
    if(millis()-tStart > TIMEOUT) break;
    if(millis()-tCtrl >= 10){ tCtrl=millis(); motorSetBoth(sL,sR); }
  }
  motorBrakeAll(); delay(60); motorCoastAll(); delay(60);
}

// ── FUNCIONES DE FLOODFILL ───────────────────────────────────
void ff_alignToFrontWall(){
  const int ALIGN_PWM = MOTOR_KICK_MIN + 5;
  const uint32_t ALIGN_MS = 350; //300 para laberinto pequeño, 600 para laberinto grande
  uint32_t t0 = millis();
  while(millis()-t0 < ALIGN_MS){
    motorL_set(+ALIGN_PWM);
    motorR_set(+ALIGN_PWM);
    delay(5);
  }
  motorBrakeAll(); delay(60); motorCoastAll(); delay(60);
  g_skipPreshift = true;  // ← el robot quedó centrado
}

void ff_moveForwardOneCell(){
  oledShow("FWD","");
  if(g_skipPreshift){
    // Ya está en centro de celda — avanza solo CELL_FWD_MM - PRESHIFT_MM
    _driveEncoder(ENC_CELL_FWD - ENC_PRESHIFT, FWD_PWM);
    g_skipPreshift = false;
  } else {
    _driveEncoder(ENC_CELL_FWD, FWD_PWM);
  }
}

void ff_turnLeft90(){
  oledShow("TURN L","");
  // Si hay pared frontal, úsala para corregir el ángulo
  if(hasWallFront()) ff_alignToFrontWall();

  float vbat  = readVBAT_filtered();
  int fwdPwm  = pwmForVolts(FWD_VOLTS,  vbat);
  int turnPwm = pwmForVolts(TURN_VOLTS, vbat);
  _driveEncoder(ENC_PRESHIFT, fwdPwm);
  _turnEncoder (ENC_TURN90, -turnPwm, +turnPwm);
}

void ff_turnRight90(){
  oledShow("TURN R","");
  if(hasWallFront()) ff_alignToFrontWall();

  float vbat  = readVBAT_filtered();
  int fwdPwm  = pwmForVolts(FWD_VOLTS,  vbat);
  int turnPwm = pwmForVolts(TURN_VOLTS, vbat);
  _driveEncoder(ENC_PRESHIFT, fwdPwm);
  _turnEncoder (ENC_TURN90, +turnPwm, -turnPwm);
}

void ff_initialAdvance(){
  oledShow("INICIO","");
  _driveEncoder(ENC_INITIAL_ADVANCE, FWD_PWM);
}

bool ff_hasWallFront() { return hasWallFront(); }
bool ff_hasWallLeft()  { return hasWallLeft();  }
bool ff_hasWallRight() { return hasWallRight(); }
