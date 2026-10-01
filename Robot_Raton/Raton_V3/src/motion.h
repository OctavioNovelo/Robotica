#pragma once
#include "settings.h"
#include "pins.h"
#include "sensors.h"
#include "status.h"
#include "bno.h"

// =============================================================
//  UMouse S3 — motion.h
//
//  Base: motion.h original (probado). DRV8871: ambos pines PWM
//    Motor izquierdo adelante = MODO B, derecho adelante = MODO A
//
//  NAV_MODE 1: idéntico al original (encoders + IR).
//  NAV_MODE 2: BNO únicamente. Giros y rumbo por BNO; DISTANCIA por
//              TIEMPO (lazo abierto). Sin IR ni encoders.
//  NAV_MODE 3: BNO = rumbo/giros. Encoders = distancia, atasco, validación
//              y respaldo si el BNO falla. IR = paredes + alineación.
// =============================================================

// Tope de PWM que ya imponía el original en _modeA/_modeB (constrain(pwm,0,100))
#define MOTOR_PWM_CAP  100

// Definida en main.cpp: dibuja la página de brújula (heading/yaw en vivo).
void drawPageCompass();

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

// ── ESTADO DE NAVEGACIÓN (NAV_MODE 2/3) ──────────────────────
float    g_navHdgTarget = 0.0f;   // rumbo nominal del robot en el laberinto (0 = arranque, 90 = este...)
uint16_t g_navFallbacks = 0;      // NAV_MODE 3: movimientos que cayeron a encoders
const char* g_navWarn   = "";     // último aviso de validación

// Con BNO (NAV_MODE 2/3), mientras el robot se mueve, la OLED ya no muestra
// "FWD"/"TURN L"/"TURN R": se refresca la brújula en vivo (throttled).
static uint32_t g_uiLastMs = 0;
static inline void navUiTick(){
  if(!NAV_USE_BNO || !g_oledOK) return;
  uint32_t now = millis();
  if(now - g_uiLastMs < NAV_UI_REFRESH_MS) return;
  g_uiLastMs = now;
  drawPageCompass();
}

// ── MODO CRUCERO ─────────────────────────────────────────────
// Varias celdas RECTAS seguidas ya no frenan entre sí: _driveEncoder/_driveTimed
// terminan sin frenar (g_cruising=true) y la siguiente llamada de avance
// extiende la distancia sin reiniciar encoders. Solo se frena cuando algo
// realmente lo requiere: un giro, una alineación, un atasco, un error o el
// final del recorrido — ver motionHaltIfCruising().
static bool g_cruising = false;
static long g_cruiseTarget = 0;   // ticks (promedio L/R) acumulados objetivo desde el último frenado
inline void motionHaltIfCruising();   // definida más abajo (necesita motorBrakeAll/motorCoastAll)

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
  ledcWrite(in1,255); ledcWrite(in2,255-constrain(pwm,0,MOTOR_PWM_CAP));
}
void _modeB(uint8_t in1,uint8_t in2,uint8_t pwm){
  ledcWrite(in2,255); ledcWrite(in1,255-constrain(pwm,0,MOTOR_PWM_CAP));
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

inline void motionHaltIfCruising(){
  if(!g_cruising) return;
  motorBrakeAll(); delay(60); motorCoastAll(); delay(60);
  g_cruising = false;
}

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

// ── FALLO DE NAVEGACIÓN: parar y esperar BOOT para reiniciar ──
// El estado del laberinto en RAM ya no es confiable; el mapa guardado en NVS sí.
[[noreturn]] static void navFailStop(const char* msg){
  motorCoastAll();
  g_cruising = false;
  setStatus(ST_ERROR);
  if(g_oledOK){
    oledHeader("ERROR NAV");
    display.setCursor(0,12); display.println(msg);
    display.setCursor(0,30); display.println("BOOT: reiniciar");
    display.display();
  }
  while(digitalRead(BOOT_PIN)==HIGH) delay(20);   // esperar pulsación
  while(digitalRead(BOOT_PIN)==LOW)  delay(20);   // y liberación (evita modo descarga)
  delay(50);
  ESP.restart();
  while(true) delay(1000);
}

// ── SINCRONIZACIÓN CON EL BNO (NAV_MODE 2/3) ─────────────────
// Antes de cada movimiento: vacía datos viejos, espera muestra fresca,
// recupera (power-cycle) si no hay datos y re-referencia si el BNO se reinició.
static bool _navSync(){
  if(!NAV_USE_BNO) return false;
  bnoDrain();
  bool ok = bnoWaitFresh(150);
  if(!ok){
    setStatus(ST_BNO_INIT);
    oledShow("BNO sin datos","recuperando...");
    if(bnoRecover()) ok = bnoWaitFresh(300);
  }
  if(ok && g_bno.needRebase){ bnoRebaseTo(g_navHdgTarget); g_bno.needRebase = false; }
  return ok;
}

// Corrección diferencial por heading. Motores: L = base-u, R = base+u
// (misma convención que el control por encoders del original).
static float _hdgCorrection(float &hdgI, int basePWM){
  float err = wrap180(bnoHeading() - g_navHdgTarget);   // >0: desviado a la derecha
  if(fabsf(err) < HDG_DEADBAND_DEG) err = 0.0f;
  hdgI += KI_HDG * err;
  hdgI  = cf(hdgI, -KI_HDG_MAX, +KI_HDG_MAX);
  float u = KP_HDG*err + hdgI;
  return cf(u, -(float)(basePWM/3), +(float)(basePWM/3));
}

// ── AVANCE POR TIEMPO + HEADING (solo NAV_MODE 2) ────────────
// NO es odometría: mm -> ms con NAV2_MS_PER_MM a FWD_PWM. Sin retroalimentación
// de distancia: depende de batería y fricción. Sin detector de atasco.
static void _driveTimed(float mm, int basePWM){
  if(!_navSync()) navFailStop("BNO sin datos");
  setStatus(ST_STRAIGHT);
  // NAV2_MS_PER_MM está calibrado a FWD_PWM; a otro PWM se escala ~inverso (aprox.)
  const uint32_t durMs = (uint32_t)(mm * NAV2_MS_PER_MM * ((float)FWD_PWM / (float)basePWM) + 0.5f);
  const uint32_t t0 = millis();
  uint32_t tCtrl = 0;
  float hdgI = 0.0f;
  g_cruising = true;             // en crucero desde ya: si algo falla, navFailStop frena igual
  while(millis()-t0 < durMs){
    if(millis()-tCtrl >= 10){
      tCtrl = millis();
      navUiTick();
      bnoUpdate();
      if(!bnoFresh()) navFailStop("BNO perdido (recto)");
      float u = _hdgCorrection(hdgI, basePWM);
      motorSetBoth(basePWM-(int)u, basePWM+(int)u);
    }
  }
  // No frena aquí: sigue en modo crucero hasta el próximo giro/parada real.
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
  // NAV_MODE 2: sin encoders -> avance por tiempo (ticks solo se usa como distancia)
  if(!NAV_USE_ENC){ _driveTimed((float)ticks / TICKS_PER_MM, basePWM); return; }

  setStatus(ST_STRAIGHT);
  // NAV_MODE 3: rumbo por BNO (si no hay BNO válido, cae al control original por encoders)
  bool useBno = NAV_USE_BNO && _navSync();
  if(NAV_USE_BNO && !useBno) g_navFallbacks++;
  float hdgI = 0.0f;
  bool  lostBno = false;

  // Modo crucero: si ya veníamos en movimiento (celda recta anterior sin
  // frenar), NO se reinician los encoders — la distancia se acumula.
  bool wasCruising = g_cruising;
  if(!wasCruising){ resetEncoders(); g_cruiseTarget = 0; }
  g_cruiseTarget += ticks;
  g_cruising = true;

  const uint32_t TIMEOUT = 5000;
  uint32_t tStart  = millis();
  uint32_t tCtrl   = 0;
  uint32_t tVel    = millis();   // timer para chequeo de velocidad
  long     velRef  = 0;          // referencia de ticks para velocidad
  float    encI    = 0.0f;
  bool     reachedTarget = false;

  { long l,lA,r,rA; getEncAll(l,lA,r,rA); velRef = (lA+rA)/2; }

  // Espera breve antes de activar detector — ignora arranque lento.
  // Si ya veníamos en crucero no hace falta esperar: el robot ya está en movimiento.
  bool detectorActivo = wasCruising;

  while(true){
    long l,lA,r,rA;
    getEncAll(l,lA,r,rA);
    long avg = (lA+rA)/2;

    if(avg >= g_cruiseTarget){ reachedTarget = true; break; }
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
        g_cruising = false;     // el atasco rompió el crucero: ya quedó frenado
        return;
      }
    }

    // Control diferencial cada 10ms
    if(millis()-tCtrl >= 10){
      tCtrl=millis();
      getEncAll(l,lA,r,rA);
      navUiTick();

      float u;
      if(useBno) bnoUpdate();
      if(useBno && bnoFresh()){
        u = _hdgCorrection(hdgI, basePWM);          // NAV_MODE 3: rumbo por BNO
      } else {
        if(useBno && !lostBno){ lostBno = true; g_navFallbacks++; }
        float err = (float)lA - (float)rA * ENC_TRIM_R;   // respaldo / NAV_MODE 1: original
        if(fabsf(err)<3.0f) err=0.0f;

        encI += KI_ENC * err;
        encI  = cf(encI,-KI_ENC_MAX,+KI_ENC_MAX);

        u = KP_ENC*err + encI;
        u = cf(u,-(float)(basePWM/3),+(float)(basePWM/3));
      }

      motorSetBoth(basePWM-(int)u, basePWM+(int)u);
    }
  }

  if(!reachedTarget){
    // Timeout real (no fue el camino de atasco, que ya hizo return arriba):
    // esto sí es una parada real, no el fin normal de una celda.
    motorBrakeAll(); delay(60); motorCoastAll(); delay(60);
    g_cruising = false;
    g_navWarn  = "fwd timeout";
    return;
  }
  // Llegó a la celda: NO frena. Sigue en modo crucero hasta que algo
  // (giro, alineación, atasco, error o fin del recorrido) llame a
  // motionHaltIfCruising() / navFailStop().
}

// ── GIRO POR ENCODERS (NAV_MODE 1 y respaldo de NAV_MODE 3) ──
static void _turnEncoder(int ticks, int sL, int sR){
  motionHaltIfCruising();         // un giro siempre parte de velocidad cero
  resetEncoders();
  const uint32_t TIMEOUT=3000;
  uint32_t tStart=millis(), tCtrl=0;

  while(true){
    long l,lA,r,rA; getEncAll(l,lA,r,rA);
    if((lA+rA)/2 >= (long)ticks) break;
    if(millis()-tStart > TIMEOUT) break;
    navUiTick();
    if(millis()-tCtrl >= 10){ tCtrl=millis(); motorSetBoth(sL,sR); }
  }
  motorBrakeAll(); delay(60); motorCoastAll(); delay(60);
}

// ── GIRO CERRADO POR BNO (NAV_MODE 2 y 3) ────────────────────
// dir: +1 derecha (heading sube), -1 izquierda. El objetivo es ABSOLUTO
// (g_navHdgTarget ± 90), así el error de un giro no se acumula en el siguiente.
// Devuelve false si no pudo empezar (BNO sin datos): nada se movió.
static bool _turnBNO(int dir, int turnPwm){
  motionHaltIfCruising();         // un giro siempre parte de velocidad cero
  if(!_navSync()) return false;
  g_navHdgTarget = wrap360(g_navHdgTarget + dir*90.0f);
  if(NAV_USE_ENC) resetEncoders();

  const uint32_t t0 = millis();
  bool usedFallback = false;
  bool timedOut = true;
  while(millis()-t0 < BNO_TURN_TIMEOUT_MS){
    navUiTick();
    bnoUpdate();
    long l,lA,r,rA; getEncAll(l,lA,r,rA);
    long avg = (lA+rA)/2;

    if(bnoFresh()){
      float err  = wrap180(g_navHdgTarget - bnoHeading());   // >0: falta girar a la derecha
      float lead = fabsf(bnoRateDegS()) * BNO_TURN_LEAD_S;   // anticipa la inercia
      if(fabsf(err) <= BNO_TURN_TOL_DEG + lead){ timedOut = false; break; }
      // NAV_MODE 3: validación cruzada con encoders (BNO congelado / patinaje)
      if(NAV_USE_ENC && avg > (long)((float)ENC_TURN90 * BNO_TURN_ENC_MAX_FACTOR)){
        g_navWarn = "giro: enc>>BNO"; timedOut = false; break;
      }
      int s = (err > 0) ? +1 : -1;
      motorSetBoth(+s*turnPwm, -s*turnPwm);
    } else if(NAV_USE_ENC){
      usedFallback = true;                                    // BNO perdido a mitad de giro
      if(avg >= ENC_TURN90){ timedOut = false; break; }
      motorSetBoth(+dir*turnPwm, -dir*turnPwm);
    } else {
      navFailStop("BNO perdido (giro)");                      // NAV_MODE 2: no hay respaldo
    }
    delay(2);
  }
  motorBrakeAll(); delay(60); motorCoastAll(); delay(60);
  if(timedOut)      g_navWarn = "giro timeout";
  if(usedFallback)  g_navFallbacks++;

  // Ajuste fino: pulsos cortos hasta quedar dentro de tolerancia
  for(uint8_t i=0; i<BNO_TURN_FINE_TRIES; i++){
    if(!bnoWaitFresh(60)) break;
    float err = wrap180(g_navHdgTarget - bnoHeading());
    if(fabsf(err) <= BNO_TURN_FINE_TOL_DEG) break;
    int s = (err > 0) ? +1 : -1;
    uint32_t tp = millis();
    while(millis()-tp < BNO_TURN_FINE_PULSE_MS){ motorSetBoth(+s*MOTOR_KICK_MIN, -s*MOTOR_KICK_MIN); delay(2); }
    motorBrakeAll(); delay(50); motorCoastAll(); delay(60);
  }
  return true;
}

// dir: +1 derecha, -1 izquierda
static void _turn90(int dir, int turnPwm){
  if(NAV_USE_BNO && _turnBNO(dir, turnPwm)) return;
  if(!NAV_USE_ENC) navFailStop("BNO sin datos");             // NAV_MODE 2
  if(NAV_USE_BNO){                                           // NAV_MODE 3 sin BNO: respaldo
    g_navFallbacks++;
    g_navHdgTarget = wrap360(g_navHdgTarget + dir*90.0f);    // mantener rumbo nominal
  }
  _turnEncoder(ENC_TURN90, +dir*turnPwm, -dir*turnPwm);      // original: der (+,-), izq (-,+)
}

// Tras alinear con la pared frontal el robot está paralelo a la celda:
// si el BNO discrepa poco, se re-referencia (el IR corrige la deriva del BNO).
static void _navSnapAfterAlign(){
  if(NAV_MODE != 3 || BNO_SNAP_MAX_DEG <= 0.0f) return;
  if(!bnoWaitFresh(80)) return;
  float e = wrap180(bnoHeading() - g_navHdgTarget);
  if(fabsf(e) <= BNO_SNAP_MAX_DEG) bnoRebaseTo(g_navHdgTarget);
  else g_navWarn = "snap>limite";
}

// ── FUNCIONES DE FLOODFILL ───────────────────────────────────
void ff_alignToFrontWall(){
  motionHaltIfCruising();   // alinear exige partir de velocidad cero
  const int ALIGN_PWM = MOTOR_KICK_MIN + 5;
  const uint32_t ALIGN_MS = 350; //300 para laberinto pequeño, 600 para laberinto grande
  uint32_t t0 = millis();
  while(millis()-t0 < ALIGN_MS){
    navUiTick();
    motorL_set(+ALIGN_PWM);
    motorR_set(+ALIGN_PWM);
    delay(5);
  }
  motorBrakeAll(); delay(60); motorCoastAll(); delay(60);
  g_skipPreshift = true;  // ← el robot quedó centrado
}

// Con BNO (NAV_MODE 2/3) no se corta la marcha para avanzar celda por celda:
// mientras el camino sea recto, las celdas se encadenan sin frenar (ver
// g_cruising/motionHaltIfCruising) y la OLED solo muestra la brújula en vivo
// (navUiTick, dentro de _driveEncoder/_driveTimed). Sin BNO (NAV_MODE 1) se
// conserva el comportamiento original: "FWD" y un frenado real por celda.
void ff_moveForwardOneCell(){
  if(!NAV_USE_BNO) oledShow("FWD","");
  if(g_skipPreshift){
    // Ya está en centro de celda — avanza solo CELL_FWD_MM - PRESHIFT_MM
    _driveEncoder(ENC_CELL_FWD - ENC_PRESHIFT, FWD_PWM);
    g_skipPreshift = false;
  } else {
    _driveEncoder(ENC_CELL_FWD, FWD_PWM);
  }
}

static void _doTurn(int dir){
  motionHaltIfCruising();   // hay que girar: frenar el crucero recto antes de evaluar/alinear
  if(!NAV_USE_BNO) oledShow(dir>0 ? "TURN R" : "TURN L", "");
  setStatus(ST_TURN);
  // Si hay pared frontal, úsala para corregir el ángulo (solo con IR)
  if(NAV_USE_IR && hasWallFront()){ ff_alignToFrontWall(); _navSnapAfterAlign(); }

  float vbat  = readVBAT_filtered();
  int fwdPwm  = pwmForVolts(FWD_VOLTS,  vbat);
  int turnPwm = pwmForVolts(TURN_VOLTS, vbat);
  _driveEncoder(ENC_PRESHIFT, fwdPwm);   // termina en crucero (no frena)
  motionHaltIfCruising();                // frenar de verdad antes de girar
  setStatus(ST_TURN);
  _turn90(dir, turnPwm);
}

void ff_turnLeft90()  { _doTurn(-1); }
void ff_turnRight90() { _doTurn(+1); }

void ff_initialAdvance(){
  if(!NAV_USE_BNO) oledShow("INICIO","");
  _driveEncoder(ENC_INITIAL_ADVANCE, FWD_PWM);
}

// NAV_MODE 2: sin IR no hay detección de paredes (siempre false).
// Solo sirve recorriendo un mapa YA guardado en NVS (ver startRun en main.cpp).
bool ff_hasWallFront() { if(!NAV_USE_IR) return false; bool w = hasWallFront(); if(w) setStatus(ST_WALL); return w; }
bool ff_hasWallLeft()  { if(!NAV_USE_IR) return false; bool w = hasWallLeft();  if(w) setStatus(ST_WALL); return w; }
bool ff_hasWallRight() { if(!NAV_USE_IR) return false; bool w = hasWallRight(); if(w) setStatus(ST_WALL); return w; }
