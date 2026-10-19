#pragma once
#include "esp_timer.h"
#include "settings.h"
#include "pins.h"
#include "sensors.h"
#include "status.h"
#include "bno.h"
#include "navgeom.h"

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

// Diagnóstico por Serial + log WiFi (ver NAV_DEBUG y WIFI_LOG_ENABLE en settings.h, netlog.h). Solo en eventos, nunca en el lazo de 10 ms.
#if NAV_DEBUG
  #define NAVLOG(...) logPrintf(__VA_ARGS__)
#else
  #define NAVLOG(...) do{}while(0)
#endif

// Tope de PWM que ya imponía el original en _modeA/_modeB (constrain(pwm,0,100))
// 120 (2026-10-16, aprobado): la rueda EXTERIOR del giro en arco necesita ~1.55x el PWM de crucero.
// La recta nunca pasa de FWD_PWM + FWD_PWM/3 = 100; el límite real de 6 V es g_dutyMax (~123 a 12.4 V).
#define MOTOR_PWM_CAP  120

// Definidas en main.cpp: página de brújula y registro de la pose lógica (coords/dirección de Maus).
void drawPageCompass();
void navLogPose(const char* tag);

// ── ENCODERS ─────────────────────────────────────────────────
volatile long encL = 0, encL_abs = 0;
volatile long encR = 0, encR_abs = 0;

// Posición longitudinal del eje dentro de la celda (reemplaza al antiguo g_skipPreshift,
// que en el atasco suponía sin base "retrocedió al centro"). Ver navgeom.h.
NavGeom g_geom;

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
static bool  g_cruising = false;
static float g_cruiseTargetMM = 0.0f;  // mm acumulados objetivo desde el último frenado (se redondea a ticks
                                       // una sola vez: sin error de truncado acumulado en crucero)
// Ticks (promedio L/R) retrocedidos en recuperaciones de atasco desde el último reinicio de encoders.
// Los contadores *_abs suben en ambos sentidos: posición real = abs - 2*retrocedido. Así no hace falta
// fiarse de los signos ENC_SIGN_* (el control nunca los usó) y el retroceso NO borra la posición.
static long  g_backTicks = 0;
// Signo real de encL/encR (los contadores con signo nunca se usaron en el control y ENC_SIGN_* no está
// verificado). Se aprende en cada tramo recto: avanzando, el contador con signo debe subir.
static int   g_sgnL = 1, g_sgnR = 1;
static bool  g_lastSegFrontBrake = false;
// true = exploración (mapa desconocido): la confirmación del frontal llega con tope de velocidad. Con mapa previo las
// paredes del camino ya se conocen y va sin tope. Lo fija main.cpp al empezar cada corrida.
bool g_navExploring = true;
static bool g_nextTurnHint = true;   // FloodFill: en la celda siguiente prevé girar o parar
void ff_hintNext(bool turnOrStop){ g_nextTurnHint = turnOrStop; }   // el último avance frenó por pared frontal (FRENO-IR)
static float g_lastOvershootMM = 0.0f;

static inline long mmToTicks(float mm){ return lroundf(mm * TICKS_PER_MM); }
static inline long _encAvgAbs(){ long l,lA,r,rA; getEncAll(l,lA,r,rA); return (lA+rA)/2; }
static inline long _posTicks(){ return _encAvgAbs() - 2*g_backTicks; }
// Odómetro de la prueba de recta (CAL_STRAIGHT_CELLS): acumula cada tramo antes de reiniciar encoders.
static long  g_odoTicks = 0;
static inline void _frameReset(){ g_odoTicks += _posTicks(); resetEncoders(); g_backTicks = 0; g_cruiseTargetMM = 0.0f; }

// Deadman de motores (ver MOTION_WD_MS): cada comando de motor lo "alimenta".
static volatile uint32_t g_motorCmdMs   = 0;
static volatile bool     g_motorsActive = false;
static volatile bool     g_wdTripped    = false;
static bool g_alignSquare = false;   // el último posicionamiento IR dejó FL≈FR (robot paralelo a la pared)
inline void motionHaltIfCruising();   // definida más abajo (necesita motorBrakeAll/motorCoastAll)

// ── DRV8871 ──────────────────────────────────────────────────
void motorCoastAll(){
  g_motorsActive = false;
  ledcWrite(ML_IN1,0); ledcWrite(ML_IN2,0);
  ledcWrite(MR_IN1,0); ledcWrite(MR_IN2,0);
}
void motorBrakeAll(){
  g_motorsActive = false;
  ledcWrite(ML_IN1,255); ledcWrite(ML_IN2,255);
  ledcWrite(MR_IN1,255); ledcWrite(MR_IN2,255);
}

void _modeA(uint8_t in1,uint8_t in2,uint8_t pwm){
  ledcWrite(in1,255); ledcWrite(in2,255-constrain(pwm,0,MOTOR_PWM_CAP));
}
void _modeB(uint8_t in1,uint8_t in2,uint8_t pwm){
  ledcWrite(in2,255); ledcWrite(in1,255-constrain(pwm,0,MOTOR_PWM_CAP));
}

// kick=false: sin piso MOTOR_KICK_MIN (solo con el robot YA en marcha: la rueda interior del arco va a ~10-30 PWM;
// con el decaimiento lento del DRV8871 un PWM bajo frena la rueda hasta esa velocidad).
void _motorSet(uint8_t in1,uint8_t in2,int speed,bool fwdIsA,bool kick=true){
  g_motorCmdMs = millis();
  if(speed != 0) g_motorsActive = true;
  speed = ci(speed,-g_dutyMax,+g_dutyMax);
  if(speed==0){ ledcWrite(in1,0); ledcWrite(in2,0); return; }
  int k=kick ? ci(MOTOR_KICK_MIN,0,g_dutyMax) : 1;
  int s=abs(speed); if(s<k) s=k;
  uint8_t pwm=(uint8_t)s;
  if(speed>0){ if(fwdIsA) _modeA(in1,in2,pwm); else _modeB(in1,in2,pwm); }
  else        { if(fwdIsA) _modeB(in1,in2,pwm); else _modeA(in1,in2,pwm); }
}

void motorL_set(int s){ _motorSet(ML_IN1,ML_IN2,s,false); }
void motorR_set(int s){ _motorSet(MR_IN1,MR_IN2,s,true);  }
void motorSetBoth(int l,int r){ motorL_set(l); motorR_set(r); g_motorsActive = (l != 0 || r != 0); }
void motorSetBothNoKick(int l,int r){
  _motorSet(ML_IN1,ML_IN2,l,false,false); _motorSet(MR_IN1,MR_IN2,r,true,false); g_motorsActive = (l != 0 || r != 0);
}

// Frena el crucero y MIDE cuánto pasó el robot del último objetivo (lectura de ~36 ms en marcha +
// frenado). Ese sobrepaso se suma a la posición (g_geom.s): el preshift siguiente lo descuenta.
inline void motionHaltIfCruising(){
  if(!g_cruising) return;
  motorBrakeAll(); delay(60); motorCoastAll(); delay(60);
  g_cruising = false;
  if(NAV_USE_ENC){
    g_lastOvershootMM = (float)(_posTicks() - mmToTicks(g_cruiseTargetMM)) / TICKS_PER_MM;
    g_geom.advance(g_lastOvershootMM);
    NAVLOG("[HALT] sobrepaso=%.1fmm s=%.1f\n", g_lastOvershootMM, g_geom.s);
  }
}

// ── DEADMAN ──────────────────────────────────────────────────
// Corre en la tarea de esp_timer (prioridad alta, NO es ISR): sigue funcionando aunque el
// lazo principal esté colgado dentro de una transacción I2C.
static esp_timer_handle_t g_wdTimer = nullptr;
static void motionWdCb(void*){
  if(g_motorsActive && (uint32_t)(millis() - g_motorCmdMs) > MOTION_WD_MS){
    motorBrakeAll();            // también pone g_motorsActive=false
    g_cruising  = false;
    g_wdTripped = true;
  }
}
static void motionWdStart(){
  esp_timer_create_args_t a = {};
  a.callback        = &motionWdCb;
  a.arg             = nullptr;
  a.dispatch_method = ESP_TIMER_TASK;
  a.name            = "motionWd";
  if(esp_timer_create(&a, &g_wdTimer) == ESP_OK) esp_timer_start_periodic(g_wdTimer, 50000);  // 50 ms
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
  motionWdStart();
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
  motorBrakeAll(); delay(60); motorCoastAll();
  g_cruising = false;
  setStatus(ST_ERROR);
  irSnapInvalidate();
  NAVLOG("[FALLO] %s  (coords NO actualizadas; mapa NVS intacto)\n", msg);
  navLogPose("FALLO");
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

// ── CENTRADO LATERAL POR IR (corredor con pared a ambos lados) ──
// Lectura rápida (pocas muestras) y auto-limitada: no se puede leer L+R en
// cada tick de 10ms sin robarle tiempo al control (ver IR_CENTER_PERIOD_MS).
// Devuelve 0 si no toca refrescar aún, o si no hay pared buena a ambos lados.
static uint32_t g_irCenterLastMs = 0;
static float    g_irCenterErr    = 0.0f;
static uint32_t g_irCenterSeq    = 0;     // sube con cada lectura nueva (la usa la ventana de paredes)
static int      g_irCenterL = 0, g_irCenterR = 0;

static inline float irCenterTick(){
  if(!NAV_USE_IR) return 0.0f;
  uint32_t now = millis();
  if(now - g_irCenterLastMs < IR_CENTER_PERIOD_MS) return g_irCenterErr;
  g_irCenterLastMs = now;
  int l = readDiff_mV_n(PIN_IR_L, PIN_FT_L, IR_CENTER_NSAMPLES);
  int r = readDiff_mV_n(PIN_IR_R, PIN_FT_R, IR_CENTER_NSAMPLES);
  g_irCenterL = l; g_irCenterR = r; g_irCenterSeq++;
  g_irCenterErr = irCenteringError(l, r);
  return g_irCenterErr;
}

// Desvío de rumbo (grados, >0 = hacia la derecha) que pide el centrado IR.
// e>0 (más cerca de la derecha) => desvío negativo (girar a la izquierda).
static inline float irBiasDeg(){
#if IR_CENTER_ENABLE
  float d = -irCenterTick() * KP_IR_CENTER_DEG;
  return (d < -IR_CENTER_MAX_DEG) ? -IR_CENTER_MAX_DEG : (d > IR_CENTER_MAX_DEG ? IR_CENTER_MAX_DEG : d);
#else
  return 0.0f;
#endif
}
// Grados -> diferencia de ticks entre ruedas (para el respaldo por encoders): theta*TRACK*ticks/mm
#define TICKS_PER_DEG  (0.0174533f * TRACK_WIDTH_MM * TICKS_PER_MM)

// ── SINCRONIZACIÓN CON EL BNO (NAV_MODE 2/3) ─────────────────
// Antes de cada movimiento: vacía datos viejos, espera muestra fresca,
// recupera (power-cycle) si no hay datos y re-referencia si el BNO se reinició.
static bool _navSync(){
  if(!NAV_USE_BNO) return false;
  bnoDrain();
  bool ok = bnoWaitFresh(150);
  if(!ok){
    // bnoRecover() bloquea >1 s: con el robot en crucero (motores latcheados al último
    // PWM) esto era un "manejar a ciegas" contra la pared. Frenar ANTES de recuperar.
    motionHaltIfCruising();
    setStatus(ST_BNO_INIT);
    oledShow("BNO sin datos","recuperando...");
    if(bnoRecover()) ok = bnoWaitFresh(300);
  }
  if(ok && g_bno.needRebase){ bnoRebaseTo(g_navHdgTarget); g_bno.needRebase = false; }
  return ok;
}

// Corrección diferencial por heading. Motores: L = base-u, R = base+u
// (misma convención que el control por encoders del original).
static float _hdgCorrection(float &hdgI, int basePWM, float offDeg = 0.0f){
  // offDeg: desvío del rumbo objetivo pedido por el centrado IR (grados; >0 = hacia la derecha)
  float err = wrap180(bnoHeading() - (g_navHdgTarget + offDeg));   // >0: desviado a la derecha
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
    if(g_wdTripped) navFailStop("WD: motores");
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

// Resultado de un avance. Los fallos irrecuperables (timeout, atasco persistente, watchdog)
// NO vuelven: navFailStop() detiene el robot. Así Maus::move() nunca actualiza coords/dirección
// sobre un movimiento no completado (maus.h queda idéntico al original).
enum MoveResult : uint8_t { MOVE_OK = 0, MOVE_SHORT };
static float g_moveShortMM = 0.0f;   // MOVE_SHORT: mm que faltaron (detenido y medido antes del objetivo)

// Ventana de validación de paredes laterales (IR_WALL_CONFIRM): cuenta, en los últimos
// IR_WALL_WINDOW_MM antes del punto de lectura, cuántas lecturas rápidas de L/R vieron pared.
struct WallWindow { bool armed = false; long fromTicks = 0; uint8_t n = 0, hitL = 0, hitR = 0; };
static WallWindow g_win;
static uint32_t   g_winSeq = 0;

// mm: distancia de este tramo (se acumula al objetivo absoluto si venía en crucero).
// scanFollows: después de este avance FloodFill lee paredes (aplica SCAN_LEAD_MM y la ventana).
// pivotAheadMM: el punto de giro está esto más allá del objetivo (frenado por perfil). TURN_MODE 2: el avance a la
// lectura temprana termina ARC_READ_BACK_MM antes del pivote.
// vEndMMs > 0: llegar al objetivo a esa velocidad como máximo (perfil sqrt(vEnd² + 2·A·resto)), haya o no pared vista.
// TURN_MODE 2: la confirmación del frontal. Los frontales ven la pared ~20 mm antes de tocarla con la cinta: a 425 mm/s
// no da para frenar (log 2026-10-16: sobrepaso 19-23 mm => golpe).
static MoveResult _driveEncoder(float mm, int basePWM, bool scanFollows, float pivotAheadMM = READ_BACK_MM, float vEndMMs = 0.0f){
  // NAV_MODE 2: sin encoders -> avance por tiempo
  if(!NAV_USE_ENC){ _driveTimed(mm, basePWM); return MOVE_OK; }

  setStatus(ST_STRAIGHT);
  // NAV_MODE 3: rumbo por BNO (si no hay BNO válido, cae al control original por encoders)
  if(g_wdTripped) navFailStop("WD: motores");
  const bool cruisingBefore = g_cruising;
  bool useBno = NAV_USE_BNO && _navSync();
  // _navSync() pudo frenar el crucero (recuperación del BNO): el sobrepaso medido ya se recorrió.
  if(cruisingBefore && !g_cruising){ mm -= g_lastOvershootMM; if(mm < 0.0f) mm = 0.0f; }
  if(NAV_USE_BNO && !useBno){ g_navFallbacks++; g_navWarn = "BNO no disponible"; setStatus(ST_ERROR); NAVLOG("[BNO] no disponible: avance por encoders\n"); }
  float hdgI = 0.0f;
  bool  lostBno = false;

  // Modo crucero: si ya veníamos en movimiento (celda recta anterior sin
  // frenar), NO se reinician los encoders — el objetivo es absoluto y se acumula.
  bool wasCruising = g_cruising;
  if(!wasCruising) _frameReset();
  g_cruiseTargetMM += mm;
  const long target = mmToTicks(g_cruiseTargetMM);
  const long stopAt = target - (scanFollows ? mmToTicks(SCAN_LEAD_MM) : 0);
  g_cruising = true;

  g_win = WallWindow();
  g_winSeq = g_irCenterSeq;
  // TURN_MODE 2: la ventana antes de la lectura TEMPRANA miraría la celda anterior y podría vetar una pared real.
  if(scanFollows && (TURN_MODE != 2 || pivotAheadMM <= READ_BACK_MM + 1.0f)){
    g_win.armed = true; g_win.fromTicks = target - mmToTicks(IR_WALL_WINDOW_MM);
  }

  uint32_t tStart  = millis();
  uint32_t tCtrl   = 0;
  uint32_t tVel    = millis();   // timer para chequeo de velocidad
  long     velRef  = _encAvgAbs();
  long     velRefL, velRefR;            // tasa POR RUEDA: con una rueda bloqueada el promedio sigue subiendo
  long     segL0, segR0;                // conteos al empezar el tramo (divergencia L/R)
  long     segSL0, segSR0;              // conteos CON SIGNO al empezar (aprendizaje de g_sgnL/R)
  getEncAll(segSL0,segL0,segSR0,segR0); velRefL = segL0; velRefR = segR0;
  float    encI    = 0.0f;
  uint8_t  stalls  = 0;
  const long pos0  = _posTicks();

  // Espera breve antes de activar detector — ignora arranque lento.
  // Si ya veníamos en crucero no hace falta esperar: el robot ya está en movimiento.
  bool detectorActivo = wasCruising;

  // Rampa de arranque: si salía de una parada empieza en FWD_START_PWM; en crucero conserva
  // el PWM que llevaba (un tramo corto puede terminar a mitad de rampa).
  static int s_ramp = 0;
  if(!wasCruising || s_ramp <= 0) s_ramp = basePWM < FWD_START_PWM ? basePWM : FWD_START_PWM;
  const int rampStart = s_ramp;

  // Frenado por pared frontal: se activa al verla y se mantiene hasta el final del tramo.
  bool     irBrake = false;
  long     irBrakePos = 0;
  uint32_t tFront = 0;
  int      frontLvl = 0, cmdPrev = s_ramp, cmdMin = basePWM;
#if IR_BRAKE_PROFILE
  // Perfil: solo en avances a celda (scanFollows). El punto de giro está pivotAheadMM más allá del objetivo.
  const bool profOk  = scanFollows;
  const long pivotAt = target + mmToTicks(pivotAheadMM);
  const int  brakeThr = IR_BRAKE_START_THR;
  float    vNow = 0.0f, vProf0 = 0.0f, vAtBrake = 0.0f;   // mm/s (encoder, ventana de 20 ms)
  long     vPos = pos0;
  uint32_t vMs  = millis();
  uint16_t profBrakes = 0;                                 // ticks de freno activo
  bool     brakeHi = false;                                // disparó un solo frontal (IR_BRAKE_ONE_THR)
  bool     capOn = false;                                  // tope de velocidad de llegada (vEndMMs) actuando
  uint16_t capBrakes = 0;
  long     seen30 = -1, seen50 = -1;                       // posición en que max(FL,FR) pasó de 30 / 50 (diagnóstico)
#else
  const int  brakeThr = IR_BRAKE_THR;
#endif

  while(true){
    long l,lA,r,rA;
    getEncAll(l,lA,r,rA);
    long pos = (lA+rA)/2 - 2*g_backTicks;

    if(g_wdTripped) navFailStop("WD: motores");
    // Una rueda avanza y la otra no (choque lateral / rueda bloqueada / patinaje): el promedio
    // NO es la posición. Antes esto contaba celdas que el robot no recorrió.
    if(labs((lA - segL0) - (rA - segR0)) > mmToTicks(WHEEL_DIVERGE_MM)){
      motorBrakeAll(); delay(60); motorCoastAll();
      g_cruising = false;
      NAVLOG("[RUEDA] divergencia L=%+ld R=%+ld ticks en el tramo (max %ld): rueda bloqueada o choque\n",
             lA - segL0, rA - segR0, mmToTicks(WHEEL_DIVERGE_MM));
      navFailStop(lA - segL0 < rA - segR0 ? "rueda IZQ bloqueada" : "rueda DER bloqueada");
    }
    if(pos >= stopAt) break;
    if(millis()-tStart > FWD_TIMEOUT_MS){
      motorBrakeAll(); delay(60); motorCoastAll();
      g_cruising = false;
      NAVLOG("[FWD] TIMEOUT obj=%ld pos=%ld faltan=%.1fmm\n", target, pos, (float)(target-pos)/TICKS_PER_MM);
      navFailStop("avance: timeout");
    }

    // Activar detector solo después de los primeros 300ms
    // (el arranque desde cero siempre es lento)
    if(!detectorActivo && millis()-tStart > 300) detectorActivo=true;

    // Chequeo de velocidad cada 200ms
    if(detectorActivo && millis()-tVel >= 200){
      long avgNow = (lA+rA)/2;
      const float dt = (float)(millis() - tVel);
      const float rateL = (float)(lA - velRefL) / dt, rateR = (float)(rA - velRefR) / dt;
      float rate  = rateL < rateR ? rateL : rateR;     // la rueda MÁS LENTA (ticks/ms)
      velRef      = avgNow; velRefL = lA; velRefR = rA;
      tVel        = millis();

      if(rate < MIN_TICKS_PER_MS){
        // Tasa muy baja: bloqueo mecánico (no se sabe la causa: pared no mapeada, poste, rueda).
        const long remain = target - pos;
        motorBrakeAll(); delay(40); motorCoastAll(); delay(60);
        g_cruising = false;
        stalls++;
        NAVLOG("[ATASCO] #%u rate L=%.2f R=%.2f t/ms obj=%ld pos=%ld faltan=%.1fmm\n",
               stalls, rateL, rateR, target, pos, (float)remain / TICKS_PER_MM);
        oledShow("ATASCO","retrocediendo");
        // Retroceso MEDIDO: la posición se conserva (g_backTicks), no se reinician encoders.
        const long a0 = _encAvgAbs();
        const uint32_t t0 = millis();
        while(_encAvgAbs() - a0 < STALL_BACK_TICKS && millis()-t0 < 1000){
          motorL_set(-basePWM);
          motorR_set(-(int)((float)basePWM * ENC_TRIM_R));
          delay(5);
        }
        motorBrakeAll(); delay(60); motorCoastAll(); delay(60);
        g_backTicks += _encAvgAbs() - a0;
        const long posNow = _posTicks();

        if(remain <= mmToTicks(STALL_ACCEPT_MM)){
          // Tope físico casi en el objetivo: se considera llegado; posición real medida.
          g_moveShortMM = (float)(target - posNow) / TICKS_PER_MM;
          g_win.armed = false;
          g_navWarn = "atasco ~objetivo";
          NAVLOG("[ATASCO] aceptado (tope a %.1fmm del objetivo); quedo %.1fmm antes\n",
                 (float)remain / TICKS_PER_MM, g_moveShortMM);
          return MOVE_SHORT;
        }
        if(stalls > STALL_MAX_RETRIES) navFailStop("ATASCO: no avanza");
        NAVLOG("[ATASCO] reintento hacia el MISMO objetivo desde pos=%ld\n", posNow);
        g_win.armed = false;          // la ventana ya no describe un pase continuo
        g_cruising = true;
        tStart = millis(); tVel = millis(); tCtrl = 0;
        velRef = _encAvgAbs();
        { long l0,r0; getEncAll(l0,segL0,r0,segR0); velRefL = segL0; velRefR = segR0; }
        detectorActivo = false; encI = 0.0f; hdgI = 0.0f;
        s_ramp = basePWM < FWD_START_PWM ? basePWM : FWD_START_PWM;   // reintento: arranca otra vez suave
        continue;
      }
    }

    // Control diferencial cada 10ms
    if(millis()-tCtrl >= 10){
      tCtrl=millis();
      getEncAll(l,lA,r,rA);
#if NAV_UI_IN_DRIVE
      navUiTick();
#endif

      if(s_ramp < basePWM) s_ramp = (s_ramp + FWD_RAMP_STEP < basePWM) ? s_ramp + FWD_RAMP_STEP : basePWM;
      else s_ramp = basePWM;
      int cmd = s_ramp;
      bool brakeNow = false;                                 // perfil: freno activo en este tick
#if IR_BRAKE_PROFILE
      if(tCtrl - vMs >= 20){
        const float vi = (float)(pos - vPos) * 1000.0f / (TICKS_PER_MM * (float)(tCtrl - vMs));
        vNow = (vNow <= 0.0f) ? vi : 0.5f * vNow + 0.5f * vi;
        vPos = pos; vMs = tCtrl;
      }
#endif
#if IR_BRAKE_ENABLE
      if(NAV_USE_IR && basePWM > IR_BRAKE_PWM){
        if(millis()-tFront >= IR_FRONT_PERIOD_MS && (IR_BRAKE_SMOOTH || IR_BRAKE_PROFILE || !irBrake)){
          tFront = millis();
          const int fl = readDiff_mV_n(PIN_IR_FL, PIN_FT_FL, IR_CENTER_NSAMPLES);
          const int fr = readDiff_mV_n(PIN_IR_FR, PIN_FT_FR, IR_CENTER_NSAMPLES);
          const int lvl = fl < fr ? fl : fr;
          if(lvl > frontLvl) frontLvl = lvl;                 // el nivel solo sube (la pared se acerca)
#if IR_BRAKE_PROFILE
          const int hi = fl > fr ? fl : fr;
          if(seen30 < 0 && hi >= 30) seen30 = pos;
          if(seen50 < 0 && hi >= 50) seen50 = pos;
          const bool trig = lvl >= brakeThr || (profOk && IR_BRAKE_ONE_THR > 0 && hi >= IR_BRAKE_ONE_THR);
          if(trig && !irBrake) brakeHi = (lvl < brakeThr);
#else
          const bool trig = lvl >= brakeThr;
#endif
#if IR_BRAKE_PROFILE
          if(profOk && (float)(pivotAt - pos) / TICKS_PER_MM > IR_BRAKE_ARM_MM) {}   // aún lejos: no armar
          else
#endif
          if(!irBrake && trig){ irBrake = true; irBrakePos = pos;
#if IR_BRAKE_PROFILE
            vAtBrake = vNow;
#endif
          }
        }
#if IR_BRAKE_PROFILE
        if(irBrake && profOk){
          // Hay pared al frente => el siguiente movimiento es un giro: llegar al punto de giro despacio.
          float remMM = (float)(pivotAt - pos) / TICKS_PER_MM;
          if(remMM < 0.0f) remMM = 0.0f;
          float vref = sqrtf(2.0f * IR_BRAKE_ACCEL * remMM);
          if(vref < IR_BRAKE_VMIN) vref = IR_BRAKE_VMIN;
          if(vProf0 <= 0.0f) vProf0 = vNow > IR_BRAKE_VMIN ? vNow : IR_BRAKE_VMIN;   // velocidad de crucero al ver la pared
          int want = (int)((float)basePWM * vref / vProf0 + 0.5f);
          want = ci(want, IR_BRAKE_PWM_MIN, basePWM);
          if(want < cmd) cmd = want;
          if(vNow > vref * IR_BRAKE_OVER){ brakeNow = true; profBrakes++; }
        } else
#endif
        if(irBrake){
#if IR_BRAKE_SMOOTH
          const int full = g_irPivot > 0 ? (int)(g_irPivot * IR_BRAKE_FULL_FRAC) : 300;
          float f = full > IR_BRAKE_THR ? (float)(frontLvl - IR_BRAKE_THR) / (float)(full - IR_BRAKE_THR) : 1.0f;
          f = f < 0.0f ? 0.0f : (f > 1.0f ? 1.0f : f);
          const int want = basePWM - (int)((basePWM - IR_BRAKE_PWM) * f + 0.5f);
          if(want < cmd) cmd = want;
#else
          cmd = IR_BRAKE_PWM;
#endif
        }
      }
#endif
#if IR_BRAKE_PROFILE
      if(vEndMMs > 0.0f && !(irBrake && profOk)){
        float rem = (float)(target - pos) / TICKS_PER_MM;
        if(rem < 0.0f) rem = 0.0f;
        const float vcap = sqrtf(vEndMMs * vEndMMs + 2.0f * CONFIRM_ACCEL * rem);
        // Lazo abierto: PWM ~ velocidad (ARC_MMS_PER_PWM). Saliendo de parada vNow se atrasa (log: decía 363 mm/s y el
        // pre-freno midió 551) y el tope no actuaba; esto limita el PWM aunque la medida vaya tarde.
        const int ffCap = (int)(vcap / ARC_MMS_PER_PWM + 0.5f);
        if(ffCap < cmd){ cmd = ffCap < IR_BRAKE_PWM_MIN ? IR_BRAKE_PWM_MIN : ffCap; capOn = true; }
        if(capOn || vNow > vcap){
          capOn = true;
          if(vProf0 <= 0.0f) vProf0 = vNow > vEndMMs ? vNow : vEndMMs;
          int want = (int)((float)basePWM * vcap / vProf0 + 0.5f);
          want = ci(want, IR_BRAKE_PWM_MIN, basePWM);
          if(want < cmd) cmd = want;
          if(vNow > vcap * CONFIRM_OVER){ brakeNow = true; capBrakes++; }
        }
      }
#endif
#if IR_BRAKE_SMOOTH
#if IR_BRAKE_PROFILE
      if(!(irBrake && profOk) && !capOn)                       // el perfil decide su propia deceleración
#endif
      if(cmd < cmdPrev - IR_BRAKE_DECEL) cmd = cmdPrev - IR_BRAKE_DECEL;   // desaceleración limitada
#endif
      cmdPrev = cmd;
      if(cmd < cmdMin) cmdMin = cmd;

      float u;
      float biasDeg = irBiasDeg();                   // 0 si no hay IR, calibración o paredes utilizables
#if IR_WALL_CONFIRM
      if(g_win.armed && pos >= g_win.fromTicks){
        irCenterTick();                              // cacheada: solo lee si toca (IR_CENTER_PERIOD_MS)
        if(g_irCenterSeq != g_winSeq && g_win.n < 255){
          g_winSeq = g_irCenterSeq;
          g_win.n++;
          if(g_irCenterL >= IR_WALL_THR_L) g_win.hitL++;
          if(g_irCenterR >= IR_WALL_THR_R) g_win.hitR++;
        }
      }
#endif
      if(useBno) bnoUpdate();
      if(useBno && bnoFresh()){
        u = _hdgCorrection(hdgI, cmd, biasDeg);  // NAV_MODE 3: rumbo por BNO (+ desvío del centrado IR)
      } else {
        if(useBno && !lostBno){ lostBno = true; g_navFallbacks++; g_navWarn = "BNO perdido"; setStatus(ST_ERROR); }
        float err = (float)lA - (float)rA * ENC_TRIM_R;   // respaldo / NAV_MODE 1: original
        err -= biasDeg * TICKS_PER_DEG;                   // desvío pedido por el centrado IR
        if(fabsf(err)<3.0f) err=0.0f;

        encI += KI_ENC * err;
        encI  = cf(encI,-KI_ENC_MAX,+KI_ENC_MAX);

        u = KP_ENC*err + encI;
        u = cf(u,-(float)(cmd/3),+(float)(cmd/3));
      }


      if(brakeNow) motorBrakeAll();          // va más rápido que el perfil: freno activo (10 ms)
      else         motorSetBoth(cmd-(int)u, cmd+(int)u);
    }
  }

  long l,lA,r,rA; getEncAll(l,lA,r,rA);
  g_lastSegFrontBrake = irBrake;
  if(stalls == 0){
    const long dAL = lA - segL0, dAR = rA - segR0, dSL = l - segSL0, dSR = r - segSR0;
    if(dAL > 200 && labs(dSL) * 5 > dAL * 4){ const int sg = dSL > 0 ? 1 : -1; if(sg != g_sgnL){ g_sgnL = sg; NAVLOG("[ENC] signo L = %d (abs %ld, con signo %ld)\n", sg, dAL, dSL); } }
    if(dAR > 200 && labs(dSR) * 5 > dAR * 4){ const int sg = dSR > 0 ? 1 : -1; if(sg != g_sgnR){ g_sgnR = sg; NAVLOG("[ENC] signo R = %d (abs %ld, con signo %ld)\n", sg, dAR, dSR); } }
  }
  NAVLOG("[FWD] %.1fmm obj=%ld tramo=%ldt (L=%ld R=%ld abs) crucero=%d atascos=%u t=%lums%s\n",
         mm, target, _posTicks() - pos0, lA, rA, wasCruising ? 1 : 0, stalls,
         (unsigned long)(millis() - tStart), lostBno ? " BNO-perdido" : "");
  if(!wasCruising && rampStart < basePWM) NAVLOG("[RAMPA] arranque %d -> %d (llegó a %d)\n", rampStart, basePWM, s_ramp);
#if IR_BRAKE_PROFILE
  if(irBrake && profOk){
    // Durante la lectura (en marcha) y la decisión, seguir despacio y recto; el próximo avance
    // en crucero (si FloodFill decide seguir) sube con la rampa desde aquí, sin tirón.
    // Log 2026-10-16: el tramo termina 10–17 mm después de ver la pared (3 ticks): el perfil casi no
    // actúa dentro del lazo y llegaba a ~425 mm/s. Si aún va más rápido que el perfil, frena DURANTE la lectura.
    float remEnd = (float)(pivotAt - _posTicks()) / TICKS_PER_MM;
    if(remEnd < 0.0f) remEnd = 0.0f;
    const float vrefEnd = sqrtf(2.0f * IR_BRAKE_ACCEL * remEnd);
    const bool brakeScan = vNow > vrefEnd;
    if(brakeScan) motorBrakeAll();
    else          motorSetBoth(IR_BRAKE_PWM_MIN, IR_BRAKE_PWM_MIN);
    s_ramp = IR_BRAKE_PWM_MIN;
    NAVLOG("[FRENO-IR] perfil: pared vista a %.1fmm del pivote (%s) v=%.0f -> %.0fmm/s al llegar (perfil %.0f) %s, PWM min %d, freno %u ticks, nivel %d\n",
           (float)(pivotAt - irBrakePos) / TICKS_PER_MM, brakeHi ? "1 sensor" : "ambos", vAtBrake, vNow, vrefEnd,
           brakeScan ? "FRENA en lectura" : "PWM min en lectura", cmdMin, profBrakes, frontLvl);
  } else
#endif
#if IR_BRAKE_PROFILE
  if(capOn && !(irBrake && profOk)){
    // Sigue a la velocidad de llegada durante la lectura; el avance siguiente sube con la rampa desde aquí.
    motorSetBoth(cmdPrev, cmdPrev);
    s_ramp = cmdPrev;
  }
  if(vEndMMs > 0.0f)
    NAVLOG("[APROX] llegada %.0fmm/s (tope %.0f)%s PWM min %d, freno %u ticks; frontal >=30 a %.1fmm, >=50 a %.1fmm del objetivo\n",
           vNow, vEndMMs, capOn ? "" : " (sin actuar)", cmdMin, capBrakes,
           seen30 < 0 ? -1.0f : (float)(target - seen30) / TICKS_PER_MM, seen50 < 0 ? -1.0f : (float)(target - seen50) / TICKS_PER_MM);
#endif
  if(irBrake) NAVLOG("[FRENO-IR] pared al frente a %.1fmm del final del tramo: PWM %d -> %d (al llegar %d, nivel %d)\n",
                     (float)(stopAt - irBrakePos) / TICKS_PER_MM, basePWM, cmdMin, cmdPrev, frontLvl);
  // Llegó: NO frena. Sigue en modo crucero hasta que algo
  // (giro, alineación, atasco, error o fin del recorrido) llame a
  // motionHaltIfCruising() / navFailStop().
  return MOVE_OK;
}

// Resultado del último giro (solo se VERIFICA; el control angular no cambia).
static bool  g_lastTurnOk     = true;
static bool  g_lastTurnTimedOut = false;
static bool  g_lastTurnUnjam    = false;   // el giro rodante necesitó desatasco
static float g_lastTurnErrDeg = 0.0f;   // error final de rumbo medido por el BNO (NAN si no hay dato)

// ── GIRO POR ENCODERS (NAV_MODE 1 y respaldo de NAV_MODE 3) ──
// Devuelve false si terminó por timeout sin completar los ticks.
static bool _turnEncoder(int ticks, int sL, int sR){
  motionHaltIfCruising();         // un giro siempre parte de velocidad cero
  resetEncoders();
  const uint32_t TIMEOUT=3000;
  uint32_t tStart=millis(), tCtrl=0;

  while(true){
    long l,lA,r,rA; getEncAll(l,lA,r,rA);
    if((lA+rA)/2 >= (long)ticks) break;
    if(millis()-tStart > TIMEOUT){
      motorBrakeAll(); delay(60); motorCoastAll(); delay(60);
      NAVLOG("[GIRO-ENC] TIMEOUT %ld/%d ticks\n", (lA+rA)/2, ticks);
      return false;
    }
#if NAV_UI_IN_TURN
    navUiTick();
#endif
    if(millis()-tCtrl >= 10){ tCtrl=millis(); motorSetBoth(sL,sR); }
  }
  motorBrakeAll(); delay(60); motorCoastAll(); delay(60);
  return true;
}

// ── GIRO CERRADO POR BNO (NAV_MODE 2 y 3) ────────────────────
// dir: +1 derecha (heading sube), -1 izquierda. El objetivo es ABSOLUTO
// (g_navHdgTarget ± 90), así el error de un giro no se acumula en el siguiente.
// Devuelve false si no pudo empezar (BNO sin datos): nada se movió.
// Odometría durante el giro: desplazamiento del EJE (mm) en el marco del rumbo de llegada
// (x = hacia delante, y = hacia la derecha), con encoders con signo + rumbo del BNO.
static float g_toX = 0.0f, g_toY = 0.0f, g_toH0 = 0.0f, g_toTh = 0.0f;
static long  g_toL = 0, g_toR = 0;
static void _turnOdoBegin(float h0){
  long l,lA,r,rA; getEncAll(l,lA,r,rA);
  g_toL = l; g_toR = r; g_toX = g_toY = 0.0f; g_toH0 = h0; g_toTh = 0.0f;
}
static void _turnOdoStep(){
  long l,lA,r,rA; getEncAll(l,lA,r,rA);
  const float d = (float)(g_sgnL * (l - g_toL) + g_sgnR * (r - g_toR)) * 0.5f / TICKS_PER_MM;
  g_toL = l; g_toR = r;
  if(bnoFresh()) g_toTh = wrap180(bnoHeading() - g_toH0) * 0.0174533f;
  g_toX += d * cosf(g_toTh); g_toY += d * sinf(g_toTh);
}

// rolling: el robot llega en marcha (TURN_MODE 1). No frena antes ni espera al BNO (ya se comprobó
// fresco); la inercia hacia delante se mide con _turnOdoStep.
static bool _turnBNO(int dir, int turnPwm, int quarters = 1, bool rolling = false){
  if(!rolling){
    motionHaltIfCruising();       // un giro siempre parte de velocidad cero
    if(!_navSync()) return false;
  }
  const float h0 = g_navHdgTarget;
  g_navHdgTarget = wrap360(g_navHdgTarget + dir*90.0f*quarters);
  if(NAV_USE_ENC) resetEncoders();
  _turnOdoBegin(h0);
  // En rodante las ruedas también cuentan el avance por inercia: margen extra a la validación por encoder.
  const long encMax = (long)((float)ENC_TURN90 * quarters * BNO_TURN_ENC_MAX_FACTOR) + (rolling ? mmToTicks(80.0f) : 0);

  const uint32_t t0 = millis();
  bool usedFallback = false;
  bool timedOut = true;
  int  pwm = turnPwm;                 // sube solo si el giro se atasca (TURN_BOOST_*)
  int  pwmMax = turnPwm;              // máximo usado (fase rápida o refuerzo)
  uint32_t tSlow = 0;                 // desde cuándo gira lento (0 = no)
  while(millis()-t0 < (uint32_t)BNO_TURN_TIMEOUT_MS * quarters){
#if NAV_UI_IN_TURN
    navUiTick();
#endif
    bnoUpdate();
    long l,lA,r,rA; getEncAll(l,lA,r,rA);
    long avg = (lA+rA)/2;
    _turnOdoStep();

    if(bnoFresh()){
      float err  = wrap180(g_navHdgTarget - bnoHeading());   // >0: falta girar a la derecha
      float lead = fabsf(bnoRateDegS()) * BNO_TURN_LEAD_S;   // anticipa la inercia
      if(fabsf(err) <= BNO_TURN_TOL_DEG + lead){ timedOut = false; break; }
      // NAV_MODE 3: validación cruzada con encoders (BNO congelado / patinaje)
      if(NAV_USE_ENC && avg > encMax){
        g_navWarn = "giro: enc>>BNO"; timedOut = false; break;
      }
      // Refuerzo: BNO casi quieto tras la gracia => más PWM, por pasos
      if(TURN_BOOST_MAX_PWM > 0 && millis()-t0 > TURN_STALL_GRACE_MS &&
         fabsf(bnoRateDegS()) < TURN_STALL_RATE_DEGS){
        if(!tSlow) tSlow = millis();
        else if(rolling && ROLL_UNJAM_MM > 0.0f && !g_lastTurnUnjam && millis()-tSlow >= TURN_STALL_MS){
          // Rodante atascado: la cola tocó la pared exterior. Avanzar un poco con el rumbo actual y seguir en el sitio.
          g_lastTurnUnjam = true;
          motorBrakeAll(); delay(40); _turnOdoStep();
          const long a0 = _encAvgAbs(); const uint32_t tu = millis();
          while(_encAvgAbs() - a0 < mmToTicks(ROLL_UNJAM_MM) && millis()-tu < 400){
            motorSetBoth(MOTOR_KICK_MIN + 5, MOTOR_KICK_MIN + 5); delay(3); _turnOdoStep();
          }
          motorBrakeAll(); delay(40); _turnOdoStep();
          NAVLOG("[GIRO-ROD] atascado a %.0f deg del objetivo: avanzo %.1fmm para despegar\n", err, (float)(_encAvgAbs() - a0) / TICKS_PER_MM);
          tSlow = 0;
          continue;
        }
        else if(millis()-tSlow >= TURN_STALL_MS && pwm < TURN_BOOST_MAX_PWM){
          pwm = min(pwm + TURN_BOOST_STEP, TURN_BOOST_MAX_PWM);
          tSlow = millis();
        }
      } else tSlow = 0;
      int s = (err > 0) ? +1 : -1;
      // Fase rápida lejos del objetivo; cerca, el PWM normal (con el freno anticipado por velocidad)
      const int p = (TURN_PWM_FAST > pwm && fabsf(err) > TURN_SLOW_DEG) ? TURN_PWM_FAST : pwm;
      if(p > pwmMax) pwmMax = p;
      motorSetBoth(+s*p, -s*p);
    } else if(NAV_USE_ENC){
      usedFallback = true;                                    // BNO perdido a mitad de giro
      if(avg >= (long)ENC_TURN90 * quarters){ timedOut = false; break; }
      motorSetBoth(+dir*turnPwm, -dir*turnPwm);
    } else {
      navFailStop("BNO perdido (giro)");                      // NAV_MODE 2: no hay respaldo
    }
    delay(2);
  }
  motorBrakeAll(); delay(60); _turnOdoStep(); motorCoastAll(); delay(60); _turnOdoStep();
  g_lastTurnTimedOut = timedOut;
  if(timedOut)      g_navWarn = "giro timeout";
  if(usedFallback)  g_navFallbacks++;

  // Ajuste fino: pulsos cortos hasta quedar dentro de tolerancia (solo si el error pasa BNO_TURN_FINE_ACCEPT_DEG)
  float errMain = NAN;
  uint8_t pulses = 0;
  for(uint8_t i=0; i<BNO_TURN_FINE_TRIES; i++){
    if(!bnoWaitFresh(60)) break;
    float err = wrap180(g_navHdgTarget - bnoHeading());
    if(i == 0){
      errMain = err;
      if(BNO_TURN_FINE_ACCEPT_DEG > 0.0f && fabsf(err) <= BNO_TURN_FINE_ACCEPT_DEG) break;
    }
    if(fabsf(err) <= BNO_TURN_FINE_TOL_DEG) break;
    pulses++;
    int s = (err > 0) ? +1 : -1;
    uint32_t tp = millis();
    while(millis()-tp < BNO_TURN_FINE_PULSE_MS){ motorSetBoth(+s*MOTOR_KICK_MIN, -s*MOTOR_KICK_MIN); delay(2); }
    motorBrakeAll(); delay(50); motorCoastAll(); delay(60); _turnOdoStep();
  }
  // Verificación (no forma parte del control): error final de rumbo.
  g_lastTurnErrDeg = bnoWaitFresh(60) ? wrap180(g_navHdgTarget - bnoHeading()) : NAN;
  _turnOdoStep();
  NAVLOG("[GIRO-BNO] obj=%.0f err=%.1f (al terminar %.1f, %u pulsos) pwm=%d->%d max=%d t=%lums%s%s odo x=%.1f y=%.1fmm%s\n", g_navHdgTarget, g_lastTurnErrDeg,
         errMain, pulses,
         turnPwm, pwm, pwmMax > pwm ? pwmMax : pwm, (unsigned long)(millis()-t0),
         timedOut ? " TIMEOUT" : "", usedFallback ? " (respaldo enc)" : "", g_toX, g_toY, rolling ? " RODANTE" : "");
  return true;
}

// dir: +1 derecha, -1 izquierda
// quarters: 1 = 90°, 2 = 180° en un solo giro.
static void _turn90(int dir, int turnPwm, int quarters = 1){
  g_lastTurnOk = true; g_lastTurnErrDeg = NAN;
  if(NAV_USE_BNO && _turnBNO(dir, turnPwm, quarters)) return;
  if(!NAV_USE_ENC) navFailStop("BNO sin datos");             // NAV_MODE 2
  if(NAV_USE_BNO){                                           // NAV_MODE 3 sin BNO: respaldo
    g_navFallbacks++;
    g_navHdgTarget = wrap360(g_navHdgTarget + dir*90.0f*quarters);   // mantener rumbo nominal
  }
  g_lastTurnOk = _turnEncoder(ENC_TURN90 * quarters, +dir*turnPwm, -dir*turnPwm);   // original: der (+,-), izq (-,+)
}

// Tras alinear con la pared frontal el robot está paralelo a la celda:
// si el BNO discrepa poco, se re-referencia (el IR corrige la deriva del BNO).
static void _navSnapAfterAlign(){
  if(NAV_MODE != 3 || BNO_SNAP_MAX_DEG <= 0.0f) return;
  if(!g_alignSquare) return;          // sin contacto con la pared, solo si el IR confirma que quedó paralelo
  if(!bnoWaitFresh(80)) return;
  float e = wrap180(bnoHeading() - g_navHdgTarget);
  if(fabsf(e) <= BNO_SNAP_MAX_DEG) bnoRebaseTo(g_navHdgTarget);
  else g_navWarn = "snap>limite";
}

// ── FUNCIONES DE FLOODFILL ───────────────────────────────────
// Antes (robot original): empujaba a ciegas 350 ms contra la pared. Ahora se hace sin tocarla:
// se acerca despacio leyendo FL/FR y frena cuando el nivel frontal llega a g_irPivot, que es el
// nivel medido con el eje de las ruedas en el PUNTO DE GIRO (centro de celda + TURN_POINT_OFFSET_MM;
// calibración manual en la página CAL PIVOTE). Mientras se acerca corrige el ángulo con FL-FR.
//
// maxMM: tope de recorrido por ENCODER (lo previsto por el modelo + ALIGN_MAX_EXTRA_MM). Antes no
// había tope: un falso "pared al frente" podía llevar al robot hasta la celda siguiente.
// g_alignMovedMM: lo que realmente recorrió (para no sumar dos posicionamientos).
enum AlignResult : uint8_t { ALIGN_NOCAL = 0, ALIGN_REACHED, ALIGN_LIMIT, ALIGN_TIMEOUT };
static float g_alignMovedMM = 0.0f;
static const char* alignStr(AlignResult a){
  return a == ALIGN_REACHED ? "llego" : a == ALIGN_LIMIT ? "tope-enc" : a == ALIGN_TIMEOUT ? "timeout" : "sin-cal";
}

AlignResult ff_alignToFrontWall(float maxMM){
  motionHaltIfCruising();   // alinear exige partir de velocidad cero
  g_alignSquare = false;
  g_alignMovedMM = 0.0f;
  if(!irPivotCalibrated()) return ALIGN_NOCAL;
  _frameReset();
  const long MAX_TICKS = mmToTicks(maxMM > 0.0f ? maxMM : 0.0f);
  const int ALIGN_PWM  = MOTOR_KICK_MIN + 5;
  const int SLOW_START = (int)(g_irPivot * IR_FRONT_SLOW_FRAC);
  const int SAT_LEVEL  = (int)(g_irPivot * IR_FRONT_SAT_FACTOR);
  const uint32_t TIMEOUT_MS = 1200;   // seguridad
  uint32_t t0 = millis();
  bool sawWall = false;
  AlignResult res = ALIGN_TIMEOUT;
  int fl = 0, fr = 0;

  while(millis()-t0 < TIMEOUT_MS){
    // (sin navUiTick: el frenado depende de la lectura IR; no bloquear el lazo con la OLED)
    if(_encAvgAbs() >= MAX_TICKS){ res = ALIGN_LIMIT; break; }       // no entrar en la celda siguiente
    fl = readDiff_mV_n(PIN_IR_FL, PIN_FT_FL, IR_ALIGN_NSAMPLES);
    fr = readDiff_mV_n(PIN_IR_FR, PIN_FT_FR, IR_ALIGN_NSAMPLES);
    int m = irFrontLevel(fl, fr);
    if(m > 0) sawWall = true;
    if(fl > SAT_LEVEL || fr > SAT_LEVEL){ res = ALIGN_REACHED; break; }   // ya pegado: no confiar, frenar YA
    if(sawWall && m >= g_irPivot){ res = ALIGN_REACHED; break; }          // llegó al punto de giro

    int pwm = ALIGN_PWM;
    if(m >= SLOW_START){
      float frac = (float)(m - SLOW_START) / (float)(g_irPivot - SLOW_START);
      pwm = ALIGN_PWM - (int)((ALIGN_PWM - MOTOR_KICK_MIN) * constrain(frac, 0.0f, 1.0f));
    }
    pwm = ci(pwm, MOTOR_KICK_MIN, ALIGN_PWM);

    float err = (float)(fl - fr) * KP_IR_SQUARE * KP_IR_SQUARE_SIGN;  // FL>FR => más cerca a la izq
    motorL_set((int)((float)pwm - err));
    motorR_set((int)((float)pwm + err));
    delay(5);
  }
  motorBrakeAll(); delay(60); motorCoastAll(); delay(60);
  g_alignMovedMM = (float)_encAvgAbs() / TICKS_PER_MM;
  int avg = (fl + fr) / 2;
  g_alignSquare = (res == ALIGN_REACHED) && avg > 0 && (abs(fl - fr) <= (int)(IR_SQUARE_TOL_FRAC * avg));
  if(res == ALIGN_TIMEOUT) g_navWarn = "align: timeout";
  if(res == ALIGN_LIMIT)   g_navWarn = "align: tope enc";
  NAVLOG("[ALIGN] %s recorrido=%.1fmm (tope %.1f) FL=%d FR=%d pivote=%d escuadra=%d\n",
         alignStr(res), g_alignMovedMM, maxMM, fl, fr, g_irPivot, g_alignSquare ? 1 : 0);
  return res;
}

// Avanza hasta el punto de lectura de la celda siguiente (distancia según navgeom.h).
// Con BNO (NAV_MODE 2/3) las celdas rectas se encadenan sin frenar (g_cruising) y la OLED solo
// muestra la brújula en vivo. Sin BNO (NAV_MODE 1) la OLED muestra "FWD".
void ff_moveForwardOneCell(){
  if(!NAV_USE_BNO) oledShow("FWD","");
  irSnapInvalidate();                      // la lectura anterior ya no corresponde a la pose
#if TURN_MODE == 2
  // Lectura temprana según la velocidad de crucero: lectura (en marcha) + pre-freno + radio máximo tienen que caber
  // antes del centro. A 120 PWM (~750 mm/s) leyendo en -100 el arco empezaba en -50 (radio mínimo) y fallaba.
  {
    const float v = (float)FWD_PWM * ARC_MMS_PER_PWM;
    float P = ARC_R_MAX_MM + v * ARC_SCAN_S + (v > ARC_VIN ? (v - ARC_VIN) * ARC_BRAKE_MM_PER_MMS : 0.0f);
    g_geom.P = cf(P, ARC_READ_BACK_MM, ARC_READ_BACK_MAX_MM);
  }
#endif
  const float d = g_geom.fwdDist();
#if TURN_MODE == 2
  if(d <= 0.0f){
    // Tras un arco el eje ya pasó el punto de lectura temprana de esta celda: se lee donde está (en marcha).
    g_geom.s -= g_geom.C;                  // la referencia pasa al centro de la celda nueva
    g_geom.atPivot = false; g_geom.net = 0;  // (tras un arco FALLIDO quedaba "en el pivote": el giro siguiente era en el
                                             //  sitio a 95 mm del centro y la posición se perdía, log 2026-10-17)
    g_win = WallWindow();
    NAVLOG("[MOV] AVANZA 0: ya %.1fmm pasado de la lectura, s=%.1f en la celda nueva\n", -d, g_geom.s);
    return;
  }
#endif
#if TURN_MODE == 2
  // Explorando, o si FloodFill prevé girar/parar en la celda siguiente: llegar a la lectura temprana a ARC_VREAD como
  // máximo (a 750 mm/s no cabe lectura + frenado + arco). Con mapa y siguiendo recto: sin tope.
  const float vRead = (g_navExploring || g_nextTurnHint) ? ARC_VREAD : 0.0f;
#else
  const float vRead = 0.0f;
#endif
  NAVLOG("[MOV] AVANZA %.1fmm (s=%.1f%s)%s\n", d, g_geom.s, g_geom.atPivot ? " desde pivote" : "",
         vRead > 0.0f ? (g_nextTurnHint ? " [prevé giro]" : " [explorando]") : "");
  MoveResult m = _driveEncoder(d, FWD_PWM, true, g_geom.P, vRead);
  g_geom.arrivedRead();
  if(m == MOVE_SHORT) g_geom.advance(-g_moveShortMM);
}

// Retrocede `mm` medido por encoder (despacio, sin control de rumbo: distancias cortas).
// La posición se conserva con g_backTicks, como en el retroceso por atasco. Devuelve lo retrocedido.
static float g_lastBackupMM = 0.0f;   // >0: acaba de retroceder al pivote (no hace falta otro preshift corto)
static float _backUpMM(float mm){
  const int pwm = MOTOR_KICK_MIN + 5;
  const long a0 = _encAvgAbs(), goal = mmToTicks(mm);
  const uint32_t t0 = millis();
  while(_encAvgAbs() - a0 < goal && millis()-t0 < 800){
    motorL_set(-pwm); motorR_set(-(int)((float)pwm * ENC_TRIM_R)); delay(3);
  }
  motorBrakeAll(); delay(60); motorCoastAll(); delay(40);
  const long d = _encAvgAbs() - a0;
  g_backTicks += d;
  return (float)d / TICKS_PER_MM;
}

#if TURN_MODE == 2
// ── GIRO EN ARCO ─────────────────────────────────────────────
// El robot viene en crucero con el eje en s0 (< 0, antes del centro). El EJE traza: recta (-s0 - r) y un cuarto de
// círculo de radio r que termina sobre el eje del pasillo nuevo. Rueda exterior = base*(1+k), interior = base*(1-k),
// k = (TRACK/2)/r; el BNO corrige el rumbo contra el ideal (recorrido/r) con ARC_KP. Termina en marcha (crucero).
// Si falla (BNO sin datos, timeout o no gira), frena y termina el giro en el sitio.
static void _arcTurn(int dir, float s0){
  // Pre-freno EN RECTA (las dos ruedas igual: no induce giro) hasta ARC_VIN. Log 2026-10-16: entrando a ~400 mm/s, la
  // rueda interior frenada a fondo (PWM 1) patinaba: el arco no giraba y la odometría se quedaba corta ~1 celda
  // => mapa corrido y "meta inalcanzable".
  const float sBrake0 = s0;
  float vIn = -1.0f, vOut = 0.0f;
  {
    const uint32_t tb = millis();
    long pPrev = _posTicks(); uint32_t tPrev = millis();
    delay(10);
    for(;;){
      const long pNow = _posTicks(); const uint32_t tNow = millis();
      vOut = (float)(pNow - pPrev) * 1000.0f / (TICKS_PER_MM * (float)(tNow - tPrev > 0 ? tNow - tPrev : 1));
      if(vIn < 0.0f) vIn = vOut;
      s0 += (float)(pNow - pPrev) / TICKS_PER_MM;
      pPrev = pNow; tPrev = tNow;
      if(vOut <= ARC_VIN || tNow - tb >= ARC_PREBRAKE_MAX_MS) break;
      // PWM bajo (decaimiento lento = freno proporcional) manteniendo el rumbo. El freno a fondo desde ~750 mm/s
      // bloqueaba las ruedas de forma desigual y el robot giraba al revés (log a 120: -17.7° antes del arco).
      bnoUpdate();
      const float u = bnoFresh() ? cf(KP_HDG * wrap180(bnoHeading() - g_navHdgTarget), -10.0f, 10.0f) : 0.0f;
      motorSetBothNoKick(ARC_PREBRAKE_PWM - (int)u, ARC_PREBRAKE_PWM + (int)u);
      delay(8);
    }
  }
  if(-s0 < ARC_R_MIN_MM) s0 = -ARC_R_MIN_MM;   // (el pre-freno recorre ~10-15 mm; con lectura en -100 no llega aquí)
  const float r    = (-s0 > ARC_R_MAX_MM) ? ARC_R_MAX_MM : -s0;
  const float lead = -s0 - r;
  // Diferencia entre ruedas: la geométrica ((vía/2)/r) se queda corta (logs: la interior SIEMPRE en el mínimo y aun así
  // atrasado 9-25°), así que se multiplica por ARC_K_GAIN. Máx 0.9: la interior nunca se para del todo.
  float k = (TRACK_WIDTH_MM * 0.5f) / r * ARC_K_GAIN;
  if(k > 0.9f) k = 0.9f;
  const int   cap  = MOTOR_PWM_CAP < g_dutyMax ? MOTOR_PWM_CAP : g_dutyMax;
  // Centro a ARC_PWM (no a FWD_PWM): con margen bajo el tope, el control puede acelerar la exterior y frenar la
  // interior. A 75 la exterior se saturaba (tope 40-77 de 54-78 ticks) y el arco se atrasaba hasta fallar.
  int base = (int)((float)cap / (1.0f + k));
  if(base > ARC_PWM) base = ARC_PWM;
  const float arcLen = r * 1.5707963f;
  const float maxLen = (lead + arcLen) * ARC_MAX_LEN_FACTOR;
  const float h0 = g_navHdgTarget;
  g_navHdgTarget = wrap360(h0 + dir * 90.0f);
  g_cruising = false;                       // nada de HALT a mitad del arco
  irSnapInvalidate();
  setStatus(ST_TURN);
  const char* nm = dir > 0 ? "DER" : "IZQ";
  _turnOdoBegin(h0);
  const long a0 = _encAvgAbs();
  const uint32_t t0 = millis();
  uint32_t tCtrl = 0, tFresh = millis();
  float th = 0.0f, d = 0.0f, eMax = 0.0f;
  int outMax = 0, inMin = 255, outRaw = 0;
  uint16_t nSat = 0, nCtl = 0;
  const char* fail = nullptr;
  while(true){
    if(g_wdTripped) navFailStop("WD: motores");
    bnoUpdate();
    if(millis() - tCtrl < 5){ delay(1); continue; }
    tCtrl = millis();
    _turnOdoStep();
    d = (float)(_encAvgAbs() - a0) / TICKS_PER_MM;
    if(bnoFresh()){ tFresh = millis(); th = wrap180(bnoHeading() - h0) * (float)dir; }   // grados girados hacia dir
    else if(millis() - tFresh > 60){ fail = "BNO sin datos"; break; }
    if(th >= 90.0f - ARC_END_DEG) break;
    if(d > maxLen){ fail = "no gira"; break; }
    if(millis() - t0 > ARC_TIMEOUT_MS){ fail = "timeout"; break; }
    // Muy atrasado respecto al arco ideal: no va a girar a tiempo. Abortar YA (log 2026-10-16: seguía recto 200 mm,
    // se salía de la celda y la posición quedaba perdida => paredes falsas => meta inalcanzable).
    if(d > lead + 30.0f && (d - lead) / r * 57.29578f - th > ARC_FAIL_LAG_DEG){ fail = "atrasado"; break; }
    // El eje no debe pasar del centro de la celda (en el rumbo de llegada) más de ARC_OVERRUN_MM: más allá la parte
    // delantera llega a la pared del frente (log: arco fallido con x=112 mm => golpe y posición perdida).
    if(s0 + g_toX > ARC_OVERRUN_MM){ fail = "se pasa del centro"; break; }
    int L, R;
    if(d < lead){
      // Tramo recto previo: mantener el rumbo de llegada (misma convención que _hdgCorrection).
      const float u = cf(KP_HDG * th * (float)dir, -(float)(base/3), +(float)(base/3));
      L = base - (int)u; R = base + (int)u;
    } else {
      float ref = (d - lead) / r * 57.29578f;
      if(ref > 90.0f) ref = 90.0f;
      const float e = ref - th;                       // >0: gira de menos respecto al arco ideal
      if(fabsf(e) > eMax) eMax = fabsf(e);
      // Hacia arriba la exterior puede llegar al tope (antes se cortaba en base*1.5 = 100 con el tope en 120).
      const float u = cf(ARC_KP * e, -(float)base * 0.5f, (float)cap);
      int out = (int)((float)base * (1.0f + k) + u + 0.5f);
      int in  = (int)((float)base * (1.0f - k) - u + 0.5f);
      if(out > outRaw) outRaw = out;
      nCtl++;
      // Exterior en el tope: bajar la interior lo mismo (se conserva la DIFERENCIA = la curvatura; baja la velocidad).
      if(out > cap){ in -= out - cap; out = cap; nSat++; }
      // Nunca 0: 0 = rueda LIBRE (coast) y sigue rodando a la velocidad del robot => no gira (log: 3 arcos a la
      // derecha con 12-57° en 200 mm). 1 = decaimiento lento casi todo el ciclo = frena esa rueda.
      if(in < ARC_IN_MIN) in = ARC_IN_MIN;           // sin frenar a fondo la interior (patina)
      if(out > outMax) outMax = out;
      if(in < inMin) inMin = in;
      L = dir > 0 ? out : in;
      R = dir > 0 ? in : out;
    }
    motorSetBothNoKick(L, R);
  }
  const uint32_t tArc = millis() - t0;

  if(fail){
    motorBrakeAll(); delay(60); motorCoastAll(); delay(60); _turnOdoStep();
    const float yA = g_toY, xA = g_toX;
    NAVLOG("[ARCO] %s pre-freno %.0f -> %.0fmm/s (%.1fmm)\n", nm, vIn, vOut, s0 - sBrake0);
    NAVLOG("[ARCO] %s FALLO (%s) r=%.0f desde s0=%.1f tras %.1fmm, %lums, girado %.1f deg, desfase max %.1f, ext max %d (tope %u/%u) int min %d, odo x=%.1f y=%.1f: termino en el sitio\n",
           nm, fail, r, s0, d, (unsigned long)tArc, th, eMax, outMax, nSat, nCtl, inMin, xA, yA);
    g_navWarn = "arco: fallo";
    g_navHdgTarget = h0;                     // _turn90 suma el giro completo: queda en h0 + 90 (absoluto)
    g_geom.setPivot();
    _turn90(dir, pwmForVolts(TURN_VOLTS, readVBAT_filtered()), 1);
    irSnapInvalidate();
    if(!g_lastTurnOk) navFailStop("giro incompleto");
    if(NAV_USE_BNO && !isnan(g_lastTurnErrDeg) && fabsf(g_lastTurnErrDeg) > TURN_FAIL_DEG) navFailStop("giro: error rumbo");
    g_geom.afterTurn(dir);
    g_geom.s = cf((float)dir * yA, -20.0f, r);
    NAVLOG("[ARCO] %s terminado en el sitio: s=%.1f\n", nm, g_geom.s);
    return;
  }

  // Sale recto (en marcha) con el rumbo nuevo: el avance siguiente (KP_HDG) corrige los últimos grados.
  motorSetBoth(base, base);
  const float sOdo = (float)dir * g_toY, lat = s0 + g_toX;
  const float sIdeal = r * (1.0f - cosf(th * 0.0174533f));
  const bool  odoOk = fabsf(sOdo - sIdeal) <= ARC_ODO_MAX_DEV_MM && fabsf(lat) <= ARC_ODO_MAX_DEV_MM;
  g_geom.setPivot(); g_geom.afterTurn(dir);
  g_geom.atPivot = false;                    // sigue rodando: no es un pivote
  g_geom.s = odoOk ? sOdo : sIdeal;
  _frameReset();                             // nuevo marco de encoder para el crucero que sigue
  g_cruising = true;
  NAVLOG("[ARCO] %s pre-freno %.0f -> %.0fmm/s (%.1fmm)\n", nm, vIn, vOut, s0 - sBrake0);
  NAVLOG("[ARCO] %s r=%.0f recta=%.1f desde s0=%.1f: base %d ext max %d (pedido %d, tope %u/%u ticks) int min %d, %lums, %.1fmm, girado %.1f, desfase max %.1f deg\n",
         nm, r, lead, s0, base, outMax, outRaw, nSat, nCtl, inMin, (unsigned long)tArc, d, th, eMax);
  NAVLOG("[ARCO] %s hecho: odo x=%.1f y=%.1f -> s=%.1f (ideal %.1f%s), desvío lateral %.1fmm hacia %s\n",
         nm, g_toX, g_toY, g_geom.s, sIdeal, odoOk ? "" : ", odo DESCARTADA", fabsf(lat),
         (lat >= 0.0f) == (dir > 0) ? "la IZQ" : "la DER");
  if(!odoOk) g_navWarn = "arco: odo descartada";
}
#endif

static void _doTurn(int dir, int quarters = 1){
#if TURN_MODE == 2
  if(quarters == 1 && g_cruising && NAV_USE_ENC && NAV_USE_BNO && !g_geom.atPivot){
    // Posición REAL ahora: el objetivo del último avance + lo que rodó durante la lectura y la decisión.
    const float over = (float)(_posTicks() - mmToTicks(g_cruiseTargetMM)) / TICKS_PER_MM;
    const float sNow = g_geom.s + over;
    bnoUpdate();
    if(-sNow >= ARC_R_MIN_MM && bnoFresh() && !g_irSnap.front){ _arcTurn(dir, sNow); return; }
    NAVLOG("[ARCO] no: s=%.1f (hace falta <= %.0f)%s%s -> giro en el sitio\n", sNow, -ARC_R_MIN_MM,
           bnoFresh() ? "" : ", BNO sin datos", g_irSnap.front ? ", pared al frente" : "");
  }
  // Sin arco (180°, ya cerca del centro, sin BNO): llegar en marcha al punto de lectura de siempre con el frenado
  // por perfil (si hay pared al frente frena hacia el pivote) y seguir con el giro en el sitio de siempre.
  if(NAV_USE_ENC && !g_geom.atPivot){
    const float sLate = g_geom.T - READ_BACK_MM;
    const float d = sLate - g_geom.s;
    if(d > 2.0f){
      NAVLOG("[ARCO] me acerco %.1fmm al punto de lectura de siempre (s=%.1f)\n", d, g_geom.s);
      MoveResult m = _driveEncoder(d, FWD_PWM, true, READ_BACK_MM, CONFIRM_VMAX);
      g_geom.s = sLate;
      if(m == MOVE_SHORT) g_geom.advance(-g_moveShortMM);
    }
  }
#elif TURN_MODE == 1
  // GIRO RODANTE: en marcha, sin pared al frente (lectura de esta celda) y sin frenado IR en curso.
  // Lado exterior del giro: a la derecha (dir=+1) es la pared IZQUIERDA, y al revés.
  const bool outWall = dir > 0 ? g_irSnap.left : g_irSnap.right;
  const int  outIr   = dir > 0 ? g_irSnap.l : g_irSnap.r;
  const int  outCal  = dir > 0 ? g_irCalL : g_irCalR;
  const bool outOk   = !outWall || (outCal > 0 && outIr <= (int)(outCal * ROLL_OUT_MAX_FRAC));
  const bool rollCand = quarters == 1 && g_cruising && NAV_USE_ENC && NAV_USE_BNO && !g_geom.atPivot &&
                        g_irSnap.full && !g_irSnap.front && !g_lastSegFrontBrake;
  if(rollCand && !outOk)
    NAVLOG("[GIRO-ROD] no: pared exterior cerca (IR %d > cal %d x %.2f) -> giro en el sitio\n", outIr, outCal, (float)ROLL_OUT_MAX_FRAC);
  if(rollCand && outOk){
    bnoUpdate();
    if(bnoFresh()){
      // Pre-freno: bajar a ROLL_VMAX antes de girar (menos derrape de la cola hacia fuera).
      const uint32_t tb = millis();
      long pPrev = _posTicks(); uint32_t tPrev = millis();
      float v0 = -1.0f, v = 0.0f;
      delay(10);
      for(;;){
        const long pNow = _posTicks(); const uint32_t tNow = millis();
        v = (float)(pNow - pPrev) * 1000.0f / (TICKS_PER_MM * (float)(tNow - tPrev > 0 ? tNow - tPrev : 1));
        if(v0 < 0.0f) v0 = v;
        pPrev = pNow; tPrev = tNow;
        if(v <= ROLL_VMAX || tNow - tb >= ROLL_PREBRAKE_MAX_MS) break;
        motorBrakeAll(); delay(10);
      }
      const float over = (float)(_posTicks() - mmToTicks(g_cruiseTargetMM)) / TICKS_PER_MM;
      g_geom.advance(over);
      const float s0 = g_geom.s;
      g_cruising = false;                  // el giro absorbe la inercia: no hay HALT
      irSnapInvalidate();
      setStatus(ST_TURN);
      const char* nm = dir > 0 ? "DER" : "IZQ";
      NAVLOG("[GIRO-ROD] %s en marcha desde s=%.1f (%.1fmm pasado del objetivo) v=%.0f -> %.0fmm/s en %lums, exterior IR %d cal %d\n",
             nm, s0, over, v0, v, (unsigned long)(millis() - tb), outIr, outCal);
      g_geom.setPivot();
      g_lastTurnOk = true; g_lastTurnErrDeg = NAN; g_lastTurnUnjam = false;
      _turnBNO(dir, pwmForVolts(TURN_VOLTS, readVBAT_filtered()), 1, true);
      irSnapInvalidate();
      if(NAV_USE_BNO && !isnan(g_lastTurnErrDeg) && fabsf(g_lastTurnErrDeg) > TURN_FAIL_DEG) navFailStop("giro: error rumbo");
      g_geom.afterTurn(dir);
      // Rumbo nuevo = derecha (dir=+1) o izquierda (dir=-1) del de llegada: avance longitudinal = dir*y.
      // Lo que avanzó en el rumbo de llegada (s0 + x) queda como desvío LATERAL (lo corrige el centrado IR).
      g_geom.s = (float)dir * g_toY;
      const float lat = s0 + g_toX;
      // La odometría solo vale si el giro fue limpio (sin timeout, sin desatasco, error pequeño y desplazamiento
      // razonable). Si no, las ruedas patinaron: se asume el pivote en el centro (como el giro en el sitio).
      const bool odoOk = !g_lastTurnTimedOut && !g_lastTurnUnjam && !isnan(g_lastTurnErrDeg) &&
                         fabsf(g_lastTurnErrDeg) <= 5.0f && fabsf(g_geom.s) <= ROLL_WARN_MM * 1.5f && fabsf(g_toX) <= 40.0f;
      if(!odoOk){
        NAVLOG("[GIRO-ROD] odometría DESCARTADA (x=%.1f y=%.1f err=%.1f%s%s): s=0\n", g_toX, g_toY, g_lastTurnErrDeg,
               g_lastTurnTimedOut ? " timeout" : "", g_lastTurnUnjam ? " desatasco" : "");
        g_geom.s = 0.0f;
        g_navWarn = "rodante: odo descartada";
      }
      NAVLOG("[GIRO-ROD] %s hecho: s=%.1f en la celda nueva, desvío lateral %.1fmm hacia %s\n",
             nm, g_geom.s, fabsf(lat), (lat >= 0.0f) == (dir > 0) ? "la IZQ" : "la DER");
      if(fabsf(g_geom.s) > ROLL_WARN_MM || fabsf(lat) > ROLL_WARN_MM){ g_navWarn = "rodante: desvio"; NAVLOG("[GIRO-ROD] AVISO: desvío grande\n"); }
      return;
    }
  }
#endif
  motionHaltIfCruising();   // hay que girar: frenar el crucero recto (mide el sobrepaso)
  irSnapInvalidate();
  if(!NAV_USE_BNO) oledShow(dir>0 ? "TURN R" : "TURN L", "");
  setStatus(ST_TURN);
  const char* name = quarters == 2 ? "180" : (dir > 0 ? "DER" : "IZQ");

  float vbat  = readVBAT_filtered();
  int fwdPwm  = pwmForVolts(FWD_VOLTS,  vbat);
  int turnPwm = pwmForVolts(TURN_VOLTS, vbat);

  if(g_geom.atPivot){
    // 2.º giro seguido (180° = dos de 90°): ya está en el punto de giro. Sin preshift ni
    // alineación (antes cada giro repetía el posicionamiento, contra la pared lateral).
    NAVLOG("[GIRO] %s consecutivo: ya en el punto de giro (s=%.1f)\n", name, g_geom.s);
  } else {
    g_lastBackupMM = 0.0f;
    // Frenó pasado del punto de giro (sobrepaso > preshift): volver antes de girar.
    if(NAV_USE_ENC && PIVOT_BACKUP_MM > 0.0f && g_geom.s - g_geom.T > PIVOT_BACKUP_MM){
      const float back = _backUpMM(g_geom.s - g_geom.T);
      g_geom.advance(-back);
      g_lastBackupMM = back;
      NAVLOG("[PIVOTE] retrocedo %.1fmm -> s=%.1f\n", back, g_geom.s);
    }
    const float need = g_geom.preshiftDist();
    bool placed = false;                 // true: el IR dejó el eje en el punto de giro
    if(NAV_USE_IR && hasWallFront()){
      wallLedsSet(true, false, false);   // pared al frente detectada ahora
      AlignResult a = ff_alignToFrontWall(need + ALIGN_MAX_EXTRA_MM);
      _navSnapAfterAlign();
      if(a == ALIGN_REACHED){
        // El IR manda: el eje está en el punto de giro. Se registra lo que esperaba el encoder.
        NAVLOG("[PRESHIFT] IR: previsto %.1fmm, recorrido %.1fmm (dif %.1f)\n",
               need, g_alignMovedMM, g_alignMovedMM - need);
        g_geom.s = g_geom.T;
        placed = true;
      } else if(a != ALIGN_NOCAL){
        g_geom.advance(g_alignMovedMM);  // se movió sin llegar: el encoder completa lo que falte
      }
    }
    if(!placed){
      const float d = g_geom.preshiftDist();
      const bool drive = d >= PRESHIFT_MIN_MM || (d >= 0.5f && g_lastBackupMM <= 0.0f);
      if(drive){
        MoveResult m = _driveEncoder(d, fwdPwm, false);
        if(m == MOVE_SHORT) g_geom.advance(d - g_moveShortMM);
        else { g_geom.advance(d); motionHaltIfCruising(); }   // frenar de verdad antes de girar
      }
      if(drive || d < 0.5f) NAVLOG("[PRESHIFT] encoder %.1fmm (previsto %.1f) s=%.1f\n", d, need, g_geom.s);
      else NAVLOG("[PRESHIFT] omitido: tras retroceder faltan %.1fmm (< %.0f) s=%.1f\n", d, (float)PRESHIFT_MIN_MM, g_geom.s);
      if(g_geom.s - g_geom.T > 10.0f){ g_navWarn = "pivote pasado"; NAVLOG("[PRESHIFT] AVISO: eje %.1fmm pasado del punto de giro\n", g_geom.s - g_geom.T); }
    }
    g_geom.setPivot();
  }

  motionHaltIfCruising();
  setStatus(ST_TURN);
  _turn90(dir, turnPwm, quarters);
  irSnapInvalidate();
  if(!g_lastTurnOk) navFailStop("giro incompleto");
  if(NAV_USE_BNO && !isnan(g_lastTurnErrDeg) && fabsf(g_lastTurnErrDeg) > TURN_FAIL_DEG) navFailStop("giro: error rumbo");
  for(int q = 0; q < quarters; q++) g_geom.afterTurn(dir);
  NAVLOG("[GIRO] %s hecho, s=%.1f\n", name, g_geom.s);
}

void ff_turnLeft90()  { navLogPose("GIRA IZQ"); _doTurn(-1); }
void ff_turnRight90() { navLogPose("GIRA DER"); _doTurn(+1); }
// Media vuelta en UN giro de 180° (antes: dos de 90° con parada entre medias). Hacia la derecha,
// igual que los dos giros que hacía antes.
void ff_turnAround180(){ navLogPose("GIRA 180"); _doTurn(+1, 2); }

// Desde la pared trasera hasta el punto de lectura de la primera celda.
void ff_initialAdvance(){
  if(!NAV_USE_BNO) oledShow("INICIO","");
  g_cruising = false;
  g_geom.init(CELL_STEP_MM, TURN_MODE == 2 ? ARC_READ_BACK_MM : READ_BACK_MM, TURN_POINT_OFFSET_MM);
  NAVLOG("[GEOM] C=%.1f (adj %.1f) P=%.1f T=%.1f lectura=%.1f preshift=%s ticks/mm=%.3f\n",
         g_geom.C, (float)CELL_STEP_ADJ_MM, g_geom.P, g_geom.T, g_geom.readS(), PRESHIFT_ENABLE ? "ON" : "OFF", TICKS_PER_MM);
  MoveResult m = _driveEncoder(INITIAL_ADVANCE_MM, FWD_PWM, true);
  g_geom.arrivedRead();
  // INITIAL_ADVANCE_MM está calibrado para dejar el eje en la lectura de SIEMPRE (T - READ_BACK_MM), no en la temprana.
  if(TURN_MODE == 2) g_geom.s = g_geom.T - READ_BACK_MM;
  if(m == MOVE_SHORT) g_geom.advance(-g_moveShortMM);
}

// ── PRUEBA DE RECTA (CAL_STRAIGHT_CELLS) ─────────────────────
// Avance inicial + n celdas en crucero (lo mismo que una corrida en pasillo recto), freno y
// medición. Devuelve los mm de encoder recorridos desde el arranque (incluye el deslizamiento
// del frenado). El desplazamiento físico D se mide con regla: pared trasera -> parte trasera del robot.
float ff_calStraight(int n){
  g_odoTicks = 0;
  _frameReset();
  g_odoTicks = 0;
  ff_initialAdvance();
  for(int i = 0; i < n; i++){ ff_moveForwardOneCell(); navUiTick(); }
  motionHaltIfCruising();
  motorCoastAll();
  delay(300);                                  // que se detenga del todo antes de medir
  const float odo  = (float)(g_odoTicks + _posTicks()) / TICKS_PER_MM;
  const float plan = INITIAL_ADVANCE_MM + n * g_geom.C;
  logPrintf("[CAL-RECTA] celdas=%d plan=%.1fmm odo=%.1fmm (deslizamiento final %.1fmm) C=%.1f adj=%.1f\n",
                n, plan, odo, g_lastOvershootMM, g_geom.C, (float)CELL_STEP_ADJ_MM);
  logPrintf("[CAL-RECTA] mide D (pared trasera -> trasera del robot) y L (paso de celda):\n"
                "            CELL_STEP_ADJ_MM = L * %.1f / D - %.1f\n", odo, (float)CELL_FWD_MM);
  return odo;
}

// Regla ampliada de pared frontal (lecturas cerca del pivote): además de la normal, los dos frontales ven algo y uno
// claro, o uno solo muy fuerte. Tras un arco o un giro el robot queda algo corrido/torcido y un frontal ve menos.
static inline bool _frontWallExtra(int fl, int fr){
  if(CONFIRM_ONE_THR <= 0) return false;
  const int mn = fl < fr ? fl : fr, mx = fl > fr ? fl : fr;
  return (mn >= CONFIRM_BOTH_MIN && mx >= CONFIRM_ONE_THR) || (CONFIRM_SOLO_THR > 0 && mx >= CONFIRM_SOLO_THR);
}

// ── LECTURA DE PAREDES PARA FLOODFILL ────────────────────────
// FloodFill::scanWalls() llama SIEMPRE las tres, en orden Frente, Izquierda, Derecha.
// Rellenan la instantánea g_irSnap y los LEDs (azul=F, blanco=L, rojo=R).
// NAV_MODE 2: sin IR no hay detección de paredes (siempre false): solo recorre un mapa NVS.
static uint32_t g_scanT0 = 0;
static long     g_scanPos0 = 0;

// IR_WALL_CONFIRM: solo descarta POSITIVOS que la ventana no respalda (poste/esquina).
static inline bool _confirmSide(bool raw, uint8_t hits){
#if IR_WALL_CONFIRM
  if(raw && g_win.armed && g_win.n >= IR_WALL_WINDOW_MIN && (uint8_t)(hits * 2) < g_win.n) return false;
#endif
  (void)hits;
  return raw;
}

// FloodFill vuelve a escanear la MISMA celda tras recalcular (callejón): el robot no se movió de
// celda, pero en crucero sigue rodando y una 2.ª lectura ya ve la celda siguiente. Se reutiliza
// la primera lectura (se invalida al avanzar o girar).
bool ff_hasWallFront(){
  if(!NAV_USE_IR) return false;
  if(g_irSnap.full){ NAVLOG("[SCAN] misma celda: reutilizo la lectura anterior\n"); return g_irSnap.front; }
  g_scanT0 = millis(); g_scanPos0 = _posTicks();
  bool w;
  if(TURN_MODE == 2 && g_geom.s < g_geom.T - READ_BACK_MM - 5.0f){
    // Lectura temprana: el frontal aún no alcanza la pared de esta celda (solo sirve para [RELOC]). Pocas muestras:
    // la lectura dura ~15 ms menos y a 750 mm/s eso son ~11 mm más para frenar y curvar.
    irFL = readDiff_mV_n(PIN_IR_FL, PIN_FT_FL, IR_CENTER_NSAMPLES);
    irFR = readDiff_mV_n(PIN_IR_FR, PIN_FT_FR, IR_CENTER_NSAMPLES);
    w = wallFrontFromReadings();
  } else {
    w = hasWallFront();
    // Lectura cerca del pivote: misma regla ampliada que la confirmación (log 2026-10-17: arranque frente a la pared
    // del borde con FL=71 FR=127 => F=0).
    if(!w && TURN_MODE == 2 && _frontWallExtra(irFL, irFR)){ w = true; NAVLOG("[SCAN] frontal %d/%d: pared (regla ampliada)\n", irFL, irFR); }
  }
  g_irSnap.fl = irFL; g_irSnap.fr = irFR; g_irSnap.front = w;
  // Una pared vista siempre vale; "sin pared" solo vale si se leyó cerca del pivote (TURN_MODE 2: lectura temprana).
  g_irSnap.frontSure = w || TURN_MODE != 2 || g_geom.s >= g_geom.T - READ_BACK_MM - 5.0f;
#if TURN_MODE == 2
  // Pared frontal FUERTE donde el modelo dice que aún está lejos (s < -60: el frontal no alcanza): el robot está
  // más adelante de lo que cree (p. ej. tras un arco que patinó). Se corrige s con el nivel IR: g_irPivot = s 0,
  // umbral de frenado (~50) = s -45 (logs [FRENO-IR]). Sin esto el error se arrastraba hasta "meta inalcanzable".
  if(w && irPivotCalibrated() && g_geom.s < -60.0f){
    const int lvl = irFrontLevel(irFL, irFR);
    float f = (float)(lvl - IR_BRAKE_START_THR) / (float)(g_irPivot - IR_BRAKE_START_THR);
    f = f < 0.0f ? 0.0f : (f > 1.0f ? 1.0f : f);
    const float sEst = g_geom.T - RELOC_FAR_MM * (1.0f - f);
    NAVLOG("[RELOC] pared frontal nivel %d con s=%.1f: el robot está más adelante -> s=%.1f\n", lvl, g_geom.s, sEst);
    g_navWarn = "reloc frontal";
    g_geom.s = sEst;
  }
#endif
  g_irSnap.valid = true; g_irSnap.ms = millis();
  irSnapShowLeds();
  if(w) setStatus(ST_WALL);
  return w;
}
bool ff_hasWallLeft(){
  if(!NAV_USE_IR) return false;
  if(g_irSnap.full) return g_irSnap.left;
  bool w = _confirmSide(hasWallLeft(), g_win.hitL);
  g_irSnap.l = irL; g_irSnap.left = w; g_irSnap.ms = millis();
  irSnapShowLeds();
  if(w) setStatus(ST_WALL);
  return w;
}
bool ff_hasWallRight(){
  if(!NAV_USE_IR) return false;
  if(g_irSnap.full) return g_irSnap.right;
  bool w = _confirmSide(hasWallRight(), g_win.hitR);
  g_irSnap.r = irR; g_irSnap.right = w; g_irSnap.ms = millis();
  irSnapShowLeds();
  if(w) setStatus(ST_WALL);
  navLogPose("SCAN");
  NAVLOG("[SCAN] FL=%d FR=%d L=%d R=%d -> F=%d%s L=%d R=%d  win n=%u hL=%u hR=%u  dur=%lums %.1fmm s=%.1f\n",
         g_irSnap.fl, g_irSnap.fr, g_irSnap.l, g_irSnap.r,
         g_irSnap.front, g_irSnap.frontSure ? "" : "?", g_irSnap.left, g_irSnap.right, g_win.n, g_win.hitL, g_win.hitR,
         (unsigned long)(millis() - g_scanT0), (float)(_posTicks() - g_scanPos0) / TICKS_PER_MM, g_geom.s);
  g_win.armed = false;
  g_irSnap.full = true;
  return w;
}

// TURN_MODE 2: FloodFill decidió seguir RECTO con la lectura temprana, donde el frontal aún no alcanza la pared
// (F=0? en [SCAN]). Avanza hasta el punto de lectura de siempre (con el frenado por perfil) y lee el frontal.
// false = hay pared: coords sin cambiar, la instantánea queda con F=1 y FloodFill vuelve a decidir (giro en el sitio).
bool ff_frontClearAhead(){
#if TURN_MODE == 2
  if(!NAV_USE_IR) return true;
  if(g_irSnap.frontSure) return !g_irSnap.front;
  const float sLate = g_geom.T - READ_BACK_MM;
  const float d = sLate - g_geom.s;
  if(d > 2.0f){
    NAVLOG("[CONFIRMA] recto: avanzo %.1fmm hasta la lectura frontal (s=%.1f)\n", d, g_geom.s);
    MoveResult m = _driveEncoder(d, FWD_PWM, true, READ_BACK_MM, g_navExploring ? CONFIRM_VMAX : 0.0f);
    g_geom.s = sLate;
    if(m == MOVE_SHORT) g_geom.advance(-g_moveShortMM);
  }
  bool w = hasWallFront();
  // Tras un arco el robot suele quedar 5-15 mm corrido de lado y un frontal ve la pared mucho menos (log 2026-10-16:
  // FL=63/FR=250, 70/142, 70/264 => F=0 y choque). Pared también si los DOS ven algo y uno ve claro. Un frontal solo
  // (156/0, 210/0) es la pared LATERAL vista de reojo: eso no cuenta.
  const bool wOne = !w && _frontWallExtra(irFL, irFR);
  if(wOne) w = true;
  g_irSnap.fl = irFL; g_irSnap.fr = irFR; g_irSnap.front = w; g_irSnap.frontSure = true;
  irSnapShowLeds();
  if(w) setStatus(ST_WALL);
  NAVLOG("[CONFIRMA] FL=%d FR=%d -> F=%d%s s=%.1f%s\n", irFL, irFR, w ? 1 : 0, wOne ? " (un frontal claro)" : "", g_geom.s,
         w ? "  PARED: vuelve a decidir" : "");
  return !w;
#else
  return true;
#endif
}

// Al llegar a un objetivo (meta o regreso al inicio). TURN_MODE 2 entra a la celda en la lectura TEMPRANA (eje 100 mm
// antes del centro): el robot quedaba entre dos celdas. Se acerca despacio al centro leyendo el frontal (si hay pared
// para en el pivote; si no, en el centro por encoder).
void ff_centerInGoal(){
#if TURN_MODE == 2
  motionHaltIfCruising();
  const float need = g_geom.T - g_geom.s;
  if(!NAV_USE_ENC || g_geom.atPivot || need < 5.0f) return;
  AlignResult a = ff_alignToFrontWall(need);
  if(a == ALIGN_REACHED) g_geom.s = g_geom.T;
  else if(a == ALIGN_NOCAL){ MoveResult m = _driveEncoder(need, pwmForVolts(FWD_VOLTS, readVBAT_filtered()), false);
                             motionHaltIfCruising(); g_geom.s += need - (m == MOVE_SHORT ? g_moveShortMM : 0.0f); }
  else g_geom.advance(g_alignMovedMM);
  g_geom.setPivot();
  NAVLOG("[META] al centro de la celda: %s, recorrido %.1fmm, s=%.1f\n", alignStr(a), g_alignMovedMM, g_geom.s);
#endif
}
