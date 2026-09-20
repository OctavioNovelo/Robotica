#pragma once
#include "settings.h"
#include "sensors.h"

// =============================================================
//  UMouse — motion.h  (ESP32-S3 pinout)
// =============================================================

// Declaraciones externas para evitar dependencia circular
extern bool g_gyroReady;
extern float g_yawDeg;
extern uint32_t g_lastBNOms;
extern void gyroUpdate();
extern bool gyroIsAlive(); // true solo si el BNO está listo Y entregó un dato reciente (no "zombie")

extern int irL, irR, irFL, irFR;
extern void readIR();

// ── PINES MOTORES ────────────────────────────────────────────
#define ML_IN1  6
#define ML_IN2  5
#define MR_IN1  13
#define MR_IN2  14

#define CH_ML_IN1  4
#define CH_ML_IN2  5
#define CH_MR_IN1  6
#define CH_MR_IN2  7

#define ENC_L_A  7    
#define ENC_L_B  15   
#define ENC_R_A  12   
#define ENC_R_B  11   

// ── ENCODERS ─────────────────────────────────────────────────
volatile long encL = 0, encL_abs = 0;
volatile long encR = 0, encR_abs = 0;

bool g_skipPreshift = false;

// true mientras el robot ejecuta una acción normal (girar/avanzar).
// El "latido" del BNO ahora vive en su propio LED NeoPixel integrado
// (ver bnoLedHeartbeat en sensors.h / gyroUpdate en motion_gyro.h), así
// que ya no compite con ledAzul/ledBlanco durante estas acciones. Esta
// bandera se mantiene disponible por si otra lógica futura la necesita.
bool g_actionInProgress = false;

// true si, EN ESTE MOMENTO de una acción en curso, la corrección de rumbo
// se está tomando del gyro; false si cayó al fallback por encoder (o
// NAV_MODE==1). Se actualiza en cada ciclo de control de _driveEncoder()
// y al entrar a _turnGyro()/_turnEncoder(). La usa gyroUpdate() (ver
// motion_gyro.h) junto con g_actionInProgress para elegir el color del
// NeoPixel (ver BnoNavMode / bnoLedHeartbeat en sensors.h).
bool g_navUsingGyro = false;

// Integral del control de rumbo por gyro (KI_GYRO_FWD) — a propósito
// FUERA de _driveEncoder(), para que persista ENTRE celdas en vez de
// reiniciarse en cada avance. Un sesgo físico real (motor más fuerte de
// un lado) tarda en "aprenderse": si se resetea cada ~1s (lo que dura
// una celda), la integral casi no alcanza a acumular nada útil antes de
// volver a cero. Se reinicia solo en eventos reales de referencia:
// cuando se cuadra contra una pared (ff_alignToFrontWall) o tras la
// recuperación de un atasco (_driveEncoder) — momentos en los que de
// todas formas targetYaw se vuelve a fijar desde cero.
float g_gyroFwdI = 0.0f;

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
  ledcWrite(CH_ML_IN1,0); ledcWrite(CH_ML_IN2,0);
  ledcWrite(CH_MR_IN1,0); ledcWrite(CH_MR_IN2,0);
}
void motorBrakeAll(){
  ledcWrite(CH_ML_IN1,255); ledcWrite(CH_ML_IN2,255);
  ledcWrite(CH_MR_IN1,255); ledcWrite(CH_MR_IN2,255);
}

void _modeA(uint8_t chIn1,uint8_t chIn2,uint8_t pwm){
  ledcWrite(chIn1,255); ledcWrite(chIn2,255-constrain(pwm,0,100));
}
void _modeB(uint8_t chIn1,uint8_t chIn2,uint8_t pwm){
  ledcWrite(chIn2,255); ledcWrite(chIn1,255-constrain(pwm,0,100));
}

void _motorSet(uint8_t chIn1,uint8_t chIn2,int speed,bool fwdIsA){
  speed = ci(speed,-g_dutyMax,+g_dutyMax);
  if(speed==0){ ledcWrite(chIn1,0); ledcWrite(chIn2,0); return; }
  int k=ci(MOTOR_KICK_MIN,0,g_dutyMax);
  int s=abs(speed); if(s<k) s=k;
  uint8_t pwm=(uint8_t)s;
  if(speed>0){ if(fwdIsA) _modeA(chIn1,chIn2,pwm); else _modeB(chIn1,chIn2,pwm); }
  else       { if(fwdIsA) _modeB(chIn1,chIn2,pwm); else _modeA(chIn1,chIn2,pwm); }
}

void motorL_set(int s){ _motorSet(CH_ML_IN1,CH_ML_IN2,s,false); }
void motorR_set(int s){ _motorSet(CH_MR_IN1,CH_MR_IN2,s,true);  }
void motorSetBoth(int l,int r){ motorL_set(l); motorR_set(r); }

// ── INIT ─────────────────────────────────────────────────────
void motionInit(){
  ledcSetup(CH_ML_IN1,20000,8); ledcAttachPin(ML_IN1,CH_ML_IN1);
  ledcSetup(CH_ML_IN2,20000,8); ledcAttachPin(ML_IN2,CH_ML_IN2);
  ledcSetup(CH_MR_IN1,20000,8); ledcAttachPin(MR_IN1,CH_MR_IN1);
  ledcSetup(CH_MR_IN2,20000,8); ledcAttachPin(MR_IN2,CH_MR_IN2);
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

// ── AVANCE CON CONTROL DIFERENCIAL (Integración BNO e IR) ─────
static const float MIN_TICKS_PER_MS = 0.20f;
static const long ENC_FAIL_TICKS_THR = 5;

// Decide si el AVANCE debe corregir rumbo con el gyro, según NAV_MODE:
//  - NAV_MODE==1: nunca (solo encoders / ENC_TRIM_R).
//  - NAV_MODE==2: si g_gyroReady, SIN filtrar por "alive" — modo de
//    prueba dedicado al BNO; queremos ver el problema, no esconderlo.
//  - NAV_MODE==3: solo si gyroIsAlive() — modo de competencia, con
//    fallback seguro al control por encoder si el BNO no está sano.
static inline bool _navUseGyroHeading(){
  if (NAV_MODE == 1) return false;
  if (NAV_MODE == 2) return g_gyroReady;
  // NAV_MODE == 3: usa su PROPIO umbral de frescura (GYRO_ALIVE_TIMEOUT_DRIVE_MS
  // en settings.h), no el de gyroIsAlive(). Así el ruido de motores durante
  // el avance no obliga a compartir el mismo umbral ajustado que usan los
  // giros — se puede calibrar cada uno por separado sin recompilar lógica.
  return g_gyroReady && (millis() - g_lastBNOms <= GYRO_ALIVE_TIMEOUT_DRIVE_MS);
}

// Igual que arriba, pero para decidir el camino de un GIRO de 90°
// (_turnGyro vs _turnEncoder). Ver comentarios de _navUseGyroHeading.
static inline bool _navShouldUseGyro(){
  if (NAV_MODE == 1) return false;
  if (NAV_MODE == 2) return true;
  return gyroIsAlive(); // NAV_MODE == 3
}

static void _driveEncoder(int ticks, int basePWM, bool enableSensors = false){
  resetEncoders();
  const uint32_t TIMEOUT = 5000;
  uint32_t tStart  = millis();
  uint32_t tCtrl   = 0;
  uint32_t tVel    = millis();   
  long     velRef  = 0;          
  float    encI    = 0.0f;
  bool     timedOut = false;
  bool     detectorActivo = false;

  // Siempre que enableSensors esté activo, sondear al BNO (mantiene vivo
  // el heartbeat del NeoPixel y g_lastBNOms), sin importar si el resultado
  // se termina usando o no para corregir rumbo — eso lo decide más abajo
  // _navUseGyroHeading() por separado.
  if(enableSensors) gyroUpdate();
  g_navUsingGyro = enableSensors && _navUseGyroHeading();
  float targetYaw = g_yawDeg;
  int currentPWM = basePWM; 

  while(true){
    long l,lA,r,rA;
    getEncAll(l,lA,r,rA);
    long avg = (lA+rA)/2;

    if(avg >= (long)ticks) break;
    if(millis()-tStart > TIMEOUT){ timedOut = true; break; }

    if(!detectorActivo && millis()-tStart > 300) detectorActivo=true;

    // Detector de atasco
    if(detectorActivo && millis()-tVel >= 200){
      long avgNow = (lA+rA)/2;
      float rate  = (float)(avgNow - velRef) / 200.0f;  
      velRef      = avgNow;
      tVel        = millis();

      if(rate < MIN_TICKS_PER_MS){
        oledShow("ATASCO","retrocediendo");
        ledRojo(true);
        motorBrakeAll(); delay(40); motorCoastAll();
        resetEncoders();
        uint32_t t0 = millis();
        while(true){
          long l2,lA2,r2,rA2; getEncAll(l2,lA2,r2,rA2);
          long avg2 = (lA2+rA2)/2;
          if(avg2 >= 75) break;          
          if(millis()-t0 > 1000) break;  
          int revL = -basePWM;
          int revR = -(int)((float)basePWM * ENC_TRIM_R);
          motorL_set(revL);
          motorR_set(revR);
          delay(5);
        }
        g_skipPreshift = true;  
        motorBrakeAll(); delay(60); motorCoastAll();
        ledRojo(false);
        
        if(enableSensors){
          gyroUpdate();
          g_navUsingGyro = _navUseGyroHeading();
          if(g_navUsingGyro) { targetYaw = g_yawDeg; g_gyroFwdI = 0.0f; }
        }
        break;
      }
    }

    if(millis()-tCtrl >= 10){
      tCtrl=millis();
      getEncAll(l,lA,r,rA);

      float u = 0.0f;

      // Arranque suave: durante los primeros FWD_RAMP_TICKS, el PWM sube
      // gradual desde FWD_RAMP_START_PWM hasta basePWM en vez de saltar de
      // golpe (ver comentario en settings.h — evita el serpenteo típico
      // justo al empezar a avanzar después de un giro).
      if(FWD_RAMP_TICKS > 0 && avg < (long)FWD_RAMP_TICKS){
        long span = (long)basePWM - (long)FWD_RAMP_START_PWM;
        currentPWM = FWD_RAMP_START_PWM + (int)((span * avg) / (long)FWD_RAMP_TICKS);
      } else if (currentPWM < basePWM) {
        currentPWM = basePWM;
      }

      // Sondear siempre que haya sensores activos — igual que arriba, esto
      // mantiene el heartbeat y g_lastBNOms al día incluso en los ciclos
      // donde el resultado no se usa para corregir (rama encoder).
      if (enableSensors) gyroUpdate();

      bool useGyroNow = enableSensors && _navUseGyroHeading();
      g_navUsingGyro = useGyroNow; // para el color del NeoPixel (ver gyroUpdate en motion_gyro.h)

      if (useGyroNow) {
          float errYaw = g_yawDeg - targetYaw;
          while(errYaw > 180.0f)  errYaw -= 360.0f;
          while(errYaw < -180.0f) errYaw += 360.0f;
          g_gyroFwdI += KI_GYRO_FWD * errYaw;
          g_gyroFwdI  = cf(g_gyroFwdI, -KI_GYRO_FWD_MAX, +KI_GYRO_FWD_MAX);
          u = KP_GYRO_FWD * errYaw + g_gyroFwdI;
      } else {
          float err = (float)lA - (float)rA * ENC_TRIM_R;
          if(fabsf(err)<3.0f) err=0.0f;
          encI += KI_ENC * err;
          encI  = cf(encI,-KI_ENC_MAX,+KI_ENC_MAX);
          u = KP_ENC*err + encI;
      }

      if (enableSensors) {
          readIR();
          bool wallLeft = (irL >= IR_WALL_THR_L);
          bool wallRight = (irR >= IR_WALL_THR_R);

          if (wallLeft && wallRight) {
              float errCenter = (float)(irL - IR_TARGET_L) - (float)(irR - IR_TARGET_R);
              u -= KP_CENTER * errCenter;
          } else if (wallLeft) {
              float errCenter = (float)(irL - IR_TARGET_L);
              u -= KP_CENTER * errCenter * 0.5f;
          } else if (wallRight) {
              float errCenter = -(float)(irR - IR_TARGET_R);
              u -= KP_CENTER * errCenter * 0.5f;
          }

          if (irFL > IR_BRAKE_THR || irFR > IR_BRAKE_THR) {
              currentPWM -= 1; 
              if(currentPWM < PWM_MIN_BRAKE) currentPWM = PWM_MIN_BRAKE;
          } else {
              if(currentPWM < basePWM) currentPWM += 1;
          }
      }

      // ANTES este límite era asimétrico (-currentPWM/1.5 vs +currentPWM/3):
      // el robot podía corregir mucho más fuerte hacia un lado que hacia el
      // otro, lo que por sí solo puede causar oscilación (corrige de más
      // para un lado, de menos para el otro, nunca se centra). Ahora usa
      // ENC_CORR_MAX_FRAC (settings.h), simétrico y con un solo número para
      // calibrar qué tan agresiva puede ser la corrección.
      float corrMax = (float)currentPWM * ENC_CORR_MAX_FRAC;
      u = cf(u, -corrMax, +corrMax);
      motorSetBoth(currentPWM - (int)u, (int)((float)currentPWM * ENC_TRIM_R) + (int)u);
    }
  }
  motorBrakeAll(); delay(60); motorCoastAll(); delay(60);

  if(timedOut){
    long l,lA,r,rA; getEncAll(l,lA,r,rA);
    long avgFinal = (lA+rA)/2;
    if(avgFinal < ENC_FAIL_TICKS_THR){ oledShow("ERROR ENCODER","sin señal"); ledErrorEncoder(); }
  }
}

// ── GIRO POR GYRO (PRECISIÓN ABSOLUTA) ───────────────────────
static void _turnGyro(float angleOffset, int maxPwm, float kp) {
  g_navUsingGyro = true; // esta función solo se llama cuando ya se decidió usar gyro
  gyroUpdate();
  float targetYaw = g_yawDeg + angleOffset;
  
  uint32_t tStart = millis();
  uint32_t tCtrl = 0;
  
  while(true) {
    if(millis() - tStart > 3000) break; // Timeout de 3 seg

    // Si el BNO dejó de entregar datos frescos (p.ej. quedó "zombie" tras
    // un reinicio), no seguir girando con un yaw congelado — cortar aquí.
    // La próxima vez, gyroIsAlive() ya dirá false y se usará el fallback
    // por encoder desde el inicio.
    if(millis() - g_lastBNOms > GYRO_ALIVE_TIMEOUT_MS) break;

    if(millis() - tCtrl >= GYRO_CTRL_INTERVAL_MS) {
      tCtrl = millis();
      gyroUpdate();
      
      // Error = Meta - Actual
      float errYaw = targetYaw - g_yawDeg;
      
      // Normalizar error entre -180 y 180
      while(errYaw > 180.0f)  errYaw -= 360.0f;
      while(errYaw < -180.0f) errYaw += 360.0f;
      
      // Si el error es menor a la tolerancia, ¡llegamos!
      if (fabs(errYaw) <= GYRO_TURN_TOLERANCE_DEG) {
          break; 
      }
      
      // Control Proporcional (KP): a menor error, menor velocidad
      float u = kp * errYaw; 
      
      int pwr = (int)fabs(u);
      if (pwr > maxPwm) pwr = maxPwm;
      // MOTOR_KICK_MIN asegura que los motores no se queden zumbando sin fuerza
      if (pwr < MOTOR_KICK_MIN) pwr = MOTOR_KICK_MIN; 
      
      // Si u > 0, necesitamos girar a la izquierda (Motor Izq atrás, Motor Der adelante)
      // Si u < 0, necesitamos girar a la derecha (Motor Izq adelante, Motor Der atrás)
      if (u > 0.0f) {
          motorSetBoth(-pwr, pwr);
      } else {
          motorSetBoth(pwr, -pwr);
      }
    }
  }
  motorBrakeAll(); delay(60); motorCoastAll(); delay(60);
}

// ── GIRO CLÁSICO (FALLBACK POR SI FALLA EL GYRO) ─────────────
static void _turnEncoder(int ticks, int sL, int sR){
  g_navUsingGyro = false; // esta función solo se llama cuando se cayó al fallback
  // No hay llamadas a gyroUpdate() aquí adentro (no aplica, es 100% encoder),
  // así que el heartbeat automático no se dispararía solo — fijamos el color
  // cian una vez, de entrada, para tener feedback visual inmediato igual.
  bnoLedSetState(BNO_LED_READY_MOVING_ENCODER);
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
  const uint32_t ALIGN_MS = 350; 
  uint32_t t0 = millis();
  while(millis()-t0 < ALIGN_MS){
    motorL_set(+ALIGN_PWM);
    motorR_set(+ALIGN_PWM);
    delay(5);
  }
  motorBrakeAll(); delay(60); motorCoastAll(); delay(60);
  g_skipPreshift = true;

  // Resincronizar el yaw: quedar cuadrado contra una pared es una
  // referencia de rumbo CONOCIDA, así que aprovechamos para poner
  // g_yawDeg en 0° aquí. Sin esto, _turnGyro() calcula cada giro como
  // "yaw actual + offset" (relativo, sin referencia absoluta), y CUALQUIER
  // error pequeño de un giro se ARRASTRA y se SUMA al siguiente giro —
  // el objetivo de calibración se movería solo con cada celda recorrida.
  // Antes esto SOLO lo hacía ff_alignToFrontWall_Gyro() (motion_gyro.h),
  // que todavía no está conectada al floodfill real — este era el punto
  // que de verdad importa para que la calibración con BNO sea consistente
  // giro tras giro.
  if(NAV_USE_GYRO && g_gyroReady){
    gyroUpdate();
    g_yawDeg = 0.0f;
    g_gyroFwdI = 0.0f;
  }
}

void ff_moveForwardOneCell(){
  oledShow("FWD","");
  g_actionInProgress = true;
  ledBlanco(true);
  
  if(g_skipPreshift){
    _driveEncoder(ENC_CELL_FWD - ENC_PRESHIFT, FWD_PWM, true);
    g_skipPreshift = false;
  } else {
    _driveEncoder(ENC_CELL_FWD, FWD_PWM, true);
  }
  
  ledBlanco(false);
  g_actionInProgress = false;
}

void ff_turnLeft90(){
  oledShow("TURN L","");
  g_actionInProgress = true;
  ledAzul(true);
  if(hasWallFront()) ff_alignToFrontWall();

  float vbat  = readVBAT_filtered();
  int fwdPwm  = pwmForVolts(FWD_VOLTS, vbat);
  int turnPwm = pwmForVolts(TURN_VOLTS_L, vbat);
  
  _driveEncoder(ENC_PRESHIFT, fwdPwm, false);
  
  if (_navShouldUseGyro()) {
      // Izquierda suma grados al yaw
      _turnGyro(GYRO_TURN90_DEG_L, turnPwm, GYRO_TURN_KP_L);
  } else {
      // Fallback seguro si el sensor falló o está "zombie" (o NAV_MODE==1)
      int sL = -turnPwm;
      int sR = +(int)((float)turnPwm * TURN_TRIM_L);
      _turnEncoder(ENC_TURN90_L, sL, sR);
  }
  
  ledAzul(false);
  g_actionInProgress = false;
}

void ff_turnRight90(){
  oledShow("TURN R","");
  g_actionInProgress = true;
  ledAzul(true);
  if(hasWallFront()) ff_alignToFrontWall();

  float vbat  = readVBAT_filtered();
  int fwdPwm  = pwmForVolts(FWD_VOLTS, vbat);
  int turnPwm = pwmForVolts(TURN_VOLTS_R, vbat);
  
  _driveEncoder(ENC_PRESHIFT, fwdPwm, false);
  
  if (_navShouldUseGyro()) {
      // Derecha resta grados al yaw
      _turnGyro(-GYRO_TURN90_DEG_R, turnPwm, GYRO_TURN_KP_R);
  } else {
      // Fallback seguro si el sensor falló o está "zombie" (o NAV_MODE==1)
      //
      // BUG CORREGIDO: antes esto tenía sR=+turnPwm y sL=-turnPwm*TRIM_R,
      // que es EXACTAMENTE el mismo movimiento que ff_turnLeft90 (izquierda
      // atrás, derecha adelante) — es decir, el giro "derecho" por encoder
      // en realidad giraba hacia la IZQUIERDA. Debe ser el espejo: derecha
      // atrás, izquierda adelante.
      int sR = -turnPwm;
      int sL = +(int)((float)turnPwm * TURN_TRIM_R);
      _turnEncoder(ENC_TURN90_R, sL, sR);
  }
  
  ledAzul(false);
  g_actionInProgress = false;
}

void ff_initialAdvance(){
  oledShow("INICIO","");
  g_actionInProgress = true;
  ledBlanco(true);
  _driveEncoder(ENC_INITIAL_ADVANCE, FWD_PWM, true);
  ledBlanco(false);
  g_actionInProgress = false;
}

bool ff_hasWallFront() { return hasWallFront(); }
bool ff_hasWallLeft()  { return hasWallLeft();  }
bool ff_hasWallRight() { return hasWallRight(); }