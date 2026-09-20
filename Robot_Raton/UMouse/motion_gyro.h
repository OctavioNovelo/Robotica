#pragma once
#include "settings.h"
#include "sensors.h"
#include "motion.h"
#include <Adafruit_BNO08x.h>
#include <math.h>

// =============================================================
//  UMouse — motion_gyro.h
//
//  Funciones de navegación equivalentes a las de motion.h, pero
//  usando el BNO085 (yaw / rotación) en vez de encoders para medir
//  o corregir rumbo durante giros y avance.
//
//  ESTADO: implementado pero NO conectado todavía a floodfill.h
//  ni maus.h — esos archivos siguen usando exclusivamente las
//  funciones ff_* de motion.h (por encoder). Estas versiones _Gyro
//  quedan disponibles para probarse de forma aislada (por ejemplo
//  desde una página de test en el OLED) antes de reemplazar nada
//  en la navegación real del floodfill.
//
//  Portado del sketch de pruebas UMouse_S3_Test_BootMotor:
//   - Se usa SH2_GAME_ROTATION_VECTOR en vez de SH2_ROTATION_VECTOR:
//     el "game rotation vector" fusiona accel+gyro pero NO usa el
//     magnetómetro, así que no se ve afectado por el campo magnético
//     de los motores/encoders cercanos (el rotation vector normal sí).
//   - Se intentan las DOS direcciones I2C típicas del BNO085 (0x4A y
//     0x4B) en vez de asumir una sola.
//   - Se maneja bno08x.wasReset(): si el sensor se reinició solo
//     (brownout, etc), hay que volver a pedir los reportes o se
//     quedan sin datos silenciosamente.
//   - gyroUpdate() drena varios eventos pendientes por llamada (hasta
//     8) en vez de solo uno, para no ir acumulando retraso si el loop
//     tarda en llamarla.
//   - Se agregan reportes de giroscopio calibrado y aceleración lineal,
//     y se calculan pitch/roll además de yaw — solo para diagnóstico
//     (página BNO del OLED); la navegación real solo usa g_yawDeg.
//
//  Convención de yaw usada aquí:
//    - g_yawDeg en grados, rango normalizado a (-180, 180].
//    - Igual que en motion.h: 1=norte, 2=este, 3=sur, 4=oeste, pero
//      el yaw del gyro es relativo (0° = rumbo al momento de
//      gyroInit() / última resincronización), NO absoluto respecto
//      al laberinto. Falta decidir si se resincroniza contra pared
//      frontal (ver ff_alignToFrontWall_Gyro) o contra el rumbo del
//      Maus antes de usarse en floodfill.
//    - Giro IZQUIERDA = yaw decrece. Giro DERECHA = yaw crece.
//      (Ajustar el signo si el montaje físico del BNO da lo
//      contrario al probar sobre el robot real.)
// =============================================================

// La librería recibe el pin de reset y lo maneja ella misma dentro de
// begin_I2C() — así lo prueba el sketch de pruebas. Evita duplicar un
// manejo manual de reset que puede desincronizarse con el de la librería.
Adafruit_BNO08x bno08x(BNO_RST);
sh2_SensorValue_t g_bnoEvent;

bool    g_gyroReady = false;
uint8_t g_bnoAddr   = 0;     // 0x4A o 0x4B, la que respondió — 0 si no hay BNO
uint32_t g_lastBNOms = 0;    // millis() del último evento recibido — usado por gyroIsAlive() y el watchdog

// Última vez que se intentó (re)inicializar el BNO — usado por gyroWatchdog()
// para no reintentar en cada vuelta del loop, solo cada GYRO_RECOVER_COOLDOWN_MS.
static uint32_t g_gyroLastRecoverAttempt = 0;

// Estado del "latido" del LED NeoPixel integrado (ver bnoLedHeartbeat en
// sensors.h): se invierte cada vez que llega un dato nuevo del BNO (ver
// gyroUpdate). El color exacto usado depende también de g_actionInProgress
// (motion.h): verde<->azul en reposo, cian<->blanco en movimiento. Al ser
// un LED dedicado y exclusivo del BNO, no hace falta pausarlo durante una
// acción normal (girar/avanzar) — ya no compite con ledRojo/ledAzul/
// ledBlanco, que ahora conservan siempre su uso original.
static bool g_bnoHeartbeatState = false;

float g_yawDeg    = 0.0f;   // yaw relativo, se actualiza en gyroUpdate()
// Solo para diagnóstico (página BNO) — la navegación no los usa todavía.
float g_pitchDeg  = 0.0f;
float g_rollDeg   = 0.0f;
float g_gyroX = 0.0f, g_gyroY = 0.0f, g_gyroZ = 0.0f;
float g_linAx = 0.0f, g_linAy = 0.0f, g_linAz = 0.0f;

// Normaliza una diferencia angular al rango (-180, 180]
static float _gyroAngleDiff(float target, float current){
  float d = target - current;
  while(d > 180.0f)  d -= 360.0f;
  while(d < -180.0f) d += 360.0f;
  return d;
}

// i=x, j=y, k=z, real=w — convención Adafruit para SH2_GAME_ROTATION_VECTOR
static void _quaternionToEulerDeg(float qr, float qi, float qj, float qk,
                                  float &yawDeg, float &pitchDeg, float &rollDeg){
  float sqi = qi*qi, sqj = qj*qj, sqk = qk*qk;

  float roll = atan2f(2.0f*(qr*qi + qj*qk), 1.0f - 2.0f*(sqi+sqj));

  float t2 = 2.0f*(qr*qj - qk*qi);
  t2 = cf(t2, -1.0f, 1.0f);
  float pitch = asinf(t2);

  float yaw = atan2f(2.0f*(qr*qk + qi*qj), 1.0f - 2.0f*(sqj+sqk));

  yawDeg   = yaw   * 180.0f / (float)M_PI;
  pitchDeg = pitch * 180.0f / (float)M_PI;
  rollDeg  = roll  * 180.0f / (float)M_PI;
}

// Pide al BNO los reportes que necesitamos. Se separa de gyroInit()
// porque también hay que volver a llamarla si wasReset() detecta que
// el sensor se reinició solo en pleno uso.
static bool _bnoSetReports(){
  bool ok = true;
  ok &= bno08x.enableReport(SH2_GAME_ROTATION_VECTOR, BNO_REPORT_INTERVAL_US);
  ok &= bno08x.enableReport(SH2_GYROSCOPE_CALIBRATED,  BNO_REPORT_INTERVAL_US);
  ok &= bno08x.enableReport(SH2_LINEAR_ACCELERATION,   BNO_REPORT_INTERVAL_US);
  return ok;
}

// Espera hasta timeoutMs a que llegue un evento REAL de rotación tras
// haber pedido los reportes con _bnoSetReports(). begin_I2C()/
// enableReport() a veces "tienen éxito" a nivel I2C (sobre todo tras un
// warm reset del ESP32 sin power-cycle del BNO) sin que el chip llegue
// a entregar ningún dato — sin esta espera, se declararía listo un BNO
// que en realidad está "zombie" desde el propio arranque. Procesa
// cualquier evento que llegue mientras espera (igual que gyroUpdate())
// para no perder las primeras muestras, pero solo devuelve true cuando
// ya llegó el SH2_GAME_ROTATION_VECTOR, que es el que usa la navegación.
static bool _waitForFirstBnoEvent(uint32_t timeoutMs){
  uint32_t t0 = millis();
  uint32_t nOtherEvents = 0;

  while(millis() - t0 < timeoutMs){
    // Si el chip se reinició por su cuenta justo aquí (común tras un warm
    // boot del ESP32, o por el pico de corriente del propio reset físico),
    // los reportes que le acabamos de pedir en _bnoSetReports() quedan
    // invalidados — sin este chequeo, nos quedaríamos esperando datos que
    // ya nunca van a llegar. Se re-piden y seguimos esperando en la misma
    // ventana de tiempo.
    if(bno08x.wasReset()){
      Serial.println("BNO: wasReset() durante la espera inicial -> re-pidiendo reportes");
      _bnoSetReports();
    }

    if(bno08x.getSensorEvent(&g_bnoEvent)){
      g_lastBNOms = millis();

      switch(g_bnoEvent.sensorId){
        case SH2_GAME_ROTATION_VECTOR:
          _quaternionToEulerDeg(
            g_bnoEvent.un.gameRotationVector.real,
            g_bnoEvent.un.gameRotationVector.i,
            g_bnoEvent.un.gameRotationVector.j,
            g_bnoEvent.un.gameRotationVector.k,
            g_yawDeg, g_pitchDeg, g_rollDeg
          );
          Serial.printf("BNO: rotation vector OK tras %lums (%lu reportes de otro tipo antes)\n",
                         (unsigned long)(millis()-t0), (unsigned long)nOtherEvents);
          return true; // dato real de orientación recibido — el BNO está vivo de verdad
        case SH2_GYROSCOPE_CALIBRATED:
          g_gyroX = g_bnoEvent.un.gyroscope.x;
          g_gyroY = g_bnoEvent.un.gyroscope.y;
          g_gyroZ = g_bnoEvent.un.gyroscope.z;
          nOtherEvents++;
          break;
        case SH2_LINEAR_ACCELERATION:
          g_linAx = g_bnoEvent.un.linearAcceleration.x;
          g_linAy = g_bnoEvent.un.linearAcceleration.y;
          g_linAz = g_bnoEvent.un.linearAcceleration.z;
          nOtherEvents++;
          break;
        default:
          nOtherEvents++;
          break;
      }
    }
    delay(5);
  }

  // Diagnóstico: si nOtherEvents==0, el BNO no entregó NADA (ni siquiera
  // gyro/accel) — apunta a que se quedó sin reportes activos (wasReset
  // repetido, o el hub SH2 nunca terminó de arrancar). Si nOtherEvents>0
  // pero nunca llegó el de rotación, el problema es más específico de
  // ese reporte (fusión game rotation).
  Serial.printf("BNO: timeout esperando rotation vector (%lu reportes de otro tipo recibidos)\n",
                (unsigned long)nOtherEvents);
  return false; // se acabó el tiempo sin recibir el reporte de rotación
}

// Inicializa el BNO085. Devuelve true si quedó listo para usarse.
// Prueba ambas direcciones I2C típicas (0x4A y luego 0x4B). Cada
// resultado posible enciende un color distinto en el NeoPixel integrado
// (ver enum BnoLedState / bnoLedError en sensors.h), para poder saber
// de un vistazo CUÁL fue el problema, no solo que "algo falló".
bool gyroInit(){
  g_gyroReady = false;
  g_bnoAddr   = 0;
  bnoLedSetState(BNO_LED_OFF);

  if(bno08x.begin_I2C(BNO085_I2C_ADDR_A, &Wire)){
    g_bnoAddr = BNO085_I2C_ADDR_A;
  } else {
    // El intento con la dirección A ya disparó su propio reset físico del
    // BNO085 (begin_I2C lo hace internamente). Si no respondió ahí, hay
    // que darle un respiro antes de probar la otra dirección — probarlas
    // una justo detrás de la otra dejaba al sensor en un estado donde
    // NINGUNA de las dos respondía ("no detectado" en vez de "zombie").
    delay(250);
    if(bno08x.begin_I2C(BNO085_I2C_ADDR_B, &Wire)){
      g_bnoAddr = BNO085_I2C_ADDR_B;
    }
  }

  if(g_bnoAddr == 0){
    oledShow("BNO085","no detectado");
    bnoLedError(BNO_LED_ERR_NOT_FOUND); // rojo — ni 0x4A ni 0x4B respondieron
    return false;
  }

  if(!_bnoSetReports()){
    oledShow("BNO085","sin reporte");
    bnoLedError(BNO_LED_ERR_NO_REPORT); // ámbar — se detectó pero no aceptó configurarse
    return false;
  }

  g_yawDeg = 0.0f;

  // No declaramos g_gyroReady=true todavía: enableReport() puede "tener
  // éxito" por I2C sin que el chip llegue a entregar nunca un dato real
  // (típico tras un reinicio en caliente del ESP32 sin power-cycle del
  // BNO). Esperamos hasta GYRO_INIT_WAIT_MS a la primera muestra real
  // de orientación antes de confiar en el sensor.
  if(!_waitForFirstBnoEvent(GYRO_INIT_WAIT_MS)){
    oledShow("BNO085","sin datos");
    bnoLedError(BNO_LED_ERR_NO_REPORT); // ámbar — aceptó configurarse pero nunca entregó datos reales
    g_gyroReady = false;
    return false;
  }

  g_gyroReady = true; // recién ahora: ya llegó un dato real de orientación
  bnoLedSetState(BNO_LED_READY); // verde — listo
  return true;
}

// true solo si el BNO está listo Y entregó al menos un dato en los
// últimos GYRO_ALIVE_TIMEOUT_MS. Distingue el caso "zombie" (g_gyroReady
// quedó en true pero no llegan datos, típico tras un reinicio en caliente)
// del caso realmente sano. Úsese esto en vez de g_gyroReady a secas antes
// de confiar en el yaw para navegar.
bool gyroIsAlive(){
  return g_gyroReady && (millis() - g_lastBNOms <= GYRO_ALIVE_TIMEOUT_MS);
}

// Vigilante del BNO: se llama periódicamente desde loop() (main.cpp).
//
//  - Si estaba listo pero no llegan datos hace rato, lo declara "zombie":
//    marca g_gyroReady=false y enciende el color magenta (BNO_LED_ERR_ZOMBIE).
//  - La recuperación en caliente SOLO vuelve a pedir los reportes por I2C
//    (_bnoSetReports()) — NUNCA vuelve a llamar gyroInit() en runtime.
//    gyroInit() dispara un reset físico del BNO085 (vía BNO_RST) que en
//    pruebas reinició TODO el ESP32, no solo el sensor — muy probablemente
//    porque el pico de corriente del BNO al resetear hunde el riel de
//    3.3V mientras los motores ya están consumiendo, y eso dispara el
//    brownout del ESP32. Un reset físico en pleno recorrido es peor que
//    el problema que intenta arreglar.
//  - Si el BNO nunca respondió ni una sola vez (g_bnoAddr==0), no hay
//    reportes que re-pedir ni nada seguro que hacer sin un reset físico:
//    se deja así (rojo, BNO_LED_ERR_NOT_FOUND), esperando un power-cycle
//    manual del robot.
void gyroWatchdog(){
  uint32_t now = millis();

  if(g_gyroReady){
    if(now - g_lastBNOms <= GYRO_ALIVE_TIMEOUT_MS) return; // sigue vivo, nada que hacer

    g_gyroReady = false;
    bnoLedError(BNO_LED_ERR_ZOMBIE); // magenta — estaba listo y dejó de entregar datos
    oledShow("BNO zombie","recuperando...");
  }

  if(g_bnoAddr == 0) return; // nunca respondió: nada seguro que reintentar sin reset físico

  if(now - g_gyroLastRecoverAttempt >= GYRO_RECOVER_COOLDOWN_MS){
    g_gyroLastRecoverAttempt = now;
    // Mismo criterio que en gyroInit(): _bnoSetReports() puede "tener
    // éxito" por I2C sin que lleguen datos reales — hay que esperar la
    // primera muestra antes de declarar g_gyroReady=true otra vez, o
    // volveríamos a caer en el mismo bucle verde->magenta.
    if(_bnoSetReports() && _waitForFirstBnoEvent(GYRO_INIT_WAIT_MS)){
      g_gyroReady = true;
      bnoLedSetState(BNO_LED_READY); // recuperado: vuelve a verde
    }
  }
}

// Debe llamarse seguido (loop o dentro de los controles de giro/avance)
// para mantener g_yawDeg (y el resto de datos de diagnóstico) al día.
// Drena hasta 8 eventos pendientes por llamada, para no acumular
// retraso si el loop tarda en volver a llamarla. Si detecta que el
// sensor se reinició solo, vuelve a pedir los reportes. Si detecta que
// dejó de recibir datos del sensor, marca g_gyroReady=false y dispara
// el código de error BNO.
void gyroUpdate(){
  if(!g_gyroReady) return;

  if(bno08x.wasReset()){
    _bnoSetReports();
  }

  bool gotAny = false;

  for(uint8_t i = 0; i < 8; i++){
    if(!bno08x.getSensorEvent(&g_bnoEvent)){
      break; // no hay más eventos pendientes por ahora — no es un error
    }

    g_lastBNOms = millis();
    gotAny = true;

    switch(g_bnoEvent.sensorId){
      case SH2_GAME_ROTATION_VECTOR:
        _quaternionToEulerDeg(
          g_bnoEvent.un.gameRotationVector.real,
          g_bnoEvent.un.gameRotationVector.i,
          g_bnoEvent.un.gameRotationVector.j,
          g_bnoEvent.un.gameRotationVector.k,
          g_yawDeg, g_pitchDeg, g_rollDeg
        );
        break;

      case SH2_GYROSCOPE_CALIBRATED:
        g_gyroX = g_bnoEvent.un.gyroscope.x;
        g_gyroY = g_bnoEvent.un.gyroscope.y;
        g_gyroZ = g_bnoEvent.un.gyroscope.z;
        break;

      case SH2_LINEAR_ACCELERATION:
        g_linAx = g_bnoEvent.un.linearAcceleration.x;
        g_linAy = g_bnoEvent.un.linearAcceleration.y;
        g_linAz = g_bnoEvent.un.linearAcceleration.z;
        break;

      default:
        break;
    }
  }

  // Latido visual: cada vez que llegó algo del BNO en esta llamada,
  // invierte el color del NeoPixel integrado dentro del par que le toca:
  // reposo (verde<->azul), movimiento con gyro (naranja<->púrpura), o
  // movimiento con encoder (cian<->blanco) — ver BnoNavMode/
  // bnoLedHeartbeat en sensors.h. g_actionInProgress dice si hay una
  // acción en curso; g_navUsingGyro (motion.h) dice, dentro de esa
  // acción, si la corrección viene del gyro o cayó al encoder. Ya no
  // toca ledRojo/ledAzul/ledBlanco, que quedan libres para su uso
  // normal (girar/avanzar/atasco) sin importar si el robot está en
  // medio de una acción o no.
  if(gotAny){
    g_bnoHeartbeatState = !g_bnoHeartbeatState;
    BnoNavMode navMode = !g_actionInProgress ? BNO_NAV_IDLE
                        : (g_navUsingGyro ? BNO_NAV_GYRO : BNO_NAV_ENCODER);
    bnoLedHeartbeat(g_bnoHeartbeatState, navMode);
  }
}

// ---------------- Equivalentes a las funciones de motion.h ----------------

// Alinea contra la pared frontal (igual que ff_alignToFrontWall, por
// tiempo fijo) y aprovecha el momento para resincronizar g_yawDeg a 0°,
// ya que se asume que quedar "cuadrado" contra la pared define un
// rumbo de referencia conocido.
// TODO pendiente de decidir: si conviene además usar los sensores IR
// diagonales (FL/FR) para corregir el ángulo de aproximación en vez
// de solo empujar recto un tiempo fijo.
void ff_alignToFrontWall_Gyro(){
  oledShow("ALIGN (gyro)","");
  if(!g_gyroReady){ oledShow("GYRO no listo",""); delay(300); return; }
  g_actionInProgress = true;

  const int ALIGN_PWM = MOTOR_KICK_MIN + 5;
  const uint32_t ALIGN_MS = 350; // mismo valor que la versión por encoder
  uint32_t t0 = millis();
  while(millis()-t0 < ALIGN_MS){
    motorL_set(+ALIGN_PWM);
    motorR_set(+ALIGN_PWM);
    delay(5);
  }
  motorBrakeAll(); delay(60); motorCoastAll(); delay(60);
  g_skipPreshift = true;

  gyroUpdate();
  g_yawDeg = 0.0f; // referencia: "de frente a la pared" = rumbo 0°
  g_actionInProgress = false;
}

// Giro de 90° usando control proporcional sobre el error de yaw.
// Implementación común a izquierda/derecha; el signo del target y
// las ruedas que empujan hacia adelante cambian según turnLeft o no.
static void _gyroTurn90(bool turnLeft, float turnDegSigned, float kp){
  (void)turnLeft; // el signo ya viene resuelto en turnDegSigned; se deja el
                  // parámetro solo por claridad en las funciones que llaman
  g_actionInProgress = true;
  g_navUsingGyro = true; // esta ruta de prueba es 100% gyro, sin fallback
  ledAzul(true);
  if(hasWallFront()) ff_alignToFrontWall_Gyro();

  gyroUpdate();
  float startYaw  = g_yawDeg;
  float targetYaw = startYaw + turnDegSigned; // negativo=izquierda, positivo=derecha

  const uint32_t TIMEOUT = 3000;
  uint32_t t0 = millis();

  while(true){
    gyroUpdate();
    if(!g_gyroReady) break; // gyroWatchdog ya marcó el error correspondiente en el NeoPixel

    // Si el yaw dejó de refrescarse (BNO "zombie"), cortar en vez de
    // seguir girando con un valor congelado — mismo criterio que
    // _turnGyro() en motion.h.
    if(millis() - g_lastBNOms > GYRO_ALIVE_TIMEOUT_MS) break;

    float err = _gyroAngleDiff(targetYaw, g_yawDeg);
    if(fabsf(err) <= GYRO_TURN_TOLERANCE_DEG) break;
    if(millis()-t0 > TIMEOUT) break;

    float u = kp * err;
    u = cf(u, -(float)g_dutyMax, (float)g_dutyMax);

    int pwm = ci((int)fabsf(u), MOTOR_KICK_MIN, g_dutyMax);

    // err<0 → falta girar hacia la izquierda (yaw debe bajar)
    // err>0 → falta girar hacia la derecha  (yaw debe subir)
    if(err < 0){ motorL_set(-pwm); motorR_set(+pwm); }
    else       { motorL_set(+pwm); motorR_set(-pwm); }
  }

  motorBrakeAll(); delay(60); motorCoastAll(); delay(60);
  ledAzul(false);
  g_actionInProgress = false;
}

void ff_turnLeft90_Gyro(){
  oledShow("TURN L (gyro)","");
  if(!g_gyroReady){ oledShow("GYRO no listo",""); delay(300); return; } // el NeoPixel ya muestra el color del último error del BNO (ver gyroInit/gyroWatchdog)
  _gyroTurn90(true, +GYRO_TURN90_DEG_L, GYRO_TURN_KP_L);
}

void ff_turnRight90_Gyro(){
  oledShow("TURN R (gyro)","");
  if(!g_gyroReady){ oledShow("GYRO no listo",""); delay(300); return; } // el NeoPixel ya muestra el color del último error del BNO (ver gyroInit/gyroWatchdog)
  _gyroTurn90(false, -GYRO_TURN90_DEG_R, GYRO_TURN_KP_R);
}

// Avance de una celda: la DISTANCIA se sigue midiendo con encoder
// (el gyro no mide distancia de forma confiable — solo rotación),
// pero el RUMBO se corrige con yaw en vez de con ENC_TRIM_R. Esto es
// útil precisamente cuando el encoder patina, ya que el gyro no se ve
// afectado por eso.
void ff_moveForwardOneCell_Gyro(){
  oledShow("FWD (gyro)","");
  if(!g_gyroReady){ oledShow("GYRO no listo",""); delay(300); return; } // el NeoPixel ya muestra el color del último error del BNO (ver gyroInit/gyroWatchdog)
  g_actionInProgress = true;
  g_navUsingGyro = true; // esta ruta de prueba es 100% gyro, sin fallback
  ledBlanco(true);

  gyroUpdate();
  float targetYaw = g_yawDeg; // mantener el rumbo actual durante el avance

  int ticks;
  if(g_skipPreshift){
    ticks = ENC_CELL_FWD - ENC_PRESHIFT;
    g_skipPreshift = false;
  } else {
    ticks = ENC_CELL_FWD;
  }

  resetEncoders();
  const uint32_t TIMEOUT = 5000;
  uint32_t tStart = millis(), tCtrl = 0;
  bool timedOut = false;

  while(true){
    long l,lA,r,rA;
    getEncAll(l,lA,r,rA);
    long avg = (lA+rA)/2;

    if(avg >= (long)ticks) break;
    if(millis()-tStart > TIMEOUT){ timedOut = true; break; }
    if(!g_gyroReady) break; // gyroWatchdog ya marcó el error correspondiente en el NeoPixel

    if(millis()-tCtrl >= 10){
      tCtrl = millis();
      gyroUpdate();

      // Error de rumbo medido por el gyro, en vez de ENC_TRIM_R
      float err = _gyroAngleDiff(targetYaw, g_yawDeg);
      float u = KP_ENC * err; // punto de partida: reusar ganancia de avance recto
      u = cf(u, -(float)(FWD_PWM/3), (float)(FWD_PWM/3));

      motorSetBoth(FWD_PWM - (int)u, FWD_PWM + (int)u);
    }
  }

  motorBrakeAll(); delay(60); motorCoastAll(); delay(60);
  ledBlanco(false);
  g_actionInProgress = false;

  if(timedOut){
    long l,lA,r,rA; getEncAll(l,lA,r,rA);
    if((lA+rA)/2 < ENC_FAIL_TICKS_THR){
      oledShow("ERROR ENCODER","sin señal de tick");
      ledErrorEncoder();
    }
  }
}