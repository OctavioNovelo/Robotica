#pragma once
#include <Wire.h>
#include <Adafruit_SSD1306.h>
#include "settings.h"

// =============================================================
//  UMouse — sensors.h
//  ESP32-S3 — IR (4 sensores: FL, L, R, FR), VBAT, OLED, LEDs debug
// =============================================================

// ── PINES IR (emisores IR383) ───────────────────────────────
#define LED_FL   17
#define LED_L    39
#define LED_R    40
#define LED_FR   9

// ── PINES IR (fototransistores PT1302) ───────────────────────
#define FT_FL    8
#define FT_L     1
#define FT_R     2
#define FT_FR    10

// ── VBAT ─────────────────────────────────────────────────────
#define VBAT_PIN  4   // ADC_BATT — divisor resistivo (ver VBAT_DIV_RATIO en settings.h)

// ── LEDC IR ──────────────────────────────────────────────────
#define CH_IR_FL  0
#define CH_IR_L   1
#define CH_IR_R   2
#define CH_IR_FR  3
static const int IR_NSAMPLES = 12;
static const int IR_ON_US    = 250;
static const int IR_OFF_US   = 250;

// ── OLED ─────────────────────────────────────────────────────
// El esquema nuevo no trae pines I2C dedicados al OLED, así que
// comparte el mismo bus que el BNO085 (SDA/SCL abajo), con su
// propia dirección (0x3C).
#define OLED_W     128
#define OLED_H     64
#define OLED_ADDR  0x3C
#define I2C_SDA    41
#define I2C_SCL    42

extern Adafruit_SSD1306 display;

// ── BNO085 (gyro/accel) ──────────────────────────────────────
// Pines definidos por hardware. El driver vive en motion_gyro.h
// (gyroInit/gyroUpdate); aquí solo quedan las constantes de pines
// y dirección I2C.
#define BNO_SDA   41   // compartido con I2C_SDA (OLED)
#define BNO_SCL   42   // compartido con I2C_SCL (OLED)
#define BNO_INT   18
#define BNO_RST   38
// El sketch de pruebas prueba ambas direcciones típicas del BNO085/BNO08x
// por I2C; algunos breakouts vienen en 0x4A y otros en 0x4B.
#define BNO085_I2C_ADDR_A  0x4A
#define BNO085_I2C_ADDR_B  0x4B

// ── LEDs DEBUG ───────────────────────────────────────────────
// Estos 3 LEDs son de uso normal del robot (girar/avanzar/atasco, ver
// motion.h y floodfill.h) — YA NO se usan para indicar el BNO085.
#define LED_ROJO     21   // RLED
#define LED_AZUL     47   // BLED
#define LED_BLANCO   16   // WLED

inline void ledRojo(bool on)   { digitalWrite(LED_ROJO,   on ? HIGH : LOW); }
inline void ledAzul(bool on)   { digitalWrite(LED_AZUL,   on ? HIGH : LOW); }
inline void ledBlanco(bool on) { digitalWrite(LED_BLANCO, on ? HIGH : LOW); }

#define BNO_LED_PIN    48   // NeoPixel/WS2812 integrado en la placa ESP32-S3

// ── LED INTEGRADO (NeoPixel) — TODO lo del BNO085 se maneja aquí ────
// Esta placa ESP32-S3 trae un LED RGB direccionable (WS2812/NeoPixel)
// integrado en el pin 48. Se controla con neopixelWrite(pin,r,g,b),
// incluida en el core Arduino-ESP32 3.x — no hace falta librería ni
// pinMode() previo, la función configura el canal RMT sola en la
// primera llamada.
//
// Este LED es el ÚNICO indicador del BNO085 — estado normal Y errores.
// Cada situación tiene su propio color para poder diagnosticar a
// simple vista, sin depender del OLED ni de los 3 LEDs de debug:
//
//   Apagado   → BNO no inicializado todavía (arrancando).
//   Verde     → listo, en REPOSO (sin dato nuevo aún).
//   Azul      → "latido" en REPOSO: llegó un dato nuevo (alterna con Verde).
//   Naranja   → en MOVIMIENTO corrigiendo con GYRO, sin dato nuevo aún.
//   Púrpura   → "latido" en MOVIMIENTO CON GYRO: llegó un dato nuevo (alterna con Naranja).
//   Cian      → en MOVIMIENTO corrigiendo con ENCODER (fallback / NAV_MODE==1),
//               sin dato nuevo aún.
//   Blanco    → "latido" en MOVIMIENTO CON ENCODER: llegó un dato nuevo del
//               BNO en ese instante, aunque no se esté usando para corregir
//               (alterna con Cian).
//               (Tres pares de colores, cada uno con su propio significado:
//               reposo vs. movimiento, Y — mientras se mueve — si la
//               corrección de rumbo en ESE momento viene del gyro o cayó al
//               encoder. Todo con más brillo que reposo — ver
//               BNO_LED_BRIGHT_MOVING — para que se note con el robot en
//               marcha bajo luz de campo. Útil para diagnosticar de un
//               vistazo si el BNO se "congela"/cae a encoder específicamente
//               al avanzar o girar.)
//   Rojo      → ERROR: no se detectó el BNO en ninguna dirección I2C
//               (ni 0x4A ni 0x4B) — probable cable/soldadura suelta.
//   Ámbar     → ERROR: el BNO respondió al I2C pero enableReport()
//               falló — se detectó pero no acepta configurarse.
//   Magenta   → ERROR: "zombie" — estaba listo y dejó de entregar
//               datos nuevos (típico tras un reinicio en caliente).
//
// Ante un error, el color parpadea unas veces para llamar la atención
// en el momento exacto en que se detecta, y luego se queda FIJO en ese
// mismo color — así el NeoPixel funciona como un "último error"
// persistente hasta que gyroInit() o la recuperación de
// gyroWatchdog() lo resuelvan y vuelva a verde.
enum BnoLedState : uint8_t {
  BNO_LED_OFF = 0,                // no inicializado / arrancando
  BNO_LED_READY,                   // listo, en reposo
  BNO_LED_HEARTBEAT,               // listo, en reposo, llegó un dato nuevo
  BNO_LED_READY_MOVING_GYRO,       // en movimiento, corrigiendo con gyro
  BNO_LED_HEARTBEAT_MOVING_GYRO,   // en movimiento con gyro, llegó un dato nuevo
  BNO_LED_READY_MOVING_ENCODER,    // en movimiento, corrigiendo con encoder (fallback)
  BNO_LED_HEARTBEAT_MOVING_ENCODER,// en movimiento con encoder, llegó un dato nuevo del BNO
  BNO_LED_ERR_NOT_FOUND,           // no detectado en 0x4A ni 0x4B
  BNO_LED_ERR_NO_REPORT,           // detectado, pero enableReport() falló
  BNO_LED_ERR_ZOMBIE,              // dejó de entregar datos en pleno uso
};

// Con qué fuente se está corrigiendo el rumbo AHORA MISMO durante una
// acción — lo decide motion.h (_navUseGyroHeading/_navShouldUseGyro) y lo
// usa gyroUpdate() (motion_gyro.h) para elegir el par de colores. BNO_NAV_IDLE
// = robot quieto, no aplica (se ignora, siempre se ve reposo verde/azul).
enum BnoNavMode : uint8_t { BNO_NAV_IDLE = 0, BNO_NAV_GYRO, BNO_NAV_ENCODER };

static const uint8_t BNO_LED_BRIGHT         = 25; // brillo bajo a propósito, no encandilar en campo (reposo/errores)
static const uint8_t BNO_LED_BRIGHT_MOVING  = 140; // mucho más brillante — necesita notarse con el robot en marcha

inline void bnoLedSetState(BnoLedState state){
  switch(state){
    case BNO_LED_READY:               neopixelWrite(BNO_LED_PIN, 0, BNO_LED_BRIGHT, 0); break;                          // verde
    case BNO_LED_HEARTBEAT:           neopixelWrite(BNO_LED_PIN, 0, 0, BNO_LED_BRIGHT); break;                          // azul
    case BNO_LED_READY_MOVING_GYRO:   neopixelWrite(BNO_LED_PIN, BNO_LED_BRIGHT_MOVING, BNO_LED_BRIGHT_MOVING/3, 0); break;   // naranja intenso
    case BNO_LED_HEARTBEAT_MOVING_GYRO: neopixelWrite(BNO_LED_PIN, BNO_LED_BRIGHT_MOVING/2, 0, BNO_LED_BRIGHT_MOVING); break; // púrpura intenso
    case BNO_LED_READY_MOVING_ENCODER:  neopixelWrite(BNO_LED_PIN, 0, BNO_LED_BRIGHT_MOVING, BNO_LED_BRIGHT_MOVING); break;   // cian intenso
    case BNO_LED_HEARTBEAT_MOVING_ENCODER: neopixelWrite(BNO_LED_PIN, BNO_LED_BRIGHT_MOVING, BNO_LED_BRIGHT_MOVING, BNO_LED_BRIGHT_MOVING); break; // blanco intenso
    case BNO_LED_ERR_NOT_FOUND:       neopixelWrite(BNO_LED_PIN, BNO_LED_BRIGHT, 0, 0); break;                          // rojo
    case BNO_LED_ERR_NO_REPORT:       neopixelWrite(BNO_LED_PIN, BNO_LED_BRIGHT, BNO_LED_BRIGHT/2, 0); break;           // ámbar
    case BNO_LED_ERR_ZOMBIE:          neopixelWrite(BNO_LED_PIN, BNO_LED_BRIGHT, 0, BNO_LED_BRIGHT); break;             // magenta
    case BNO_LED_OFF:
    default:                          neopixelWrite(BNO_LED_PIN, 0, 0, 0); break;                                      // apagado
  }
}

// Alterna entre tres pares de colores según reposo/movimiento y, en
// movimiento, según de dónde viene la corrección de rumbo AHORA (mode):
//   BNO_NAV_IDLE    → Verde   <-> Azul     (reposo, brillo bajo)
//   BNO_NAV_GYRO    → Naranja <-> Púrpura  (movimiento con gyro, brillo alto)
//   BNO_NAV_ENCODER → Cian    <-> Blanco   (movimiento con encoder, brillo alto)
// gotNewData = true → acaba de llegar un dato (color de "latido" de ese par).
inline void bnoLedHeartbeat(bool gotNewData, BnoNavMode mode = BNO_NAV_IDLE){
  switch(mode){
    case BNO_NAV_GYRO:
      bnoLedSetState(gotNewData ? BNO_LED_HEARTBEAT_MOVING_GYRO : BNO_LED_READY_MOVING_GYRO);
      break;
    case BNO_NAV_ENCODER:
      bnoLedSetState(gotNewData ? BNO_LED_HEARTBEAT_MOVING_ENCODER : BNO_LED_READY_MOVING_ENCODER);
      break;
    case BNO_NAV_IDLE:
    default:
      bnoLedSetState(gotNewData ? BNO_LED_HEARTBEAT : BNO_LED_READY);
      break;
  }
}

// Parpadea el color de un error unas cuantas veces (para llamar la
// atención justo cuando se detecta) y lo deja FIJO en ese color al
// terminar, como indicador persistente del último error hasta que se
// resuelva. Usar SIEMPRE que se detecte un error específico del BNO.
inline void bnoLedError(BnoLedState errState, uint8_t blinks = 4){
  for(uint8_t i = 0; i < blinks; i++){
    bnoLedSetState(errState); delay(120);
    bnoLedSetState(BNO_LED_OFF); delay(120);
  }
  bnoLedSetState(errState);
}

// Parpadeo bloqueante simple para señalizar eventos puntuales
// (dead end, atasco, fin de recorrido, etc). Ver LED_BLINK_MS en settings.h.
inline void ledBlink(void (*ledFn)(bool), int times){
  for(int i = 0; i < times; i++){
    ledFn(true);  delay(LED_BLINK_MS);
    ledFn(false); delay(LED_BLINK_MS);
  }
}

// ── CÓDIGO DE ERROR POR PARPADEO (ENCODERS) ──────────────────
// El BNO085 YA NO usa este mecanismo — todos sus errores se muestran
// con color propio en el NeoPixel integrado (ver bnoLedError arriba).
// Este parpadeo queda exclusivamente para el encoder, sin necesitar
// el OLED: LED_ROJO y LED_AZUL alternando, 6 veces rápido — indica que
// un encoder no está registrando ticks (cable suelto o rueda
// desacoplada). Bloqueante (delay), de uso puntual, igual que ledBlink().
inline void ledErrorEncoder(){
  for(int i = 0; i < 6; i++){
    ledRojo(true);  ledAzul(false); delay(80);
    ledRojo(false); ledAzul(true);  delay(80);
  }
  ledAzul(false);
}

// ── ESTADO IR ────────────────────────────────────────────────
// Valor usado para comparar contra los umbrales IR_WALL_THR_* (respeta
// IR_USE_ABS_DIFF). Se mantiene el nombre irFL/irL/irR/irFR para no
// romper el resto del código (floodfill, motion.h, etc).
int irFL = 0, irL = 0, irR = 0, irFR = 0;

// Valores crudos OFF/ON y diferencia con signo, solo para diagnóstico
// (página IR del OLED) — portados del sketch de pruebas.
int irFL_off = 0, irFL_on = 0, irFL_signed = 0;
int irL_off  = 0, irL_on  = 0, irL_signed  = 0;
int irR_off  = 0, irR_on  = 0, irR_signed  = 0;
int irFR_off = 0, irFR_on = 0, irFR_signed = 0;

// ── VBAT ─────────────────────────────────────────────────────
inline float readVBAT_V(){
  return (analogReadMilliVolts(VBAT_PIN) * VBAT_DIV_RATIO) / 1000.0f;
}

// Techo duro de PWM (0-255), independiente de si la lectura de VBAT
// salió bien o mal. MOTOR_ALLOW_OVERDRIVE=true lo quita por completo.
inline int motorHardDutyLimit(){
  return MOTOR_ALLOW_OVERDRIVE
           ? 255
           : (int)(255.0f * MOTOR_HARD_DUTY_FRACTION + 0.5f);
}

inline int calcDutyMax(float vbat){
  int hardLimit = motorHardDutyLimit();

  // Si el ADC de batería falla (o no está conectado), usar un límite
  // conservador en vez de asumir que hay batería llena.
  if(vbat < 1.0f) return constrain(MOTOR_SAFE_FALLBACK_MAX, MOTOR_KICK_MIN, hardLimit);

  float d = 255.0f * (V_MOTOR_LIMIT / vbat);
  int byVoltage = (int)constrain(d, 0.0f, 255.0f);
  int limited = min(byVoltage, hardLimit);
  return constrain(limited, MOTOR_KICK_MIN, 255);
}

float readVBAT_filtered(){
  uint32_t acc = 0;
  for(int i = 0; i < 16; i++){ acc += analogReadMilliVolts(VBAT_PIN); delay(2); }
  return ((float)(acc/16) * VBAT_DIV_RATIO) / 1000.0f;
}

// ── IR ───────────────────────────────────────────────────────
// offOut/onOut/signedOut: valores crudos para diagnóstico (página IR del OLED).
// Retorno: valor a comparar contra el umbral, según IR_USE_ABS_DIFF.
int readDiff_mV(uint8_t ledChannel, uint8_t adcPin, int &offOut, int &onOut, int &signedOut){
  uint32_t offAcc = 0, onAcc = 0; 
  for(int i = 0; i < IR_NSAMPLES; i++){
    ledcWrite(ledChannel, 0);   delayMicroseconds(IR_OFF_US);
    offAcc += analogReadMilliVolts(adcPin);
    ledcWrite(ledChannel, 180); delayMicroseconds(IR_ON_US);
    onAcc  += analogReadMilliVolts(adcPin);
    ledcWrite(ledChannel, 0);   delayMicroseconds(IR_OFF_US);
  }
  offOut = (int)(offAcc/IR_NSAMPLES);
  onOut  = (int)(onAcc/IR_NSAMPLES);
  signedOut = offOut - onOut;

  int used = IR_USE_ABS_DIFF ? abs(signedOut) : signedOut;
  return (used < 0) ? 0 : used;
}

void readIR(){
  irFL = readDiff_mV(CH_IR_FL, FT_FL, irFL_off, irFL_on, irFL_signed);
  irL  = readDiff_mV(CH_IR_L,  FT_L,  irL_off,  irL_on,  irL_signed);
  irR  = readDiff_mV(CH_IR_R,  FT_R,  irR_off,  irR_on,  irR_signed);
  irFR = readDiff_mV(CH_IR_FR, FT_FR, irFR_off, irFR_on, irFR_signed);
}

inline bool hasWallLeft()  { readIR(); return irL >= IR_WALL_THR_L; }
inline bool hasWallRight() { readIR(); return irR >= IR_WALL_THR_R; }
// Sin sensor central: pared frontal = FL o FR detectan pared
inline bool hasWallFront() {
  readIR();
  return (irFL >= IR_WALL_THR_FL) || (irFR >= IR_WALL_THR_FR);
}

// ── OLED ─────────────────────────────────────────────────────
void oledHeader(const char* t){
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0,0);
  display.println(t);
}

void oledShow(const char* l1, const char* l2=""){
  oledHeader(l1);
  display.setCursor(0,12);
  display.println(l2);
  display.display();
}

// ── INIT ─────────────────────────────────────────────────────
void sensorsInit(){
  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);
  analogSetPinAttenuation(VBAT_PIN, ADC_11db);
  analogSetPinAttenuation(FT_FL, ADC_11db);
  analogSetPinAttenuation(FT_L,  ADC_11db);
  analogSetPinAttenuation(FT_R,  ADC_11db);
  analogSetPinAttenuation(FT_FR, ADC_11db);

  ledcSetup(CH_IR_FL,20000,8); ledcAttachPin(LED_FL,CH_IR_FL); ledcWrite(CH_IR_FL,0);
  ledcSetup(CH_IR_L, 20000,8); ledcAttachPin(LED_L, CH_IR_L);  ledcWrite(CH_IR_L, 0);
  ledcSetup(CH_IR_R, 20000,8); ledcAttachPin(LED_R, CH_IR_R);  ledcWrite(CH_IR_R, 0);
  ledcSetup(CH_IR_FR,20000,8); ledcAttachPin(LED_FR,CH_IR_FR); ledcWrite(CH_IR_FR,0);

  pinMode(LED_ROJO, OUTPUT);
  pinMode(LED_AZUL, OUTPUT);
  pinMode(LED_BLANCO, OUTPUT);
  ledRojo(false); ledAzul(false); ledBlanco(false);
  bnoLedSetState(BNO_LED_OFF); // NeoPixel apagado hasta que gyroInit() decida su estado
}