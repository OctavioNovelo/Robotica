#pragma once
#include <Wire.h>
#include <Adafruit_SSD1306.h>
#include "settings.h"
#include "pins.h"
#include "status.h"

// =============================================================
//  UMouse S3 — sensors.h
//  IR diferencial (4 sensores: FL, L, R, FR), VBAT, OLED
//  Misma técnica del original: OFF/ON alternado con emisor por LEDC.
// =============================================================

// ── LEDC IR ──────────────────────────────────────────────────
// 4 emisores IR + 4 pines de motor = 8 canales = TODOS los del ESP32-S3.
static const int IR_NSAMPLES = 12;
static const int IR_ON_US    = 250;
static const int IR_OFF_US   = 250;
static const int IR_LED_DUTY = 180;

// ── OLED ─────────────────────────────────────────────────────
#define OLED_W     128
#define OLED_H     64
#define OLED_ADDR  0x3C

extern Adafruit_SSD1306 display;
extern bool g_oledOK;

// ── ESTADO IR ────────────────────────────────────────────────
int irFL = 0, irL = 0, irR = 0, irFR = 0;

// ── VBAT ─────────────────────────────────────────────────────
// ADC1 a 12 dB: Espressif da 0-3100 mV como rango medible (Waveshare recomienda ~2900).
// Con VBAT_DIV=4, una 3S llena (12.6 V) pone ~3.15 V en el pin: FUERA de rango, el ADC
// se pega al techo y la lectura se queda en ~11.6 V aunque la batería tenga 12.5 V.
// g_vbatSat=true => "la lectura está en el techo, el voltaje real es >= lo mostrado".
// Arreglo real = divisor mayor (ver VBAT_DIV) o menor ganancia de entrada; por software
// no se puede recuperar una señal recortada.
// V_bateria = raw_promedio * VBAT_V_PER_COUNT
// Calibrado: 12.60 V reales => raw ~3556 => 12.60 / 3556 = 0.003544
// Re-calibrar: con multímetro mira "raw:" en la página DIAG y pon V_real / raw.
static const float    VBAT_V_PER_COUNT = 0.003544f;
static const uint32_t VBAT_RAW_SAT     = 4080;     // cerca de 4095 = techo del ADC
static bool     g_vbatSat = false;
static uint32_t g_vbatRaw = 0;
static uint32_t _vbatRawAvg(int n){
  uint32_t acc = 0;
  for(int i = 0; i < n; i++){ acc += analogRead(VBAT_PIN); if(n > 1) delay(2); }
  return acc / n;
}
inline float readVBAT_V(){
  g_vbatRaw = _vbatRawAvg(4);
  g_vbatSat = (g_vbatRaw >= VBAT_RAW_SAT);
  return g_vbatRaw * VBAT_V_PER_COUNT;
}
float readVBAT_filtered(){
  g_vbatRaw = _vbatRawAvg(16);
  g_vbatSat = (g_vbatRaw >= VBAT_RAW_SAT);
  return g_vbatRaw * VBAT_V_PER_COUNT;
}

inline int calcDutyMax(float vbat){
  if(vbat < 1.0f) return 30;
  float d = 255.0f * (V_MOTOR_LIMIT / vbat);
  return (int)constrain(d, 0.0f, 255.0f);
}

// ── IR ───────────────────────────────────────────────────────
// nsamples configurable: IR_NSAMPLES (12) para decisiones de pared (scanWalls,
// página IR — no es tiempo-crítico, se llama una vez por celda); menos
// muestras para usos dentro de lazos de control donde sí importa la latencia
// (ver IR_ALIGN_NSAMPLES, IR_CENTER_NSAMPLES en settings.h).
int readDiff_mV_n(uint8_t ledPin, uint8_t adcPin, int nsamples){
  uint32_t offAcc = 0, onAcc = 0;
  for(int i = 0; i < nsamples; i++){
    ledcWrite(ledPin, 0);           delayMicroseconds(IR_OFF_US);
    offAcc += analogReadMilliVolts(adcPin);
    ledcWrite(ledPin, IR_LED_DUTY); delayMicroseconds(IR_ON_US);
    onAcc  += analogReadMilliVolts(adcPin);
    ledcWrite(ledPin, 0);           delayMicroseconds(IR_OFF_US);
  }
  int diff = (int)(offAcc/nsamples) - (int)(onAcc/nsamples);
#if IR_USE_ABS_DIFF
  if(diff < 0) diff = -diff;
#else
  if(diff < 0) diff = 0;
#endif
  return diff;
}
inline int readDiff_mV(uint8_t ledPin, uint8_t adcPin){ return readDiff_mV_n(ledPin, adcPin, IR_NSAMPLES); }

inline void readIRFront(){
  irFL = readDiff_mV(PIN_IR_FL, PIN_FT_FL);
  irFR = readDiff_mV(PIN_IR_FR, PIN_FT_FR);
}
void readIR(){                        // todos (páginas OLED)
  readIRFront();
  irL = readDiff_mV(PIN_IR_L, PIN_FT_L);
  irR = readDiff_mV(PIN_IR_R, PIN_FT_R);
}

inline bool wallFrontFromReadings(){
#if IR_FRONT_REQUIRE_BOTH
  return irFL >= IR_WALL_THR_FL && irFR >= IR_WALL_THR_FR;
#else
  return irFL >= IR_WALL_THR_FL || irFR >= IR_WALL_THR_FR;
#endif
}

// Igual que el original: cada consulta mide justo antes de decidir.
inline bool hasWallLeft()  { irL = readDiff_mV(PIN_IR_L, PIN_FT_L); return irL >= IR_WALL_THR_L; }
inline bool hasWallRight() { irR = readDiff_mV(PIN_IR_R, PIN_FT_R); return irR >= IR_WALL_THR_R; }
inline bool hasWallFront() { readIRFront(); return wallFrontFromReadings(); }

// ── INSTANTÁNEA IR (decisión de paredes + LEDs) ──────────────
// La rellenan las mismas lecturas que usa FloodFill (ff_hasWall* en motion.h) y la
// página IR. valid=false (y LEDs apagados) cuando deja de corresponder a la pose
// actual: al empezar un giro, al fallar un movimiento o al salir de la página IR.
struct IrSnap {
  int  fl = 0, fr = 0, l = 0, r = 0;
  bool front = false, left = false, right = false;
  uint32_t ms = 0;
  bool valid = false;
  bool full  = false;     // F, L y R leídos en esta pose (FloodFill puede volver a pedirlos)
  bool frontSure = false; // el frontal se leyó cerca del pivote (TURN_MODE 2: la lectura temprana no alcanza)
};
static IrSnap g_irSnap;
inline void irSnapShowLeds(){ wallLedsSet(g_irSnap.front, g_irSnap.left, g_irSnap.right); }
inline void irSnapInvalidate(){ g_irSnap = IrSnap(); wallLedsOff(); }

// ── ZONA BUENA / CENTRADO / FRENADO (ver explicación en settings.h) ──
// "Zona buena" = por encima del umbral de ruido y por debajo del margen de
// saturación (IR_FRONT_SAT_FACTOR * target). Fuera de ahí el valor no sirve
// para estimar distancia.
inline bool irInGoodZone(int diff, int thr, int target){
  return diff >= thr && diff <= (int)(target * IR_FRONT_SAT_FACTOR);
}

// Referencia de centrado: lecturas L y R con el robot CENTRADO entre dos paredes
// (0 = sin calibrar => el centrado no actúa). Se guardan en NVS desde la página IR.
static int g_irCalL = 0, g_irCalR = 0;
inline bool irCalibrated(){ return g_irCalL > 0 && g_irCalR > 0; }

// Referencia del PIVOTE frontal: nivel FL/FR con el EJE de las ruedas sobre el centro de la
// celda y una pared al frente (0 = sin calibrar). Se guarda en NVS desde la página CAL PIVOTE.
static int g_irPivot = 0;
inline bool irPivotCalibrated(){ return g_irPivot > 0; }
// Nivel frontal único: promedio si los dos ven pared; si solo uno, ese.
inline int irFrontLevel(int fl, int fr){
  bool a = fl >= IR_WALL_THR_FL, b = fr >= IR_WALL_THR_FR;
  if(a && b) return (fl + fr) / 2;
  return a ? fl : (b ? fr : 0);
}

// Error de centrado e = fR - fL (f = lectura / lectura_centrada). e>0: más cerca de la
// derecha. Un lado cuenta solo si hay pared (>= umbral) y su f está en
// [IR_CENTER_MIN_FRAC, IR_CENTER_MAX_FRAC]; si falta uno se asume f=1 y la ganancia baja.
// Devuelve 0 si no hay calibración o ningún lado es utilizable. okSides: 0,1(L),2(R),3(ambos).
inline float irCenteringError(int l, int r, uint8_t* okSides = nullptr){
  if(okSides) *okSides = 0;
  if(!irCalibrated()) return 0.0f;
  float fL = (float)l / (float)g_irCalL;
  float fR = (float)r / (float)g_irCalR;
  bool okL = (l >= IR_WALL_THR_L) && fL >= IR_CENTER_MIN_FRAC && fL <= IR_CENTER_MAX_FRAC;
  bool okR = (r >= IR_WALL_THR_R) && fR >= IR_CENTER_MIN_FRAC && fR <= IR_CENTER_MAX_FRAC;
  if(okSides) *okSides = (okL ? 1 : 0) | (okR ? 2 : 0);
  if(!okL && !okR) return 0.0f;
  float e = (okR ? fR : 1.0f) - (okL ? fL : 1.0f);
  if(!(okL && okR)) e *= IR_CENTER_ONE_WALL_GAIN;
  return e * KP_IR_CENTER_SIGN;
}

// ── OLED ─────────────────────────────────────────────────────
void oledHeader(const char* t){
  if(!g_oledOK) return;
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0,0);
  display.println(t);
}

void oledShow(const char* l1, const char* l2=""){
  if(!g_oledOK) return;
  oledHeader(l1);
  display.setCursor(0,12);
  display.println(l2);
  display.display();
}

// ── INIT ─────────────────────────────────────────────────────
void sensorsInit(){
  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);
  analogSetPinAttenuation(VBAT_PIN,  ADC_11db);
  analogSetPinAttenuation(PIN_FT_FL, ADC_11db);
  analogSetPinAttenuation(PIN_FT_L,  ADC_11db);
  analogSetPinAttenuation(PIN_FT_R,  ADC_11db);
  analogSetPinAttenuation(PIN_FT_FR, ADC_11db);

  ledcAttach(PIN_IR_FL, 20000, 8); ledcWrite(PIN_IR_FL, 0);
  ledcAttach(PIN_IR_L,  20000, 8); ledcWrite(PIN_IR_L,  0);
  ledcAttach(PIN_IR_R,  20000, 8); ledcWrite(PIN_IR_R,  0);
  ledcAttach(PIN_IR_FR, 20000, 8); ledcWrite(PIN_IR_FR, 0);
}