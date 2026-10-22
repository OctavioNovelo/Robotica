// =============================================================
//  UMouse S3 — main.cpp   (antes UMouse.ino)
//  ESP32-S3 + Arduino-ESP32 core 3.x + BNO085
// =============================================================
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Preferences.h>
#include <math.h>

#if __has_include(<esp_arduino_version.h>)
  #include <esp_arduino_version.h>
#endif
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR < 3
  #error "Este proyecto requiere Arduino-ESP32 core 3.x (ver platformio.ini)"
#endif

#include "settings.h"
#include "config.h"
#include "pins.h"
#include "status.h"
#include "sensors.h"
#include "bno.h"
#include "netlog.h"
#include "motion.h"
#include "maus.h"
#include "floodfill.h"

// ── GLOBALES ─────────────────────────────────────────────────
// clkDuring = clkAfter = I2C_CLOCK_HZ: si no, la librería deja el bus en 100 kHz tras cada dibujo.
Adafruit_SSD1306 display(OLED_W, OLED_H, &Wire, -1, I2C_CLOCK_HZ, I2C_CLOCK_HZ);
bool g_oledOK = false;

// ── BOTÓN (BOOT, GPIO0 — el mismo del proyecto original) ─────
// Debounce por tiempo, sin delay(). Reporta flancos y duración de la pulsación.
#define BTN_DEBOUNCE_MS    30
#define BTN_SHORT_MIN_MS   30
#define BTN_SHORT_MAX_MS   600
#define HOLD_MENU_MS       3000     // mantener 3 s: aparecen las opciones
#define HOLD_ERASE_MS      5000     // mantener 5 s o más: borrar NVS
#define HOLD_ACTION_MS     800      // acciones de hold en páginas ENC / BNO

struct Button {
  bool     raw = false, down = false, pressed = false, released = false;
  uint32_t tRaw = 0, tDown = 0, lastHeld = 0;
  void update(){
    pressed = released = false;
    bool r = (digitalRead(BOOT_PIN) == LOW);
    uint32_t now = millis();
    if(r != raw){ raw = r; tRaw = now; }
    if(r != down && (now - tRaw) >= BTN_DEBOUNCE_MS){
      down = r;
      if(down){ tDown = tRaw; pressed = true; }
      else    { lastHeld = now - tDown; released = true; }
    }
  }
  uint32_t heldMs() const { return down ? (millis() - tDown) : 0; }
};
Button btn;

// ── AVISO TEMPORAL EN OLED (reemplaza delay() bloqueantes) ───
char     g_toast1[22] = "", g_toast2[22] = "";
uint32_t g_toastUntil = 0;
void toast(const char* a, const char* b = "", uint32_t ms = 900){
  strncpy(g_toast1, a, sizeof(g_toast1)-1); g_toast1[sizeof(g_toast1)-1] = 0;
  strncpy(g_toast2, b, sizeof(g_toast2)-1); g_toast2[sizeof(g_toast2)-1] = 0;
  g_toastUntil = millis() + ms;
}

// ── NVS ──────────────────────────────────────────────────────
// Formato de "walls" SIN cambios: namespace "maze", clave "walls", bool[WALL_ROWS][WALL_COLS].
// Se AÑADE la clave "meta" (MazeMeta). Compatibilidad: un mapa sin "meta" (grabado por versiones
// anteriores) se carga igual que antes y se marca "legado"; nada se borra automáticamente.
// g_explored sigue significando "hay mapa del tamaño correcto" (lo que espera Floodfill/startRun).
Preferences prefs;
bool g_walls[WALL_ROWS][WALL_COLS] = {false};
bool g_explored = false;

#define MAZE_META_VER 1
struct MazeMeta { uint8_t ver, rows, cols, goalR, goalC, startR, startC, complete; };
static bool    g_mapComplete = false;   // meta dice: hubo ida + vuelta completas con este mapa
static bool    g_mapLegacy   = false;   // mapa sin "meta" (versión anterior del firmware)
static bool    g_mapBad      = false;   // con las paredes guardadas la meta es inalcanzable
static uint8_t g_runSaves    = 0;       // guardados en la corrida actual (1 = meta, 2 = vuelta al inicio)
static bool    g_runFresh    = false;   // la corrida actual empezó sin mapa (exploración completa)

static Maus*   g_navMaus     = nullptr; // pose lógica de la corrida en curso (solo para diagnóstico)

// Registro de la pose LÓGICA (Maus): la llama motion.h en cada decisión/lectura.
void navLogPose(const char* tag){
#if NAV_DEBUG
  if(!g_navMaus) return;
  static const char DIRS[] = "?NESO";
  int d = g_navMaus->direction;
  logPrintf("[POSE] %-8s celda=(%d,%d) dir=%c\n", tag, g_navMaus->coords[0], g_navMaus->coords[1],
                DIRS[(d >= 1 && d <= 4) ? d : 0]);
#else
  (void)tag;
#endif
}

// BFS propio (misma indexación que Floodfill) para validar un mapa SIN tocar floodfill.h.
static bool mapReachable(int r0, int c0, int r1, int c1){
  bool seen[MAZE_ROWS][MAZE_COLS] = {{false}};
  int q[MAZE_ROWS*MAZE_COLS][2]; int h = 0, t = 0;
  q[t][0] = r0; q[t][1] = c0; t++; seen[r0][c0] = true;
  const int dr[4] = {-1,0,1,0}, dc[4] = {0,1,0,-1};
  while(h != t){
    int r = q[h][0], c = q[h][1]; h++;
    if(r == r1 && c == c1) return true;
    for(int k = 0; k < 4; k++){
      int nr = r + dr[k], nc = c + dc[k];
      if(nr < 0 || nr >= MAZE_ROWS || nc < 0 || nc >= MAZE_COLS || seen[nr][nc]) continue;
      if(g_walls[r*2 + dr[k]][c*2 + dc[k]]) continue;
      seen[nr][nc] = true; q[t][0] = nr; q[t][1] = nc; t++;
    }
  }
  return false;
}

// Dibujo ASCII del mapa por Serial (S = inicio, G = meta). Fuera del lazo de control.
static void dumpMap(const char* tag){
#if NAV_DEBUG
  logPrintf("[MAPA] %s %dx%d meta=(%d,%d) inicio=(%d,%d) completo=%d legado=%d\n", tag,
                MAZE_ROWS, MAZE_COLS, GOAL_ROW, GOAL_COL, MOUSE_ROW, MOUSE_COL, g_mapComplete, g_mapLegacy);
  char line[4*MAZE_COLS + 4];
  for(int r = 0; r <= MAZE_ROWS; r++){
    int k = 0; line[k++] = '+';
    for(int c = 0; c < MAZE_COLS; c++){
      bool w = (r == 0 || r == MAZE_ROWS) || g_walls[2*r-1][2*c];
      const char* seg = w ? "---+" : "   +";
      for(int i = 0; i < 4; i++) line[k++] = seg[i];
    }
    line[k] = 0; logPrintln(line);
    if(r == MAZE_ROWS) break;
    k = 0; line[k++] = '|';
    for(int c = 0; c < MAZE_COLS; c++){
      char ch = (r == GOAL_ROW && c == GOAL_COL) ? 'G' : (r == MOUSE_ROW && c == MOUSE_COL) ? 'S' : ' ';
      line[k++] = ' '; line[k++] = ch; line[k++] = ' ';
      line[k++] = (c == MAZE_COLS-1 || g_walls[2*r][2*c+1]) ? '|' : ' ';
    }
    line[k] = 0; logPrintln(line);
  }
#else
  (void)tag;
#endif
}

void saveWallsToFlash(){            // la llama Floodfill al llegar a cada objetivo
  motionHaltIfCruising();           // llegó a la meta: frenar si venía en modo crucero
  ff_centerInGoal();                // TURN_MODE 2: quedaba entre dos celdas (entra a la celda 100 mm antes del centro)
  g_runSaves++;
  if(g_runFresh && g_runSaves >= 2) g_mapComplete = true;   // ida + vuelta exploradas
  MazeMeta m = { MAZE_META_VER, MAZE_ROWS, MAZE_COLS, GOAL_ROW, GOAL_COL, MOUSE_ROW, MOUSE_COL,
                 (uint8_t)(g_mapComplete ? 1 : 0) };
  prefs.begin("maze",false);
  prefs.putBytes("walls",g_walls,sizeof(g_walls));
  prefs.putBytes("meta",&m,sizeof(m));
  prefs.end();
  g_mapLegacy = false;
  setStatus(ST_GOAL);
  navLogPose("OBJETIVO");
  dumpMap("guardado");
}

// Borra TODO el namespace "maze", deja la RAM limpia y verifica la lectura.
bool eraseNVS(){
  memset(g_walls,0,sizeof(g_walls));
  g_explored = false;
  g_mapComplete = g_mapLegacy = g_mapBad = false;
  prefs.begin("maze",false);
  bool ok = prefs.clear();
  prefs.end();
  prefs.begin("maze",true);
  size_t left = prefs.getBytesLength("walls");
  prefs.end();
  return ok && left == 0;
}

// ── CALIBRACIÓN DE CENTRADO IR ───────────────────────────────
// Namespace "ircal" aparte de "maze": borrar el mapa NO borra la calibración.
static bool irCalibrateHere(){
  long sl = 0, sr = 0;
  for(int i = 0; i < 4; i++){ sl += readDiff_mV(PIN_IR_L, PIN_FT_L); sr += readDiff_mV(PIN_IR_R, PIN_FT_R); }
  int l = (int)(sl / 4), r = (int)(sr / 4);
  if(l < IR_WALL_THR_L || r < IR_WALL_THR_R) return false;     // hacen falta las DOS paredes
  g_irCalL = l; g_irCalR = r;
  Preferences p; p.begin("ircal", false);
  p.putInt("L", l); p.putInt("R", r);
  p.end();
  return true;
}
// Pivote frontal: robot con el EJE de las ruedas sobre el centro de la celda y pared al frente.
static bool irPivotCalibrateHere(){
  long sum = 0; int n = 0;
  for(int i = 0; i < 4; i++){
    int fl = readDiff_mV(PIN_IR_FL, PIN_FT_FL), fr = readDiff_mV(PIN_IR_FR, PIN_FT_FR);
    int m = irFrontLevel(fl, fr);
    if(m > 0){ sum += m; n++; }
  }
  if(n < 3) return false;                                     // no ve la pared de forma estable
  g_irPivot = (int)(sum / n);
  Preferences p; p.begin("ircal", false); p.putInt("P", g_irPivot); p.end();
  return true;
}
static void irCalLoad(){
  Preferences p; p.begin("ircal", true);
  g_irCalL = p.getInt("L", 0); g_irCalR = p.getInt("R", 0); g_irPivot = p.getInt("P", 0);
  p.end();
}

void loadWalls(){
  prefs.begin("maze",true);
  size_t len=prefs.getBytesLength("walls");
  prefs.end();
  if(len==sizeof(g_walls)){
    g_explored=true;
    MazeMeta m = {};
    prefs.begin("maze",true);
    prefs.getBytes("walls",g_walls,sizeof(g_walls));
    size_t mlen = prefs.getBytesLength("meta");
    if(mlen == sizeof(m)) prefs.getBytes("meta",&m,sizeof(m));
    prefs.end();
    g_mapLegacy = (mlen != sizeof(m));
    bool same = !g_mapLegacy && m.ver == MAZE_META_VER && m.rows == MAZE_ROWS && m.cols == MAZE_COLS &&
                m.goalR == GOAL_ROW && m.goalC == GOAL_COL && m.startR == MOUSE_ROW && m.startC == MOUSE_COL;
    g_mapComplete = same && m.complete;
    if(!g_mapLegacy && !same) NAVLOG("[NVS] AVISO: meta/inicio distintos a los del mapa guardado\n");
    g_mapBad = !mapReachable(MOUSE_ROW, MOUSE_COL, GOAL_ROW, GOAL_COL);
    dumpMap("cargado");
    if(g_mapBad){
      setStatus(ST_ERROR);
      NAVLOG("[NVS] mapa INCONSISTENTE: la meta es inalcanzable con las paredes guardadas\n");
      oledShow("Mapa NVS invalido","meta inalcanzable");
    } else {
      setStatus(ST_MAP_LOADED);
      oledShow("Mapa previo", g_mapLegacy ? "cargado (legado)" : g_mapComplete ? "cargado completo" : "cargado parcial");
    }
  } else {
    memset(g_walls,0,sizeof(g_walls));
    oledShow("Mapa nuevo","sin datos");
    NAVLOG("[NVS] sin mapa guardado\n");
  }
  delay(500);
}

// ── PÁGINAS UI ───────────────────────────────────────────────
// PAGE_RUN es la primera — más fácil acceso en campo
enum Page : uint8_t { PAGE_RUN=0, PAGE_STATUS, PAGE_IR, PAGE_PIVOT, PAGE_ENC, PAGE_BNO, PAGE_COMPASS, PAGE_DIAG, PAGE_COUNT };
Page g_page = PAGE_RUN;

enum HoldState : uint8_t { HS_IDLE=0, HS_MENU, HS_ERASED };
HoldState g_hold = HS_IDLE;
bool g_holdLatched = false;

void drawBar(int x, int y, int w, int h, float frac){
  frac = cf(frac, 0.0f, 1.0f);
  display.drawRect(x, y, w, h, SSD1306_WHITE);
  display.fillRect(x, y, (int)(w * frac), h, SSD1306_WHITE);
}

void drawPageRun(float vbat){
  // LEDs de pared en vivo también en la página principal (azul = frente, blanco = izq, rojo = der)
  readIR();
  wallLedsSet(wallFrontFromReadings(), irL >= IR_WALL_THR_L, irR >= IR_WALL_THR_R);
  oledHeader("RUN FLOODFILL");
  display.setCursor(0,12);
  display.print("Mapa: "); display.print(g_explored?"PREVIO":"NUEVO");
  display.print("  M:"); display.print(NAV_MODE);
  display.setCursor(0,22);
  display.print("VBAT:"); display.print(vbat,2); display.print(g_vbatSat?"+V ":"V ");
  display.print(PROFILE==1?"TEST":(PROFILE==2?"COMP":"ROBG"));
  display.setCursor(0,32);
  display.print("BNO:"); display.print(bnoStatusStr());
  display.setCursor(0,44);
  display.println("HOLD 3s: opciones");
  display.setCursor(0,54);
  display.println("SHORT: sig pagina");
  display.display();
}

// Menú de mantener pulsado en PAGE_RUN
void drawRunHold(uint32_t held){
  oledHeader("OPCIONES");
  if(g_hold == HS_ERASED){
    display.setCursor(0,16); display.println("NVS BORRADA");
    display.setCursor(0,30); display.println("Suelta el boton");
  } else if(held < HOLD_MENU_MS){
    display.setCursor(0,14); display.println("Mantener 3s...");
    drawBar(0, 30, 128, 10, (float)held / HOLD_MENU_MS);
  } else {
    display.setCursor(0,12); display.println("SOLTAR = INICIAR");
    display.setCursor(0,26); display.println("Seguir = BORRAR NVS");
    drawBar(0, 40, 128, 10, (float)(held - HOLD_MENU_MS) / (HOLD_ERASE_MS - HOLD_MENU_MS));
    display.setCursor(0,54); display.print("en "); display.print((HOLD_ERASE_MS - held)/1000.0f,1); display.print(" s");
  }
  display.display();
}

void drawPageStatus(float vbat){
  oledHeader("STATUS");
  display.setCursor(0,12);
  display.print("VBAT:"); display.print(vbat,2); display.print(g_vbatSat?"+":""); display.print(" dMax:"); display.print(g_dutyMax);
  display.setCursor(0,22);
  display.print("CELL:"); display.print((int)CELL_MM);
  display.print(" PASO:"); display.print(CELL_STEP_MM,1);
  display.setCursor(0,32);
  display.print("P:"); display.print(READ_BACK_MM,1); display.print(PRESHIFT_ENABLE ? "" : "off");
  display.print(" T:"); display.print(TURN_POINT_OFFSET_MM,0);
  display.print(" T90:"); display.print(ENC_TURN90);
  display.setCursor(0,42);
  display.print("FWD_PWM:"); display.print(FWD_PWM);
  display.print(" TRIM:"); display.print(ENC_TRIM_R,3);
  display.setCursor(0,52);
  display.print("KP:"); display.print(KP_ENC,2);
  display.print(" KI:"); display.print(KI_ENC,2);
  display.display();
}

void drawPageIR(){
  readIR();
  wallLedsSet(wallFrontFromReadings(), irL >= IR_WALL_THR_L, irR >= IR_WALL_THR_R);   // en vivo
  oledHeader("IR");
  display.setCursor(0,12);
  display.print("FL:"); display.print(irFL);
  display.print(" FR:"); display.print(irFR);
  display.setCursor(0,22);
  display.print("L:"); display.print(irL);
  display.print(" R:"); display.print(irR);
  display.setCursor(0,34);
  display.print("WL:"); display.print(irL>=IR_WALL_THR_L?"Y":"N");
  display.print(" WF:"); display.print(wallFrontFromReadings()?"Y":"N");
  display.print(" WR:"); display.print(irR>=IR_WALL_THR_R?"Y":"N");
  display.setCursor(0,46);
  // Centrado lateral (sin mover motores). Calibrar: robot CENTRADO entre 2 paredes + HOLD.
  // Verificación: pegado a la pared DERECHA debe decir "gira IZQ"; a la IZQUIERDA, "gira DER".
  // Si sale al revés: L/R intercambiados en el montaje, o KP_IR_CENTER_SIGN=-1.
  if(!irCalibrated()){
    display.print("ctr: HOLD=calibrar");
  } else {
    uint8_t ok = 0;
    float c = irCenteringError(irL, irR, &ok);
    if(ok == 0) display.print("ctr: sin pared util");
    else {
      display.print("ctr:"); display.print(c,2);
      display.print(c > 0.03f ? " gira IZQ" : c < -0.03f ? " gira DER" : " recto");
      if(ok != 3) display.print(ok == 1 ? " [L]" : " [R]");   // solo una pared utilizable
    }
  }
  display.setCursor(0,56);
  if(irCalibrated()){
    display.print("c"); display.print(g_irCalL); display.print("/"); display.print(g_irCalR);
    display.print(" f"); display.print((float)irL/g_irCalL,1); display.print("/"); display.print((float)irR/g_irCalR,1);
  } else {
    display.print("sin cal lateral  P:"); display.print(g_irPivot);
  }
  display.display();
}

void drawPagePivot(){
  readIRFront();
  int lvl = irFrontLevel(irFL, irFR);
  wallLedsSet(wallFrontFromReadings(), false, false);   // en vivo (solo frente)
  oledHeader("CAL PIVOTE FRONTAL");
  display.setCursor(0,12);
  display.print("FL:"); display.print(irFL); display.print(" FR:"); display.print(irFR);
  display.setCursor(0,22);
  display.print("nivel:"); display.print(lvl);
  display.print("  guard:"); if(irPivotCalibrated()) display.print(g_irPivot); else display.print("--");
  display.setCursor(0,34);
  display.print("Eje en PUNTO DE GIRO");
  display.setCursor(0,44);
  display.print("centro+T, pared al");
  display.setCursor(0,54);
  display.print("frente. HOLD=guardar");
  display.display();
}

void drawPageEnc(){
  long l,lA,r,rA; getEncAll(l,lA,r,rA);
  oledHeader("ENCODERS");
  display.setCursor(0,12);
  display.print("L:"); display.print(l); display.print("/"); display.print(lA);
  display.setCursor(0,22);
  display.print("R:"); display.print(r); display.print("/"); display.print(rA);
  display.setCursor(0,36);
  display.println("HOLD: reset encoders");
  display.display();
}

void drawPageBNO(){
  oledHeader("BNO085");
  display.setCursor(64,0);
  display.print(bnoStatusStr()); display.print(" M:"); display.print(NAV_MODE);
  display.setCursor(0,12);
  display.print("Hdg:"); display.print(bnoHeading(),1);
  display.print(" Yaw:"); display.print(bnoYawRel(),0);
  display.setCursor(0,22);
  display.print("Ref:"); display.print(g_bno.yawRef,1);
  display.print(" Cal:"); display.print(g_bno.accuracy);
  display.setCursor(0,32);
  display.print("Gz:"); display.print(g_bno.gyroZ,2);
  display.print(" T:"); display.print(g_navHdgTarget,0);
  display.setCursor(0,42);
  display.print("age:"); display.print(g_bno.lastYawMs ? (int)(millis()-g_bno.lastYawMs) : -1);
  display.print(" rst:"); display.print(g_bno.resets);
  display.print("/"); display.print(g_bno.recoveries);
  display.print(" gap:"); display.print(g_bno.maxGapMs);   // máx ms sin yaw sondeando continuo
  display.setCursor(0,52);
  if(g_bno.state==BNO_ST_ERROR){ display.print("ERR:"); display.print(g_bno.err); }
  else if(g_navWarn[0])        { display.print("W:"); display.print(g_navWarn); }
  else                         { display.print("HOLD: cero  fb:"); display.print(g_navFallbacks); }
  display.display();
}

// Brújula 2D: N arriba = dirección de arranque (heading 0). La flecha es el robot.
void drawPageCompass(){
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  const int cx = 30, cy = 34, R = 20;
  display.drawCircle(cx, cy, R, SSD1306_WHITE);
  display.setCursor(cx-3, 2);   display.print("N");
  display.setCursor(cx-3, 56);  display.print("S");
  display.setCursor(cx+R+4, cy-3); display.print("E");
  display.setCursor(0, cy-3);      display.print("W");

  const float h = bnoHeading() * (PI / 180.0f);
  const float s = sinf(h), c = cosf(h);
  int tx = cx + (int)lroundf((R-2) * s), ty = cy - (int)lroundf((R-2) * c);
  display.drawLine(cx, cy, tx, ty, SSD1306_WHITE);
  // punta de flecha
  const float a1 = h + 2.6f, a2 = h - 2.6f;                 // ±150° respecto al rumbo
  display.drawLine(tx, ty, tx + (int)lroundf(6*sinf(a1)), ty - (int)lroundf(6*cosf(a1)), SSD1306_WHITE);
  display.drawLine(tx, ty, tx + (int)lroundf(6*sinf(a2)), ty - (int)lroundf(6*cosf(a2)), SSD1306_WHITE);

  display.setCursor(66, 0);  display.print("Heading");
  display.setCursor(66, 10); display.print(bnoHeading(),1); display.print((char)247);
  display.setCursor(66, 22); display.print("Yaw ");  display.print(bnoYawRel(),0);
  display.setCursor(66, 34); display.print("BNO:"); display.print(bnoStatusStr());
  display.setCursor(66, 44); display.print("MODE:"); display.print(NAV_MODE);
  display.setCursor(66, 54);
  if(g_bno.state==BNO_ST_ERROR){ display.print(g_bno.err); }
  else { display.print("Cal:"); display.print(g_bno.accuracy); display.print(" T:"); display.print(g_navHdgTarget,0); }
  display.display();
}

// ── DIAGNÓSTICO MOTORES + BNO (robot LEVANTADO, ruedas libres) ──
// Hace girar los motores en ciclos (avance / freno / libre, como una corrida) mientras
// loop() atiende el BNO sin pausas. Si "gap", "rst" o "yaw/s" empeoran con motores ON,
// el problema es ELÉCTRICO (3V3, tierra, ruido en SDA/SCL), no de software.
static bool     g_diagOn = false;
static uint32_t g_diagT0 = 0, g_diagRateT = 0, g_diagRateYaw = 0;
static float    g_diagRate = 0;
#define DIAG_MAX_MS     10000
#define DIAG_FWD_MS      1500
#define DIAG_BRAKE_MS     300
#define DIAG_COAST_MS     500

void diagMotors(bool on){
  if(on == g_diagOn) return;
  g_diagOn = on;
  if(on){ bnoClearDiag(); g_diagT0 = millis(); }
  else   { motorCoastAll(); }
}
void diagTick(){
  if(!g_diagOn) return;
  uint32_t t = millis() - g_diagT0;
  if(t > DIAG_MAX_MS){ diagMotors(false); return; }
  uint32_t ph = t % (DIAG_FWD_MS + DIAG_BRAKE_MS + DIAG_COAST_MS);
  if(ph < DIAG_FWD_MS)                    motorSetBoth(FWD_PWM, FWD_PWM);
  else if(ph < DIAG_FWD_MS+DIAG_BRAKE_MS) motorBrakeAll();
  else                                    motorCoastAll();
}
void drawPageDiag(float vbat){
  uint32_t now = millis();
  if(now - g_diagRateT >= 500){
    g_diagRate = (g_bno.yawCount - g_diagRateYaw) * 1000.0f / (float)(now - g_diagRateT);
    g_diagRateYaw = g_bno.yawCount; g_diagRateT = now;
  }
  oledHeader("DIAG MOTOR+BNO");
  display.setCursor(0,12);
  display.print("MOT:"); display.print(g_diagOn ? "ON " : "OFF");
  if(g_diagOn){ display.print(" "); display.print((DIAG_MAX_MS - (now-g_diagT0))/1000); display.print("s"); }
  display.setCursor(0,22);
  display.print("yaw/s:"); display.print(g_diagRate,0); display.print(" (~100)");
  display.setCursor(0,32);
  display.print("gap:"); display.print(g_bno.maxGapMs); display.print("ms rst:"); display.print(g_bno.resets);
  display.setCursor(0,42);
  display.print("age:"); display.print(g_bno.lastYawMs ? (int)(now-g_bno.lastYawMs) : -1);
  display.print(" "); display.print(bnoStatusStr());
  display.print(" "); display.print(g_bno.recoveries);
  display.setCursor(0,52);
  display.print("raw:"); display.print(g_vbatRaw); display.print(" ");
  display.print(analogReadMilliVolts(VBAT_PIN)); display.print("mV HOLD:on/off");
  display.display();
}

void drawCurrentPage(float vbat){
  if(millis() < g_toastUntil){ oledShow(g_toast1, g_toast2); return; }
  if(g_page != PAGE_IR && g_page != PAGE_PIVOT && g_page != PAGE_RUN) wallLedsOff();   // LEDs de pared: solo lecturas vigentes
  switch(g_page){
    case PAGE_RUN:
      if(btn.down && (g_hold != HS_IDLE || btn.heldMs() >= 400)) drawRunHold(btn.heldMs());
      else drawPageRun(vbat);
      break;
    case PAGE_STATUS:  drawPageStatus(vbat); break;
    case PAGE_IR:      drawPageIR();         break;
    case PAGE_PIVOT:   drawPagePivot();      break;
    case PAGE_ENC:     drawPageEnc();        break;
    case PAGE_BNO:     drawPageBNO();        break;
    case PAGE_COMPASS: drawPageCompass();    break;
    case PAGE_DIAG:    drawPageDiag(vbat);   break;
    default: break;
  }
}

// ── RUN ──────────────────────────────────────────────────────
static void showRunError(const char* a, const char* b){
  setStatus(ST_ERROR);
  toast(a, b, 2500);
}

static void startRun(){
  diagMotors(false);
  // Requisitos según NAV_MODE
  if(NAV_USE_BNO){
    if(g_bno.state != BNO_ST_OK && !bnoRecover()){ showRunError("BNO no disponible", g_bno.err); return; }
    if(!bnoWaitFresh(500)){ showRunError("BNO sin datos", "revisar I2C/pwr"); return; }
    bnoZeroHeading();            // el robot está quieto contra la pared trasera: Norte = 0°
    bnoClearDiag();              // contadores y presupuesto de recuperaciones por corrida
    g_bno.needRebase = false;
    g_navHdgTarget = 0.0f;
    g_navWarn = "";
    g_navFallbacks = 0;
  }
#if CAL_STRAIGHT_CELLS > 0
  {
    // Prueba de recta (settings.h): sin FloodFill, sin lectura de paredes, sin tocar NVS.
    oledShow("CAL RECTA", "arrancando...");
    setStatus(ST_STRAIGHT);
    delay(400);
    const float odo = ff_calStraight(CAL_STRAIGHT_CELLS);
    setStatus(ST_GOAL);
    char l2[64];
    snprintf(l2, sizeof(l2), "%d celdas\nodo=%.1fmm\nC=%.1f adj=%.1f\nmide D. SHORT: UI",
             CAL_STRAIGHT_CELLS, odo, (float)CELL_STEP_MM, (float)CELL_STEP_ADJ_MM);
    oledShow("CAL RECTA", l2);
    while(true){
      btn.update();
      if(btn.released && btn.lastHeld >= BTN_SHORT_MIN_MS && btn.lastHeld < BTN_SHORT_MAX_MS) break;
      delay(10);
    }
    setStatus(ST_OFF);
    return;
  }
#endif
  if(NAV_MODE == 2 && !g_explored){
    // Sin IR no se pueden descubrir paredes: solo se puede recorrer un mapa ya guardado.
    showRunError("NAV2 necesita mapa", "explora con M1/M3");
    return;
  }
  if(NAV_MODE == 2 && !g_mapComplete && !g_mapLegacy){
    // Sin IR, las celdas no exploradas se asumirían abiertas: ruta insegura.
    showRunError("NAV2: mapa parcial", "explora ida+vuelta");
    return;
  }
  if(g_explored && g_mapBad){
    showRunError("Mapa NVS invalido", "HOLD 5s: borrar");
    return;
  }

  oledShow("Iniciando","floodfill...");
  setStatus(ST_FLOOD);
  delay(400);

  Maus maus;
  maus.coords[0]=MOUSE_ROW;
  maus.coords[1]=MOUSE_COL;
  maus.direction=MOUSE_START_DIRECTION;

  g_navMaus  = &maus;
  g_runSaves = 0;
  g_runFresh = !g_explored;
  g_navExploring = !g_explored;     // con mapa previo la confirmación del frontal va sin tope de velocidad
  NAVLOG("\n[RUN] mapa %s%s  NAV_MODE=%d  preshift=%s\n", g_explored ? "PREVIO" : "NUEVO",
         g_explored ? (g_mapLegacy ? " (legado)" : g_mapComplete ? " (completo)" : " (parcial)") : "",
         NAV_MODE, PRESHIFT_ENABLE ? "ON" : "OFF");
  navLogPose("INICIO");

  const int goal[2]={GOAL_ROW,GOAL_COL};
  Floodfill flood(g_walls,goal,&maus,g_explored);
  ff_initialAdvance();
  flood.solve();

  g_explored=true;
  g_navMaus = nullptr;
  irSnapInvalidate();
  setStatus(ST_GOAL);
  logPrintf("BNO corrida: yaw=%lu rst=%u rec=%u gapMax=%ums fallbacks=%u stat=%s\n",
                (unsigned long)g_bno.yawCount, g_bno.resets, g_bno.recoveries,
                g_bno.maxGapMs, g_navFallbacks, bnoStatusStr());
  oledShow("Completado!","SHORT: UI");
  uint32_t tBlink = millis(); bool on = true;
  while(true){                       // igual que el original: espera SHORT para volver a la UI
    btn.update();
    if(btn.released && btn.lastHeld >= BTN_SHORT_MIN_MS && btn.lastHeld < BTN_SHORT_MAX_MS) break;
    if(millis() - tBlink >= 400){    // ST_GOAL parpadea: no se confunde con el verde de "recto"/"BNO OK"
      tBlink = millis(); on = !on;
      if(on) setStatus(ST_GOAL); else setStatusColor(0,0,0);
    }
    delay(10);
  }
  setStatus(ST_OFF);
}

// ── BOTÓN: navegación de páginas, HOLD 3 s / 5 s, acciones ───
static void nextPage(){ diagMotors(false); g_page = (Page)((g_page + 1) % PAGE_COUNT); }

static void doEraseNVS(){
  bool ok = eraseNVS();
  g_hold = HS_ERASED;
  if(ok){ setStatus(ST_NVS_CLEARED); }
  else  { setStatus(ST_ERROR); toast("NVS ERROR","no se pudo borrar",2000); }
}

static void handleButton(){
  btn.update();
  if(btn.pressed){ g_holdLatched = false; g_hold = HS_IDLE; }

  if(btn.down){
    const uint32_t held = btn.heldMs();
    if(g_page == PAGE_RUN){
      if(held >= HOLD_MENU_MS && g_hold == HS_IDLE) g_hold = HS_MENU;
      if(held >= HOLD_ERASE_MS && g_hold != HS_ERASED) doEraseNVS();       // una sola vez
    } else if(!g_holdLatched && held >= HOLD_ACTION_MS){
      if(g_page == PAGE_ENC){ g_holdLatched = true; resetEncoders(); toast("Encoders","reseteados",400); }
      if(g_page == PAGE_BNO){ g_holdLatched = true; bnoZeroHeading(); g_navWarn = ""; toast("BNO","heading = 0",600); }
      if(g_page == PAGE_DIAG){ g_holdLatched = true; diagMotors(!g_diagOn); }
      if(g_page == PAGE_PIVOT){
        g_holdLatched = true;
        if(irPivotCalibrateHere()){ char b[22]; snprintf(b, sizeof(b), "nivel:%d", g_irPivot); toast("Pivote guardado", b, 1500); }
        else toast("Pivote ERROR", "no ve la pared", 1500);
      }
      if(g_page == PAGE_IR){
        g_holdLatched = true;
        bool ok = irCalibrateHere();
        if(ok){ char b[22]; snprintf(b, sizeof(b), "L:%d R:%d", g_irCalL, g_irCalR); toast("IR centro guardado", b, 1500); }
        else  toast("IR cal ERROR", "necesita 2 paredes", 1500);
      }
    }
  }

  if(btn.released){
    const uint32_t h = btn.lastHeld;
    const HoldState hs = g_hold;
    g_hold = HS_IDLE;
    if(g_page == PAGE_RUN){
      if(hs == HS_ERASED){ toast("NVS borrada","mapa nuevo",1500); setStatus(ST_OFF); return; }   // ya se borró: no iniciar
      if(h >= HOLD_ERASE_MS){ doEraseNVS(); g_hold = HS_IDLE; toast("NVS borrada","mapa nuevo",1500); return; }   // liberó justo pasados 5 s
      if(h >= HOLD_MENU_MS){ startRun(); return; }                         // 3 s <= h < 5 s
    }
    if(!g_holdLatched && h >= BTN_SHORT_MIN_MS && h < BTN_SHORT_MAX_MS) nextPage();
  }
}

// ── SETUP ────────────────────────────────────────────────────
// FloodFill no encuentra camino a la meta con las paredes conocidas (cambio mínimo en floodfill.h).
void ff_goalUnreachable(){
  dumpMap("inalcanzable");
  navFailStop("meta inalcanzable");
}

void setup(){
  Serial.setTxBufferSize(1024);       // los logs NAV_DEBUG no deben bloquear el control
  Serial.begin(115200);
  logInit();                          // log en RAM (sobrevive a ESP.restart) + motivo del reinicio
  pinMode(BOOT_PIN, INPUT_PULLUP);
  statusInit();
  setStatus(ST_BOOT);

  // BNO (PDF): apagar -> esperar -> encender -> esperar. SIN tráfico I2C todavía.
  bnoBootPowerCycle();

  // Recién ahora I2C, OLED y BNO
  i2cBegin();
  g_oledOK = display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR);
  oledShow("UMouse S3","iniciando...");

  sensorsInit();
  motionInit();

  float vbat = readVBAT_V();
  g_dutyMax  = calcDutyMax(vbat);
  if(g_dutyMax<20) g_dutyMax=20;

  setStatus(ST_BNO_INIT);
  oledShow("BNO085","inicializando...");
  bool bnoOK = bnoInit();
  if(bnoOK && bnoWaitFresh(800)){ bnoZeroHeading(); }
  logPrintf("BNO: %s addr=0x%02X\n", bnoOK ? "inicializado OK" : "ERROR", g_bno.addr);
  if(bnoOK){
    setStatus(ST_BNO_OK);
    oledShow("BNO085 OK", "");
  } else {
    setStatus(ST_ERROR);
    oledShow("BNO085 ERROR", g_bno.err);
  }
  delay(500);

  // Indicador de NAV_MODE
  setStatus(NAV_MODE==1 ? ST_MODE1 : NAV_MODE==2 ? ST_MODE2 : ST_MODE3);
  char m[16]; snprintf(m, sizeof(m), "NAV_MODE %d", NAV_MODE);
  oledShow(m, NAV_MODE==1 ? "IR + encoders" : NAV_MODE==2 ? "BNO solo" : "BNO + IR/enc");
  delay(600);

  irCalLoad();                        // referencia de centrado IR (si existe)
  loadWalls();                        // (el borrado ahora se hace desde PAGE_RUN)
#if WIFI_LOG_ENABLE
  netlogStartAP();                    // red WIFI_AP_SSID -> http://192.168.4.1 (log en vivo)
  oledShow("Listo", "WiFi: " WIFI_AP_SSID "\nhttp://192.168.4.1");
  delay(1200);
#else
  oledShow("Listo","");
#endif
  delay(300);
}

// ── LOOP ─────────────────────────────────────────────────────
void loop(){
  bnoUpdate();
  diagTick();
  handleButton();

  // VBAT cada 500ms
  static uint32_t tV=0; static float vbat=0.0f;
  if(millis()-tV>500){
    tV=millis(); vbat=readVBAT_V();
    int dm=calcDutyMax(vbat); if(dm<20) dm=20; g_dutyMax=dm;
  }

  // Refresco UI 120ms
  static uint32_t tUI=0;
  if(millis()-tUI<120) return;
  tUI=millis();
  drawCurrentPage(vbat);
}