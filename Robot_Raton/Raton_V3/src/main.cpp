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
#include "motion.h"
#include "maus.h"
#include "floodfill.h"

// ── GLOBALES ─────────────────────────────────────────────────
Adafruit_SSD1306 display(OLED_W, OLED_H, &Wire, -1);
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
// Sin cambios de formato: namespace "maze", clave "walls", bool[WALL_ROWS][WALL_COLS].
// (El original no guarda posición, meta ni estado de exploración: g_explored = "hay mapa".)
Preferences prefs;
bool g_walls[WALL_ROWS][WALL_COLS] = {false};
bool g_explored = false;

void saveWallsToFlash(){            // la llama Floodfill al llegar a cada objetivo
  motionHaltIfCruising();           // llegó a la meta: frenar si venía en modo crucero
  prefs.begin("maze",false);
  prefs.putBytes("walls",g_walls,sizeof(g_walls));
  prefs.end();
  setStatus(ST_GOAL);
}

// Borra TODO el namespace "maze", deja la RAM limpia y verifica la lectura.
bool eraseNVS(){
  memset(g_walls,0,sizeof(g_walls));
  g_explored = false;
  prefs.begin("maze",false);
  bool ok = prefs.clear();
  prefs.end();
  prefs.begin("maze",true);
  size_t left = prefs.getBytesLength("walls");
  prefs.end();
  return ok && left == 0;
}

void loadWalls(){
  prefs.begin("maze",true);
  size_t len=prefs.getBytesLength("walls");
  prefs.end();
  if(len==sizeof(g_walls)){
    g_explored=true;
    prefs.begin("maze",true);
    prefs.getBytes("walls",g_walls,sizeof(g_walls));
    prefs.end();
    setStatus(ST_MAP_LOADED);
    oledShow("Mapa previo","cargado OK");
  } else {
    memset(g_walls,0,sizeof(g_walls));
    oledShow("Mapa nuevo","sin datos");
  }
  delay(500);
}

// ── PÁGINAS UI ───────────────────────────────────────────────
// PAGE_RUN es la primera — más fácil acceso en campo
enum Page : uint8_t { PAGE_RUN=0, PAGE_STATUS, PAGE_IR, PAGE_ENC, PAGE_BNO, PAGE_COMPASS, PAGE_COUNT };
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
  oledHeader("RUN FLOODFILL");
  display.setCursor(0,12);
  display.print("Mapa: "); display.print(g_explored?"PREVIO":"NUEVO");
  display.print("  M:"); display.print(NAV_MODE);
  display.setCursor(0,22);
  display.print("VBAT:"); display.print(vbat,2); display.print("V ");
  display.print(PROFILE==1?"TEST":"COMP");
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
  display.print("VBAT:"); display.print(vbat,2); display.print(" dMax:"); display.print(g_dutyMax);
  display.setCursor(0,22);
  display.print("CELL:"); display.print((int)CELL_MM);
  display.print(" FWD:"); display.print((int)CELL_FWD_MM);
  display.setCursor(0,32);
  display.print("Pre:"); display.print(ENC_PRESHIFT);
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
  display.print("TL:"); display.print(IR_WALL_THR_L);
  display.print(" TF:"); display.print(IR_WALL_THR_FL);
  display.print(" TR:"); display.print(IR_WALL_THR_R);
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

void drawCurrentPage(float vbat){
  if(millis() < g_toastUntil){ oledShow(g_toast1, g_toast2); return; }
  switch(g_page){
    case PAGE_RUN:
      if(btn.down && (g_hold != HS_IDLE || btn.heldMs() >= 400)) drawRunHold(btn.heldMs());
      else drawPageRun(vbat);
      break;
    case PAGE_STATUS:  drawPageStatus(vbat); break;
    case PAGE_IR:      drawPageIR();         break;
    case PAGE_ENC:     drawPageEnc();        break;
    case PAGE_BNO:     drawPageBNO();        break;
    case PAGE_COMPASS: drawPageCompass();    break;
    default: break;
  }
}

// ── RUN ──────────────────────────────────────────────────────
static void showRunError(const char* a, const char* b){
  setStatus(ST_ERROR);
  toast(a, b, 2500);
}

static void startRun(){
  // Requisitos según NAV_MODE
  if(NAV_USE_BNO){
    if(g_bno.state != BNO_ST_OK && !bnoRecover()){ showRunError("BNO no disponible", g_bno.err); return; }
    if(!bnoWaitFresh(500)){ showRunError("BNO sin datos", "revisar I2C/pwr"); return; }
    bnoZeroHeading();            // el robot está quieto contra la pared trasera: Norte = 0°
    g_bno.needRebase = false;
    g_navHdgTarget = 0.0f;
    g_navWarn = "";
    g_navFallbacks = 0;
  }
  if(NAV_MODE == 2 && !g_explored){
    // Sin IR no se pueden descubrir paredes: solo se puede recorrer un mapa ya guardado.
    showRunError("NAV2 necesita mapa", "explora con M1/M3");
    return;
  }

  oledShow("Iniciando","floodfill...");
  setStatus(ST_FLOOD);
  delay(400);

  Maus maus;
  maus.coords[0]=MOUSE_ROW;
  maus.coords[1]=MOUSE_COL;
  maus.direction=MOUSE_START_DIRECTION;

  const int goal[2]={GOAL_ROW,GOAL_COL};
  Floodfill flood(g_walls,goal,&maus,g_explored);
  ff_initialAdvance();
  flood.solve();

  g_explored=true;
  setStatus(ST_GOAL);
  oledShow("Completado!","SHORT: UI");
  while(true){                       // igual que el original: espera SHORT para volver a la UI
    btn.update();
    if(btn.released && btn.lastHeld >= BTN_SHORT_MIN_MS && btn.lastHeld < BTN_SHORT_MAX_MS) break;
    delay(10);
  }
  setStatus(ST_OFF);
}

// ── BOTÓN: navegación de páginas, HOLD 3 s / 5 s, acciones ───
static void nextPage(){ g_page = (Page)((g_page + 1) % PAGE_COUNT); }

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
void setup(){
  Serial.begin(115200);
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
  Serial.printf("BNO: %s addr=0x%02X\n", bnoOK ? "inicializado OK" : "ERROR", g_bno.addr);
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

  loadWalls();                        // (el borrado ahora se hace desde PAGE_RUN)
  oledShow("Listo","");
  delay(300);
}

// ── LOOP ─────────────────────────────────────────────────────
void loop(){
  bnoUpdate();
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
