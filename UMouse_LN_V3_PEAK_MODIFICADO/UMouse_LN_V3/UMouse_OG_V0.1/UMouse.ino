// =============================================================
//  UMouse.ino — Archivo principal
//  ESP32, Arduino-ESP32 core 3.x
// =============================================================
#include <Wire.h>
#include <Adafruit_SSD1306.h>
#include <Preferences.h>
#include <math.h>

#include "settings.h"
#include "config.h"
#include "sensors.h"
#include "motion.h"
#include "maus.h"
#include "floodfill.h"

// ── GLOBALES ─────────────────────────────────────────────────
Adafruit_SSD1306 display(OLED_W, OLED_H, &Wire, -1);

// ── BOOT ─────────────────────────────────────────────────────
#define BOOT_PIN 0
inline bool bootDown(){ return digitalRead(BOOT_PIN)==LOW; }

bool bootShortPress(){
  static bool last=false; static uint32_t tD=0;
  bool now=bootDown(), fired=false;
  if(now&&!last) tD=millis();
  if(!now&&last){ uint32_t dt=millis()-tD; if(dt>30&&dt<600) fired=true; }
  last=now; return fired;
}
bool bootLongPress(uint32_t ms){
  static bool last=false; static uint32_t tD=0;
  bool now=bootDown();
  if(now&&!last) tD=millis();
  last=now;
  return now&&(millis()-tD>=ms);
}

// ── NVS ──────────────────────────────────────────────────────
Preferences prefs;
bool g_walls[WALL_ROWS][WALL_COLS] = {false};
bool g_explored = false;

void saveWallsToFlash(){
  prefs.begin("maze",false);
  prefs.putBytes("walls",g_walls,sizeof(g_walls));
  prefs.end();
}
void clearAndSaveWalls(){
  memset(g_walls,0,sizeof(g_walls));
  prefs.begin("maze",false); prefs.remove("walls"); prefs.end();
  g_explored=false;
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
    oledShow("Mapa previo","cargado OK");
  } else {
    memset(g_walls,0,sizeof(g_walls));
    oledShow("Mapa nuevo","sin datos");
  }
  delay(500);
}

// ── PÁGINAS UI ───────────────────────────────────────────────
// PAGE_RUN es la primera — más fácil acceso en campo
enum Page : uint8_t { PAGE_RUN=0, PAGE_STATUS, PAGE_IR, PAGE_ENC, PAGE_COUNT };
Page g_page = PAGE_RUN;

void drawPageRun(float vbat){
  oledHeader("RUN FLOODFILL");
  display.setCursor(0,12);
  display.print("Mapa: "); display.println(g_explored?"PREVIO":"NUEVO");
  display.setCursor(0,22);
  display.print("VBAT:"); display.print(vbat,2); display.println("V");
  display.setCursor(0,32);
  display.print("Profile:"); display.println(PROFILE==1?"TEST":"COMP");
  display.setCursor(0,44);
  display.println("HOLD: iniciar");
  display.setCursor(0,54);
  display.println("SHORT: sig pagina");
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
  display.print("L:"); display.print(irL);
  display.print(" C:"); display.print(irC);
  display.print(" R:"); display.print(irR);
  display.setCursor(0,24);
  display.print("WL:"); display.print(hasWallLeft()?"Y":"N");
  display.print(" WF:"); display.print(hasWallFront()?"Y":"N");
  display.print(" WR:"); display.print(hasWallRight()?"Y":"N");
  display.setCursor(0,36);
  display.print("TL:"); display.print(IR_WALL_THR_L);
  display.print(" TC:"); display.print(IR_WALL_THR_C);
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

// ── SETUP ────────────────────────────────────────────────────
void setup(){
  pinMode(BOOT_PIN, INPUT_PULLUP);
  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(400000);
  display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR);
  oledShow("UMouse","iniciando...");
  delay(300);

  sensorsInit();
  motionInit();

  float vbat = readVBAT_V();
  g_dutyMax  = calcDutyMax(vbat);
  if(g_dutyMax<20) g_dutyMax=20;

  // BOOT al arrancar = borrar NVS
  if(bootDown()){
    oledShow("BOOT: soltar","para borrar NVS");
    delay(1200);
    if(bootDown()){ clearAndSaveWalls(); oledShow("NVS borrado",""); delay(800); }
    while(bootDown()) delay(10);
  }

  loadWalls();
  oledShow("Listo","");
  delay(300);
}

// ── LOOP ─────────────────────────────────────────────────────
void loop(){
  // VBAT cada 500ms
  static uint32_t tV=0; static float vbat=0.0f;
  if(millis()-tV>500){
    tV=millis(); vbat=readVBAT_V();
    int dm=calcDutyMax(vbat); if(dm<20) dm=20; g_dutyMax=dm;
  }

  // Cambio de página
  if(bootShortPress()){
    g_page=(Page)((g_page+1)%PAGE_COUNT);
  }

  // Acción PAGE_ENC: long press = reset encoders
  if(g_page==PAGE_ENC && bootLongPress(800)){
    while(bootDown()) delay(10);
    resetEncoders();
    oledShow("Encoders","reseteados"); delay(400);
  }

  // Acción PAGE_RUN: long press = iniciar floodfill
  if(g_page==PAGE_RUN && bootLongPress(1500)){
    while(bootDown()) delay(10);
    oledShow("Iniciando","floodfill...");
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
    oledShow("Completado!","SHORT: UI");
    while(!bootShortPress()) delay(50);
  }

  // Refresco UI 120ms
  static uint32_t tUI=0;
  if(millis()-tUI<120) return;
  tUI=millis();

  switch(g_page){
    case PAGE_RUN:    drawPageRun(vbat);    break;
    case PAGE_STATUS: drawPageStatus(vbat); break;
    case PAGE_IR:     drawPageIR();         break;
    case PAGE_ENC:    drawPageEnc();        break;
    default: break;
  }
}
