// =============================================================
//  UMouse.ino — Archivo principal (UI Completa)
// =============================================================
#include <Wire.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_BNO08x.h>
#include <Preferences.h>
#include <math.h>

#include "settings.h"
#include "config.h"
#include "sensors.h"
#include "motion.h"
#include "maus.h"
#include "floodfill.h"
#include "telemetry.h"

// ── PINES Y DIRECCIONES BNO085 ───────────────────────────────
#define PIN_BNO_INT 18  
#define PIN_BNO_RST 38  
#define BNO_ADDR_A  0x4A 
#define BNO_ADDR_B  0x4B 
#define BNO_REPORT_INTERVAL_US 10000 

// ── GLOBALES ─────────────────────────────────────────────────
Adafruit_SSD1306 display(OLED_W, OLED_H, &Wire, -1);
Adafruit_BNO08x bno08x(PIN_BNO_RST); 
sh2_SensorValue_t bnoEvent;          

bool g_oledOK = false;
bool g_bnoOK = false;
uint8_t g_bnoAddr = 0;
float g_yawDeg = 0.0f, g_pitchDeg = 0.0f, g_rollDeg = 0.0f; 
float g_gyroX = 0.0f, g_gyroY = 0.0f, g_gyroZ = 0.0f;       
float g_linAx = 0.0f, g_linAy = 0.0f, g_linAz = 0.0f;       
uint32_t g_lastBNOms = 0;

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

// ── LÓGICA BNO085 ────────────────────────────────────────────
void quaternionToEulerDeg(float qr, float qi, float qj, float qk,
                          float &yawDeg, float &pitchDeg, float &rollDeg) {
  float sqi = qi * qi;
  float sqj = qj * qj;
  float sqk = qk * qk;
  float roll = atan2f(2.0f * (qr * qi + qj * qk), 1.0f - 2.0f * (sqi + sqj));
  float t2 = 2.0f * (qr * qj - qk * qi);
  t2 = t2 < -1.0f ? -1.0f : (t2 > 1.0f ? 1.0f : t2);
  float pitch = asinf(t2);
  float yaw = atan2f(2.0f * (qr * qk + qi * qj), 1.0f - 2.0f * (sqj + sqk));
  
  yawDeg   = yaw   * 180.0f / PI;
  pitchDeg = pitch * 180.0f / PI;
  rollDeg  = roll  * 180.0f / PI;
} 

bool setBNOReports() {
  if (!g_bnoOK) return false;
  bool ok = true;
  ok &= bno08x.enableReport(SH2_GAME_ROTATION_VECTOR, BNO_REPORT_INTERVAL_US);
  ok &= bno08x.enableReport(SH2_GYROSCOPE_CALIBRATED, BNO_REPORT_INTERVAL_US);
  ok &= bno08x.enableReport(SH2_LINEAR_ACCELERATION, BNO_REPORT_INTERVAL_US);
  return ok;
} 

void updateBNO() {
  if (!g_bnoOK) return;
  if (bno08x.wasReset()) setBNOReports();
  for (uint8_t i = 0; i < 8; i++) {
    if (!bno08x.getSensorEvent(&bnoEvent)) return;
    g_lastBNOms = millis();
    switch (bnoEvent.sensorId) {
      case SH2_GAME_ROTATION_VECTOR:
        quaternionToEulerDeg(
          bnoEvent.un.gameRotationVector.real, bnoEvent.un.gameRotationVector.i,
          bnoEvent.un.gameRotationVector.j, bnoEvent.un.gameRotationVector.k,
          g_yawDeg, g_pitchDeg, g_rollDeg
        );
        break;
      case SH2_GYROSCOPE_CALIBRATED:
        g_gyroX = bnoEvent.un.gyroscope.x; g_gyroY = bnoEvent.un.gyroscope.y; g_gyroZ = bnoEvent.un.gyroscope.z;
        break;
      case SH2_LINEAR_ACCELERATION:
        g_linAx = bnoEvent.un.linearAcceleration.x; g_linAy = bnoEvent.un.linearAcceleration.y; g_linAz = bnoEvent.un.linearAcceleration.z;
        break;
    }
  }
} 

void initBNO() {
  pinMode(PIN_BNO_INT, INPUT_PULLUP);
  g_bnoOK = false;
  if (bno08x.begin_I2C(BNO_ADDR_A, &Wire)) {
    g_bnoOK = true; g_bnoAddr = BNO_ADDR_A;
  } else if (bno08x.begin_I2C(BNO_ADDR_B, &Wire)) {
    g_bnoOK = true; g_bnoAddr = BNO_ADDR_B;
  }
  if (g_bnoOK) setBNOReports();
} 


// ── PÁGINAS UI ───────────────────────────────────────────────
// Integración de TODAS las páginas de diagnóstico de UMouse_S3_Test_BootMotor
enum Page : uint8_t { PAGE_RUN=0, PAGE_STATUS, PAGE_IR_DIFF, PAGE_IR_RAW, PAGE_WALLS, PAGE_ENC, PAGE_BNO, PAGE_COUNT };
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
  display.print("CELL:"); display.print((int)CELL_MM); display.print(" FWD:"); display.print((int)CELL_FWD_MM);
  display.setCursor(0,32);
  display.print("Pre:"); display.print(ENC_PRESHIFT); display.print(" T90:"); display.print(ENC_TURN90);
  display.setCursor(0,42);
  display.print("OLED:"); display.print(g_oledOK?"OK":"NO"); display.print(" BNO:"); display.print(g_bnoOK?"OK":"NO"); //
  display.setCursor(0,52);
  display.print("KP:"); display.print(KP_ENC,2); display.print(" KI:"); display.print(KI_ENC,2);
  display.display();
}

void drawPageIRDiff(){
  readIR();
  oledHeader("IR DIFF mV"); //
  display.setCursor(0,12);
  display.print("FL:"); display.print(irFL); display.print("  FR:"); display.print(irFR); //
  display.setCursor(0,24);
  display.print("L :"); display.print(irL);  display.print("  R :"); display.print(irR); //
  display.setCursor(0,38);
  display.print("C(Fusion):"); display.print(irC);
  display.display();
}

void drawPageIRRaw(){
  readIR();
  oledHeader("IR RAW OFF/ON"); //
  display.setCursor(0, 12);
  display.print("FL "); display.print(offFL); display.print("/"); display.print(onFL); //
  display.setCursor(0, 22);
  display.print("FR "); display.print(offFR); display.print("/"); display.print(onFR); //
  display.setCursor(0, 34);
  display.print("L  "); display.print(offL);  display.print("/"); display.print(onL); //
  display.setCursor(0, 44);
  display.print("R  "); display.print(offR);  display.print("/"); display.print(onR); //
  display.display();
}

void drawPageWalls(){
  readIR();
  oledHeader("PAREDES IR"); //
  display.setCursor(0, 12);
  display.print("FL:"); display.print(irFL >= IR_WALL_THR_FL ? "Y" : "N"); //
  display.print(" FR:"); display.print(irFR >= IR_WALL_THR_FR ? "Y" : "N"); //
  display.print(" F:"); display.print(hasWallFront() ? "Y" : "N"); //
  display.setCursor(0, 24);
  display.print("L:"); display.print(hasWallLeft() ? "Y" : "N"); //
  display.print(" R:"); display.print(hasWallRight() ? "Y" : "N"); //
  display.setCursor(0, 38);
  display.print("Thr FL/FR:"); display.print(IR_WALL_THR_FL); display.print("/"); display.print(IR_WALL_THR_FR); //
  display.setCursor(0, 52);
  display.print("Thr L/R:"); display.print(IR_WALL_THR_L); display.print("/"); display.print(IR_WALL_THR_R); //
  display.display();
}

void drawPageEnc(){
  long l,lA,r,rA; getEncAll(l,lA,r,rA);
  oledHeader("ENCODERS"); //
  display.setCursor(0, 12);
  display.print("L: "); display.print(l); display.print(" abs:"); display.print(lA); //
  display.setCursor(0, 26);
  display.print("R: "); display.print(r); display.print(" abs:"); display.print(rA); //
  display.setCursor(0, 42);
  display.print("Mant BOOT: reset"); //
  display.display();
}

void drawPageBNO() {
  oledHeader("BNO085"); //
  if (!g_bnoOK) {
    display.setCursor(0, 24);
    display.println("BNO NO DETECTADO");
    display.display();
    return;
  }
  display.setCursor(0, 12);
  display.print("Yaw:"); display.print(g_yawDeg, 1); display.print(" P:"); display.print(g_pitchDeg, 1); //
  display.setCursor(0, 24);
  display.print("Roll:"); display.print(g_rollDeg, 1); //
  display.setCursor(0, 36);
  display.print("Gz:"); display.print(g_gyroZ, 2); display.print(" rad/s"); //
  display.setCursor(0, 48);
  display.print("Ax:"); display.print(g_linAx, 1); display.print(" Ay:"); display.print(g_linAy, 1); //
  display.display();
} 

// ── SETUP ────────────────────────────────────────────────────
void setup(){
  pinMode(BOOT_PIN, INPUT_PULLUP);
  
  Wire.begin(I2C_SDA, I2C_SCL); 
  Wire.setClock(400000);
  
  g_oledOK = display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR);
  if (g_oledOK) oledShow("UMouse","iniciando...");
  delay(300);

  sensorsInit();
  motionInit();
  initBNO(); 
  initTelemetry();

  float vbat = readVBAT_V();
  g_dutyMax  = calcDutyMax(vbat);
  if(g_dutyMax<20) g_dutyMax=20;

  if(bootDown()){
    oledShow("BOOT: soltar","para borrar NVS");
    delay(1200);
    if(bootDown()){ clearAndSaveWalls();
    oledShow("NVS borrado",""); delay(800); }
    while(bootDown()) delay(10);
  }

  loadWalls();
  oledShow("Listo","");
  delay(300);
}

// ── LOOP ─────────────────────────────────────────────────────
void loop(){
  updateBNO();
  if (g_wifiConnected) webSocket.loop();
  
  static uint32_t tV=0; static float vbat=0.0f;
  if(millis()-tV>500){
    tV=millis(); vbat=readVBAT_V();
    int dm=calcDutyMax(vbat); if(dm<20) dm=20; g_dutyMax=dm;
  }

  if(bootShortPress()){
    g_page=(Page)((g_page+1)%PAGE_COUNT);
  }

  if(g_page==PAGE_ENC && bootLongPress(800)){
    while(bootDown()) delay(10);
    resetEncoders();
    oledShow("Encoders","reseteados"); delay(400);
  }

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

  static uint32_t tUI=0;
  if(millis()-tUI<120) return;
  tUI=millis();
  switch(g_page){
    case PAGE_RUN:     drawPageRun(vbat);    break;
    case PAGE_STATUS:  drawPageStatus(vbat); break;
    case PAGE_IR_DIFF: drawPageIRDiff();     break;
    case PAGE_IR_RAW:  drawPageIRRaw();      break;
    case PAGE_WALLS:   drawPageWalls();      break;
    case PAGE_ENC:     drawPageEnc();        break;
    case PAGE_BNO:     drawPageBNO();        break;
    default: break;
  }
}
