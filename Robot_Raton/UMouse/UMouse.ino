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
#include "motion_gyro.h"   // gyro (BNO085) implementado, pero aún NO conectado a floodfill.h ni maus.h
#include "diagnostics.h"   // escaneo I2C + secuencia de prueba de motores (páginas PAGE_BNO / PAGE_MOTOR)
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

// Lectura de BOOT con anti-rebote simple: el estado solo se considera
// "cambiado" cuando la señal se mantiene estable por DEBOUNCE_MS.
// Evita que un rebote mecánico del botón dispare parpadeo/falsos toggles
// en la lógica de hold de PAGE_RUN.
bool bootStable(){
  static bool stable=false;
  static bool lastRaw=false;
  static uint32_t tChange=0;
  const uint32_t DEBOUNCE_MS = 20;

  bool raw = bootDown();
  if(raw != lastRaw){ lastRaw = raw; tChange = millis(); }
  if(millis()-tChange >= DEBOUNCE_MS) stable = raw;
  return stable;
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
enum Page : uint8_t { PAGE_RUN=0, PAGE_STATUS, PAGE_IR, PAGE_ENC, PAGE_BNO, PAGE_MOTOR, PAGE_COUNT };
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
  display.println("SHORT: sig pagina");
  display.display();
}

// Pantalla mostrada mientras se mantiene BOOT presionado en PAGE_RUN.
// Muestra los DOS umbrales al mismo tiempo (no se alternan), junto con
// el tiempo transcurrido, para que se pueda leer todo de un vistazo.
void drawPageRunHold(uint32_t heldMs){
  oledHeader("INICIAR / BORRAR");

  display.setCursor(0,22);
  display.println("SOLTAR = Floodfill");
  display.setCursor(0,42);
  display.println("MANTENER = Borrar NVS");
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
  display.print("WL:"); display.print(hasWallLeft()?"Y":"N");
  display.print(" WF:"); display.print(hasWallFront()?"Y":"N");
  display.print(" WR:"); display.print(hasWallRight()?"Y":"N");
  display.setCursor(0,46);
  display.print("TFL:"); display.print(IR_WALL_THR_FL);
  display.print(" TFR:"); display.print(IR_WALL_THR_FR);
  display.setCursor(0,56);
  display.print("TL:"); display.print(IR_WALL_THR_L);
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
  gyroUpdate();
  oledHeader("BNO085");
  display.setCursor(0,12);
  display.print("Addr:");
  if(g_gyroReady) display.printf("0x%02X", g_bnoAddr); else display.print("--");
  display.print(" OK:"); display.print(g_gyroReady ? "Y" : "N");

  display.setCursor(0,22);
  display.print("Yaw:"); display.print(g_yawDeg,1);
  display.print(" P:"); display.print(g_pitchDeg,1);

  display.setCursor(0,32);
  display.print("Roll:"); display.print(g_rollDeg,1);

  display.setCursor(0,42);
  display.print("Gz:"); display.print(g_gyroZ,2); display.print(" rad/s");

  display.setCursor(0,52);
  display.print("Ax:"); display.print(g_linAx,1);
  display.print(" Ay:"); display.print(g_linAy,1);
  display.display();
}

void drawPageMotor(){
  float vbat = readVBAT_V();
  int testPWM = calcMotorTestPWM();

  oledHeader("MOTORES");
  display.setCursor(0,12);
  display.print("VB:"); display.print(vbat,1);
  display.print(" Max:"); display.print(g_dutyMax);

  display.setCursor(0,22);
  display.print("Test:"); display.print(testPWM);
  display.print(" "); display.print(g_motorTestPercent); display.print("%");

  display.setCursor(0,32);
  display.print("Modo:"); display.print(g_motorState);

  display.setCursor(0,42);
  display.print("Cmd L:"); display.print(g_motorCmdL);
  display.print(" R:"); display.print(g_motorCmdR);

  display.setCursor(0,52);
  display.print("HOLD: secuencia");
  display.display();
}

// Llamada en vivo desde motorTimedTest()/motorFullSequenceTest() (diagnostics.h)
// mientras corre la prueba, para refrescar el progreso en pantalla.
void drawMotorTestStage(const char *stage, int pwm, uint32_t elapsedMs, uint32_t totalMs){
  long l,lA,r,rA; getEncAll(l,lA,r,rA);

  oledHeader("TEST MOTORES");
  display.setCursor(0,12);
  display.print(stage);

  display.setCursor(0,24);
  display.print("PWM:"); display.print(pwm);
  display.print(" Max:"); display.print(g_dutyMax);

  display.setCursor(0,36);
  display.print("L:"); display.print(lA);
  display.print(" R:"); display.print(rA);

  display.setCursor(0,48);
  if(totalMs > 0){
    uint8_t pct = (uint8_t)ci((int)((elapsedMs * 100UL) / totalMs), 0, 100);
    display.print("Progreso: "); display.print(pct); display.print("%");
  } else {
    display.print("Mantente listo");
  }
  display.display();
}

void drawMotorTestResult(){
  oledHeader("RESULTADO MOTOR");
  display.setCursor(0,12);
  display.print("Ticks L:"); display.print(g_lastMotorTicksL);
  display.setCursor(0,24);
  display.print("Ticks R:"); display.print(g_lastMotorTicksR);
  display.setCursor(0,36);
  display.print("RPM L:"); display.print(g_lastRpmL,0);
  display.print(" R:"); display.print(g_lastRpmR,0);
  display.setCursor(0,52);
  display.print("SHORT: salir");
  display.display();
}

// ── SETUP ────────────────────────────────────────────────────
void setup(){
  Serial.begin(115200);
  delay(300);
  Serial.println("\nArrancando UMouse ESP32-S3...");

  pinMode(BOOT_PIN, INPUT_PULLUP);
  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(400000);
  display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR);
  oledShow("UMouse","iniciando...");
  delay(300);

  sensorsInit();
  motionInit();

  // Escaneo I2C de diagnóstico — confirma por Serial que OLED (0x3C) y
  // BNO085 (0x4A/0x4B) responden antes de intentar inicializarlos.
  scanI2C(true);

  // BNO085 — solo se toca si NAV_MODE lo requiere (ver settings.h). En
  // NAV_MODE==1 (solo encoders) NI SIQUIERA SE LLAMA gyroInit() — ni se
  // toca el pin de reset — para aislar por completo si el BNO es la
  // causa de reinicios/fallas del sistema.
  if(NAV_USE_GYRO){
    gyroInit();
  } else {
    oledShow("BNO085","desactivado");
    delay(600);
  }

  float vbat = readVBAT_V();
  g_dutyMax  = calcDutyMax(vbat);
  if(g_dutyMax<20) g_dutyMax=20;


  loadWalls();
  oledShow("Listo","");
  delay(300);
}

// ── LOOP ─────────────────────────────────────────────────────
// ── LOOP ─────────────────────────────────────────────────────
void loop(){
  // VBAT cada 500ms
  static uint32_t tV=0; static float vbat=0.0f;
  if(millis()-tV>500){
    tV=millis(); vbat=readVBAT_V();
    int dm=calcDutyMax(vbat); if(dm<20) dm=20; g_dutyMax=dm;
  }

  // Gyro: mantener g_yawDeg fresco si el BNO está listo, y vigilar que no
  // se quede "zombie" — todo esto solo corre si NAV_MODE lo requiere
  // (ver settings.h). En NAV_MODE==1 esta rama nunca se ejecuta y el
  // BNO queda completamente sin tocar durante todo el funcionamiento.
  static uint32_t tGyro=0;
  if(NAV_USE_GYRO && millis()-tGyro>100){
    tGyro=millis();
    if(g_gyroReady) gyroUpdate();
    gyroWatchdog();
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

  // Acción PAGE_MOTOR: long press = secuencia completa de prueba
  // (coast, adelante, freno, reversa, freno, coast), luego muestra
  // ticks/RPM medidos y espera un toque corto para volver a la página.
  if(g_page==PAGE_MOTOR && bootLongPress(1200)){
    while(bootDown()) delay(10);
    motorFullSequenceTest();
    drawMotorTestResult();
    delay(1200);
  }

  // Acción PAGE_RUN: 
  // - Si se mantiene >= 3s: borra NVS de inmediato y espera a que se suelte.
  // - Si se soltó entre 1.5s y 3s: inicia el floodfill.
  static bool holdDown = false;
  static uint32_t tHoldStart = 0;
  bool bootHeldNow = false;
  uint32_t heldMs = 0;

  // CORRECCIÓN CLAVE: Usar bootStable() para saber si está presionado, no bootShortPress
  bool down = bootStable();

  if(g_page != PAGE_RUN){
    holdDown = false;
  } else {
    if(down && !holdDown) tHoldStart = millis();
    bootHeldNow = down;
    heldMs = down ? (millis() - tHoldStart) : 0;

    // 1. Borrado NVS: se activa en cuanto llegas a 3000 ms (3s) presionado
    if(down && heldMs >= 3000){
      clearAndSaveWalls();
      oledShow("NVS borrado","mapa nuevo");
      delay(800); 
      
      while(bootDown()) delay(10); // Espera activa a que sueltes el botón
      
      // Reseteamos el estado para que el Floodfill no se dispare al soltar
      holdDown = false; 
      down = false;     
      bootHeldNow = false;
    }
    // 2. Floodfill: se activa solo si sueltas el botón entre 1.5s y 3s
    else if(!down && holdDown){
      uint32_t held = millis() - tHoldStart;

      if(held >= 1500 && held < 3000){
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
        ledBlink(ledBlanco, 3);
        oledShow("Completado!","SHORT: UI");
        while(!bootShortPress()) delay(50);
      }
    }

    holdDown = down;
  }

// Refresco UI 120ms — Muestra la pantalla correspondiente
  static uint32_t tUI=0;
  if(millis()-tUI<120) return;
  tUI=millis();

  // CORRECCIÓN: Solo mostramos la pantalla de "Mantener" si el botón 
  // lleva presionado más de 250ms. Esto evita el parpadeo con toques rápidos.
  if(g_page==PAGE_RUN && bootHeldNow && heldMs > 250){
    drawPageRunHold(heldMs); // Muestra el mensaje de INICIAR / BORRAR
  } else {
    switch(g_page){
      case PAGE_RUN:    drawPageRun(vbat);    break;
      case PAGE_STATUS: drawPageStatus(vbat); break;
      case PAGE_IR:     drawPageIR();         break;
      case PAGE_ENC:    drawPageEnc();        break;
      case PAGE_BNO:    drawPageBNO();        break;
      case PAGE_MOTOR:  drawPageMotor();      break;
      default: break;
    }
  }
}