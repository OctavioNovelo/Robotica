// Pruebas de FloodFill + Maus (archivos REALES de src/) contra un laberinto simulado.
// El "robot físico" es perfecto: aquí se verifica la LÓGICA (decisiones, paredes, coordenadas,
// NVS), no la física. Compilar con -DMAZE_ROWS=.. -DMAZE_COLS=.. (ver run.sh).
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>
#define WALL_ROWS (2*MAZE_ROWS-1)
#define WALL_COLS (2*MAZE_COLS-1)
inline void delay(unsigned long){}
#include "../../src/maus.h"
#include "../../src/floodfill.h"

static bool T_h[MAZE_ROWS+1][MAZE_COLS];   // pared horizontal sobre la fila r (r=0 y r=ROWS = borde)
static bool T_v[MAZE_ROWS][MAZE_COLS+1];   // pared vertical a la izquierda de la col c
static int  sR, sC, gR, gC;
static int  pr, pc, pdir;                   // pose FÍSICA simulada
static Maus* M;
static bool g_walls[WALL_ROWS][WALL_COLS];
static std::string trace;
static int fails = 0, saves = 0, fwdCount = 0, turnsBeforeFwd = 0;
static bool unreachable = false;
struct Unreach {};

#define CHECK(c, ...) do{ if(!(c)){ fails++; printf("   FALLO: "); printf(__VA_ARGS__); printf("\n"); } }while(0)

static void parse(const char* const* rows){
  memset(T_h,0,sizeof T_h); memset(T_v,0,sizeof T_v);
  for(int r = 0; r <= MAZE_ROWS; r++){
    const char* h = rows[2*r];
    for(int c = 0; c < MAZE_COLS; c++) T_h[r][c] = h[4*c+1] == '-';
    if(r == MAZE_ROWS) break;
    const char* v = rows[2*r+1];
    for(int c = 0; c <= MAZE_COLS; c++) T_v[r][c] = v[4*c] == '|';
    for(int c = 0; c < MAZE_COLS; c++){
      if(v[4*c+2] == 'S'){ sR = r; sC = c; }
      if(v[4*c+2] == 'G'){ gR = r; gC = c; }
    }
  }
}
// pared física en dirección d (1N 2E 3S 4O) desde (r,c)
static bool wallAt(int r, int c, int d){
  switch(d){ case 1: return T_h[r][c]; case 3: return T_h[r+1][c]; case 4: return T_v[r][c]; default: return T_v[r][c+1]; }
}
static int rotL(int d){ return d == 1 ? 4 : d-1; }
static int rotR(int d){ return d == 4 ? 1 : d+1; }

static void checkPose(const char* when){
  CHECK(M->coords[0] == pr && M->coords[1] == pc && M->direction == pdir,
        "%s: pose logica (%d,%d,d%d) != fisica (%d,%d,d%d)", when, M->coords[0], M->coords[1], M->direction, pr, pc, pdir);
}
// frontLate (TURN_MODE 2, peor caso): la lectura temprana NUNCA ve la pared frontal; solo la confirmación
// antes de ir recto (ff_frontClearAhead) la ve. Como en el robot, la pared confirmada queda en la lectura de esa pose.
static bool frontLate = false;
static int  cfR = -1, cfC = -1, cfD = -1, confirms = 0, confirmWalls = 0;
bool ff_hasWallFront(){
  checkPose("scan");
  if(frontLate) return pr == cfR && pc == cfC && pdir == cfD;
  return wallAt(pr,pc,pdir);
}
void ff_hintNext(bool){}
bool ff_frontClearAhead(){
  confirms++;
  if(!wallAt(pr,pc,pdir)) return true;
  confirmWalls++; cfR = pr; cfC = pc; cfD = pdir;
  return false;
}
bool ff_hasWallLeft()  { return wallAt(pr,pc,rotL(pdir)); }
bool ff_hasWallRight() { return wallAt(pr,pc,rotR(pdir)); }
void ff_turnLeft90()  { pdir = rotL(pdir); trace += 'L'; if(!fwdCount) turnsBeforeFwd++; }
void ff_turnRight90() { pdir = rotR(pdir); trace += 'R'; if(!fwdCount) turnsBeforeFwd++; }
// 180° en un giro: misma orientación final que dos giros a la derecha (traza 'RR', como antes).
void ff_turnAround180(){ pdir = rotR(rotR(pdir)); trace += "RR"; if(!fwdCount) turnsBeforeFwd++; }
void ff_moveForwardOneCell(){
  CHECK(!wallAt(pr,pc,pdir), "CHOQUE: avanzar desde (%d,%d) dir %d contra una pared real", pr, pc, pdir);
  if(pdir == 1) pr--; else if(pdir == 3) pr++; else if(pdir == 2) pc++; else pc--;
  CHECK(pr >= 0 && pr < MAZE_ROWS && pc >= 0 && pc < MAZE_COLS, "salio del laberinto");
  trace += 'F'; fwdCount++;
  if(fails > 20) exit(1);
}
void saveWallsToFlash(){ saves++; }
void ff_goalUnreachable(){ unreachable = true; throw Unreach(); }

// Verifica que toda pared del mapa existe en el laberinto real (sin falsos positivos)
static void checkMapSubset(){
  for(int r = 0; r < MAZE_ROWS; r++) for(int c = 0; c < MAZE_COLS; c++){
    if(r+1 < MAZE_ROWS && g_walls[2*r+1][2*c])   CHECK(T_h[r+1][c], "pared falsa sur de (%d,%d)", r, c);
    if(c+1 < MAZE_COLS && g_walls[2*r][2*c+1])   CHECK(T_v[r][c+1], "pared falsa este de (%d,%d)", r, c);
  }
}
static int bfsTrue(){   // longitud del camino minimo real inicio -> meta
  int d[MAZE_ROWS][MAZE_COLS]; memset(d,-1,sizeof d);
  int q[MAZE_ROWS*MAZE_COLS][2], h=0, t=0; q[t][0]=sR; q[t][1]=sC; t++; d[sR][sC]=0;
  while(h<t){ int r=q[h][0], c=q[h][1]; h++;
    for(int k=1;k<=4;k++){ if(wallAt(r,c,k)) continue;
      int nr=r+(k==3)-(k==1), nc=c+(k==2)-(k==4);
      if(d[nr][nc]<0){ d[nr][nc]=d[r][c]+1; q[t][0]=nr; q[t][1]=nc; t++; } } }
  return d[gR][gC];
}

// explored=false: carrera nueva (ida, vuelta, ida). true: carrera con mapa NVS (solo ida).
static void run(bool explored, int startDir){
  Maus m; M = &m; m.coords[0] = pr = sR; m.coords[1] = pc = sC; m.direction = pdir = startDir;
  trace.clear(); saves = 0; fwdCount = 0; turnsBeforeFwd = 0; unreachable = false;
  const int goal[2] = {gR, gC};
  Floodfill f(g_walls, goal, &m, explored);
  try { f.solve(); } catch(Unreach&){}
}

struct Case { const char* name; std::vector<const char*> rows; int dir; char firstTurn; };

int main(){
  printf("FloodFill %dx%d\n", MAZE_ROWS, MAZE_COLS);
  std::vector<Case> cases;
#if MAZE_ROWS == 3 && MAZE_COLS == 3
  // Situaciones locales en la celda de inicio (centro abajo), mirando al norte.
  cases.push_back({"solo pared frontal (sale izq o der, nunca recto)", {
    "+---+---+---+",
    "| G         |",
    "+   +   +   +",
    "|           |",
    "+   +---+   +",
    "|     S     |",
    "+---+---+---+"}, 1, 'T'});
  cases.push_back({"solo pared derecha (sigue recto)", {
    "+---+---+---+",
    "|     G     |",
    "+   +   +   +",
    "|           |",
    "+   +   +   +",
    "|     S |   |",
    "+---+---+---+"}, 1, '-'});
  cases.push_back({"solo pared izquierda (sigue recto)", {
    "+---+---+---+",
    "|     G     |",
    "+   +   +   +",
    "|           |",
    "+   +   +   +",
    "|   | S     |",
    "+---+---+---+"}, 1, '-'});
  cases.push_back({"frente + derecha: unica salida IZQUIERDA", {
    "+---+---+---+",
    "|     G     |",
    "+   +   +   +",
    "|           |",
    "+   +---+   +",
    "|     S |   |",
    "+---+---+---+"}, 1, 'L'});
  cases.push_back({"frente + izquierda: unica salida DERECHA", {
    "+---+---+---+",
    "|     G     |",
    "+   +   +   +",
    "|           |",
    "+   +---+   +",
    "|   | S     |",
    "+---+---+---+"}, 1, 'R'});
  // Arranca mirando a la pared con OTRA pared detrás (nunca leída): la media vuelta no puede avanzar a ciegas.
  cases.push_back({"arranque mirando a pared con pared detras (2026-10-17)", {
    "+---+---+---+",
    "|     G     |",
    "+   +   +   +",
    "|           |",
    "+   +---+   +",
    "|     S     |",
    "+---+---+---+"}, 3, '?'});
  cases.push_back({"callejon sin salida (media vuelta)", {
    "+---+---+---+",
    "|   |   | G |",
    "+   +   +   +",
    "|           |",
    "+---+   +---+",
    "|   | S |   |",
    "+---+---+---+"}, 3, 'R'});
#elif MAZE_ROWS == 3 && MAZE_COLS == 6
  // Configuración real del proyecto: inicio (2,5) al norte, meta (0,0).
  cases.push_back({"3x6 pasillo largo + giros", {
    "+---+---+---+---+---+---+",
    "| G                     |",
    "+---+---+---+---+---+   +",
    "|                   |   |",
    "+   +---+---+---+   +   +",
    "|                   | S |",
    "+---+---+---+---+---+---+"}, 1, '-'});
  cases.push_back({"3x6 serpiente con callejones", {
    "+---+---+---+---+---+---+",
    "| G     |               |",
    "+---+   +   +---+---+   +",
    "|       |   |       |   |",
    "+   +---+   +   +   +   +",
    "|           |   |     S |",
    "+---+---+---+---+---+---+"}, 1, '?'});
  // Laberinto FÍSICO del usuario (foto, girada a coordenadas del código; verificada contra
  // cada [SCAN] del log real): inicio (2,2) al norte, meta (2,5). Bordes abiertos en (2,0).
  cases.push_back({"3x6 laberinto fisico (2,2)->(2,5)", {
    "+---+---+---+---+---+---+",
    "|                       |",
    "+   +   +   +---+   +   +",
    "|   |   |   |   |   |   |",
    "+---+   +---+   +   +   +",
    "    |     S     |   | G |",
    "+   +---+---+---+---+---+"}, 1, '?'});
  // 2.º laberinto físico (foto 2): inicio (2,5) al norte, meta (0,0). El robot real marcó una
  // pared FALSA al oeste de (1,3) (poste de la celda siguiente, R=213) => "meta inalcanzable".
  cases.push_back({"3x6 laberinto fisico 2 (2,5)->(0,0)", {
    "+---+---+---+---+---+---+",
    "| G |       |           |",
    "+   +   +   +   +   +   +",
    "|       |       |   |   |",
    "+   +---+   +   +   +   +",
    "|           |       | S |",
    "+---+---+---+---+---+---+"}, 1, '?'});
#endif
  for(int late = 0; late < 2; late++)
  for(auto& k : cases){
    frontLate = late; confirms = confirmWalls = 0; cfR = -1;
    printf(" - %s%s\n", k.name, late ? "  [TURN_MODE 2: frontal solo al confirmar]" : "");
    parse(k.rows.data());
    memset(g_walls, 0, sizeof g_walls);
    run(false, k.dir);
    printf("   carrera nueva: %s  (guardados=%d, confirmaciones=%d, con pared=%d)\n", trace.c_str(), saves, confirms, confirmWalls);
    CHECK(!unreachable, "meta declarada inalcanzable en un laberinto resoluble");
    CHECK(saves == 3, "se esperaban 3 guardados (ida, vuelta, ida); hubo %d", saves);
    CHECK(pr == gR && pc == gC, "no termino en la meta");
    if(k.firstTurn == 'L') CHECK(trace[0] == 'L', "debia girar a la IZQUIERDA primero");
    if(k.firstTurn == 'R') CHECK(trace[0] == 'R', "debia girar a la DERECHA primero");
    if(k.firstTurn == 'T') CHECK(trace[0] != 'F', "con pared frontal no puede avanzar recto");
    if(k.firstTurn == '-') CHECK(trace[0] == 'F', "debia avanzar recto primero");
    if(k.firstTurn == 'L') CHECK(trace.find('R') == std::string::npos || trace.find('R') > trace.find('F'),
                                 "ordeno girar a la derecha (pared) antes de salir");
    checkMapSubset();
    // Segunda carrera con el mapa "NVS" (misma matriz): debe ir directo por el camino minimo
    bool saved[WALL_ROWS][WALL_COLS]; memcpy(saved, g_walls, sizeof saved);
    run(true, k.dir);
    printf("   carrera NVS:   %s  (avances=%d, minimo real=%d)\n", trace.c_str(), fwdCount, bfsTrue());
    CHECK(saves == 1, "con mapa previo debe guardar 1 vez (solo ida); hubo %d", saves);
    CHECK(fwdCount == bfsTrue(), "la carrera con NVS no siguio el camino minimo");
  }
#if MAZE_ROWS == 3 && MAZE_COLS == 3
  {
    printf(" - mapa con pared FALSA que encierra la meta (antes: bucle infinito)\n");
    const char* rows[] = {
      "+---+---+---+", "|     G     |", "+   +   +   +", "|           |", "+   +   +   +", "|     S     |", "+---+---+---+"};
    parse(rows);
    memset(g_walls, 0, sizeof g_walls);
    g_walls[1][2] = true; g_walls[0][1] = true; g_walls[0][3] = true;   // falsas alrededor de G (0,1)
    run(true, 1);
    CHECK(unreachable, "debia llamar ff_goalUnreachable()");
    printf("   ff_goalUnreachable() llamado: %s\n", unreachable ? "si" : "NO");
  }
#endif
  printf(fails ? "RESULTADO: %d FALLOS\n" : "RESULTADO: OK\n", fails);
  return fails ? 1 : 0;
}
