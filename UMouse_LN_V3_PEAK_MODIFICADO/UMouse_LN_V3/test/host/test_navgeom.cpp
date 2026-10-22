// Escenarios A-H del modelo longitudinal (src/navgeom.h) con los valores reales de settings.h.
#include <cstdio>
#include <cmath>
#include "../../src/navgeom.h"
static int fails = 0;
static void eq(const char* what, float got, float want){
  bool ok = fabsf(got - want) < 0.01f;
  if(!ok) fails++;
  printf("   %-46s %8.2f  (esperado %8.2f) %s\n", what, got, want, ok ? "" : "<-- FALLO");
}
static void scenarios(float C, float P, float T){
  printf("\n== C=%.1f P=%.1f T=%.1f  (punto de lectura s=%.1f) ==\n", C, P, T, T - P);
  NavGeom g; g.init(C, P, T);
  printf(" A. recto 1 celda\n");   eq("avance", g.fwdDist(), C); g.arrivedRead(); eq("s en lectura", g.s, T-P);
  printf(" B. recto 5 celdas (crucero, objetivo absoluto)\n");
  float tot = 0; for(int i = 0; i < 5; i++){ tot += g.fwdDist(); g.arrivedRead(); }
  eq("acumulado", tot, 5*C);
  printf(" C. avance + giro derecha\n");
  g.init(C,P,T); g.arrivedRead();
  eq("preshift", g.preshiftDist(), P); g.advance(g.preshiftDist()); g.setPivot(); eq("s en pivote", g.s, T);
  g.afterTurn(+1); eq("s tras giro", g.s, 0); eq("avance tras giro", g.fwdDist(), C - P + T);
  g.arrivedRead(); eq("s en lectura nueva celda", g.s, T-P);
  printf(" D. avance + giro izquierda (con sobrepaso de frenado 4 mm)\n");
  g.init(C,P,T); g.advance(4.0f);
  eq("preshift descuenta sobrepaso", g.preshiftDist(), P > 4 ? P - 4 : 0);
  g.advance(g.preshiftDist()); g.setPivot(); g.afterTurn(-1); eq("avance tras giro", g.fwdDist(), C - P + T);
  printf(" E. giro con pared frontal (IR deja el eje en el pivote)\n");
  g.init(C,P,T); g.s = g.T; g.setPivot(); eq("preshift extra", g.preshiftDist(), 0); g.afterTurn(+1);
  eq("avance tras giro", g.fwdDist(), C - P + T);
  printf(" E2. IR se detiene corto (tope/timeout) tras 10 mm: encoder completa\n");
  g.init(C,P,T); g.advance(10.0f); eq("resto por encoder", g.preshiftDist(), (P - 10 > 0) ? P - 10 : 0);
  printf(" F. giro sin pared frontal (encoder)\n");
  g.init(C,P,T); float d = g.preshiftDist(); g.advance(d); g.setPivot(); g.afterTurn(+1);
  eq("preshift", d, P); eq("avance tras giro", g.fwdDist(), C - P + T);
  printf(" G. regreso por el mismo pasillo (180 = dos giros, 1 solo preshift)\n");
  g.init(C,P,T); g.advance(g.preshiftDist()); g.setPivot(); g.afterTurn(+1);
  eq("preshift del 2.o giro", g.preshiftDist(), 0); g.setPivot(); g.afterTurn(+1);
  eq("s tras 180", g.s, -T); eq("avance de regreso", g.fwdDist(), C - P + 2*T);
  printf(" H. atasco aceptado a 3 mm del objetivo + retroceso 18.8 mm\n");
  g.init(C,P,T); g.arrivedRead(); g.advance(-(3.0f + 18.8f));
  eq("avance siguiente compensa", g.fwdDist(), C + 21.8f);
  eq("preshift si gira ahi", g.preshiftDist(), P + 21.8f);
}
int main(){
  const float C = 160.0f - 17.5f;   // CELL_FWD_MM
  scenarios(C, 17.5f, 0.0f);        // PRESHIFT_ENABLE 1, T = 0 (configuración entregada)
  scenarios(C, 0.0f, 0.0f);         // PRESHIFT_ENABLE 0 (comportamiento anterior)
  scenarios(C, 17.5f, 5.0f);        // ejemplo con punto de giro desplazado
  printf(fails ? "\nRESULTADO: %d FALLOS\n" : "\nRESULTADO: OK\n", fails);
  return fails ? 1 : 0;
}
