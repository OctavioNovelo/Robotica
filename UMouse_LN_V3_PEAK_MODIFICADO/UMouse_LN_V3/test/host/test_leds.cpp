// LEDs físicos de pared independientes del RGB de estado (src/status.h real, Arduino simulado).
#include <cstdio>
#include "Arduino.h"
int g_pinLevel[64], g_pinWrites[64]; uint8_t g_rgb[3];
#include "../../src/status.h"
static int fails = 0;
#define CHECK(c, ...) do{ if(!(c)){ fails++; printf("   FALLO: "); printf(__VA_ARGS__); printf("\n"); } }while(0)
int main(){
  statusInit();
  printf(" - 8 combinaciones frente/izquierda/derecha\n");
  for(int m = 0; m < 8; m++){
    bool f = m & 1, l = m & 2, r = m & 4;
    wallLedsSet(f, l, r);
    CHECK(g_pinLevel[PIN_LED_BLUE] == f && g_pinLevel[PIN_LED_WHITE] == l && g_pinLevel[PIN_LED_RED] == r,
          "F%d L%d R%d -> azul%d blanco%d rojo%d", f, l, r,
          g_pinLevel[PIN_LED_BLUE], g_pinLevel[PIN_LED_WHITE], g_pinLevel[PIN_LED_RED]);
    printf("   F=%d L=%d R=%d -> azul(47)=%d blanco(16)=%d rojo(21)=%d\n", f, l, r,
           g_pinLevel[PIN_LED_BLUE], g_pinLevel[PIN_LED_WHITE], g_pinLevel[PIN_LED_RED]);
  }
  printf(" - setStatus() no toca los LEDs de pared y cada estado tiene color propio\n");
  wallLedsSet(true, false, true);
  int w0 = g_pinWrites[PIN_LED_BLUE] + g_pinWrites[PIN_LED_WHITE] + g_pinWrites[PIN_LED_RED];
  unsigned col[ST_GOAL+1];
  for(int s = ST_OFF; s <= ST_GOAL; s++){
    g_argbLast = 0xFFFFFFFFu; setStatus((StatusId)s);
    col[s] = (g_rgb[0] << 16) | (g_rgb[1] << 8) | g_rgb[2];
  }
  int w1 = g_pinWrites[PIN_LED_BLUE] + g_pinWrites[PIN_LED_WHITE] + g_pinWrites[PIN_LED_RED];
  CHECK(w0 == w1, "setStatus escribio en los LEDs fisicos");
  CHECK(g_pinLevel[PIN_LED_BLUE] == 1 && g_pinLevel[PIN_LED_WHITE] == 0 && g_pinLevel[PIN_LED_RED] == 1, "LEDs alterados");
  for(int a = ST_BOOT; a <= ST_GOAL; a++) for(int b = a+1; b <= ST_GOAL; b++)
    CHECK(col[a] != col[b], "estados %d y %d con el mismo color", a, b);
  CHECK(col[ST_FLOOD] != col[ST_BNO_INIT], "FLOOD = BNO_INIT");
  printf(fails ? "RESULTADO: %d FALLOS\n" : "RESULTADO: OK\n", fails);
  return fails ? 1 : 0;
}
