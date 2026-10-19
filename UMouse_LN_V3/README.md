# UMouse S3 — Micromouse

Robot Micromouse con ESP32-S3 que explora un laberinto con FloodFill, guarda el mapa y hace la corrida
rápida. Explora y corre a **120 PWM** (tope del motor) y gira **en arco, sin detenerse**.

## Hardware
| Parte | Detalle |
|---|---|
| MCU | ESP32-S3 (YD-ESP32-S3, WROOM-1), PlatformIO + Arduino core 3.x (pioarduino) |
| Motores | 2 × N20 500 RPM 6 V con encoder de 7 ppr (reducción 30:1), drivers DRV8871 |
| Ruedas | Pololu 32×7 mm (diámetro real 31.5 mm), vía 78 mm |
| IMU | BNO085 (rumbo para giros y rectas) |
| IR | 4 sensores: 2 frontales (FL, FR, 5° hacia fuera) y 2 laterales (L, R, a 95°, 58.7 mm delante del eje) |
| Chasis | 85 × 85 mm, frente redondeado, eje a 68.25 mm del frente (+10 mm de cinta amortiguadora) |
| Otros | OLED SSD1306, LED RGB de estado, LEDs de pared, botón BOOT |
| Laberinto | Paredes de 20 mm, postes 20×20, pasillo útil 140 mm, paso de celda 160 mm |

## Estructura
```
src/
  settings.h   ÚNICO lugar de calibración (perfil, velocidades, IR, arco, frenado…)
  config.h     tamaño del laberinto, inicio, meta y dirección inicial según PROFILE
  motion.h     avance, frenado, giros (en el sitio y en arco), lectura de paredes para FloodFill
  navgeom.h    modelo longitudinal: posición del eje dentro de la celda (se prueba en PC)
  floodfill.h  FloodFill (ida, vuelta y corrida rápida) + previsión de giro
  maus.h       movimiento entre celdas (giro + avance) y confirmación del frente
  sensors.h    IR, batería, OLED          bno.h     BNO085
  netlog.h     log en RAM + página web por WiFi
  main.cpp     menú, páginas de la OLED, NVS (mapa guardado), arranque de la corrida
test/host/     pruebas de lógica en PC (FloodFill con laberintos reales, geometría, LEDs)
UMouse_OG_V0.1/  firmware original de referencia (no se compila)
```

## Compilar y probar
```sh
pio run                 # compilar (o ~/.platformio/penv/bin/pio run)
pio run -t upload       # cargar al robot
sh test/host/run.sh     # pruebas en PC: deben salir 4 × "RESULTADO: OK"
```

## Configurar un laberinto nuevo
En `src/config.h`, dentro del `PROFILE` activo (`src/settings.h`):
- `MAZE_ROWS`, `MAZE_COLS`: tamaño.
- `GOAL_ROW`, `GOAL_COL`: meta. `MOUSE_ROW`, `MOUSE_COL`: inicio.
- `MOUSE_START_DIRECTION`: 1 = Norte, 2 = Este, 3 = Sur, 4 = Oeste.

**Filas y columnas se cuentan desde 0** (en un 3×6: filas 0–2, columnas 0–5). Si la meta o el inicio
quedan fuera del laberinto, el código no compila. Al cambiar de laberinto, **borra el mapa guardado**
(menú del botón, mantener 5 s).

## Cómo navega (`TURN_MODE 2`)
1. **Lectura temprana:** las paredes se leen con el eje 100 mm antes del centro de la celda. Los IR
   laterales ya ven la pared de esa celda. El frontal no alcanza la pared desde ahí (`F=0?` en el log).
2. **Si FloodFill gira:** el robot frena en recta a ~300 mm/s y traza un **arco** (radio ≤ 78 mm, rueda
   exterior más rápida). El BNO corrige el rumbo contra el arco ideal. Sale en marcha al pasillo nuevo.
3. **Si sigue recto:** primero avanza hasta 17.5 mm antes del centro y **confirma el frente**. Si hay
   pared, no avanza y FloodFill vuelve a decidir; ese giro se hace en el sitio.
4. **Media vuelta:** después de girar 180° confirma el frente antes de avanzar (lo de atrás nunca se leyó).
5. **Velocidad:**
   - Explorando, llega a la lectura a ≤ 400 mm/s y a la confirmación a ≤ 300 mm/s: los frontales solo ven
     la pared ~30–40 mm antes.
   - Con el mapa completo va a 120 sin tope y solo frena antes de las curvas que FloodFill prevé.
6. **Re-localización:** si en la lectura temprana aparece una pared frontal fuerte, corrige la posición
   con el nivel IR.
7. **Meta:** al llegar se acerca despacio al centro de la celda. Entre ida, vuelta y corrida rápida
   espera 5 s.

Con `TURN_MODE 1` (giro rodante) o `0` (giro en el sitio, validado a 75 PWM) se vuelve al comportamiento
anterior.

### Modos de navegación (`NAV_MODE` en `settings.h`)
- **3 (por defecto, el único probado con arcos):** BNO085 para el rumbo + encoders + IR. Si el BNO deja
  de responder a mitad de un movimiento, pasa solo a encoders.
- **1:** IR + encoders, sin BNO (la lógica original). Gira en el sitio: el arco necesita el BNO.
- **2:** solo BNO, sin IR ni encoders (avance por tiempo). Únicamente recorre un mapa ya guardado.

Los modos 1 y 2 compilan y se conservan como respaldo para diagnóstico (por ejemplo, si el BNO falla).

## Uso
- **Botón BOOT:**
  - Pulsación corta: página siguiente de la OLED.
  - En la página RUN, mantener 3 s y soltar: inicia la corrida. Seguir hasta 5 s: borra el mapa guardado.
- **Páginas:** RUN, STATUS, IR (calibrar centrado con el robot centrado entre 2 paredes), CAL PIVOTE
  (nivel frontal con el eje en el centro de la celda), ENC, BNO, brújula y DIAG.
- **Log por WiFi:** el robot crea la red `UMouse-S3` (clave `umouse123`). Abre `http://192.168.4.1`:
  log en vivo con botones Copiar todo, Descargar y Borrar.

## Parámetros principales (`src/settings.h`)
| Parámetro | Valor | Qué hace |
|---|---|---|
| `FWD_PWM` | 120 | velocidad en recta |
| `TURN_MODE` | 2 | 2 = arco, 1 = rodante, 0 = en el sitio |
| `ARC_PWM`, `ARC_VIN` | 50, 300 mm/s | velocidad del centro en el arco y pre-freno |
| `ARC_R_MAX_MM` / `ARC_R_MIN_MM` | 78 / 50 | radio del arco |
| `ARC_READ_BACK_MM` | 100 | lectura temprana (no subir: a 124 el IR lateral ve la celda anterior) |
| `ARC_VREAD`, `CONFIRM_VMAX` | 400, 300 mm/s | topes de llegada (lectura temprana / confirmación) |
| `CONFIRM_ONE_THR`, `CONFIRM_BOTH_MIN`, `CONFIRM_SOLO_THR` | 100, 30, 300 | regla ampliada de pared frontal |
| `MOTOR_PWM_CAP` | 120 (en `motion.h`) | tope de PWM (~6 V del motor) |

## Calibración
Antes de cada sesión: batería ≥ 11.5 V. Primero con las ruedas en el aire (página DIAG) y luego en el
suelo. Las calibraciones IR (centrado L/R y pivote) se guardan en NVS y **no** se borran al borrar el mapa.

1. **Paso de celda (recta).** En `settings.h` pon `CAL_STRAIGHT_CELLS 5` (hacen falta 6 celdas rectas
   libres) y compila. Con la parte trasera del robot contra la pared de salida, mantén 3 s en RUN: hace
   el avance inicial y 5 celdas, frena y muestra `odo`. Mide con regla **D** (pared de salida → parte
   trasera del robot) y **L** (paso real, de centro de poste a centro de poste). Entonces
   `CELL_STEP_ADJ_MM = L × odo / D − CELL_FWD_MM`; el log `[CAL-RECTA]` imprime la fórmula con los
   números. Repite 3 veces, promedia y vuelve a `CAL_STRAIGHT_CELLS 0`. No toques `WHEEL_DIAM_MM`,
   `CELL_MM` ni `PRESHIFT_MM`: el paso de encoder actual (157.5 mm por celda) ya absorbe el diámetro real.
2. **Centrado lateral (página IR).** Robot centrado entre dos paredes → mantener pulsado para guardar
   la referencia L/R.
3. **Pivote frontal (página CAL PIVOTE).** Eje de las ruedas en el centro de la celda con pared al
   frente → mantener pulsado. Es el nivel IR en que el robot se detiene para girar en el sitio
   (`[ALIGN] llego`). Si sale `tope-enc`, el pivote está mal calibrado.
4. **Giros.** En `[GIRO-BNO]` el `err` debe quedar en ±3°. En `[ARCO]`, el `desfase max` debe quedar por
   debajo de ~20° y no deben aparecer `FALLO`.
5. **Exploración completa** con el mapa borrado: ida, vuelta e ida, con `[MAPA] guardado` 3 veces.
   Compara el dibujo ASCII con el laberinto real. Después, una corrida con el mapa cargado:
   `[RUN] mapa PREVIO (completo)` y una sola ida por el camino mínimo.

Nota: `TURN_VOLTS` y `FWD_VOLTS` no son voltios reales: `pwmForVolts` los convierte a un PWM (4.5 ≈ 36).

## Cómo leer el log
| Línea | Qué indica |
|---|---|
| `[SCAN] FL FR L R -> F L R ... s=` | lectura de paredes. `F=0?` = lectura temprana, el frontal no alcanza |
| `[CONFIRMA]` | lectura del frente antes de seguir recto o tras una media vuelta (`PARED: vuelve a decidir`) |
| `[ARCO]` | pre-freno, radio, PWM de las ruedas, desfase y resultado (`hecho` / `FALLO`) |
| `[APROX] llegada X (tope Y)` | velocidad al llegar a una lectura con tope |
| `[FRENO-IR]` | el frontal vio la pared durante el avance y frenó por perfil |
| `[HALT] sobrepaso` | cuánto se pasó al frenar |
| `[ALIGN]`, `[PRESHIFT]`, `[GIRO-BNO]` | giro en el sitio: acercamiento por IR, ajuste y error final |
| `[RELOC]` | pared frontal fuerte donde no se esperaba: corrige la posición |
| `[META]` | acercamiento al centro de la celda objetivo |
| `[MAPA] guardado / inalcanzable` | mapa ASCII al llegar a un objetivo o al fallar |
| `[ATASCO]`, `[RUEDA] divergencia`, `[FALLO]` | atasco, rueda bloqueada o choque, parada segura |

Ante un `[FALLO]` el robot se detiene sin actualizar las coordenadas (el mapa guardado queda intacto) y
espera BOOT para reiniciar. El último `[POSE]` y `[FWD]`/`[ATASCO]` indican la causa.

El log (los últimos 16 KB) vive en RAM y **sobrevive a un reinicio** por error, pánico o watchdog. Cada
arranque escribe su motivo (`software`, `PANICO`, `BROWNOUT`…). Se pierde al quitar la alimentación.

## LEDs y mapa guardado
- **LEDs de pared:** azul = frente, blanco = izquierda, rojo = derecha (última lectura). El LED RGB
  muestra el estado (avance, giro, pared, meta, error).
- **Mapa (NVS):** se guarda al llegar a cada objetivo, con meta, inicio y si la exploración está
  completa (ida + vuelta). Al cargar se comprueba que la meta sea alcanzable; si no lo es, la corrida no
  arranca hasta borrar el mapa (mantener 5 s en RUN).

## Mejoras pendientes
1. **Corrida rápida sin lecturas.** Con el mapa completo, el robot todavía lee paredes en cada celda
   (~27 ms) y hace la confirmación del frente. En la corrida rápida podría ir sin leer, encadenar rectas
   largas a fondo y frenar solo antes de cada curva. Es la mayor ganancia de tiempo que queda.
2. **Arcos a la derecha.** Se atrasan más que a la izquierda (desfase de 15–21° contra 8–12° en los logs),
   probablemente por diferencia entre motores. Se puede compensar con una ganancia distinta para cada lado
   (`ARC_K_GAIN` / `ARC_KP` por dirección).
3. **Desvío lateral al salir del arco.** El robot queda 5–11 mm hacia el interior de la curva y la
   odometría marca ~15 mm menos que el arco ideal. Ajustar la geometría del arco reduciría el desvío, y
   los frontales verían la pared de forma más pareja.
4. **Margen de la regla de "un solo frontal fuerte".** La pared lateral vista de reojo llegó a 239 y el
   umbral (`CONFIRM_SOLO_THR`) está en 300. Funciona, pero hay que vigilarlo en laberintos nuevos.
5. **Alcance de los sensores frontales (hardware).** Los IR frontales solo ven la pared a ~3–4 cm. A
   120 PWM (~750 mm/s) el robot necesita ~11 cm para frenar, así que al **explorar** frena a 300 mm/s
   en cada celda donde podría haber pared (con el mapa guardado no lo hace). Con frontales que vean más
   lejos (apuntados más al frente o con más potencia en el LED IR) podría explorar sin esos frenazos.
