# UMouse S3: cambios, validación y calibración

Leyenda: **[CONFIRMADO]** es un error demostrado en el código. **[HIPÓTESIS]** es un comportamiento físico probable. **[MEDIR]** indica que el dato requiere medición en el robot. **[PROBADO-PC]** significa verificado con pruebas de lógica en el PC. **[PENDIENTE-ROBOT]** significa que todavía requiere una prueba física.

## A. Archivos

| Archivo | Estado |
|---|---|
| `src/navgeom.h` | **Nuevo.** Modelo longitudinal único (avance / lectura / preshift / giro). Sin Arduino, así que se puede probar en el PC. |
| `src/motion.h` | Avance con resultado y recuperación medida, preshift restaurado, alineación con tope, giros verificados, lecturas de pared con instantánea y LEDs, y registro Serial. |
| `src/settings.h` | Constantes nuevas: `PRESHIFT_ENABLE`, `READ_BACK_MM`, `TURN_POINT_OFFSET_MM`, `SCAN_LEAD_MM`, `ALIGN_MAX_EXTRA_MM`, `IR_WALL_CONFIRM`, seguridad (`FWD_TIMEOUT_MS`, `STALL_*`, `TURN_FAIL_DEG`) y `NAV_DEBUG`. Se eliminó `TURN_ADVANCE_ADJ_MM`. `CELL_MM` y `PRESHIFT_MM` se toman tal cual del perfil. `CELL_STEP_ADJ_MM` y `CAL_STRAIGHT_CELLS` (corrección y prueba del paso de celda). |
| `src/status.h` | LEDs físicos dedicados a paredes. `setStatus()` ya no los toca. `ST_FLOOD` tiene un color propio. |
| `src/sensors.h` | Instantánea `g_irSnap`, compartida entre la decisión de paredes y los LEDs. |
| `src/main.cpp` | Metadatos NVS `meta`, validación del mapa (BFS propio), volcado ASCII del mapa, `navLogPose`, `ff_goalUnreachable`, LEDs en vivo en las páginas IR/PIVOTE, meta parpadeando y textos de las páginas STATUS/PIVOTE. |
| `src/floodfill.h` | **Solo los 2 cambios mínimos aprobados**: `flush()` antes de `flood()` en `solve()` y el gancho `ff_goalUnreachable()` cuando la meta es inalcanzable. No cambian el BFS, el orden de vecinos ni la conversión cardinal. |
| `src/maus.h`, `pins.h`, `bno.h`, `config.h`, `platformio.ini` | **Sin cambios** (`maus.h` sigue siendo idéntico al histórico). |
| `test/host/` | Pruebas de lógica en el PC: `sh test/host/run.sh`. |

## B/C. Qué cambió y por qué (antes → después)

### Paso de celda [CORREGIDO tras la prueba en pasillo largo]
El avance recto es `CELL_MM − PRESHIFT_MM`. El 160 histórico (`UMouse/`) se calibró **frenando en cada celda**: el deslizamiento del frenado no se contaba (los encoders se reiniciaban en la celda siguiente) y quedaba incluido en ese valor sin verse. En crucero no hay frenado entre celdas, así que en un pasillo largo ese extra se pierde y el robot se queda corto (≈ media celda tras 5). La afirmación anterior de que "142.5 es el paso real" era una inferencia y no se sostiene.
Corrección: `CELL_STEP_ADJ_MM` (por defecto 0 = valores del perfil, sin cambios), que se mide con la prueba de recta `CAL_STRAIGHT_CELLS` (paso 1 de la calibración). `CELL_MM` y `PRESHIFT_MM` se toman tal cual de `settings.h`.

### Preshift (modelo de `navgeom.h`)
Sea `s` la posición del eje respecto al centro de la celda, con C = 142.5, P = `READ_BACK_MM` y T = `TURN_POINT_OFFSET_MM`.

| Maniobra | Antes | Después |
|---|---|---|
| Recto tras recto | 142.5 | 142.5 (sin cambio) |
| Antes de girar, sin pared | 0 mm (preshift anulado) | `T − s` = **17.5 mm** menos el sobrepaso de frenado medido |
| Antes de girar, con pared | IR hasta `g_irPivot` sin tope; si fallaba, posición desconocida | IR hasta `g_irPivot` **con tope por encoder** (previsto + 25 mm). Si no llega, el encoder completa solo lo que falta (nunca dos posicionamientos). |
| Tras girar | 142.5, y la lectura quedaba 17.5 mm adelantada | **125 mm** (`C − P + T`): la lectura vuelve al mismo punto relativo de la celda |
| 2.º giro de un 180° | Repetía alineación y avance contra la pared lateral | Sin preshift ni alineación (ya está en el punto de giro) |
| Lectura de paredes | Dos convenciones mezcladas (eje en el centro tras alinear; offset inicial tras girar sin pared) | Siempre con el eje en `T − P` (17.5 mm antes del punto de giro) |
| `g_skipPreshift` | Restaba 17.5 mm tras un atasco sin base | Eliminado; lo sustituye el estado `s` |

`PRESHIFT_ENABLE 0` reproduce el esquema anterior (P = 0: lectura en el punto de giro, sin avance previo, 142.5 mm tras girar) y conserva el resto de correcciones. Sirve para comparar.

El histórico (`UMouse/motion.h`) hacía empuje ciego, luego el preshift chocaba contra la pared, luego el detector de atasco retrocedía 18.8 mm, y después avanzaba 125 mm. Sin pared avanzaba 17.5 mm y luego 142.5 mm, así que su punto de lectura también se desplazaba. El modelo nuevo formaliza su intención de forma coherente.

### Avance continuo
- Se conserva. El objetivo sigue siendo absoluto desde el último frenado, pero ahora se acumula en **mm** y se redondea una sola vez: se elimina el truncado de unos 0.7 ticks por celda. [CONFIRMADO]
- Al frenar desde crucero, el sobrepaso (lectura en marcha + frenado) se **mide** y se descuenta del preshift (log `[HALT]`).
- `SCAN_LEAD_MM` (por defecto 0) adelanta el fin del tramo para centrar la lectura, que dura unos 36 ms, sin introducir deriva.

### Lecturas IR
- Se usan los mismos umbrales y las mismas funciones. La corrección principal es el **punto longitudinal de lectura**. [HIPÓTESIS: 5° a ~50 mm desplazan la mancha solo ~4 mm; la posición longitudinal pesa mucho más. Ver calibración 3.]
- `IR_WALL_CONFIRM 1` (por defecto 0) activa la validación por ventana: descarta una pared lateral si las lecturas rápidas de los últimos 20 mm no la respaldan, porque un poste es corto y una pared es continua. Solo descarta positivos y reutiliza las lecturas del centrado, sin coste extra.
- El centrado lateral nunca escribe paredes; esto no cambió.

### Atascos y coordenadas
| | Antes | Después |
|---|---|---|
| Timeout de avance | Retornaba en silencio y `Maus` contaba la celda | `navFailStop()`: el robot se detiene con un diagnóstico, las coords **no** avanzan y el mapa NVS queda intacto |
| Atasco | Retrocedía 75 ticks con `resetEncoders()`, la posición se perdía, retornaba y la celda se daba por completada | Retroceso **medido** (contadores abs: posición = abs − 2·retroceso, sin depender de `ENC_SIGN_*`), seguido de **1 reintento al mismo objetivo**. Si faltaban ≤ 8 mm, se acepta como llegada con la posición real medida. Si no, `navFailStop()`. |
| Giro con timeout | Se ignoraba | Se **verifica** el resultado: un timeout por encoder o un error final de BNO > 25° detienen el robot. El control angular **no cambió**. |
| Meta inalcanzable (pared falsa) | Bucle infinito sin diagnóstico | `ff_goalUnreachable()` vuelca el mapa y detiene el robot |

Limitación física [HIPÓTESIS]: si las ruedas patinan sin bloquearse, el encoder sigue contando. Esto no se puede detectar solo con encoders.

### LEDs
- Los LEDs físicos son **solo de paredes**: azul (47) = frente, **blanco (16) = izquierda**, **rojo (21) = derecha**. Funcionan de forma independiente y reflejan la última lectura vigente: se apagan al empezar un giro o un avance y tras un fallo. En las páginas IR/PIVOTE se actualizan en vivo; en el resto de páginas están apagados. Usan `digitalWrite`, sin LEDC.
- El RGB conserva todos los estados y valores. Solo cambia `ST_FLOOD` (antes era idéntico a `ST_BNO_INIT`), y `ST_GOAL` parpadea en la pantalla final.

### NVS
- `walls` mantiene el mismo formato. Se añade la clave `meta` (versión, tamaño, meta, inicio y `complete` = ida + vuelta exploradas).
- Un mapa sin `meta` (versión anterior) **se sigue cargando** y se marca "legado". No se borra nada automáticamente.
- Al cargar se comprueba con un BFS propio que la meta sea alcanzable. Si no lo es, se muestra "Mapa NVS inválido" y la corrida no arranca hasta que se borre el mapa (HOLD 5 s).
- NAV_MODE 2 no arranca con un mapa parcial, porque supondría abiertas celdas no exploradas.
- Se verificó que la carrera nueva hace ida → guarda → vuelta → guarda → ida → guarda, y que la carrera con NVS hace **solo la ida**, por el camino mínimo.
- `floodfill.h`: la 3.ª fase usaba distancias viejas sin `flush()`. Esto quedó corregido. [CONFIRMADO]

### Diagnóstico Serial (`NAV_DEBUG 1`, 115200 baud, búfer TX de 1024)
`[RUN]` mapa nuevo/previo · `[GEOM]` constantes · `[POSE]` celda/dirección lógica · `[MOV]` decisión · `[FWD]` objetivo/ticks/tiempo · `[HALT]` sobrepaso · `[ALIGN]` resultado, mm recorridos y FL/FR · `[PRESHIFT]` · `[GIRO-BNO]` error final · `[SCAN]` FL/FR/L/R → F/L/R, ventana, duración y mm recorridos durante la lectura · `[ATASCO]` · `[FALLO]` · `[MAPA]` dibujo ASCII · `[NVS]`. Nada se imprime dentro del lazo de 10 ms.

## D. Validación realizada

- **Compilación real** con PlatformIO (pioarduino, Arduino-ESP32 3.3.12) `[SUCCESS]` en la configuración entregada y en 5 variantes: NAV_MODE 1, NAV_MODE 2, `PRESHIFT_ENABLE 0`, `IR_WALL_CONFIRM 1` y `NAV_DEBUG 0`. Los 6 avisos que aparecen ya estaban en el original. Uso: RAM 8.5 %, Flash 11.9 %.
- **Pruebas en el PC** (`sh test/host/run.sh`, todas OK) [PROBADO-PC]:
  - FloodFill y `maus.h` reales contra un laberinto simulado con robot perfecto: solo pared frontal (nunca avanza recto), solo derecha, solo izquierda, frente + derecha (**primer movimiento = izquierda, nunca derecha**), frente + izquierda, callejón (media vuelta), pasillo largo 3×6 y serpiente con callejones 3×6. En todos los casos se comprobó: la pose lógica coincide con la física en cada lectura, no hay choques, no hay paredes falsas en el mapa, hay 3 guardados en la carrera nueva, y la carrera con NVS hace 1 guardado y sigue **exactamente el camino mínimo**. También se comprobó que una pared falsa que encierra la meta llama a `ff_goalUnreachable()`.
  - `navgeom.h`: escenarios A–H con P = 17.5/T = 0, P = 0 y T = 5.
  - LEDs: las 8 combinaciones son independientes, `setStatus()` no escribe los LEDs y todos los estados RGB tienen valores distintos.
- **No verificable sin robot** [PENDIENTE-ROBOT]: distancias reales, sobrepaso, punto de lectura, `g_irPivot` en el nuevo punto, detección de atasco/patinaje, verificación de giros con el BNO, tiempos de lectura en crucero y si los colores verdes (`BNO_OK`/`STRAIGHT`) se distinguen a simple vista (son numéricamente distintos, pero parecidos).

## E. Plan de calibración física (en orden)

Antes de cada sesión: batería ≥ 11.5 V. Primero con las ruedas en el aire (página DIAG), luego en el suelo. Ten el monitor Serial abierto (`pio device monitor`). **Borra el mapa NVS** (HOLD 5 s) tras instalar esta versión: los mapas grabados con el punto de lectura antiguo pueden contener paredes falsas. La calibración `ircal` (L/R/P) **no** se borra.

1. **Recto (paso de celda).** En `settings.h` pon `CAL_STRAIGHT_CELLS 5` (necesita 6 celdas rectas libres) y compila. Robot con la parte trasera contra la pared de salida, HOLD 3 s en RUN: hace el avance inicial y 5 celdas en crucero, frena y muestra `odo` (mm de encoder recorridos, incluido el deslizamiento final). Mide con regla **D** = pared de salida → parte trasera del robot, y **L** = paso real de celda (centro de un poste al centro del siguiente). Calcula `CELL_STEP_ADJ_MM = L × odo / D − CELL_FWD_MM` (el log `[CAL-RECTA]` imprime la fórmula con los números puestos). Repite 3 veces y usa el promedio. Comprueba: con el valor nuevo, `D` debe quedar ≈ (inicial + 5 × L) en físico. Vuelve a `CAL_STRAIGHT_CELLS 0`. No toques `WHEEL_DIAM_MM`, `CELL_MM` ni `PRESHIFT_MM`.
2. **Avance inicial.** Mide `a` = distancia de la pared trasera al eje con el robot apoyado. El ideal es `INITIAL_ADVANCE_MM = (70 − a)·1.018 − READ_BACK_MM`. Mientras no lo midas, se mantiene en 17.5.
3. **Punto de lectura.** En un pasillo donde termina una pared lateral, avanza la celda y mira `[SCAN]`: L/R deben estar claramente por encima del umbral donde hay pared y por debajo donde no la hay. Si una pared de la celda **siguiente** aparece en la actual, aumenta `READ_BACK_MM` de 2.5 en 2.5 mm (requiere volver a medir el paso 5). Con `[SCAN] dur=..ms ..mm` fija `SCAN_LEAD_MM` ≈ la mitad de esos mm.
4. **Punto de giro T.** Mide desde el eje hasta las esquinas delanteras del chasis. Con el robot girando en una celda con 3 paredes, observa si el frente o la cola rozan. Ajusta `TURN_POINT_OFFSET_MM` de ±2 mm en ±2 mm (positivo = gira más adelante). Empieza en 0.
5. **Pivote frontal.** Coloca el eje en el punto de giro (centro + T) con pared al frente, ve a la página CAL PIVOTE y mantén pulsado (HOLD). En el log `[ALIGN]` el recorrido debe acercarse a 17.5 mm: `[PRESHIFT] IR: previsto/recorrido`. Si sale "tope-enc", el pivote está mal calibrado o la pared detectada no era la de enfrente.
6. **Preshift sin pared.** Giro en una celda abierta: `[HALT] sobrepaso` + `[PRESHIFT] encoder` deben sumar unos 17.5 mm. Compara con `PRESHIFT_ENABLE 0` si hay dudas.
7. **Giros izquierda/derecha**, con y sin pared: `[GIRO-BNO] err` debe quedar en ±2°. No cambies el control del BNO salvo con evidencia.
8. **Pasillo largo, esquinas y callejón.** Tras una media vuelta solo debe haber un `[PRESHIFT]` y luego "consecutivo". Si aparecen paredes de esquina falsas, prueba `IR_WALL_CONFIRM 1` y revisa `win n/hL/hR` en `[SCAN]`.
9. **Primera exploración** con NVS borrado: ida, vuelta e ida, y `[MAPA] guardado` 3 veces. Compara el dibujo ASCII con el laberinto real.
10. **Segunda carrera con NVS:** `[RUN] mapa PREVIO (completo)`, una sola ida por el camino mínimo.

Ante un `[FALLO]`, el robot espera BOOT para reiniciar. El último `[POSE]` y `[FWD]`/`[ATASCO]` indican qué lo causó (mapa, percepción, odometría o mecánica).

## F. Log por WiFi (sin cable Serial)
- `WIFI_LOG_ENABLE 1` (settings.h): al arrancar, el robot crea la red **UMouse-S3** (clave `umouse123`). Con el móvil conectado, abre **http://192.168.4.1**: el log aparece en vivo (POSE, SCAN, ALIGN, PRESHIFT, GIRO, ATASCO, FALLO, MAPA). Botones: pausar, descargar `.txt`, borrar.
- El log (16 KB, lo más reciente) vive en RAM `.noinit`: **sobrevive al reinicio** tras "ERROR NAV" (BOOT) y a pánicos/watchdog. Cada arranque escribe su motivo (`software`, `PANICO`, `BROWNOUT`, ...). Se pierde al quitar la alimentación.
- Puedes conectarte después de la corrida: el log queda ahí hasta que se llene o se borre.
- El WiFi no usa LEDC ni ADC1 (IR y VBAT están en ADC1). Potencia de TX reducida (8.5 dBm). Si notas lecturas IR distintas o reinicios por BROWNOUT, compara con `WIFI_LOG_ENABLE 0`.

## G. Giro en ARCO (`TURN_MODE 2`, 2026-10-16) — respaldo: `../Raton_V3_pre_ARCO_2026-10-16.zip`
- **Lectura temprana:** las paredes se leen con el eje en s = −`ARC_READ_BACK_MM` (−100) en vez de −17.5. Los IR
  laterales están 58.7 mm delante del eje con 5° hacia delante: tocan la pared en s ≈ −35, lejos del poste de entrada.
  El frontal ahí no alcanza: `[SCAN] F=0?` = "no vi pared, pero no es seguro".
- **Si FloodFill gira:** arco del eje de radio r = −s (máx `ARC_R_MAX_MM` 78, con recta previa si sobra; mín
  `ARC_R_MIN_MM` 50), rueda exterior ×(1+45/r), interior ×(1−45/r) sin piso de arranque, BNO corrige contra el arco
  ideal. Sale en marcha; si ya pasó la lectura temprana de la celda nueva, lee ahí mismo (`[MOV] AVANZA 0`).
- **Si FloodFill sigue recto:** `ff_frontClearAhead()` avanza hasta la lectura de siempre (−17.5, con frenado por perfil)
  y lee el frontal (`[CONFIRMA]`). Con pared: `Maus::move` devuelve false (coords sin cambiar) y FloodFill vuelve a decidir.
- **180° o sin espacio para el arco:** se acerca a −17.5 con el perfil y gira en el sitio como el modo 0.
- `MOTOR_PWM_CAP` 100 → 120 (solo la rueda exterior del arco pasa de 100).
- Volver al giro anterior: `TURN_MODE 1` (rodante) o `0` (en el sitio).
