# Contexto para Claude Code — UMouse S3 (Micromouse)

Este archivo resume el trabajo hecho en la laptop el 2026-10-16 para continuarlo en otra PC.
Léelo completo antes de tocar código. `CAMBIOS_Y_CALIBRACION.md` tiene el detalle histórico.

## 1. Qué es
- Robot Micromouse con ESP32-S3 (PlatformIO, Arduino). Código en `src/`. `UMouse/` es el firmware
  ORIGINAL de referencia (no se compila; sirve para comparar).
- Sensores: 4 IR (FL, FR frontales; L, R laterales, en ADC1), encoders en ambas ruedas,
  IMU BNO085 (rumbo), OLED, LED RGB + LEDs de pared. FloodFill en `src/maus.h` + `src/floodfill.h`.
- Compilar: `pio run` (o `~/.platformio/penv/bin/pio run`). Pruebas de lógica en PC:
  `sh test/host/run.sh` (FloodFill con laberintos reales, geometría `navgeom.h`, LEDs). Deben dar
  4 × `RESULTADO: OK`.
- El usuario prueba TODO en el robot real y pega el log. El robot crea un AP WiFi
  `UMouse-S3` (clave `umouse123`) con el log en `http://192.168.4.1` (`src/netlog.h`). No hay Serial
  durante las corridas. Claude no puede conectarse al robot: solo trabaja con el log que pega el usuario.

## 2. Estado actual (VALIDADO)
- **PROFILE 2 = COMPETENCIA** (`src/settings.h` línea `#define PROFILE`): pista real 9×19,
  inicio (8,18) mirando al Norte, meta (7,4). El usuario confirmó que resuelve el laberinto
  correctamente a `FWD_PWM 75`.
- PROFILE 1 = PRUEBAS (3×6, mismas medidas). PROFILE 3 = ROBGAM (celda 190 mm, avance inicial 65 mm,
  13×13; `CELL_STEP_ADJ_MM` NO calibrado para ese perfil).
- Copias de seguridad (zip, una carpeta arriba del proyecto en la laptop):
  `Raton_V3_NAV_OK_2026-10-16.zip` (navegación validada a 55 PWM) y
  `Raton_V3_COMPETENCIA_75pwm_2026-10-16.zip` (la versión actual de competencia).

## 3. Modelo de navegación (no romper)
- `src/navgeom.h`: `s` = posición del eje respecto al CENTRO de la celda (mm de encoder).
  Lectura de paredes en s = −17.5 (`PRESHIFT_MM`), punto de giro en s = 0. Avance recto entre celdas =
  `CELL_STEP_MM` = 160 − 17.5 + 15 = 157.5 mm de encoder (celda real 162 mm cruz a cruz, el encoder
  cuenta 0.972 de lo real).
- **Crucero**: entre celdas rectas el robot NO frena; el objetivo de encoder es absoluto y se acumula
  (`g_cruiseTargetMM`). `motionHaltIfCruising()` frena y mide el sobrepaso (`[HALT] sobrepaso`).
- Giro (`_doTurn` en `src/motion.h`): frena → (si se pasó > `PIVOT_BACKUP_MM` retrocede al pivote) →
  si hay pared al frente se alinea con IR (`ff_alignToFrontWall`, nivel `g_irPivot`≈431) → giro por
  BNO en el sitio (`_turnBNO`) → avanza 140 mm a la lectura de la celda nueva. Los giros de 180° son
  UN giro (`ff_turnAround180`).
- Paredes: `ff_hasWall*` leen una instantánea por celda (se reutiliza si FloodFill re-escanea la
  misma celda). Laterales confirmadas por ventana (`IR_WALL_CONFIRM 1`, `IR_WALL_WINDOW_MM 50`,
  necesita `win n>=3`). Frontal exige los DOS sensores (`IR_FRONT_REQUIRE_BOTH 1`).
- Rueda bloqueada → `[RUEDA] divergencia` → parada segura (no cuenta celdas falsas).

## 4. Valores clave en `src/settings.h`
| Ajuste | Valor | Nota |
|---|---|---|
| `FWD_PWM` | 75 | crucero (PWM 8 bits; tope `MOTOR_PWM_CAP` 100 en motion.h; `dMax`=255·6V/VBAT) |
| `TURN_VOLTS` | 4.5 | NO son voltios: `pwmForVolts` devuelve % usado como cuenta 0–255 ⇒ PWM ≈36 |
| `TURN_PWM_FAST` / `TURN_SLOW_DEG` | 60 / 30° | fase rápida del giro; los últimos 30° a ~36 |
| `TURN_BOOST_MAX_PWM` | 55 | refuerzo si el giro se atasca |
| `IR_WALL_THR_L/R/C` | 70/80/80 | **el usuario NO quiere cambiarlos** |
| `SCAN_LEAD_MM` | 10 | lee 10 mm antes (la lectura dura 43 ms en marcha) |
| `IR_CENTER_PERIOD_MS` | 20 | más muestras para la ventana de paredes |
| `IR_BRAKE_*` | THR 80, PWM 55, SMOOTH 1, FULL_FRAC 0.6, DECEL 4 | frenado por pared frontal |
| `PIVOT_BACKUP_MM` | 15 | retroceso al pivote tras sobrepaso |
| `FWD_START_PWM` / `FWD_RAMP_STEP` | 55 / 3 | rampa de arranque tras paradas |
| `PRESHIFT_MIN_MM` | 8 | tras retroceder no hace otro avance corto |

## 5. Preferencias del usuario (importante)
- Habla en español. Prueba cada cambio en el robot y pega el log; explica qué mirar en el log.
- NO frenar en cada punto de lectura (quiere mantener el crucero entre celdas). Ya rechazó un perfil
  "lento antes de cada lectura".
- No cambiar umbrales IR ni PWMs sin que lo pida. Proponer cambios de comportamiento y dejar que decida;
  cuando pide algo explícito, hacerlo.
- Antes de un cambio grande, guardar copia (zip) de la versión que funciona.
- Siempre compilar y correr `sh test/host/run.sh`; decir claramente que falta probar en el robot.

## 6. PRIORIDADES AHORA (a 75 PWM)
1. **Giros continuos**: hoy el robot frena en cada celda donde gira (frena → retrocede/alinea →
   gira en el sitio → arranca). Objetivo: girar sin detenerse (giro en arco / "search turn" de
   micromouse) cuando sea posible. Datos útiles del log:
   - La decisión de girar llega en el punto de lectura (s = −17.5 mm, más `SCAN_LEAD_MM`): solo
     ~17 mm antes del centro. Un arco necesita empezar antes o tener radio pequeño; considerar mover
     el punto de lectura / anticipar la lectura lateral de la celda siguiente, o arcos que empiecen
     en el punto de lectura con offset longitudinal/lateral compensado en `navgeom.h`.
   - Sobrepaso al frenar desde 75: ~30–50 mm sin pared al frente (por eso existe `PIVOT_BACKUP`).
   - Mantener el modo actual (giro en el sitio) para 180° y cuando haya pared al frente para alinear,
     o justificar el cambio. Dejarlo activable con un `#define` para comparar.
2. **Frenado frente a muros**: el robot todavía se "aporrea" contra la pared del frente. Lo que se ve
   en los logs (`[FRENO-IR] pared al frente a X mm del final del tramo: PWM 75 -> a (al llegar b, nivel n)`):
   - Los frontales solo empiezan a ver la pared (nivel ≥ 80) a ~5–40 mm del final del tramo; muchas
     veces llega a la lectura a PWM 70+ en vez de 55. El sensor ve tarde ⇒ casi no hay distancia para
     frenar. Ideas: frenar en función de la pared frontal que ya se conoce del mapa / de la lectura de
     la celda anterior (en la lectura de la celda k el frontal ya puede indicar pared en k+1),
     curva nivel-IR→distancia para frenar con perfil, o bajar `IR_BRAKE_THR` (ojo con falsos).
   - `motionHaltIfCruising()` frena en seco (brake 60 ms + coast 60 ms); con pared delante podría
     convenir una aproximación final controlada por IR en lugar del frenazo + `ALIGN`.

## 7. Problema abierto (no resuelto)
- A 65 y 85 PWM algunas corridas **pierden una celda de posición** (el mapa queda desplazado una
  columna; el robot "atraviesa" paredes conocidas). A 75 funcionó, pero puede ser suerte.
- Síntoma raro: tras algunos giros el tramo de 140 mm termina 40–85 ticks más allá de `stopAt`
  (`[FWD] 140.0mm obj=559 tramo=560..605t`, cuando debería parar en ~519). Revisado el bucle de
  `_driveEncoder` y no se encontró la causa. También `[ALIGN] tope-enc` con IR diciendo que la pared
  aún está lejos ⇒ el robot está físicamente detrás de lo que cree el encoder.
- Si los cambios de prioridad afectan esto, avisar.

## 8. Cómo leer el log (líneas clave)
`[SCAN] FL FR L R -> F L R win n hL hR dur Xms Ymm` (paredes; Y = lo que avanzó durante la lectura) ·
`[HALT] sobrepaso` · `[PIVOTE] retrocedo` · `[ALIGN] llego/tope-enc` · `[GIRO-BNO] obj err pwm max t` ·
`[FWD] mm obj tramo t` · `[RAMPA]` · `[FRENO-IR]` · `[RUEDA] divergencia` · `[MAPA] guardado/inalcanzable`.
Para diagnosticar falsas paredes: comparar el mapa del log con el mapa de una corrida buena del mismo
laberinto (el de 75 PWM del 2026-10-16 es correcto).
