#pragma once

// =============================================================
//  UMouse — settings.h
//  Cambia PROFILE para alternar entre prueba, competencia y ROBGAM.
// =============================================================

// ── PERFIL ───────────────────────────────────────────────────
//   1 = PRUEBAS     — Celdas de 140mm y laberinto 3x6.
//   2 = COMPETENCIA — Celdas de 140mm y pista real 9x19.
//   3 = ROBGAM      — Celdas de 180 mm.
#define PROFILE  2

// ── MODO DE NAVEGACIÓN ───────────────────────────────────────
//   1 = sensores IR + encoders (lógica original probada; BNO solo informativo)
//   2 = BNO085 únicamente (sin IR, sin encoders).
//   3 = BNO085 referencia principal + IR/encoders de apoyo
#define NAV_MODE  3

#if NAV_MODE < 1 || NAV_MODE > 3
  #error "NAV_MODE debe ser 1, 2 o 3"
#endif

#define NAV_USE_ENC  (NAV_MODE != 2)   // encoders para distancia/atasco/giro
#define NAV_USE_IR   (NAV_MODE != 2)   // IR para paredes y alineación
#define NAV_USE_BNO  (NAV_MODE != 1)   // BNO para heading/giros

// ── BNO085 ───────────────────────────────────────────────────
// 1 = GPIO38 controla el MOSFET IRLZ44N (corte de GND, ver PDF)  [hardware actual]
// 0 = GPIO38 conectado directo al pin RST del BNO (esquemático anterior)
#define BNO_USE_PWR_GATE   1
#define BNO_OFF_MS         350     // tiempo con el BNO apagado (PDF: 300-350)
#define BNO_ON_SETTLE_MS   300     // espera tras encender (PDF: 200-350)
#define BNO_INIT_TRIES     3
#define BNO_REPORT_US      10000   // 100 Hz orientación y gyro
#define BNO_ACCEL_US       50000   // 20 Hz aceleración (solo informativa)
#define BNO_STALE_MS       200     // sin muestra de yaw en este tiempo = dato viejo
#define BNO_MAX_RECOVERIES 3       // power-cycles automáticos por arranque

// Con BNO, el tiempo de refresca de la OLED con el compás en
// vivo mientras el robot se mueve (navUiTick en motion.h).
#define NAV_UI_REFRESH_MS  120
// 1 = también refrescar la brújula DENTRO de los lazos de giro/aproximación a pared.
// 0 (recomendado): un dibujo de OLED bloquea ~30 ms el lazo; en un giro a ~300°/s son ~9° "a ciegas". La brújula se actualiza justo antes y después del giro.
#define NAV_UI_IN_TURN     0

// Deadman de motores: si los motores están activos y nadie les manda comando en este
// tiempo (lazo colgado por I2C, recuperación del BNO, etc.), una tarea de esp_timer los
// frena y el firmware se detiene con error. Sin esto, en modo crucero un cuelgue deja
// los motores latcheados al último PWM.
#define MOTION_WD_MS       400

// Reportes del BNO. La librería Adafruit solo entrega UN reporte por llamada y, si un
// paquete trae varios, se queda con el ÚLTIMO (sensorhubInputHdlr decodifica todos en
// la misma variable). Con 3 reportes activos y sondeo irregular, el yaw se puede perder.
//   0 = solo GAME_ROTATION_VECTOR; la velocidad angular se deriva del yaw (recomendado)
//   1 = además gyro calibrado y aceleración lineal (comportamiento anterior)
#define BNO_EXTRA_REPORTS  0

// heading = wrap360( SIGN * (yaw_bno - yaw_ref) )   (compás: horario = +)
// El yaw del BNO es antihorario-positivo (Z arriba) => SIGN = -1.
// Verificar en OLED (página BNO): girar el robot a la derecha DEBE subir heading.
// Si sube al revés: cambiar a +1 (o el BNO no está montado plano/Z arriba).
#define BNO_HEADING_SIGN   (-1.0f)

// ── NAV_MODE 2/3: control por heading ────────────────────────
// Recto: corrección diferencial por error de heading (grados).
// Equivalencia con el control original: 1° de heading ≈ 6.3 ticks de
// diferencia entre ruedas (TRACK_WIDTH 90mm, 3.99 ticks/mm) => KP_ENC(0.30)*6.3 ≈ 1.9.
// Estos valores son NUEVOS y no están calibrados en el robot.
#define KP_HDG            1.5f
#define KI_HDG            0.10f
#define KI_HDG_MAX        7.0f
#define KD_HDG            0.04f   // PWM por deg/s del giroscopio (400 deg/s => 16): frena el giro que queda tras un arco
#define HDG_DEADBAND_DEG  0.5f

// Giro cerrado por BNO
#define BNO_TURN_TOL_DEG        2.0f    // se detiene a ±tol (+ anticipación)
#define BNO_TURN_LEAD_S         0.05f   // anticipación de frenado = rate * lead
#define BNO_TURN_TIMEOUT_MS     2500
#define BNO_TURN_FINE_TOL_DEG   1.5f
#define BNO_TURN_FINE_TRIES     3
#define BNO_TURN_FINE_PULSE_MS  25
// Log 2026-10-16: cada pulso de ajuste fino cuesta ~135 ms (casi la mitad de los giros hacía 3 = ~400 ms
// quieto) y aun así quedaba en ±1.5–2.6°. El rumbo objetivo es ABSOLUTO: el control de rumbo de la recta
// siguiente corrige lo que quede. Solo se hace ajuste fino si el error supera esto. 0 = siempre (anterior).
#define BNO_TURN_FINE_ACCEPT_DEG 3.0f
#define BNO_TURN_ENC_MAX_FACTOR 1.6f    // NAV_MODE 3: validación por encoder
// Refuerzo de giro ATASCADO: si tras la gracia el BNO gira a menos de TURN_STALL_RATE_DEGS
// durante TURN_STALL_MS, el PWM del giro sube TURN_BOOST_STEP (hasta TURN_BOOST_MAX_PWM).
// Un giro normal no cambia. TURN_BOOST_MAX_PWM 0 = desactivado (comportamiento anterior).
#define TURN_STALL_GRACE_MS     150
#define TURN_STALL_RATE_DEGS    15.0f
#define TURN_STALL_MS           100
#define TURN_BOOST_STEP         5
#define TURN_BOOST_MAX_PWM      55
// Giro RÁPIDO: mientras falten más de TURN_SLOW_DEG grados gira a TURN_PWM_FAST; el final lo hace
// con el PWM normal (TURN_VOLTS, ~36) para no pasarse. TURN_PWM_FAST 0 = desactivado (anterior).
#define TURN_PWM_FAST           60
#define TURN_SLOW_DEG           30.0f
// Tras alinear con pared frontal (NAV_MODE 3) el heading se re-referencia
// al rumbo nominal si el error es menor a este valor (0 = desactivado)
#define BNO_SNAP_MAX_DEG        12.0f

// NAV_MODE 2: distancia por TIEMPO (lazo abierto, NO es odometría).
// Derivado del comentario original: ~1750 ms para 142.5 mm a FWD_PWM=45, 12.6 V
// => 12.3 ms/mm. DEPENDE DE LA BATERÍA: calibrar en el robot.
#define NAV2_MS_PER_MM    12.3f

// ── GEOMETRÍA ────────────────────────────────────────────────
#define WHEEL_DIAM_MM    33.5f   // diámetro real de la rueda (mm)
#define TRACK_WIDTH_MM   78.0f  // centro-rueda a centro-rueda: 85 (ancho total, ruedas al ras) - 7 (ancho de rueda). Antes 90.

// ── ENCODER ──────────────────────────────────────────────────
#define ENC_PPR_MOTOR    7      // pulsos por rev en eje del motor
#define ENC_GEAR_RATIO   30     // reducción de la caja
#define ENC_EDGES        2      // 2=canal A solo, 4=cuadratura A+B

// Derivados — no tocar
#define ENC_TICKS_PER_REV  ((float)(ENC_PPR_MOTOR * ENC_GEAR_RATIO * ENC_EDGES))
#define WHEEL_CIRC_MM      ((float)(3.14159265f * WHEEL_DIAM_MM))
#define TICKS_PER_MM       ((float)(ENC_TICKS_PER_REV / WHEEL_CIRC_MM))

// ── LABERINTO ────────────────────────────────────────────────
#if PROFILE == 1 || PROFILE == 2
  #define CELL_MM   160.0f   // PRUEBAS y COMPETENCIA (con CELL_STEP_ADJ_MM: celda real de 162 mm)
#else
  #define CELL_MM   200.0f   // ROBGAM (18cm)
#endif

// Preshift: distancia desde posición de reposo hasta eje de giro
// = mitad de celda - distancia eje rueda al centro robot
// Competencia: 90mm - 17.5mm = ~72mm... pero empíricamente 38mm funcionó
// Prueba 14cm: ajustar proporcionalmente o medir
#if PROFILE == 1 || PROFILE == 2
  #define PRESHIFT_MM   17.5f
#else
  #define PRESHIFT_MM   17.5f   // ROBGAM
#endif

// ── MODELO LONGITUDINAL (ver navgeom.h) ──────────────────────
// Todas las distancias en "mm odométricos" (los que cuenta el encoder).
// CELL_FWD_MM = CELL_MM - PRESHIFT_MM (valores del perfil, sin tocar).
// OJO: el 160 histórico (UMouse/) se calibró FRENANDO en cada celda: el deslizamiento del
// frenado no se contaba (los encoders se reiniciaban) y quedaba "escondido" en el valor.
// En crucero no hay frenado entre celdas, así que en pasillos largos ese extra falta.
#define CELL_FWD_MM    ((float)(CELL_MM - PRESHIFT_MM))
// Corrección del paso de celda (mm de ENCODER por celda física) — [MEDIR] con CAL_STRAIGHT_CELLS.
//   CELL_STEP_ADJ_MM = L_celda * odo / D  -  CELL_FWD_MM
//   L_celda = paso real de celda (centro a centro de dos postes), D = desplazamiento físico medido,
//   odo = "odo" que muestra la prueba. 0 = comportamiento de los valores del perfil.
#if PROFILE == 1 || PROFILE == 2
  #define CELL_STEP_ADJ_MM     15.0f   // L = 162 mm (cruz a cruz) x 0.972 (odo 729.2 / D 750) = 157.5 mm de encoder por celda
#else
  #define CELL_STEP_ADJ_MM     15.0f   // ROBGAM: [MEDIR] (calibrado para celdas de 162 mm, no de 190)
#endif
#define CELL_STEP_MM           ((float)(CELL_FWD_MM + CELL_STEP_ADJ_MM))
// Prueba de RECTA (calibración): N > 0 => HOLD 3 s en RUN hace SOLO avance inicial + N celdas en
// crucero, frena y muestra lo recorrido (OLED + Serial). Sin FloodFill, sin paredes, sin NVS.
// Necesita N+1 celdas rectas libres delante del robot. 0 = corrida normal.
#define CAL_STRAIGHT_CELLS     0
// Cronómetro de corrida (main.cpp): para al ENTRAR en la celda objetivo (antes del frenado y el centrado lento, que
// sumaban ~0.4-0.7 s: la OLED marcaba 11.27 s con 10.9 s reales). Ajuste fino: ms que se restan a cada tramo.
// 200 (2026-10-21): cronometrado a mano 10.56 s, OLED 10.76 s. El robot para el reloj en la lectura temprana de la meta
// (frente ya ~55 mm dentro de la celda, ~0.13 s después de "entrar") y arranca antes de que se vea moverse.
#define RUN_TIMER_OFFSET_MS    200
// Con mapa previo: FloodFill va solo por tramos ya conocidos si hay camino (no explora callejones nuevos). 0 = como antes.
#define KNOWN_ROUTE_ONLY       1
#define ENC_CELL_FWD   ((int)lroundf(CELL_FWD_MM * TICKS_PER_MM))
#define ENC_PRESHIFT   ((int)lroundf(PRESHIFT_MM * TICKS_PER_MM))

// Preshift ACTIVABLE para calibrar:
//   1 = las paredes se leen con el eje READ_BACK_MM ANTES del punto de giro; antes de girar
//       se avanza esa distancia (o la alineación IR frontal deja el eje en el punto de giro)
//       y tras el giro se avanza CELL_FWD_MM - READ_BACK_MM.
//   0 = lectura en el mismo punto de giro: sin avance previo, tras el giro CELL_FWD_MM
//       (equivale al comportamiento anterior, con el resto de correcciones).
#define PRESHIFT_ENABLE        1
#define READ_BACK_MM           (PRESHIFT_ENABLE ? PRESHIFT_MM : 0.0f)
// Punto de giro respecto al CENTRO de la celda (mm, + = hacia delante). [MEDIR] 0 = eje sobre
// el centro (lo que asume la calibración CAL PIVOTE actual). Si se cambia, re-calibrar g_irPivot
// con el eje en (centro + TURN_POINT_OFFSET_MM).
#define TURN_POINT_OFFSET_MM   0.0f
// En crucero, terminar el avance este tanto ANTES del punto de lectura para que la lectura
// (que dura ~36 ms con el robot en marcha) quede centrada en él. [MEDIR] con el log "[SCAN]"
// (campo dur): poner ~la mitad de ese valor. 0 = desactivado.
#define SCAN_LEAD_MM           10.0f  // a 85 PWM la lectura avanzaba 23 mm (13 a 55): termina en el mismo punto
// La aproximación IR a la pared frontal no puede recorrer más que lo previsto + este margen.
#define ALIGN_MAX_EXTRA_MM     25.0f

// Tras posicionar con el IR, el BNO solo se re-referencia si FL y FR difieren menos que esto
// (fracción de su promedio): sin contacto con la pared, "alineado" hay que comprobarlo.
#define IR_SQUARE_TOL_FRAC    0.10f

// Giro 90°: arco = π × track_width / 4
#define TURN90_ARC_MM  ((float)(3.14159265f * TRACK_WIDTH_MM / 4.0f))
#define ENC_TURN90_CALC ((int)(TURN90_ARC_MM * TICKS_PER_MM))

// Override manual del giro (ajustar por prueba física)
// Comenta estas dos líneas para usar el valor calculado
#undef  ENC_TURN90_CALC
#define ENC_TURN90_CALC  255   // valor calibrado empíricamente

#define ENC_TURN90  ENC_TURN90_CALC

// Avance inicial desde pared trasera al centro de primera celda
#if PROFILE == 1 || PROFILE == 2
  #define INITIAL_ADVANCE_MM  17.5f
#else
  #define INITIAL_ADVANCE_MM  65.0f   // ROBGAM
#endif
// Tras el avance inicial el eje queda, por convención, en el punto de lectura de la 1.ª celda.
// [MEDIR] a = distancia pared trasera -> eje con el robot apoyado: ideal = (70 - a)*1.018 - READ_BACK_MM
#define ENC_INITIAL_ADVANCE  ((int)lroundf(INITIAL_ADVANCE_MM * TICKS_PER_MM))

// ── MOTORES ──────────────────────────────────────────────────
// Tope de voltaje en motores: g_dutyMax = 255 * V_MOTOR_LIMIT / VBAT (sensors.h).
// 12 V (2026-10-20; antes 9 V y 6 V): ~243 a 12.6 V, ~247 a 12.4 V y 255 (batería completa) con VBAT <= 12 V.
// Con una 3S es prácticamente SIN límite. Motores N20 de 6 V nominales (al 200 %): vigilar temperatura y atascos.
// MOTOR_PWM_CAP (motion.h) = 255 para que el límite sea este.
#define V_MOTOR_LIMIT   12.0f  // voltaje máximo motores (V)
#define MOTOR_KICK_MIN  30     // PWM mínimo para romper estática (0-255)

// PWM de avance (0-255, relativo a la batería: NO cambia con V_MOTOR_LIMIT). 45 funcionó bien con batería 12.6V
// 75 = validado en la pista de competencia (2026-10-16). 60 = PRUEBAS del giro en arco (autorizado por el usuario):
// a ~340 mm/s el frenado por perfil para antes de la pared sin el tope CONFIRM_VMAX (que frenaba en cada celda).
// 150 (2026-10-20, motores a 9 V): ~930 mm/s estimados (120 = ~750). Con mapa frena para cada curva con varias celdas
// de anticipación (_vCapBeforeTurn en motion.h). Subir de 10 en 10 vigilando [HALT] y [ARCO] en el log.
// 240 (2026-10-20, 12 V, PRUEBA "del máximo hacia abajo" pedida por el usuario): ~= g_dutyMax con la 3S llena (243).
// Sin margen para que la corrección de rumbo ACELERE una rueda (solo frena la otra, como a 120/120). Bajar de 20 en 20.
#define FWD_PWM        240

// Voltaje objetivo para giros — normalizado por VBAT
// Ajustar hasta que giro sea exactamente 90°
#define FWD_VOLTS       3.1f
// ── FRENADO POR PARED FRONTAL (el crucero NO frena en cada lectura) ──
// En recta, si los dos sensores frontales ven una pared adelante (min(FL,FR) >= IR_BRAKE_THR),
// baja a IR_BRAKE_PWM (velocidad validada) hasta el final del tramo. Sin pared adelante: FWD_PWM.
// Los frontales solo ven ~1 celda: protege los choques DE FRENTE, no las esquinas abiertas.
#define IR_BRAKE_ENABLE     1
#define IR_BRAKE_THR        80     // = umbral de pared frontal: frena en cuanto aparece la pared
#define IR_BRAKE_PWM        55     // velocidad con la que se validó la navegación
// Frenado SUAVE: la velocidad baja en proporción a lo cerca que está la pared (nivel frontal de
// IR_BRAKE_THR a IR_BRAKE_FULL_FRAC*pivote => de FWD_PWM a IR_BRAKE_PWM), y nunca más de
// IR_BRAKE_DECEL de PWM cada 10 ms (85->55 en ~80 ms). 1 = suave, 0 = escalón (anterior).
#define IR_BRAKE_SMOOTH     1
#define IR_BRAKE_FULL_FRAC  0.6f
#define IR_BRAKE_DECEL      4
#define IR_FRONT_PERIOD_MS  20     // lectura frontal durante el avance (3 muestras por sensor)
// Frenado con PERFIL hacia el punto de giro (reemplaza al suave/escalón cuando vale 1). Si los dos
// frontales ven algo (min(FL,FR) >= IR_BRAKE_START_THR) en un avance a FWD_PWM, el siguiente movimiento
// será un giro: el robot frena por encoder para llegar al punto de giro (s = 0) a baja velocidad
// (v_ref = sqrt(2·A·mm_restantes)); si va más rápido que v_ref*IR_BRAKE_OVER, freno activo ese tick.
// IR_BRAKE_START_THR solo dispara el frenado: las paredes del mapa siguen usando IR_WALL_THR_C.
// 0 = frenado anterior (IR_BRAKE_SMOOTH / IR_BRAKE_THR).
#define IR_BRAKE_PROFILE    1
#define IR_BRAKE_START_THR  50     // [MEDIR] FL/FR sin pared delante en [SCAN] deben quedar bien por debajo
// Disparo con UN solo frontal: max(FL,FR) >= IR_BRAKE_ONE_THR también frena (log 2026-10-16: paredes reales
// con FL=40/FR=185, 101/66, 165/58 no disparaban y el robot llegaba a 75 y se pasaba 28 mm hacia la pared).
// Un falso disparo solo cuesta frenar de más (las paredes del mapa no cambian). 0 = solo min(FL,FR).
#define IR_BRAKE_ONE_THR    100
// El disparo solo se arma en los últimos IR_BRAKE_ARM_MM antes del pivote: más lejos, una pared frontal
// no da señal y lo que se ve es una pared lateral/poste (log: disparo a 154 mm tras un giro rodante).
#define IR_BRAKE_ARM_MM     100.0f
#define IR_BRAKE_ACCEL      2000.0f // mm/s² del perfil (frenazo en seco desde 75 ≈ 2500: se pasaba 30–50 mm)
#define IR_BRAKE_VMIN       150.0f  // mm/s: el perfil nunca pide menos (llega al pivote rodando despacio)
#define IR_BRAKE_OVER       1.15f   // margen sobre v_ref antes de freno activo
#define IR_BRAKE_PWM_MIN    (MOTOR_KICK_MIN + 5)   // = PWM de la aproximación ALIGN
// 1 = refrescar la brújula de la OLED dentro del lazo de avance (cada NAV_UI_REFRESH_MS bloquea ~30 ms:
// el robot rueda 13–20 mm sin control; sospecha del sobrepaso de tramo del §7 del handoff).
// 0 = solo entre movimientos.
#define NAV_UI_IN_DRIVE     0
// Si al frenar para girar el eje queda más de esto pasado del punto de giro, retrocede (encoder)
// antes de girar: a más velocidad, más sobrepaso ([HALT]). Solo actúa al girar. 0 = desactivado.
#define PIVOT_BACKUP_MM     15.0f
// Rampa de ARRANQUE: tras una parada (giro, alineación, atasco) sale a FWD_START_PWM y sube
// FWD_RAMP_STEP cada 10 ms hasta FWD_PWM (55->85 en ~100 ms con 3). Evita que las ruedas patinen
// y el encoder cuente de más. En crucero (entre celdas sin parar) no actúa.
#define FWD_START_PWM       55
// Tras retroceder al pivote el eje queda a pocos mm: no hace falta otro avance (ahorra ~0.2 s).
#define PRESHIFT_MIN_MM     8.0f
#define FWD_RAMP_STEP        6   // 2026-10-21 (antes 3): tras cada arco tardaba ~570 ms en llegar a 240 y la celda dura ~300 ms => nunca crucero. Si patina al arrancar: 5
#define TURN_VOLTS      4.5f
// Modo de giro de 90° SIN pared al frente:
//   0 = frena (HALT) -> preshift/retroceso -> giro en el sitio (validado 2026-10-16)
//   1 = RODANTE: empieza a girar en cuanto FloodFill decide, sin frenazo previo (la inercia hace un arco
//       corto); encoder+BNO miden dónde quedó el eje y eso corrige la posición en la celda nueva.
//   2 = ARCO: lectura de paredes TEMPRANA (eje ARC_READ_BACK_MM antes del centro, los IR laterales miran
//       58.7 mm por delante del eje => ya ven la pared de ESTA celda). Si FloodFill gira, el robot traza un
//       cuarto de círculo en marcha (rueda exterior más rápida) y sale centrado en el pasillo nuevo.
//       Si FloodFill sigue recto, antes de avanzar confirma el frontal en el punto de lectura de siempre
//       (con el frenado por perfil); si hay pared, vuelve a decidir y gira en el sitio como el modo 0.
// Con pared al frente (alineación IR), con el frenado por perfil activo y en 180° siempre se usa el modo 0.
#define TURN_MODE       2
// ── GIRO EN ARCO (TURN_MODE 2) ── geometría: IR laterales 58.7 mm delante del eje, 13.4 mm del centro, 5°
// hacia delante (tocan la pared ~65 mm delante del eje); chasis 85x85, eje -> frente 68.25 + 10 de cinta.
// Lectura temprana: s = -ARC_READ_BACK_MM (el sensor toca la pared en s ≈ -35: lejos del poste de entrada).
#define ARC_READ_BACK_MM     100.0f
// Radio del arco del EJE = -s al decidir (el arco termina sobre el eje del pasillo nuevo), acotado a
// [ARC_R_MIN_MM, ARC_R_MAX_MM]. Más lejos que R_MAX: tramo recto y luego arco. Más cerca que R_MIN: giro en el sitio.
// R 78 (vía 78 mm): exterior x1.5, interior x0.5. Holgura con paredes de 20 mm (cara a 70 del centro): ~20 mm
// a la pared exterior (esquina redondeada delantera a 128 mm del centro de giro, pared a 148).
#define ARC_R_MAX_MM         78.0f
#define ARC_R_MIN_MM         50.0f
// PWM del CENTRO durante el arco (exterior = x(1+39/r), interior = x(1-39/r)). Con 60 y r=78: 90 / 30, margen de
// 30 hasta el tope para corregir. 75 (crucero) saturaba la exterior y los arcos a la derecha fallaban.
// 60 (2026-10-20, 9 V): la exterior tiene margen hasta g_dutyMax (~185 a 9 V, ~245 a 12 V). Antes 50.
// 140 (2026-10-20, 12 V, PRUEBA del máximo): _arcTurn ya limita la base a cap/(1+k) ~ 143 con r=78. La entrada al
// arco sigue limitada por ARC_VREAD/ARC_VIN (mm/s): el arco ACELERA desde ~350 mientras gira. Bajar de 20 en 20.
// 80 (2026-10-20, log de exploración a 140): el arco salió bien (girado 92.8, desfase 18.7) pero SALE a ~1 m/s; el tramo
// siguiente (confirmación, ~71 mm) no frena hasta CONFIRM_VMAX (llegó a 697 con tope 300), la cola derrapó hacia FUERA
// y la rueda exterior rozó la pared ([RUEDA] divergencia L=306 R=180 => FALLO). Para frenar a 300 en ~70 mm la salida
// del arco tiene que ser <= ~500 mm/s => ARC_PWM ~80 (x ARC_MMS_PER_PWM). Subir solo si [APROX] tras el arco <= tope.
// 70 (2026-10-20): a 80 sale del arco a ~550 mm/s y gira a ~500 deg/s; en las S el arco siguiente pre-frenaba 20 mm
// (radio 61-71) y no podía invertir el giro. Con 70 sale más lento y la S tiene más sitio.
#define ARC_PWM              70
#define ARC_VIN              350.0f   // entrada al arco IZQ. 380 (2026-10-21): FALLO atrasado entrando a 376 mm/s en (1,14)
#define ARC_PREBRAKE_MAX_MS  100
// Curva en S: si al empezar el arco el robot todavía gira hacia el lado contrario a más de ARC_SPIN_MAX_DEGS, el pre-freno
// sigue (aunque ya vaya a ARC_VIN) frenando ese giro: u = KP_HDG*err + ARC_SPIN_KD*vel_giro, acotado a ±ARC_SPIN_UMAX.
// Solo mientras el eje quede a más de ARC_R_MIN_MM + 5 del centro. ARC_SPIN_MAX_DEGS 1e9 = desactivado.
#define ARC_SPIN_MAX_DEGS    80.0f   // freno del giro contrario en las S. 120: la S de (1,8) no invirtió el giro (girado 2.6°) => FALLO
#define ARC_SPIN_KD          0.08f   // PWM por deg/s (500 deg/s => 40)
#define ARC_SPIN_UMAX        40
#define ARC_PREBRAKE_PWM     15      // pre-freno con PWM bajo + rumbo (no freno a fondo: giraba al revés a 120)
// Lectura temprana según la velocidad (P = R_MAX + v*SCAN_S + (v-VIN)*BRAKE, entre ARC_READ_BACK_MM y el máximo).
// v = FWD_PWM * ARC_MMS_PER_PWM (logs: 75 -> ~425 mm/s, 120 -> ~750). 120 PWM => P ~124.
#define ARC_MMS_PER_PWM      6.2f
#define ARC_SCAN_S           0.032f  // s de lectura en marcha (con el frontal rápido)
#define ARC_BRAKE_MM_PER_MMS 0.05f   // mm de pre-freno por mm/s por encima de ARC_VIN (log: 750 -> 282 en ~20-26 mm)
// Máximo REAL medido: con -124 el IR lateral vio la pared de la celda ANTERIOR (L=485 sin pared => pared falsa =>
// meta inalcanzable, log 2026-10-16). 100 funciona. No subir sin probar.
#define ARC_READ_BACK_MAX_MM 100.0f
// Velocidad máxima al llegar a la lectura temprana cuando la siguiente decisión puede ser un giro (explorando, o con
// mapa si FloodFill prevé giro/parada). Con lectura en -100: 26 ms de lectura (~10 mm) + pre-freno a 300 (~5 mm)
// => el arco aún empieza en ~-85 con radio 78. A 120 PWM (~750 mm/s) sin esto el arco empezaba en -50 y fallaba.
#define ARC_VREAD            430.0f   // 2026-10-21 (antes 400): llegada a la lectura antes de una curva
#define ARC_IN_MIN           8      // PWM mínimo de la rueda interior (1 = freno a fondo => patinaba)
#define ARC_OVERRUN_MM       15.0f   // aborta si el eje pasa del centro de la celda más que esto (rumbo de llegada)
#define ARC_KP               1.5f
#define ARC_K_GAIN           1.4f    // x diferencia geométrica: con r=78 exterior x1.7, interior x0.3    // PWM por grado de retraso del rumbo respecto al arco ideal (BNO)
#define ARC_END_DEG          2.0f    // termina el arco a 90-esto: la recta siguiente (KP_HDG) corrige el resto. Antes 4 (sin anticipación)
// Corta el arco cuando girado + vel_giro * esto >= 90 - ARC_END_DEG: la inercia completa el giro (como BNO_TURN_LEAD_S).
// Log 2026-10-20 (ARC_PWM 80, 12 V): ~350 deg/s al final y ~7-14° de giro extra tras cortar => 0.025 s ≈ 9°.
// Si sigue pasándose de 90: subir de 0.005 en 0.005. Si queda corto (la recta corrige mucho hacia el lado del giro): bajar.
// 0.012 (2026-10-21): con 0.025 los arcos cortaban en 76-80° y la recta siguiente empezaba con 8-14° de error; tras los
// arcos a la DERECHA el robot rozaba la pared izquierda ([RUEDA] divergencia, rueda IZQ). Mirar "rumbo max" del [FWD]
// siguiente: si baja de ~5° está bien; si los arcos se pasan de 90 (error al otro lado) subir de 0.003 en 0.003.
#define ARC_END_LEAD_S       0.018f   // 0.012: girado 85-87° pero la recta se pasaba 13-16° (ahora con KD_HDG)
#define ARC_TIMEOUT_MS       1200
#define ARC_MAX_LEN_FACTOR   1.6f    // recorrido máximo = (recta + arco) * esto; si no giró: fallo -> giro en el sitio
// Arcos a la DERECHA (se atrasaban más que a la izquierda; logs 2026-10-21: 3 FALLO "atrasado" con 23-25° de desfase,
// la rueda exterior ya en el tope de la corrección y la interior en ARC_IN_MIN). Más diferencia de ruedas, más
// corrección y entrada más lenta. Si a la derecha sigue el desfase > ~18°: bajar ARC_VIN_R de 25 en 25.
#define ARC_K_GAIN_R         1.55f
#define ARC_KP_R             2.0f
#define ARC_VIN_R            310.0f   // entrada al arco DER. 330: FALLO atrasado en (3,7) con la exterior al tope (179). Antes 300
// Arco a la DERECHA solo si al decidir el eje está al menos esto antes del centro (el pre-freno come ~12-17 mm). Logs
// 2026-10-21: con radio 56-64 (lectura retrasada por la relectura del lateral) salían con 73-76° y el robot rozaba la
// pared en la recta siguiente. Si no hay sitio: giro en el sitio.
#define ARC_DECIDE_MIN_R_MM  80.0f   // 85 rechazaba arcos normales por 0.3-2 mm (s=-84.7, -83.0) => giros en el sitio (~1 s c/u)
// ...y si tras el pre-freno el radio queda por debajo de esto, se cancela antes de girar (giro en el sitio).
#define ARC_R_MIN_R_MM       62.0f   // 70 cancelaba casi todas las S de la escalera (~0.7 s c/u); con la base escalada por el
                                     // radio, r 62-70 gira a ~400 deg/s como los demás
// Los arcos acababan 4-10 mm hacia el interior: arco con radio menor en esto y recta previa más larga en lo mismo.
#define ARC_INSIDE_COMP_MM   6.0f
#define ARC_FAIL_LAG_DEG     25.0f   // atraso máx respecto al arco ideal antes de abortar (el 1.er arco bueno: 17.8)
// Tras el arco, si encoder+BNO dicen otra cosa que el arco ideal en más de esto, se usa el ideal.
#define ARC_ODO_MAX_DEV_MM   35.0f
// Velocidad máxima al llegar a la lectura frontal cuando FloodFill sigue recto (TURN_MODE 2). Los frontales ven la pared
// ~20-30 mm antes de que la cinta la toque; a 250 mm/s el robot frena en ~12 mm (+5 mm hasta la siguiente lectura).
// El avance siguiente vuelve a FWD_PWM con la rampa. 0 = sin tope (golpeaba: sobrepaso 19-23 mm).
// Solo EXPLORANDO (con mapa previo va sin tope). A 120 PWM es obligatorio: desde ~750 mm/s no se frena en los ~30 mm
// en que el frontal ve la pared. Con 300 mm/s frena en ~18 mm.
#define CONFIRM_VMAX         300.0f
// Deceleración PLANIFICADA para ese tope: con 2000 empezaba solo 29 mm antes y bajar el PWM frena poco (llegaba a
// ~400 mm/s). Con 1200 empieza ~50 mm antes; si va más rápido que tope*CONFIRM_OVER, freno activo ese tick.
#define CONFIRM_ACCEL        1500.0f   // 2026-10-21 (antes 1200): frena más tarde antes de las curvas. Si la llegada [APROX] pasa del tope por >60 mm/s: 1300
// Tope antes de una curva lejana (tramos conocidos): se descuenta la lectura en marcha y se deja margen por el retraso.
#define CAP_SCAN_MARGIN_MM   25.0f   // 2026-10-21 (antes 35)
#define CAP_SPEED_FACTOR     0.95f   // 2026-10-21 (antes 0.9)
// Recta con el rumbo torcido más de STRAIGHT_SLOW_DEG: PWM como máximo STRAIGHT_SLOW_PWM hasta corregirlo.
// 15/100 (2026-10-21): con 5/70 casi TODAS las rectas iban LENTO (error típico 8-14° por el centrado IR y la salida del
// arco) => ~1 s perdido por vuelta. El roce real era con 20-30° (ahora además el arco no se pasa y la divergencia
// descuenta el giro). Si vuelve a rozar tras un arco: bajar a 12.
#define STRAIGHT_SLOW_DEG    15.0f
#define STRAIGHT_SLOW_PWM    100
// Con el rumbo torcido más de esto la rampa NO sube (mantiene la velocidad que lleva; no frena). 2026-10-22: tras los arcos
// de (1,14) y (3,2) aceleraba a ~700 mm/s con 18-24° de error y rozaba la pared lateral. Si [FWD] muestra "rampa retenida"
// en casi todas las rectas y se pierde tiempo: subir a 10-12.
#define STRAIGHT_HOLD_DEG    8.0f
// Confirmación del frontal (TURN_MODE 2, autorizado por el usuario para prueba): además de la regla normal
// (FL y FR >= IR_WALL_THR_C), pared si min(FL,FR) >= CONFIRM_BOTH_MIN y max >= CONFIRM_ONE_THR. 0 = solo la normal.
#define CONFIRM_ONE_THR      100     // log a 50 PWM: paredes reales con 136/44 y 114/41 => F=0 y choque
#define CONFIRM_BOTH_MIN     30      // pared lateral de reojo: el otro frontal da 0-19 (239/19, 216/0)
// Un solo frontal MUY fuerte también es pared (log 2026-10-17: FL=0 FR=327 en pared real => choque). La pared lateral
// de reojo nunca pasó de 239 en los logs. 0 = desactivado.
#define CONFIRM_SOLO_THR     300
#define CONFIRM_OVER         1.0f    // a 120 PWM con 1.05 llegaba a 434 mm/s (tope 300): freno activo en cuanto lo pasa
// Re-localización (TURN_MODE 2): pared frontal vista con fuerza en la lectura temprana => s según el nivel IR,
// entre -RELOC_FAR_MM (nivel = IR_BRAKE_START_THR) y el pivote (nivel = g_irPivot).
#define RELOC_FAR_MM         45.0f
// Si encoder+BNO dicen que el eje quedó más de esto pasado (o antes) del centro de la celda nueva, se avisa.
#define ROLL_WARN_MM    25.0f
// Log 2026-10-16: girando en marcha a ~425 mm/s la cola derrapa hacia FUERA de la curva y se atascó contra
// esa pared. Antes de girar frena hasta ROLL_VMAX (máx ROLL_PREBRAKE_MAX_MS) para que el derrape sea menor.
#define ROLL_VMAX            250.0f  // mm/s
#define ROLL_PREBRAKE_MAX_MS 80
// No rueda si la pared del lado EXTERIOR del giro está más cerca que el centrado (lectura > cal * esto).
// Usa la calibración L0/R0 de la página IR (no es un umbral de pared). Sin calibrar: solo sin pared exterior.
#define ROLL_OUT_MAX_FRAC    1.00f
// Si el giro rodante se atasca (BNO casi quieto), avanza esto con el rumbo actual (ya mira al pasillo nuevo)
// para despegar la cola de la pared y termina el giro en el sitio. 0 = sin desatasco.
#define ROLL_UNJAM_MM        12.0f

// Signos de encoder (corregir si cuentan al revés)
#define ENC_SIGN_L   (+1)
#define ENC_SIGN_R   (-1)

// Trim de encoder derecho para compensar asimetría mecánica
// 1.0 = sin corrección, >1.0 = R avanza "menos" en el cálculo
// Ajustar de 0.01 en 0.01 si hay deriva sistemática
// ++ = gira mas la derecha, -- =gira menos la derecha
#define ENC_TRIM_R   1.00f //MEJOR RESULTADO CON BATERIA A 12.4V EXACTOS, tanto para giros con el desatacasmiento como para giros 

// ── CONTROL DIFERENCIAL ──────────────────────────────────────
// Estos valores funcionaron bien el día de pruebas
#define KP_ENC      0.30f
#define KI_ENC      0.13f
#define KI_ENC_MAX  7.0f

// ── SENSORES IR - COMPETENCIA(solo para detección de paredes) ─────────────
#define IR_TARGET_L   235
#define IR_TARGET_R   265
#define IR_TARGET_C   265

// ── SENSORES IR - TEST(solo para detección de paredes) ─────────────
//#define IR_TARGET_L   235
//#define IR_TARGET_R   265
////#define IR_TARGET_C   265

#define IR_WALL_THR_L  70    // umbral pared lateral izquierda
#define IR_WALL_THR_R  80    // umbral pared lateral derecha
#define IR_WALL_THR_C  80    // umbral pared frontal

// Hardware S3: 2 sensores frontales (FL/FR) reemplazan al central (C).
// Mismo umbral que tenía C (y que el sketch de pruebas S3).
#define IR_WALL_THR_FL  IR_WALL_THR_C
#define IR_WALL_THR_FR  IR_WALL_THR_C
// Pared frontal = FL o FR (0, como en el sketch de pruebas S3)
//               = FL y FR (1, más robusto contra falsos positivos de paredes laterales)
#define IR_FRONT_REQUIRE_BOTH  1   // logs: FR solo ~130 en (7,16)/(3,16) = pared falsa
// 0 = diff = OFF-ON (semántica original; el esquemático confirma la polaridad:
//     fototransistor a GND con pull-up 10k => más luz reflejada = menos voltaje)
// 1 = diff = |OFF-ON| (opción del sketch de pruebas S3)
#define IR_USE_ABS_DIFF  0

//TEST TH
//#define IR_WALL_THR_L  70    // umbral pared lateral izquierda
//#define IR_WALL_THR_R  80    // umbral pared lateral derecha
//#define IR_WALL_THR_C  80    // umbral pared frontal

// ── CENTRADO/FRENADO POR IR (sin tocar la pared) ──────────────
// Un sensor IR reflectivo (emisor+fototransistor) como este tiene 3 zonas:
//   1) RUIDO / SIN PARED   (diff < IR_WALL_THR_*): señal indistinguible del
//      ruido electrónico. Ya se usa bien: es el umbral de "hay pared".
//   2) ZONA BUENA           (diff entre el umbral y ~el TARGET): la señal
//      sube de forma repetible al acercarse. Es la ÚNICA zona donde el valor
//      analógico sirve para estimar distancia (frenado proporcional, centrado).
//   3) SATURACIÓN CERCANA   (diff muy por encima del TARGET): pegado a la
//      pared el fototransistor satura, la lectura deja de ser confiable
//      (puede hasta bajar). Si se llega aquí para decidir "qué tan cerca
//      estoy" ya es tarde — por eso antes el robot terminaba tocando la pared.
//
// IR_TARGET_L/R/C YA EXISTÍAN en este archivo, definidos pero sin usar en
// ningún lado del original — son casi seguro un setpoint de calibración de
// una versión anterior (con el sensor central único, no FL/FR). Se reusan
// aquí como el valor de "pared a la distancia correcta" dentro de la zona
// buena, en vez de inventar números nuevos. OJO: vienen del sensor L/C/R
// viejo, no del hardware S3 de 4 sensores — son un PUNTO DE PARTIDA, hay
// que confirmarlos/ajustarlos en el robot real (procedimiento en el chat).
#define IR_FRONT_TARGET   IR_TARGET_C   // FL y FR comparten target, igual que comparten umbral

// Fracción del target a la que se empieza a bajar el PWM al aproximarse a
// una pared frontal (ESTIMADO, ajustar si frena muy brusco o muy temprano).
#define IR_FRONT_SLOW_FRAC     0.6f
// Margen de seguridad sobre el target: si la lectura lo supera por este
// factor, se asume saturación/demasiado cerca y se frena YA, sin esperar
// más control (ESTIMADO).
#define IR_FRONT_SAT_FACTOR    1.8f
// ALIGN sin llegar al pivote: si el nivel frontal ya es >= pivote * esto, se da por llegado (ya está en la pared).
#define ALIGN_ACCEPT_FRAC      0.75f

// Ganancias de las correcciones por IR. Signo sin verificar en el robot:
// si al acercarse a una pared frontal el robot gira HACIA ella en vez de
// alejarse (empeora en vez de mejorar), cambiar KP_IR_SQUARE_SIGN a -1.
// Mismo criterio para KP_IR_CENTER_SIGN si al centrar se va hacia la pared
// más cercana en vez de alejarse de ella.
#define KP_IR_SQUARE        0.10f
#define KP_IR_SQUARE_SIGN   (+1.0f)
// ── CENTRADO LATERAL POR IR (L/R) ─────────────────────────────
// IR_TARGET_L/R (235/265) venían del robot viejo: en el S3, con el robot en el pasillo,
// L y R leen ~580-720 mV (2.5x-2.7x esos valores), así que NO sirven como referencia.
// Ahora la referencia se MIDE en el robot: en la página IR, con el robot CENTRADO entre
// dos paredes, mantener pulsado => guarda L0 y R0 (NVS, namespace "ircal", aparte del
// mapa: borrar el mapa NO borra la calibración). Sin calibrar, el centrado vale 0.
//
// Error de centrado e = fR - fL, con f = lectura/lectura_centrada (compensa que L y R
// tengan distinta ganancia). Si falta una pared se asume f=1 en ese lado y la ganancia
// baja a la mitad. e>0 = más cerca de la derecha => hay que girar a la izquierda.
//
// Se aplica como un DESVÍO DEL RUMBO OBJETIVO (grados), no sumado al PWM: el lazo de
// rumbo tiene integral y habría cancelado un empuje sumado al PWM en ~0.2 s. Con la
// cascada (IR => rumbo => PWM) el robot se mueve lateralmente hasta centrarse y el desvío
// vuelve solo a 0.
#define IR_CENTER_ENABLE      1
#define KP_IR_CENTER_DEG      15.0f   // grados de desvío por unidad de e (e~0.5 => 7.5 deg)
#define IR_CENTER_MAX_DEG      8.0f   // tope del desvío de rumbo
#define IR_CENTER_MIN_FRAC     0.6f   // por debajo de f=0.6 se considera "pared lejana/borde": no se usa
#define IR_CENTER_MAX_FRAC     1.8f   // ESTIMADO: por encima, posible saturación: no se usa (medir!)
#define IR_CENTER_ONE_WALL_GAIN 0.5f
#define KP_IR_CENTER_SIGN    (+1.0f)

// Cada cuánto se refresca el centrado lateral dentro del lazo de control
// (lectura rápida y ruidosa de L/R; no se puede leer en cada tick de 10ms).
#define IR_CENTER_PERIOD_MS   20     // antes 40: a 85 PWM la ventana de paredes solo juntaba 2 lecturas
#define IR_CENTER_NSAMPLES    3     // IR_NSAMPLES=12 es para decisiones (scanWalls); esto es solo para centrar
// Muestras durante la aproximación/escuadrado a la pared frontal (más rápido
// que IR_NSAMPLES para no alargar la maniobra, más preciso que el centrado).
#define IR_ALIGN_NSAMPLES     6

// ── VALIDACIÓN DE PAREDES LATERALES (ventana) ──────────────────
// 1 = una pared lateral solo se marca si la lectura del escaneo la ve Y también la mayoría de
// las lecturas rápidas de L/R tomadas en los últimos IR_WALL_WINDOW_MM antes del punto de
// lectura (una pared es continua; un poste/esquina es corto). Sin lecturas extra: usa las del
// centrado. 0 = lectura única (comportamiento anterior). Activar tras calibrar el punto de lectura.
#define IR_WALL_CONFIRM        1   // logs: postes/finales de muro leían 81–213 (paredes falsas)
#define IR_WALL_WINDOW_MM      50.0f  // 20 mm daba 1-2 muestras (IR cada 40 ms): nunca llegaba al mínimo
#define IR_WALL_WINDOW_MIN     3      // mínimo de muestras en la ventana para usarla
// Lateral dudoso en la lectura temprana (TURN_MODE 2): por debajo de IR_SIDE_SURE se vuelve a leer tras avanzar
// IR_SIDE_RECHECK_MM en marcha (el poste de la esquina dio 142 y 554; paredes reales lejanas 150-450 siguen ahí).
// 250 (2026-10-21): con 600 casi todas las lecturas (300-500, paredes reales) se releían; la relectura retrasaba el arco
// a la derecha (hace falta s <= -85) y el robot giraba en el sitio en (3,7), donde luego perdía la posición. Los casos
// de 554 en tramos ya recorridos los cubre ff_wallAllowed().
#define IR_SIDE_SURE           250
#define IR_SIDE_RECHECK_MM     10.0f   // 15 => ~20 mm reales (la lectura también rueda)
#define IR_SIDE_RECHECK_MAX_MS 120

// ── SEGURIDAD DE MOVIMIENTO ──────────────────────────────────
#define FWD_TIMEOUT_MS         5000   // por tramo de avance
#define STALL_BACK_TICKS       75     // retroceso tras atasco (~18.8 mm)
#define STALL_MAX_RETRIES      1      // reintentos hacia el MISMO objetivo antes de detenerse
#define STALL_ACCEPT_MM        8.0f   // atasco a menos de esto del objetivo = llegó (tope físico)
// Diferencia máxima entre lo que avanzan las dos ruedas en un tramo recto. Normal en el log: <= ~11 mm.
// Más = una rueda bloqueada o patinando (choque): parada segura, la posición ya no es fiable.
#define WHEEL_DIVERGE_MM       30.0f
#define TURN_FAIL_DEG          25.0f  // error de rumbo final que se considera giro fallido (solo verifica)

// ── DIAGNÓSTICO SERIAL ───────────────────────────────────────
// 1 = una línea por evento de navegación (nunca dentro del lazo de 10 ms). 115200 baud.
#define NAV_DEBUG              1
// Log por WiFi: el ESP32 crea la red WIFI_AP_SSID; abre http://192.168.4.1 en el móvil.
// El log (16 KB) también sobrevive al reinicio tras "ERROR NAV". 0 = sin WiFi (solo Serial).
#define WIFI_LOG_ENABLE        1
#define WIFI_AP_SSID           "UMouse-S3"
#define WIFI_AP_PASS           "umouse123"   // mínimo 8 caracteres

#define IR_OPEN_THR   20  
#define IR_CLOSE_THR  600