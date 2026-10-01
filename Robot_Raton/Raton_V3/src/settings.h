#pragma once

// =============================================================
//  UMouse — settings.h
//  UN SOLO LUGAR para calibrar todo.
//  Cambia PROFILE para alternar entre prueba y competencia.
// =============================================================

// ── PERFIL ───────────────────────────────────────────────────
// Cambia SOLO esta línea:
//   1 = prueba (laberinto 3x7, celdas 14cm)
//   2 = competencia (laberinto 19x17, celdas 18cm)
#define PROFILE  1

// ── MODO DE NAVEGACIÓN ───────────────────────────────────────
// Cambia SOLO esta línea:
//   1 = sensores IR + encoders (lógica original probada; BNO solo informativo)
//   2 = BNO085 únicamente (sin IR, sin encoders). LIMITADO: ver abajo
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

// Con BNO (NAV_MODE 2/3), cada cuánto se refresca la OLED con el compás en
// vivo mientras el robot se mueve (ver navUiTick en motion.h).
#define NAV_UI_REFRESH_MS  120

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
#define HDG_DEADBAND_DEG  0.5f

// Giro cerrado por BNO
#define BNO_TURN_TOL_DEG        2.0f    // se detiene a ±tol (+ anticipación)
#define BNO_TURN_LEAD_S         0.05f   // anticipación de frenado = rate * lead
#define BNO_TURN_TIMEOUT_MS     2500
#define BNO_TURN_FINE_TOL_DEG   1.5f
#define BNO_TURN_FINE_TRIES     3
#define BNO_TURN_FINE_PULSE_MS  25
#define BNO_TURN_ENC_MAX_FACTOR 1.6f    // NAV_MODE 3: validación por encoder
// Tras alinear con pared frontal (NAV_MODE 3) el heading se re-referencia
// al rumbo nominal si el error es menor a este valor (0 = desactivado)
#define BNO_SNAP_MAX_DEG        12.0f

// NAV_MODE 2: distancia por TIEMPO (lazo abierto, NO es odometría).
// Derivado del comentario original: ~1750 ms para 142.5 mm a FWD_PWM=45, 12.6 V
// => 12.3 ms/mm. DEPENDE DE LA BATERÍA: calibrar en el robot.
#define NAV2_MS_PER_MM    12.3f

// ── GEOMETRÍA ────────────────────────────────────────────────
#define WHEEL_DIAM_MM    33.5f   // diámetro real de la rueda (mm)
#define TRACK_WIDTH_MM   90.0f  // distancia centro-rueda a centro-rueda (mm)

// ── ENCODER ──────────────────────────────────────────────────
#define ENC_PPR_MOTOR    7      // pulsos por rev en eje del motor
#define ENC_GEAR_RATIO   30     // reducción de la caja
#define ENC_EDGES        2      // 2=canal A solo, 4=cuadratura A+B

// Derivados — no tocar
#define ENC_TICKS_PER_REV  ((float)(ENC_PPR_MOTOR * ENC_GEAR_RATIO * ENC_EDGES))
#define WHEEL_CIRC_MM      ((float)(3.14159265f * WHEEL_DIAM_MM))
#define TICKS_PER_MM       ((float)(ENC_TICKS_PER_REV / WHEEL_CIRC_MM))

// ── LABERINTO ────────────────────────────────────────────────
#if PROFILE == 1
  #define CELL_MM   160.0f   // celda de prueba  14cm
#else
  #define CELL_MM   190.0f   // celda de competencia 18cm
#endif

// Preshift: distancia desde posición de reposo hasta eje de giro
// = mitad de celda - distancia eje rueda al centro robot
// Competencia: 90mm - 17.5mm = ~72mm... pero empíricamente 38mm funcionó
// Prueba 14cm: ajustar proporcionalmente o medir
#if PROFILE == 1
  #define PRESHIFT_MM   17.5f
#else
  #define PRESHIFT_MM   17.5f
#endif

// Avance real por celda = CELL_MM - PRESHIFT_MM
// El preshift del giro siguiente completa los últimos mm
#define CELL_FWD_MM    ((float)(CELL_MM - PRESHIFT_MM))
#define ENC_CELL_FWD   ((int)(CELL_FWD_MM * TICKS_PER_MM))
#define ENC_PRESHIFT   ((int)(PRESHIFT_MM * TICKS_PER_MM))

// Giro 90°: arco = π × track_width / 4
#define TURN90_ARC_MM  ((float)(3.14159265f * TRACK_WIDTH_MM / 4.0f))
#define ENC_TURN90_CALC ((int)(TURN90_ARC_MM * TICKS_PER_MM))

// Override manual del giro (ajustar por prueba física)
// Comenta estas dos líneas para usar el valor calculado
#undef  ENC_TURN90_CALC
#define ENC_TURN90_CALC  255   // valor calibrado empíricamente

#define ENC_TURN90  ENC_TURN90_CALC

// Avance inicial desde pared trasera al centro de primera celda
#if PROFILE == 1
  #define INITIAL_ADVANCE_MM  17.5f
#else
  #define INITIAL_ADVANCE_MM  65.0f
#endif
#define ENC_INITIAL_ADVANCE  ((int)(INITIAL_ADVANCE_MM * TICKS_PER_MM))

// ── MOTORES ──────────────────────────────────────────────────
#define V_MOTOR_LIMIT   6.0f   // voltaje máximo motores (V)
#define MOTOR_KICK_MIN  30     // PWM mínimo para romper estática (0-100)

// PWM de avance (0-100). 45 funcionó bien con batería 12.6V
#define FWD_PWM         45

// Voltaje objetivo para giros — normalizado por VBAT
// Ajustar hasta que giro sea exactamente 90°
#define FWD_VOLTS       3.1f
#define TURN_VOLTS      3.1f

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
#define IR_FRONT_REQUIRE_BOTH  0
// 0 = diff = OFF-ON (semántica original; el esquemático confirma la polaridad:
//     fototransistor a GND con pull-up 10k => más luz reflejada = menos voltaje)
// 1 = diff = |OFF-ON| (opción del sketch de pruebas S3)
#define IR_USE_ABS_DIFF  0

//TEST TH
//#define IR_WALL_THR_L  70    // umbral pared lateral izquierda
//#define IR_WALL_THR_R  80    // umbral pared lateral derecha
//#define IR_WALL_THR_C  80    // umbral pared frontal

#define IR_OPEN_THR   20  
#define IR_CLOSE_THR  600
