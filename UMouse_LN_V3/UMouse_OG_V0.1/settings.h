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

//TEST TH
//#define IR_WALL_THR_L  70    // umbral pared lateral izquierda
//#define IR_WALL_THR_R  80    // umbral pared lateral derecha
//#define IR_WALL_THR_C  80    // umbral pared frontal

#define IR_OPEN_THR   20  
#define IR_CLOSE_THR  600
