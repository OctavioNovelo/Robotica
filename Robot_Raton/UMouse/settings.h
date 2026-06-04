#pragma once

// =============================================================
//  UMouse — settings.h (Versión ESP32-S3)
// =============================================================

#define PROFILE  1

// ── PINES I2C (Para OLED y Sensores) ─────────────────────────
#define I2C_SDA          41  //
#define I2C_SCL          42  //

// ── GEOMETRÍA ────────────────────────────────────────────────
#define WHEEL_DIAM_MM    33.5f   
#define TRACK_WIDTH_MM   90.0f  

// ── ENCODER ──────────────────────────────────────────────────
#define ENC_PPR_MOTOR    7      
#define ENC_GEAR_RATIO   30     
#define ENC_EDGES        2      

#define ENC_TICKS_PER_REV  ((float)(ENC_PPR_MOTOR * ENC_GEAR_RATIO * ENC_EDGES))
#define WHEEL_CIRC_MM      ((float)(3.14159265f * WHEEL_DIAM_MM))
#define TICKS_PER_MM       ((float)(ENC_TICKS_PER_REV / WHEEL_CIRC_MM))

// ── LABERINTO ────────────────────────────────────────────────
#if PROFILE == 1
  #define CELL_MM   160.0f   
  #define PRESHIFT_MM   17.5f
  #define INITIAL_ADVANCE_MM  17.5f
#else
  #define CELL_MM   190.0f   
  #define PRESHIFT_MM   17.5f
  #define INITIAL_ADVANCE_MM  65.0f
#endif

#define CELL_FWD_MM    ((float)(CELL_MM - PRESHIFT_MM))
#define ENC_CELL_FWD   ((int)(CELL_FWD_MM * TICKS_PER_MM))
#define ENC_PRESHIFT   ((int)(PRESHIFT_MM * TICKS_PER_MM))

#define TURN90_ARC_MM  ((float)(3.14159265f * TRACK_WIDTH_MM / 4.0f))
#define ENC_TURN90_CALC ((int)(TURN90_ARC_MM * TICKS_PER_MM))

#undef  ENC_TURN90_CALC
#define ENC_TURN90_CALC  255   

#define ENC_TURN90  ENC_TURN90_CALC
#define ENC_INITIAL_ADVANCE  ((int)(INITIAL_ADVANCE_MM * TICKS_PER_MM))

// ── MOTORES ──────────────────────────────────────────────────
#define V_MOTOR_LIMIT   6.0f   
#define MOTOR_KICK_MIN  30     
#define FWD_PWM         45

#define FWD_VOLTS       3.1f
#define TURN_VOLTS      3.1f
#define ALIGN_VOLTS     2.2f   
#define ALIGN_MS        350    

#define ENC_SIGN_L   (+1)
#define ENC_SIGN_R   (-1)
#define ENC_TRIM_R   1.00f 

// ── CONTROL DIFERENCIAL ──────────────────────────────────────
#define KP_ENC      0.30f
#define KI_ENC      0.13f
#define KI_ENC_MAX  7.0f

// ── DETECTOR DE ATASCO ───────────────────────────────────────
#define MIN_TICKS_PER_MS  0.20f  

// ── SENSORES IR (ESP32-S3) ───────────────────────────────────
#define IR_WALL_THR_L   70    //
#define IR_WALL_THR_R   80    //
#define IR_WALL_THR_FL  80    //
#define IR_WALL_THR_FR  80    //

#define IR_OPEN_THR   20  
#define IR_CLOSE_THR  600
