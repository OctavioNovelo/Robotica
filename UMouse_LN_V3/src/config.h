#pragma once
#include "settings.h"

// =============================================================
//  UMouse — config.h
//  Derivado automáticamente de PROFILE en settings.h
// =============================================================

#if PROFILE == 1
  // PRUEBA
  #define MAZE_ROWS   7
  #define MAZE_COLS   15
  #define GOAL_ROW    2
  #define GOAL_COL    0 
  #define MOUSE_ROW   4
  #define MOUSE_COL   14
  // Hacia dónde MIRA el robot al arrancar (convención de maus.h, coords = fila, columna):
  //   1 = Norte (fila-1)  2 = Este (col+1)  3 = Sur (fila+1)  4 = Oeste (col-1)
  #define MOUSE_START_DIRECTION  1

#elif PROFILE == 2
  // COMPETENCIA
  #define MAZE_ROWS   9
  #define MAZE_COLS   19
  #define GOAL_ROW    7
  #define GOAL_COL    4
  #define MOUSE_ROW   8
  #define MOUSE_COL   18
  #define MOUSE_START_DIRECTION  1

#else
  // ROBGAM
  #define MAZE_ROWS   18
  #define MAZE_COLS   18
  #define GOAL_ROW    6
  #define GOAL_COL    6
  #define MOUSE_ROW   12
  #define MOUSE_COL   0
  #define MOUSE_START_DIRECTION  1
#endif

#define WALL_ROWS  (2 * MAZE_ROWS - 1)
#define WALL_COLS  (2 * MAZE_COLS - 1)

// Filas 0..MAZE_ROWS-1 y columnas 0..MAZE_COLS-1 (se cuenta desde 0). Fuera de rango, FloodFill escribe fuera del mapa.
static_assert(GOAL_ROW  >= 0 && GOAL_ROW  < MAZE_ROWS && GOAL_COL  >= 0 && GOAL_COL  < MAZE_COLS, "META fuera del laberinto (filas/columnas empiezan en 0)");
static_assert(MOUSE_ROW >= 0 && MOUSE_ROW < MAZE_ROWS && MOUSE_COL >= 0 && MOUSE_COL < MAZE_COLS, "INICIO fuera del laberinto (filas/columnas empiezan en 0)");
static_assert(MOUSE_START_DIRECTION >= 1 && MOUSE_START_DIRECTION <= 4, "MOUSE_START_DIRECTION: 1=N 2=E 3=S 4=O");
