#pragma once
#include "settings.h"

// =============================================================
//  UMouse — config.h
//  Derivado automáticamente de PROFILE en settings.h
// =============================================================

#if PROFILE == 1
  // Laberinto de prueba 1x6
  #define MAZE_ROWS   1
  #define MAZE_COLS   6
  #define GOAL_ROW    0
  #define GOAL_COL    0
  #define MOUSE_ROW   0
  #define MOUSE_COL   5
  #define MOUSE_START_DIRECTION  4

  /*
  #define MAZE_ROWS   13
  #define MAZE_COLS   13
  #define GOAL_ROW    6
  #define GOAL_COL    6
  #define MOUSE_ROW   12
  #define MOUSE_COL   0
  #define MOUSE_START_DIRECTION  1
  */

#else
  // Laberinto de competencia 3x6
  #define MAZE_ROWS   3
  #define MAZE_COLS   5
  #define GOAL_ROW    0
  #define GOAL_COL    0
  #define MOUSE_ROW   2
  #define MOUSE_COL   4
  #define MOUSE_START_DIRECTION  1
#endif

#define WALL_ROWS  (2 * MAZE_ROWS - 1)
#define WALL_COLS  (2 * MAZE_COLS - 1)
