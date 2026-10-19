#pragma once

// =============================================================
//  UMouse S3 — navgeom.h
//  Modelo longitudinal ÚNICO de avance / lectura / giro (sin Arduino: se prueba en PC,
//  ver test/host). Reemplaza al antiguo g_skipPreshift.
//
//  s = posición del EJE de las ruedas respecto al CENTRO de la celda lógica actual,
//      a lo largo del rumbo actual, en mm odométricos (+ = hacia delante).
//
//    C = paso de celda (CELL_FWD_MM)          T = punto de giro (TURN_POINT_OFFSET_MM)
//    P = lectura antes del giro (READ_BACK_MM)  punto de lectura = T - P
//
//  Avance a la celda siguiente  : C + (T - P) - s     (recto tras recto = C; tras giro 90° = C - P + T)
//  Preshift antes de girar      : T - s (>= 0); 0 si ya está en el punto de giro
//  Giro 90° en el punto de giro : el offset longitudinal pasa a ser lateral => s = 0
//  2.º giro (180° total)        : sin preshift; s = -pivotS
// =============================================================

struct NavGeom {
  float C = 0.0f, P = 0.0f, T = 0.0f;
  float s = 0.0f;          // posición del eje (ver arriba)
  bool  atPivot = false;   // el eje está en el punto de giro (no avanzó desde que llegó)
  float pivotS  = 0.0f;    // s en el momento de fijar el pivote (en el rumbo de llegada)
  int   net     = 0;       // giros netos (en cuartos de vuelta, 0..3) desde que llegó al pivote

  void  init(float c, float p, float t){ C = c; P = p; T = t; arrivedRead(); }
  float readS() const       { return T - P; }
  float fwdDist() const     { return C + readS() - s; }
  void  arrivedRead()       { s = readS(); atPivot = false; net = 0; }
  float preshiftDist() const{ if(atPivot) return 0.0f; float d = T - s; return d > 0.0f ? d : 0.0f; }
  void  advance(float mm)   { s += mm; }
  void  setPivot()          { if(!atPivot){ atPivot = true; pivotS = s; net = 0; } }
  void  afterTurn(int dir){
    net = ((net + dir) % 4 + 4) % 4;
    s = (net == 0) ? pivotS : (net == 2) ? -pivotS : 0.0f;
  }
};
