#ifndef UMOUSE_S3_TEST_TYPES_H
#define UMOUSE_S3_TEST_TYPES_H

#include <Arduino.h>

// Este archivo evita que el preprocesador de Arduino genere prototipos
// antes de conocer los tipos personalizados del sketch.
struct IRReading {
  const char *name;
  uint8_t ledPin;
  uint8_t ftPin;
  int threshold;
  int offmV;
  int onmV;
  int diffSigned;
  int diffUsed;
  bool wall;
};

enum Page : uint8_t {
  PAGE_STATUS = 0,
  PAGE_IR_DIFF,
  PAGE_IR_RAW,
  PAGE_WALLS,
  PAGE_ENCODERS,
  PAGE_BNO,
  PAGE_MOTOR,
  PAGE_COUNT
};

#endif
