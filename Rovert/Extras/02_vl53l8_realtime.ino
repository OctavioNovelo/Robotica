/*
 * PASO 2 - Lectura 8x8 en tiempo real del SATEL-VL53L8
 *
 * Muestra en el Monitor Serie una rejilla 8x8 con la distancia en mm
 * de cada zona, mas un mapa visual de densidad.
 *
 * Librerias necesarias (Gestor de Librerias de Arduino):
 *   - "VL53L8CX" de STMicroelectronics
 *
 * Monitor Serie a 115200 baudios.
 */

#include <Wire.h>
#include <vl53l8cx.h>

#define SDA_PIN 21
#define SCL_PIN 22

// LPn esta atado por hardware al riel de 1.8V, no lo controlamos por GPIO.
#define LPN_PIN  -1
#define I2C_RST_PIN -1

VL53L8CX sensor(&Wire, LPN_PIN, I2C_RST_PIN);
VL53L8CX_ResultsData resultados;

// Cuantos frames por segundo pedimos al sensor.
// 15 Hz es un buen punto de partida para 8x8.
const uint8_t FRECUENCIA_HZ = 15;

void setup() {
  Serial.begin(115200);
  delay(1500);

  Serial.println();
  Serial.println("=== VL53L8 - lectura 8x8 en tiempo real ===");

  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000);

  sensor.begin();

  if (sensor.init() != 0) {
    Serial.println("ERROR: no se pudo inicializar el sensor.");
    Serial.println("Correr primero el escaner I2C y verificar 0x29.");
    while (1) delay(1000);
  }

  sensor.set_resolution(VL53L8CX_RESOLUTION_8X8);
  sensor.set_ranging_frequency_hz(FRECUENCIA_HZ);
  sensor.start_ranging();

  Serial.println("Sensor listo. Iniciando medicion...");
  delay(1000);
}

// Devuelve un caracter segun que tan cerca esta el objeto.
char simboloPorDistancia(int mm) {
  if (mm < 200)  return '#';   // muy cerca
  if (mm < 500)  return 'O';
  if (mm < 1000) return 'o';
  if (mm < 2000) return '+';
  return '.';                  // lejos
}

void loop() {
  uint8_t listo = 0;
  sensor.check_data_ready(&listo);

  if (!listo) {
    delay(5);
    return;
  }

  sensor.get_ranging_data(&resultados);

  // Secuencia ANSI para limpiar pantalla y volver al inicio.
  // Si tu terminal no la soporta, veras basura: comenta estas 2 lineas
  // y descomenta el Serial.println() de separador mas abajo.
  Serial.print("\033[2J");
  Serial.print("\033[H");
  // Serial.println("--------------------------------------------");

  Serial.println("Mapa 8x8 (distancia en mm):");
  Serial.println();

  for (int fila = 0; fila < 8; fila++) {
    // Primero la linea con los numeros
    for (int col = 0; col < 8; col++) {
      int idx = fila * 8 + col;
      int estado = resultados.target_status[idx];
      int mm = resultados.distance_mm[idx];

      if (estado == 5 || estado == 9) {
        // 5 = medicion valida, 9 = valida pero con poca señal
        if (mm < 1000) Serial.print(" ");
        if (mm < 100)  Serial.print(" ");
        Serial.print(mm);
        Serial.print(" ");
      } else {
        Serial.print("---- ");
      }
    }

    // Luego el mapa visual de esa misma fila
    Serial.print("   |  ");
    for (int col = 0; col < 8; col++) {
      int idx = fila * 8 + col;
      int estado = resultados.target_status[idx];

      if (estado == 5 || estado == 9) {
        Serial.print(simboloPorDistancia(resultados.distance_mm[idx]));
      } else {
        Serial.print(' ');
      }
      Serial.print(' ');
    }
    Serial.println();
  }

  Serial.println();
  Serial.println("#=muy cerca  O=cerca  o=medio  +=lejos  .=muy lejos  ----=sin lectura");
}
