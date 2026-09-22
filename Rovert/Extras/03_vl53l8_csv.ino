/*
 * PASO 3 - Salida CSV para el visualizador web
 *
 * Emite una linea por frame con este formato:
 *   F,d0:s0,d1:s1,...,d63:s63
 * donde d = distancia en mm y s = target_status de esa zona.
 *
 * Abrir "visualizador.html" en Chrome/Edge para ver la rejilla en vivo.
 * NO tener el Monitor Serie de VS Code abierto al mismo tiempo:
 * el puerto solo admite un cliente a la vez.
 */

#include <Wire.h>
#include <vl53l8cx.h>

#define SDA_PIN 21
#define SCL_PIN 22

#define LPN_PIN     -1
#define I2C_RST_PIN -1

VL53L8CX sensor(&Wire, LPN_PIN, I2C_RST_PIN);
VL53L8CX_ResultsData resultados;

const uint8_t FRECUENCIA_HZ = 15;

void setup() {
  Serial.begin(115200);
  delay(1500);

  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000);

  sensor.begin();

  if (sensor.init() != 0) {
    // Marcamos el error de forma que el visualizador lo pueda mostrar.
    while (1) {
      Serial.println("E,init_failed");
      delay(1000);
    }
  }

  sensor.set_resolution(VL53L8CX_RESOLUTION_8X8);
  sensor.set_ranging_frequency_hz(FRECUENCIA_HZ);
  sensor.start_ranging();
}

void loop() {
  uint8_t listo = 0;
  sensor.check_data_ready(&listo);

  if (!listo) {
    delay(5);
    return;
  }

  sensor.get_ranging_data(&resultados);

  Serial.print("F");
  for (int i = 0; i < 64; i++) {
    Serial.print(",");
    Serial.print(resultados.distance_mm[i]);
    Serial.print(":");
    Serial.print(resultados.target_status[i]);
  }
  Serial.println();
}
