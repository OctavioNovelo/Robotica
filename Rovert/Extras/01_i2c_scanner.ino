/*
 * PASO 1 - Escaner I2C
 * Objetivo: confirmar que el SATEL-VL53L8 responde antes de intentar
 * inicializarlo. Debe aparecer el dispositivo en la direccion 0x29.
 *
 * Hardware:
 *   ESP32 GPIO22 -> HV1 del level shifter -> LV1 -> SCL del SATEL
 *   ESP32 GPIO21 -> HV2 del level shifter -> LV2 -> SDA del SATEL
 *   LPn y NCS del SATEL atados al riel de 1.8V
 *   I2C_N (SPI_I2C_n) del SATEL atado a GND
 */

#include <Wire.h>

#define SDA_PIN 21
#define SCL_PIN 22

void setup() {
  Serial.begin(115200);
  delay(1500);

  Serial.println();
  Serial.println("=== Escaner I2C ===");

  Wire.begin(SDA_PIN, SCL_PIN);
  // Arrancamos lento a proposito: si hay ruido o cables largos,
  // 100 kHz es mucho mas tolerante que 400 kHz.
  Wire.setClock(100000);
}

void loop() {
  uint8_t encontrados = 0;

  Serial.println("Escaneando bus...");

  for (uint8_t dir = 0x08; dir <= 0x77; dir++) {
    Wire.beginTransmission(dir);
    uint8_t error = Wire.endTransmission();

    if (error == 0) {
      Serial.print("  Dispositivo encontrado en 0x");
      if (dir < 16) Serial.print("0");
      Serial.print(dir, HEX);

      if (dir == 0x29) {
        Serial.print("   <-- VL53L8 (correcto)");
      }
      Serial.println();
      encontrados++;
    }
  }

  if (encontrados == 0) {
    Serial.println("  Nada encontrado.");
    Serial.println("  Revisar: alimentacion 1.8V, HV/LV del level shifter,");
    Serial.println("  continuidad de las soldaduras SCL y SDA, GND comun.");
  } else {
    Serial.print("  Total: ");
    Serial.println(encontrados);
  }

  Serial.println();
  delay(3000);
}
