#include <RadioLib.h>

SX1278 radio = new Module(5, 26, 14, 27);

enum Mode {
  MODE_LORA,
  MODE_FSK
};

Mode currentMode = MODE_LORA;

int telemetryCount = 0;
int imageCount = 0;

// Modos
void initLoRa() {

  // frecuencia en MHz
  // bw ?
  // sf ?
  // cr ?
  // sync
  // potencia
  // preamble ?
  // gain ?
  int state = radio.begin(434.0, 125.0, 7, 5, 0x34, 17, 8);

  if (state != RADIOLIB_ERR_NONE) {
    Serial.println("Error LoRa");
    while(true);
  }
  
  Serial.println("Modo: LoRa: Envio Telemetria... ");
}

void initFSK() {

  // frecuencia en MHz
  // bitrate
  // freqDev (desviacion)
  // rxbw ?
  // potencia
  // preamble ? 
  // gain ?
  int state = radio.beginFSK(434.0, 50.0, 50.0, 125.0, 17, 16);

  if (state != RADIOLIB_ERR_NONE) {
    Serial.println("Error FSK");
    while(true);
  }

  Serial.println("Modo FSK: Imagen");
}

// Informacion de los sensores

void bme280() {

  // Wire.beginTransmission(...)
  // Leer registros BME280
  // Convertir a temperatura/presión reales

  float temp = 20 + random(-5,5);
  float pressure = 1013 + random(-10,10);

  String packet = "Temperatura = " + String(temp) + "Presion = " + String(pressure);

  radio.transmit(packet); //SX127x.cpp
  Serial.println(packet);
}

void bno085() {

  // Lectura real IMU por I2C/SPI
  // Obtener aceleraciones

  float ax = random(-10,10);
  float ay = random(-10,10);

  String packet = "IMU|AX=" + String(ax) + "|AY=" + String(ay);

  radio.transmit(packet);
  Serial.println(packet);
}

void esp32_image() {

  // UART con ESP32-CAM
  // Recepción de fragmento JPEG real

  String fragment = "Imagen = " + String(imageCount);

  radio.transmit(fragment);
  Serial.println(fragment);

  imageCount++;
}

// ===================================================================================================================================================

void setup() {
  
  Serial.begin(115200);
  delay(2000);

  initLoRa();
}


// ===================================================================================================================================================

void loop() {

  if (currentMode == MODE_LORA) {

    bme280();
    delay(500);

    bno085();
    delay(500);

    telemetryCount++;

    if (telemetryCount >= 5) {
      
      Serial.println(">>> Imagen >>>");
      initFSK();
      currentMode = MODE_FSK;
      imageCount = 0;
    }
  }

  else if (currentMode == MODE_FSK) {

    esp32_image();
    delay(200);

    if (imageCount >= 20) {

      Serial.println(">>> Telemetria >>>");
      initLoRa();
      currentMode = MODE_LORA;
      telemetryCount = 0;
    }
  }
}
