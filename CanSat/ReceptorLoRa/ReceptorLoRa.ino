/*
  RECEPTOR JPG MINIMO - Arduino Uno + SX1278
*/
#include <SPI.h>
#include <LoRa.h>

// Pines
#define NSS 10
#define RST 9
#define DIO0 2

// Variables
unsigned long tiempoInicio = 0;
unsigned long tiempoFin = 0;
unsigned long bytesRecibidos = 0;
unsigned long paquetesRecibidos = 0;

// Configuración (DEBE SER IGUAL AL TRANSMISOR)
const long FRECUENCIA = 434E6;
const int SF = 7;
const long BW = 125E3;
const int CR = 5;

// Buffer
#define MAX_PAQUETE 200
uint8_t buffer[MAX_PAQUETE + 4]; // +4 para offset

void setup() {
  Serial.begin(115200);
  while (!Serial);
  
  Serial.println(F("RECEPTOR JPG MINIMO - LoRa"));
  
  // Inicializar LoRa
  LoRa.setPins(NSS, RST, DIO0);
  
  if (!LoRa.begin(FRECUENCIA)) {
    Serial.println(F("Error iniciando LoRa!"));
    while (1);
  }
  
  // Misma configuración
  LoRa.setSpreadingFactor(SF);
  LoRa.setSignalBandwidth(BW);
  LoRa.setCodingRate4(CR);
  
  // Solo escuchar
  LoRa.receive();
  
  Serial.println(F("LoRa iniciado - Escuchando..."));
  Serial.print(F("Frec: ")); Serial.print(FRECUENCIA/1E6); Serial.println(F(" MHz"));
}

void loop() {
  // Verificar si hay paquetes
  int packetSize = LoRa.parsePacket();
  
  if (packetSize) {
    procesarPaquete(packetSize);
  }
}

void procesarPaquete(int packetSize) {
  if (!tiempoInicio) {
    tiempoInicio = millis();
    Serial.println(F("\n=== INICIANDO RECEPCION ==="));
  }
  
  // Leer paquete
  for (int i = 0; i < packetSize && i < sizeof(buffer); i++) {
    buffer[i] = LoRa.read();
  }
  
  // Extraer offset (primeros 4 bytes)
  unsigned long offset = 0;
  for (int i = 0; i < 4 && i < packetSize; i++) {
    offset = (offset << 8) | buffer[i];
  }
  
  // Datos reales (sin los 4 bytes de offset)
  int datosReales = packetSize - 4;
  if (datosReales > 0) {
    paquetesRecibidos++;
    bytesRecibidos += datosReales;
    
    // Mostrar progreso
    if (paquetesRecibidos % 5 == 0) {
      Serial.print(F("Paq: "));
      Serial.print(paquetesRecibidos);
      Serial.print(F(" Bytes: "));
      Serial.print(bytesRecibidos);
      Serial.print(F(" RSSI: "));
      Serial.print(LoRa.packetRssi());
      Serial.println(F(" dBm"));
    }
    
    // Aquí podrías guardar los datos en SD o procesarlos
    // Por ahora solo medimos
    
    // Verificar si debemos terminar (por timeout)
    tiempoFin = millis();
    if (tiempoFin - tiempoInicio > 30000) { // 30 seg timeout
      mostrarResultados();
    }
  }
}

void mostrarResultados() {
  unsigned long tiempoMs = tiempoFin - tiempoInicio;
  float tiempoSeg = tiempoMs / 1000.0;
  
  Serial.println(F("\n=== RESULTADOS RECEPCION ==="));
  Serial.print(F("Tiempo: "));
  Serial.print(tiempoSeg, 2);
  Serial.println(F(" seg"));
  
  Serial.print(F("Paquetes: "));
  Serial.println(paquetesRecibidos);
  
  Serial.print(F("Bytes: "));
  Serial.println(bytesRecibidos);
  
  if (tiempoSeg > 0) {
    float throughput = (bytesRecibidos * 8.0) / tiempoSeg;
    Serial.print(F("Throughput: "));
    Serial.print(throughput / 1000.0, 2);
    Serial.println(F(" kbps"));
  }
  
  // Resetear para siguiente
  tiempoInicio = 0;
  bytesRecibidos = 0;
  paquetesRecibidos = 0;
  
  Serial.println(F("\nListo para nueva recepcion..."));
}
