// =============================================================
//  telemetry.h — Con soporte de lectura y calibración (JSON)
// =============================================================
#pragma once
#include <WiFi.h>
#include <WebSocketsClient.h>
#include <ArduinoJson.h>  // <-- Añadimos esta librería para procesar las respuestas de la PC

#include "sensors.h"
#include "motion.h"

// Variables de red (Ajusta con tus datos)
const char* WIFI_SSID     = "TU_SSID_WIFI";       
const char* WIFI_PASSWORD = "TU_CONTRASEÑA";
const char* SERVER_HOST   = "192.168.1.100"; // IP de tu PC
const int   SERVER_PORT   = 8765;                 

WebSocketsClient webSocket;
bool g_wifiConnected = false;
bool g_wsConnected   = false;

// Función que se ejecuta AUTOMÁTICAMENTE cuando la PC le manda un JSON al robot
void webSocketEvent(WStype_t type, uint8_t * payload, size_t length) {
  switch(type) {
    case WStype_DISCONNECTED:
      g_wsConnected = false;
      break;
    case WStype_CONNECTED:
      g_wsConnected = true;
      break;
    case WStype_TEXT: {
      // 1. Reservar memoria para parsear el JSON recibido de la PC
      StaticJsonDocument<256> doc;
      DeserializationError error = deserializeJson(doc, payload, length);
      
      if (error) {
        // Error de parseo, ignoramos la trama
        return;
      }

      // 2. Leer las órdenes o calibraciones dinámicas de la IA
      if (doc.containsKey("cmd")) {
        const char* cmd = doc["cmd"];
        
        if (strcmp(cmd, "CALIBRATE_TRIM") == 0) {
          // La IA nos manda un nuevo factor de compensación para los motores en marcha recta
          if (doc.containsKey("trim_r")) {
            float nuevoTrim = doc["trim_r"];
            // Modificamos la variable global de motion.h en tiempo real
            ENC_TRIM_R = nuevoTrim; 
          }
          // La IA recalibró los umbrales de las paredes por cambios de luz
          if (doc.containsKey("thr_l")) {
            IR_WALL_THR_L = doc["thr_l"];
          }
        }
        
        else if (strcmp(cmd, "STOP_MOTOR") == 0) {
          // Un comando de emergencia directo de la IA para frenar el ratón
          extern void motorStop(); // Asumiendo que existe en motion.h
          motorStop();
        }
      }
      break;
    }
    default:
      break;
  }
}

void initTelemetry() {
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  uint32_t startAttempt = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - startAttempt < 3000) {
    delay(100);
  }
  if (WiFi.status() == WL_CONNECTED) {
    g_wifiConnected = true;
    webSocket.begin(SERVER_HOST, SERVER_PORT, "/");
    webSocket.onEvent(webSocketEvent);
    webSocket.setReconnectInterval(2000);
  }
}

void sendTelemetryToAI(const char* eventType) {
  if (!g_wsConnected) return;

  extern float g_yawDeg, g_pitchDeg, g_rollDeg, g_gyroZ, g_linAx, g_linAy;
  extern int offL, onL, irL, offR, onR, irR, offFL, onFL, irFL, offFR, onFR, irFR;
  
  // Recuperar los ticks absolutos actuales de los encoders
  long l, lA, r, rA; 
  getEncAll(l, lA, r, rA);

  char jsonBuffer[380];
  snprintf(jsonBuffer, sizeof(jsonBuffer),
    "{"
      "\"event\":\"%s\","
      "\"imu\":{\"y\":%.1f,\"p\":%.1f,\"r\":%.1f,\"gz\":%.2f,\"ax\":%.1f,\"ay\":%.1f},"
      "\"ir_diff\":{\"l\":%d,\"r\":%d,\"fl\":%d,\"fr\":%d},"
      "\"ir_raw\":{\"l_off\":%d,\"l_on\":%d,\"r_off\":%d,\"r_on\":%d,\"f_off\":%d,\"f_on\":%d},"
      "\"enc\":{\"l\":%ld,\"r\":%ld}"
    "}",
    eventType,
    g_yawDeg, g_pitchDeg, g_rollDeg, g_gyroZ, g_linAx, g_linAy,
    irL, irR, irFL, irFR,
    offL, onL, offR, onR, (offFL + offFR)/2, (onFL + onFR)/2,
    lA, rA
  );

  webSocket.sendTXT(jsonBuffer);
}
