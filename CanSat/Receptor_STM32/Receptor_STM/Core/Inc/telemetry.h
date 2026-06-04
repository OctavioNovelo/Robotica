/*
 * telemetry.h
 *
 *  Created on: Mar 25, 2026
 *      Author: gashadokuro
 */

#ifndef TELEMETRY_H
#define TELEMETRY_H

#include <stdint.h>

#pragma pack(push, 1)   // ← CRÍTICO: sin padding entre campos

typedef struct
{
    uint8_t  magic;        // 0xCA
    uint8_t  pkt;          // número de secuencia

    int16_t  temperatura;  // °C enteros
    int16_t altitud;      // metros
    uint16_t presion;      // hPa

    uint8_t  verificacion; // flag
    uint8_t  checksum;     // XOR de todos los bytes anteriores

} TelemetryPacketLoRa;    // 10 bytes total

typedef struct {
    float accel_x, accel_y, accel_z;
    float giro_x,  giro_y,  giro_z;
} PacketLoRaBNO;           // 24 bytes (no se envía aún)

#pragma pack(pop)

void telemetry_build_LoRa(TelemetryPacketLoRa *p,
                           int16_t temperatura, uint16_t altitud,
                           uint16_t presion,    uint8_t verificacion);

void telemetry_LoRa_BNO(PacketLoRaBNO *p,
                         float ax, float ay, float az,
                         float gx, float gy, float gz);

#endif
