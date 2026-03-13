/*
 * telemetry.h
 *
 *  Created on: Mar 13, 2026
 *      Author: shuten-doji
 */
#ifndef TELEMETRY_H
#define TELEMETRY_H

#include <stdint.h>

#pragma pack(push,1)
typedef struct
{
    uint8_t magic;
    uint8_t seq;
    uint8_t temp;
    int16_t altitude;
    uint16_t pressure;
    uint8_t checksum;

} TelemetryPacket;
#pragma pack(pop)

void telemetry_build(TelemetryPacket *pkt,
                     int16_t altitude,
                     uint16_t pressure,
                     int8_t temp);

#endif
