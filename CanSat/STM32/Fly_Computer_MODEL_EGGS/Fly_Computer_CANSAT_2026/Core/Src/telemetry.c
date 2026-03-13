/*
 * telemetry.c
 *
 *  Created on: Mar 13, 2026
 *      Author: shuten-doji
 */

#include "telemetry.h"

static uint8_t seq = 0;

static uint8_t telemetry_checksum(uint8_t *data, uint8_t len)
{
    uint8_t c = 0;

    for(int i = 0; i < len; i++)
        c ^= data[i];

    return c;
}

void telemetry_build(TelemetryPacket *pkt,
                     int16_t altitude,
                     uint16_t pressure,
                     int8_t temp)
{
    pkt->magic = 0xCA;

    pkt->seq = seq++;

    pkt->temp = temp + 40;

    pkt->altitude = altitude;

    pkt->pressure = pressure;

    pkt->checksum = telemetry_checksum((uint8_t*)pkt,7);
}
