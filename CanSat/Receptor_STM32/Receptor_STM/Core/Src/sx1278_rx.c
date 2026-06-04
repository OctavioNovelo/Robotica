/*
 * sx1278.c
 *
 *  Created on: Mar 25, 2026
 *      Author: gashadokuro
 */
#include "main.h"
#include "sx1278_tx.h"
#include "sx1278_rx.h"

/* Reutiliza sx1278_write / sx1278_read / sx1278_reset del transmisor */

void sx1278_rx_init()
{
    sx1278_reset();

    // sleep
    sx1278_write(0x01, 0x80);

    HAL_Delay(10);

    // LoRa standby
    sx1278_write(0x01, 0x81);

    // frecuencia 433 MHz
    sx1278_write(0x06, 0x6C);
    sx1278_write(0x07, 0x80);
    sx1278_write(0x08, 0x00);

    // BW 125kHz SF7 CR 4/5
    sx1278_write(0x1D, 0x72);
    sx1278_write(0x1E, 0x74);


    // PA config
    sx1278_write(0x09, 0x8F);

    // LNA
    sx1278_write(0x0C, 0x23);

    // FIFO base
    sx1278_write(0x0E, 0x00);
    sx1278_write(0x0F, 0x00);

    // idk
    // DIO0 = TX_DONE
    sx1278_write(0x40, 0x00);


    // preamble
    sx1278_write(0x20, 0x00);
    sx1278_write(0x21, 0x08);

    // sync word
    sx1278_write(0x39, 0x34);

    sx1278_write(0x0E, 0x00); // TX base addr
    sx1278_write(0x0F, 0x00); // RX base addr

    sx1278_write(0x33, 0x27);
    sx1278_write(0x3B, 0x1D);
}

/* Retorna 1 si hay un paquete listo, 0 si no */
uint8_t sx1278_rx_available(void)
{
    uint8_t irq = sx1278_read(0x12); // RegIrqFlags
    return (irq & 0x40) ? 1 : 0;    // bit 6 = RxDone
}

/* Lee el payload. Retorna bytes leídos, 0 si CRC error o nada */
uint8_t sx1278_rx_read(uint8_t *buf, uint8_t max_len)
{
    uint8_t irq = sx1278_read(0x12);
    if (!(irq & 0x40)) return 0;

    if (irq & 0x20) {
        sx1278_write(0x12, 0xFF);
        sx1278_write(0x01, 0x85);
        return 0;
    }

    uint8_t nb_bytes = sx1278_read(0x13);

    // ✅ Siempre leer desde base 0x00 en lugar de RegFifoRxCurrentAddr
    sx1278_write(0x0D, 0x00);

    uint8_t to_read = (nb_bytes < max_len) ? nb_bytes : max_len;
    for (uint8_t i = 0; i < to_read; i++)
        buf[i] = sx1278_read(0x00);

    sx1278_write(0x12, 0xFF);
    sx1278_write(0x01, 0x85);
    return to_read;
}

/* RSSI del último paquete (dBm) */
int8_t sx1278_rx_rssi(void)
{
    return (int8_t)(sx1278_read(0x1A) - 164); // RegPktRssiValue, HF port
}

/* SNR del último paquete */
int8_t sx1278_rx_snr(void)
{
    return (int8_t)sx1278_read(0x19) / 4;
}

