/*
 * sx1278.h
 *
 *  Created on: Mar 25, 2026
 *      Author: gashadokuro
 */

#ifndef SX1278_RX_H
#define SX1278_RX_H

#include <stdint.h>

void    sx1278_rx_init(void);
uint8_t sx1278_rx_available(void);
uint8_t sx1278_rx_read(uint8_t *buf, uint8_t max_len);
int8_t  sx1278_rx_rssi(void);
int8_t  sx1278_rx_snr(void);

#endif
