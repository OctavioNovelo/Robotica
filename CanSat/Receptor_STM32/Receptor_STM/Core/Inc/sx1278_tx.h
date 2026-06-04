/*
 * sx1278_tx.h
 *
 *  Created on: Mar 25, 2026
 *      Author: gashadokuro
 */

#ifndef INC_SX1278_TX_H_
#define INC_SX1278_TX_H_

void sx1278_tx_init(void);
void sx1278_send(uint8_t *data, uint8_t len);
void sx1278_onDio0Irq(void);
void sx1278_reset();
uint8_t sx1278_read(uint8_t reg);
void sx1278_write(uint8_t reg, uint8_t value);
uint8_t sx1278_check(void);

#endif /* INC_SX1278_TX_H_ */
