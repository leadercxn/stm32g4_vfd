#ifndef SPI_H__
#define SPI_H__

#include "boards.h"

extern SPI_HandleTypeDef g_spi1_handle;

int spi1_init(void);

uint8_t spi1_one_byte_wr(uint8_t txdata);
int spi1_bytes_wr(uint8_t *p_tx_data, uint8_t *p_rx_data, uint32_t len);

void w25nxx_select(void);
void w25nxx_disselect(void);
#endif

