#ifndef I2C_H__
#define I2C_H__

#include "boards.h"

extern I2C_HandleTypeDef g_i2c3_handle;

int i2c3_init(void);

uint8_t i2c3_at24cxx_one_byte_read(uint16_t reg);
void i2c3_at24cxx_one_byte_write(uint16_t reg, uint8_t txdata);
void i2c3_at24cxx_read(uint16_t reg, uint8_t *p_rx_data, uint16_t len);
void i2c3_at24cxx_write(uint16_t reg, uint8_t *p_tx_data, uint16_t len);

#endif