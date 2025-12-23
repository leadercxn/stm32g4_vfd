#ifndef VIRT_I2C_H__
#define VIRT_I2C_H__

void virt_i2c_start(void);
void virt_i2c_stop(void);
void virt_i2c_wait_ack(void);
void virt_i2c_ack(void);
void virt_i2c_nack(void);
void virt_i2c_send_byte(uint8_t txdata);
uint8_t virt_i2c_read_byte(uint8_t ack);


void virt_i2c_io_init(void);

#endif

