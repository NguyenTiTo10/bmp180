#ifndef BSP_I2C_H
#define BSP_I2C_H

#include "driver/i2c.h"
#include "esp_err.h"

bool bsp_i2c_is_device_ready(uint16_t dev_addr);
bool bsp_i2c_write_mem(uint16_t dev_addr, uint16_t mem_addr,  uint8_t *data, size_t length);



#endif