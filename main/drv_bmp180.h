#ifndef DRV_BMP180_H
#define DRV_BMP180_H

#include "stdio.h"
#include "esp_log.h"
#include "driver/i2c.h"
#include "portmacro.h"

#include "bsp_i2c.h"
#include "bsp_timer.h"


typedef enum
{
  DRV_BMP180_ERROR = 0,
  DRV_BMP180_OK
} drv_bmp180_ret_t;


// Function to read calibration data from BMP180
drv_bmp180_ret_t drv_bmp180_read_calibration();

// Read raw temperature data from BMP180
esp_err_t bmp180_read_raw_temperature(int32_t *raw_temp);

// Read raw pressure data from BMP180
esp_err_t bmp180_read_raw_pressure(int32_t *raw_press);

// Temperature compensation function
int32_t bmp180_calculate_temperature(int32_t raw_temp);

// Pressure compensation function
int32_t bmp180_calculate_pressure(int32_t raw_press, int32_t temp);


#endif