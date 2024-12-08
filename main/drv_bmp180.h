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


typedef struct 
{
  temputure,
  pressure
} bmp180_struct_t;


drv_bmp180_ret_t drv_bmp180_init();

drv_bmp180_ret_t drv_bmp180_start_read ();




#endif