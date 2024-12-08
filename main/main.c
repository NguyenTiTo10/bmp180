#include "stdio.h"
#include "esp_log.h"
#include "driver/i2c.h"
#include "drv_bmp180.h"
#include "bsp_timer.h"
// I2C Configuration
#define I2C_MASTER_SCL_IO           22  // GPIO number for SCL
#define I2C_MASTER_SDA_IO           21  // GPIO number for SDA
#define I2C_MASTER_NUM              I2C_NUM_0
#define I2C_MASTER_FREQ_HZ          400000  // 100kHz



// I2C Initialization
esp_err_t i2c_master_init() 
{
    i2c_config_t conf = 
    {
      .mode = I2C_MODE_MASTER,
      .sda_io_num = I2C_MASTER_SDA_IO,
      .scl_io_num = I2C_MASTER_SCL_IO,
      .sda_pullup_en = GPIO_PULLUP_ENABLE,
      .scl_pullup_en = GPIO_PULLUP_ENABLE,
      .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };
    
    i2c_param_config(I2C_MASTER_NUM, &conf);                                      
    return i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);   
}


// Main task to initialize and read data from BMP180
void app_main(void) 
{
    esp_err_t ret = i2c_master_init();
    if (ret != ESP_OK) 
      ESP_LOGE("I2C", "I2C initialization failed!");
  

    while (1)
    {
        

        bsp_timer_delay(2000);
    }
    
    
}
