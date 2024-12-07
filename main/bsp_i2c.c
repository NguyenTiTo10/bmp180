#include "bsp_i2c.h"

#define I2C_MASTER_NUM I2C_NUM_0     // I2C port number


static i2c_cmd_handle_t cmd;

bool bsp_i2c_is_ready(void)
{
  return (cmd == NULL) ? false : true;  // Directly return the result of the condition
}

bool bsp_i2c_is_device_ready(uint16_t dev_addr) 
{
  cmd = i2c_cmd_link_create();
  if (cmd == NULL) 
      return false; 

  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, (dev_addr << 1) | I2C_MASTER_WRITE, true);
  i2c_master_stop(cmd);

  esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS);
  i2c_cmd_link_delete(cmd);

  return (ret == ESP_OK);
}

bool bsp_i2c_write_mem(uint16_t dev_addr, uint16_t mem_addr,  uint8_t *data, size_t length)
{
    if (data == NULL || length == 0) 
      return ESP_ERR_INVALID_ARG;

    // Create an I2C command link
    cmd = i2c_cmd_link_create();
    if (cmd == NULL) 
      return ESP_ERR_NO_MEM;

    i2c_master_start(cmd);

    i2c_master_write_byte(cmd, dev_addr, true);
    i2c_master_write_byte(cmd, mem_addr, true);
    i2c_master_write(cmd, data, length, true);
    i2c_master_stop(cmd);

    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(1000));

    i2c_cmd_link_delete(cmd);

    return (ret == ESP_OK);
}