#include "drv_bmp180.h"


#define I2C_MASTER_NUM              I2C_NUM_0

#define BMP180_SENSOR_ADDR          0x77  // Default I2C address of BMP180

// BMP180 Registers
#define BMP180_REG_CAL_AC1          0xAA
#define BMP180_REG_CAL_AC2          0xAC
#define BMP180_REG_CAL_AC3          0xAE
#define BMP180_REG_CAL_AC4          0xB0
#define BMP180_REG_CAL_AC5          0xB2
#define BMP180_REG_CAL_AC6          0xB4
#define BMP180_REG_CAL_B1           0xB6
#define BMP180_REG_CAL_B2           0xB8
#define BMP180_REG_CAL_MB           0xBA
#define BMP180_REG_CAL_MC           0xBC
#define BMP180_REG_CAL_MD           0xBE

#define BMP180_REG_CTRL_VALUE       0xF4
#define BMP180_REG_OUT_MSB          0xF6
#define BMP180_REG_OUT_LSB          0xF7
#define BMP180_REG_OUT_XLSB         0xF8

#define BMP180_CMD_READ_TEMP        0x2E
#define BMP180_CMD_READ_PRESSURE    0x34

// Calibration data variables
int16_t AC1, AC2, AC3;
uint16_t AC4, AC5, AC6;
int16_t B1, B2;
int16_t MB, MC, MD;

// Write to BMP180 register
static drv_bmp180_ret_t drv_bmp180_send_command (uint8_t reg_addr)
{
  bool ret = false;                                     
  ret = bsp_i2c_write_mem((BMP180_SENSOR_ADDR << 1) | I2C_MASTER_WRITE, BMP180_REG_CTRL_VALUE, &reg_addr, 1);
  return (ret == true) ? DRV_BMP180_OK : DRV_BMP180_ERROR;
}

// Read from BMP180 register
esp_err_t bmp180_read_register(uint8_t reg_addr, uint8_t *data, size_t len) 
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (BMP180_SENSOR_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg_addr, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (BMP180_SENSOR_ADDR << 1) | I2C_MASTER_READ, true);
    i2c_master_read(cmd, data, len, I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmd);
    esp_err_t err = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    return err;
}


static drv_bmp180_ret_t drv_bmp180_read_reg (uint16_t reg_addr, uint8_t *data, size_t length)
{
  bool ret = false;                                     
  ret = bsp_i2c_read_mem(BMP180_SENSOR_ADDR, reg_addr, data, 1);
  return (ret == true) ? DRV_BMP180_ERROR : DRV_BMP180_OK;
}


// Function to read calibration data from BMP180
esp_err_t bmp180_read_calibration() 
{
    uint8_t data[22];
    esp_err_t ret = bmp180_read_register(BMP180_REG_CAL_AC1, data, 22);
    if (ret != ESP_OK) {
        return ret;
    }

    // drv_bmp180_read_reg(BMP180_REG_CAL_AC1, data, 22);


    AC1 = (data[0] << 8) | data[1];
    AC2 = (data[2] << 8) | data[3];
    AC3 = (data[4] << 8) | data[5];
    AC4 = (data[6] << 8) | data[7];
    AC5 = (data[8] << 8) | data[9];
    AC6 = (data[10] << 8) | data[11];
    B1 = (data[12] << 8) | data[13];
    B2 = (data[14] << 8) | data[15];
    MB = (data[16] << 8) | data[17];
    MC = (data[18] << 8) | data[19];
    MD = (data[20] << 8) | data[21];

    return ESP_OK;
}

// Read raw temperature data from BMP180
esp_err_t bmp180_read_raw_temperature(int32_t *raw_temp) 
{

    drv_bmp180_send_command(BMP180_CMD_READ_TEMP);

    vTaskDelay(5 / portTICK_PERIOD_MS);  // Wait for conversion

    uint8_t data[2];
    esp_err_t ret = bmp180_read_register(BMP180_REG_OUT_MSB, data, 2);
    if (ret != ESP_OK) {
        return ret;
    }

    // drv_bmp180_read_reg(BMP180_REG_OUT_MSB, data, 2);

    *raw_temp = (data[0] << 8) | data[1];
    return ESP_OK;
}

// Read raw pressure data from BMP180
esp_err_t bmp180_read_raw_pressure(int32_t *raw_press) 
{
    drv_bmp180_send_command(BMP180_CMD_READ_PRESSURE);

    vTaskDelay(25 / portTICK_PERIOD_MS);  // Wait for conversion

    uint8_t data[3];
    esp_err_t ret = bmp180_read_register(BMP180_REG_OUT_MSB, data, 3);
    if (ret != ESP_OK) {
        return ret;
    }

    // drv_bmp180_read_reg(BMP180_REG_OUT_MSB, data, 3);

    *raw_press = (data[0] << 16) | (data[1] << 8) | data[2];
    return ESP_OK;
}

// Temperature compensation function
int32_t bmp180_calculate_temperature(int32_t raw_temp) {
    int32_t X1 = ((raw_temp - AC6) * AC5) >> 15;
    int32_t X2 = (MC << 11) / (X1 + MD);
    int32_t B5 = X1 + X2;
    return (B5 + 8) >> 4;  // Temperature in 0.1°C
}

// Pressure compensation function
int32_t bmp180_calculate_pressure(int32_t raw_press, int32_t temp) {
    int32_t B6 = temp - 4000;
    int32_t X1 = (B2 * (B6 * B6 >> 12)) >> 11;
    int32_t X2 = (AC2 * B6) >> 11;
    int32_t X3 = X1 + X2;
    int32_t B3 = (((AC1 * 4 + X3) << 1) + 2) >> 2;

    X1 = (AC3 * B6) >> 13;
    X2 = (B1 * (B6 * B6 >> 12)) >> 16;
    X3 = ((X1 + X2) + 2) >> 2;
    int32_t B4 = (AC4 * (unsigned int)(X3 + 32768)) >> 15;
    int32_t B7 = ((unsigned int)(raw_press - B3) * (50000));

    int32_t p = (B7 < 0x80000000) ? ((B7 << 1) / B4) : ((B7 / B4) << 1);
    X1 = (p >> 8) * (p >> 8);
    X1 = (X1 * 3038) >> 16;
    X2 = (-7357 * p) >> 16;
    return p + ((X1 + X2 + 3791) >> 4);
}