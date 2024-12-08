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

#define EEPROM_SIZE                 22                  // 11 words, 16 bit each --> 176 bit = 8 * (22 bytes)

// Calibration data variables
int16_t AC1, AC2, AC3;
uint16_t AC4, AC5, AC6;
int16_t B1, B2;
int16_t MB, MC, MD;

// Write to BMP180 register
static drv_bmp180_ret_t drv_bmp180_send_command (uint8_t control_reg_value)
{
  bool ret = false;                                     
  ret = bsp_i2c_write_mem((BMP180_SENSOR_ADDR << 1) | I2C_MASTER_WRITE, BMP180_REG_CTRL_VALUE, &control_reg_value, 1);
  return (ret == true) ? DRV_BMP180_OK : DRV_BMP180_ERROR;
}


// Read from BMP180 register
static drv_bmp180_ret_t drv_bmp180_read_reg (uint16_t reg_addr, uint8_t *data, size_t length)
{
  bool ret = false;                                     
  ret = bsp_i2c_read_mem(BMP180_SENSOR_ADDR, reg_addr, data, length);
  return (ret == true) ? DRV_BMP180_OK : DRV_BMP180_ERROR;
}


// Function to read calibration data from BMP180
drv_bmp180_ret_t drv_bmp180_read_calibration() 
{
    uint8_t data[EEPROM_SIZE];
    if (drv_bmp180_read_reg(BMP180_REG_CAL_AC1, data, EEPROM_SIZE) != DRV_BMP180_OK)
        return DRV_BMP180_ERROR;

    AC1 = (data[0]  << 8) | data[1] ;
    AC2 = (data[2]  << 8) | data[3] ;
    AC3 = (data[4]  << 8) | data[5] ;
    AC4 = (data[6]  << 8) | data[7] ;
    AC5 = (data[8]  << 8) | data[9] ;
    AC6 = (data[10] << 8) | data[11];
    B1  = (data[12] << 8) | data[13];
    B2  = (data[14] << 8) | data[15];
    MB  = (data[16] << 8) | data[17];
    MC  = (data[18] << 8) | data[19];
    MD  = (data[20] << 8) | data[21];

    return DRV_BMP180_OK;
}


// Read raw temperature data from BMP180
drv_bmp180_ret_t drv_bmp180_read_raw_temperature(int32_t *raw_temp)                     // uncompensated temperature
{
    if (drv_bmp180_send_command(BMP180_CMD_READ_TEMP) != DRV_BMP180_OK)
        return DRV_BMP180_ERROR;

    bsp_timer_delay(5); 

    uint8_t temp_raw_msb;
    uint8_t temp_raw_lsb;

    drv_bmp180_read_reg(BMP180_REG_OUT_MSB, &temp_raw_msb, 1);

    drv_bmp180_read_reg(BMP180_REG_OUT_LSB, &temp_raw_lsb, 1);

    *raw_temp = (temp_raw_msb << 8) | temp_raw_lsb;
    
    return DRV_BMP180_OK;
}


// Read raw pressure data from BMP180
drv_bmp180_ret_t drv_bmp180_read_raw_pressure (int32_t *raw_press)                      // uncompensated pressure   
{
    drv_bmp180_send_command(BMP180_CMD_READ_PRESSURE);

    bsp_timer_delay(25);

    uint8_t data[3];

    uint8_t press_raw_msb;
    uint8_t press_raw_lsb;

    drv_bmp180_read_reg(BMP180_REG_OUT_MSB, data, sizeof(data));

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