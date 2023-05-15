/**
 ******************************************************************************
 * @file    LIS2MDLSensor.cpp
 * @author  SRA
 * @version V1.0.0
 * @date    February 2019
 * @brief   Implementation of an LIS2MDL 3 axes magnetometer sensor.
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; COPYRIGHT(c) 2019 STMicroelectronics</center></h2>
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *   1. Redistributions of source code must retain the above copyright notice,
 *      this list of conditions and the following disclaimer.
 *   2. Redistributions in binary form must reproduce the above copyright notice,
 *      this list of conditions and the following disclaimer in the documentation
 *      and/or other materials provided with the distribution.
 *   3. Neither the name of STMicroelectronics nor the names of its contributors
 *      may be used to endorse or promote products derived from this software
 *      without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ******************************************************************************
 */


/* Includes ------------------------------------------------------------------*/

#include "LIS2MDLSensor.h"


/* Class Implementation ------------------------------------------------------*/

/** Constructor
 * @param spi object of an helper class which handles the SPI peripheral
 * @param cs_pin the chip select pin
 * @param int_pin the interrupt pin
 * @param spi_type the SPI type
 */
LIS2MDLSensor::LIS2MDLSensor(SPI *spi, PinName cs_pin, PinName int_pin, SPI_type_t spi_type) : _dev_spi(spi), _cs_pin(cs_pin), _int_irq(int_pin), _spi_type(spi_type)
{
    assert(spi);
    if (cs_pin == NC) {
        printf("ERROR LIS2MDL CS MUST NOT BE NC\n\r");
        _dev_spi = NULL;
        _dev_i2c = NULL;
        return;
    }

    _reg_ctx.write_reg = LIS2MDL_io_write;
    _reg_ctx.read_reg = LIS2MDL_io_read;
    _reg_ctx.handle = (void *)this;
    _cs_pin = 1;
    _dev_i2c = NULL;
    _address = 0;

    if (_spi_type == SPI4W) {
        /* Enable SPI 4-Wires on the component (in this way we lose the usage of INT pin) */
        uint8_t data = 0x34;
        lis2mdl_write_reg(&_reg_ctx, LIS2MDL_CFG_REG_C, &data, 1);
    }
}

/** Constructor
 * @param i2c object of an helper class which handles the I2C peripheral
 * @param address the address of the component's instance
 * @param int_pin the interrupt pin
 */
LIS2MDLSensor::LIS2MDLSensor(DevI2C *i2c, uint8_t address, PinName int_pin) : _dev_i2c(i2c), _address(address), _cs_pin(NC), _int_irq(int_pin)
{
    assert(i2c);
    _dev_spi = NULL;
    _reg_ctx.write_reg = LIS2MDL_io_write;
    _reg_ctx.read_reg = LIS2MDL_io_read;
    _reg_ctx.handle = (void *)this;
}

/**
 * @brief  Initializing the component
 * @param  init pointer to device specific initalization structure
 * @retval 0 in case of success, an error code otherwise
 */
int LIS2MDLSensor::init(void *init)
{
    /* Enable BDU */
    if (lis2mdl_block_data_update_set(&(_reg_ctx), PROPERTY_ENABLE) != 0) {
        return 1;
    }

    /* Operating mode selection - power down */
    if (lis2mdl_operating_mode_set(&(_reg_ctx), LIS2MDL_POWER_DOWN) != 0) {
        return 1;
    }

    /* Output data rate selection */
    if (lis2mdl_data_rate_set(&(_reg_ctx), LIS2MDL_ODR_100Hz) != 0) {
        return 1;
    }

    /* Self Test disabled. */
    if (lis2mdl_self_test_set(&(_reg_ctx), PROPERTY_DISABLE) != 0) {
        return 1;
    }

    _mag_is_enabled = 0;

    return 0;
}

/**
 * @brief  Read component ID
 * @param  id the WHO_AM_I value
 * @retval 0 in case of success, an error code otherwise
 */
int LIS2MDLSensor::read_id(uint8_t *id)
{
    if (lis2mdl_device_id_get(&_reg_ctx, id) != 0) {
        return 1;
    }

    return 0;
}


/**
 * @brief Enable the LIS2MDL magnetometer sensor
 * @retval 0 in case of success, an error code otherwise
 */
int LIS2MDLSensor::enable()
{
    /* Check if the component is already enabled */
    if (_mag_is_enabled == 1U) {
        return 0;
    }

    /* Output data rate selection. */
    if (lis2mdl_operating_mode_set(&_reg_ctx, LIS2MDL_CONTINUOUS_MODE) != 0) {
        return 1;
    }

    _mag_is_enabled = 1;

    return 0;
}

/**
 * @brief Disable the LIS2MDL magnetometer sensor
 * @retval 0 in case of success, an error code otherwise
 */
int LIS2MDLSensor::disable()
{
    /* Check if the component is already disabled */
    if (_mag_is_enabled == 0U) {
        return 0;
    }

    /* Output data rate selection - power down. */
    if (lis2mdl_operating_mode_set(&_reg_ctx, LIS2MDL_POWER_DOWN) != 0) {
        return 1;
    }

    _mag_is_enabled = 0;

    return 0;
}

/**
 * @brief  Get the LIS2MDL magnetometer sensor sensitivity
 * @param  sensitivity pointer where the sensitivity is written
 * @retval 0 in case of success, an error code otherwise
 */
int LIS2MDLSensor::get_m_sensitivity(float *sensitivity)
{
    *sensitivity = LIS2MDL_MAG_SENSITIVITY_FS_50GAUSS;

    return 0;
}

/**
 * @brief  Get the LIS2MDL magnetometer sensor output data rate
 * @param  odr pointer where the output data rate is written
 * @retval 0 in case of success, an error code otherwise
 */
int LIS2MDLSensor::get_m_odr(float *odr)
{
    int ret = 0;
    lis2mdl_odr_t odr_low_level;

    /* Get current output data rate. */
    if (lis2mdl_data_rate_get(&_reg_ctx, &odr_low_level) != 0) {
        return 1;
    }

    switch (odr_low_level) {
        case LIS2MDL_ODR_10Hz:
            *odr = 10.0f;
            break;

        case LIS2MDL_ODR_20Hz:
            *odr = 20.0f;
            break;

        case LIS2MDL_ODR_50Hz:
            *odr = 50.0f;
            break;

        case LIS2MDL_ODR_100Hz:
            *odr = 100.0f;
            break;

        default:
            ret = 1;
            break;
    }

    return ret;
}

/**
 * @brief  Set the LIS2MDL magnetometer sensor output data rate
 * @param  odr the output data rate value to be set
 * @retval 0 in case of success, an error code otherwise
 */
int LIS2MDLSensor::set_m_odr(float odr)
{
    lis2mdl_odr_t new_odr;

    new_odr = (odr <= 10.000f) ? LIS2MDL_ODR_10Hz
              : (odr <= 20.000f) ? LIS2MDL_ODR_20Hz
              : (odr <= 50.000f) ? LIS2MDL_ODR_50Hz
              :                    LIS2MDL_ODR_100Hz;

    if (lis2mdl_data_rate_set(&_reg_ctx, new_odr) != 0) {
        return 1;
    }

    return 0;
}

/**
 * @brief  Get the LIS2MDL magnetometer sensor full scale
 * @param  full_scale pointer where the full scale is written
 * @retval 0 in case of success, an error code otherwise
 */
int LIS2MDLSensor::get_m_fs(float *full_scale)
{
    *full_scale = 50.0f;

    return 0;
}

/**
 * @brief  Set the LIS2MDL magnetometer sensor full scale
 * @param  full_scale the functional full scale to be set
 * @retval 0 in case of success, an error code otherwise
 */
int LIS2MDLSensor::set_m_fs(float full_scale)
{
    (void)full_scale;
    return 0;
}

/**
 * @brief  Get the LIS2MDL magnetometer sensor axes
 * @param  magnetic_field pointer where the values of the axes are written
 * @retval 0 in case of success, an error code otherwise
 */
int LIS2MDLSensor::get_m_axes(int32_t *magnetic_field)
{
    axis3bit16_t data_raw;
    float sensitivity;

    /* Read raw data values. */
    if (lis2mdl_magnetic_raw_get(&_reg_ctx, data_raw.u8bit) != 0) {
        return 1;
    }

    /* Get LIS2MDL actual sensitivity. */
    if (get_m_sensitivity(&sensitivity) != 0) {
        return 1;
    }

    /* Calculate the data. */
    magnetic_field[0] = (int32_t)((float)((float)data_raw.i16bit[0] * sensitivity));
    magnetic_field[1] = (int32_t)((float)((float)data_raw.i16bit[1] * sensitivity));
    magnetic_field[2] = (int32_t)((float)((float)data_raw.i16bit[2] * sensitivity));

    return 0;
}

/**
 * @brief  Get the LIS2MDL magnetometer sensor raw axes
 * @param  value pointer where the raw values of the axes are written
 * @retval 0 in case of success, an error code otherwise
 */
int LIS2MDLSensor::get_m_axes_raw(int16_t *value)
{
    axis3bit16_t data_raw;

    /* Read raw data values. */
    if (lis2mdl_magnetic_raw_get(&_reg_ctx, data_raw.u8bit) != 0) {
        return 1;
    }

    /* Format the data. */
    value[0] = data_raw.i16bit[0];
    value[1] = data_raw.i16bit[1];
    value[2] = data_raw.i16bit[2];

    return 0;
}

/**
 * @brief  Get the LIS2MDL register value for magnetic sensor
 * @param  reg address to be read
 * @param  data pointer where the value is written
 * @retval 0 in case of success, an error code otherwise
 */
int LIS2MDLSensor::read_reg(uint8_t reg, uint8_t *data)
{
    if (lis2mdl_read_reg(&_reg_ctx, reg, data, 1) != 0) {
        return 1;
    }

    return 0;
}

/**
 * @brief  Set the LIS2MDL register value for magnetic sensor
 * @param  pObj the device pObj
 * @param  reg address to be written
 * @param  data value to be written
 * @retval 0 in case of success, an error code otherwise
 */
int LIS2MDLSensor::write_reg(uint8_t reg, uint8_t data)
{
    if (lis2mdl_write_reg(&_reg_ctx, reg, &data, 1) != 0) {
        return 1;
    }

    return 0;
}

/**
 * @brief  Set self test
 * @param  status the value of self_test in reg CFG_REG_C
 * @retval 0 in case of success, an error code otherwise
 */
int LIS2MDLSensor::set_m_self_test(uint8_t status)
{
    if (lis2mdl_self_test_set(&_reg_ctx, status) != 0) {
        return 1;
    }

    return 0;
}

/**
 * @brief  Get the LIS2MDL MAG data ready bit value
 * @param  status the status of data ready bit
 * @retval 0 in case of success, an error code otherwise
 */
int LIS2MDLSensor::get_m_drdy_status(uint8_t *status)
{
    if (lis2mdl_mag_data_ready_get(&_reg_ctx, status) != 0) {
        return 1;
    }

    return 0;
}



int32_t LIS2MDL_io_write(void *handle, uint8_t WriteAddr, uint8_t *pBuffer, uint16_t nBytesToWrite)
{
    return ((LIS2MDLSensor *)handle)->io_write(pBuffer, WriteAddr, nBytesToWrite);
}

int32_t LIS2MDL_io_read(void *handle, uint8_t ReadAddr, uint8_t *pBuffer, uint16_t nBytesToRead)
{
    return ((LIS2MDLSensor *)handle)->io_read(pBuffer, ReadAddr, nBytesToRead);
}
