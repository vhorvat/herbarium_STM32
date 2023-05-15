/**
 ******************************************************************************
 * @file    STTS751Sensor.cpp
 * @author  SRA
 * @version V1.0.0
 * @date    February 2019
 * @brief   Implementation of a STTS751 temperature sensor.
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

#include "STTS751Sensor.h"


/* Class Implementation ------------------------------------------------------*/

/** Constructor
 * @param i2c object of an helper class which handles the I2C peripheral
 * @param address the address of the component's instance
 * @param int_pin the interrupt pin
 */
STTS751Sensor::STTS751Sensor(DevI2C *i2c, uint8_t address, PinName int_pin) : _dev_i2c(i2c), _address(address), _int_irq(int_pin)
{
    assert(i2c);
    _reg_ctx.write_reg = STTS751_io_write;
    _reg_ctx.read_reg = STTS751_io_read;
    _reg_ctx.handle = (void *)this;
}

/**
 * @brief  Initializing the component
 * @param  init pointer to device specific initalization structure
 * @retval 0 in case of success, an error code otherwise
 */
int STTS751Sensor::init(void *init)
{
    /* Disable EVENT pin of SMBus. */
    if (stts751_pin_event_route_set(&_reg_ctx,  PROPERTY_ENABLE) != 0) {
        return 1;
    }

    /* Set default ODR */
    _last_odr = 1.0f;

    /* Set the resolution to the maximum allowed value */
    if (stts751_resolution_set(&_reg_ctx, STTS751_12bit) != 0) {
        return 1;
    }

    /* Put the component in standby mode. */
    if (stts751_temp_data_rate_set(&_reg_ctx, STTS751_TEMP_ODR_OFF) != 0) {
        return 1;
    }

    _is_enabled = 0;

    return 0;
}

/**
 * @brief  Get WHO_AM_I value
 * @param  id the WHO_AM_I value
 * @retval 0 in case of success, an error code otherwise
 */
int STTS751Sensor::read_id(uint8_t *id)
{
    stts751_id_t buf;

    if (stts751_device_id_get(&_reg_ctx, &buf) != 0) {
        return 1;
    }

    *id = buf.manufacturer_id;

    return 0;
}

/**
 * @brief  Enable the STTS751 temperature sensor
 * @retval 0 in case of success, an error code otherwise
 */
int STTS751Sensor::enable()
{
    /* Check if the component is already enabled */
    if (_is_enabled == 1U) {
        return 0;
    }

    /* Power on the component and set the odr. */
    if (set_odr(_last_odr) != 0) {
        return 1;
    }

    _is_enabled = 1;

    return 0;
}

/**
 * @brief  Disable the STTS751 temperature sensor
 * @retval 0 in case of success, an error code otherwise
 */
int STTS751Sensor::disable()
{
    /* Check if the component is already disabled */
    if (_is_enabled == 0U) {
        return 0;
    }

    /* Save the current odr. */
    if (get_odr(&_last_odr) != 0) {
        return 1;
    }

    /* Put the component in standby mode. */
    if (stts751_temp_data_rate_set(&_reg_ctx, STTS751_TEMP_ODR_OFF) != 0) {
        return 1;
    }

    _is_enabled = 0;

    return 0;
}

/**
 * @brief  Get the STTS751 temperature sensor output data rate
 * @param  odr pointer where the output data rate is written
 * @retval 0 in case of success, an error code otherwise
 */
int STTS751Sensor::get_odr(float *odr)
{
    int ret = 0;
    stts751_odr_t odr_low_level;

    if (stts751_temp_data_rate_get(&_reg_ctx, &odr_low_level) != 0) {
        return 1;
    }

    switch (odr_low_level) {
        case STTS751_TEMP_ODR_OFF:
        case STTS751_TEMP_ODR_ONE_SHOT:
            *odr = 0.0f;
            break;

        case STTS751_TEMP_ODR_62mHz5:
            *odr = 0.0625f;
            break;

        case STTS751_TEMP_ODR_125mHz:
            *odr = 0.125f;
            break;

        case STTS751_TEMP_ODR_250mHz:
            *odr = 0.250f;
            break;

        case STTS751_TEMP_ODR_500mHz:
            *odr = 0.500f;
            break;

        case STTS751_TEMP_ODR_1Hz:
            *odr = 1.0f;
            break;

        case STTS751_TEMP_ODR_2Hz:
            *odr = 2.0f;
            break;

        case STTS751_TEMP_ODR_4Hz:
            *odr = 4.0f;
            break;

        case STTS751_TEMP_ODR_8Hz:
            *odr = 8.0f;
            break;

        case STTS751_TEMP_ODR_16Hz:
            *odr = 16.0f;
            break;

        case STTS751_TEMP_ODR_32Hz:
            *odr = 32.0f;
            break;

        default:
            ret = 1;
            break;
    }

    return ret;
}

/**
 * @brief  Set the STTS751 temperature sensor output data rate
 * @param  odr the output data rate value to be set
 * @retval 0 in case of success, an error code otherwise
 */
int STTS751Sensor::set_odr(float odr)
{
    stts751_odr_t new_odr;
    stts751_tres_t res;

    /* Get the current resolution */
    if (stts751_resolution_get(&_reg_ctx, &res) != 0) {
        return 1;
    }

    /* If the requested odr is 16Hz we cannot use the 12 bits resolution */
    if (odr == 16.0f && res == STTS751_12bit) {
        /* We force resolution to the maximum allowed value */
        if (stts751_resolution_set(&_reg_ctx, STTS751_11bit) != 0) {
            return 1;
        }
    }

    /* If the requested odr is 32Hz we cannot use the 12 bits and 11 bits resolutions */
    if (odr == 32.0f && (res == STTS751_12bit || res == STTS751_11bit)) {
        /* We force resolution to the maximum allowed value */
        if (stts751_resolution_set(&_reg_ctx, STTS751_10bit) != 0) {
            return 1;
        }
    }

    new_odr = (odr <= 0.0625f) ? STTS751_TEMP_ODR_62mHz5
              : (odr <= 0.125f) ? STTS751_TEMP_ODR_125mHz
              : (odr <= 0.25f) ? STTS751_TEMP_ODR_250mHz
              : (odr <= 0.5f) ? STTS751_TEMP_ODR_500mHz
              : (odr <= 1.0f) ? STTS751_TEMP_ODR_1Hz
              : (odr <= 2.0f) ? STTS751_TEMP_ODR_2Hz
              : (odr <= 4.0f) ? STTS751_TEMP_ODR_4Hz
              : (odr <= 8.0f) ? STTS751_TEMP_ODR_8Hz
              : (odr <= 16.0f) ? STTS751_TEMP_ODR_16Hz
              :                    STTS751_TEMP_ODR_32Hz;

    if (stts751_temp_data_rate_set(&_reg_ctx, new_odr) != 0) {
        return 1;
    }

    return 0;
}

/**
 * @brief  Get the STTS751 temperature value
 * @param  value pointer where the temperature value is written
 * @retval 0 in case of success, an error code otherwise
 */
int STTS751Sensor::get_temperature(float *value)
{
    int16_t raw_value = 0;

    /* Get the temperature */
    if (stts751_temperature_raw_get(&_reg_ctx, &raw_value) != 0) {
        return 1;
    }

    *value = stts751_from_lsb_to_celsius(raw_value);

    return 0;
}

/**
 * @brief  Get the STTS751 temperature data ready bit value
 * @param  status the status of data ready bit
 * @retval 0 in case of success, an error code otherwise
 */
int STTS751Sensor::get_temp_drdy_status(uint8_t *status)
{
    uint8_t val;

    if (stts751_flag_busy_get(&_reg_ctx, &val) != 0) {
        return 1;
    }

    if (val) {
        *status = 0;
    } else {
        *status = 1;
    }

    return 0;
}

/**
 * @brief  Set the STTS751 low temperature threshold value
 * @param  value the low temperature threshold to be set
 * @retval 0 in case of success, an error code otherwise
 */
int STTS751Sensor::set_low_temp_thr(float value)
{
    int16_t raw_value;

    raw_value = stts751_from_celsius_to_lsb(value);

    /* Set the temperature threshold */
    if (stts751_low_temperature_threshold_set(&_reg_ctx, raw_value) != 0) {
        return 1;
    }

    return 0;
}

/**
 * @brief  Set the STTS751 high temperature threshold value
 * @param  value the high temperature threshold to be set
 * @retval 0 in case of success, an error code otherwise
 */
int STTS751Sensor::set_high_temp_thr(float value)
{
    int16_t raw_value;

    raw_value = stts751_from_celsius_to_lsb(value);

    /* Set the temperature threshold */
    if (stts751_high_temperature_threshold_set(&_reg_ctx, raw_value) != 0) {
        return 1;
    }

    return 0;
}

/**
 * @brief  Get the STTS751 temperature limits status
 * @param  high_limit indicates that high temperature limit has been exceeded
 * @param  low_limit indicates that low temperature limit has been exceeded
 * @param  therm_limit indicates that therm temperature limit has been exceeded
 * @retval 0 in case of success, an error code otherwise
 */
int STTS751Sensor::get_temp_limit_status(uint8_t *high_limit, uint8_t *low_limit, uint8_t *therm_limit)
{
    stts751_status_t status;

    /* Read status register */
    if (stts751_status_reg_get(&_reg_ctx, &status) != 0) {
        return 1;
    }

    if(high_limit) {
        *high_limit = status.t_high;
    }

    if(low_limit) {
        *low_limit = status.t_low;
    }

    if(therm_limit) {
        *therm_limit = status.thrm;
    }

    return 0;
}

/**
 * @brief  Enable or disable interrupt on EVENT pin
 * @param  enable 0 disable the EVENT pin, 1 enable EVENT pin
 * @retval 0 in case of success, an error code otherwise
 */
int STTS751Sensor::set_event_pin(uint8_t enable)
{
    uint8_t state;

    /* The MASK1 bit in configuration register has inverted logic */
    if (enable == 0) {
        state = PROPERTY_ENABLE;
    } else {
        state = PROPERTY_DISABLE;
    }

    if (stts751_pin_event_route_set(&_reg_ctx,  state) != 0) {
        return 1;
    }

    return 0;
}

/**
 * @brief  Get the STTS751 register value
 * @param  reg address to be read
 * @param  data pointer where the value is written
 * @retval 0 in case of success, an error code otherwise
 */
int STTS751Sensor::read_reg(uint8_t reg, uint8_t *data)
{
    if (stts751_read_reg(&_reg_ctx, reg, data, 1) != 0) {
        return 1;
    }

    return 0;
}

/**
 * @brief  Set the STTS751 register value
 * @param  reg address to be written
 * @param  data value to be written
 * @retval 0 in case of success, an error code otherwise
 */
int STTS751Sensor::write_reg(uint8_t reg, uint8_t data)
{
    if (stts751_write_reg(&_reg_ctx, reg, &data, 1) != 0) {
        return 1;
    }

    return 0;
}

/**
 * @brief  Set the STTS751 One Shot Mode
 * @retval 0 in case of success, an error code otherwise
 */
int STTS751Sensor::set_one_shot()
{
    /* Start One Shot Measurement */
    if (stts751_temp_data_rate_set(&_reg_ctx, STTS751_TEMP_ODR_ONE_SHOT) != 0) {
        return 1;
    }

    return 0;
}

/**
 * @brief  Get the STTS751 One Shot Status
 * @param  status pointer to the one shot status (1 means measurements available, 0 means measurements not available yet)
 * @retval 0 in case of success, an error code otherwise
 */
int STTS751Sensor::get_one_shot_status(uint8_t *status)
{
    uint8_t busy;

    /* Get Busy flag */
    if (stts751_flag_busy_get(&_reg_ctx, &busy) != 0) {
        return 1;
    }

    if (busy) {
        *status = 0;
    } else {
        *status = 1;
    }

    return 0;
}


int32_t STTS751_io_write(void *handle, uint8_t WriteAddr, uint8_t *pBuffer, uint16_t nBytesToWrite)
{
    return ((STTS751Sensor *)handle)->io_write(pBuffer, WriteAddr, nBytesToWrite);
}

int32_t STTS751_io_read(void *handle, uint8_t ReadAddr, uint8_t *pBuffer, uint16_t nBytesToRead)
{
    return ((STTS751Sensor *)handle)->io_read(pBuffer, ReadAddr, nBytesToRead);
}
