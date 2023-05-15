/**
 ******************************************************************************
 * @file    LPS22HHSensor.cpp
 * @author  SRA
 * @version V1.0.0
 * @date    February 2019
 * @brief   Implementation of a LPS22HH pressure sensor.
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

#include "LPS22HHSensor.h"


/* Class Implementation ------------------------------------------------------*/

/** Constructor
 * @param spi object of an helper class which handles the SPI peripheral
 * @param cs_pin the chip select pin
 * @param int_pin the interrupt pin
 * @param spi_type the SPI type
 */
LPS22HHSensor::LPS22HHSensor(SPI *spi, PinName cs_pin, PinName int_pin, SPI_type_t spi_type) : _dev_spi(spi), _cs_pin(cs_pin), _int_irq(int_pin), _spi_type(spi_type)
{
    assert(spi);
    if (cs_pin == NC) {
        printf("ERROR LPS22HH CS MUST NOT BE NC\n\r");
        _dev_spi = NULL;
        _dev_i2c = NULL;
        return;
    }

    _reg_ctx.write_reg = LPS22HH_io_write;
    _reg_ctx.read_reg = LPS22HH_io_read;
    _reg_ctx.handle = (void *)this;
    _cs_pin = 1;
    _dev_i2c = NULL;
    _address = 0;

    if (_spi_type == SPI3W) {
        /* Enable SPI 3-Wires on the component */
        uint8_t data = 0x01;
        lps22hh_write_reg(&_reg_ctx, LPS22HH_CTRL_REG1, &data, 1);
    }
}

/** Constructor
 * @param i2c object of an helper class which handles the I2C peripheral
 * @param address the address of the component's instance
 * @param int_pin the interrupt pin
 */
LPS22HHSensor::LPS22HHSensor(DevI2C *i2c, uint8_t address, PinName int_pin) : _dev_i2c(i2c), _address(address), _cs_pin(NC), _int_irq(int_pin)
{
    assert(i2c);
    _dev_spi = NULL;
    _reg_ctx.write_reg = LPS22HH_io_write;
    _reg_ctx.read_reg = LPS22HH_io_read;
    _reg_ctx.handle = (void *)this;
}


/**
 * @brief  Initializing the component
 * @param  init pointer to device specific initalization structure
 * @retval 0 in case of success, an error code otherwise
 */
int LPS22HHSensor::init(void *init)
{
    /* Disable MIPI I3C(SM) interface */
    if (lps22hh_i3c_interface_set(&_reg_ctx, LPS22HH_I3C_DISABLE) != 0) {
        return 1;
    }

    /* Power down the device, set Low Noise Enable (bit 5), clear One Shot (bit 4) */
    if (lps22hh_data_rate_set(&_reg_ctx, (lps22hh_odr_t)(LPS22HH_POWER_DOWN | 0x10)) != 0) {
        return 1;
    }

    /* Disable low-pass filter on LPS22HH pressure data */
    if (lps22hh_lp_bandwidth_set(&_reg_ctx, LPS22HH_LPF_ODR_DIV_2) != 0) {
        return 1;
    }

    /* Set block data update mode */
    if (lps22hh_block_data_update_set(&_reg_ctx, PROPERTY_ENABLE) != 0) {
        return 1;
    }

    /* Set autoincrement for multi-byte read/write */
    if (lps22hh_auto_increment_set(&_reg_ctx, PROPERTY_ENABLE) != 0) {
        return 1;
    }

    _last_odr = LPS22HH_25_Hz;
    _is_enabled = 0;

    return 0;
}

/**
 * @brief  Get WHO_AM_I value
 * @param  id the WHO_AM_I value
 * @retval 0 in case of success, an error code otherwise
 */
int LPS22HHSensor::read_id(uint8_t *id)
{
    if (lps22hh_device_id_get(&_reg_ctx, id) != 0) {
        return 1;
    }

    return 0;
}

/**
 * @brief  Enable the LPS22HH pressure sensor
 * @retval 0 in case of success, an error code otherwise
 */
int LPS22HHSensor::enable()
{
    /* Check if the component is already _is_enabled */
    if (_is_enabled == 1U) {
        return 0;
    }

    /* Output data rate selection. */
    if (lps22hh_data_rate_set(&_reg_ctx, _last_odr) != 0) {
        return 1;
    }

    _is_enabled = 1;

    return 0;
}

/**
 * @brief  Disable the LPS22HH pressure sensor
 * @retval 0 in case of success, an error code otherwise
 */
int LPS22HHSensor::disable()
{
    /* Check if the component is already disabled */
    if (_is_enabled == 0U) {
        return 0;
    }

    /* Get current output data rate. */
    if (lps22hh_data_rate_get(&_reg_ctx, &_last_odr) != 0) {
        return 1;
    }
    /* Output data rate selection - power down. */
    if (lps22hh_data_rate_set(&_reg_ctx, LPS22HH_POWER_DOWN) != 0) {
        return 1;
    }


    _is_enabled = 0;

    return 0;
}


/**
 * @brief  Get output data rate
 * @param  odr the output data rate value
 * @retval 0 in case of success, an error code otherwise
 */
int LPS22HHSensor::get_odr(float *odr)
{
    int ret = 0;
    lps22hh_odr_t odr_low_level;

    if (lps22hh_data_rate_get(&_reg_ctx, &odr_low_level) != 0) {
        return 1;
    }

    switch (odr_low_level) {
        case LPS22HH_POWER_DOWN:
            *odr = 0.0f;
            break;

        case LPS22HH_1_Hz:
            *odr = 1.0f;
            break;

        case LPS22HH_10_Hz:
            *odr = 10.0f;
            break;

        case LPS22HH_25_Hz:
            *odr = 25.0f;
            break;

        case LPS22HH_50_Hz:
            *odr = 50.0f;
            break;

        case LPS22HH_75_Hz:
            *odr = 75.0f;
            break;

        case LPS22HH_100_Hz:
            *odr = 100.0f;
            break;

        case LPS22HH_200_Hz:
            *odr = 200.0f;
            break;

        default:
            ret = 1;
            break;
    }

    return ret;
}

/**
 * @brief  Set the LPS22HH pressure sensor output data rate
 * @param  odr the output data rate value to be set
 * @retval 0 in case of success, an error code otherwise
 */
int LPS22HHSensor::set_odr(float odr)
{
    /* Check if the component is _is_enabled */
    if (_is_enabled == 1U) {
        return set_odr_when_enabled(odr);
    } else {
        return set_odr_when_disabled(odr);
    }
}


/**
 * @brief  Set output data rate
 * @param  odr the output data rate value to be set
 * @retval 0 in case of success, an error code otherwise
 */
int LPS22HHSensor::set_odr_when_enabled(float odr)
{
    lps22hh_odr_t new_odr;

    new_odr = (odr <=   1.0f) ? LPS22HH_1_Hz
              : (odr <=  10.0f) ? LPS22HH_10_Hz
              : (odr <=  25.0f) ? LPS22HH_25_Hz
              : (odr <=  50.0f) ? LPS22HH_50_Hz
              : (odr <=  75.0f) ? LPS22HH_75_Hz
              : (odr <= 100.0f) ? LPS22HH_100_Hz
              :                   LPS22HH_200_Hz;

    if (lps22hh_data_rate_set(&_reg_ctx, new_odr) != 0) {
        return 1;
    }

    if (lps22hh_data_rate_get(&_reg_ctx, &_last_odr) != 0) {
        return 1;
    }

    return 0;
}

/**
 * @brief  Set output data rate when disabled
 * @param  odr the output data rate value to be set
 * @retval 0 in case of success, an error code otherwise
 */
int LPS22HHSensor::set_odr_when_disabled(float odr)
{
    _last_odr = (odr <=   1.0f) ? LPS22HH_1_Hz
                : (odr <=  10.0f) ? LPS22HH_10_Hz
                : (odr <=  25.0f) ? LPS22HH_25_Hz
                : (odr <=  50.0f) ? LPS22HH_50_Hz
                : (odr <=  75.0f) ? LPS22HH_75_Hz
                : (odr <= 100.0f) ? LPS22HH_100_Hz
                :                   LPS22HH_200_Hz;

    return 0;
}

/**
 * @brief  Get the LPS22HH pressure value
 * @param  value pointer where the pressure value is written
 * @retval 0 in case of success, an error code otherwise
 */
int LPS22HHSensor::get_pressure(float *value)
{
    axis1bit32_t data_raw_pressure;

    (void)memset(data_raw_pressure.u8bit, 0x00, sizeof(int32_t));
    if (lps22hh_pressure_raw_get(&_reg_ctx, data_raw_pressure.u8bit) != 0) {
        return 1;
    }

    *value = LPS22HH_FROM_LSB_TO_hPa((float)(data_raw_pressure.i32bit));

    return 0;
}

/**
 * @brief  Get the LPS22HH pressure data ready bit value
 * @param  status the status of data ready bit
 * @retval 0 in case of success, an error code otherwise
 */
int LPS22HHSensor::get_press_drdy_status(uint8_t *status)
{
    if (lps22hh_press_flag_data_ready_get(&_reg_ctx, status) != 0) {
        return 1;
    }

    return 0;
}

/**
 * @brief  Get the LPS22HH temperature value
 * @param  value pointer where the temperature value is written
 * @retval 0 in case of success, an error code otherwise
 */
int LPS22HHSensor::get_temperature(float *value)
{
    axis1bit16_t data_raw_temperature;

    (void)memset(data_raw_temperature.u8bit, 0x00, sizeof(int16_t));
    if (lps22hh_temperature_raw_get(&_reg_ctx, data_raw_temperature.u8bit) != 0) {
        return 1;
    }

    *value = LPS22HH_FROM_LSB_TO_degC((float)(data_raw_temperature.i16bit));

    return 0;
}

/**
 * @brief  Get the LPS22HH temperature data ready bit value
 * @param  status the status of data ready bit
 * @retval 0 in case of success, an error code otherwise
 */
int LPS22HHSensor::get_temp_drdy_status(uint8_t *status)
{
    if (lps22hh_temp_flag_data_ready_get(&_reg_ctx, status) != 0) {
        return 1;
    }

    return 0;
}

/**
 * @brief  Get the LPS22HH register value
 * @param  reg address to be written
 * @param  data value to be written
 * @retval 0 in case of success, an error code otherwise
 */
int LPS22HHSensor::read_reg(uint8_t reg, uint8_t *data)
{
    if (lps22hh_read_reg(&_reg_ctx, reg, data, 1) != 0) {
        return 1;
    }

    return 0;
}

/**
 * @brief  Set the LPS22HH register value
 * @param  reg address to be written
 * @param  data value to be written
 * @retval 0 in case of success, an error code otherwise
 */
int LPS22HHSensor::write_reg(uint8_t reg, uint8_t data)
{
    if (lps22hh_write_reg(&_reg_ctx, reg, &data, 1) != 0) {
        return 1;
    }

    return 0;
}

/**
 * @brief  Get the LPS22HH FIFO data
 * @param  press the pointer where FIFO pressure value is stored
 * @param  temp the pointer where FIFO temperature value is stored
 * @retval 0 in case of success, an error code otherwise
 */
int LPS22HHSensor::get_fifo_data(float *press, float *temp)
{
    axis1bit32_t data_raw_pressure;
    axis1bit16_t data_raw_temperature;

    (void)memset(data_raw_pressure.u8bit, 0x00, sizeof(int32_t));
    if (lps22hh_fifo_pressure_raw_get(&_reg_ctx, data_raw_pressure.u8bit) != 0) {
        return 1;
    }

    *press = LPS22HH_FROM_LSB_TO_hPa((float)(data_raw_pressure.i32bit));

    (void)memset(data_raw_temperature.u8bit, 0x00, sizeof(int16_t));
    if (lps22hh_fifo_temperature_raw_get(&_reg_ctx, data_raw_temperature.u8bit) != 0) {
        return 1;
    }

    *temp = LPS22HH_FROM_LSB_TO_degC((float)(data_raw_temperature.i16bit));

    return 0;
}

/**
 * @brief  Get the LPS22HH FIFO threshold
 * @param  status the status of FIFO threshold
 * @retval 0 in case of success, an error code otherwise
 */
int LPS22HHSensor::get_fifo_fth_status(uint8_t *status)
{
    if (lps22hh_fifo_wtm_flag_get(&_reg_ctx, status) != 0) {
        return 1;
    }

    return 0;
}

/**
 * @brief  Get the LPS22HH FIFO full status
 * @param  status the status of FIFO full status
 * @retval 0 in case of success, an error code otherwise
 */
int LPS22HHSensor::get_fifo_full_status(uint8_t *status)
{
    if (lps22hh_fifo_full_flag_get(&_reg_ctx, status) != 0) {
        return 1;
    }

    return 0;
}

/**
 * @brief  Get the LPS22HH FIFO OVR status
 * @param  status the status of FIFO OVR status
 * @retval 0 in case of success, an error code otherwise
 */
int LPS22HHSensor::get_fifo_ovr_status(uint8_t *status)
{
    if (lps22hh_fifo_ovr_flag_get(&_reg_ctx, status) != 0) {
        return 1;
    }

    return 0;
}

/**
 * @brief  Get the LPS22HH FIFO data level
 * @param  status the status of FIFO data level
 * @retval 0 in case of success, an error code otherwise
 */
int LPS22HHSensor::get_fifo_level(uint8_t *status)
{
    if (lps22hh_fifo_data_level_get(&_reg_ctx, status) != 0) {
        return 1;
    }

    return 0;
}

/**
 * @brief  Reset the FIFO interrupt
 * @param  interrupt The FIFO interrupt to be reset; values: 0 = FTH; 1 = FULL; 2 = OVR
 * @retval 0 in case of success, an error code otherwise
 */
int LPS22HHSensor::reset_fifo_interrupt(uint8_t interrupt)
{
    switch (interrupt) {
        case 0:
            if (lps22hh_fifo_threshold_on_int_set(&_reg_ctx, PROPERTY_DISABLE) != 0) {
                return 1;
            }
            break;
        case 1:
            if (lps22hh_fifo_full_on_int_set(&_reg_ctx, PROPERTY_DISABLE) != 0) {
                return 1;
            }
            break;
        case 2:
            if (lps22hh_fifo_ovr_on_int_set(&_reg_ctx, PROPERTY_DISABLE) != 0) {
                return 1;
            }
            break;
        default:
            return 1;
    }

    return 0;
}

/**
 * @brief  Set the FIFO interrupt
 * @param  interrupt The FIFO interrupt to be set; values: 0 = FTH; 1 = FULL; 2 = OVR
 * @retval 0 in case of success, an error code otherwise
 */
int LPS22HHSensor::set_fifo_interrupt(uint8_t interrupt)
{
    switch (interrupt) {
        case 0:
            if (lps22hh_fifo_threshold_on_int_set(&_reg_ctx, PROPERTY_ENABLE) != 0) {
                return 1;
            }
            break;
        case 1:
            if (lps22hh_fifo_full_on_int_set(&_reg_ctx, PROPERTY_ENABLE) != 0) {
                return 1;
            }
            break;
        case 2:
            if (lps22hh_fifo_ovr_on_int_set(&_reg_ctx, PROPERTY_ENABLE) != 0) {
                return 1;
            }
            break;
        default:
            return 1;
    }

    return 0;
}

/**
 * @brief  Set the FIFO mode
 * @param  Mode the FIFO mode to be set
 * @retval 0 in case of success, an error code otherwise
 */
int LPS22HHSensor::set_fifo_mode(uint8_t mode)
{
    /* Verify that the passed parameter contains one of the valid values */
    switch ((lps22hh_f_mode_t)mode) {
        case LPS22HH_BYPASS_MODE:
        case LPS22HH_FIFO_MODE:
        case LPS22HH_STREAM_MODE:
        case LPS22HH_STREAM_TO_FIFO_MODE:
        case LPS22HH_BYPASS_TO_STREAM_MODE:
        case LPS22HH_BYPASS_TO_FIFO_MODE:
            break;
        default:
            return 1;
    }

    if (lps22hh_fifo_mode_set(&_reg_ctx, (lps22hh_f_mode_t)mode) != 0) {
        return 1;
    }

    return 0;
}

/**
 * @brief  Set the LPS22HH FIFO watermark level
 * @param  watermark the FIFO watermark level to be set
 * @retval 0 in case of success, an error code otherwise
 */
int LPS22HHSensor::set_fifo_watermark_level(uint8_t watermark)
{
    if (lps22hh_fifo_watermark_set(&_reg_ctx, watermark) != 0) {
        return 1;
    }

    return 0;
}

/**
 * @brief  Set the LPS22HH stop on watermark function
 * @param  stop the state of stop on watermark function
 * @retval 0 in case of success, an error code otherwise
 */
int LPS22HHSensor::stop_fifo_on_watermark(uint8_t stop)
{
    if (lps22hh_fifo_stop_on_wtm_set(&_reg_ctx, stop) != 0) {
        return 1;
    }

    return 0;
}

/**
 * @brief  Set the LPS22HH One Shot Mode
 * @retval 0 in case of success, an error code otherwise
 */
int LPS22HHSensor::set_one_shot()
{
    /* Start One Shot Measurement */
    if (lps22hh_data_rate_set(&_reg_ctx, LPS22HH_ONE_SHOOT) != 0) {
        return 1;
    }

    return 0;
}

/**
 * @brief  Get the LPS22HH One Shot Status
 * @param  status pointer to the one shot status (1 means measurements available, 0 means measurements not available yet)
 * @retval 0 in case of success, an error code otherwise
 */
int LPS22HHSensor::get_one_shot_status(uint8_t *status)
{
    uint8_t p_da;
    uint8_t t_da;

    /* Get DataReady for pressure */
    if (lps22hh_press_flag_data_ready_get(&_reg_ctx, &p_da) != 0) {
        return 1;
    }

    /* Get DataReady for temperature */
    if (lps22hh_temp_flag_data_ready_get(&_reg_ctx, &t_da) != 0) {
        return 1;
    }

    if (p_da && t_da) {
        *status = 1;
    } else {
        *status = 0;
    }

    return 0;
}

int32_t LPS22HH_io_write(void *handle, uint8_t WriteAddr, uint8_t *pBuffer, uint16_t nBytesToWrite)
{
    return ((LPS22HHSensor *)handle)->io_write(pBuffer, WriteAddr, nBytesToWrite);
}

int32_t LPS22HH_io_read(void *handle, uint8_t ReadAddr, uint8_t *pBuffer, uint16_t nBytesToRead)
{
    return ((LPS22HHSensor *)handle)->io_read(pBuffer, ReadAddr, nBytesToRead);
}
