/**
 ******************************************************************************
 * @file    LIS2DW12Sensor.cpp
 * @author  CLab
 * @version V1.0.0
 * @date    15 November 2018
 * @brief   Implementation of an LIS2DW12 Inertial Measurement Unit (IMU) 3 axes
 *          sensor.
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; COPYRIGHT(c) 2018 STMicroelectronics</center></h2>
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

#include "LIS2DW12Sensor.h"


/* Class Implementation ------------------------------------------------------*/

/** Constructor
 * @param i2c object of an helper class which handles the I2C peripheral
 * @param address the address of the component's instance
 * @param int1_pin the interrupt 1 pin
 * @param int2_pin the interrupt 2 pin
 */
LIS2DW12Sensor::LIS2DW12Sensor(DevI2C *i2c, uint8_t address, PinName int1_pin, PinName int2_pin) :
    _dev_i2c(i2c), _address(address), _cs_pin(NC), _int1_irq(int1_pin), _int2_irq(int2_pin)
{
    assert(i2c);
    _dev_spi = NULL;
    _reg_ctx.write_reg = LIS2DW12_io_write;
    _reg_ctx.read_reg = LIS2DW12_io_read;
    _reg_ctx.handle = (void *)this;
}

/** Constructor
 * @param spi object of an helper class which handles the SPI peripheral
 * @param cs_pin the chip select pin
 * @param int1_pin the interrupt 1 pin
 * @param int2_pin the interrupt 2 pin
 * @param spi_type the SPI type (4-Wires or 3-Wires)
 */
LIS2DW12Sensor::LIS2DW12Sensor(SPI *spi, PinName cs_pin, PinName int1_pin, PinName int2_pin, SPI_type_t spi_type) :
    _dev_spi(spi), _cs_pin(cs_pin), _int1_irq(int1_pin), _int2_irq(int2_pin), _spi_type(spi_type)
{
    assert(spi);
    if (cs_pin == NC) {
        printf("ERROR LPS22HBSensor CS MUST NOT BE NC\n\r");
        _dev_spi = NULL;
        _dev_i2c = NULL;
        return;
    }
    _reg_ctx.write_reg = LIS2DW12_io_write;
    _reg_ctx.read_reg = LIS2DW12_io_read;
    _reg_ctx.handle = (void *)this;
    _cs_pin = 1;
    _dev_i2c = NULL;
    _address = 0;

    if (_spi_type == SPI3W) {
        /* Enable SPI 3-Wires on the component */
        uint8_t data = 0x05;
        lis2dw12_write_reg(&_reg_ctx, LIS2DW12_CTRL2, &data, 1);
    }

    /* Disable I2C on the component */
    lis2dw12_i2c_interface_set(&_reg_ctx, LIS2DW12_I2C_DISABLE);
}

/**
 * @brief     Initializing the component.
 * @param[in] init pointer to device specific initalization structure.
 * @retval    "0" in case of success, an error code otherwise.
 */
int LIS2DW12Sensor::init(void *init)
{
    /* Enable register address automatically incremented during a multiple byte
    access with a serial interface. */
    if (lis2dw12_auto_increment_set(&_reg_ctx, PROPERTY_ENABLE) != 0) {
        return 1;
    }

    /* Enable BDU */
    if (lis2dw12_block_data_update_set(&_reg_ctx, PROPERTY_ENABLE) != 0) {
        return 1;
    }

    /* FIFO mode selection */
    if (lis2dw12_fifo_mode_set(&_reg_ctx, LIS2DW12_BYPASS_MODE) != 0) {
        return 1;
    }

    /* Power mode selection */
    if (lis2dw12_power_mode_set(&_reg_ctx, LIS2DW12_HIGH_PERFORMANCE) != 0) {
        return 1;
    }

    /* Output data rate selection - power down. */
    if (lis2dw12_data_rate_set(&_reg_ctx, LIS2DW12_XL_ODR_OFF) != 0) {
        return 1;
    }

    /* Full scale selection. */
    if (lis2dw12_full_scale_set(&_reg_ctx, LIS2DW12_2g) != 0) {
        return 1;
    }

    /* Select default output data rate. */
    _x_last_odr = 100.0f;

    _x_last_operating_mode = LIS2DW12_HIGH_PERFORMANCE_MODE;

    _x_last_noise = LIS2DW12_LOW_NOISE_DISABLE;

    _x_is_enabled = 0;

    return 0;
}

/**
 * @brief  Enable LIS2DW12 Accelerator
 * @retval 0 in case of success, an error code otherwise
 */
int LIS2DW12Sensor::enable_x(void)
{
    /* Check if the component is already enabled */
    if (_x_is_enabled == 1) {
        return 0;
    }

    /* Output data rate selection. */
    if (set_x_odr_when_enabled(_x_last_odr, _x_last_operating_mode, _x_last_noise) == 1) {
        return 1;
    }

    _x_is_enabled = 1;

    return 0;
}

/**
 * @brief  Disable LIS2DW12 Accelerator
 * @retval 0 in case of success, an error code otherwise
 */
int LIS2DW12Sensor::disable_x(void)
{
    /* Check if the component is already disabled */
    if (_x_is_enabled == 0) {
        return 0;
    }

    /* Output data rate selection - power down. */
    if (lis2dw12_data_rate_set(&_reg_ctx, LIS2DW12_XL_ODR_OFF) != 0) {
        return 1;
    }

    _x_is_enabled = 0;

    return 0;
}

/**
 * @brief  Read ID of LIS2DW12 Accelerometer and Gyroscope
 * @param  p_id the pointer where the ID of the device is stored
 * @retval 0 in case of success, an error code otherwise
 */
int LIS2DW12Sensor::read_id(uint8_t *id)
{
    if (!id) {
        return 1;
    }

    /* Read WHO AM I register */
    if (lis2dw12_device_id_get(&_reg_ctx, id) != 0) {
        return 1;
    }

    return 0;
}

/**
 * @brief  Read data from LIS2DW12 Accelerometer
 * @param  acceleration the pointer where the accelerometer data are stored
 * @retval 0 in case of success, an error code otherwise
 */
int LIS2DW12Sensor::get_x_axes(int32_t *acceleration)
{
    int16_t data_raw[3];
    float sensitivity = 0;

    /* Read raw data from LIS2DW12 output register. */
    if (get_x_axes_raw(data_raw) == 1) {
        return 1;
    }

    /* Get LIS2DW12 actual sensitivity. */
    if (get_x_sensitivity(&sensitivity) == 1) {
        return 1;
    }

    /* Calculate the data. */
    acceleration[0] = (int32_t)(data_raw[0] * sensitivity);
    acceleration[1] = (int32_t)(data_raw[1] * sensitivity);
    acceleration[2] = (int32_t)(data_raw[2] * sensitivity);

    return 0;
}

/**
 * @brief  Read Accelerometer Sensitivity
 * @param  sensitivity the pointer where the accelerometer sensitivity is stored
 * @retval 0 in case of success, an error code otherwise
 */
int LIS2DW12Sensor::get_x_sensitivity(float *sensitivity)
{
    int32_t ret = 0;
    lis2dw12_fs_t full_scale;
    lis2dw12_mode_t mode;

    /* Read actual full scale selection from sensor. */
    if (lis2dw12_full_scale_get(&_reg_ctx, &full_scale) != 0) {
        return 1;
    }

    /* Read actual power mode selection from sensor. */
    if (lis2dw12_power_mode_get(&_reg_ctx, &mode) != 0) {
        return 1;
    }

    switch (mode) {
        case LIS2DW12_CONT_LOW_PWR_12bit:
        case LIS2DW12_SINGLE_LOW_PWR_12bit:
        case LIS2DW12_CONT_LOW_PWR_LOW_NOISE_12bit:
        case LIS2DW12_SINGLE_LOW_LOW_NOISE_PWR_12bit:
            switch (full_scale) {
                case LIS2DW12_2g:
                    *sensitivity = LIS2DW12_ACC_SENSITIVITY_FOR_FS_2G_LOPOW1_MODE;
                    break;

                case LIS2DW12_4g:
                    *sensitivity = LIS2DW12_ACC_SENSITIVITY_FOR_FS_4G_LOPOW1_MODE;
                    break;

                case LIS2DW12_8g:
                    *sensitivity = LIS2DW12_ACC_SENSITIVITY_FOR_FS_8G_LOPOW1_MODE;
                    break;

                case LIS2DW12_16g:
                    *sensitivity = LIS2DW12_ACC_SENSITIVITY_FOR_FS_16G_LOPOW1_MODE;
                    break;

                default:
                    *sensitivity = -1.0f;
                    ret = 1;
                    break;
            }
            break;

        case LIS2DW12_HIGH_PERFORMANCE:
        case LIS2DW12_CONT_LOW_PWR_4:
        case LIS2DW12_CONT_LOW_PWR_3:
        case LIS2DW12_CONT_LOW_PWR_2:
        case LIS2DW12_SINGLE_LOW_PWR_4:
        case LIS2DW12_SINGLE_LOW_PWR_3:
        case LIS2DW12_SINGLE_LOW_PWR_2:
        case LIS2DW12_HIGH_PERFORMANCE_LOW_NOISE:
        case LIS2DW12_CONT_LOW_PWR_LOW_NOISE_4:
        case LIS2DW12_CONT_LOW_PWR_LOW_NOISE_3:
        case LIS2DW12_CONT_LOW_PWR_LOW_NOISE_2:
        case LIS2DW12_SINGLE_LOW_PWR_LOW_NOISE_4:
        case LIS2DW12_SINGLE_LOW_PWR_LOW_NOISE_3:
        case LIS2DW12_SINGLE_LOW_PWR_LOW_NOISE_2:
            switch (full_scale) {
                case LIS2DW12_2g:
                    *sensitivity = LIS2DW12_ACC_SENSITIVITY_FOR_FS_2G_OTHER_MODES;
                    break;

                case LIS2DW12_4g:
                    *sensitivity = LIS2DW12_ACC_SENSITIVITY_FOR_FS_4G_OTHER_MODES;
                    break;

                case LIS2DW12_8g:
                    *sensitivity = LIS2DW12_ACC_SENSITIVITY_FOR_FS_8G_OTHER_MODES;
                    break;

                case LIS2DW12_16g:
                    *sensitivity = LIS2DW12_ACC_SENSITIVITY_FOR_FS_16G_OTHER_MODES;
                    break;

                default:
                    *sensitivity = -1.0f;
                    ret = 1;
                    break;
            }
            break;

        default:
            *sensitivity = -1.0f;
            ret = 1;
            break;
    }

    return ret;
}

/**
 * @brief  Read raw data from LIS2DW12 Accelerometer
 * @param  value the pointer where the accelerometer raw data are stored
 * @retval 0 in case of success, an error code otherwise
 */
int LIS2DW12Sensor::get_x_axes_raw(int16_t *value)
{
    axis3bit16_t data_raw;
    lis2dw12_mode_t mode;
    int32_t ret = 0;

    /* Read actual power mode selection from sensor. */
    if (lis2dw12_power_mode_get(&_reg_ctx, &mode) != 0) {
        return 1;
    }

    /* Read raw data values. */
    if (lis2dw12_acceleration_raw_get(&_reg_ctx, data_raw.u8bit) != 0) {
        return 1;
    }

    switch (mode) {
        case LIS2DW12_CONT_LOW_PWR_12bit:
        case LIS2DW12_SINGLE_LOW_PWR_12bit:
        case LIS2DW12_CONT_LOW_PWR_LOW_NOISE_12bit:
        case LIS2DW12_SINGLE_LOW_LOW_NOISE_PWR_12bit:
            /* Data format 12 bits. */
            value[0] = (data_raw.i16bit[0] / 16);
            value[1] = (data_raw.i16bit[1] / 16);
            value[2] = (data_raw.i16bit[2] / 16);
            break;

        case LIS2DW12_HIGH_PERFORMANCE:
        case LIS2DW12_CONT_LOW_PWR_4:
        case LIS2DW12_CONT_LOW_PWR_3:
        case LIS2DW12_CONT_LOW_PWR_2:
        case LIS2DW12_SINGLE_LOW_PWR_4:
        case LIS2DW12_SINGLE_LOW_PWR_3:
        case LIS2DW12_SINGLE_LOW_PWR_2:
        case LIS2DW12_HIGH_PERFORMANCE_LOW_NOISE:
        case LIS2DW12_CONT_LOW_PWR_LOW_NOISE_4:
        case LIS2DW12_CONT_LOW_PWR_LOW_NOISE_3:
        case LIS2DW12_CONT_LOW_PWR_LOW_NOISE_2:
        case LIS2DW12_SINGLE_LOW_PWR_LOW_NOISE_4:
        case LIS2DW12_SINGLE_LOW_PWR_LOW_NOISE_3:
        case LIS2DW12_SINGLE_LOW_PWR_LOW_NOISE_2:
            /* Data format 14 bits. */
            value[0] = (data_raw.i16bit[0] / 4);
            value[1] = (data_raw.i16bit[1] / 4);
            value[2] = (data_raw.i16bit[2] / 4);
            break;

        default:
            ret = 1;
            break;
    }

    return ret;
}

/**
 * @brief  Read LIS2DW12 Accelerometer output data rate
 * @param  odr the pointer to the output data rate
 * @retval 0 in case of success, an error code otherwise
 */
int LIS2DW12Sensor::get_x_odr(float *odr)
{
    int32_t ret = 0;
    lis2dw12_odr_t odr_low_level;
    lis2dw12_mode_t mode;

    /* Get current output data rate. */
    if (lis2dw12_data_rate_get(&_reg_ctx, &odr_low_level) != 0) {
        return 1;
    }

    /* Read actual power mode selection from sensor. */
    if (lis2dw12_power_mode_get(&_reg_ctx, &mode) != 0) {
        return 1;
    }

    switch (odr_low_level) {
        case LIS2DW12_XL_ODR_OFF:
        case LIS2DW12_XL_SET_SW_TRIG:
        case LIS2DW12_XL_SET_PIN_TRIG:
            *odr = 0.0f;
            break;

        case LIS2DW12_XL_ODR_1Hz6_LP_ONLY:
            switch (mode) {
                case LIS2DW12_HIGH_PERFORMANCE:
                case LIS2DW12_HIGH_PERFORMANCE_LOW_NOISE:
                    *odr = 12.5f;
                    break;

                case LIS2DW12_CONT_LOW_PWR_4:
                case LIS2DW12_CONT_LOW_PWR_3:
                case LIS2DW12_CONT_LOW_PWR_2:
                case LIS2DW12_CONT_LOW_PWR_12bit:
                case LIS2DW12_SINGLE_LOW_PWR_4:
                case LIS2DW12_SINGLE_LOW_PWR_3:
                case LIS2DW12_SINGLE_LOW_PWR_2:
                case LIS2DW12_SINGLE_LOW_PWR_12bit:
                case LIS2DW12_CONT_LOW_PWR_LOW_NOISE_4:
                case LIS2DW12_CONT_LOW_PWR_LOW_NOISE_3:
                case LIS2DW12_CONT_LOW_PWR_LOW_NOISE_2:
                case LIS2DW12_CONT_LOW_PWR_LOW_NOISE_12bit:
                case LIS2DW12_SINGLE_LOW_PWR_LOW_NOISE_4:
                case LIS2DW12_SINGLE_LOW_PWR_LOW_NOISE_3:
                case LIS2DW12_SINGLE_LOW_PWR_LOW_NOISE_2:
                case LIS2DW12_SINGLE_LOW_LOW_NOISE_PWR_12bit:
                    *odr = 1.6f;
                    break;

                default:
                    *odr = -1.0f;
                    ret = 1;
                    break;
            }
            break;

        case LIS2DW12_XL_ODR_12Hz5:
            *odr = 12.5f;
            break;

        case LIS2DW12_XL_ODR_25Hz:
            *odr = 25.0f;
            break;

        case LIS2DW12_XL_ODR_50Hz:
            *odr = 50.0f;
            break;

        case LIS2DW12_XL_ODR_100Hz:
            *odr = 100.0f;
            break;

        case LIS2DW12_XL_ODR_200Hz:
            *odr = 200.0f;
            break;

        case LIS2DW12_XL_ODR_400Hz:
            switch (mode) {
                case LIS2DW12_HIGH_PERFORMANCE:
                case LIS2DW12_HIGH_PERFORMANCE_LOW_NOISE:
                    *odr = 400.0f;
                    break;

                case LIS2DW12_CONT_LOW_PWR_4:
                case LIS2DW12_CONT_LOW_PWR_3:
                case LIS2DW12_CONT_LOW_PWR_2:
                case LIS2DW12_CONT_LOW_PWR_12bit:
                case LIS2DW12_SINGLE_LOW_PWR_4:
                case LIS2DW12_SINGLE_LOW_PWR_3:
                case LIS2DW12_SINGLE_LOW_PWR_2:
                case LIS2DW12_SINGLE_LOW_PWR_12bit:
                case LIS2DW12_CONT_LOW_PWR_LOW_NOISE_4:
                case LIS2DW12_CONT_LOW_PWR_LOW_NOISE_3:
                case LIS2DW12_CONT_LOW_PWR_LOW_NOISE_2:
                case LIS2DW12_CONT_LOW_PWR_LOW_NOISE_12bit:
                case LIS2DW12_SINGLE_LOW_PWR_LOW_NOISE_4:
                case LIS2DW12_SINGLE_LOW_PWR_LOW_NOISE_3:
                case LIS2DW12_SINGLE_LOW_PWR_LOW_NOISE_2:
                case LIS2DW12_SINGLE_LOW_LOW_NOISE_PWR_12bit:
                    *odr = 200.0f;
                    break;

                default:
                    *odr = -1.0f;
                    ret = 1;
                    break;
            }
            break;

        case LIS2DW12_XL_ODR_800Hz:
            switch (mode) {
                case LIS2DW12_HIGH_PERFORMANCE:
                case LIS2DW12_HIGH_PERFORMANCE_LOW_NOISE:
                    *odr = 800.0f;
                    break;

                case LIS2DW12_CONT_LOW_PWR_4:
                case LIS2DW12_CONT_LOW_PWR_3:
                case LIS2DW12_CONT_LOW_PWR_2:
                case LIS2DW12_CONT_LOW_PWR_12bit:
                case LIS2DW12_SINGLE_LOW_PWR_4:
                case LIS2DW12_SINGLE_LOW_PWR_3:
                case LIS2DW12_SINGLE_LOW_PWR_2:
                case LIS2DW12_SINGLE_LOW_PWR_12bit:
                case LIS2DW12_CONT_LOW_PWR_LOW_NOISE_4:
                case LIS2DW12_CONT_LOW_PWR_LOW_NOISE_3:
                case LIS2DW12_CONT_LOW_PWR_LOW_NOISE_2:
                case LIS2DW12_CONT_LOW_PWR_LOW_NOISE_12bit:
                case LIS2DW12_SINGLE_LOW_PWR_LOW_NOISE_4:
                case LIS2DW12_SINGLE_LOW_PWR_LOW_NOISE_3:
                case LIS2DW12_SINGLE_LOW_PWR_LOW_NOISE_2:
                case LIS2DW12_SINGLE_LOW_LOW_NOISE_PWR_12bit:
                    *odr = 200.0f;
                    break;

                default:
                    *odr = -1.0f;
                    ret = 1;
                    break;
            }
            break;

        case LIS2DW12_XL_ODR_1k6Hz:
            switch (mode) {
                case LIS2DW12_HIGH_PERFORMANCE:
                case LIS2DW12_HIGH_PERFORMANCE_LOW_NOISE:
                    *odr = 1600.0f;
                    break;

                case LIS2DW12_CONT_LOW_PWR_4:
                case LIS2DW12_CONT_LOW_PWR_3:
                case LIS2DW12_CONT_LOW_PWR_2:
                case LIS2DW12_CONT_LOW_PWR_12bit:
                case LIS2DW12_SINGLE_LOW_PWR_4:
                case LIS2DW12_SINGLE_LOW_PWR_3:
                case LIS2DW12_SINGLE_LOW_PWR_2:
                case LIS2DW12_SINGLE_LOW_PWR_12bit:
                case LIS2DW12_CONT_LOW_PWR_LOW_NOISE_4:
                case LIS2DW12_CONT_LOW_PWR_LOW_NOISE_3:
                case LIS2DW12_CONT_LOW_PWR_LOW_NOISE_2:
                case LIS2DW12_CONT_LOW_PWR_LOW_NOISE_12bit:
                case LIS2DW12_SINGLE_LOW_PWR_LOW_NOISE_4:
                case LIS2DW12_SINGLE_LOW_PWR_LOW_NOISE_3:
                case LIS2DW12_SINGLE_LOW_PWR_LOW_NOISE_2:
                case LIS2DW12_SINGLE_LOW_LOW_NOISE_PWR_12bit:
                    *odr = 200.0f;
                    break;

                default:
                    *odr = -1.0f;
                    ret = 1;
                    break;
            }
            break;

        default:
            *odr = -1.0f;
            ret = 1;
            break;
    }

    return ret;
}

/**
 * @brief  Set LIS2DW12 Accelerometer output data rate
 * @param  odr the output data rate to be set
 * @retval 0 in case of success, an error code otherwise
 */
int LIS2DW12Sensor::set_x_odr(float odr)
{
    return set_x_odr_with_mode(odr, LIS2DW12_HIGH_PERFORMANCE_MODE, LIS2DW12_LOW_NOISE_DISABLE);
}

/**
 * @brief  Set LIS2DW12 Accelerometer output data rate
 * @param  odr the output data rate to be set
 * @param  mode the operating mode to be used
 * @param  noise the low noise option
 * @retval 0 in case of success, an error code otherwise
 */
int LIS2DW12Sensor::set_x_odr_with_mode(float odr, LIS2DW12_Operating_Mode_t mode, LIS2DW12_Low_Noise_t noise)
{
    if (_x_is_enabled == 1) {
        if (set_x_odr_when_enabled(odr, mode, noise) != 0) {
            return 1;
        }
    } else {
        if (set_x_odr_when_disabled(odr, mode, noise) != 0) {
            return 1;
        }
    }

    return 0;
}

/**
 * @brief  Set LIS2DW12 Accelerometer output data rate when enabled
 * @param  odr the output data rate to be set
 * @param  mode the operating mode to be used
 * @param  noise the low noise option
 * @retval 0 in case of success, an error code otherwise
 */
int LIS2DW12Sensor::set_x_odr_when_enabled(float odr, LIS2DW12_Operating_Mode_t mode, LIS2DW12_Low_Noise_t noise)
{
    lis2dw12_odr_t new_odr;
    lis2dw12_mode_t new_power_mode;

    switch (mode) {
        case LIS2DW12_HIGH_PERFORMANCE_MODE:
        default:
            switch (noise) {
                case LIS2DW12_LOW_NOISE_DISABLE:
                default:
                    new_power_mode = LIS2DW12_HIGH_PERFORMANCE;
                    break;
                case LIS2DW12_LOW_NOISE_ENABLE:
                    new_power_mode = LIS2DW12_HIGH_PERFORMANCE_LOW_NOISE;
                    break;
            }

            /* If High Performance mode minimum ODR is 12.5Hz */
            if (odr < 12.5f) {
                odr = 12.5f;
            }
            break;
        case LIS2DW12_LOW_POWER_MODE4:
            switch (noise) {
                case LIS2DW12_LOW_NOISE_DISABLE:
                default:
                    new_power_mode = LIS2DW12_CONT_LOW_PWR_4;
                    break;
                case LIS2DW12_LOW_NOISE_ENABLE:
                    new_power_mode = LIS2DW12_CONT_LOW_PWR_LOW_NOISE_4;
                    break;
            }

            /* If Low Power mode maximum ODR is 200Hz */
            if (odr > 200.0f) {
                odr = 200.0f;
            }
            break;
        case LIS2DW12_LOW_POWER_MODE3:
            switch (noise) {
                case LIS2DW12_LOW_NOISE_DISABLE:
                default:
                    new_power_mode = LIS2DW12_CONT_LOW_PWR_3;
                    break;
                case LIS2DW12_LOW_NOISE_ENABLE:
                    new_power_mode = LIS2DW12_CONT_LOW_PWR_LOW_NOISE_3;
                    break;
            }

            /* If Low Power mode maximum ODR is 200Hz */
            if (odr > 200.0f) {
                odr = 200.0f;
            }
            break;
        case LIS2DW12_LOW_POWER_MODE2:
            switch (noise) {
                case LIS2DW12_LOW_NOISE_DISABLE:
                default:
                    new_power_mode = LIS2DW12_CONT_LOW_PWR_2;
                    break;
                case LIS2DW12_LOW_NOISE_ENABLE:
                    new_power_mode = LIS2DW12_CONT_LOW_PWR_LOW_NOISE_2;
                    break;
            }

            /* If Low Power mode maximum ODR is 200Hz */
            if (odr > 200.0f) {
                odr = 200.0f;
            }
            break;
        case LIS2DW12_LOW_POWER_MODE1:
            switch (noise) {
                case LIS2DW12_LOW_NOISE_DISABLE:
                default:
                    new_power_mode = LIS2DW12_CONT_LOW_PWR_12bit;
                    break;
                case LIS2DW12_LOW_NOISE_ENABLE:
                    new_power_mode = LIS2DW12_CONT_LOW_PWR_LOW_NOISE_12bit;
                    break;
            }

            /* If Low Power mode maximum ODR is 200Hz */
            if (odr > 200.0f) {
                odr = 200.0f;
            }
            break;
    }


    new_odr = (odr <=    1.6f) ? LIS2DW12_XL_ODR_1Hz6_LP_ONLY
              : (odr <=   12.5f) ? LIS2DW12_XL_ODR_12Hz5
              : (odr <=   25.0f) ? LIS2DW12_XL_ODR_25Hz
              : (odr <=   50.0f) ? LIS2DW12_XL_ODR_50Hz
              : (odr <=  100.0f) ? LIS2DW12_XL_ODR_100Hz
              : (odr <=  200.0f) ? LIS2DW12_XL_ODR_200Hz
              : (odr <=  400.0f) ? LIS2DW12_XL_ODR_400Hz
              : (odr <=  800.0f) ? LIS2DW12_XL_ODR_800Hz
              :                    LIS2DW12_XL_ODR_1k6Hz;

    /* Output data rate selection. */
    if (lis2dw12_data_rate_set(&_reg_ctx, new_odr) != 0) {
        return 1;
    }

    /* Power mode selection. */
    if (lis2dw12_power_mode_set(&_reg_ctx, new_power_mode) != 0) {
        return 1;
    }

    /* Store actual output data rate, operating mode and low noise. */
    _x_last_odr = odr;
    _x_last_operating_mode = mode;
    _x_last_noise = noise;

    return 0;
}

/**
 * @brief  Set LIS2DW12 Accelerometer output data rate when disabled
 * @param  odr the output data rate to be set
 * @param  mode the operating mode to be used
 * @param  noise the low noise option
 * @retval 0 in case of success, an error code otherwise
 */
int LIS2DW12Sensor::set_x_odr_when_disabled(float odr, LIS2DW12_Operating_Mode_t mode, LIS2DW12_Low_Noise_t noise)
{
    _x_last_operating_mode = mode;
    _x_last_noise = noise;

    _x_last_odr = (odr <=    1.6f) ?    1.6f
                  : (odr <=   12.5f) ?   12.5f
                  : (odr <=   25.0f) ?   25.0f
                  : (odr <=   50.0f) ?   50.0f
                  : (odr <=  100.0f) ?  100.0f
                  : (odr <=  200.0f) ?  200.0f
                  : (odr <=  400.0f) ?  400.0f
                  : (odr <=  800.0f) ?  800.0f
                  :                    1600.0f;

    return 0;
}

/**
 * @brief  Read LIS2DW12 Accelerometer full scale
 * @param  full_scale the pointer to the full scale
 * @retval 0 in case of success, an error code otherwise
 */
int LIS2DW12Sensor::get_x_fs(float *full_scale)
{
    int32_t ret = 0;
    lis2dw12_fs_t fs_low_level;

    /* Read actual full scale selection from sensor. */
    if (lis2dw12_full_scale_get(&_reg_ctx, &fs_low_level) != 0) {
        return 1;
    }

    switch (fs_low_level) {
        case LIS2DW12_2g:
            *full_scale =  2;
            break;

        case LIS2DW12_4g:
            *full_scale =  4;
            break;

        case LIS2DW12_8g:
            *full_scale =  8;
            break;

        case LIS2DW12_16g:
            *full_scale = 16;
            break;

        default:
            *full_scale = -1;
            ret = 1;
            break;
    }

    return ret;
}

/**
 * @brief  Set LIS2DW12 Accelerometer full scale
 * @param  full_scale the full scale to be set
 * @retval 0 in case of success, an error code otherwise
 */
int LIS2DW12Sensor::set_x_fs(float full_scale)
{
    lis2dw12_fs_t new_fs;

    /* Seems like MISRA C-2012 rule 14.3a violation but only from single file statical analysis point of view because
       the parameter passed to the function is not known at the moment of analysis */
    new_fs = (full_scale <= 2) ? LIS2DW12_2g
             : (full_scale <= 4) ? LIS2DW12_4g
             : (full_scale <= 8) ? LIS2DW12_8g
             :                    LIS2DW12_16g;

    if (lis2dw12_full_scale_set(&_reg_ctx, new_fs) != 0) {
        return 1;
    }

    return 0;
}

/**
 * @brief Enable the wake up detection for LIS2DW12 accelerometer sensor
 * @note  This function sets the LIS2DW12 accelerometer ODR to 200Hz and the LIS2DW12 accelerometer full scale to 2g
 * @retval 0 in case of success, an error code otherwise
 */
int LIS2DW12Sensor::enable_wake_up_detection(void)
{
    int32_t ret = 0;
    lis2dw12_ctrl4_int1_pad_ctrl_t val;

    /* Output Data Rate selection */
    if (set_x_odr(200.0f) != 0) {
        return 1;
    }

    /* Full scale selection */
    if (set_x_fs(2) != 0) {
        return 1;
    }

    /* WAKE_DUR setting */
    if (lis2dw12_wkup_dur_set(&_reg_ctx, 0x00) != 0) {
        return 1;
    }

    /* Set wake up threshold. */
    if (lis2dw12_wkup_threshold_set(&_reg_ctx, 0x02) != 0) {
        return 1;
    }

    if (lis2dw12_pin_int1_route_get(&_reg_ctx, &val) != 0) {
        return 1;
    }

    val.int1_wu = PROPERTY_ENABLE;

    if (lis2dw12_pin_int1_route_set(&_reg_ctx, &val) != 0) {
        return 1;
    }

    return ret;
}

/**
 * @brief Disable the wake up detection for LIS2DW12 accelerometer sensor
 * @retval 0 in case of success, an error code otherwise
 */
int LIS2DW12Sensor::disable_wake_up_detection(void)
{
    lis2dw12_ctrl4_int1_pad_ctrl_t ctrl4_int1_reg;
    lis2dw12_ctrl5_int2_pad_ctrl_t ctrl5_int2_reg;
    lis2dw12_ctrl_reg7_t ctrl_reg7;

    /* Disable wake up event on INT1 pin. */
    if (lis2dw12_read_reg(&_reg_ctx, LIS2DW12_CTRL4_INT1_PAD_CTRL, (uint8_t *)&ctrl4_int1_reg, 1) != 0) {
        return 1;
    }

    ctrl4_int1_reg.int1_wu = PROPERTY_DISABLE;

    if (lis2dw12_write_reg(&_reg_ctx, LIS2DW12_CTRL4_INT1_PAD_CTRL, (uint8_t *)&ctrl4_int1_reg, 1) != 0) {
        return 1;
    }

    /* Read INT2 Sleep Change */
    if (lis2dw12_read_reg(&_reg_ctx, LIS2DW12_CTRL5_INT2_PAD_CTRL, (uint8_t *)&ctrl5_int2_reg, 1) != 0) {
        return 1;
    }

    /*Disable Interrupts bit if none event is still enabled */
    if (ctrl5_int2_reg.int2_sleep_chg == 0 && ctrl4_int1_reg.int1_wu == 0 && ctrl4_int1_reg.int1_6d == 0) {
        if (lis2dw12_read_reg(&_reg_ctx, LIS2DW12_CTRL_REG7, (uint8_t *)&ctrl_reg7, 1) != 0) {
            return 1;
        }

        ctrl_reg7.interrupts_enable = PROPERTY_DISABLE;

        if (lis2dw12_write_reg(&_reg_ctx, LIS2DW12_CTRL_REG7, (uint8_t *)&ctrl_reg7, 1) != 0) {
            return 1;
        }
    }

    /* Reset wake up threshold. */
    if (lis2dw12_wkup_threshold_set(&_reg_ctx, 0x00) != 0) {
        return 1;
    }

    /* WAKE_DUR setting */
    if (lis2dw12_wkup_dur_set(&_reg_ctx, 0x00) != 0) {
        return 1;
    }

    return 0;
}

/**
 * @brief Set the wake up threshold for LIS2DW12 accelerometer sensor
 * @param thr the threshold to be set
 * @retval 0 in case of success, an error code otherwise
 */
int LIS2DW12Sensor::set_wake_up_threshold(uint8_t thr)
{
    /* Set wake up threshold. */
    if (lis2dw12_wkup_threshold_set(&_reg_ctx, thr) != 0) {
        return 1;
    }

    return 0;
}

/**
 * @brief Set the wake up duration for LIS2DW12 accelerometer sensor
 * @param dur the duration to be set
 * @retval 0 in case of success, an error code otherwise
 */
int LIS2DW12Sensor::set_wake_up_duration(uint8_t dur)
{
    /* Set wake up duration. */
    if (lis2dw12_wkup_dur_set(&_reg_ctx, dur) != 0) {
        return 1;
    }

    return 0;
}

/**
 * @brief Enable the inactivity detection for LIS2DW12 accelerometer sensor
 * @retval 0 in case of success, an error code otherwise
 */
int LIS2DW12Sensor::enable_inactivity_detection(void)
{
    int32_t ret = 0;
    lis2dw12_ctrl5_int2_pad_ctrl_t val;

    /* Output Data Rate and Full scale must be selected externally */

    /* SLEEP_DUR setting */
    if (lis2dw12_act_sleep_dur_set(&_reg_ctx, 0x01) != 0) {
        return 1;
    }

    /* Set wake up threshold. */
    if (lis2dw12_wkup_threshold_set(&_reg_ctx, 0x02) != 0) {
        return 1;
    }

    /* Enable inactivity detection. */
    if (lis2dw12_act_mode_set(&_reg_ctx, LIS2DW12_DETECT_ACT_INACT) != 0) {
        return 1;
    }

    if (lis2dw12_pin_int2_route_get(&_reg_ctx, &val) != 0) {
        return 1;
    }

    val.int2_sleep_chg = PROPERTY_ENABLE;

    if (lis2dw12_pin_int2_route_set(&_reg_ctx, &val) != 0) {
        return 1;
    }

    return ret;
}

/**
 * @brief Disable the inactivity detection for LIS2DW12 accelerometer sensor
 * @retval 0 in case of success, an error code otherwise
 */
int LIS2DW12Sensor::disable_inactivity_detection(void)
{
    lis2dw12_ctrl4_int1_pad_ctrl_t ctrl4_int1_reg;
    lis2dw12_ctrl5_int2_pad_ctrl_t ctrl5_int2_reg;
    lis2dw12_ctrl_reg7_t ctrl_reg7;

    /* Disable inactivity event on INT2 pin */
    if (lis2dw12_read_reg(&_reg_ctx, LIS2DW12_CTRL5_INT2_PAD_CTRL, (uint8_t *)&ctrl5_int2_reg, 1) != 0) {
        return 1;
    }

    ctrl5_int2_reg.int2_sleep_chg = PROPERTY_DISABLE;

    if (lis2dw12_write_reg(&_reg_ctx, LIS2DW12_CTRL5_INT2_PAD_CTRL, (uint8_t *)&ctrl5_int2_reg, 1) != 0) {
        return 1;
    }

    /* Read INT1 Wake Up event and INT1 6D Orientation event */
    if (lis2dw12_read_reg(&_reg_ctx, LIS2DW12_CTRL4_INT1_PAD_CTRL, (uint8_t *)&ctrl4_int1_reg, 1) != 0) {
        return 1;
    }

    /*Disable Interrupts bit if none event is still enabled */
    if (ctrl5_int2_reg.int2_sleep_chg == 0 && ctrl4_int1_reg.int1_wu == 0 && ctrl4_int1_reg.int1_6d == 0) {
        if (lis2dw12_read_reg(&_reg_ctx, LIS2DW12_CTRL_REG7, (uint8_t *)&ctrl_reg7, 1) != 0) {
            return 1;
        }

        ctrl_reg7.interrupts_enable = PROPERTY_DISABLE;

        if (lis2dw12_write_reg(&_reg_ctx, LIS2DW12_CTRL_REG7, (uint8_t *)&ctrl_reg7, 1) != 0) {
            return 1;
        }
    }

    /* Disable inactivity detection. */
    if (lis2dw12_act_mode_set(&_reg_ctx, LIS2DW12_NO_DETECTION) != 0) {
        return 1;
    }

    /* Reset wake up threshold. */
    if (lis2dw12_wkup_threshold_set(&_reg_ctx, 0x00) != 0) {
        return 1;
    }

    /* SLEEP_DUR setting */
    if (lis2dw12_act_sleep_dur_set(&_reg_ctx, 0x00) != 0) {
        return 1;
    }

    return 0;
}

/**
 * @brief Set the sleep duration for LIS2DW12 accelerometer sensor
 * @param dur the duration to be set
 * @retval 0 in case of success, an error code otherwise
 */
int LIS2DW12Sensor::set_sleep_duration(uint8_t dur)
{
    /* Set sleep duration. */
    if (lis2dw12_act_sleep_dur_set(&_reg_ctx, dur) != 0) {
        return 1;
    }

    return 0;
}

/**
 * @brief Enable the 6D orientation detection for LIS2DW12 accelerometer sensor
 * @note  This function sets the LIS2DW12 accelerometer ODR to 200Hz and the LIS2DW12 accelerometer full scale to 2g
 * @retval 0 in case of success, an error code otherwise
 */
int LIS2DW12Sensor::enable_6d_orientation(void)
{
    int32_t ret = 0;
    lis2dw12_ctrl4_int1_pad_ctrl_t val;

    /* Output Data Rate selection */
    if (set_x_odr(200.0f) == 1) {
        return 1;
    }

    /* Full scale selection. */
    if (set_x_fs(2.0f) == 1) {
        return 1;
    }

    /* 6D orientation threshold. */
    if (lis2dw12_6d_threshold_set(&_reg_ctx, 2) != 0) { /* 60 degrees */
        return 1;
    }

    /* Enable 6D orientation event on INT1 pin */
    if (lis2dw12_pin_int1_route_get(&_reg_ctx, &val) != 0) {
        return 1;
    }

    val.int1_6d = PROPERTY_ENABLE;

    if (lis2dw12_pin_int1_route_set(&_reg_ctx, &val) != 0) {
        return 1;
    }

    return ret;
}

/**
 * @brief Disable the 6D orientation detection for LIS2DW12 accelerometer sensor
 * @retval 0 in case of success, an error code otherwise
 */
int LIS2DW12Sensor::disable_6d_orientation(void)
{
    lis2dw12_ctrl4_int1_pad_ctrl_t ctrl4_int1_reg;
    lis2dw12_ctrl5_int2_pad_ctrl_t ctrl5_int2_reg;
    lis2dw12_ctrl_reg7_t ctrl_reg7;

    /* Disable 6D orientation event on INT1 pin */
    if (lis2dw12_read_reg(&_reg_ctx, LIS2DW12_CTRL4_INT1_PAD_CTRL, (uint8_t *)&ctrl4_int1_reg, 1) != 0) {
        return 1;
    }

    ctrl4_int1_reg.int1_6d = PROPERTY_DISABLE;

    if (lis2dw12_write_reg(&_reg_ctx, LIS2DW12_CTRL4_INT1_PAD_CTRL, (uint8_t *)&ctrl4_int1_reg, 1) != 0) {
        return 1;
    }

    /* Read INT2 Sleep Change */
    if (lis2dw12_read_reg(&_reg_ctx, LIS2DW12_CTRL5_INT2_PAD_CTRL, (uint8_t *)&ctrl5_int2_reg, 1) != 0) {
        return 1;
    }

    /*Disable Interrupts bit if none event is still enabled */
    if (ctrl5_int2_reg.int2_sleep_chg == 0 && ctrl4_int1_reg.int1_wu == 0 && ctrl4_int1_reg.int1_6d == 0) {
        if (lis2dw12_read_reg(&_reg_ctx, LIS2DW12_CTRL_REG7, (uint8_t *)&ctrl_reg7, 1) != 0) {
            return 1;
        }

        ctrl_reg7.interrupts_enable = PROPERTY_DISABLE;

        if (lis2dw12_write_reg(&_reg_ctx, LIS2DW12_CTRL_REG7, (uint8_t *)&ctrl_reg7, 1) != 0) {
            return 1;
        }
    }

    /* Reset 6D orientation threshold. */
    if (lis2dw12_6d_threshold_set(&_reg_ctx, 0) != 0) {
        return 1;
    }

    return 0;
}

/**
 * @brief Set the 6D orientation threshold for LIS2DW12 accelerometer sensor
 * @param thr the threshold to be set
 * @retval 0 in case of success, an error code otherwise
 */
int LIS2DW12Sensor::set_6d_orientation_threshold(uint8_t thr)
{
    if (thr > 3) {
        return 1;
    }

    if (lis2dw12_6d_threshold_set(&_reg_ctx, thr) != 0) {
        return 1;
    }

    return 0;
}

/**
 * @brief Get the 6D orientation XL axis for LIS2DW12 accelerometer sensor
 * @param xl the pointer to the 6D orientation XL axis
 * @retval 0 in case of success, an error code otherwise
 */
int LIS2DW12Sensor::get_6d_orientation_xl(uint8_t *xl)
{
    lis2dw12_sixd_src_t data;

    if (lis2dw12_read_reg(&_reg_ctx, LIS2DW12_SIXD_SRC, (uint8_t *)&data, 1) != 0) {
        return 1;
    }

    *xl = data.xl;

    return 0;
}

/**
 * @brief Get the 6D orientation XH axis for LIS2DW12 accelerometer sensor
 * @param xh the pointer to the 6D orientation XH axis
 * @retval 0 in case of success, an error code otherwise
 */
int LIS2DW12Sensor::get_6d_orientation_xh(uint8_t *xh)
{
    lis2dw12_sixd_src_t data;

    if (lis2dw12_read_reg(&_reg_ctx, LIS2DW12_SIXD_SRC, (uint8_t *)&data, 1) != 0) {
        return 1;
    }

    *xh = data.xh;

    return 0;
}

/**
 * @brief Get the 6D orientation YL axis for LIS2DW12 accelerometer sensor
 * @param yl the pointer to the 6D orientation YL axis
 * @retval 0 in case of success, an error code otherwise
 */
int LIS2DW12Sensor::get_6d_orientation_yl(uint8_t *yl)
{
    lis2dw12_sixd_src_t data;

    if (lis2dw12_read_reg(&_reg_ctx, LIS2DW12_SIXD_SRC, (uint8_t *)&data, 1) != 0) {
        return 1;
    }

    *yl = data.yl;

    return 0;
}

/**
 * @brief Get the 6D orientation YH axis for LIS2DW12 accelerometer sensor
 * @param yh the pointer to the 6D orientation YH axis
 * @retval 0 in case of success, an error code otherwise
 */
int LIS2DW12Sensor::get_6d_orientation_yh(uint8_t *yh)
{
    lis2dw12_sixd_src_t data;

    if (lis2dw12_read_reg(&_reg_ctx, LIS2DW12_SIXD_SRC, (uint8_t *)&data, 1) != 0) {
        return 1;
    }

    *yh = data.yh;

    return 0;
}

/**
 * @brief Get the 6D orientation ZL axis for LIS2DW12 accelerometer sensor
 * @param zl the pointer to the 6D orientation ZL axis
 * @retval 0 in case of success, an error code otherwise
 */
int LIS2DW12Sensor::get_6d_orientation_zl(uint8_t *zl)
{
    lis2dw12_sixd_src_t data;

    if (lis2dw12_read_reg(&_reg_ctx, LIS2DW12_SIXD_SRC, (uint8_t *)&data, 1) != 0) {
        return 1;
    }

    *zl = data.zl;

    return 0;
}

/**
 * @brief Get the 6D orientation ZH axis for LIS2DW12 accelerometer sensor
 * @param zh the pointer to the 6D orientation ZH axis
 * @retval 0 in case of success, an error code otherwise
 */
int LIS2DW12Sensor::get_6d_orientation_zh(uint8_t *zh)
{
    lis2dw12_sixd_src_t data;

    if (lis2dw12_read_reg(&_reg_ctx, LIS2DW12_SIXD_SRC, (uint8_t *)&data, 1) != 0) {
        return 1;
    }

    *zh = data.zh;

    return 0;
}

/**
 * @brief Get the status of all hardware events for LIS2DW12 accelerometer sensor
 * @param status the pointer to the status of all hardware events
 * @retval 0 in case of success, an error code otherwise
 */
int LIS2DW12Sensor::get_event_status(LIS2DW12_Event_Status_t *status)
{
    lis2dw12_status_t status_reg;
    lis2dw12_ctrl4_int1_pad_ctrl_t ctrl4_int1_reg;
    lis2dw12_ctrl5_int2_pad_ctrl_t ctrl5_int2_reg;

    (void)memset((void *)status, 0x0, sizeof(LIS2DW12_Event_Status_t));

    if (lis2dw12_read_reg(&_reg_ctx, LIS2DW12_STATUS, (uint8_t *)&status_reg, 1) != 0) {
        return 1;
    }

    if (lis2dw12_read_reg(&_reg_ctx, LIS2DW12_CTRL4_INT1_PAD_CTRL, (uint8_t *)&ctrl4_int1_reg, 1) != 0) {
        return 1;
    }

    if (lis2dw12_read_reg(&_reg_ctx, LIS2DW12_CTRL5_INT2_PAD_CTRL, (uint8_t *)&ctrl5_int2_reg, 1) != 0) {
        return 1;
    }

    if (ctrl4_int1_reg.int1_wu == 1U) {
        if (status_reg.wu_ia == 1U) {
            status->WakeUpStatus = 1;
        }
    }

    if (ctrl4_int1_reg.int1_6d == 1U) {
        if (status_reg._6d_ia == 1U) {
            status->D6DOrientationStatus = 1;
        }
    }

    if (ctrl5_int2_reg.int2_sleep_chg == 1U) {
        if (status_reg.sleep_state == 1U) {
            status->SleepStatus = 1;
        }
    }

    return 0;
}

/**
 * @brief  Get the number of samples contained into the FIFO
 * @param  num_samples the number of samples contained into the FIFO
 * @retval 0 in case of success, an error code otherwise
 */
int LIS2DW12Sensor::get_fifo_num_samples(uint16_t *num_samples)
{
    lis2dw12_fifo_samples_t fifo_samples;

    if (lis2dw12_read_reg(&_reg_ctx, LIS2DW12_FIFO_SAMPLES, (uint8_t *)&fifo_samples, 1) != 0) {
        return 1;
    }

    if (fifo_samples.diff == 0x20) {
        *num_samples = 32;
    } else {
        *num_samples = fifo_samples.diff;
    }

    return 0;
}

/**
 * @brief  Set the FIFO mode
 * @param  mode FIFO mode
 * @retval 0 in case of success, an error code otherwise
 */
int LIS2DW12Sensor::set_fifo_mode(uint8_t mode)
{
    int32_t ret = 0;

    /* Verify that the passed parameter contains one of the valid values. */
    switch ((lis2dw12_fmode_t)mode) {
        case LIS2DW12_BYPASS_MODE:
        case LIS2DW12_FIFO_MODE:
        case LIS2DW12_STREAM_TO_FIFO_MODE:
        case LIS2DW12_BYPASS_TO_STREAM_MODE:
        case LIS2DW12_STREAM_MODE:
            break;

        default:
            ret = 1;
            break;
    }

    if (ret == 1) {
        return ret;
    }

    if (lis2dw12_fifo_mode_set(&_reg_ctx, (lis2dw12_fmode_t)mode) != 0) {
        return 1;
    }

    return ret;
}

/**
 * @brief Read the data from register
 * @param reg register address
 * @param data register data
 * @retval 0 in case of success, an error code otherwise
 */
int LIS2DW12Sensor::read_reg(uint8_t reg, uint8_t *data)
{

    if (lis2dw12_read_reg(&_reg_ctx, reg, data, 1) != 0) {
        return 1;
    }

    return 0;
}

/**
 * @brief Write the data to register
 * @param reg register address
 * @param data register data
 * @retval 0 in case of success, an error code otherwise
 */
int LIS2DW12Sensor::write_reg(uint8_t reg, uint8_t data)
{

    if (lis2dw12_write_reg(&_reg_ctx, reg, &data, 1) != 0) {
        return 1;
    }

    return 0;
}


int32_t LIS2DW12_io_write(void *handle, uint8_t WriteAddr, uint8_t *pBuffer, uint16_t nBytesToWrite)
{
    return ((LIS2DW12Sensor *)handle)->io_write(pBuffer, WriteAddr, nBytesToWrite);
}

int32_t LIS2DW12_io_read(void *handle, uint8_t ReadAddr, uint8_t *pBuffer, uint16_t nBytesToRead)
{
    return ((LIS2DW12Sensor *)handle)->io_read(pBuffer, ReadAddr, nBytesToRead);
}
