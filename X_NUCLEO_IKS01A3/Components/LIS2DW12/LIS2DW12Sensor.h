/**
 ******************************************************************************
 * @file    LIS2DW12Sensor.h
 * @author  CLab
 * @version V1.0.0
 * @date    15 November 2018
 * @brief   Abstract Class of an LIS2DW12 Inertial Measurement Unit (IMU) 3 axes
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


/* Prevent recursive inclusion -----------------------------------------------*/

#ifndef __LIS2DW12Sensor_H__
#define __LIS2DW12Sensor_H__


/* Includes ------------------------------------------------------------------*/

#include "DevI2C.h"
#include "lis2dw12_reg.h"
#include "MotionSensor.h"
#include <assert.h>

/* Defines -------------------------------------------------------------------*/

#define LIS2DW12_ACC_SENSITIVITY_FOR_FS_2G_LOPOW1_MODE   0.976f  /**< Sensitivity value for 2g full scale, Low-power1 mode [mg/LSB] */
#define LIS2DW12_ACC_SENSITIVITY_FOR_FS_2G_OTHER_MODES   0.244f  /**< Sensitivity value for 2g full scale, all other modes except Low-power1 [mg/LSB] */

#define LIS2DW12_ACC_SENSITIVITY_FOR_FS_4G_LOPOW1_MODE   1.952f  /**< Sensitivity value for 4g full scale, Low-power1 mode [mg/LSB] */
#define LIS2DW12_ACC_SENSITIVITY_FOR_FS_4G_OTHER_MODES   0.488f  /**< Sensitivity value for 4g full scale, all other modes except Low-power1 [mg/LSB] */

#define LIS2DW12_ACC_SENSITIVITY_FOR_FS_8G_LOPOW1_MODE   3.904f  /**< Sensitivity value for 8g full scale, Low-power1 mode [mg/LSB] */
#define LIS2DW12_ACC_SENSITIVITY_FOR_FS_8G_OTHER_MODES   0.976f  /**< Sensitivity value for 8g full scale, all other modes except Low-power1 [mg/LSB] */

#define LIS2DW12_ACC_SENSITIVITY_FOR_FS_16G_LOPOW1_MODE  7.808f  /**< Sensitivity value for 16g full scale, Low-power1 mode [mg/LSB] */
#define LIS2DW12_ACC_SENSITIVITY_FOR_FS_16G_OTHER_MODES  1.952f  /**< Sensitivity value for 16g full scale, all other modes except Low-power1 [mg/LSB] */

/* Typedefs ------------------------------------------------------------------*/

typedef struct {
    unsigned int WakeUpStatus : 1;
    unsigned int D6DOrientationStatus : 1;
    unsigned int SleepStatus : 1;
} LIS2DW12_Event_Status_t;

typedef enum {
    LIS2DW12_HIGH_PERFORMANCE_MODE,
    LIS2DW12_LOW_POWER_MODE4,
    LIS2DW12_LOW_POWER_MODE3,
    LIS2DW12_LOW_POWER_MODE2,
    LIS2DW12_LOW_POWER_MODE1
} LIS2DW12_Operating_Mode_t;

typedef enum {
    LIS2DW12_LOW_NOISE_DISABLE,
    LIS2DW12_LOW_NOISE_ENABLE
} LIS2DW12_Low_Noise_t;

/* Class Declaration ---------------------------------------------------------*/

/**
 * Abstract class of an LIS2DW12 Inertial Measurement Unit (IMU) 3 axes
 * sensor.
 */
class LIS2DW12Sensor : public MotionSensor {
public:
    enum SPI_type_t {SPI3W, SPI4W};
    LIS2DW12Sensor(DevI2C *i2c, uint8_t address = LIS2DW12_I2C_ADD_H, PinName int1_pin = NC, PinName int2_pin = NC);
    LIS2DW12Sensor(SPI *spi, PinName cs_pin, PinName int1_pin = NC, PinName int2_pin = NC, SPI_type_t spi_type = SPI4W);
    virtual int init(void *init);
    virtual int read_id(uint8_t *id);
    virtual int get_x_axes(int32_t *pData);
    virtual int get_x_sensitivity(float *pfData);
    virtual int get_x_axes_raw(int16_t *pData);
    virtual int get_x_odr(float *odr);
    virtual int set_x_odr(float odr);
    virtual int get_x_fs(float *fullScale);
    virtual int set_x_fs(float fullScale);
    int set_x_odr_with_mode(float odr, LIS2DW12_Operating_Mode_t mode = LIS2DW12_HIGH_PERFORMANCE_MODE, LIS2DW12_Low_Noise_t noise = LIS2DW12_LOW_NOISE_DISABLE);
    int enable_x(void);
    int disable_x(void);
    int enable_wake_up_detection(void);
    int disable_wake_up_detection(void);
    int set_wake_up_threshold(uint8_t thr);
    int set_wake_up_duration(uint8_t dur);
    int enable_inactivity_detection(void);
    int disable_inactivity_detection(void);
    int set_sleep_duration(uint8_t dur);
    int enable_6d_orientation(void);
    int disable_6d_orientation(void);
    int set_6d_orientation_threshold(uint8_t thr);
    int get_6d_orientation_xl(uint8_t *xl);
    int get_6d_orientation_xh(uint8_t *xh);
    int get_6d_orientation_yl(uint8_t *yl);
    int get_6d_orientation_yh(uint8_t *yh);
    int get_6d_orientation_zl(uint8_t *zl);
    int get_6d_orientation_zh(uint8_t *zh);
    int get_event_status(LIS2DW12_Event_Status_t *status);
    int get_fifo_num_samples(uint16_t *num_samples);
    int set_fifo_mode(uint8_t mode);
    int read_reg(uint8_t reg, uint8_t *data);
    int write_reg(uint8_t reg, uint8_t data);

    /**
     * @brief  Attaching an interrupt handler to the INT1 interrupt.
     * @param  fptr An interrupt handler.
     * @retval None.
     */
    void attach_int1_irq(void (*fptr)(void))
    {
        _int1_irq.rise(fptr);
    }

    /**
     * @brief  Enabling the INT1 interrupt handling.
     * @param  None.
     * @retval None.
     */
    void enable_int1_irq(void)
    {
        _int1_irq.enable_irq();
    }

    /**
     * @brief  Disabling the INT1 interrupt handling.
     * @param  None.
     * @retval None.
     */
    void disable_int1_irq(void)
    {
        _int1_irq.disable_irq();
    }

    /**
     * @brief  Attaching an interrupt handler to the INT2 interrupt.
     * @param  fptr An interrupt handler.
     * @retval None.
     */
    void attach_int2_irq(void (*fptr)(void))
    {
        _int2_irq.rise(fptr);
    }

    /**
     * @brief  Enabling the INT2 interrupt handling.
     * @param  None.
     * @retval None.
     */
    void enable_int2_irq(void)
    {
        _int2_irq.enable_irq();
    }

    /**
     * @brief  Disabling the INT2 interrupt handling.
     * @param  None.
     * @retval None.
     */
    void disable_int2_irq(void)
    {
        _int2_irq.disable_irq();
    }

    /**
     * @brief Utility function to read data.
     * @param  pBuffer: pointer to data to be read.
     * @param  RegisterAddr: specifies internal address register to be read.
     * @param  NumByteToRead: number of bytes to be read.
     * @retval 0 if ok, an error code otherwise.
     */
    uint8_t io_read(uint8_t *pBuffer, uint8_t RegisterAddr, uint16_t NumByteToRead)
    {
        if (_dev_spi) {
            /* Write Reg Address */
            _dev_spi->lock();
            _cs_pin = 0;
            if (_spi_type == SPI4W) {
                _dev_spi->write(RegisterAddr | 0x80);
                for (int i = 0; i < NumByteToRead; i++) {
                    *(pBuffer + i) = _dev_spi->write(0x00);
                }
            } else if (_spi_type == SPI3W) {
                /* Write RD Reg Address with RD bit*/
                uint8_t TxByte = RegisterAddr | 0x80;
                _dev_spi->write((char *)&TxByte, 1, (char *)pBuffer, (int) NumByteToRead);
            }
            _cs_pin = 1;
            _dev_spi->unlock();
            return 0;
        }
        if (_dev_i2c) {
            return (uint8_t) _dev_i2c->i2c_read(pBuffer, _address, RegisterAddr, NumByteToRead);
        }
        return 1;
    }

    /**
     * @brief Utility function to write data.
     * @param  pBuffer: pointer to data to be written.
     * @param  RegisterAddr: specifies internal address register to be written.
     * @param  NumByteToWrite: number of bytes to write.
     * @retval 0 if ok, an error code otherwise.
     */
    uint8_t io_write(uint8_t *pBuffer, uint8_t RegisterAddr, uint16_t NumByteToWrite)
    {
        if (_dev_spi) {
            _dev_spi->lock();
            _cs_pin = 0;
            _dev_spi->write(RegisterAddr);
            _dev_spi->write((char *)pBuffer, (int) NumByteToWrite, NULL, 0);
            _cs_pin = 1;
            _dev_spi->unlock();
            return 0;
        }
        if (_dev_i2c) {
            return (uint8_t) _dev_i2c->i2c_write(pBuffer, _address, RegisterAddr, NumByteToWrite);
        }
        return 1;
    }

private:
    int set_x_odr_when_enabled(float odr, LIS2DW12_Operating_Mode_t mode, LIS2DW12_Low_Noise_t noise);
    int set_x_odr_when_disabled(float odr, LIS2DW12_Operating_Mode_t mode, LIS2DW12_Low_Noise_t noise);

    /* Helper classes. */
    DevI2C *_dev_i2c;
    SPI    *_dev_spi;

    /* Configuration */
    uint8_t _address;
    DigitalOut  _cs_pin;
    InterruptIn _int1_irq;
    InterruptIn _int2_irq;
    SPI_type_t _spi_type;

    uint8_t _x_is_enabled;
    float _x_last_odr;
    LIS2DW12_Operating_Mode_t _x_last_operating_mode;
    LIS2DW12_Low_Noise_t _x_last_noise;

    lis2dw12_ctx_t _reg_ctx;
};

#ifdef __cplusplus
extern "C" {
#endif
int32_t LIS2DW12_io_write(void *handle, uint8_t WriteAddr, uint8_t *pBuffer, uint16_t nBytesToWrite);
int32_t LIS2DW12_io_read(void *handle, uint8_t ReadAddr, uint8_t *pBuffer, uint16_t nBytesToRead);
#ifdef __cplusplus
}
#endif

#endif
