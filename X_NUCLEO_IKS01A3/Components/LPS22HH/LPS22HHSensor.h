/**
 ******************************************************************************
 * @file    LPS22HHSensor.h
 * @author  SRA
 * @version V1.0.0
 * @date    February 2019
 * @brief   Abstract Class of a LPS22HH pressure sensor.
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


/* Prevent recursive inclusion -----------------------------------------------*/

#ifndef __LPS22HHSensor_H__
#define __LPS22HHSensor_H__


/* Includes ------------------------------------------------------------------*/

#include "DevI2C.h"
#include "lps22hh_reg.h"
#include "PressureSensor.h"
#include "TempSensor.h"
#include <assert.h>

/* Defines -------------------------------------------------------------------*/
/* Typedefs ------------------------------------------------------------------*/
/* Class Declaration ---------------------------------------------------------*/

/**
 * Abstract class of a LPS22HH pressure sensor.
 */
class LPS22HHSensor : public PressureSensor, public TempSensor {
public:
    enum SPI_type_t {SPI3W, SPI4W};
    LPS22HHSensor(SPI *spi, PinName cs_pin, PinName int_pin = NC, SPI_type_t spi_type = SPI4W);
    LPS22HHSensor(DevI2C *i2c, uint8_t address = LPS22HH_I2C_ADD_H, PinName int_pin = NC);
    virtual int init(void *init);
    virtual int read_id(uint8_t *id);
    virtual int get_pressure(float *value);
    virtual int get_temperature(float *value);
    int enable(void);
    int disable(void);
    int get_odr(float *odr);
    int set_odr(float odr);
    int read_reg(uint8_t reg, uint8_t *data);
    int write_reg(uint8_t reg, uint8_t data);
    int get_press_drdy_status(uint8_t *status);
    int get_temp_drdy_status(uint8_t *status);
    int get_fifo_data(float *press, float *temp);
    int get_fifo_fth_status(uint8_t *status);
    int get_fifo_full_status(uint8_t *status);
    int get_fifo_ovr_status(uint8_t *status);
    int get_fifo_level(uint8_t *status);
    int reset_fifo_interrupt(uint8_t interrupt);
    int set_fifo_interrupt(uint8_t interrupt);
    int set_fifo_mode(uint8_t mode);
    int set_fifo_watermark_level(uint8_t watermark);
    int stop_fifo_on_watermark(uint8_t stop);
    int set_one_shot();
    int get_one_shot_status(uint8_t *status);

    /**
     * @brief  Attaching an interrupt handler to the INT interrupt.
     * @param  fptr an interrupt handler.
     * @retval None.
     */
    void attach_int_irq(void (*fptr)(void))
    {
        _int_irq.rise(fptr);
    }

    /**
     * @brief  Enabling the INT interrupt handling.
     * @param  None.
     * @retval None.
     */
    void enable_int_irq(void)
    {
        _int_irq.enable_irq();
    }

    /**
     * @brief  Disabling the INT interrupt handling.
     * @param  None.
     * @retval None.
     */
    void disable_int_irq(void)
    {
        _int_irq.disable_irq();
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
    int set_odr_when_enabled(float odr);
    int set_odr_when_disabled(float odr);

    /* Helper classes. */
    DevI2C *_dev_i2c;
    SPI    *_dev_spi;

    /* Configuration */
    uint8_t _address;
    DigitalOut  _cs_pin;
    InterruptIn _int_irq;
    SPI_type_t _spi_type;

    uint8_t _is_enabled;
    lps22hh_odr_t _last_odr;

    lps22hh_ctx_t _reg_ctx;

};

#ifdef __cplusplus
extern "C" {
#endif
int32_t LPS22HH_io_write(void *handle, uint8_t WriteAddr, uint8_t *pBuffer, uint16_t nBytesToWrite);
int32_t LPS22HH_io_read(void *handle, uint8_t ReadAddr, uint8_t *pBuffer, uint16_t nBytesToRead);
#ifdef __cplusplus
}
#endif

#endif
