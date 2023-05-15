/**
 ******************************************************************************
 * @file    STTS751Sensor.h
 * @author  SRA
 * @version V1.0.0
 * @date    February 2018
 * @brief   Abstract Class of a STTS751 temperature sensor.
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

#ifndef __STTS751Sensor_H__
#define __STTS751Sensor_H__


/* Includes ------------------------------------------------------------------*/

#include "DevI2C.h"
#include "stts751_reg.h"
#include "TempSensor.h"
#include <assert.h>

/* Defines -------------------------------------------------------------------*/
/* Typedefs ------------------------------------------------------------------*/
/* Class Declaration ---------------------------------------------------------*/

/**
 * Abstract class of a STTS751 temperature sensor.
 */
class STTS751Sensor : public TempSensor {
public:

    STTS751Sensor(DevI2C *i2c, uint8_t address = STTS751_0xxxx_ADD_7K5, PinName int_pin = NC);
    virtual int init(void *init);
    virtual int read_id(uint8_t *id);
    virtual int get_temperature(float *value);
    int enable(void);
    int disable(void);
    int get_odr(float *odr);
    int set_odr(float odr);
    int read_reg(uint8_t reg, uint8_t *data);
    int write_reg(uint8_t reg, uint8_t data);
    int get_temp_drdy_status(uint8_t *status);
    int set_low_temp_thr(float value);
    int set_high_temp_thr(float value);
    int get_temp_limit_status(uint8_t *high_limit, uint8_t *low_limit, uint8_t *therm_limit);
    int set_event_pin(uint8_t enable);
    int set_one_shot();
    int get_one_shot_status(uint8_t *status);

    /**
     * @brief  Attaching an interrupt handler to the INT interrupt.
     * @param  fptr an interrupt handler.
     * @retval None.
     */
    void attach_int_irq(void (*fptr)(void))
    {
        _int_irq.fall(fptr);
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
        if (_dev_i2c) {
            return (uint8_t) _dev_i2c->i2c_write(pBuffer, _address, RegisterAddr, NumByteToWrite);
        }
        return 1;
    }

private:

    /* Helper classes. */
    DevI2C *_dev_i2c;

    /* Configuration */
    uint8_t _address;
    InterruptIn _int_irq;

    uint8_t _is_enabled;
    float _last_odr;

    stts751_ctx_t _reg_ctx;
};

#ifdef __cplusplus
extern "C" {
#endif
int32_t STTS751_io_write(void *handle, uint8_t WriteAddr, uint8_t *pBuffer, uint16_t nBytesToWrite);
int32_t STTS751_io_read(void *handle, uint8_t ReadAddr, uint8_t *pBuffer, uint16_t nBytesToRead);
#ifdef __cplusplus
}
#endif

#endif
