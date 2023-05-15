/**
 ******************************************************************************
 * @file    XNucleoIKS01A3.cpp
 * @author  SRA
 * @version V1.0.0
 * @date    5-March-2019
 * @brief   Implementation file for the X_NUCLEO_IKS01A3 singleton class
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
#include "mbed.h"
#include "XNucleoIKS01A3.h"

/* Static variables ----------------------------------------------------------*/
XNucleoIKS01A3 *XNucleoIKS01A3::_instance = NULL;


/* Methods -------------------------------------------------------------------*/
/**
 * @brief  Constructor
 */
XNucleoIKS01A3::XNucleoIKS01A3(DevI2C *ext_i2c, PinName lsm6dso_int1, PinName lsm6dso_int2, PinName lis2dw12_int1, PinName lps22hh_int, PinName stts751_int) : dev_i2c(ext_i2c),
    ht_sensor(new HTS221Sensor(dev_i2c)),
    magnetometer(new LIS2MDLSensor(dev_i2c)),
    accelerometer(new LIS2DW12Sensor(dev_i2c, LIS2DW12_I2C_ADD_H, lis2dw12_int1)),
    pt_sensor(new LPS22HHSensor(dev_i2c, LPS22HH_I2C_ADD_H, lps22hh_int)),
    acc_gyro(new LSM6DSOSensor(dev_i2c, LSM6DSO_I2C_ADD_H, lsm6dso_int1, lsm6dso_int2)),
    t_sensor(new STTS751Sensor(dev_i2c, STTS751_1xxxx_ADD_7K5, stts751_int))
{
    ht_sensor->init(NULL);
    magnetometer->init(NULL);
    accelerometer->init(NULL);
    pt_sensor->init(NULL);
    acc_gyro->init(NULL);
    t_sensor->init(NULL);
}

/**
 * @brief     Get singleton instance
 * @return    a pointer to the initialized singleton instance of class XNucleoIKS01A3.
 *            A return value of NULL indicates an out of memory situation.
 * @param[in] ext_i2c (optional) pointer to an instance of DevI2C to be used
 *            for communication on the expansion board.
 *            Defaults to NULL.
 *            Taken into account only on the very first call of one of the 'Instance' functions.
 *            If not provided a new DevI2C will be created with standard
 *            configuration parameters.
 *            The used DevI2C object gets saved in instance variable dev_i2c.
 * @param[in] lsm6dso_int1 LSM6DSO INT1 pin.
 *            Taken into account only on the very first call of one of the 'Instance' functions.
 *            It maps the INT1 pin for LSM6DSO. Defaults to IKS01A3_PIN_LSM6DSO_INT1.
 * @param[in] lsm6dso_int2 LSM6DSO INT2 pin.
 *            Taken into account only on the very first call of one of the 'Instance' functions.
 *            It maps the INT2 pin for LSM6DSO. Defaults to IKS01A3_PIN_LSM6DSO_INT2.
 * @param[in] lis2dw12_int1 LIS2DW12 INT1 pin.
 *            Taken into account only on the very first call of one of the 'Instance' functions.
 *            It maps the INT1 pin for LIS2DW12. Defaults to IKS01A3_PIN_LIS2DW12_INT1.
 * @param[in] lps22hh_int LPS22HH INT pin.
 *            Taken into account only on the very first call of one of the 'Instance' functions.
 *            It maps the INT pin for LPS22HH. Defaults to IKS01A3_PIN_LPS22HH_INT.
 * @param[in] stts751_int STTS751 INT pin.
 *            Taken into account only on the very first call of one of the 'Instance' functions.
 *            It maps the INT pin for STTS751. Defaults to IKS01A3_PIN_STTS751_INT.
 */
XNucleoIKS01A3 *XNucleoIKS01A3::instance(DevI2C *ext_i2c, PinName lsm6dso_int1, PinName lsm6dso_int2, PinName lis2dw12_int1, PinName lps22hh_int, PinName stts751_int)
{
    if (_instance == NULL) {
        if (ext_i2c == NULL) {
            ext_i2c = new DevI2C(IKS01A3_PIN_I2C_SDA, IKS01A3_PIN_I2C_SCL);
        }

        if (ext_i2c != NULL) {
            _instance = new XNucleoIKS01A3(ext_i2c, lsm6dso_int1, lsm6dso_int2, lis2dw12_int1, lps22hh_int, stts751_int);
        }
    }

    return _instance;
}

/**
 * @brief     Get singleton instance
 * @return    a pointer to the initialized singleton instance of class X_NUCLEO_IKS01A3.
 *            A return value of NULL indicates an out of memory situation.
 * @param[in] sda I2C data line pin.
 *            Taken into account only on the very first call of one of the 'Instance' functions.
 *            A new DevI2C will be created based on parameters 'sda' and 'scl'.
 *            The used DevI2C object gets saved in instance variable dev_i2c.
 * @param[in] scl I2C clock line pin.
 *            Taken into account only on the very first call of one of the 'Instance' functions.
 *            A new DevI2C will be created based on parameters 'sda' and 'scl'.
 *            The used DevI2C object gets saved in instance variable dev_i2c.
 * @param[in] lsm6dso_int1 LSM6DSO INT1 pin.
 *            Taken into account only on the very first call of one of the 'Instance' functions.
 *            It maps the INT1 pin for LSM6DSO. Defaults to IKS01A3_PIN_LSM6DSO_INT1.
 * @param[in] lsm6dso_int2 LSM6DSO INT2 pin.
 *            Taken into account only on the very first call of one of the 'Instance' functions.
 *            It maps the INT2 pin for LSM6DSO. Defaults to IKS01A3_PIN_LSM6DSO_INT2.
 * @param[in] lis2dw12_int1 LIS2DW12 INT1 pin.
 *            Taken into account only on the very first call of one of the 'Instance' functions.
 *            It maps the INT1 pin for LIS2DW12. Defaults to IKS01A3_PIN_LIS2DW12_INT1.
 * @param[in] lps22hh_int LPS22HH INT pin.
 *            Taken into account only on the very first call of one of the 'Instance' functions.
 *            It maps the INT pin for LPS22HH. Defaults to IKS01A3_PIN_LPS22HH_INT.
 * @param[in] stts751_int STTS751 INT pin.
 *            Taken into account only on the very first call of one of the 'Instance' functions.
 *            It maps the INT pin for STTS751. Defaults to IKS01A3_PIN_STTS751_INT.
 */
XNucleoIKS01A3 *XNucleoIKS01A3::instance(PinName sda, PinName scl, PinName lsm6dso_int1, PinName lsm6dso_int2, PinName lis2dw12_int1, PinName lps22hh_int, PinName stts751_int)
{
    if (_instance == NULL) {
        DevI2C *ext_i2c = new DevI2C(sda, scl);

        if (ext_i2c != NULL) {
            _instance = new XNucleoIKS01A3(ext_i2c, lsm6dso_int1, lsm6dso_int2, lis2dw12_int1, lps22hh_int, stts751_int);
        }
    }

    return _instance;
}
