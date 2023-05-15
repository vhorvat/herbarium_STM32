/**
 ******************************************************************************
 * @file    XNucleoIKS01A3.h
 * @author  SRA
 * @version V1.0.0
 * @date    5-March-2019
 * @brief   Header file for class XNucleoIKS01A3 representing a X-NUCLEO-IKS01A3
 *          expansion board
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

/* Define to prevent from recursive inclusion --------------------------------*/
#ifndef __X_NUCLEO_IKS01A3_H
#define __X_NUCLEO_IKS01A3_H

/* Includes ------------------------------------------------------------------*/
#include "mbed.h"
#include "x_nucleo_iks01a3_targets.h"
#include "HTS221Sensor.h"
#include "LIS2DW12Sensor.h"
#include "LIS2MDLSensor.h"
#include "LPS22HHSensor.h"
#include "LSM6DSOSensor.h"
#include "STTS751Sensor.h"
#include "DevI2C.h"

/* Macros -------------------------------------------------------------------*/
#define CALL_METH(obj, meth, param, ret) ((obj == NULL) ?       \
                      ((*(param) = (ret)), 0) : \
                      ((obj)->meth(param))      \
                      )

/* Classes -------------------------------------------------------------------*/
/** Class XNucleoIKS01A3 is intended to represent the MEMS Inertial & Environmental
 *  Nucleo Expansion Board with the same name.
 *
 *  The expansion board is featuring basically four IPs:\n
 *  -# a HTS221 Relative Humidity and Temperature Sensor\n
 *  -# a LPS22HH MEMS Pressure Sensor (and Temperature Sensor)\n
 *  -# a STTS751 MEMS Temperature Sensor\n
 *  -# a LIS2DW12 3D Acceleromenter\n
 *  -# a LIS2MDL 3D Magnetometer\n
 *  -# and a LSM6DSO 3D Acceleromenter and 3D Gyroscope\n
 *
 * The expansion board features also a DIL 24-pin socket which makes it possible
 * to add further MEMS adapters and other sensors (e.g. UV index).
 *
 * It is intentionally implemented as a singleton because only one
 * X_NUCLEO_IKS01A3 at a time might be deployed in a HW component stack.\n
 * In order to get the singleton instance you have to call class method `Instance()`,
 * e.g.:
 * @code
 * // Inertial & Environmental expansion board singleton instance
 * static X_NUCLEO_IKS01A3 *<TODO>_expansion_board = X_NUCLEO_IKS01A3::Instance();
 * @endcode
 */
class XNucleoIKS01A3 {
protected:
    XNucleoIKS01A3(DevI2C *ext_i2c, PinName lsm6dso_int1, PinName lsm6dso_int2, PinName lis2dw12_int1, PinName lps22hh_int, PinName stts751_int);

    ~XNucleoIKS01A3(void)
    {
        /* should never be called */
        error("Trial to delete XNucleoIKS01A3 singleton!\n");
    }

public:
    static XNucleoIKS01A3 *instance(DevI2C *ext_i2c = NULL, PinName lsm6dso_int1 = IKS01A3_PIN_LSM6DSO_INT1, PinName lsm6dso_int2 = IKS01A3_PIN_LSM6DSO_INT2, PinName lis2dw12_int1 = IKS01A3_PIN_LIS2DW12_INT1, PinName lps22hh_int = IKS01A3_PIN_LPS22HH_INT, PinName stts751_int = IKS01A3_PIN_STTS751_INT);
    static XNucleoIKS01A3 *instance(PinName sda, PinName scl, PinName lsm6dso_int1 = IKS01A3_PIN_LSM6DSO_INT1, PinName lsm6dso_int2 = IKS01A3_PIN_LSM6DSO_INT2, PinName lis2dw12_int1 = IKS01A3_PIN_LIS2DW12_INT1, PinName lps22hh_int = IKS01A3_PIN_LPS22HH_INT, PinName stts751_int = IKS01A3_PIN_STTS751_INT);

    DevI2C  *dev_i2c;

    HTS221Sensor  *ht_sensor;
    LIS2MDLSensor *magnetometer;
    LIS2DW12Sensor *accelerometer;
    LPS22HHSensor  *pt_sensor;
    LSM6DSOSensor *acc_gyro;
    STTS751Sensor  *t_sensor;

private:
    static XNucleoIKS01A3 *_instance;
};

#endif /* __X_NUCLEO_IKS01A3_H */
