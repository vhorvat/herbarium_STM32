/**
  ******************************************************************************
  * @file    x_nucleo_iks01a3_targets.h
  * @author  SRA
  * @version V1.0.0
  * @date    5-March-2019
  * @brief   This header file is intended to manage the differences between
  *          the different supported base-boards which might mount the
  *          X_NUCLEO_IKS01A3 MEMS Inertial & Environmental Nucleo Expansion Board.
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
#ifndef _X_NUCLEO_IKS01A3_TARGETS_H_
#define _X_NUCLEO_IKS01A3_TARGETS_H_

/*** I2C ***/
/* Use Arduino I2C Connectors */
#define IKS01A3_PIN_I2C_SDA         (D14)
#define IKS01A3_PIN_I2C_SCL         (D15)

/* LSM6DSO INT1 */
#define IKS01A3_PIN_LSM6DSO_INT1     (D4)
/* LSM6DSO INT2 */
#define IKS01A3_PIN_LSM6DSO_INT2     (D5)
/* LIS2DW12 INT1 */
#define IKS01A3_PIN_LIS2DW12_INT1    (A3)
/* LPS22HH INT */
#define IKS01A3_PIN_LPS22HH_INT      (D6)
/* STTS751 INT */
#define IKS01A3_PIN_STTS751_INT      (A4)

#endif // _X_NUCLEO_IKS01A3_TARGETS_H_
