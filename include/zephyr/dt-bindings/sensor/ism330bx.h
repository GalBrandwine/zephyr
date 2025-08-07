/*
 * Copyright (c) 2023 STMicroelectronics
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef ZEPHYR_INCLUDE_DT_BINDINGS_ST_ISM330BX_H_
#define ZEPHYR_INCLUDE_DT_BINDINGS_ST_ISM330BX_H_

/*****************************************
 *                                        *
 *                                        *
 *           ACCL config options          *
 *                                        *
 *                                        *
 *****************************************/

// Accel range
#define ISM330BX_ACCL_DT_FS_2G         0 // (0.061 mg/LSB)
#define ISM330BX_ACCL_DT_FS_4G         1 // (0.122 mg/LSB)
#define ISM330BX_ACCL_DT_FS_8G         2 // (0.244 mg/LSB)
#define ISM330BX_ACCL_DT_FS_16G        3 // (0.488 mg/LSB)
// Accel odr
#define ISM330BX_ACCL_DT_ODR_OFF       0x00
#define ISM330BX_ACCL_DT_ODR_AT_1Hz875 0x01 // Low-power mode 1/2/3 only
#define ISM330BX_ACCL_DT_ODR_AT_7Hz5   0x02 // High-performance mode only
#define ISM330BX_ACCL_DT_ODR_AT_15Hz   0x03
#define ISM330BX_ACCL_DT_ODR_AT_30Hz   0x04
#define ISM330BX_ACCL_DT_ODR_AT_60Hz   0x05
#define ISM330BX_ACCL_DT_ODR_AT_120Hz  0x06
#define ISM330BX_ACCL_DT_ODR_AT_240Hz  0x07
#define ISM330BX_ACCL_DT_ODR_AT_480Hz  0x08 // High-performance mode only
#define ISM330BX_ACCL_DT_ODR_AT_960Hz  0x09 // High-performance mode only
#define ISM330BX_ACCL_DT_ODR_AT_1920Hz 0x0a // High-performance mode only
#define ISM330BX_ACCL_DT_ODR_AT_3840Hz 0x0b // High-performance mode only

/*
Accelerometer power modes and output data rates.

When the accelerometer is configured in power-down mode (ACCL ODR set to 0), almost all internal
blocks of the device are switched off to minimize power consumption. Digital interfaces (I²C, MIPI
I3C®, and SPI) are still active to allow communication with the device. The content of the
configuration registers is preserved and the output data registers are not updated, keeping the last
data sampled in memory before going into power-down mode.

When the accelerometer is configured in high-performance mode, its reading chain is always on. The
antialiasing filter is enabled and the accelerometer ODR is selectable up to 3840 Hz.
High-performance mode provides the best performance in terms of noise.

NOTE:
The power-down mode is selected if ODR_XL = 0000, regardless of the configuration of the OP_MODE_XL
bits.
*/
#define ISM330BX_ACCEL_PM_HP_DEFAULT 0
#define ISM330BX_ACCEL_PM_HP_TDM     1

/*
When the accelerometer is configured in low-power mode, its reading chain is automatically turned on
and off to optimize the supply current. Increasing the number of averaged measurements allows
reducing the noise, while decreasing them allows reducing the supply current. In the low-power
modes, the antialiasing filter is disabled and accelerometer ODR is selectable up to 240 Hz.
*/
#define ISM330BX_ACCEL_PM_LP_MODE_1 4 // where two measurements are averaged
#define ISM330BX_ACCEL_PM_LP_MODE_2 5 // where four measurements are averaged
#define ISM330BX_ACCEL_PM_LP_MODE_3 6 // where eight measurements are averaged

/*****************************************
 *                                        *
 *                                        *
 *           Gyro config options          *
 *                                        *
 *                                        *
 *****************************************/

// Gyro range
#define ISM330BX_GYRO_DT_FS_125DPS  0x0 // (4.375 mdps/LSB)
#define ISM330BX_GYRO_DT_FS_250DPS  0x1 // (8.75 mdps/LSB)
#define ISM330BX_GYRO_DT_FS_500DPS  0x2 // (17.50 mdps/LSB)
#define ISM330BX_GYRO_DT_FS_1000DPS 0x3 // (35 mdps/LSB)
#define ISM330BX_GYRO_DT_FS_2000DPS 0x4 // (70 mdps/LSB)
#define ISM330BX_GYRO_DT_FS_4000DPS 0xc // (140 mdps/LSB)

// Gyro odr
#define ISM330BX_GYRO_DT_ODR_OFF       0x00
#define ISM330BX_GYRO_DT_ODR_AT_7Hz5   0x02
#define ISM330BX_GYRO_DT_ODR_AT_15Hz   0x03
#define ISM330BX_GYRO_DT_ODR_AT_30Hz   0x04
#define ISM330BX_GYRO_DT_ODR_AT_60Hz   0x05
#define ISM330BX_GYRO_DT_ODR_AT_120Hz  0x06
#define ISM330BX_GYRO_DT_ODR_AT_240Hz  0x07
#define ISM330BX_GYRO_DT_ODR_AT_480Hz  0x08 // High-performance mode only
#define ISM330BX_GYRO_DT_ODR_AT_960Hz  0x09 // High-performance mode only
#define ISM330BX_GYRO_DT_ODR_AT_1920Hz 0x0a // High-performance mode only
#define ISM330BX_GYRO_DT_ODR_AT_3840Hz 0x0b // High-performance mode only

/*
When the gyroscope is configured in high-performance mode, its reading chain is always on.
The gyroscope ODR is selectable up to 3840 Hz.
High-performance mode provides the best performance in terms of noise.
*/
#define ISM330BX_GYRO_PM_HP_DEFAULT 0

/*
When the gyroscope is configured in power-down mode (GYRO ODR is 0), almost all internal blocks of
the device are switched off to minimize the supply current. Digital interfaces (I²C, MIPI I3C®, and
SPI) are still active to allow communication with the device. The content of the configuration
registers is preserved and the output data registers are not updated, keeping the last data sampled
in memory before going into power-down mode.

When the gyroscope is in sleep mode, the circuitry that drives the oscillation of the gyroscope mass
is active, but the reading chain is turned off. Compared to power-down mode, the turn-on time from
sleep mode to any active mode is drastically reduced.
*/
#define ISM330BX_GYRO_PM_SLEEP 4

/*
When the gyroscope is configured in low-power mode, the driving circuitry is always on,
but the reading chain is automatically turned on and off to optimize the supply current.
The gyroscope ODR is selectable up to 240 Hz.
*/
#define ISM330BX_GYRO_PM_LP 5

/*****************************************
 *                                        *
 *                                        *
 *           Wakeup config options        *
 *                                        *
 *                                        *
 *****************************************/

#define ISM330BX_WAKEUP_THRESHOLD_7_8125 0
#define ISM330BX_WAKEUP_THRESHOLD_15_625 1
#define ISM330BX_WAKEUP_THRESHOLD_31_25  2
#define ISM330BX_WAKEUP_THRESHOLD_62_5   3
#define ISM330BX_WAKEUP_THRESHOLD_125    4
#define ISM330BX_WAKEUP_THRESHOLD_250    5

#endif /* ZEPHYR_INCLUDE_DT_BINDINGS_ST_ISM330BX_H_ */
