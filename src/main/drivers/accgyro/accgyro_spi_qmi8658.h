/*
 * This file is part of Cleanflight and Betaflight.
 *
 * Cleanflight and Betaflight are free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Cleanflight and Betaflight are distributed in the hope that they
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#include "drivers/bus.h"

// QMI8658 registers used by the SPI ACC/GYRO driver.
typedef enum {
    QMI8658_REG_WHO_AM_I     = 0x00,
    QMI8658_REG_REVISION_ID  = 0x01,
    QMI8658_REG_CTRL1        = 0x02,
    QMI8658_REG_CTRL2        = 0x03,
    QMI8658_REG_CTRL3        = 0x04,
    QMI8658_REG_CTRL5        = 0x06,
    QMI8658_REG_CTRL7        = 0x08,
    QMI8658_REG_CTRL9        = 0x0A,
    QMI8658_REG_STATUS0      = 0x2E,
    QMI8658_REG_TEMP_L       = 0x33,
    QMI8658_REG_TEMP_H       = 0x34,
    QMI8658_REG_AX_L         = 0x35,
    QMI8658_REG_AX_H         = 0x36,
    QMI8658_REG_AY_L         = 0x37,
    QMI8658_REG_AY_H         = 0x38,
    QMI8658_REG_AZ_L         = 0x39,
    QMI8658_REG_AZ_H         = 0x3A,
    QMI8658_REG_GX_L         = 0x3B,
    QMI8658_REG_GX_H         = 0x3C,
    QMI8658_REG_GY_L         = 0x3D,
    QMI8658_REG_GY_H         = 0x3E,
    QMI8658_REG_GZ_L         = 0x3F,
    QMI8658_REG_GZ_H         = 0x40,
    QMI8658_REG_RESET        = 0x60,
} qmi8658Register_e;

uint8_t qmi8658Detect(const extDevice_t *dev);
bool qmi8658SpiAccDetect(accDev_t *acc);
bool qmi8658SpiGyroDetect(gyroDev_t *gyro);
