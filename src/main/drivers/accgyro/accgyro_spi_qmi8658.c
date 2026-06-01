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

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include "platform.h"

#ifdef USE_ACCGYRO_QMI8658

#include "drivers/accgyro/accgyro.h"
#include "drivers/accgyro/accgyro_spi_qmi8658.h"
#include "drivers/bus_spi.h"
#include "drivers/exti.h"
#include "drivers/io.h"
#include "drivers/nvic.h"
#include "drivers/sensor.h"
#include "drivers/system.h"
#include "drivers/time.h"

#include "sensors/gyro.h"

// Use a conservative clock until the target board has been verified at the device maximum.
#define QMI8658_MAX_SPI_CLK_HZ 10000000

// Need to see at least this many interrupts during initialisation to confirm EXTI connectivity.
#define GYRO_EXTI_DETECT_THRESHOLD 1000

#define QMI8658_CTRL1_ADDR_AI             (1 << 6)
#define QMI8658_CTRL1_INT1_ENABLE         (1 << 3)
#define QMI8658_CTRL1_INT2_ENABLE         (1 << 4)

// Override from a target if the board wires data-ready to INT2 instead of INT1.
#ifndef QMI8658_CTRL1_INT_CONFIG
#define QMI8658_CTRL1_INT_CONFIG          QMI8658_CTRL1_INT1_ENABLE
#endif

#define QMI8658_CTRL2_ACC_FS_16G          (0x03 << 4)
#define QMI8658_CTRL2_ACC_ODR_896HZ       0x03

#define QMI8658_CTRL3_GYRO_FS_2048DPS     (0x07 << 4)
#define QMI8658_CTRL3_GYRO_ODR_7174HZ     0x00

#define QMI8658_CTRL5_GYRO_LPF_ENABLE     (1 << 4)
#define QMI8658_CTRL5_ACC_LPF_ENABLE      (1 << 0)
#define QMI8658_CTRL5_ACC_LPF_ODR_13_37   (0x03 << 1)

#define QMI8658_CTRL7_ACC_ENABLE          (1 << 0)
#define QMI8658_CTRL7_GYRO_ENABLE         (1 << 1)

#define QMI8658_RESET_CMD                 0xB0

static int16_t qmi8658ReadInt16LE(const uint8_t *data)
{
    return (int16_t)((uint16_t)data[0] | ((uint16_t)data[1] << 8));
}

static uint8_t qmi8658RegisterRead(const extDevice_t *dev, qmi8658Register_e registerId)
{
    return spiReadRegMsk(dev, registerId);
}

static void qmi8658RegisterWrite(const extDevice_t *dev, qmi8658Register_e registerId, uint8_t value, unsigned delayMs)
{
    spiWriteReg(dev, registerId, value);
    if (delayMs) {
        delay(delayMs);
    }
}

uint8_t qmi8658Detect(const extDevice_t *dev)
{
    for (int attempt = 0; attempt < 5; attempt++) {
        const uint8_t whoAmI = qmi8658RegisterRead(dev, QMI8658_REG_WHO_AM_I);
        if (whoAmI == QMI8658_WHO_AM_I_CONST) {
            return QMI_8658_SPI;
        }
        delay(1);
    }

    return MPU_NONE;
}

static uint8_t qmi8658GyroLpfConfig(void)
{
    switch (gyroConfig()->gyro_hardware_lpf) {
    case GYRO_HARDWARE_LPF_NORMAL:
        return 0x00; // 2.66% ODR
    case GYRO_HARDWARE_LPF_OPTION_1:
        return 0x01; // 3.63% ODR
    case GYRO_HARDWARE_LPF_OPTION_2:
        return 0x02; // 5.39% ODR
#ifdef USE_GYRO_DLPF_EXPERIMENTAL
    case GYRO_HARDWARE_LPF_EXPERIMENTAL:
        return 0x03; // 13.37% ODR
#endif
    default:
        return 0x00;
    }
}

static void qmi8658Config(gyroDev_t *gyro)
{
    const extDevice_t *dev = &gyro->dev;

    qmi8658RegisterWrite(dev, QMI8658_REG_RESET, QMI8658_RESET_CMD, 20);

    qmi8658RegisterWrite(dev, QMI8658_REG_CTRL1,
        QMI8658_CTRL1_ADDR_AI | QMI8658_CTRL1_INT_CONFIG,
        1);

    qmi8658RegisterWrite(dev, QMI8658_REG_CTRL7, 0x00, 1);

    qmi8658RegisterWrite(dev, QMI8658_REG_CTRL2,
        QMI8658_CTRL2_ACC_FS_16G | QMI8658_CTRL2_ACC_ODR_896HZ,
        1);

    qmi8658RegisterWrite(dev, QMI8658_REG_CTRL3,
        QMI8658_CTRL3_GYRO_FS_2048DPS | QMI8658_CTRL3_GYRO_ODR_7174HZ,
        1);

    qmi8658RegisterWrite(dev, QMI8658_REG_CTRL5,
        (qmi8658GyroLpfConfig() << 5) |
        QMI8658_CTRL5_GYRO_LPF_ENABLE |
        QMI8658_CTRL5_ACC_LPF_ODR_13_37 |
        QMI8658_CTRL5_ACC_LPF_ENABLE,
        1);

    qmi8658RegisterWrite(dev, QMI8658_REG_CTRL7,
        QMI8658_CTRL7_GYRO_ENABLE | QMI8658_CTRL7_ACC_ENABLE,
        150);
}

#ifdef USE_DMA
static busStatus_e qmi8658IntCallback(uintptr_t arg)
{
    gyroDev_t *gyro = (gyroDev_t *)arg;
    const int32_t gyroDmaDuration = cmpTimeCycles(getCycleCounter(), gyro->gyroLastEXTI);

    if (gyroDmaDuration > gyro->gyroDmaMaxDuration) {
        gyro->gyroDmaMaxDuration = gyroDmaDuration;
    }

    gyro->dataReady = true;

    return BUS_READY;
}
#endif

static void qmi8658ExtiHandler(extiCallbackRec_t *cb)
{
    gyroDev_t *gyro = container_of(cb, gyroDev_t, exti);

    const uint32_t nowCycles = getCycleCounter();
    gyro->gyroSyncEXTI = gyro->gyroLastEXTI + gyro->gyroDmaMaxDuration;
    gyro->gyroLastEXTI = nowCycles;

    if (gyro->gyroModeSPI == GYRO_EXTI_INT_DMA) {
        spiSequence(&gyro->dev, gyro->segments);
    }

    gyro->detectedEXTI++;
}

static void qmi8658IntExtiInit(gyroDev_t *gyro)
{
    if (gyro->mpuIntExtiTag == IO_TAG_NONE) {
        return;
    }

    const IO_t mpuIntIO = IOGetByTag(gyro->mpuIntExtiTag);

    IOInit(mpuIntIO, OWNER_GYRO_EXTI, 0);
    EXTIHandlerInit(&gyro->exti, qmi8658ExtiHandler);
    EXTIConfig(mpuIntIO, &gyro->exti, NVIC_PRIO_MPU_INT_EXTI, IOCFG_IN_FLOATING, BETAFLIGHT_EXTI_TRIGGER_RISING);
    EXTIEnable(mpuIntIO);
}

static bool qmi8658AccRead(accDev_t *acc)
{
    extDevice_t *dev = &acc->gyro->dev;

    switch (acc->gyro->gyroModeSPI) {
    case GYRO_EXTI_INT:
    case GYRO_EXTI_NO_INT:
    {
        dev->txBuf[0] = QMI8658_REG_AX_L | 0x80;

        busSegment_t segments[] = {
            {.u.buffers = {NULL, NULL}, 7, true, NULL},
            {.u.link = {NULL, NULL}, 0, true, NULL},
        };
        segments[0].u.buffers.txData = dev->txBuf;
        segments[0].u.buffers.rxData = dev->rxBuf;

        spiSequence(dev, &segments[0]);
        spiWait(dev);

        FALLTHROUGH;
    }

    case GYRO_EXTI_INT_DMA:
        acc->ADCRaw[X] = qmi8658ReadInt16LE(&dev->rxBuf[1]);
        acc->ADCRaw[Y] = qmi8658ReadInt16LE(&dev->rxBuf[3]);
        acc->ADCRaw[Z] = qmi8658ReadInt16LE(&dev->rxBuf[5]);
        break;

    case GYRO_EXTI_INIT:
    default:
        break;
    }

    return true;
}

static bool qmi8658GyroRead(gyroDev_t *gyro)
{
    extDevice_t *dev = &gyro->dev;

    switch (gyro->gyroModeSPI) {
    case GYRO_EXTI_INIT:
    {
        memset(dev->txBuf, 0x00, 14);
        gyro->gyroDmaMaxDuration = 5;

        if (gyro->detectedEXTI > GYRO_EXTI_DETECT_THRESHOLD) {
#ifdef USE_DMA
            if (spiUseDMA(dev)) {
                dev->callbackArg = (uintptr_t)gyro;
                dev->txBuf[0] = QMI8658_REG_AX_L | 0x80;
                gyro->segments[0].len = 13;
                gyro->segments[0].callback = qmi8658IntCallback;
                gyro->segments[0].u.buffers.txData = dev->txBuf;
                gyro->segments[0].u.buffers.rxData = dev->rxBuf;
                gyro->segments[0].negateCS = true;
                gyro->gyroModeSPI = GYRO_EXTI_INT_DMA;
            } else
#endif
            {
                gyro->gyroModeSPI = GYRO_EXTI_INT;
            }
        } else {
            gyro->gyroModeSPI = GYRO_EXTI_NO_INT;
        }
        break;
    }

    case GYRO_EXTI_INT:
    case GYRO_EXTI_NO_INT:
    {
        dev->txBuf[0] = QMI8658_REG_GX_L | 0x80;

        busSegment_t segments[] = {
            {.u.buffers = {NULL, NULL}, 7, true, NULL},
            {.u.link = {NULL, NULL}, 0, true, NULL},
        };
        segments[0].u.buffers.txData = dev->txBuf;
        segments[0].u.buffers.rxData = dev->rxBuf;

        spiSequence(dev, &segments[0]);
        spiWait(dev);

        gyro->gyroADCRaw[X] = qmi8658ReadInt16LE(&dev->rxBuf[1]);
        gyro->gyroADCRaw[Y] = qmi8658ReadInt16LE(&dev->rxBuf[3]);
        gyro->gyroADCRaw[Z] = qmi8658ReadInt16LE(&dev->rxBuf[5]);
        break;
    }

    case GYRO_EXTI_INT_DMA:
        gyro->gyroADCRaw[X] = qmi8658ReadInt16LE(&dev->rxBuf[7]);
        gyro->gyroADCRaw[Y] = qmi8658ReadInt16LE(&dev->rxBuf[9]);
        gyro->gyroADCRaw[Z] = qmi8658ReadInt16LE(&dev->rxBuf[11]);
        break;

    default:
        break;
    }

    return true;
}

static void qmi8658SpiGyroInit(gyroDev_t *gyro)
{
    extDevice_t *dev = &gyro->dev;

    busSegment_t nullSegment = {.u.link = {NULL, NULL}, 0, false, NULL};
    gyro->segments[0] = nullSegment;
    gyro->segments[1] = nullSegment;
    gyro->accDataReg = QMI8658_REG_AX_L;
    gyro->gyroDataReg = QMI8658_REG_GX_L;
    gyro->tempDataReg = QMI8658_REG_TEMP_L;
    gyro->dmaReadRegStart = QMI8658_REG_AX_L;

    qmi8658Config(gyro);
    qmi8658IntExtiInit(gyro);

    spiSetClkDivisor(dev, spiCalculateDivider(QMI8658_MAX_SPI_CLK_HZ));
}

static void qmi8658SpiAccInit(accDev_t *acc)
{
    // Sensor is configured during gyro init.
    acc->acc_1G = 512 * 4; // 16G sensor scale, 2048 LSB/g
}

bool qmi8658SpiAccDetect(accDev_t *acc)
{
    if (acc->mpuDetectionResult.sensor != QMI_8658_SPI) {
        return false;
    }

    acc->initFn = qmi8658SpiAccInit;
    acc->readFn = qmi8658AccRead;

    return true;
}

bool qmi8658SpiGyroDetect(gyroDev_t *gyro)
{
    if (gyro->mpuDetectionResult.sensor != QMI_8658_SPI) {
        return false;
    }

    gyro->initFn = qmi8658SpiGyroInit;
    gyro->readFn = qmi8658GyroRead;
    gyro->scale = GYRO_SCALE_2048DPS;

    return true;
}

#endif // USE_ACCGYRO_QMI8658
