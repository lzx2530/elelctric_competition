#include "bsp/bsp_i2c.h"
#include "bsp/bsp_spi_imu.h"

#include "ti_msp_dl_config.h"
#include <ti/driverlib/dl_spi.h>

#define BSP_I2C_TIMEOUT_LOOPS    (50000UL)
#define BSP_I2C_MAX_BURST        (32U)
#define BSP_SPI_IMU_TIMEOUT_LOOPS (5000U)
#define BSP_SPI_IMU_CLOCK_DIVIDER (15U)

static status_t bsp_i2c_wait_idle(void)
{
    uint32_t timeout = BSP_I2C_TIMEOUT_LOOPS;

    /* 所有事务都在控制器空闲后再发起，避免前一笔传输尾巴未收干净。 */
    while ((DL_I2C_getControllerStatus(I2C_SENSOR_BUS_INST) & DL_I2C_CONTROLLER_STATUS_IDLE) == 0U) {
        if (timeout-- == 0U) {
            return STATUS_TIMEOUT;
        }
    }

    return STATUS_OK;
}

static status_t bsp_i2c_wait_bus_complete(void)
{
    uint32_t timeout = BSP_I2C_TIMEOUT_LOOPS;
    uint32_t status;

    /* 这里等待总线忙标志释放，再统一检查 error 位。 */
    while ((DL_I2C_getControllerStatus(I2C_SENSOR_BUS_INST) & DL_I2C_CONTROLLER_STATUS_BUSY_BUS) != 0U) {
        if (timeout-- == 0U) {
            return STATUS_TIMEOUT;
        }
    }

    status = DL_I2C_getControllerStatus(I2C_SENSOR_BUS_INST);
    if ((status & DL_I2C_CONTROLLER_STATUS_ERROR) != 0U) {
        return STATUS_ERROR;
    }

    return STATUS_OK;
}

static status_t bsp_i2c_write_raw(uint8_t dev_addr, const uint8_t *data, uint16_t length)
{
    status_t ret;
    uint16_t written;

    ret = bsp_i2c_wait_idle();
    if (ret != STATUS_OK) {
        return ret;
    }

    /* 先灌 FIFO 再发起 start，保持和 TI DriverLib 示例一致。 */
    if (length == 0U) {
        DL_I2C_startControllerTransfer(I2C_SENSOR_BUS_INST, dev_addr, DL_I2C_CONTROLLER_DIRECTION_TX, 0U);
        return bsp_i2c_wait_bus_complete();
    }

    written = DL_I2C_fillControllerTXFIFO(I2C_SENSOR_BUS_INST, (uint8_t *) data, length);
    DL_I2C_startControllerTransfer(I2C_SENSOR_BUS_INST, dev_addr, DL_I2C_CONTROLLER_DIRECTION_TX, length);

    while (written < length) {
        uint32_t timeout = BSP_I2C_TIMEOUT_LOOPS;

        while (DL_I2C_isControllerTXFIFOFull(I2C_SENSOR_BUS_INST)) {
            if ((DL_I2C_getControllerStatus(I2C_SENSOR_BUS_INST) & DL_I2C_CONTROLLER_STATUS_ERROR) != 0U) {
                return STATUS_ERROR;
            }
            if (timeout-- == 0U) {
                return STATUS_TIMEOUT;
            }
        }

        written += DL_I2C_fillControllerTXFIFO(
            I2C_SENSOR_BUS_INST,
            (uint8_t *) &data[written],
            (uint16_t) (length - written));
    }

    return bsp_i2c_wait_bus_complete();
}

void bsp_i2c_init(void)
{
}

status_t bsp_i2c_probe(uint8_t dev_addr)
{
    uint8_t dummy = 0U;
    return bsp_i2c_write_raw(dev_addr, &dummy, 0U);
}

status_t bsp_i2c_write_bytes(uint8_t dev_addr, const uint8_t *data, uint16_t length)
{
    if ((data == NULL) || (length == 0U)) {
        return STATUS_INVALID_ARG;
    }

    return bsp_i2c_write_raw(dev_addr, data, length);
}

status_t bsp_i2c_mem_write(uint8_t dev_addr, uint8_t reg_addr, const uint8_t *data, uint16_t length)
{
    uint8_t buffer[BSP_I2C_MAX_BURST + 1U];

    if ((data == NULL) || (length > BSP_I2C_MAX_BURST)) {
        return STATUS_INVALID_ARG;
    }

    buffer[0] = reg_addr;
    for (uint16_t i = 0U; i < length; i++) {
        buffer[i + 1U] = data[i];
    }

    return bsp_i2c_write_raw(dev_addr, buffer, (uint16_t) (length + 1U));
}

status_t bsp_i2c_mem_read(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, uint16_t length)
{
    status_t ret;
    uint16_t i;

    if ((data == NULL) || (length == 0U)) {
        return STATUS_INVALID_ARG;
    }

    /* 常见寄存器读流程：先写寄存器地址，再发起读事务。 */
    ret = bsp_i2c_write_raw(dev_addr, &reg_addr, 1U);
    if (ret != STATUS_OK) {
        return ret;
    }

    ret = bsp_i2c_wait_idle();
    if (ret != STATUS_OK) {
        return ret;
    }

    DL_I2C_startControllerTransfer(I2C_SENSOR_BUS_INST, dev_addr, DL_I2C_CONTROLLER_DIRECTION_RX, length);
    for (i = 0U; i < length; i++) {
        uint32_t timeout = BSP_I2C_TIMEOUT_LOOPS;
        /* 逐字节轮询 RX FIFO，起步阶段先保持阻塞式实现，逻辑更稳。 */
        while (DL_I2C_isControllerRXFIFOEmpty(I2C_SENSOR_BUS_INST)) {
            if (timeout-- == 0U) {
                return STATUS_TIMEOUT;
            }
        }
        data[i] = DL_I2C_receiveControllerData(I2C_SENSOR_BUS_INST);
    }

    return bsp_i2c_wait_bus_complete();
}

static void bsp_spi_imu_set_cs(bool selected)
{
    if (selected) {
        DL_GPIO_clearPins(GPIOB, DL_GPIO_PIN_12);
    } else {
        DL_GPIO_setPins(GPIOB, DL_GPIO_PIN_12);
    }
}

static status_t bsp_spi_imu_exchange(uint8_t tx_data, uint8_t *rx_data)
{
    uint32_t timeout = BSP_SPI_IMU_TIMEOUT_LOOPS;

    while (DL_SPI_isTXFIFOFull(SPI1)) {
        if (timeout-- == 0U) {
            return STATUS_TIMEOUT;
        }
    }

    DL_SPI_transmitData8(SPI1, tx_data);
    timeout = BSP_SPI_IMU_TIMEOUT_LOOPS;
    while (DL_SPI_isRXFIFOEmpty(SPI1)) {
        if (timeout-- == 0U) {
            return STATUS_TIMEOUT;
        }
    }

    if (rx_data != NULL) {
        *rx_data = DL_SPI_receiveData8(SPI1);
    } else {
        (void) DL_SPI_receiveData8(SPI1);
    }
    return STATUS_OK;
}

void bsp_spi_imu_init(void)
{
    static const DL_SPI_Config spi_config = {
        .mode = DL_SPI_MODE_CONTROLLER,
        .frameFormat = DL_SPI_FRAME_FORMAT_MOTO4_POL0_PHA0,
        .parity = DL_SPI_PARITY_NONE,
        .dataSize = DL_SPI_DATA_SIZE_8,
        .bitOrder = DL_SPI_BIT_ORDER_MSB_FIRST,
        .chipSelectPin = DL_SPI_CHIP_SELECT_NONE,
    };
    static const DL_SPI_ClockConfig clock_config = {
        .clockSel = DL_SPI_CLOCK_BUSCLK,
        .divideRatio = DL_SPI_CLOCK_DIVIDE_RATIO_1,
    };

    DL_SPI_reset(SPI1);
    DL_SPI_enablePower(SPI1);
    delay_cycles(320U);

    DL_GPIO_initPeripheralInputFunction(IOMUX_PINCM24, IOMUX_PINCM24_PF_SPI1_POCI);
    DL_GPIO_initPeripheralOutputFunction(IOMUX_PINCM25, IOMUX_PINCM25_PF_SPI1_PICO);
    DL_GPIO_enableOutput(GPIOB, DL_GPIO_PIN_8);
    DL_GPIO_initPeripheralOutputFunction(IOMUX_PINCM26, IOMUX_PINCM26_PF_SPI1_SCLK);
    DL_GPIO_enableOutput(GPIOB, DL_GPIO_PIN_9);
    DL_GPIO_disableInterrupt(GPIO_BUTTONS_PORT, GPIO_BUTTONS_MODE_PIN);
    DL_GPIO_clearInterruptStatus(GPIO_BUTTONS_PORT, GPIO_BUTTONS_MODE_PIN);
    DL_GPIO_initDigitalOutput(IOMUX_PINCM29);
    DL_GPIO_enableOutput(GPIOB, DL_GPIO_PIN_12);
    bsp_spi_imu_set_cs(false);

    DL_SPI_setClockConfig(SPI1, (DL_SPI_ClockConfig *) &clock_config);
    DL_SPI_init(SPI1, (DL_SPI_Config *) &spi_config);
    DL_SPI_setBitRateSerialClockDivider(SPI1, BSP_SPI_IMU_CLOCK_DIVIDER);
    DL_SPI_setFIFOThreshold(SPI1, DL_SPI_RX_FIFO_LEVEL_ONE_FRAME,
        DL_SPI_TX_FIFO_LEVEL_ONE_FRAME);
    DL_SPI_enable(SPI1);
}

status_t bsp_spi_imu_write(uint8_t reg_addr, const uint8_t *data, uint16_t length)
{
    status_t status;

    if ((data == NULL) || (length == 0U)) {
        return STATUS_INVALID_ARG;
    }

    bsp_spi_imu_set_cs(true);
    status = bsp_spi_imu_exchange((uint8_t) (reg_addr & 0x7FU), NULL);
    for (uint16_t index = 0U; (status == STATUS_OK) && (index < length); index++) {
        status = bsp_spi_imu_exchange(data[index], NULL);
    }
    bsp_spi_imu_set_cs(false);
    return status;
}

status_t bsp_spi_imu_read(uint8_t reg_addr, uint8_t *data, uint16_t length)
{
    status_t status;

    if ((data == NULL) || (length == 0U)) {
        return STATUS_INVALID_ARG;
    }

    while (!DL_SPI_isRXFIFOEmpty(SPI1)) {
        (void) DL_SPI_receiveData8(SPI1);
    }

    bsp_spi_imu_set_cs(true);
    status = bsp_spi_imu_exchange((uint8_t) (reg_addr | 0x80U), NULL);
    for (uint16_t index = 0U; (status == STATUS_OK) && (index < length); index++) {
        status = bsp_spi_imu_exchange(0x00U, &data[index]);
    }
    bsp_spi_imu_set_cs(false);
    return status;
}
