#include "bmi270_port.h"

static SPI_HandleTypeDef *bmi2_hspi = NULL;

// CS 引脚控制宏（BSRR 直接置位/复位）
#define BMI270_CS_LOW()     (BMI270_CS_GPIO_Port->BSRR = ((uint32_t)BMI270_CS_Pin << 16U))
#define BMI270_CS_HIGH()    (BMI270_CS_GPIO_Port->BSRR = (uint32_t)BMI270_CS_Pin)
#define BMI270_SPI_BUF_SIZE 260U
#define BMI270_SPI_TIMEOUT_MS 5U

/**
 * @brief SPI 读函数 (同步轮询，避免 DMA 中断与大缓存开销)
 */
BMI2_INTF_RETURN_TYPE bmi2_spi_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr)
{
    (void)intf_ptr;
    if (bmi2_hspi == NULL || reg_data == NULL) {
        return BMI2_E_NULL_PTR;
    }

    static uint8_t tx_buf[BMI270_SPI_BUF_SIZE];
    static uint8_t rx_buf[BMI270_SPI_BUF_SIZE];
    if ((len + 1U) > sizeof(tx_buf)) {
        return BMI2_E_COM_FAIL;
    }

    tx_buf[0] = (uint8_t)(reg_addr | 0x80U);
    for (uint32_t i = 0U; i < len; ++i) {
        tx_buf[i + 1U] = 0U;
    }

    BMI270_CS_LOW();
    if (HAL_SPI_TransmitReceive(bmi2_hspi, tx_buf, rx_buf, (uint16_t)(len + 1U), BMI270_SPI_TIMEOUT_MS) != HAL_OK) {
        BMI270_CS_HIGH();
        return BMI2_E_COM_FAIL;
    }
    BMI270_CS_HIGH();

    for (uint32_t i = 0U; i < len; ++i) {
        reg_data[i] = rx_buf[i + 1U];
    }

    return BMI2_OK;
}

/**
 * @brief SPI 写函数 (同步轮询)
 */
BMI2_INTF_RETURN_TYPE bmi2_spi_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len, void *intf_ptr)
{
    (void)intf_ptr;
    if (bmi2_hspi == NULL || reg_data == NULL) {
        return BMI2_E_NULL_PTR;
    }

    static uint8_t tx_buf[BMI270_SPI_BUF_SIZE];
    if ((len + 1U) > sizeof(tx_buf)) {
        return BMI2_E_COM_FAIL;
    }

    tx_buf[0] = (uint8_t)(reg_addr & 0x7FU);
    for (uint32_t i = 0U; i < len; ++i) {
        tx_buf[i + 1U] = reg_data[i];
    }

    BMI270_CS_LOW();
    if (HAL_SPI_Transmit(bmi2_hspi, tx_buf, (uint16_t)(len + 1U), BMI270_SPI_TIMEOUT_MS) != HAL_OK) {
        BMI270_CS_HIGH();
        return BMI2_E_COM_FAIL;
    }
    BMI270_CS_HIGH();

    return BMI2_OK;
}

/**
 * @brief 精确微秒延时函数 (基于 DWT)
 * @param period : 延时周期，单位微秒
 * @param intf_ptr : 接口指针 (未使用)
 * @return 无
 */
void bmi2_delay_us(uint32_t period, void *intf_ptr)
{
    (void)intf_ptr;
    uint32_t start = DWT->CYCCNT;
    uint32_t ticks = period * (SystemCoreClock / 1000000U);
    while ((DWT->CYCCNT - start) < ticks);
}

/**
 * @brief 初始化 DWT (用于精确延时)
 */
static void dwt_init(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk; // 使能 DWT
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk; // 使能周期计数器
    DWT->CYCCNT = 0; // 计数器清零
}

/**
 * @brief 初始化 BMI270 传感器 (SPI)
 */
int8_t bmi270_sensor_init_spi(struct bmi2_dev *dev, SPI_HandleTypeDef *hspi)
{
    
    if (hspi == NULL || dev == NULL) {
        return BMI2_E_NULL_PTR;
    }
    
    dwt_init(); // 初始化 DWT
    
    bmi2_hspi = hspi;
    
    dev->intf = BMI2_SPI_INTF;
    dev->read = bmi2_spi_read;
    dev->write = bmi2_spi_write;
    dev->delay_us = bmi2_delay_us;
    dev->intf_ptr = NULL; 
    dev->read_write_len = 256; 
    dev->config_file_ptr = NULL; 
    
    // 根据BMI270数据手册，SPI接口在初始化时需要一个dummy read来进入SPI模式
    uint8_t dummy;
    bmi2_spi_read(BMI2_CHIP_ID_ADDR, &dummy, 1, NULL);
    bmi2_delay_us(1000, NULL); // 延时 1ms
    
    // 初始化 BMI270
    int8_t rslt = bmi270_init(dev);
    // printf("rslt:%d\r\n",rslt);
    return rslt;
}

/**
 * @brief 配置加速度计和陀螺仪的参数
 */
int8_t bmi270_set_config(struct bmi2_dev *dev)
{
    int8_t rslt;
    struct bmi2_sens_config config;

    // 1. 配置加速度计
    config.type = BMI2_ACCEL;
    rslt = bmi2_get_sensor_config(&config, 1, dev);
    if (rslt != BMI2_OK) return rslt;

    config.cfg.acc.odr = BMI2_ACC_ODR_1600HZ;       // 输出数据速率
    config.cfg.acc.range = BMI2_ACC_RANGE_8G;     // 量程 G (2G, 4G, 8G, 16G)
    config.cfg.acc.bwp = BMI2_ACC_NORMAL_AVG4;    // 滤波器
    config.cfg.acc.filter_perf = BMI2_PERF_OPT_MODE; // 性能模式

    rslt = bmi2_set_sensor_config(&config, 1, dev);
    if (rslt != BMI2_OK) return rslt;

    // 2. 配置陀螺仪
    config.type = BMI2_GYRO;
    rslt = bmi2_get_sensor_config(&config, 1, dev);
    if (rslt != BMI2_OK) return rslt;

    config.cfg.gyr.odr = BMI2_GYR_ODR_1600HZ;       // 输出数据速率
    config.cfg.gyr.range = BMI2_GYR_RANGE_2000; // 量程 dps (125, 250, 500, 1000, 2000)
    config.cfg.gyr.bwp = BMI2_GYR_NORMAL_MODE;    // 带宽参数
    config.cfg.gyr.filter_perf = BMI2_PERF_OPT_MODE; // 性能模式
    config.cfg.gyr.noise_perf = BMI2_GYR_NOISE_PERF_MODE_MASK; // 噪声性能

    rslt = bmi2_set_sensor_config(&config, 1, dev);
    if (rslt != BMI2_OK) return rslt;
    
    // 3. 使能加速度计和陀螺仪
    uint8_t sens_list[] = { BMI2_ACCEL, BMI2_GYRO };
    rslt = bmi2_sensor_enable(sens_list, 2, dev);
    
    return rslt;
}

/**
 * @brief 获取加速度计和陀螺仪的传感器数据
 * @details 该函数一次性获取所有传感器数据。
 * @param[in] dev : 已初始化的 bmi2_dev 结构体指针
 * @param[out] acc : 用于存储加速度数据的结构体指针
 * @param[out] gyr : 用于存储陀螺仪数据的结构体指针
 * @return API 执行结果，BMI2_OK 表示成功
 */
int8_t bmi270_get_accel_gyro(struct bmi2_dev *dev, struct bmi2_sens_data *acc, struct bmi2_sens_data *gyr)
{
    struct bmi2_sens_data sensor_data;
    int8_t rslt = bmi2_get_sensor_data(&sensor_data, dev);
    if (rslt == BMI2_OK)
    {
        *acc = sensor_data;
        *gyr = sensor_data;
    }
    return rslt;
}

int8_t bmi270_get_gyro_z_raw(SPI_HandleTypeDef *hspi, int16_t *gyro_z)
{
    uint8_t tx[4] = { (uint8_t)((BMI2_GYR_X_LSB_ADDR + 4U) | 0x80U), 0U, 0U, 0U };
    uint8_t rx[4] = { 0U, 0U, 0U, 0U };

    if (hspi == NULL || gyro_z == NULL) {
        return BMI2_E_NULL_PTR;
    }

    BMI270_CS_LOW();
    if (HAL_SPI_TransmitReceive(hspi, tx, rx, 4U, 2U) != HAL_OK) {
        BMI270_CS_HIGH();
        return BMI2_E_COM_FAIL;
    }
    BMI270_CS_HIGH();

    *gyro_z = (int16_t)(((uint16_t)rx[3] << 8U) | (uint16_t)rx[2]);
    return BMI2_OK;
}