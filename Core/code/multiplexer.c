#include "multiplexer.h"
#include "bmi270_port.h"

/**
 * @brief 精确微秒延时函数 (基于 DWT)
 * @param period : 延时周期，单位微秒
 * @param intf_ptr : 接口指针 (未使用)
 * @return 无
 */
void multiplexer_delay_us(uint32_t period, void *intf_ptr)
{
    (void)intf_ptr;
    if ((DWT->CTRL & DWT_CTRL_CYCCNTENA_Msk) == 0U) {
        CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
        DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    }
    uint32_t start = DWT->CYCCNT;
    uint32_t ticks = period * (SystemCoreClock / 1000000U);
    while ((DWT->CYCCNT - start) < ticks);
}

int16_t multiplexer_set_channel(int16_t channel){
    if(channel<0||channel>15) return -1; // 通道号错误，返回-1
    MULTIPLEXER_0_PORT->BSRR = ((channel & 1) != 0) ? (uint32_t)MULTIPLEXER_0_PIN : ((uint32_t)MULTIPLEXER_0_PIN << 16U);
    MULTIPLEXER_1_PORT->BSRR = (((channel >> 1) & 1) != 0) ? (uint32_t)MULTIPLEXER_1_PIN : ((uint32_t)MULTIPLEXER_1_PIN << 16U);
    MULTIPLEXER_2_PORT->BSRR = (((channel >> 2) & 1) != 0) ? (uint32_t)MULTIPLEXER_2_PIN : ((uint32_t)MULTIPLEXER_2_PIN << 16U);
    MULTIPLEXER_3_PORT->BSRR = (((channel >> 3) & 1) != 0) ? (uint32_t)MULTIPLEXER_3_PIN : ((uint32_t)MULTIPLEXER_3_PIN << 16U);
    return 0;
}

/**
 * @brief 读取多路复用器的值
 * @param[out] value 指向存储读取值的变量的指针
 */
void MUX_get_value(uint16_t *value){
    uint16_t result = 0U;
    for(int16_t i=0;i<MULTIPLEXER_CHANNEL_NUM;i++){
        multiplexer_set_channel(i);
        multiplexer_delay_us(1U,NULL);
        if((MULTIPLEXER_READ_PORT->IDR & MULTIPLEXER_READ_PIN) != 0U){
            result |= (uint16_t)(1U << (MULTIPLEXER_CHANNEL_NUM - i - 1));
        }
    }
    *value = result;
}