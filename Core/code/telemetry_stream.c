#include "telemetry_stream.h"

#if TELEMETRY_STREAM_ENABLE

#include "dodo_BMI270.h"
#include <stdint.h>
#include <stdio.h>

/* 编码器计数器直接用 CMSIS 寄存器读，不依赖 main.c 里的 htim3/htim4 句柄。
   读取 CNT 是非破坏性的，不会影响控制环自己的计数与速度计算。 */
#define TELE_LEFT_CNT   ((uint16_t)TIM3->CNT)
#define TELE_RIGHT_CNT  ((uint16_t)TIM4->CNT)

static UART_HandleTypeDef *s_huart = NULL;
static uint32_t s_last_tick = 0;
static uint32_t s_last_report_tick = 0;
static uint16_t s_prev_left = 0;
static uint16_t s_prev_right = 0;
/* 上次成功启动发送以来的增量；暂存为 64 位以避免累计溢出。 */
static int64_t s_pending_left = 0;
static int64_t s_pending_right = 0;
static int s_left_sign = 1;
static int s_right_sign = -1;
static uint32_t s_seq = 0;
static uint32_t s_dropped = 0;
static char s_buf[128];

/* 16 位计数器差值，允许单次采样间跨过一次回绕 */
static int32_t tele_delta16(uint16_t current, uint16_t previous)
{
    int32_t delta = (int32_t)(uint16_t)(current - previous);

    if (delta > 32767) {
        delta -= 65536;
    }
    return delta;
}

void TelemetryStream_Init(UART_HandleTypeDef *huart, int left_sign, int right_sign)
{
    s_huart = huart;
    s_left_sign = (left_sign < 0) ? -1 : 1;
    s_right_sign = (right_sign < 0) ? -1 : 1;

    s_last_tick = HAL_GetTick();
    s_last_report_tick = s_last_tick;
    s_prev_left = TELE_LEFT_CNT;
    s_prev_right = TELE_RIGHT_CNT;
    s_pending_left = 0;
    s_pending_right = 0;

    s_seq = 0u;
    s_dropped = 0u;
}

void TelemetryStream_Poll(void)
{
    uint32_t now;
    uint32_t dt_ms;
    uint32_t sample_seq;
    uint16_t left_cnt;
    uint16_t right_cnt;
    int32_t left_delta;
    int32_t right_delta;
    int valid;
    int length;

    if (s_huart == NULL) {
        return;
    }

    now = HAL_GetTick();
    if ((uint32_t)(now - s_last_tick) < TELEMETRY_PERIOD_MS) {
        return;
    }

    /* 采样序号包含被跳过的帧，让接收端能检测缺口。 */
    sample_seq = s_seq++;
    s_last_tick = now;

    left_cnt = TELE_LEFT_CNT;
    right_cnt = TELE_RIGHT_CNT;
    left_delta = tele_delta16(left_cnt, s_prev_left) * s_left_sign;
    right_delta = tele_delta16(right_cnt, s_prev_right) * s_right_sign;
    s_prev_left = left_cnt;
    s_prev_right = right_cnt;
    s_pending_left += left_delta;
    s_pending_right += right_delta;
    dt_ms = (uint32_t)(now - s_last_report_tick);

    /* 20 Hz 读一次陀螺仪；读失败时驱动会把三轴清零，所以必须靠 valid 判断 */
    dodo_BMI270_get_data();
    valid = (BMI270_last_rslt == 0) ? 1 : 0;

    /* 上一帧还没发完就丢本帧，避免覆盖正在发送的缓冲区 */
    if (HAL_UART_GetState(s_huart) != HAL_UART_STATE_READY) {
        ++s_dropped;
        return;
    }

    /* 原协议增量是 32 位；极长时间无法发送时放弃不可表示的数据。
       递增的序号和丢帧计数使接收端可发现缺口。 */
    if (s_pending_left > INT32_MAX || s_pending_left < INT32_MIN ||
        s_pending_right > INT32_MAX || s_pending_right < INT32_MIN) {
        s_pending_left = 0;
        s_pending_right = 0;
        s_last_report_tick = now;
        ++s_dropped;
        return;
    }

    /* 全部整数，避免 microlib 不支持 %f 的问题；速度由电脑端用 delta/dt 折算 */
    length = snprintf(s_buf, sizeof(s_buf),
                      "%lu,%lu,%lu,%ld,%ld,%d,%d,%d,%d\r\n",
                      (unsigned long)sample_seq,
                      (unsigned long)now,
                      (unsigned long)dt_ms,
                      (long)s_pending_left,
                      (long)s_pending_right,
                      (int)BMI270_gyro_x,
                      (int)BMI270_gyro_y,
                      (int)BMI270_gyro_z,
                      valid);

    if ((length > 0) && ((unsigned int)length < sizeof(s_buf))) {
        if (HAL_UART_Transmit_IT(s_huart, (uint8_t *)s_buf, (uint16_t)length) == HAL_OK) {
            s_pending_left = 0;
            s_pending_right = 0;
            s_last_report_tick = now;
            return;
        }
    }
    ++s_dropped;
}

uint32_t TelemetryStream_GetDropped(void)
{
    return s_dropped;
}

#else /* !TELEMETRY_STREAM_ENABLE —— 关闭时退化成空实现 */

void TelemetryStream_Init(UART_HandleTypeDef *huart, int left_sign, int right_sign)
{
    (void)huart;
    (void)left_sign;
    (void)right_sign;
}

void TelemetryStream_Poll(void) {}

uint32_t TelemetryStream_GetDropped(void) { return 0u; }

#endif /* TELEMETRY_STREAM_ENABLE */
