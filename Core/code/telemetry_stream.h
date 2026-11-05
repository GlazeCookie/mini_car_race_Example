/*
 * telemetry_stream.h —— 「双轮编码器 + BMI270 陀螺仪」数据流（USART3）
 *
 * 与 PID Helper 调参协议互斥：两者都占用 USART3，
 * 二进制帧和文本混在一个串口里会把双方的解析都搞乱，所以编译期二选一。
 *
 * 打开本模块（TELEMETRY_STREAM_ENABLE=1）后：
 *   - 自动关闭 PID Helper 协议（PID_HELPER_ENABLE 被强制为 0）
 *   - 自动关闭 ENCODER_TEST_MODE==2 的 CSV 调试输出
 *   - 每 50 ms（20 Hz）输出一行 CSV，见下方字段说明
 *
 * 输出格式（每行，\r\n 结尾，全部为十进制整数）：
 *   seq, t_ms, dt_ms, l_delta, r_delta, gx, gy, gz, valid
 *
 *   seq      采样序号，从 0 开始；串口忙或发送失败时也递增，接收端可识别缺口
 *   t_ms     采样时刻，HAL_GetTick()，毫秒（STM32 上电起算，约 49.7 天回绕）
 *   dt_ms    本次与上次成功启动 UART 发送的采样间隔（毫秒），不是固定 50
 *   l_delta  两次成功启动发送之间的左轮累计增量（已乘 LEFT_ENCODER_SIGN）
 *   r_delta  两次成功启动发送之间的右轮累计增量（已乘 RIGHT_ENCODER_SIGN）
 *   gx/gy/gz 陀螺仪三轴原始值（int16，±2000 dps 量程）
 *   valid    1 = 本次 BMI270 读取成功；0 = 读取失败或未初始化（此时三轴为 0，不代表静止）
 * 若无线链路丢失已经发送的帧，后续帧不会补上这些增量，接收端应检查序号缺口。
 *
 * 电脑端换算：
 *   轮速(计数/秒) = delta * 1000 / dt_ms
 *   米/秒        = 轮速 / 19841          （实测左轮，1 米约 19841 计数）
 *   角速度(°/s)  = gx / 16.4             （±2000 dps 量程下的换算系数）
 *   加速度(g)    = ax / 4096             （±8 g 量程，如需读取加速度需扩展字段）
 */

#ifndef TELEMETRY_STREAM_H
#define TELEMETRY_STREAM_H

#include "main.h"

/* 1: 输出编码器 + 陀螺仪数据流（此时 PID Helper 调参不可用）
   0: 关闭数据流，交回给 PID Helper 调参 / 循迹监听 */
#ifndef TELEMETRY_STREAM_ENABLE
#define TELEMETRY_STREAM_ENABLE 0
#endif

#if TELEMETRY_STREAM_ENABLE
#undef PID_HELPER_ENABLE
#define PID_HELPER_ENABLE 0
#endif

#define TELEMETRY_PERIOD_MS 50u /* 50 ms 一帧 = 20 Hz */

/*
 * huart           传 &huart3
 * left_sign       传 LEFT_ENCODER_SIGN
 * right_sign      传 RIGHT_ENCODER_SIGN
 */
void TelemetryStream_Init(UART_HandleTypeDef *huart, int left_sign, int right_sign);

/* 主循环里反复调用，内部按 TELEMETRY_PERIOD_MS 自己节流 */
void TelemetryStream_Poll(void);

/* 因串口忙而丢掉的帧数，用于诊断链路带宽 */
uint32_t TelemetryStream_GetDropped(void);

#endif /* TELEMETRY_STREAM_H */
