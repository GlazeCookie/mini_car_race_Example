/*
 * pid_helper_app.h —— PID Helper 调参协议适配层（给 mini_car_race 小车用）
 *
 * 作用：把 pid_helper_protocol 的协议栈接到本车的 USART3 / 双轮速度环上，
 *       让上位机「PID 串口调参助手」能连上、看曲线、跑自动辨识和阶跃测试。
 *
 * 集成方式（main.c 里只加了 6 处，全部有 PID_HELPER_ENABLE 开关保护）：
 *   1) #include "pid_helper_app.h"
 *   2) SpeedPid_Update 里把 SPEED_KP/KI/KD 换成 PidHelper_Kp()/Ki()/Kd()
 *   3) Line_FollowStep 里加一段「上位机接管」分支
 *   4) WaitForRunKey 里调用 PidHelper_Poll()，上位机接管时不必等发车键
 *   5) main() 里 PidHelper_Init(...)
 *   6) 主循环调 PidHelper_Poll()
 *
 * 行为约定：
 *   - 只收 APPMODE / PID 时不动车，收到 SP 或 OUT 才进入「上位机接管」；
 *   - 接管后两轮共用同一个目标速度（速度环模式）；
 *   - MODE=open 时 OUT 直接当 PWM 下发；
 *   - 目标速度限制在 [0, max_target]，PWM 限制在 [0, max_pwm]，倒车/负值一律按 0 处理
 *     （本车 PWM 单向，不支持反转）。
 */

#ifndef PID_HELPER_APP_H
#define PID_HELPER_APP_H

#include "main.h"

/*
 * 1: 用 USART3 跑 PID 调参协议。此时原来 100 ms 一行的 CSV 文本调试输出会自动关闭，
 *    因为两种数据混在同一个串口上会把协议帧打乱。
 * 0: 完全恢复原样（CSV 调试输出照旧），调参协议不启用。
 */
#ifndef PID_HELPER_ENABLE
#define PID_HELPER_ENABLE 1
#endif

/* huart 传 &huart3；max_target/max_pwm 传 SPEED_MAX_TARGET / SPEED_MAX_PWM */
void PidHelper_Init(UART_HandleTypeDef *huart,
                    float kp,
                    float ki,
                    float kd,
                    float max_target,
                    float max_pwm);

/* 主循环（含等待发车键的循环）里反复调用，间隔约 1 ms */
void PidHelper_Poll(void);

/* 由循迹/控制代码每条控制周期上报一次，供上位机画曲线 */
void PidHelper_Report(float actual, float target, float input);

/* 1 = 上位机正在接管，循迹逻辑应跳过 */
uint8_t PidHelper_IsTuning(void);

/* 1 = 上位机要求开环，TIM1->CCR 应直接取 PidHelper_OpenOutput() */
uint8_t PidHelper_IsOpenMode(void);

/* 上位机下发的目标速度（计数/秒，两轮共用） */
float PidHelper_TargetSpeed(void);

/* 上位机下发的开环 PWM */
float PidHelper_OpenOutput(void);

/* 当前生效的 PID 增益（未收到下发时等于编译期的 SPEED_KP/KI/KD） */
float PidHelper_Kp(void);
float PidHelper_Ki(void);
float PidHelper_Kd(void);

#endif /* PID_HELPER_APP_H */
