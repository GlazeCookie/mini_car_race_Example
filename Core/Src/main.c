/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2025 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 * 本文件内含BMI270陀螺仪的使用示例
 * 本文件内含无刷电桥的启动初始化的使用示例
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "dodo_BMI270.h" //陀螺仪驱动
#include "multiplexer.h" //多路复用器驱动，用于读取光电管读数
#include "stdio.h"
#include <math.h>
#include "pid_helper_app.h" //PID Helper 上位机调参协议（USART3）
#include "telemetry_stream.h" //双轮编码器 + 陀螺仪数据流（USART3，与调参互斥）
#include "track_nav.h" // 赛道全遍历分段状态机（12路多簇 + BMI270 + 编码器）
/* USER CODE END Includes */
/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */
/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define IF_start_Brushless_motor 0 //是否启动无刷电机，1为启动，0为不启动
#define ENCODER_TEST_MODE 5 /* 0: 基础循迹；1: 编码器；2: 基础遥测；3: 赛道+PID Helper；4: 赛道比赛；5: 赛道非阻塞诊断；6: 纯基础循迹 */
#define ENCODER_TEST_PERIOD_MS 200U
#define LINE_TELEMETRY_PERIOD_MS 100U
#if ENCODER_TEST_MODE < 0 || ENCODER_TEST_MODE > 6
#error "ENCODER_TEST_MODE must be 0 through 6"
#endif
#define NAVIGATION_MODE (ENCODER_TEST_MODE >= 3 && ENCODER_TEST_MODE <= 5)
#define PID_HELPER_IN_MODE (ENCODER_TEST_MODE != 4 && ENCODER_TEST_MODE != 5 && ENCODER_TEST_MODE != 6)
#define BASIC_LINE_MODE (ENCODER_TEST_MODE == 0 || ENCODER_TEST_MODE == 2 || ENCODER_TEST_MODE == 6)
#define RUN_KEY_DEBOUNCE_MS 20U
#define RUN_START_DELAY_MS  1000U /* 按发车键后等待 1 秒发车 */
#define RUN_KEY_SETTLE_MS   200U  /* 导航模式按键松手稳定延时（与 800 ms 陀螺仪标定合计 1 s） */
/* 循迹和速度参数均需根据电机、编码器和赛道实车标定。 */
#define LINE_ACTIVE_LEVEL  1U /* 白线输出高电平 */
#define LINE_MAX_ACTIVE    12U
#define LINE_PERIOD_US     2000U /* 控制周期 2 ms (500 Hz)，基于 DWT->CYCCNT 精确触发 */
#define LINE_KP            1100.0f /* 基础循迹位置环比例系数 */
#define LINE_KP2           110.0f  /* 基础循迹大偏差二次项 */
#define LINE_KD            1.30f   /* 基础循迹微分项（按 dt_s 归一化） */
#define LINE_SEARCH_SPEED  24000.0f /* 基础循迹丢线寻线外轮速度 */
#define LINE_SEARCH_REV_SPEED 8000.0f /* 基础循迹丢线寻线内轮反转速度 */
#define LINE_SEARCH_SWEEP_MS 1500U   /* 坚持按上次方向寻线 1.5 s 后才反向兜底 */
#define SPEED_BASE_TARGET  36000.0f /* 基础循迹直线目标轮速 */
#define SPEED_MAX_TARGET   70000.0f /* 基础循迹正向最高目标轮速 */
#define SPEED_MAX_REV_TARGET 10000.0f /* 基础循迹内轮温和反转上限，防止急弯甩过掉头 */
#define SPEED_FF_OFFSET    200.0f  /* 死区补偿 PWM */
#define SPEED_FF_GAIN      0.010f  /* PWM / (计数每秒) */
#define SPEED_MAX_PWM      7199.0f /* 满量程 PWM（对应 TIM1->ARR = 7199） */
#define SPEED_KP           0.055f
#define SPEED_KI           0.70f
#define SPEED_KD           0.0f
#define SPEED_INTEGRAL_LIMIT 2800.0f /* 积分项在 PWM 中的最大绝对贡献 */
#define LEFT_ENCODER_SIGN  1
#define RIGHT_ENCODER_SIGN (-1)
/* USER CODE END PD */
/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */
/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;
DMA_HandleTypeDef hdma_spi1_rx;
DMA_HandleTypeDef hdma_spi1_tx;
DMA_HandleTypeDef hdma_tim1_up;
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
//负压启动相关宏
//负压启动流程：900us脉宽50Hzpwm信号输出2秒后，改为持续2000us脉宽50Hzpwm信号输出
#define ESC_PULSE_SLOTS       200U       /* 20 ms at the 10 kHz TIM1 update rate */
#define ESC_START_PULSE_SLOTS 9U         /* 0.9 ms */
#define ESC_RUN_PULSE_SLOTS   20U        /* 2.0 ms */
#define ESC_START_PERIODS     100U       /* 100 x 20 ms = 2 s */

static uint32_t esc_dma_table[ESC_PULSE_SLOTS];
static uint32_t esc_high_slots;    /* high time currently stored in the table */
static volatile uint32_t esc_period_count;

/* Fill a whole 20 ms frame.  Only used once, before the DMA is started. */
static void ESC_BuildDmaTable(uint32_t high_slots) {
  uint32_t i;

  for (i = 0U; i < ESC_PULSE_SLOTS; ++i) {
    esc_dma_table[i] = (i < high_slots) ? GPIO_PIN_11 : ((uint32_t)GPIO_PIN_11 << 16U);
  }
  esc_high_slots = high_slots;
}

/* Rewrite only the slots whose level really changes, in increasing index
   order: the circular DMA consumes the table in that same order, and slot 9
   is not transferred before ~0.9 ms after this interrupt, so the frame that
   is already running is never disturbed. */
static void ESC_SetPulseWidth(uint32_t high_slots) {
  uint32_t first;
  uint32_t last;
  uint32_t i;

  if (high_slots == esc_high_slots) {
    return;
  }

  first = (high_slots < esc_high_slots) ? high_slots : esc_high_slots;
  last  = (high_slots > esc_high_slots) ? high_slots : esc_high_slots;

  for (i = first; i < last; ++i) {
    esc_dma_table[i] = (i < high_slots) ? GPIO_PIN_11 : ((uint32_t)GPIO_PIN_11 << 16U);
  }
  esc_high_slots = high_slots;
}
static void ESC_fullPulseWidth(void){
  uint32_t i;
  for (i = 0U; i < ESC_PULSE_SLOTS; ++i) {
    esc_dma_table[i] = GPIO_PIN_11;
  }
  esc_high_slots = ESC_PULSE_SLOTS;
}
/* One interrupt per 20 ms frame: DMA transfer complete in circular mode.
   The 50 Hz waveform itself is produced by TIM1 + DMA in hardware; this
   handler only counts frames and switches 0.9 ms -> 2.0 ms after 2 s. */
static void ESC_DmaTransferComplete(DMA_HandleTypeDef *hdma) {
  if (esc_period_count < ESC_START_PERIODS) {
    ++esc_period_count;
    if (esc_period_count == ESC_START_PERIODS) {
      /* The frame that has just been completed was the last 0.9 ms frame:
         100 x 20 ms = 2 s have elapsed, 2.0 ms is now held forever. */
      if (!IF_start_Brushless_motor)
        ESC_SetPulseWidth(10U);
      else
        ESC_SetPulseWidth(ESC_RUN_PULSE_SLOTS);
      /* Nothing left to time, so stop the 50 Hz interrupt. */
      __HAL_DMA_DISABLE_IT(hdma, DMA_IT_TC);
    }
  }
}
/* USER CODE END PV */
/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_TIM4_Init(void);
/* USER CODE BEGIN PFP */
/* USER CODE END PFP */
/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static void DWT_Init(void) {
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
  DWT->CYCCNT = 0U;
}

int fputc(int ch, FILE *f) {
  (void)f;
  HAL_UART_Transmit(&huart3, (uint8_t *)&ch, 1, 0xffff);
  return ch;
}
typedef struct {
  float integral;
  float previous_error;
  uint8_t has_previous_error;
} SpeedPid;

static SpeedPid left_speed_pid;
static SpeedPid right_speed_pid;
static uint16_t previous_left_count;
static uint16_t previous_right_count;
static int8_t left_motor_dir = 1;
static int8_t right_motor_dir = 1;
static uint16_t left_last_pwm = 0U;
static uint16_t right_last_pwm = 0U;

static uint16_t Motor_UpdateAxis(int32_t signed_pwm, GPIO_TypeDef *dir_port, uint16_t dir_pin,
                                 int8_t *cur_dir, uint16_t *last_pwm) {
  int32_t limit = (int32_t)SPEED_MAX_PWM;
  if (signed_pwm > limit) signed_pwm = limit;
  if (signed_pwm < -limit) signed_pwm = -limit;

  if (signed_pwm == 0) {
    *last_pwm = 0U;
    return 0U;
  }

  int8_t want_dir = (signed_pwm > 0) ? 1 : -1;
  uint16_t mag = (uint16_t)((signed_pwm > 0) ? signed_pwm : -signed_pwm);

  if (want_dir != *cur_dir) {
    /* 换向时给 1 个控制周期的零 PWM 死区，防止反接电流冲击拉垮供电。 */
    if (*last_pwm != 0U) {
      *last_pwm = 0U;
      return 0U;
    }
    dir_port->BSRR = (want_dir > 0) ? ((uint32_t)dir_pin << 16U) : (uint32_t)dir_pin;
    *cur_dir = want_dir;
  }

  *last_pwm = mag;
  return mag;
}

static void Motor_SetOutput(int32_t left_pwm, int32_t right_pwm) {
  TIM1->CCR1 = Motor_UpdateAxis(left_pwm, L_DIR_GPIO_Port, L_DIR_Pin,
                                &left_motor_dir, &left_last_pwm);
  TIM1->CCR2 = Motor_UpdateAxis(right_pwm, R_DIR_GPIO_Port, R_DIR_Pin,
                                &right_motor_dir, &right_last_pwm);
}

static void Motor_StopAll(void) {
  TIM1->CCR1 = 0U;
  TIM1->CCR2 = 0U;
  left_last_pwm = 0U;
  right_last_pwm = 0U;
  L_DIR_GPIO_Port->BSRR = (uint32_t)L_DIR_Pin << 16U;
  R_DIR_GPIO_Port->BSRR = (uint32_t)R_DIR_Pin << 16U;
  left_motor_dir = 1;
  right_motor_dir = 1;
}

#if NAVIGATION_MODE
static float previous_nav_left_target;
static float previous_nav_right_target;
#if ENCODER_TEST_MODE == 5 && !TELEMETRY_STREAM_ENABLE
#define NAV_DIAG_QUEUE_SIZE 16U
typedef struct {
  uint32_t tick, dt_ms;
  int32_t seg_mm, yaw10, left_target, right_target, left_speed, right_speed;
  uint16_t mask, left_pwm, right_pwm;
  uint8_t state, fault, fault_state, count, searching;
  int8_t search_dir;
} NavDiagSample;
static NavDiagSample nav_diag_queue[NAV_DIAG_QUEUE_SIZE];
static uint8_t nav_diag_head, nav_diag_tail, nav_diag_have_sample;
static uint32_t nav_diag_last_tick, nav_diag_dropped;
static uint16_t nav_diag_last_mask;
static uint8_t nav_diag_last_state, nav_diag_last_fault, nav_diag_last_search;
static int8_t nav_diag_last_dir;
static int32_t nav_diag_left_speed, nav_diag_right_speed;
static uint32_t nav_diag_dt_ms;
static char nav_diag_buffer[224];

/* 记录状态和光电管变化，串口中断忙时只排队，不等待发送。 */
static void NavDiag_Poll(void) {
  uint32_t now = HAL_GetTick();
  uint8_t state = (uint8_t)TrackNav_GetState();
  uint8_t fault = (uint8_t)TrackNav_GetFault();
  uint16_t mask = TrackNav_GetLastLineMask();
  uint8_t searching = TrackNav_IsSearching();
  int8_t dir = TrackNav_GetSearchDirection();
  uint8_t important = !nav_diag_have_sample || state != nav_diag_last_state || fault != nav_diag_last_fault;
  uint8_t changed = important || mask != nav_diag_last_mask || searching != nav_diag_last_search || dir != nav_diag_last_dir;
  if (important || (changed && (uint32_t)(now - nav_diag_last_tick) >= 10U) ||
      (uint32_t)(now - nav_diag_last_tick) >= 100U) {
    uint8_t next = (uint8_t)((nav_diag_head + 1U) % NAV_DIAG_QUEUE_SIZE);
    NavDiagSample *sample;
    if (next == nav_diag_tail) {
      nav_diag_tail = (uint8_t)((nav_diag_tail + 1U) % NAV_DIAG_QUEUE_SIZE);
      ++nav_diag_dropped;
    }
    sample = &nav_diag_queue[nav_diag_head];
    sample->tick = now;
    sample->dt_ms = nav_diag_dt_ms;
    sample->state = state;
    sample->fault = fault;
    sample->fault_state = (uint8_t)TrackNav_GetFaultState();
    sample->seg_mm = (int32_t)TrackNav_GetSegMm();
    sample->yaw10 = (int32_t)(TrackNav_GetYawDeg() * 10.0f);
    sample->mask = mask;
    sample->count = TrackNav_GetLastLineCount();
    sample->searching = searching;
    sample->search_dir = dir;
    sample->left_target = (int32_t)previous_nav_left_target;
    sample->right_target = (int32_t)previous_nav_right_target;
    sample->left_speed = nav_diag_left_speed;
    sample->right_speed = nav_diag_right_speed;
    sample->left_pwm = (uint16_t)TIM1->CCR1;
    sample->right_pwm = (uint16_t)TIM1->CCR2;
    nav_diag_head = next;
    nav_diag_last_tick = now;
    nav_diag_last_state = state;
    nav_diag_last_fault = fault;
    nav_diag_last_mask = mask;
    nav_diag_last_search = searching;
    nav_diag_last_dir = dir;
    nav_diag_have_sample = 1U;
  }
  if (nav_diag_tail != nav_diag_head && huart3.gState == HAL_UART_STATE_READY) {
    NavDiagSample *sample = &nav_diag_queue[nav_diag_tail];
    int length = snprintf(nav_diag_buffer, sizeof(nav_diag_buffer),
        "NAV,t=%lu,st=%u,f=%u,fs=%u,mm=%ld,yaw10=%ld,mask=%03X,n=%u,s=%u,d=%d,lt=%ld,rt=%ld,lv=%ld,rv=%ld,lp=%u,rp=%u,dt=%lu,drop=%lu,wd=0\r\n",
        (unsigned long)sample->tick, (unsigned int)sample->state, (unsigned int)sample->fault,
        (unsigned int)sample->fault_state, (long)sample->seg_mm, (long)sample->yaw10,
        (unsigned int)sample->mask, (unsigned int)sample->count, (unsigned int)sample->searching,
        (int)sample->search_dir, (long)sample->left_target, (long)sample->right_target,
        (long)sample->left_speed, (long)sample->right_speed, (unsigned int)sample->left_pwm,
        (unsigned int)sample->right_pwm, (unsigned long)sample->dt_ms, (unsigned long)nav_diag_dropped);
    if (length > 0 && (unsigned int)length < sizeof(nav_diag_buffer) &&
        HAL_UART_Transmit_IT(&huart3, (uint8_t *)nav_diag_buffer, (uint16_t)length) == HAL_OK)
      nav_diag_tail = (uint8_t)((nav_diag_tail + 1U) % NAV_DIAG_QUEUE_SIZE);
  }
}
#endif

#endif
#if BASIC_LINE_MODE
static int8_t last_line_direction; /* -1 左，1 右，0 未知 */
static int8_t line_search_direction;
static uint8_t line_lost;
static uint32_t line_lost_tick;
static float previous_line_error;
static uint8_t has_previous_line_error;

#endif
typedef enum {
  LINE_LOST,
  LINE_FOUND,
  LINE_TOO_WIDE
} LineStatus;

#if ENCODER_TEST_MODE == 2
typedef struct {
  uint32_t sample_tick;
  uint16_t mux_value;
  uint8_t active_count;
  char state; /* F=循迹, S=寻线, X=丢线停车, W=检测过宽 */
  int32_t error_x100;
  int32_t left_speed;
  int32_t right_speed;
  int32_t left_target;
  int32_t right_target;
  uint32_t left_pwm;
  uint32_t right_pwm;
} LineTelemetry;

static LineTelemetry line_telemetry;
#if !PID_HELPER_ENABLE && !TELEMETRY_STREAM_ENABLE
/* 发送完成前 HAL 会继续读取此缓冲区，因此必须保持有效且不能覆盖。 */
static char line_telemetry_buffer[160];

static void Line_SendTelemetry(void) {
  int length;
  if (HAL_UART_GetState(&huart3) != HAL_UART_STATE_READY) return;

  length = snprintf(line_telemetry_buffer, sizeof(line_telemetry_buffer),
                    "%lu,%c,%03X,%u,%ld,%ld,%ld,%ld,%ld,%lu,%lu\r\n",
                    (unsigned long)line_telemetry.sample_tick,
                    line_telemetry.state, (unsigned int)line_telemetry.mux_value,
                    (unsigned int)line_telemetry.active_count,
                    (long)line_telemetry.error_x100,
                    (long)line_telemetry.left_speed, (long)line_telemetry.right_speed,
                    (long)line_telemetry.left_target, (long)line_telemetry.right_target,
                    (unsigned long)line_telemetry.left_pwm, (unsigned long)line_telemetry.right_pwm);
  if (length > 0 && (unsigned int)length < sizeof(line_telemetry_buffer))
    (void)HAL_UART_Transmit_IT(&huart3, (uint8_t *)line_telemetry_buffer, (uint16_t)length);
}
#endif /* !PID_HELPER_ENABLE && !TELEMETRY_STREAM_ENABLE */
#endif

static float ClampFloat(float value, float minimum, float maximum) {
  if (value < minimum) return minimum;
  if (value > maximum) return maximum;
  return value;
}

/* 16 位自由运行计数器的差值，允许单次采样间跨过一次回绕。 */
static int32_t Encoder_Delta(uint16_t current, uint16_t previous) {
  int32_t delta = (uint16_t)(current - previous);
  if (delta > 32767) delta -= 65536;
  return delta;
}

static void SpeedPid_Reset(SpeedPid *pid) {
  pid->integral = 0.0f;
  pid->previous_error = 0.0f;
  pid->has_previous_error = 0U;
}

/* 双向速度环：支持正/反转及反接主动制动，输出带符号 PWM [-SPEED_MAX_PWM, +SPEED_MAX_PWM] */
static int32_t SpeedPid_Update(SpeedPid *pid, float target, float actual, float dt) {
  if (fabsf(target) < 1.0f && fabsf(actual) <= 1000.0f) {
    pid->integral *= 0.9f;
    pid->previous_error = 0.0f;
    pid->has_previous_error = 0U;
    return 0;
  }
  float error = target - actual;
  float derivative = pid->has_previous_error ? (error - pid->previous_error) / dt : 0.0f;
  float candidate_integral = ClampFloat(pid->integral + PidHelper_Ki() * error * dt,
                                        -SPEED_INTEGRAL_LIMIT, SPEED_INTEGRAL_LIMIT);
  float feedforward = 0.0f;
  if (target >= 1.0f) {
    feedforward = SPEED_FF_OFFSET + SPEED_FF_GAIN * target;
  } else if (target <= -1.0f) {
    feedforward = -(SPEED_FF_OFFSET + SPEED_FF_GAIN * (-target));
  }
  float output = feedforward + PidHelper_Kp() * error + candidate_integral + PidHelper_Kd() * derivative;

  if ((output <= SPEED_MAX_PWM || error < 0.0f) && (output >= -SPEED_MAX_PWM || error > 0.0f)) {
    pid->integral = candidate_integral;
  }
  else {
    output = feedforward + PidHelper_Kp() * error + pid->integral + PidHelper_Kd() * derivative;
  }
  pid->previous_error = error;
  pid->has_previous_error = 1U;
  return (int32_t)ClampFloat(output, -SPEED_MAX_PWM, SPEED_MAX_PWM);
}

/* 光电管从左到右编号 0~11，位置权重为 -11、-9、...、9、11。 */
#if BASIC_LINE_MODE
static LineStatus Line_GetError(float *error) {
  uint16_t mux_value;
  int32_t position_sum = 0;
  uint8_t active_count = 0;
  uint8_t i;

  MUX_get_value(&mux_value);
  for (i = 0; i < MULTIPLEXER_CHANNEL_NUM; ++i) {
    if (MUX_GET_CHANNEL(mux_value, i) == LINE_ACTIVE_LEVEL) {
      position_sum += (int32_t)i * 2 - 11;
      ++active_count;
    }
  }

#if ENCODER_TEST_MODE == 2
  line_telemetry.mux_value = mux_value;
  line_telemetry.active_count = active_count;
#endif

  if (active_count == 0U) return LINE_LOST;
  if (active_count > LINE_MAX_ACTIVE) return LINE_TOO_WIDE;
  *error = (float)position_sum / active_count;
  return LINE_FOUND;
}
#endif

static void Line_FollowStep(float dt) {
  float left_target;
  float right_target;
  int32_t left_pwm;
  int32_t right_pwm;
#if BASIC_LINE_MODE
  float error;
  float correction;
  float error_rate;
  float abs_error;
  float abs_diff_5ms;
  float severity;
  float speed_scale;
  float base_speed;
  LineStatus line_status;
#endif
  uint16_t left_count = (uint16_t)__HAL_TIM_GET_COUNTER(&htim3);
  uint16_t right_count = (uint16_t)__HAL_TIM_GET_COUNTER(&htim4);
  int32_t left_delta = Encoder_Delta(left_count, previous_left_count) * LEFT_ENCODER_SIGN;
  int32_t right_delta = Encoder_Delta(right_count, previous_right_count) * RIGHT_ENCODER_SIGN;
  float left_speed = (float)left_delta / dt;
  float right_speed = (float)right_delta / dt;
#if ENCODER_TEST_MODE == 5 && !TELEMETRY_STREAM_ENABLE
  nav_diag_dt_ms = (uint32_t)(dt * 1000.0f + 0.5f);
  nav_diag_left_speed = (int32_t)left_speed;
  nav_diag_right_speed = (int32_t)right_speed;
#endif

  previous_left_count = left_count;
  previous_right_count = right_count;

#if PID_HELPER_ENABLE && PID_HELPER_IN_MODE
  /* 上位机接管：跳过循迹，两轮以上位机下发的同一目标速度跑同一个速度环 */
  if (PidHelper_IsTuning()) {
    float pid_target = 0.0f;

    if (PidHelper_IsOpenMode()) {
      left_pwm = (int32_t)PidHelper_OpenOutput();
      right_pwm = left_pwm;
    }
    else {
      pid_target = PidHelper_TargetSpeed();
      left_pwm = SpeedPid_Update(&left_speed_pid, pid_target, left_speed, dt);
      right_pwm = SpeedPid_Update(&right_speed_pid, pid_target, right_speed, dt);
    }

    Motor_SetOutput(left_pwm, right_pwm);
    PidHelper_Report(left_speed, pid_target, (float)left_pwm);
    return;
  }
#endif
#if NAVIGATION_MODE
  if (TrackNav_Step(left_delta, right_delta, dt, &left_target, &right_target)) {
    previous_nav_left_target = 0.0f;
    previous_nav_right_target = 0.0f;
    SpeedPid_Reset(&left_speed_pid);
    SpeedPid_Reset(&right_speed_pid);
    Motor_StopAll();
    PidHelper_Report(left_speed, 0.0f, 0.0f);
    return;
  }
  previous_nav_left_target = left_target;
  previous_nav_right_target = right_target;
  left_pwm = SpeedPid_Update(&left_speed_pid, left_target, left_speed, dt);
  right_pwm = SpeedPid_Update(&right_speed_pid, right_target, right_speed, dt);
  Motor_SetOutput(left_pwm, right_pwm);
  PidHelper_Report(left_speed, left_target, (float)left_pwm);
  return;
#endif
#if BASIC_LINE_MODE
  line_status = Line_GetError(&error);
#if ENCODER_TEST_MODE == 2
  line_telemetry.sample_tick = HAL_GetTick();
  line_telemetry.left_speed = (int32_t)left_speed;
  line_telemetry.right_speed = (int32_t)right_speed;
#endif
  if (line_status != LINE_FOUND) {
    uint32_t now = HAL_GetTick();

    has_previous_line_error = 0U;
    if (!line_lost) {
      line_lost = 1U;
      line_lost_tick = now;
      line_search_direction = (last_line_direction != 0) ? last_line_direction : 1;
    } else if ((uint32_t)(now - line_lost_tick) >= LINE_SEARCH_SWEEP_MS) {
      line_search_direction = (int8_t)(-line_search_direction);
      line_lost_tick = now;
    }

    /* 丢线时不停车，采用外轮正转+内轮反转高速扫线，超时后反向扫线 */
    left_target = (line_search_direction > 0) ? LINE_SEARCH_SPEED : -LINE_SEARCH_REV_SPEED;
    right_target = (line_search_direction < 0) ? LINE_SEARCH_SPEED : -LINE_SEARCH_REV_SPEED;
    left_pwm = SpeedPid_Update(&left_speed_pid, left_target, left_speed, dt);
    right_pwm = SpeedPid_Update(&right_speed_pid, right_target, right_speed, dt);
    Motor_SetOutput(left_pwm, right_pwm);
#if ENCODER_TEST_MODE == 2
    line_telemetry.state = (line_status == LINE_TOO_WIDE) ? 'W' : 'S';
    line_telemetry.error_x100 = 0;
    line_telemetry.left_target = (int32_t)left_target;
    line_telemetry.right_target = (int32_t)right_target;
    line_telemetry.left_pwm = TIM1->CCR1;
    line_telemetry.right_pwm = TIM1->CCR2;
#endif
    PidHelper_Report(left_speed, left_target, (float)left_pwm);
    return;
  }

  line_lost = 0U;
  if (error >= 2.0f) last_line_direction = 1;
  else if (error <= -2.0f) last_line_direction = -1;
  else if (last_line_direction == 0) {
    if (error > 0.5f) last_line_direction = 1;
    else if (error < -0.5f) last_line_direction = -1;
  }

  error_rate = has_previous_line_error ? (error - previous_line_error) / dt : 0.0f;
  previous_line_error = error;
  has_previous_line_error = 1U;
  abs_error = (error < 0.0f) ? -error : error;
  correction = LINE_KP * error + LINE_KP2 * error * abs_error + LINE_KD * error_rate;

  abs_diff_5ms = ((error_rate < 0.0f) ? -error_rate : error_rate) * 0.005f;
  severity = abs_error;
  if (abs_diff_5ms * 2.5f > severity) severity = abs_diff_5ms * 2.5f;

  speed_scale = 1.0f - 0.14f * (severity - 3.5f);
  if (speed_scale > 1.0f) speed_scale = 1.0f;
  if (speed_scale < 0.30f) speed_scale = 0.30f;
  base_speed = SPEED_BASE_TARGET * speed_scale;

  left_target = ClampFloat(base_speed + correction, -SPEED_MAX_REV_TARGET, SPEED_MAX_TARGET);
  right_target = ClampFloat(base_speed - correction, -SPEED_MAX_REV_TARGET, SPEED_MAX_TARGET);
  left_pwm = SpeedPid_Update(&left_speed_pid, left_target, left_speed, dt);
  right_pwm = SpeedPid_Update(&right_speed_pid, right_target, right_speed, dt);
  Motor_SetOutput(left_pwm, right_pwm);
#if ENCODER_TEST_MODE == 2
  line_telemetry.state = 'F';
  line_telemetry.error_x100 = (int32_t)(error * 100.0f);
  line_telemetry.left_target = (int32_t)left_target;
  line_telemetry.right_target = (int32_t)right_target;
  line_telemetry.left_pwm = TIM1->CCR1;
  line_telemetry.right_pwm = TIM1->CCR2;
#endif
  PidHelper_Report(left_speed, left_target, (float)left_pwm);
#endif
}

/* 发车键低电平按下；导航模式下等待 200 ms 消抖后交由 TrackNav_Start 完成剩余 800 ms 标定，总计停 1 秒。 */
static void WaitForRunKey(void) {
  for (;;) {
    /* 上电就开始发遥测：不按发车键也能架空轮子做静态验证（此时电机不动） */
#if ENCODER_TEST_MODE != 6
    TelemetryStream_Poll();
#endif
#if PID_HELPER_ENABLE && PID_HELPER_IN_MODE
    /* 上位机接管时不必等发车键；这里同时保持命令接收和遥测在线 */
    PidHelper_Poll();
    if (PidHelper_IsTuning()) {
      return;
    }
#endif
    if (HAL_GPIO_ReadPin(GPIO_RUN_GPIO_Port, GPIO_RUN_Pin) == GPIO_PIN_RESET) {
      HAL_Delay(RUN_KEY_DEBOUNCE_MS);
      if (HAL_GPIO_ReadPin(GPIO_RUN_GPIO_Port, GPIO_RUN_Pin) == GPIO_PIN_RESET) {
        break;
      }
    }
    else {
      HAL_Delay(1U);
    }
  }

#if NAVIGATION_MODE
  HAL_Delay(RUN_KEY_SETTLE_MS);
#else
  HAL_Delay(RUN_START_DELAY_MS);
#endif
}
/* USER CODE END 0 */
void Init_brushless_motor(void) {
  /* PA11 idles low: the first high level then also lasts exactly 0.9 ms,
     because the DMA writes the first slot 0.1 ms after TIM1 is started. */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_RESET);
  ESC_BuildDmaTable(ESC_START_PULSE_SLOTS);
  esc_period_count = 0U;
  hdma_tim1_up.XferCpltCallback = ESC_DmaTransferComplete;
  if (HAL_DMA_Start_IT(&hdma_tim1_up,(uint32_t)esc_dma_table,(uint32_t)&GPIOA->BSRR,ESC_PULSE_SLOTS) != HAL_OK)
    Error_Handler();
  __HAL_TIM_CLEAR_FLAG(&htim1, TIM_FLAG_UPDATE);
  __HAL_TIM_ENABLE_DMA(&htim1, TIM_DMA_UPDATE);
  if (IF_start_Brushless_motor)
    while (esc_period_count < ESC_START_PERIODS)
      HAL_Delay(200);//等待无刷电桥负压启动完成
  /* Disabled fan: return immediately; DMA completes the existing
     0.9 ms -> 1.0 ms idle-signal sequence in the background. */
}
/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {
  /* USER CODE BEGIN 1 */
#if ENCODER_TEST_MODE != 1
  uint32_t last_control_cyc;
  uint32_t period_cyc;
#if ENCODER_TEST_MODE == 2
  uint32_t last_telemetry_tick;
#endif
#endif
  /* USER CODE END 1 */
  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  /* USER CODE END Init */
  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  DWT_Init();
  /* USER CODE END SysInit */
  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_SPI1_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_USART3_UART_Init();
  MX_TIM4_Init();
  if (HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL) != HAL_OK ||
      HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL) != HAL_OK)
    Error_Handler();
#if ENCODER_TEST_MODE != 1
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  Init_brushless_motor(); //等待无刷电调负压启动完成
#endif
  /* USER CODE BEGIN 2 */
  /* 双轮编码器 + BMI270 陀螺仪数据流，20 Hz。与 PID Helper 互斥（都占 USART3），
     由 telemetry_stream.h 里的 TELEMETRY_STREAM_ENABLE 选择用哪个。 */
#if ENCODER_TEST_MODE != 6
  TelemetryStream_Init(&huart3, LEFT_ENCODER_SIGN, RIGHT_ENCODER_SIGN);
#endif
  /* PID Helper 调参协议挂在 USART3（PB10/PB11）；默认增益取编译期参数。 */
#if ENCODER_TEST_MODE == 4 || ENCODER_TEST_MODE == 5 || ENCODER_TEST_MODE == 6
  /* 保留速度 PID 的默认增益，不启用 PID Helper 串口收发。 */
  PidHelper_Init(NULL, SPEED_KP, SPEED_KI, SPEED_KD, TRACK_MAX_SPEED, SPEED_MAX_PWM);
#else
  PidHelper_Init(&huart3, SPEED_KP, SPEED_KI, SPEED_KD, TRACK_MAX_SPEED, SPEED_MAX_PWM);
#endif
#if ENCODER_TEST_MODE != 1 || TELEMETRY_STREAM_ENABLE
  dodo_BMI270_init(); // 初始化陀螺仪
#endif
#if ENCODER_TEST_MODE != 1
  WaitForRunKey();
#endif
#if NAVIGATION_MODE
  TrackNav_Start();
#endif
  previous_left_count = (uint16_t)__HAL_TIM_GET_COUNTER(&htim3);
  previous_right_count = (uint16_t)__HAL_TIM_GET_COUNTER(&htim4);
#if ENCODER_TEST_MODE != 1
  last_control_cyc = DWT->CYCCNT;
  period_cyc = LINE_PERIOD_US * (SystemCoreClock / 1000000U);
#if ENCODER_TEST_MODE == 2
  last_telemetry_tick = HAL_GetTick();
#endif
#endif
  /* USER CODE END 2 */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1) {
#if ENCODER_TEST_MODE == 1
#if TELEMETRY_STREAM_ENABLE
    /* 数据流模式独占串口；保持快速轮询，不发送编码器测试文本。 */
    HAL_Delay(1U);
#else
    uint16_t left_count;
    uint16_t right_count;
    int32_t left_delta;
    int32_t right_delta;
    char message[96];
    int length;

    HAL_Delay(ENCODER_TEST_PERIOD_MS);
    left_count = (uint16_t)__HAL_TIM_GET_COUNTER(&htim3);
    right_count = (uint16_t)__HAL_TIM_GET_COUNTER(&htim4);
    left_delta = Encoder_Delta(left_count, previous_left_count);
    right_delta = Encoder_Delta(right_count, previous_right_count);
    previous_left_count = left_count;
    previous_right_count = right_count;
    length = snprintf(message, sizeof(message),
                      "L_raw=%+ld R_raw=%+ld L_forward=%+ld R_forward=%+ld\r\n",
                      (long)left_delta, (long)right_delta,
                      (long)(left_delta * LEFT_ENCODER_SIGN),
                      (long)(right_delta * RIGHT_ENCODER_SIGN));
    if (length > 0 && (unsigned int)length < sizeof(message))
      HAL_UART_Transmit(&huart3, (uint8_t *)message, (uint16_t)length, 100U);
#endif
#else
    uint32_t now_cyc = DWT->CYCCNT;
    uint32_t elapsed_cyc = now_cyc - last_control_cyc;
    if (elapsed_cyc >= period_cyc) {
      float dt_s = (float)elapsed_cyc / (float)SystemCoreClock;
      if (dt_s > 0.05f) dt_s = 0.05f;
      last_control_cyc = now_cyc;
      Line_FollowStep(dt_s);
#if ENCODER_TEST_MODE == 2
      uint32_t now_tick = HAL_GetTick();
      if ((uint32_t)(now_tick - last_telemetry_tick) >= LINE_TELEMETRY_PERIOD_MS) {
        last_telemetry_tick = now_tick;
#if !PID_HELPER_ENABLE && !TELEMETRY_STREAM_ENABLE
        /* 调参协议或数据流占用同一个串口时，CSV 文本输出必须让位 */
        Line_SendTelemetry();
#endif
      }
#endif
    }
#endif
    /* USER CODE END WHILE */
    /* USER CODE BEGIN 3 */
#if PID_HELPER_IN_MODE
    PidHelper_Poll(); /* 20 ms 一帧遥测 + 串口接收兜底 */
#endif
#if ENCODER_TEST_MODE != 6
    TelemetryStream_Poll(); /* 50 ms 一帧：双轮增量 + 陀螺仪三轴 */
#endif
#if ENCODER_TEST_MODE == 5 && !TELEMETRY_STREAM_ENABLE
    NavDiag_Poll();
#endif
  }
  /* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
   * in the RCC_OscInitTypeDef structure.
   */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) { Error_Handler(); }

  /** Initializes the CPU, AHB and APB buses clocks
   */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) { Error_Handler(); }
}

/**
 * @brief SPI1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_SPI1_Init(void) {
  /* USER CODE BEGIN SPI1_Init 0 */
  /* USER CODE END SPI1_Init 0 */
  /* USER CODE BEGIN SPI1_Init 1 */
  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK) { Error_Handler(); }
  /* USER CODE BEGIN SPI1_Init 2 */
  /* USER CODE END SPI1_Init 2 */
}

/**
 * @brief TIM1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM1_Init(void) {
  /* USER CODE BEGIN TIM1_Init 0 */
  /* USER CODE END TIM1_Init 0 */
  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */
  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 7199;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK) { Error_Handler(); }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK) { Error_Handler(); }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK) { Error_Handler(); }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK) { Error_Handler(); }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK) { Error_Handler(); }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK) { Error_Handler(); }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK) { Error_Handler(); }
  /* USER CODE BEGIN TIM1_Init 2 */
  TIM1->CCMR1 |= TIM_CCMR1_OC1PE | TIM_CCMR1_OC2PE;
  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);
}

/**
 * @brief TIM2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM2_Init(void) {
  /* USER CODE BEGIN TIM2_Init 0 */
  /* USER CODE END TIM2_Init 0 */
  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */
  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 71;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK) { Error_Handler(); }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK) { Error_Handler(); }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK) { Error_Handler(); }
  /* USER CODE BEGIN TIM2_Init 2 */
  /* USER CODE END TIM2_Init 2 */
}

/**
 * @brief TIM3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM3_Init(void) {
  /* USER CODE BEGIN TIM3_Init 0 */
  /* USER CODE END TIM3_Init 0 */
  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */
  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim3, &sConfig) != HAL_OK) { Error_Handler(); }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK) { Error_Handler(); }
  /* USER CODE BEGIN TIM3_Init 2 */
  /* USER CODE END TIM3_Init 2 */
}

/**
 * @brief TIM4 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM4_Init(void) {
  /* USER CODE BEGIN TIM4_Init 0 */
  /* USER CODE END TIM4_Init 0 */
  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */
  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim4, &sConfig) != HAL_OK) { Error_Handler(); }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK) { Error_Handler(); }
  /* USER CODE BEGIN TIM4_Init 2 */
  /* USER CODE END TIM4_Init 2 */
}

/**
 * @brief USART3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART3_UART_Init(void) {
  /* USER CODE BEGIN USART3_Init 0 */
  /* USER CODE END USART3_Init 0 */
  /* USER CODE BEGIN USART3_Init 1 */
  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK) { Error_Handler(); }
  /* USER CODE BEGIN USART3_Init 2 */
  /* USER CODE END USART3_Init 2 */
}

/**
 * Enable DMA controller clock
 */
static void MX_DMA_Init(void) {
  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);
  /* DMA1_Channel3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel3_IRQn);
}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */
  /* USER CODE END MX_GPIO_Init_1 */
  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4 | R_DIR_Pin | GPIO_PIN_11 | MUX_0_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, L_DIR_Pin | MUX_1_Pin | MUX_2_Pin | MUX_3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PA4 */
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : GPIO_RUN_Pin */
  GPIO_InitStruct.Pin = GPIO_RUN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIO_RUN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : L_DIR_Pin */
  GPIO_InitStruct.Pin = L_DIR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(L_DIR_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : R_DIR_Pin PA11 */
  GPIO_InitStruct.Pin = R_DIR_Pin | GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : MUX_READ_Pin */
  GPIO_InitStruct.Pin = MUX_READ_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(MUX_READ_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : MUX_0_Pin */
  GPIO_InitStruct.Pin = MUX_0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
  HAL_GPIO_Init(MUX_0_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : MUX_1_Pin MUX_2_Pin MUX_3_Pin */
  GPIO_InitStruct.Pin = MUX_1_Pin | MUX_2_Pin | MUX_3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
/* USER CODE END 4 */
/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  MOTOR_EMERGENCY_STOP();
  __disable_irq();
  while (1) {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t *file, uint32_t line) {
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
