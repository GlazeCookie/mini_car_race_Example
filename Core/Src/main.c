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
#include "pid_helper_app.h" //PID Helper 上位机调参协议（USART3）
#include "telemetry_stream.h" //双轮编码器 + 陀螺仪数据流（USART3，与调参互斥）
/* USER CODE END Includes */
/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */
/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define IF_start_Brushless_motor 0 //是否启动无刷电机，1为启动，0为不启动
#define ENCODER_TEST_MODE 2 /* 0: 循迹；1: 编码器测试；2: 循迹并发送实时数据 */
#define ENCODER_TEST_PERIOD_MS 200U
#define LINE_TELEMETRY_PERIOD_MS 100U
#if ENCODER_TEST_MODE < 0 || ENCODER_TEST_MODE > 2
#error "ENCODER_TEST_MODE must be 0, 1 or 2"
#endif
#define RUN_KEY_DEBOUNCE_MS 20U
#define RUN_START_DELAY_MS  2000U
/* 循迹和速度参数均需根据电机、编码器和赛道实车标定。 */
#define LINE_ACTIVE_LEVEL  1U /* 白线输出高电平 */
#define LINE_MAX_ACTIVE    12U
#define LINE_PERIOD_MS     5U
/* 转向修正系数随基准速度等比放大（2500 -> 12000 是 ×4.8），再针对折线急弯加强：
   修正量 = LINE_KP×e + LINE_KP2×e×|e| + LINE_KD×Δe
   基准 12000 时，修正量必须能超过 12000，内侧轮才可能降到 0（绕内轮支点急转）。
   取 KP=700 / KP2=60：e=10 时修正 13000 -> 内侧轮归零；e=1 时只有 760（直道不画龙）。 */
#define LINE_KP            700.0f  /* 每单位位置偏差对应的轮速修正，计数/秒 */
#define LINE_KP2           60.0f   /* 大偏差时的二次加强，急弯靠它把内侧轮压到 0 */
#define LINE_KD            80.0f   /* 相邻采样偏差之差的系数，抑制画龙 */
/* 丢线后单轮寻线的 PWM。必须和巡航速度匹配：
   带载 PWM = 200 + 0.010×转速，600 PWM 约等于 2.0 m/s，单轮转起来车体
   角速度接近 950°/s，一找到线车头已经转过头了，表现为"跑反方向"。
   降到 300 PWM ≈ 0.5 m/s，寻线时慢慢转，找到线能及时停下。 */
#define LINE_SEARCH_PWM    300U    /* 丢线后单轮寻线 PWM，需实车调整 */
/* 寻线时间随车速提高而延长：基准速度从 2500 提到 12000（约 4.8 倍）。
   原来 500ms 在 0.13 m/s 下够用，但 0.6 m/s 时 0.5 秒已冲出 30cm，
   超过就会双轮停死、再也恢复不了（环岛卡住就是这么来的）。
   第二圈可能沿旧方向转错，暂时设为 1000 ms 验证，需根据实车情况调整。 */
#define LINE_SEARCH_TIME_MS 1000U /* 最长寻线时间，超过后停车 */
#define LINE_DIRECTION_MAX_AGE_MS 100U /* 最近偏差方向的有效期，需实车调整 */
/* 实测每米约 19841 计数（左轮，人工推车 1 米标定）：
     0.5 m/s ≈ 10000 计数/秒，1.0 m/s ≈ 20000，1.5 m/s ≈ 30000
   原来的 2500 只有 0.13 m/s，整条速度标尺小了约 8 倍。 */
#define SPEED_BASE_TARGET  14000.0f /* 直线目标轮速 ≈ 0.71 m/s */
#define SPEED_MAX_TARGET   29000.0f /* 留足转向修正量：14000 + 最大修正 ≈ 29000 */
/* 前馈 = 死区补偿 + 斜率 × 目标速度（只有目标 > 0 时才生效）。
   实测带载巡航：目标 12000 时稳态 PWM ≈ 322，据此反算
     FF(12000) = 200 + 0.010 × 12000 = 320 ≈ 322  ✓
   这样巡航时积分器几乎不工作；弯道里内侧轮目标骤降时前馈能立刻跟着掉，
   内侧轮才压得下来、差速才够（原来 OFFSET=300 太大，内侧轮压不下去，弯道往外推）。 */
#define SPEED_FF_OFFSET    200.0f  /* 死区补偿 PWM */
#define SPEED_FF_GAIN      0.010f  /* PWM / (计数每秒) */
#define SPEED_MAX_PWM      4000.0f
/* 以下增益由开环辨识 + 闭环阶跃搜索实测得出（空载，轮子悬空）：
     目标 20000~30000 计数/秒：静差 1.1~1.4%，超调 0.9~2.7%，上升 ~170ms
   注意：这是空载特性，上车带载后增益可按需再提高。 */
#define SPEED_KP           0.0012f
#define SPEED_KI           0.07f
#define SPEED_KD           0.0f
#define SPEED_INTEGRAL_LIMIT 1000.0f /* 积分项在 PWM 中的最大绝对贡献 */
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
int fputc(int ch, FILE *f) {
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
static int8_t last_line_direction; /* -1 左，1 右，0 无法判断 */
/* 最近一次明确方向只在误差非 0 时更新，并记录时间。
   居中后短暂丢线可用它兜底；记录过旧时不再猜测方向。 */
static int8_t last_valid_direction;
static uint32_t last_valid_direction_tick;
static int8_t line_search_direction; /* 本次丢线时锁定的寻线方向 */
static uint8_t line_lost;
static uint8_t line_search_expired;
static uint32_t line_lost_tick;
static float previous_line_error;
static uint8_t has_previous_line_error;

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

/* dt 为实际采样间隔（秒）；积分项限幅，并在输出饱和时抑制积分累积。 */
static uint32_t SpeedPid_Update(SpeedPid *pid, float target, float actual, float dt) {
  /* 本车只支持单向 PWM；零目标立即关断并清除历史积分。 */
  if (target <= 0.0f) {
    SpeedPid_Reset(pid);
    return 0U;
  }
  float error = target - actual;
  float derivative = pid->has_previous_error ? (error - pid->previous_error) / dt : 0.0f;
  float candidate_integral = ClampFloat(pid->integral + PidHelper_Ki() * error * dt,
                                        -SPEED_INTEGRAL_LIMIT, SPEED_INTEGRAL_LIMIT);
  /* 死区补偿 + 斜率前馈：目标为 0 时不加，避免停车时轮子还在爬 */
  float feedforward = (target > 0.0f) ? (SPEED_FF_OFFSET + SPEED_FF_GAIN * target) : 0.0f;
  float output = feedforward + PidHelper_Kp() * error + candidate_integral + PidHelper_Kd() * derivative;

  if ((output <= SPEED_MAX_PWM || error < 0.0f) && (output >= 0.0f || error > 0.0f)) {
    pid->integral = candidate_integral;
  }
  else {
    output = feedforward + PidHelper_Kp() * error + pid->integral + PidHelper_Kd() * derivative;
  }
  pid->previous_error = error;
  pid->has_previous_error = 1U;
  return (uint32_t)ClampFloat(output, 0.0f, SPEED_MAX_PWM);
}

/* 光电管从左到右编号 0~11，位置权重为 -11、-9、...、9、11。 */
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

static void Line_FollowStep(uint32_t elapsed_ms) {
  float error;
  float correction;
  float left_target;
  float right_target;
  float error_difference;
  float abs_error;
  float abs_error_diff;
  float severity;
  float speed_scale;
  float base_speed;
  LineStatus line_status;
  float dt = (float)elapsed_ms / 1000.0f;
  uint16_t left_count = (uint16_t)__HAL_TIM_GET_COUNTER(&htim3);
  uint16_t right_count = (uint16_t)__HAL_TIM_GET_COUNTER(&htim4);
  float left_speed = (float)(Encoder_Delta(left_count, previous_left_count) * LEFT_ENCODER_SIGN) / dt;
  float right_speed = (float)(Encoder_Delta(right_count, previous_right_count) * RIGHT_ENCODER_SIGN) / dt;

  previous_left_count = left_count;
  previous_right_count = right_count;

  line_status = Line_GetError(&error);
#if ENCODER_TEST_MODE == 2
  line_telemetry.sample_tick = HAL_GetTick();
  line_telemetry.left_speed = (int32_t)left_speed;
  line_telemetry.right_speed = (int32_t)right_speed;
#endif
#if PID_HELPER_ENABLE
  /* 上位机接管：跳过循迹，两轮以上位机下发的同一目标速度跑同一个速度环 */
  if (PidHelper_IsTuning()) {
    uint32_t left_pwm;
    uint32_t right_pwm;
    float pid_target = 0.0f;

    if (PidHelper_IsOpenMode()) {
      left_pwm = (uint32_t)PidHelper_OpenOutput();
      right_pwm = left_pwm;
    }
    else {
      pid_target = PidHelper_TargetSpeed();
      left_pwm = SpeedPid_Update(&left_speed_pid, pid_target, left_speed, dt);
      right_pwm = SpeedPid_Update(&right_speed_pid, pid_target, right_speed, dt);
    }

    TIM1->CCR1 = left_pwm;
    TIM1->CCR2 = right_pwm;
    PidHelper_Report(left_speed, pid_target, (float)left_pwm);
    return;
  }
#endif
  if (line_status != LINE_FOUND) {
    uint32_t now = HAL_GetTick();

    SpeedPid_Reset(&left_speed_pid);
    SpeedPid_Reset(&right_speed_pid);

    has_previous_line_error = 0U;

    /* 检测范围过宽时不猜测方向，立即停车。 */
    if (line_status == LINE_TOO_WIDE) {
      last_line_direction = 0;
      last_valid_direction = 0;
      line_search_direction = 0;
      line_lost = 0U;
      line_search_expired = 0U;
    }
    else if (!line_lost) {
      line_lost = 1U;
      line_lost_tick = now;
      /* 只使用最近的有效方向，丢线后保持该方向直到找到线或超时。 */
      line_search_direction = (last_valid_direction != 0 &&
          (uint32_t)(now - last_valid_direction_tick) <= LINE_DIRECTION_MAX_AGE_MS)
          ? last_valid_direction : 0;
      line_search_expired = (line_search_direction == 0);
    }

    if (line_lost && !line_search_expired &&
        (uint32_t)(now - line_lost_tick) >= LINE_SEARCH_TIME_MS) {
      line_search_expired = 1U;
      last_valid_direction = 0;
      line_search_direction = 0;
    }

    /* 单轮低速前进，另一轮停转；方向未知、过期或超时时停车。 */
    TIM1->CCR1 = (line_status == LINE_LOST && !line_search_expired && line_search_direction > 0)
                 ? LINE_SEARCH_PWM : 0U;
    TIM1->CCR2 = (line_status == LINE_LOST && !line_search_expired && line_search_direction < 0)
                 ? LINE_SEARCH_PWM : 0U;
#if ENCODER_TEST_MODE == 2
    line_telemetry.state = (line_status == LINE_TOO_WIDE) ? 'W' :
                           (line_search_expired ? 'X' : 'S');
    line_telemetry.error_x100 = 0; /* 无有效偏差，须结合状态判断 */
    line_telemetry.left_target = 0; /* 寻线时直接输出 PWM，不使用速度目标 */
    line_telemetry.right_target = 0;
    line_telemetry.left_pwm = TIM1->CCR1;
    line_telemetry.right_pwm = TIM1->CCR2;
#endif
    PidHelper_Report(left_speed, 0.0f, (float)TIM1->CCR1);
    return;
  }

  line_lost = 0U;
  line_search_expired = 0U;
  last_line_direction = (error > 0.0f) ? 1 : ((error < 0.0f) ? -1 : 0);
  if (last_line_direction != 0) {
    last_valid_direction = last_line_direction;
    last_valid_direction_tick = HAL_GetTick();
  }
  error_difference = has_previous_line_error ? error - previous_line_error : 0.0f;
  previous_line_error = error;
  has_previous_line_error = 1U;
  abs_error = (error < 0.0f) ? -error : error;
  correction = LINE_KP * error + LINE_KP2 * error * abs_error + LINE_KD * error_difference;
  /* 差分项可以减小转向，但不能让修正方向与当前白线偏差相反。 */
  if ((error > 0.0f && correction < 0.0f) ||
      (error < 0.0f && correction > 0.0f) || error == 0.0f)
    correction = 0.0f;

  /* 弯道减速：偏差越大，两侧基础速度一起降（差速不变）。
     差速决定转向的角速度，基础速度决定前进速度；只降前进速度就能减小转弯半径。
     除了看偏差大小，还看偏差"变化多快"：压到直角弯口时偏差会突然跳变，
     这一下就要收油，否则等偏差涨到最大再减速，车头已经过弯口了（0.7 m/s 下
     轮子减速要 ~200ms，这 200ms 车已经前进 14cm）。 */
  abs_error_diff = (error_difference < 0.0f) ? -error_difference : error_difference;
  severity = abs_error;
  if (abs_error_diff * 2.5f > severity) severity = abs_error_diff * 2.5f;

  speed_scale = 1.0f - 0.16f * (severity - 3.0f);
  if (speed_scale > 1.0f) speed_scale = 1.0f;
  if (speed_scale < 0.22f) speed_scale = 0.22f; /* 最慢降到 0.22 倍 ≈ 0.16 m/s */
  base_speed = SPEED_BASE_TARGET * speed_scale;

  left_target = ClampFloat(base_speed + correction, 0.0f, SPEED_MAX_TARGET);
  right_target = ClampFloat(base_speed - correction, 0.0f, SPEED_MAX_TARGET);
  TIM1->CCR1 = SpeedPid_Update(&left_speed_pid, left_target, left_speed, dt);
  TIM1->CCR2 = SpeedPid_Update(&right_speed_pid, right_target, right_speed, dt);
#if ENCODER_TEST_MODE == 2
  line_telemetry.state = 'F';
  line_telemetry.error_x100 = (int32_t)(error * 100.0f);
  line_telemetry.left_target = (int32_t)left_target;
  line_telemetry.right_target = (int32_t)right_target;
  line_telemetry.left_pwm = TIM1->CCR1;
  line_telemetry.right_pwm = TIM1->CCR2;
#endif
  PidHelper_Report(left_speed, left_target, (float)TIM1->CCR1);
}

/* 发车键低电平按下；确认按下后等待 2 秒再开始循迹。 */
static void WaitForRunKey(void) {
  for (;;) {
    /* 上电就开始发遥测：不按发车键也能架空轮子做静态验证（此时电机不动） */
    TelemetryStream_Poll();
#if PID_HELPER_ENABLE
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

  HAL_Delay(RUN_START_DELAY_MS);
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
  uint32_t last_control_tick;
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
  TelemetryStream_Init(&huart3, LEFT_ENCODER_SIGN, RIGHT_ENCODER_SIGN);
  /* PID Helper 调参协议挂在 USART3（PB10/PB11）；默认增益取编译期参数。
     第 5 个参数是调参时允许的目标速度上限，单独给 30000，不改动循迹用的
     SPEED_MAX_TARGET（那个宏循迹逻辑还在用）。 */
  PidHelper_Init(&huart3, SPEED_KP, SPEED_KI, SPEED_KD, 30000.0f, SPEED_MAX_PWM);
#if ENCODER_TEST_MODE != 1 || TELEMETRY_STREAM_ENABLE
  dodo_BMI270_init(); // 初始化陀螺仪
  /* BMI270 初始化也会启用光电管微秒延时所需的 DWT 计数器。 */
#endif
#if ENCODER_TEST_MODE != 1
  WaitForRunKey();
#endif
  previous_left_count = (uint16_t)__HAL_TIM_GET_COUNTER(&htim3);
  previous_right_count = (uint16_t)__HAL_TIM_GET_COUNTER(&htim4);
#if ENCODER_TEST_MODE != 1
  last_control_tick = HAL_GetTick();
#if ENCODER_TEST_MODE == 2
  last_telemetry_tick = last_control_tick;
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
    uint32_t now = HAL_GetTick();
    uint32_t elapsed_ms = now - last_control_tick;
    if (elapsed_ms >= LINE_PERIOD_MS) {
      last_control_tick = now;
      Line_FollowStep(elapsed_ms);
#if ENCODER_TEST_MODE == 2
      if ((uint32_t)(now - last_telemetry_tick) >= LINE_TELEMETRY_PERIOD_MS) {
        last_telemetry_tick = now;
#if !PID_HELPER_ENABLE && !TELEMETRY_STREAM_ENABLE
        /* 调参协议或数据流占用同一个串口时，CSV 文本输出必须让位 */
        Line_SendTelemetry();
#endif
      }
#endif
    }
    else {
      HAL_Delay(1U);
    }
#endif
    /* USER CODE END WHILE */
    /* USER CODE BEGIN 3 */
    PidHelper_Poll(); /* 20 ms 一帧遥测 + 串口接收兜底 */
    TelemetryStream_Poll(); /* 50 ms 一帧：双轮增量 + 陀螺仪三轴 */
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
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(L_DIR_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : R_DIR_Pin PA11 */
  GPIO_InitStruct.Pin = R_DIR_Pin | GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
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
