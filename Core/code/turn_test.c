#include "turn_test.h"
#include "main.h"
#include "dodo_BMI270.h"
#include <math.h>
#include <stdint.h>
#include <stdio.h>

/* 以下转向 PID 和 PWM 均为实车试调初值。 */
#define TURN_SAMPLE_MS 5U
#define TURN_MAX_SAMPLE_MS 50U
#define TURN_CALIBRATE_MS 1200U
#define TURN_CALIBRATE_SPAN_RAW 160
#define TURN_PAUSE_MS 1000U
#define TURN_SETTLE_MS 200U
#define TURN_DIRECTION_DELAY_MS 100U
#define TURN_TIMEOUT_MS 20000U
#define TURN_PROGRESS_TIMEOUT_MS 2500U
#define TURN_ANGLE_TOLERANCE 2.0f
#define TURN_ANGLE_SCALE 0.9807f /* 实跑四次约 352°、积分 358.93°，暂按比例修正 */
#define TURN_SETTLE_RATE 3.0f
#define TURN_MIN_PWM 280U
#define TURN_MAX_PWM 900U
#define TURN_RATE_MAX 200.0f       /* 最大目标角速度，度/秒 */
#define TURN_RATE_MIN 18.0f        /* 小角度修正的目标角速度，度/秒 */
#define TURN_ANGLE_KP 4.0f         /* 剩余角度 -> 目标角速度 */
#define TURN_RATE_KP 1.5f          /* 角速度 PID，输出单位为 PWM */
#define TURN_RATE_KI 0.8f
#define TURN_RATE_KD 0.003f
#define TURN_RATE_I_LIMIT 80.0f
#define TURN_RATE_FF_OFFSET 240.0f
#define TURN_RATE_FF_GAIN 0.9f
#define TURN_ENCODER_VERIFY_MS 600U

extern UART_HandleTypeDef huart3;

typedef enum { TURN_CAL, TURN_MOVING, TURN_PAUSE, TURN_DONE, TURN_ERROR } TurnState;
typedef enum { FAULT_NONE, FAULT_GYRO, FAULT_MOVED, FAULT_SAMPLE,
               FAULT_TIMEOUT, FAULT_NO_PROGRESS, FAULT_WRONG_TURN,
               FAULT_ENCODER_DIRECTION, FAULT_ENCODER_STALL } TurnFault;

static TurnState state;
static TurnFault fault;
static uint32_t last_sample, state_tick, last_report, settle_tick, direction_tick;
static uint32_t progress_tick, drive_tick;
static int32_t cal_sum, cal_count;
static int16_t cal_min, cal_max;
static float gyro_bias, angle, rate, best_error, drive_start_angle;
static float rate_integral, previous_measured_rate, target_rate;
static uint8_t rate_pid_valid;
static uint32_t output_pwm;
static uint8_t completed, settling, drive_started;
static int8_t direction, pending_direction;
static uint16_t prev_left, prev_right;
static int32_t drive_left, drive_right;
static char report_buffer[192];

static void motor_stop(void) {
  TIM1->CCR1 = 0U;
  TIM1->CCR2 = 0U;
  rate_integral = 0.0f;
  rate_pid_valid = 0U;
  target_rate = 0.0f;
  output_pwm = 0U;
}

/* 角速度环按目标方向归一化测量值；微分作用于测量值，避免目标跳变冲击。 */
static uint32_t rate_pid_update(float target, float measured, float dt) {
  float error = target - measured;
  float derivative = rate_pid_valid ? -(measured - previous_measured_rate) / dt : 0.0f;
  float candidate = rate_integral + TURN_RATE_KI * error * dt;
  float output;
  if (candidate > TURN_RATE_I_LIMIT) candidate = TURN_RATE_I_LIMIT;
  if (candidate < -TURN_RATE_I_LIMIT) candidate = -TURN_RATE_I_LIMIT;
  output = TURN_RATE_FF_OFFSET + TURN_RATE_FF_GAIN * target + TURN_RATE_KP * error +
           candidate + TURN_RATE_KD * derivative;
  /* 限幅时不继续积累会使输出更饱和的积分。 */
  if (!((output > TURN_MAX_PWM && error > 0.0f) || (output < 0.0f && error < 0.0f)))
    rate_integral = candidate;
  output = TURN_RATE_FF_OFFSET + TURN_RATE_FF_GAIN * target + TURN_RATE_KP * error +
           rate_integral + TURN_RATE_KD * derivative;
  previous_measured_rate = measured;
  rate_pid_valid = 1U;
  if (measured > target + 5.0f || output <= 0.0f) return 0U;
  if (output < TURN_MIN_PWM) output = TURN_MIN_PWM;
  if (output > TURN_MAX_PWM) output = TURN_MAX_PWM;
  return (uint32_t)output;
}

static void fail(TurnFault why) {
  motor_stop();
  fault = why;
  state = TURN_ERROR;
}

static int32_t delta16(uint16_t current, uint16_t previous) {
  int32_t delta = (int32_t)(uint16_t)(current - previous);
  return delta > 32767 ? delta - 65536 : delta;
}

/* 电机反向前先关闭两路 PWM，等待后再改变 DIR；1 表示顺时针。 */
static uint8_t set_direction(int8_t desired, uint32_t now) {
  if (desired != direction) {
    motor_stop();
    if (pending_direction != desired) {
      pending_direction = desired;
      direction_tick = now;
      drive_started = 0U;
    }
    if ((uint32_t)(now - direction_tick) < TURN_DIRECTION_DELAY_MS) return 0U;
    /* 顺时针：左轮前进、右轮后退；方向引脚关系须先架空验证。 */
    HAL_GPIO_WritePin(L_DIR_GPIO_Port, L_DIR_Pin, desired > 0 ? GPIO_PIN_RESET : GPIO_PIN_SET);
    HAL_GPIO_WritePin(R_DIR_GPIO_Port, R_DIR_Pin, desired > 0 ? GPIO_PIN_SET : GPIO_PIN_RESET);
    direction = desired;
    pending_direction = 0;
  }
  return 1U;
}

static void report(uint32_t now) {
  static const char *const states[] = { "CAL", "TURN", "WAIT", "DONE", "ERROR" };
  static const char *const faults[] = { "NONE", "GYRO", "MOVED", "SAMPLE", "TIMEOUT",
    "NO_PROGRESS", "WRONG_TURN", "ENCODER_DIRECTION", "ENCODER_STALL" };
  int length;
  if ((uint32_t)(now - last_report) < 250U || HAL_UART_GetState(&huart3) != HAL_UART_STATE_READY) return;
  length = snprintf(report_buffer, sizeof(report_buffer),
                    "TURN,state=%s,done=%u,angle_cdeg=%ld,target_cdeg=%ld,rate_cdeg_s=%ld,fault=%s,target_rate_cdeg_s=%ld,pwm=%lu\r\n",
                    states[state], (unsigned int)completed, (long)(angle * 100.0f),
                    (long)((completed < 4U ? completed + 1U : 4U) * 9000L),
                    (long)(rate * 100.0f), faults[fault],
                    (long)(target_rate * 100.0f), (unsigned long)output_pwm);
  if (length > 0 && (unsigned int)length < sizeof(report_buffer) &&
      HAL_UART_Transmit_IT(&huart3, (uint8_t *)report_buffer, (uint16_t)length) == HAL_OK)
    last_report = now;
}

void TurnTest_Start(void) {
  motor_stop();
  state = TURN_CAL;
  fault = FAULT_NONE;
  last_sample = state_tick = last_report = HAL_GetTick();
  cal_sum = 0;
  cal_count = 0;
  cal_min = INT16_MAX;
  cal_max = INT16_MIN;
  gyro_bias = angle = rate = 0.0f;
  completed = settling = drive_started = 0U;
  direction = pending_direction = 0;
  if (BMI270_last_rslt != 0) fail(FAULT_GYRO);
}

void TurnTest_Poll(void) {
  uint32_t now = HAL_GetTick();
  uint32_t dt;
  float error, absolute_error;
  int8_t desired;

  if (state == TURN_DONE || state == TURN_ERROR) {
    motor_stop();
    report(now);
    return;
  }
  if ((uint32_t)(now - last_sample) < TURN_SAMPLE_MS) return;
  dodo_BMI270_get_data();
  now = HAL_GetTick();
  dt = (uint32_t)(now - last_sample);
  last_sample = now;
  if (BMI270_last_rslt != 0) { fail(FAULT_GYRO); report(now); return; }
  if (dt > TURN_MAX_SAMPLE_MS) { fail(FAULT_SAMPLE); report(now); return; }

  if (state == TURN_CAL) {
    int16_t raw = BMI270_gyro_z;
    cal_sum += raw;
    ++cal_count;
    if (raw < cal_min) cal_min = raw;
    if (raw > cal_max) cal_max = raw;
    if ((int32_t)cal_max - cal_min > TURN_CALIBRATE_SPAN_RAW) fail(FAULT_MOVED);
    else if ((uint32_t)(now - state_tick) >= TURN_CALIBRATE_MS) {
      gyro_bias = (float)cal_sum / (float)cal_count;
      state = TURN_MOVING;
      state_tick = progress_tick = now;
      best_error = 90.0f;
    }
    report(now);
    return;
  }

  /* 手动实测为逆时针时原始 Z 正值，取负后内部正角表示顺时针。 */
  rate = -((float)BMI270_gyro_z - gyro_bias) / 16.4f;
  angle += rate * (float)dt / 1000.0f * TURN_ANGLE_SCALE;
  if (state == TURN_PAUSE) {
    motor_stop();
    if ((uint32_t)(now - state_tick) >= TURN_PAUSE_MS) {
      state = TURN_MOVING;
      state_tick = progress_tick = now;
      best_error = fabsf((completed + 1U) * 90.0f - angle);
      settling = drive_started = 0U;
    }
    report(now);
    return;
  }

  error = (completed + 1U) * 90.0f - angle;
  absolute_error = fabsf(error);
  if (absolute_error + 0.5f < best_error) { best_error = absolute_error; progress_tick = now; }
  if ((uint32_t)(now - state_tick) >= TURN_TIMEOUT_MS) { fail(FAULT_TIMEOUT); report(now); return; }
  if ((uint32_t)(now - progress_tick) >= TURN_PROGRESS_TIMEOUT_MS) { fail(FAULT_NO_PROGRESS); report(now); return; }

  if (absolute_error <= TURN_ANGLE_TOLERANCE && fabsf(rate) < TURN_SETTLE_RATE) {
    motor_stop();
    drive_started = 0U;
    if (!settling) { settling = 1U; settle_tick = now; }
    if ((uint32_t)(now - settle_tick) >= TURN_SETTLE_MS) {
      ++completed;
      state_tick = now;
      state = completed == 4U ? TURN_DONE : TURN_PAUSE;
      settling = 0U;
    }
    report(now);
    return;
  }
  settling = 0U;
  desired = error > 0.0f ? 1 : -1;
  /* 考虑惯性，接近目标时先断开驱动；停稳后仍有误差则再低速修正。 */
  if (rate * desired > 0.0f && absolute_error <= fabsf(rate) * 0.10f + 0.8f) {
    motor_stop();
    drive_started = 0U;
    report(now);
    return;
  }
  if (set_direction(desired, now)) {
    uint16_t left = (uint16_t)TIM3->CNT, right = (uint16_t)TIM4->CNT;
    target_rate = TURN_ANGLE_KP * absolute_error;
    if (target_rate < TURN_RATE_MIN) target_rate = TURN_RATE_MIN;
    if (target_rate > TURN_RATE_MAX) target_rate = TURN_RATE_MAX;
    output_pwm = rate_pid_update(target_rate, rate * desired, (float)dt / 1000.0f);
    if (output_pwm == 0U) {
      /* 实际角速度过高时先滑行减速，不让最小 PWM 持续推动车轮。 */
      motor_stop();
      drive_started = 0U;
      report(now);
      return;
    }
    if (!drive_started) {
      drive_started = 1U;
      drive_tick = now;
      drive_start_angle = angle;
      prev_left = left;
      prev_right = right;
      drive_left = drive_right = 0;
    }
    /* 按本次指令的方向归一化；左右轮前进的编码器原始符号相反。 */
    drive_left += delta16(left, prev_left) * desired;
    drive_right += delta16(right, prev_right) * desired;
    prev_left = left;
    prev_right = right;
    if (drive_left < -50 || drive_right < -50) { fail(FAULT_ENCODER_DIRECTION); report(now); return; }
    if ((uint32_t)(now - drive_tick) >= TURN_ENCODER_VERIFY_MS && (drive_left < 20 || drive_right < 20)) {
      fail(FAULT_ENCODER_STALL); report(now); return;
    }
    if ((angle - drive_start_angle) * desired < -6.0f) { fail(FAULT_WRONG_TURN); report(now); return; }
    TIM1->CCR1 = output_pwm;
    TIM1->CCR2 = output_pwm;
  }
  report(now);
}
