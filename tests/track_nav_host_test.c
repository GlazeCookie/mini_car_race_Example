#include "main.h"
#include <assert.h>
#include <stdint.h>
#include <stdio.h>

/* 使用真实导航实现；只替换定时、光电管和 BMI270 硬件接口。 */
#define _MULTIPLEXER_H_
#define MULTIPLEXER_CHANNEL_NUM 12
#define MUX_GET_CHANNEL(value, ch) (((value) >> (11 - (ch))) & 1U)
void MUX_get_value(uint16_t *value);
#define _DODO_BMI270_H_
extern int16_t BMI270_gyro_z;
extern int8_t BMI270_last_rslt;
int8_t dodo_BMI270_init(void);
void dodo_BMI270_get_data(void);
int8_t dodo_BMI270_get_gyro_z(int16_t *gyro_z);
#include "../Core/code/track_nav.c"

static uint32_t mock_tick;
static uint16_t mock_line;
static int16_t mock_gyro;
static uint8_t mock_motion;
static uint8_t mock_bad_read;
static uint8_t mock_bad_init;
static uint32_t mock_reads;
int16_t BMI270_gyro_z;
int8_t BMI270_last_rslt;

uint32_t HAL_GetTick(void) { return mock_tick; }
void HAL_Delay(uint32_t delay_ms) { mock_tick += delay_ms; }
void MUX_get_value(uint16_t *value) { *value = mock_line; }
int8_t dodo_BMI270_init(void) { return mock_bad_init ? -1 : 0; }
void dodo_BMI270_get_data(void) {
  BMI270_last_rslt = mock_bad_read ? -1 : 0;
  BMI270_gyro_z = mock_motion ? ((mock_reads++ & 1U) ? 100 : -100) : mock_gyro;
}
int8_t dodo_BMI270_get_gyro_z(int16_t *gyro_z) {
  if (mock_bad_read) {
    BMI270_last_rslt = -1;
    return -1;
  }
  BMI270_last_rslt = 0;
  BMI270_gyro_z = mock_motion ? ((mock_reads++ & 1U) ? 100 : -100) : mock_gyro;
  if (gyro_z != NULL) *gyro_z = BMI270_gyro_z;
  return 0;
}

static void setup(TrackState state) {
  mock_tick = 0; mock_line = 0; mock_gyro = 0; mock_motion = 0;
  mock_bad_read = mock_bad_init = 0; mock_reads = 0;
  BMI270_gyro_z = BMI270_last_rslt = 0;
  s_fault = TRACK_FAULT_NONE; s_state = TRK_START_CLEAR; s_search_active = 0;
  s_gyro_ok = 1U; s_last_raw_rate = 0.0f; s_last_dir = 0;
  s_yaw_deg = s_yaw_ref = s_yaw_west = s_rate_lpf = s_gyro_bias = 0.0f;
  s_has_prev_err = 0U; enter_state(state);
}

static uint8_t step(int32_t left, int32_t right, float *l, float *r) {
  mock_tick += 2U;
  return TrackNav_Step(left, right, 0.002f, l, r);
}

static void test_calibration_and_resilience(void) {
  float l, r;
  setup(TRK_START_CLEAR); TrackNav_Start();
  assert(s_state == TRK_START_CLEAR && s_gyro_ok == 1U);

  /* 剧烈震动或较大零偏不能阻断发车 */
  setup(TRK_START_CLEAR); mock_motion = 1U; TrackNav_Start();
  assert(s_state == TRK_START_CLEAR && s_gyro_ok == 1U);
  setup(TRK_START_CLEAR); mock_gyro = 100; TrackNav_Start();
  assert(s_state == TRK_START_CLEAR && fabsf(s_gyro_bias - 100.0f) < 1e-3f);

  /* 陀螺仪初始化失败或单次读取错误不锁死停车，继续循迹 */
  setup(TRK_START_CLEAR); mock_bad_init = 1U; TrackNav_Start();
  assert(s_state == TRK_START_CLEAR && s_gyro_ok == 0U);
  setup(TRK_ZIGZAG_1); mock_line = 0x60; mock_bad_read = 1U;
  assert(step(10, 10, &l, &r) == 0U && s_state == TRK_ZIGZAG_1 && l > 0.0f && r > 0.0f);
}

static void test_gap(void) {
  float l, r;
  /* 断路前未到 850 mm 的短暂丢线不能提前锁航向，仍走正常巡线寻线 */
  setup(TRK_APPROACH_BROKEN); s_seg_mm = 300.0f; s_last_dir = 1;
  assert(step(0, 0, &l, &r) == 0U && s_state == TRK_APPROACH_BROKEN && l > 0.0f && r < 0.0f);

  /* 到达 850 mm 后连续 3 帧丢线进入断路直行清除段 */
  setup(TRK_APPROACH_BROKEN); s_seg_mm = 860.0f;
  for (int i = 0; i < 3; ++i) {
    assert(step(0, 0, &l, &r) == 0U);
    assert(l == TRACK_BROKEN_SPEED && r == TRACK_BROKEN_SPEED);
  }
  assert(s_state == TRK_BROKEN_CLEAR);

  /* 抓到线后进入巡线对齐并在满足最小距离后进入 PRE_T1 */
  setup(TRK_BROKEN_ALIGN); s_seg_mm = 61.0f; mock_line = 0x60;
  assert(step(0, 0, &l, &r) == 0U && s_state == TRK_TL_SQ_PRE_T1);

  /* 刚进入 PRE_T1 (<120 mm) 即使有左分叉也不误触发 */
  setup(TRK_TL_SQ_PRE_T1); mock_line = 0x3FC;
  assert(step(0, 0, &l, &r) == 0U && s_state == TRK_TL_SQ_PRE_T1);
}

static void test_turn_and_reverse_diff(void) {
  float l, r;
  /* 定角转向使用外轮正转+内轮反转 */
  setup(TRK_HEX_BL_IN);
  assert(step(0, 0, &l, &r) == 0U && l == TRACK_TURN_OUTER_SPEED && r == -TRACK_TURN_INNER_REV_SPEED);

  /* 即使转弯超时也不故障锁死，兜底退出继续跑 */
  setup(TRK_TL_SQ_TURN_E); mock_tick = NAV_TURN_FALLBACK_MS;
  assert(step(0, 0, &l, &r) == 0U && s_state == TRK_TL_SQ_TO_TR2);

  /* 消抖：单帧路口不误切状态，连续两帧才切 */
  setup(TRK_APPROACH_HEX_R); s_seg_mm = 300.0f; mock_line = 0x3FC;
  assert(step(0, 0, &l, &r) == 0U && s_state == TRK_APPROACH_HEX_R);
  assert(step(0, 0, &l, &r) == 0U && s_state == TRK_HEX_R_ENTER);
}

static void test_finish(void) {
  float l, r;
  setup(TRK_FINISH_ARMED); mock_line = 0x1F8; /* 6 路普通白线 */
  for (int i = 0; i < 10; ++i) assert(step(0, 0, &l, &r) == 0U);
  assert(s_state == TRK_FINISH_ARMED);
  mock_line = 0x3FC; /* 8 路横线 */
  for (int i = 0; i < 2; ++i) assert(step(0, 0, &l, &r) == 0U);
  assert(s_state == TRK_FINISH_ARMED);
  assert(step(0, 0, &l, &r) == 0U && s_state == TRK_FINISH_BAR1);
  for (int i = 0; i < 20; ++i) assert(step(1200, 1200, &l, &r) == 0U);
  assert(s_state == TRK_FINISH_BAR1 && !s_finish_released);
  mock_line = 0x60;
  for (int i = 0; i < 3; ++i) assert(step(0, 0, &l, &r) == 0U);
  assert(s_finish_released);
  mock_line = 0x3FC;
  for (int i = 0; i < 2; ++i) assert(step(0, 0, &l, &r) == 0U);
  assert(step(0, 0, &l, &r) == 1U && s_state == TRK_DONE && l == 0 && r == 0);
}

static void test_search_never_stops(void) {
  float l, r;
  /* 状态切换保留上次方向，丢线立刻双向差速寻线 */
  setup(TRK_START_CLEAR); s_seg_mm = 899.5f; s_last_dir = 1;
  assert(step(30, 30, &l, &r) == 0U && s_state == TRK_ZIGZAG_1);
  assert(l == NAV_SEARCH_SPEED && r == -NAV_SEARCH_REV_SPEED);

  /* 扫线超时后自动反向来回扫，不停车 */
  for (unsigned int i = 0; i < NAV_SEARCH_SWEEP_MS / 2U; ++i) {
    assert(step(0, 0, &l, &r) == 0U);
  }
  assert(s_search_dir == -1 && l == -NAV_SEARCH_REV_SPEED && r == NAV_SEARCH_SPEED);

  /* 重新见到线恢复正常巡线 */
  mock_line = 0x60;
  assert(step(0, 0, &l, &r) == 0U && l > 0.0f && r > 0.0f && !s_search_active);
}

int main(void) {
  test_calibration_and_resilience();
  test_gap();
  test_turn_and_reverse_diff();
  test_finish();
  test_search_never_stops();
  puts("track navigation host tests: OK");
  return 0;
}
