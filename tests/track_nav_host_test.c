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
static uint32_t mock_bump_reads;
int16_t BMI270_gyro_z;
int8_t BMI270_last_rslt;

uint32_t HAL_GetTick(void) { return mock_tick; }
void HAL_Delay(uint32_t delay_ms) { mock_tick += delay_ms; }
void MUX_get_value(uint16_t *value) { *value = mock_line; }
int8_t dodo_BMI270_init(void) { return mock_bad_init ? -1 : 0; }
void dodo_BMI270_get_data(void) {
  BMI270_last_rslt = mock_bad_read ? -1 : 0;
  if (mock_reads < mock_bump_reads) {
    uint32_t idx = mock_reads++;
    BMI270_gyro_z = (idx < 25U) ? (((idx & 1U) != 0U) ? 1500 : -1500) : 1500;
  } else {
    BMI270_gyro_z = mock_motion ? ((mock_reads++ & 1U) ? 100 : -100) : mock_gyro;
  }
}
int8_t dodo_BMI270_get_gyro_z(int16_t *gyro_z) {
  if (mock_bad_read) {
    BMI270_last_rslt = -1;
    return -1;
  }
  BMI270_last_rslt = 0;
  if (mock_reads < mock_bump_reads) {
    uint32_t idx = mock_reads++;
    BMI270_gyro_z = (idx < 25U) ? (((idx & 1U) != 0U) ? 1500 : -1500) : 1500;
  } else {
    BMI270_gyro_z = mock_motion ? ((mock_reads++ & 1U) ? 100 : -100) : mock_gyro;
  }
  if (gyro_z != NULL) *gyro_z = BMI270_gyro_z;
  return 0;
}

static void setup(TrackState state) {
  mock_tick = 0; mock_line = 0; mock_gyro = 0; mock_motion = 0;
  mock_bad_read = mock_bad_init = 0; mock_reads = 0; mock_bump_reads = 0U;
  BMI270_gyro_z = BMI270_last_rslt = 0;
  s_fault = TRACK_FAULT_NONE; s_state = TRK_START_CLEAR; s_lap = 1U; s_search_active = 0;
  s_search_phase = 0U; s_recapture_ms = 0U;
  s_gyro_ok = 1U; s_gyro_fail_cnt = 0U; s_last_raw_rate = 0.0f; s_last_dir = 0;
  s_small_dir_cand = 0; s_small_dir_cnt = 0U; s_rate_pid_integral = 0.0f;
  s_yaw_deg = s_yaw_ref = s_yaw_west = s_yaw_hex300 = s_rate_lpf = s_gyro_bias = 0.0f;
  s_yaw_hex300_valid = 0U;
  s_prev_err = 0.0f; s_err_rate_lpf = 0.0f; s_has_prev_err = 0U; enter_state(state);
}

static uint8_t step(int32_t left, int32_t right, float *l, float *r) {
  mock_tick += 2U;
  return TrackNav_Step(left, right, 0.002f, l, r);
}

static uint8_t step_1ms(int32_t left, int32_t right, float *l, float *r) {
  mock_tick += 1U;
  return TrackNav_Step(left, right, 0.001f, l, r);
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

  /* 标定前期碰车/手晃脉冲窗口必须被剔除，不能污染静止零偏 */
  setup(TRK_START_CLEAR); mock_bump_reads = 75U; mock_gyro = 12; TrackNav_Start();
  assert(s_state == TRK_START_CLEAR && s_gyro_ok == 1U && fabsf(s_gyro_bias - 12.0f) < 1e-3f);

  /* 陀螺仪初始化失败或偶发读取错误不锁死停车，继续循迹 */
  setup(TRK_START_CLEAR); mock_bad_init = 1U; TrackNav_Start();
  assert(s_state == TRK_START_CLEAR && s_gyro_ok == 0U);
  setup(TRK_ZIGZAG_1); mock_line = 0x60; mock_bad_read = 1U;
  assert(step(10, 10, &l, &r) == 0U && s_state == TRK_ZIGZAG_1 && l > 0.0f && r > 0.0f);

  /* 连续读失败：允许保持 <=2 帧旧角速度，第 3 帧起清零避免死积分，恢复后清零计数，连续 25 帧降级 s_gyro_ok */
  setup(TRK_ZIGZAG_1); mock_line = 0x60; mock_gyro = -1640; /* +100 °/s */
  assert(step(10, 10, &l, &r) == 0U && fabsf(s_last_raw_rate - 100.0f) < 1e-2f);
  mock_bad_read = 1U;
  assert(step(10, 10, &l, &r) == 0U && s_gyro_fail_cnt == 1U && fabsf(s_last_raw_rate - 100.0f) < 1e-2f);
  assert(step(10, 10, &l, &r) == 0U && s_gyro_fail_cnt == 2U && fabsf(s_last_raw_rate - 100.0f) < 1e-2f);
  float yaw_before_drop = s_yaw_deg;
  assert(step(10, 10, &l, &r) == 0U && s_gyro_fail_cnt == 3U && s_last_raw_rate == 0.0f);
  assert(fabsf(s_yaw_deg - yaw_before_drop) < 1e-5f && s_gyro_ok == 1U);
  mock_bad_read = 0U; mock_gyro = 0;
  assert(step(10, 10, &l, &r) == 0U && s_gyro_fail_cnt == 0U && s_gyro_ok == 1U);
  mock_bad_read = 1U;
  for (int i = 0; i < (int)NAV_GYRO_DROP_MAX_FAILS; ++i) {
    assert(step(10, 10, &l, &r) == 0U && s_state == TRK_ZIGZAG_1);
  }
  assert(s_gyro_ok == 0U && l > 0.0f && r > 0.0f);
}

static void test_gap(void) {
  float l, r;
  /* TRK_APPROACH_BROKEN 入口前段保持 HEX 速，过两处 90° 弯道区间 (>=120..<720mm) 自动预减速防飞出，>=720mm 切到 BROKEN 对接断路 */
  float exp_broken_corner = (TRACK_ZIGZAG_SPEED < TRACK_BROKEN_SPEED)
                                ? TRACK_ZIGZAG_SPEED
                                : TRACK_BROKEN_SPEED;
  setup(TRK_APPROACH_BROKEN); s_seg_mm = 80.0f; mock_line = 0x60;
  assert(step(0, 0, &l, &r) == 0U && s_state == TRK_APPROACH_BROKEN &&
         fabsf(l - TRACK_HEX_SPEED) < 1e-1f && fabsf(r - TRACK_HEX_SPEED) < 1e-1f);
  s_seg_mm = 200.0f;
  assert(step(0, 0, &l, &r) == 0U && s_state == TRK_APPROACH_BROKEN &&
         fabsf(l - exp_broken_corner) < 1e-1f && fabsf(r - exp_broken_corner) < 1e-1f);
  s_seg_mm = 740.0f;
  assert(step(0, 0, &l, &r) == 0U && s_state == TRK_APPROACH_BROKEN &&
         fabsf(l - TRACK_BROKEN_SPEED) < 1e-1f && fabsf(r - TRACK_BROKEN_SPEED) < 1e-1f);

  /* 断路前未到 850 mm 的短暂丢线不能提前锁航向，仍走正常巡线定角寻线 */
  setup(TRK_APPROACH_BROKEN); s_seg_mm = 300.0f; s_last_dir = 1;
  assert(step(0, 0, &l, &r) == 0U && s_state == TRK_APPROACH_BROKEN && l > r);

  /* 到达 850 mm 后连续 3 帧丢线进入断路直行清除段 */
  setup(TRK_APPROACH_BROKEN); s_seg_mm = 860.0f;
  for (int i = 0; i < 3; ++i) {
    assert(step(0, 0, &l, &r) == 0U);
    assert(l == TRACK_BROKEN_SPEED && r == TRACK_BROKEN_SPEED);
  }
  assert(s_state == TRK_BROKEN_CLEAR);

  /* 断路斜插保持 NAV_BROKEN_OFFSET_DEG 直到 580 mm，不会在差 7 cm（550 mm）处提前归中 */
  setup(TRK_BROKEN_SEEK); s_seg_mm = 550.0f; s_yaw_deg = NAV_BROKEN_OFFSET_DEG;
  assert(step(0, 0, &l, &r) == 0U && l == TRACK_BROKEN_SPEED && r == TRACK_BROKEN_SPEED);

  /* 抓到线后进入巡线对齐并在满足最小距离后进入 PRE_T1 */
  setup(TRK_BROKEN_ALIGN); s_seg_mm = 61.0f; mock_line = 0x60;
  assert(step(0, 0, &l, &r) == 0U && s_state == TRK_TL_SQ_PRE_T1);

  /* TRK_BROKEN_ALIGN 若丢线超 150 ms 退回 TRK_BROKEN_SEEK，不能清零再盲跑 580 mm，而应立即回扫并允许重新抓线 */
  setup(TRK_BROKEN_ALIGN); s_yaw_west = 0.0f; s_yaw_deg = 20.0f; s_last_dir = -1;
  mock_line = 0x000;
  for (int i = 0; i < 75; ++i) {
    assert(step(0, 0, &l, &r) == 0U);
  }
  assert(s_state == TRK_BROKEN_SEEK && s_seg_mm > NAV_BROKEN_SEEK_MM && !s_search_active);
  assert(step(0, 0, &l, &r) == 0U && s_state == TRK_BROKEN_SEEK && r > l);
  mock_line = 0x060;
  assert(step(0, 0, &l, &r) == 0U && s_state == TRK_BROKEN_ALIGN);

  /* 左上 500 末端的 0xFE0 是向南 90° 左折角而非直行 T 口：保持循迹转南至 <=-65° 回正后再切入 TO_BL1 */
  setup(TRK_TL_SQ_PRE_T1); s_seg_mm = 480.0f; mock_line = 0xFE0;
  assert(step(0, 0, &l, &r) == 0U && s_state == TRK_TL_SQ_PRE_T1 && r > l);
  s_yaw_deg = -75.0f; mock_line = 0x060;
  assert(step(0, 0, &l, &r) == 0U && s_state == TRK_TL_SQ_TO_BL1);

  /* 进入 200 mm 转角判定区后若已转南并居中 (0x060)，不得在同帧先把南向覆盖成 s_yaw_west */
  setup(TRK_TL_SQ_PRE_T1); s_seg_mm = 220.0f; s_yaw_west = 0.0f; s_yaw_deg = -75.0f; mock_line = 0x060;
  assert(step(0, 0, &l, &r) == 0U && fabsf(s_yaw_west) < 1e-3f && s_state == TRK_TL_SQ_TO_BL1);
}

static void test_turn_and_reverse_diff(void) {
  float l, r;
  /* 左上 250x250：TO_BL1 遇 0xFFF 直行穿入 TO_TR2 -> 绕三边至 <=-335° 进入 TO_BL2 -> 遇 0xFFF 向西直行穿出至 LEFT_SQ1 */
  setup(TRK_TL_SQ_TO_BL1); s_seg_mm = 150.0f; s_yaw_deg = -90.0f; mock_line = 0xFFF;
  assert(step(0, 0, &l, &r) == 0U && s_state == TRK_TL_SQ_TO_BL1);
  assert(step(0, 0, &l, &r) == 0U && s_state == TRK_TL_SQ_T1_PASS);
  for (int i = 0; i < 4; ++i) assert(step(1000, 1000, &l, &r) == 0U);
  assert(s_state == TRK_TL_SQ_TO_TR2);

  s_seg_mm = 600.0f; s_yaw_deg = -355.0f; mock_line = 0x060;
  assert(step(0, 0, &l, &r) == 0U && s_state == TRK_TL_SQ_TO_BL2);
  s_seg_mm = 10.0f; mock_line = 0xFFF;
  assert(step(0, 0, &l, &r) == 0U && s_state == TRK_TL_SQ_TO_BL2);
  assert(step(0, 0, &l, &r) == 0U && s_state == TRK_TL_SQ_TO_BL2);
  s_seg_mm = 30.0f; mock_line = 0xFFF;
  assert(step(0, 0, &l, &r) == 0U && s_state == TRK_TL_SQ_TO_BL2);
  assert(step(0, 0, &l, &r) == 0U && s_state == TRK_TL_SQ_EXIT_W);
  for (int i = 0; i < 4; ++i) assert(step(1000, 1000, &l, &r) == 0U);
  assert(s_state == TRK_LEFT_SQ1_PRE);

  /* 左上 350 方框前置段仅 ~135 mm，>=75 mm 遇 0xFFF 应稳妥切入；左中 250 方框在 700 mm (>=650 mm, 270°) 应正常穿出 */
  setup(TRK_LEFT_SQ1_PRE); s_seg_mm = 100.0f; mock_line = 0xFFF;
  assert(step(0, 0, &l, &r) == 0U && s_state == TRK_LEFT_SQ1_PRE);
  assert(step(0, 0, &l, &r) == 0U && s_state == TRK_LEFT_SQ1_IN);
  setup(TRK_LEFT_SQ2_LOOP); mock_line = 0x060;
  assert(step(0, 0, &l, &r) == 0U && fabsf(l - TRACK_SQ_SPEED) < 1e-1f && fabsf(r - TRACK_SQ_SPEED) < 1e-1f);
  setup(TRK_LEFT_SQ3_LOOP); mock_line = 0x060;
  assert(step(0, 0, &l, &r) == 0U && fabsf(l - TRACK_SQ_SPEED) < 1e-1f && fabsf(r - TRACK_SQ_SPEED) < 1e-1f);
  setup(TRK_LEFT_SQ2_LOOP); s_seg_mm = 700.0f; s_yaw_seg = 270.0f; mock_line = 0xFFF;
  assert(step(0, 0, &l, &r) == 0U && s_state == TRK_LEFT_SQ2_LOOP);
  assert(step(0, 0, &l, &r) == 0U && s_state == TRK_LEFT_SQ2_OUT);

  /* 消抖：单帧路口不误切状态，连续两帧才切 */
  setup(TRK_APPROACH_HEX_R); s_seg_mm = 400.0f; mock_line = 0x3FC;
  assert(step(0, 0, &l, &r) == 0U && s_state == TRK_APPROACH_HEX_R);
  assert(step(0, 0, &l, &r) == 0U && s_state == TRK_HEX_R_ENTER);
}

static void test_square_and_hex_r_transitions(void) {
  float l, r;

  /* 单侧 90° 弯 0x1FE / 0x1FF / 0x0FF 绝不能在 ZIGZAG_1 提前误判成十字路口 */
  setup(TRK_ZIGZAG_1); s_seg_mm = 2375.0f;
  mock_line = 0x1FE;
  assert(step(0, 0, &l, &r) == 0U && s_state == TRK_ZIGZAG_1);
  mock_line = 0x1FF;
  assert(step(0, 0, &l, &r) == 0U && s_state == TRK_ZIGZAG_1);

  /* 分裂多簇 0x3DF (n=9, max_width=5) 不误判十字，连续宽横线 0x3FE (n=9, max_width=9) 正常切入 */
  setup(TRK_ZIGZAG_1); s_seg_mm = 2890.0f; mock_line = 0x3DF;
  assert(step(0, 0, &l, &r) == 0U && s_state == TRK_ZIGZAG_1);
  assert(step(0, 0, &l, &r) == 0U && s_state == TRK_ZIGZAG_1);
  mock_line = 0x3FE;
  assert(step(0, 0, &l, &r) == 0U && s_state == TRK_ZIGZAG_1);
  assert(step(0, 0, &l, &r) == 0U && s_state == TRK_SQ_RM_IN);

  /* 真正双向全覆盖十字路口 0xFFF 在 >=2600mm 连续两帧才切入 250 方框 */
  setup(TRK_ZIGZAG_1);
  s_seg_mm = 2890.0f; mock_line = 0xFFF;
  assert(step(0, 0, &l, &r) == 0U && s_state == TRK_ZIGZAG_1);
  assert(step(0, 0, &l, &r) == 0U && s_state == TRK_SQ_RM_IN);

  /* 250 方框绕圈第 3 角单侧直角弯 0x0FF 不提前切出；第 4 边偏斜踩入十字前缘 0x03F (>=620mm, >=210°) 不锁存 90° 猛甩；实测 728mm 遇 0xFFF 正常切出 */
  setup(TRK_SQ_RM_LOOP); s_seg_mm = 550.0f; s_yaw_seg = 180.0f; mock_line = 0x0FF;
  assert(step(0, 0, &l, &r) == 0U && s_state == TRK_SQ_RM_LOOP);
  assert(step(0, 0, &l, &r) == 0U && s_state == TRK_SQ_RM_LOOP);
  s_seg_mm = 723.0f; s_yaw_seg = 258.5f; mock_line = 0x03F;
  assert(step(0, 0, &l, &r) == 0U && s_state == TRK_SQ_RM_LOOP && s_corner_latch_ms == 0U);
  s_seg_mm = 728.0f; s_yaw_seg = 261.0f; mock_line = 0xFFF;
  assert(step(0, 0, &l, &r) == 0U && s_state == TRK_SQ_RM_LOOP);
  assert(step(0, 0, &l, &r) == 0U && s_state == TRK_SQ_RM_OUT);

  /* 420 段 120mm 起提前降速至 TRACK_HEX_SPEED 且禁用 90° 锁存；320mm 回流线 0xE60 与单线右偏 0x0F8/0x078 不误触发；458mm 右侧分叉 0x3F8->0x3B8 才进环 */
  setup(TRK_APPROACH_HEX_R); s_seg_mm = 130.0f; mock_line = 0x060;
  assert(step(0, 0, &l, &r) == 0U && s_state == TRK_APPROACH_HEX_R &&
         fabsf(l - TRACK_HEX_SPEED) < 1e-1f && fabsf(r - TRACK_HEX_SPEED) < 1e-1f);
  s_seg_mm = 240.0f;
  assert(step(0, 0, &l, &r) == 0U && s_state == TRK_APPROACH_HEX_R &&
         fabsf(l - TRACK_HEX_SPEED) < 1e-1f && fabsf(r - TRACK_HEX_SPEED) < 1e-1f);
  setup(TRK_APPROACH_HEX_R); s_seg_mm = 325.0f; mock_line = 0xE60;
  assert(step(0, 0, &l, &r) == 0U && s_state == TRK_APPROACH_HEX_R && s_corner_latch_ms == 0U);
  assert(step(0, 0, &l, &r) == 0U && s_state == TRK_APPROACH_HEX_R);
  s_seg_mm = 398.0f; mock_line = 0x0F8;
  assert(step(0, 0, &l, &r) == 0U && s_state == TRK_APPROACH_HEX_R);
  assert(step(0, 0, &l, &r) == 0U && s_state == TRK_APPROACH_HEX_R);
  s_seg_mm = 408.0f; mock_line = 0x078;
  assert(step(0, 0, &l, &r) == 0U && s_state == TRK_APPROACH_HEX_R);
  assert(step(0, 0, &l, &r) == 0U && s_state == TRK_APPROACH_HEX_R);
  s_seg_mm = 458.0f; mock_line = 0x3F8;
  assert(step(0, 0, &l, &r) == 0U && s_state == TRK_APPROACH_HEX_R);
  mock_line = 0x3B8;
  assert(step(0, 0, &l, &r) == 0U && s_state == TRK_HEX_R_ENTER);

  /* 八边形入口防错触发与防甩：1600mm 90° 弯降速且不误入十字；>=1700mm 靠近 1842mm 入口十字时禁用 90° 反甩锁存 (0xFC0)；真正十字稳切 OCT_IN */
  setup(TRK_APPROACH_OCT); s_seg_mm = 1500.0f; mock_line = 0x060;
  float exp_oct_corner = (TRACK_ZIGZAG_SPEED < TRACK_SQ_SPEED) ? TRACK_ZIGZAG_SPEED : TRACK_SQ_SPEED;
  assert(step(0, 0, &l, &r) == 0U && s_state == TRK_APPROACH_OCT &&
         fabsf(0.5f * (l + r) - exp_oct_corner) < 1e-1f);
  s_seg_mm = 1605.0f; mock_line = 0xFFF;
  assert(step(0, 0, &l, &r) == 0U && s_state == TRK_APPROACH_OCT);
  assert(step(0, 0, &l, &r) == 0U && s_state == TRK_APPROACH_OCT);
  s_seg_mm = 1837.0f; mock_line = 0xFC0;
  assert(step(0, 0, &l, &r) == 0U && s_state == TRK_APPROACH_OCT && s_corner_latch_ms == 0U);
  s_seg_mm = 1840.0f; mock_line = 0xFFF;
  assert(step(0, 0, &l, &r) == 0U && s_state == TRK_APPROACH_OCT);
  assert(step(0, 0, &l, &r) == 0U && s_state == TRK_OCT_IN);

  /* 八边形内第 7 角 (2110mm, yaw_seg=185°) 预减速且不得误提前退出；真实出口十字 (2352mm, yaw_seg=265°) 稳切 OCT_OUT */
  setup(TRK_OCT_LOOP); s_seg_mm = 2110.0f; s_yaw_seg = 185.0f; mock_line = 0x3FF;
  assert(step(0, 0, &l, &r) == 0U && s_state == TRK_OCT_LOOP);
  assert(step(0, 0, &l, &r) == 0U && s_state == TRK_OCT_LOOP);
  s_seg_mm = 2352.0f; s_yaw_seg = 265.0f; mock_line = 0xFFF;
  assert(step(0, 0, &l, &r) == 0U && s_state == TRK_OCT_LOOP);
  assert(step(0, 0, &l, &r) == 0U && s_state == TRK_OCT_OUT);

  /* 第 5 角单线宽块 0xFE0 (895mm, s_yaw_seg=-245°, 即使总角=-305°) 绝不得提前触发 HEX_R_EXIT；真实分叉 0x1CF->0x0FE 在 <=-275° 连贯切出 */
  setup(TRK_HEX_R_LOOP); s_yaw_ref = 170.0f; s_seg_mm = 895.0f; s_yaw_seg = -245.0f; s_yaw_deg = -135.0f;
  mock_line = 0xFE0;
  assert(step(0, 0, &l, &r) == 0U && s_state == TRK_HEX_R_LOOP && s_corner_latch_ms == 0U);
  assert(step(0, 0, &l, &r) == 0U && s_state == TRK_HEX_R_LOOP && s_corner_latch_ms == 0U);

  setup(TRK_HEX_R_LOOP); s_yaw_ref = 170.0f; s_seg_mm = 860.0f; s_yaw_seg = -275.0f; s_yaw_deg = -140.0f;
  mock_line = 0x1CF;
  assert(step(0, 0, &l, &r) == 0U && s_state == TRK_HEX_R_LOOP);
  mock_line = 0x0FE;
  assert(step(0, 0, &l, &r) == 0U && s_state == TRK_HEX_R_EXIT);

  /* HEX_R_EXIT 前 65mm 在 320mm 回流口完整左咬 (0x1CF->r>l)，65~195mm 经过 420mm 入环分叉 (0xF0C / <=-338° 宽单簇 0x7F0) 时改取右簇/右沿 (l>r) 直出 330 */
  s_seg_mm = 30.0f; s_yaw_deg = -142.0f; mock_line = 0x1CF;
  assert(step(0, 0, &l, &r) == 0U && s_state == TRK_HEX_R_EXIT && r > l);
  s_seg_mm = 70.0f; s_yaw_deg = -170.0f; mock_line = 0x7F0; /* hex_total_yaw=-340°<=-338°, 右沿 end_ch=7 (+3.0) -> 应强制取右沿 l>r */
  assert(step(0, 0, &l, &r) == 0U && s_state == TRK_HEX_R_EXIT && l > r);
  s_seg_mm = 115.0f; s_yaw_deg = -174.0f; mock_line = 0xF0C; /* 左 ch0..3 回环 vs 右 ch8..9 直出 330 */
  assert(step(0, 0, &l, &r) == 0U && s_state == TRK_HEX_R_EXIT && l > r);
  s_seg_mm = 185.0f; s_yaw_deg = -175.0f; mock_line = 0x060;
  assert(step(0, 0, &l, &r) == 0U && s_state == TRK_HEX_R_EXIT);
  s_seg_mm = 200.0f; s_yaw_deg = -175.0f; mock_line = 0x1CF;
  assert(step(0, 0, &l, &r) == 0U && s_state == TRK_HEX_R_EXIT);
  mock_line = 0xFE0;
  assert(step(0, 0, &l, &r) == 0U && s_state == TRK_APPROACH_OCT && s_corner_latch_ms == 0U);

  /* 右中六边形：单灯坏点假双簇 0x005 (间隔仅 1 路暗管) 不得误触发出环；320mm 双簇 0x1CF 在 830mm, s_yaw_seg=-265° 可稳定切出；若偏晚触发 (s_yaw_seg<=-315°) 自动跳过左咬直接右避让 */
  setup(TRK_HEX_R_LOOP); s_yaw_ref = 170.0f; s_seg_mm = 830.0f; s_yaw_seg = -265.0f; s_yaw_deg = -140.0f;
  mock_line = 0x005;
  assert(step(0, 0, &l, &r) == 0U && s_state == TRK_HEX_R_LOOP);
  assert(step(0, 0, &l, &r) == 0U && s_state == TRK_HEX_R_LOOP);
  mock_line = 0x1CF;
  assert(step(0, 0, &l, &r) == 0U && s_state == TRK_HEX_R_LOOP);
  assert(step(0, 0, &l, &r) == 0U && s_state == TRK_HEX_R_EXIT);

  setup(TRK_HEX_R_LOOP); s_yaw_ref = 170.0f; s_seg_mm = 890.0f; s_yaw_seg = -320.0f; s_yaw_deg = -165.0f;
  mock_line = 0xF0C;
  assert(step(0, 0, &l, &r) == 0U && s_state == TRK_HEX_R_LOOP);
  assert(step(0, 0, &l, &r) == 0U && s_state == TRK_HEX_R_EXIT && s_seg_mm >= 65.0f);
  assert(step(0, 0, &l, &r) == 0U && s_state == TRK_HEX_R_EXIT && l > r);

  /* 无陀螺仪降级模式下走到 900mm 遇分叉也可出环 */
  setup(TRK_HEX_R_LOOP); s_gyro_ok = 0U; s_seg_mm = 910.0f; mock_line = 0x1CF;
  assert(step(0, 0, &l, &r) == 0U && s_state == TRK_HEX_R_LOOP);
  assert(step(0, 0, &l, &r) == 0U && s_state == TRK_HEX_R_EXIT);

  /* -345° 兜底：若仍在 420mm 双簇分叉、全丢线 (0x000) 或未过 980mm，切进 HEX_R_EXIT 右咬保护段防重进第二圈；仅见单线且 >=980mm 才直切 APPROACH_OCT */
  setup(TRK_HEX_R_LOOP); s_seg_mm = 900.0f; s_yaw_seg = -350.0f; mock_line = 0xF0C;
  assert(step(0, 0, &l, &r) == 0U && s_state == TRK_HEX_R_EXIT && s_seg_mm >= 65.0f);
  assert(step(0, 0, &l, &r) == 0U && s_state == TRK_HEX_R_EXIT && l > r);

  setup(TRK_HEX_R_LOOP); s_seg_mm = 1000.0f; s_yaw_seg = -350.0f; mock_line = 0x000;
  assert(step(0, 0, &l, &r) == 0U && s_state == TRK_HEX_R_EXIT && s_seg_mm >= 65.0f);

  setup(TRK_HEX_R_LOOP); s_seg_mm = 1000.0f; s_yaw_seg = -350.0f; mock_line = 0x0FF;
  assert(step(0, 0, &l, &r) == 0U && s_state == TRK_APPROACH_OCT && l > r);

  /* 分叉首帧恰逢 1220 mm 里程兜底且转角仅 -290° (> -315°) 时，不能跳过前 65 mm 左咬而错切右选线 */
  setup(TRK_HEX_R_LOOP); s_seg_mm = 1220.0f; s_yaw_seg = -290.0f; mock_line = 0x1CF;
  assert(step(0, 0, &l, &r) == 0U && s_state == TRK_HEX_R_EXIT && s_seg_mm < 65.0f);
  assert(step(0, 0, &l, &r) == 0U && s_state == TRK_HEX_R_EXIT && r > l);

  /* 左下 200 六边形：单线弯 0x03C 不误触发；60° 交叉口 0x0E7 直行切入 -> 过滤第 5 角 0x7E0 -> 回原点 0x0E7 直行穿出 */
  setup(TRK_APPROACH_HEX_BL); s_seg_mm = 600.0f; mock_line = 0x03C;
  assert(step(0, 0, &l, &r) == 0U && s_state == TRK_APPROACH_HEX_BL);
  assert(step(0, 0, &l, &r) == 0U && s_state == TRK_APPROACH_HEX_BL);
  mock_line = 0x0E7;
  assert(step(0, 0, &l, &r) == 0U && s_state == TRK_APPROACH_HEX_BL);
  assert(step(0, 0, &l, &r) == 0U && s_state == TRK_HEX_BL_IN);
  assert(step(0, 0, &l, &r) == 0U && l == TRACK_CROSS_SPEED && r == TRACK_CROSS_SPEED);
  for (int i = 0; i < 4; ++i) assert(step(1000, 1000, &l, &r) == 0U);
  assert(s_state == TRK_HEX_BL_LOOP);

  s_seg_mm = 960.0f; s_yaw_seg = 240.0f; mock_line = 0x7E0;
  assert(step(0, 0, &l, &r) == 0U && s_state == TRK_HEX_BL_LOOP);
  assert(step(0, 0, &l, &r) == 0U && s_state == TRK_HEX_BL_LOOP);
  s_seg_mm = 1015.0f; s_yaw_seg = 300.0f; mock_line = 0x0E7;
  assert(step(0, 0, &l, &r) == 0U && s_state == TRK_HEX_BL_LOOP);
  assert(step(0, 0, &l, &r) == 0U && s_state == TRK_HEX_BL_EXIT);
  for (int i = 0; i < 4; ++i) assert(step(1000, 1000, &l, &r) == 0U);
  assert(s_state == TRK_APPROACH_HEX_MID);

  /* 实车日志复现：TRK_HEX_BL_LOOP 第 5 角处 ch10 坏点产生 1-bit 假双簇 0x005 / 0x00D (mm=1094, yaw=273°) 绝不误出环；真实交叉口 0x0E5(即使 ch10 坏) 与 0x0E7 可正常出环 */
  setup(TRK_HEX_BL_LOOP); s_seg_mm = 1094.0f; s_yaw_seg = 273.0f;
  mock_line = 0x005;
  assert(step(0, 0, &l, &r) == 0U && s_state == TRK_HEX_BL_LOOP);
  assert(step(0, 0, &l, &r) == 0U && s_state == TRK_HEX_BL_LOOP);
  mock_line = 0x00D;
  assert(step(0, 0, &l, &r) == 0U && s_state == TRK_HEX_BL_LOOP);
  assert(step(0, 0, &l, &r) == 0U && s_state == TRK_HEX_BL_LOOP);
  mock_line = 0x0E5;
  assert(step(0, 0, &l, &r) == 0U && s_state == TRK_HEX_BL_LOOP);
  assert(step(0, 0, &l, &r) == 0U && s_state == TRK_HEX_BL_EXIT);

  /* 双六边形 300+200：内切过弯 0x0EF 在 ~380mm (<= -35°) 直行切入小 200 六边形 -> 过滤第 5 角 0x7E0 -> 1015mm 处 >=270° 直行穿回 300_L */
  setup(TRK_HEX_300_R); s_seg_mm = 380.0f; s_yaw_seg = -55.0f; mock_line = 0x0EF;
  assert(step(0, 0, &l, &r) == 0U && s_state == TRK_HEX_300_R);
  assert(step(0, 0, &l, &r) == 0U && s_state == TRK_HEX_200_IN);
  assert(step(0, 0, &l, &r) == 0U && l == TRACK_CROSS_SPEED && r == TRACK_CROSS_SPEED);
  for (int i = 0; i < 4; ++i) assert(step(1000, 1000, &l, &r) == 0U);
  assert(s_state == TRK_HEX_200_LOOP);
  s_seg_mm = 960.0f; s_yaw_seg = 240.0f; mock_line = 0x7E0;
  assert(step(0, 0, &l, &r) == 0U && s_state == TRK_HEX_200_LOOP);
  assert(step(0, 0, &l, &r) == 0U && s_state == TRK_HEX_200_LOOP);
  s_seg_mm = 1015.0f; s_yaw_seg = 300.0f; mock_line = 0x0EF;
  assert(step(0, 0, &l, &r) == 0U && s_state == TRK_HEX_200_LOOP);
  assert(step(0, 0, &l, &r) == 0U && s_state == TRK_HEX_200_CROSS);
  for (int i = 0; i < 4; ++i) assert(step(1000, 1000, &l, &r) == 0U);
  assert(s_state == TRK_HEX_300_L);

  /* 双环岛入口 590 东向基准航向采集，以及新圈重置有效性防跨圈错比 */
  setup(TRK_APPROACH_HEX_MID); s_seg_mm = 850.0f; s_yaw_deg = 713.3f; mock_line = 0x060;
  assert(step(0, 0, &l, &r) == 0U && s_state == TRK_APPROACH_HEX_MID &&
         s_yaw_hex300_valid == 1U && fabsf(s_yaw_hex300 - 713.3f) < 1e-2f);
  s_seg_mm = 1103.0f; mock_line = 0x860;
  assert(step(0, 0, &l, &r) == 0U && s_state == TRK_APPROACH_HEX_MID);
  assert(step(0, 0, &l, &r) == 0U && s_state == TRK_HEX_300_ENTER && s_yaw_hex300_valid == 1U);
  enter_state(TRK_START_CLEAR);
  assert(s_yaw_hex300_valid == 0U);

  /* 实车日志复现：TRK_HEX_300_L 左下第 3 角 (mm=948, yaw_seg=-119.5°, 0xFC0) 绝不能提前触发出环 */
  setup(TRK_HEX_300_L); s_yaw_hex300 = 713.2f; s_yaw_hex300_valid = 1U;
  s_seg_mm = 948.0f; s_yaw_seg = -119.5f; s_yaw_deg = 801.6f; mock_line = 0xFC0;
  for (int i = 0; i < 4; ++i) {
    assert(step(0, 0, &l, &r) == 0U && s_state == TRK_HEX_300_L && s_corner_latch_ms == 0U);
  }

  /* 真正底切点 (mm=1224, yaw_seg=-168.9°, 0x1C3) 才切入 TRK_HEX_300_EXIT */
  s_seg_mm = 1224.0f; s_yaw_seg = -168.9f; s_yaw_deg = 752.2f; mock_line = 0x1C3;
  assert(step(0, 0, &l, &r) == 0U && s_state == TRK_HEX_300_L);
  assert(step(0, 0, &l, &r) == 0U && s_state == TRK_HEX_300_EXIT && s_seg_mm < 65.0f);

  /* HEX_300_EXIT 两段防重入：前 65mm 左咬忽略右侧 590 (0x1C3->r>l)；过底顶点后右咬忽略左侧 300 东南重入边 (0x3F0 / 0xE18 -> l>r) */
  s_seg_mm = 20.0f; s_yaw_deg = 750.0f; mock_line = 0x1C3;
  assert(step(0, 0, &l, &r) == 0U && s_state == TRK_HEX_300_EXIT && r > l);
  s_seg_mm = 90.0f; s_yaw_deg = 720.0f; mock_line = 0x3F0;
  assert(step(0, 0, &l, &r) == 0U && s_state == TRK_HEX_300_EXIT && l > r && s_corner_latch_ms == 0U);
  mock_line = 0xE18;
  assert(step(0, 0, &l, &r) == 0U && s_state == TRK_HEX_300_EXIT && l > r);
  s_seg_mm = 190.0f; mock_line = 0x060;
  assert(step(0, 0, &l, &r) == 0U && s_state == TRK_HEX_300_EXIT);
  s_seg_mm = 210.0f; mock_line = 0xE60;
  assert(step(0, 0, &l, &r) == 0U && s_state == TRK_HEX_300_EXIT);
  s_seg_mm = 205.0f; s_yaw_deg = 713.3f; mock_line = 0x060;
  assert(step(0, 0, &l, &r) == 0U && s_state == TRK_ZIGZAG_2);

  /* 出环切入 ZIGZAG_2 前 200mm 内禁用 90° 猛反甩锁存，200mm 后恢复 */
  s_seg_mm = 50.0f; mock_line = 0xFC0;
  assert(step(0, 0, &l, &r) == 0U && s_state == TRK_ZIGZAG_2 && s_corner_latch_ms == 0U);
  s_seg_mm = 400.0f; mock_line = 0x07F;
  assert(step(0, 0, &l, &r) == 0U && s_state == TRK_ZIGZAG_2 && s_corner_dir == 1 && s_corner_latch_ms > 0U);

  /* TRK_HEX_300_L 偏晚触发 (rel_yaw<=12°) 跳过左咬直接右咬保护；航向/里程兜底防重入第二圈 */
  setup(TRK_HEX_300_L); s_yaw_hex300 = 713.2f; s_yaw_hex300_valid = 1U;
  s_seg_mm = 1260.0f; s_yaw_seg = -202.0f; s_yaw_deg = 724.0f; mock_line = 0xE18;
  assert(step(0, 0, &l, &r) == 0U && s_state == TRK_HEX_300_L);
  assert(step(0, 0, &l, &r) == 0U && s_state == TRK_HEX_300_EXIT && s_seg_mm >= 65.0f);
  assert(step(0, 0, &l, &r) == 0U && s_state == TRK_HEX_300_EXIT && l > r);

  setup(TRK_HEX_300_L); s_yaw_hex300 = 713.2f; s_yaw_hex300_valid = 1U;
  s_seg_mm = 1200.0f; s_yaw_seg = -205.0f; s_yaw_deg = 720.0f; mock_line = 0x0E0;
  assert(step(0, 0, &l, &r) == 0U && s_state == TRK_HEX_300_EXIT && s_seg_mm >= 65.0f);

  setup(TRK_HEX_300_L); s_yaw_hex300 = 713.2f; s_yaw_hex300_valid = 1U;
  s_seg_mm = 1400.0f; s_yaw_seg = -210.0f; s_yaw_deg = 714.0f; mock_line = 0x000;
  assert(step(0, 0, &l, &r) == 0U && s_state == TRK_HEX_300_EXIT && s_seg_mm >= 65.0f);

  setup(TRK_HEX_300_L); s_yaw_hex300 = 713.2f; s_yaw_hex300_valid = 1U;
  s_seg_mm = 1400.0f; s_yaw_seg = -210.0f; s_yaw_deg = 714.0f; mock_line = 0x060;
  assert(step(0, 0, &l, &r) == 0U && s_state == TRK_ZIGZAG_2);

  /* TRK_HEX_300_L 分叉首帧恰逢 1450 mm 兜底且 rel_yaw=38.8° (> 12°) 时，同样先保留前 65 mm 左咬 */
  setup(TRK_HEX_300_L); s_yaw_hex300 = 713.2f; s_yaw_hex300_valid = 1U;
  s_seg_mm = 1450.0f; s_yaw_seg = -168.9f; s_yaw_deg = 752.0f; mock_line = 0x1C3;
  assert(step(0, 0, &l, &r) == 0U && s_state == TRK_HEX_300_EXIT && s_seg_mm < 65.0f);
  assert(step(0, 0, &l, &r) == 0U && s_state == TRK_HEX_300_EXIT && r > l);

  /* 无陀螺仪降级模式下在 950mm 过滤第 3 角，在 1150mm 遇分叉正常出环 */
  setup(TRK_HEX_300_L); s_gyro_ok = 0U; s_seg_mm = 950.0f; mock_line = 0xFC0;
  assert(step(0, 0, &l, &r) == 0U && s_state == TRK_HEX_300_L);
  assert(step(0, 0, &l, &r) == 0U && s_state == TRK_HEX_300_L);
  s_seg_mm = 1150.0f; mock_line = 0x1C3;
  assert(step(0, 0, &l, &r) == 0U && s_state == TRK_HEX_300_L);
  assert(step(0, 0, &l, &r) == 0U && s_state == TRK_HEX_300_EXIT);
}

static void test_finish(void) {
  float l, r;
  /* TRK_ZIGZAG_2 过最后 90° 弯 (2850~3150 mm) 自动降至 TRACK_SQ_SPEED 且尚未臂合；>=3180 mm 才切入 TRK_FINISH_ARMED */
  setup(TRK_ZIGZAG_2); s_seg_mm = 2950.0f; mock_line = 0x060;
  assert(step(0, 0, &l, &r) == 0U && s_state == TRK_ZIGZAG_2 &&
         fabsf(0.5f * (l + r) - TRACK_SQ_SPEED) < 1e-1f);
  s_seg_mm = 3100.0f; mock_line = 0x0FF;
  for (int i = 0; i < 6; ++i) assert(step(0, 0, &l, &r) == 0U);
  assert(s_state == TRK_ZIGZAG_2);
  s_seg_mm = 3180.0f; mock_line = 0x060;
  assert(step(0, 0, &l, &r) == 0U && s_state == TRK_FINISH_ARMED);

  setup(TRK_FINISH_ARMED);
  mock_line = 0x1F8; /* 6 路普通白线 */
  for (int i = 0; i < 10; ++i) assert(step(0, 0, &l, &r) == 0U);
  mock_line = 0x0FF; /* 单侧直角弯 ch4..11 (n=8) */
  for (int i = 0; i < 6; ++i) assert(step(0, 0, &l, &r) == 0U);
  mock_line = 0xFF0; /* 单侧直角弯 ch0..7 (n=8) */
  for (int i = 0; i < 6; ++i) assert(step(0, 0, &l, &r) == 0U);
  mock_line = 0x1FF; /* 单侧直角弯 ch3..11 (start_ch=3 > 2) */
  for (int i = 0; i < 6; ++i) assert(step(0, 0, &l, &r) == 0U);
  mock_line = 0xFF8; /* 单侧直角弯 ch0..8 (end_ch=8 < 9) */
  for (int i = 0; i < 6; ++i) assert(step(0, 0, &l, &r) == 0U);
  mock_line = 0x3DF; /* 分裂双簇 n=9 但 max_width=5 < 8 */
  for (int i = 0; i < 6; ++i) assert(step(0, 0, &l, &r) == 0U);
  assert(s_state == TRK_FINISH_ARMED);

  /* 正中 8 灯横线 (0x3FC, ch2..9) 满足双向边界，可稳定识别为终点横线 */
  mock_line = 0x3FC;
  assert(step(0, 0, &l, &r) == 0U && s_state == TRK_FINISH_ARMED);
  assert(step(0, 0, &l, &r) == 0U && s_state == TRK_FINISH_BAR1 && s_lap == 1U);
  setup(TRK_FINISH_ARMED);

  /* 急弯高速旋转期间 (300 °/s > 220 °/s) 即使宽线 0xFFF 也不判定为终点横线 */
  mock_line = 0x060; mock_gyro = -4920; /* +300 °/s 预热滤波角速度 */
  for (int i = 0; i < 6; ++i) assert(step(0, 0, &l, &r) == 0U);
  mock_line = 0xFFF;
  for (int i = 0; i < 6; ++i) assert(step(0, 0, &l, &r) == 0U);
  assert(s_state == TRK_FINISH_ARMED);

  /* 第 1 圈出弯余振回正 (~150 °/s) 下碾过偏边 0x3FF -> 0xFFF，2 帧 (4 ms) 即稳定触发 BAR1：保持 TRACK_BASE_SPEED 冲过第 2 条线 */
  mock_gyro = -2460; s_rate_lpf = 150.0f; s_yaw_deg = 0.0f;
  mock_line = 0x3FF;
  assert(step(0, 0, &l, &r) == 0U && s_state == TRK_FINISH_ARMED);
  mock_line = 0xFFF;
  assert(step(0, 0, &l, &r) == 0U && s_state == TRK_FINISH_BAR1 && s_lap == 1U);
  mock_gyro = 0; s_rate_lpf = 0.0f; s_yaw_deg = 0.0f;
  for (int i = 0; i < 20; ++i) assert(step(1200, 1200, &l, &r) == 0U);
  assert(s_state == TRK_FINISH_BAR1 && !s_finish_released);
  mock_line = 0x60;
  assert(step(0, 0, &l, &r) == 0U && !s_finish_released);
  assert(step(0, 0, &l, &r) == 0U && s_finish_released);
  assert(fabsf(0.5f * (l + r) - TRACK_BASE_SPEED) < 1e-1f && fabsf(l - r) < 20.0f);

  /* 第 1 圈到达第二条横线前若再遇到单侧 90° 弯 0x1FF，仍保持寻线不提前切圈 */
  s_seg_mm = 340.0f; mock_line = 0x1FF;
  for (int i = 0; i < 6; ++i) assert(step(0, 0, &l, &r) == 0U);
  assert(s_state == TRK_FINISH_BAR1);

  /* 第 1 圈过第一条横线后保持全速 TRACK_BASE_SPEED；碾过第二条终点横线 0xFFF 后切回 TRK_START_CLEAR 开始第 2 圈 */
  s_seg_mm = 340.0f; mock_line = 0x060;
  assert(step(0, 0, &l, &r) == 0U && s_state == TRK_FINISH_BAR1 &&
         fabsf(0.5f * (l + r) - TRACK_BASE_SPEED) < 1e-1f);
  mock_line = 0xFFF;
  assert(step(0, 0, &l, &r) == 0U && s_state == TRK_FINISH_BAR1);
  assert(step(0, 0, &l, &r) == 0U && s_state == TRK_START_CLEAR && s_lap == 2U &&
         fabsf(s_seg_mm - NAV_LAP2_START_SEG_MM) < 1e-1f);

  /* 第 1 圈若漏检第二条白线，600 mm 尚未误触发兜底；越过 NAV_FINISH_LAP1_BYPASS_MM (700 mm) 后自动切入第 2 圈 */
  setup(TRK_FINISH_BAR1); s_lap = 1U; s_finish_released = 1U; s_seg_mm = 600.0f; mock_line = 0x060;
  assert(step(0, 0, &l, &r) == 0U && s_state == TRK_FINISH_BAR1 && s_lap == 1U);
  s_seg_mm = 700.0f;
  assert(step(0, 0, &l, &r) == 0U && s_state == TRK_START_CLEAR && s_lap == 2U && s_seg_mm > 450.0f);

  /* 第 2 圈若漏检第二条白线，600 mm 仍在等待白线；越过 NAV_FINISH_LAP2_STOP_MM (680 mm) 后兜底刹停防跑第三圈 */
  setup(TRK_FINISH_BAR1); s_lap = 2U; s_finish_released = 1U; s_seg_mm = 600.0f; mock_line = 0x060;
  assert(step(0, 0, &l, &r) == 0U && s_state == TRK_FINISH_BAR1 && s_lap == 2U);
  s_seg_mm = 680.0f;
  assert(step(0, 0, &l, &r) == 0U && s_state == TRK_DONE);

  /* 第 2 圈（停车圈）碾过第一条终点横线 (0x7FE) 进入 BAR1 后减速 30%（TRACK_BASE_SPEED * 0.70f），碾过第二条线 0xFFF 才完赛刹车停车 */
  setup(TRK_FINISH_ARMED); s_lap = 2U;
  mock_line = 0x7FE;
  assert(step(0, 0, &l, &r) == 0U && s_state == TRK_FINISH_ARMED);
  assert(step(0, 0, &l, &r) == 0U && s_state == TRK_FINISH_BAR1 && s_lap == 2U);
  mock_line = 0x060;
  assert(step(0, 0, &l, &r) == 0U && !s_finish_released);
  assert(step(0, 0, &l, &r) == 0U && s_finish_released);
  assert(fabsf(l - 0.70f * TRACK_BASE_SPEED) < 1e-1f &&
         fabsf(r - 0.70f * TRACK_BASE_SPEED) < 1e-1f);
  s_seg_mm = 340.0f;
  mock_line = 0xFFF;
  assert(step(0, 0, &l, &r) == 0U && s_state == TRK_FINISH_BAR1);
  assert(step(0, 0, &l, &r) == 0U && s_state == TRK_DONE && l == 0.0f && r == 0.0f);
  for (int i = 0; i < 2; ++i) assert(step(0, 0, &l, &r) == 0U && s_state == TRK_DONE);
  assert(step(0, 0, &l, &r) == 1U && s_state == TRK_DONE && l == 0 && r == 0);

  /* 带速度踩到第 2 圈第二条终点横线时：高速阶段反接制动 -> 瞬时抱死/量化低速不提前断电 -> 回弹/重新滚快仍继续锁停或反接 -> 连续 4 帧停稳后彻底断电 (1U) */
  setup(TRK_FINISH_BAR1); s_lap = 2U; s_finish_released = 1U; s_seg_mm = 340.0f; mock_line = 0xFFF;
  assert(step(36, 36, &l, &r) == 0U && s_state == TRK_FINISH_BAR1);
  assert(step(36, 36, &l, &r) == 0U && s_state == TRK_DONE &&
         l == -NAV_FINISH_BRAKE_REV_SPEED && r == -NAV_FINISH_BRAKE_REV_SPEED);
  assert(step(3, 3, &l, &r) == 0U && s_state == TRK_DONE && l == 0.0f && r == 0.0f && s_finish_stop_ms == 0U);
  assert(step(1, 1, &l, &r) == 0U && s_state == TRK_DONE && l == 0.0f && r == 0.0f && !s_finish_brake_done);
  assert(step(1, 1, &l, &r) == 0U && s_state == TRK_DONE && l == 0.0f && r == 0.0f && !s_finish_brake_done);
  assert(step(10, 10, &l, &r) == 0U && s_state == TRK_DONE &&
         l == -NAV_FINISH_BRAKE_REV_SPEED && r == -NAV_FINISH_BRAKE_REV_SPEED && s_finish_stop_ms == 0U);
  assert(step(-5, -5, &l, &r) == 0U && s_state == TRK_DONE && l == 0.0f && r == 0.0f && s_finish_stop_ms == 0U);
  for (int i = 0; i < 3; ++i) {
    assert(step(1, 1, &l, &r) == 0U && s_state == TRK_DONE && l == 0.0f && r == 0.0f && !s_finish_brake_done);
  }
  assert(step(1, 1, &l, &r) == 1U && s_state == TRK_DONE && l == 0.0f && r == 0.0f && s_finish_brake_done == 1U);

  /* 主动制动超时兜底（如架空或编码器未完全归零）不超过 NAV_FINISH_BRAKE_MAX_MS 必定断电 */
  setup(TRK_DONE);
  for (int i = 0; i < (int)(NAV_FINISH_BRAKE_MAX_MS / 2U) - 1; ++i) {
    assert(step(36, 36, &l, &r) == 0U);
  }
  assert(step(36, 36, &l, &r) == 1U && l == 0.0f && r == 0.0f);
}

static void test_search_never_stops(void) {
  float l, r;
  /* 明显偏左 (-5.0) 后中心单灯微晃 (+1.0) 不应覆盖真实的左偏方向；连续 4 帧同向小偏差才会更新 */
  setup(TRK_ZIGZAG_1);
  mock_line = 0x100; /* ch 3 -> -5.0 */
  assert(step(10, 10, &l, &r) == 0U && s_last_dir == -1);
  mock_line = 0x020; /* ch 6 -> +1.0 */
  assert(step(10, 10, &l, &r) == 0U && s_last_dir == -1);
  assert(step(10, 10, &l, &r) == 0U && s_last_dir == -1);
  assert(step(10, 10, &l, &r) == 0U && s_last_dir == -1);
  assert(step(10, 10, &l, &r) == 0U && s_last_dir == 1);

  /* 状态切换保留上次方向，丢线后由陀螺仪转向 PID 按上次方向(+1)向 +135° 寻线 */
  setup(TRK_START_CLEAR); s_seg_mm = 899.5f; s_last_dir = 1;
  assert(step(30, 30, &l, &r) == 0U && s_state == TRK_ZIGZAG_1);
  assert(s_search_dir == 1 && l > r && l > 0.0f);

  /* 接近 +135° 前持续保持 +1 方向且自动减速，防止甩过 180° 掉头 */
  s_yaw_deg = 120.0f;
  assert(step(0, 0, &l, &r) == 0U && s_search_dir == 1);

  /* 到达 +135° 目标边界（rem <= 4°）仍未见线时，向对称 -135° 回扫而不超过 180° */
  s_yaw_deg = 132.0f;
  assert(step(0, 0, &l, &r) == 0U && s_search_dir == -1 && r > l);

  /* 重新见到线恢复正常巡线并开启稳姿阻尼窗口 */
  mock_line = 0x60;
  assert(step(0, 0, &l, &r) == 0U && l > 0.0f && r > 0.0f && !s_search_active && s_recapture_ms > 0U);

  /* 抓线稳姿前期，短暂反向越过中心 (+3.0) 不应立刻翻转 s_last_dir 引起左右摆头；高速自转时压低前进基速 */
  setup(TRK_APPROACH_BROKEN); s_last_dir = -1; s_search_active = 1U; s_search_dir = -1;
  mock_gyro = 4920; /* -300 °/s 正在左转寻线 */
  for (int i = 0; i < 6; ++i) {
    mock_line = 0x000;
    assert(step(0, 0, &l, &r) == 0U);
  }
  mock_line = 0x008; /* ch 8 -> +5.0 (< 6.0)，仅 2 帧反向过冲不应立即把 -1 翻成 +1，且保持稳姿限速 */
  assert(step(0, 0, &l, &r) == 0U && s_recapture_ms > (NAV_RECAPTURE_MS / 2U));
  assert(0.5f * (l + r) <= NAV_RECAPTURE_SPEED + 1.0f);
  assert(step(0, 0, &l, &r) == 0U && s_last_dir == -1);
  /* 持续反向达到 4 帧或直接偏到外沿 (+7.0) 才会确认换向 */
  mock_line = 0x002; /* ch 10 -> +9.0 (>= 6.0) */
  assert(step(0, 0, &l, &r) == 0U && s_last_dir == 1);

  /* 重抓自适应提前释放：大外偏 (0x001) 时保留稳姿限速不提前解锁；回正居中 (0x060) 后若 >=20ms 则提前清零恢复全速 */
  s_recapture_ms = NAV_RECAPTURE_MS - 20U;
  mock_line = 0x001;
  assert(step(0, 0, &l, &r) == 0U && s_recapture_ms > 0U);
  mock_line = 0x060; mock_gyro = 0; s_rate_lpf = 0.0f;
  assert(step(0, 0, &l, &r) == 0U && s_recapture_ms == 0U);
}

static void test_follow_line_cascade(void) {
  float l0, r0, l_fast, r_fast, l_exit, r_exit, l_curve, r_curve;

  /* 极端外沿偏差 (+11.0 >= 8.0) 允许更低降速下限 (min_scale=0.48) 换取强内轮反转反甩；实测角速度超限时差速收缩 */
  setup(TRK_SQ_RM_LOOP);
  mock_line = 0x001; /* ch 11 -> +11.0 */
  assert(step(0, 0, &l0, &r0) == 0U && l0 > r0 && r0 >= -NAV_FOLLOW_MAX_REV_SPEED);
  assert(0.5f * (l0 + r0) < 0.60f * TRACK_HEX_SPEED && r0 < -8000.0f);

  mock_gyro = -9840; /* 对应 +600 °/s 顺时针旋转 (> 495 °/s) */
  for (int i = 0; i < 8; ++i) {
    assert(step(0, 0, &l_fast, &r_fast) == 0U);
  }
  assert(s_rate_lpf > NAV_FOLLOW_MAX_RATE_DPS);
  assert((l_fast - r_fast) < (l0 - r0));
  assert((l_fast - r_fast) < 2.0f * s_rate_pid_gains[TRK_SQ_RM_LOOP].kff * NAV_FOLLOW_MAX_RATE_DPS);

  /* 出弯刚刚压回中心线 (err = 0) 而车身仍在顺时针快转时，内环主动反打制动并压低前进基速，同时禁止外轮反接倒转消除顿挫 */
  mock_line = 0x060; /* ch 5,6 -> 0.0 */
  assert(step(0, 0, &l_exit, &r_exit) == 0U);
  assert(r_exit > l_exit && l_exit >= 0.0f);
  assert(0.5f * (l_exit + r_exit) < TRACK_BASE_SPEED);

  /* 非最外沿常规弯道急变 (0x060 -> 0x004, err=+7.0 < 8.0) 保持 0.68f 平顺巡弯下限 */
  setup(TRK_SQ_RM_LOOP);
  mock_line = 0x060;
  assert(step(0, 0, &l_curve, &r_curve) == 0U);
  mock_line = 0x004;
  assert(step(0, 0, &l_curve, &r_curve) == 0U &&
         fabsf(0.5f * (l_curve + r_curve) - 0.68f * TRACK_HEX_SPEED) < 1.0f);

  /* 锯齿段目标角速度上限 (290 °/s) 比普通弯道 (320 °/s) 更收敛 */
  setup(TRK_ZIGZAG_1); s_seg_mm = 100.0f; mock_line = 0x001;
  float l_zig, r_zig, l_norm, r_norm;
  assert(step(0, 0, &l_zig, &r_zig) == 0U && l_zig > r_zig);
  setup(TRK_ZIGZAG_1); s_seg_mm = 1600.0f; mock_line = 0x001;
  assert(step(0, 0, &l_norm, &r_norm) == 0U && (l_zig - r_zig) < (l_norm - r_norm));

  /* 高速入弯 (TRACK_BASE_SPEED=65000, err=+7.0) 内轮刹车不因外轮近饱和而被对称截断 */
  setup(TRK_ZIGZAG_1); s_seg_mm = 1600.0f; mock_line = 0x004; /* ch 9 -> +7.0 */
  assert(step(0, 0, &l_norm, &r_norm) == 0U && (l_norm - r_norm) > 2.0f * (TRACK_MAX_SPEED - TRACK_BASE_SPEED));

  /* 无陀螺仪兜底时三环内环自动退化为前馈差速，仍能正常向右纠偏 */
  setup(TRK_ZIGZAG_1); s_gyro_ok = 0U; mock_line = 0x008; /* ch 8 -> +5.0 */
  assert(step(0, 0, &l_norm, &r_norm) == 0U && l_norm > r_norm && l_norm > 0.0f);
}

static void test_double_hex_corner_response(void) {
  const TrackState loops[] = {TRK_HEX_300_R, TRK_HEX_200_LOOP, TRK_HEX_300_L};
  for (unsigned i = 0; i < sizeof(loops) / sizeof(loops[0]); ++i) {
    float left, right;
    int turn_dir = (loops[i] == TRK_HEX_200_LOOP) ? 1 : -1;
    setup(loops[i]);
    mock_line = 0x060;
    step_1ms(0, 0, &left, &right);
    mock_line = (turn_dir > 0) ? 0x007 : 0xE00;
    step_1ms(0, 0, &left, &right);
    float initial_diff = (left - right) * turn_dir;
    assert(initial_diff > 0.0f && left > 0.0f && right > 0.0f);
    assert(s_prev_err == 9.0f * turn_dir);
    for (int j = 0; j < 20; ++j) step_1ms(0, 0, &left, &right);
    assert((left - right) * turn_dir > 2.0f * initial_diff);
    mock_line = 0U;
    step_1ms(0, 0, &left, &right);
    assert(s_search_active && s_search_dir == turn_dir);
  }
}

static void test_corner_latch_and_in_place_search(void) {
  float l, r;

  /* 双向十字 0x3FF / 0xFFF 与双簇六边形口 0x0E7 不得误触发单侧 90° 锁存 */
  setup(TRK_SQ_RM_LOOP); mock_line = 0x3FF;
  assert(step(0, 0, &l, &r) == 0U && s_corner_latch_ms == 0U);
  mock_line = 0xFFF;
  assert(step(0, 0, &l, &r) == 0U && s_corner_latch_ms == 0U);
  mock_line = 0x0E7;
  assert(step(0, 0, &l, &r) == 0U && s_corner_latch_ms == 0U);

  /* 右中方框压上右侧 90° 折角 0x07F -> 立即向右锁定并用更强内轮反转反甩，不会因下一拍中心单灯 0x040 残留反向掰直 */
  setup(TRK_SQ_RM_LOOP); s_seg_mm = 656.0f;
  mock_line = 0x07F; /* ch 5..11 */
  assert(step(0, 0, &l, &r) == 0U && s_corner_dir == 1 && s_corner_latch_ms > 0U);
  assert(s_last_dir == 1 && l > r && r < -NAV_FOLLOW_MAX_REV_SPEED &&
         0.5f * (l + r) <= 0.40f * TRACK_HEX_SPEED + 1.0f);

  mock_line = 0x07C; /* 离角瞬间外侧先脱线剩 width=5 (0x07C) 仍维持右转锁定，不提前放开 */
  assert(step(0, 0, &l, &r) == 0U && s_corner_dir == 1 && s_corner_latch_ms > 0U && l > r && r < 0.0f);

  mock_line = 0x040; /* 离角前瞬间中心单灯残留 (-1.0) 仍被右拐锁定滤除，不翻向也不左打 */
  assert(step(0, 0, &l, &r) == 0U && s_corner_dir == 1 && s_last_dir == 1 && l > r && r < 0.0f);

  /* 紧接着冲出角尖丢线 0x000 -> 锁死向右原点纯旋转寻线（严格对称满差速，零前窜） */
  mock_line = 0x000;
  assert(step(0, 0, &l, &r) == 0U && s_search_active && s_search_dir == 1 && s_search_fwd_spd == 0.0f);
  assert(fabsf(0.5f * (l + r)) < 1e-3f && fabsf(l - TRACK_MAX_REV_SPEED) < 1e-3f &&
         fabsf(r + TRACK_MAX_REV_SPEED) < 1e-3f);

  /* 原地旋过 65° 后自动带前速斜切抓线，120° 边界前持续前向切线，超界则反向回扫 */
  s_yaw_deg = 78.0f;
  assert(step(0, 0, &l, &r) == 0U && s_search_dir == 1 && 0.5f * (l + r) > 10000.0f);
  s_yaw_deg = 95.0f;
  assert(step(0, 0, &l, &r) == 0U && s_search_dir == 1 && 0.5f * (l + r) > 14000.0f);
  s_yaw_deg = 117.0f;
  assert(step(0, 0, &l, &r) == 0U && s_search_dir == -1 && s_search_phase == 1U);

  /* 锯齿出口经短直道折弯段全程保持 TRACK_SQ_SPEED 平滑进十字，不再插 120 mm 全速脉冲 */
  setup(TRK_ZIGZAG_1); s_seg_mm = 1600.0f; mock_line = 0x060;
  assert(step(0, 0, &l, &r) == 0U && fabsf(0.5f * (l + r) - TRACK_ZIGZAG_SPEED) < 1e-3f);
  setup(TRK_ZIGZAG_1); s_seg_mm = 2600.0f; mock_line = 0x060;
  assert(step(0, 0, &l, &r) == 0U && fabsf(0.5f * (l + r) - TRACK_SQ_SPEED) < 1e-3f);
  setup(TRK_ZIGZAG_1); s_seg_mm = 2720.0f; mock_line = 0x060;
  assert(step(0, 0, &l, &r) == 0U && fabsf(0.5f * (l + r) - TRACK_SQ_SPEED) < 1e-3f);

  /* 重抓边缘灯珠 (err=+7.0) 时保留较强内轮反甩制动，不被 6000 限死 */
  setup(TRK_ZIGZAG_1); s_seg_mm = 1800.0f; s_recapture_ms = NAV_RECAPTURE_MS; mock_line = 0x004;
  assert(step(0, 0, &l, &r) == 0U && r < -8000.0f);

  /* 只要后续踩回常规多灯直道 0x060（不必死等 48°），立即释放折角锁存并恢复满速巡线 */
  setup(TRK_SQ_RM_LOOP); mock_line = 0x07F;
  assert(step(0, 0, &l, &r) == 0U && s_corner_latch_ms > 0U);
  mock_line = 0x060;
  assert(step(0, 0, &l, &r) == 0U && s_corner_latch_ms == 0U &&
         l > 0.95f * TRACK_HEX_SPEED && r > 0.95f * TRACK_HEX_SPEED);

  /* 右中方框内第 1 弯已确立右转方向 (yaw_seg>=45°) 后，即使没触发折角识别直接丢线也锁死右转原地旋转 */
  setup(TRK_SQ_RM_LOOP); s_yaw_seg = 90.0f; s_last_dir = -1; mock_line = 0x000;
  assert(step(0, 0, &l, &r) == 0U && s_search_active && s_search_dir == 1 && s_search_fwd_spd == 0.0f);
}

static void test_1ms_control_rate(void) {
  float l, r;

  /* 1 ms 控制周期下陀螺仪低通滤波器保持约 3.7 ms 等效时间常数，不放大高频震动噪声 */
  setup(TRK_ZIGZAG_1); mock_line = 0x060; mock_gyro = -1640; /* +100 °/s */
  assert(step_1ms(0, 0, &l, &r) == 0U && s_rate_lpf > 20.0f && s_rate_lpf < 23.0f);
  for (int i = 0; i < 7; ++i) {
    assert(step_1ms(0, 0, &l, &r) == 0U);
  }
  assert(s_rate_lpf > 80.0f && s_rate_lpf < 88.0f);

  /* 1 ms 周期下断路连续丢线按物理时间 6 ms (6 帧) 确认，而非 3 ms */
  setup(TRK_APPROACH_BROKEN); s_seg_mm = 860.0f; mock_line = 0x000;
  for (int i = 0; i < 5; ++i) {
    assert(step_1ms(0, 0, &l, &r) == 0U && s_state == TRK_APPROACH_BROKEN);
  }
  assert(step_1ms(0, 0, &l, &r) == 0U && s_state == TRK_BROKEN_CLEAR);

  /* 1 ms 周期下终点横线消抖按物理时间 3 ms (3 帧) 确认 */
  setup(TRK_FINISH_ARMED); mock_line = 0xFFF;
  for (int i = 0; i < 2; ++i) {
    assert(step_1ms(0, 0, &l, &r) == 0U && s_state == TRK_FINISH_ARMED);
  }
  assert(step_1ms(0, 0, &l, &r) == 0U && s_state == TRK_FINISH_BAR1);

  /* 1 ms 周期下重抓稳姿窗口按 1 ms 精确递减并在满 20 ms 回正时提前释放 */
  setup(TRK_ZIGZAG_1); s_recapture_ms = NAV_RECAPTURE_MS; mock_line = 0x008;
  for (int i = 0; i < 20; ++i) {
    assert(step_1ms(0, 0, &l, &r) == 0U && s_recapture_ms > 0U);
  }
  assert(s_recapture_ms == (NAV_RECAPTURE_MS - 20U));
  mock_line = 0x060; mock_gyro = 0; s_rate_lpf = 0.0f;
  assert(step_1ms(0, 0, &l, &r) == 0U && s_recapture_ms == 0U);
}

int main(void) {
  test_calibration_and_resilience();
  test_gap();
  test_turn_and_reverse_diff();
  test_square_and_hex_r_transitions();
  test_finish();
  test_search_never_stops();
  test_follow_line_cascade();
  test_double_hex_corner_response();
  test_corner_latch_and_in_place_search();
  test_1ms_control_rate();
  puts("track navigation host tests: OK");
  return 0;
}
