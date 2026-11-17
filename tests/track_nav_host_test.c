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
  s_search_phase = 0U; s_recapture_ms = 0U;
  s_gyro_ok = 1U; s_last_raw_rate = 0.0f; s_last_dir = 0;
  s_small_dir_cand = 0; s_small_dir_cnt = 0U; s_rate_pid_integral = 0.0f;
  s_yaw_deg = s_yaw_ref = s_yaw_west = s_rate_lpf = s_gyro_bias = 0.0f;
  s_prev_err = 0.0f; s_err_rate_lpf = 0.0f; s_has_prev_err = 0U; enter_state(state);
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

  /* 断路斜插保持 +20° 直到 580 mm，不会在差 7 cm（550 mm）处提前归中 */
  setup(TRK_BROKEN_SEEK); s_seg_mm = 550.0f; s_yaw_deg = 20.0f;
  assert(step(0, 0, &l, &r) == 0U && l == TRACK_BROKEN_SPEED && r == TRACK_BROKEN_SPEED);

  /* 抓到线后进入巡线对齐并在满足最小距离后进入 PRE_T1 */
  setup(TRK_BROKEN_ALIGN); s_seg_mm = 61.0f; mock_line = 0x60;
  assert(step(0, 0, &l, &r) == 0U && s_state == TRK_TL_SQ_PRE_T1);

  /* 左上 500 末端的 0xFE0 是向南 90° 左折角而非直行 T 口：保持循迹转南至 <=-65° 回正后再切入 TO_BL1 */
  setup(TRK_TL_SQ_PRE_T1); s_seg_mm = 480.0f; mock_line = 0xFE0;
  assert(step(0, 0, &l, &r) == 0U && s_state == TRK_TL_SQ_PRE_T1 && r > l);
  s_yaw_deg = -75.0f; mock_line = 0x060;
  assert(step(0, 0, &l, &r) == 0U && s_state == TRK_TL_SQ_TO_BL1);
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
  s_seg_mm = 80.0f; mock_line = 0xFFF;
  assert(step(0, 0, &l, &r) == 0U && s_state == TRK_TL_SQ_TO_BL2);
  assert(step(0, 0, &l, &r) == 0U && s_state == TRK_TL_SQ_EXIT_W);
  for (int i = 0; i < 4; ++i) assert(step(1000, 1000, &l, &r) == 0U);
  assert(s_state == TRK_LEFT_SQ1_PRE);

  /* 左上 350 方框前置段仅 ~135 mm，>=75 mm 遇 0xFFF 应稳妥切入；左中 250 方框在 700 mm (>=650 mm, 270°) 应正常穿出 */
  setup(TRK_LEFT_SQ1_PRE); s_seg_mm = 100.0f; mock_line = 0xFFF;
  assert(step(0, 0, &l, &r) == 0U && s_state == TRK_LEFT_SQ1_PRE);
  assert(step(0, 0, &l, &r) == 0U && s_state == TRK_LEFT_SQ1_IN);
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

  /* 250 方框绕圈内的单侧直角弯 0x0FF 不提前切出；绕够 270° 且遇 0xFFF 才出方框 */
  setup(TRK_SQ_RM_LOOP); s_seg_mm = 760.0f; s_yaw_seg = 180.0f; mock_line = 0x0FF;
  assert(step(0, 0, &l, &r) == 0U && s_state == TRK_SQ_RM_LOOP);
  assert(step(0, 0, &l, &r) == 0U && s_state == TRK_SQ_RM_LOOP);
  s_yaw_seg = 270.0f; mock_line = 0xFFF;
  assert(step(0, 0, &l, &r) == 0U && s_state == TRK_SQ_RM_LOOP);
  assert(step(0, 0, &l, &r) == 0U && s_state == TRK_SQ_RM_OUT);

  /* 420 段 320mm 处左侧回流线 0xE60 (end_ch=6) 不提前进环；400mm 处右侧分叉 0x3B8 (end_ch=8) 才进环 */
  setup(TRK_APPROACH_HEX_R); s_seg_mm = 325.0f; mock_line = 0xE60;
  assert(step(0, 0, &l, &r) == 0U && s_state == TRK_APPROACH_HEX_R);
  assert(step(0, 0, &l, &r) == 0U && s_state == TRK_APPROACH_HEX_R);
  s_seg_mm = 400.0f; mock_line = 0x3B8;
  assert(step(0, 0, &l, &r) == 0U && s_state == TRK_APPROACH_HEX_R);
  assert(step(0, 0, &l, &r) == 0U && s_state == TRK_HEX_R_ENTER);

  /* 六边形绕完 ~860mm 遇出口叉口 0x1C3 切入 HEX_R_EXIT；若已超 -325° 则直接进 APPROACH_OCT 并正常右转 0x0FF */
  setup(TRK_HEX_R_LOOP); s_seg_mm = 860.0f; s_yaw_seg = -275.0f; mock_line = 0x1C3;
  assert(step(0, 0, &l, &r) == 0U && s_state == TRK_HEX_R_LOOP);
  assert(step(0, 0, &l, &r) == 0U && s_state == TRK_HEX_R_EXIT);

  setup(TRK_HEX_R_LOOP); s_seg_mm = 1000.0f; s_yaw_seg = -330.0f; mock_line = 0x0FF;
  assert(step(0, 0, &l, &r) == 0U && s_state == TRK_APPROACH_OCT && l > r);

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
  s_seg_mm = 1120.0f; s_yaw_seg = 300.0f; mock_line = 0x0E7;
  assert(step(0, 0, &l, &r) == 0U && s_state == TRK_HEX_BL_LOOP);
  assert(step(0, 0, &l, &r) == 0U && s_state == TRK_HEX_BL_EXIT);
  for (int i = 0; i < 4; ++i) assert(step(1000, 1000, &l, &r) == 0U);
  assert(s_state == TRK_APPROACH_HEX_MID);

  /* 双六边形 300+200：内切过弯 0x0EF 在 ~380mm (<= -35°) 直行切入小 200 六边形 -> 绕完 6 边直行穿回 300_L */
  setup(TRK_HEX_300_R); s_seg_mm = 380.0f; s_yaw_seg = -55.0f; mock_line = 0x0EF;
  assert(step(0, 0, &l, &r) == 0U && s_state == TRK_HEX_300_R);
  assert(step(0, 0, &l, &r) == 0U && s_state == TRK_HEX_200_IN);
  assert(step(0, 0, &l, &r) == 0U && l == TRACK_CROSS_SPEED && r == TRACK_CROSS_SPEED);
  for (int i = 0; i < 4; ++i) assert(step(1000, 1000, &l, &r) == 0U);
  assert(s_state == TRK_HEX_200_LOOP);
  s_seg_mm = 1100.0f; s_yaw_seg = 300.0f; mock_line = 0x0EF;
  assert(step(0, 0, &l, &r) == 0U && s_state == TRK_HEX_200_LOOP);
  assert(step(0, 0, &l, &r) == 0U && s_state == TRK_HEX_200_CROSS);
  for (int i = 0; i < 4; ++i) assert(step(1000, 1000, &l, &r) == 0U);
  assert(s_state == TRK_HEX_300_L);
}

static void test_finish(void) {
  float l, r;
  /* TRK_ZIGZAG_2 在 2360 mm 尚未提前臂合；单侧直角弯 0x0FF/0x1FF/0xFF8 或转弯角速度过大时绝不误触终点线或提前停车 */
  setup(TRK_ZIGZAG_2); s_seg_mm = 2360.0f; mock_line = 0x0FF;
  for (int i = 0; i < 6; ++i) assert(step(0, 0, &l, &r) == 0U);
  assert(s_state == TRK_ZIGZAG_2);

  setup(TRK_FINISH_ARMED);
  mock_line = 0x1F8; /* 6 路普通白线 */
  for (int i = 0; i < 10; ++i) assert(step(0, 0, &l, &r) == 0U);
  mock_line = 0x0FF; /* 单侧直角弯 ch0..7 */
  for (int i = 0; i < 6; ++i) assert(step(0, 0, &l, &r) == 0U);
  mock_line = 0xFF8; /* 单侧直角弯 ch3..11 */
  for (int i = 0; i < 6; ++i) assert(step(0, 0, &l, &r) == 0U);
  mock_line = 0x3FE; /* n=9 但 end_ch=9 < 10 */
  for (int i = 0; i < 6; ++i) assert(step(0, 0, &l, &r) == 0U);
  assert(s_state == TRK_FINISH_ARMED);

  /* 转弯旋转期间即使宽线 0xFFF 也不判定为终点横线 */
  mock_line = 0xFFF; mock_gyro = -2460; /* +150 °/s */
  for (int i = 0; i < 6; ++i) assert(step(0, 0, &l, &r) == 0U);
  assert(s_state == TRK_FINISH_ARMED);
  mock_gyro = 0; s_rate_lpf = 0.0f; s_yaw_deg = 0.0f;

  /* 直行碾过横跨两侧的第一条终点横线 (0x7FE) 触发 BAR1 */
  mock_line = 0x7FE;
  for (int i = 0; i < 2; ++i) assert(step(0, 0, &l, &r) == 0U);
  assert(s_state == TRK_FINISH_ARMED);
  assert(step(0, 0, &l, &r) == 0U && s_state == TRK_FINISH_BAR1);
  for (int i = 0; i < 20; ++i) assert(step(1200, 1200, &l, &r) == 0U);
  assert(s_state == TRK_FINISH_BAR1 && !s_finish_released);
  mock_line = 0x60;
  for (int i = 0; i < 3; ++i) assert(step(0, 0, &l, &r) == 0U);
  assert(s_finish_released);

  /* 到达第二条横线前若再遇到单侧 90° 弯 0x1FF，仍保持寻线不提前 TRK_DONE */
  s_seg_mm = 340.0f; mock_line = 0x1FF;
  for (int i = 0; i < 6; ++i) assert(step(0, 0, &l, &r) == 0U);
  assert(s_state == TRK_FINISH_BAR1);

  /* 正常直行碾过第二条终点横线 0xFFF 才完赛停车 */
  mock_line = 0xFFF;
  for (int i = 0; i < 2; ++i) assert(step(0, 0, &l, &r) == 0U);
  assert(step(0, 0, &l, &r) == 1U && s_state == TRK_DONE && l == 0 && r == 0);
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
}

static void test_follow_line_cascade(void) {
  float l0, r0, l_fast, r_fast, l_exit, r_exit;

  /* 极端外沿偏差 (+11.0) 下目标角速度被外环钳位在 320 °/s，且当实测角速度超限时差速自动收缩 */
  setup(TRK_SQ_RM_LOOP);
  mock_line = 0x001; /* ch 11 -> +11.0 */
  assert(step(0, 0, &l0, &r0) == 0U && l0 > r0 && r0 >= -NAV_FOLLOW_MAX_REV_SPEED);

  mock_gyro = -6560; /* 对应 +400 °/s 顺时针旋转 */
  for (int i = 0; i < 8; ++i) {
    assert(step(0, 0, &l_fast, &r_fast) == 0U);
  }
  assert(s_rate_lpf > NAV_FOLLOW_MAX_RATE_DPS);
  assert((l_fast - r_fast) < (l0 - r0));
  assert((l_fast - r_fast) < 2.0f * NAV_RATE_FF_GAIN * NAV_FOLLOW_MAX_RATE_DPS);

  /* 出弯刚刚压回中心线 (err = 0) 而车身仍在顺时针快转时，内环主动反打制动并压低前进基速 */
  mock_line = 0x060; /* ch 5,6 -> 0.0 */
  assert(step(0, 0, &l_exit, &r_exit) == 0U);
  assert(r_exit > l_exit);
  assert(0.5f * (l_exit + r_exit) < TRACK_BASE_SPEED);

  /* 锯齿段目标角速度上限 (290 °/s) 比普通弯道 (320 °/s) 更收敛 */
  setup(TRK_ZIGZAG_1); s_seg_mm = 100.0f; mock_line = 0x001;
  float l_zig, r_zig, l_norm, r_norm;
  assert(step(0, 0, &l_zig, &r_zig) == 0U && l_zig > r_zig);
  setup(TRK_ZIGZAG_1); s_seg_mm = 1600.0f; mock_line = 0x001;
  assert(step(0, 0, &l_norm, &r_norm) == 0U && (l_zig - r_zig) < (l_norm - r_norm));

  /* 无陀螺仪兜底时三环内环自动退化为前馈差速，仍能正常向右纠偏 */
  setup(TRK_ZIGZAG_1); s_gyro_ok = 0U; mock_line = 0x008; /* ch 8 -> +5.0 */
  assert(step(0, 0, &l_norm, &r_norm) == 0U && l_norm > r_norm && l_norm > 0.0f);
}

int main(void) {
  test_calibration_and_resilience();
  test_gap();
  test_turn_and_reverse_diff();
  test_square_and_hex_r_transitions();
  test_finish();
  test_search_never_stops();
  test_follow_line_cascade();
  puts("track navigation host tests: OK");
  return 0;
}
