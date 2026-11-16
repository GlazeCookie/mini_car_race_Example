#include "track_nav.h"
#include "main.h"
#include "multiplexer.h"
#include "dodo_BMI270.h"
#include <math.h>
#include <stddef.h>

/* 复用 turn_test 实测的陀螺仪角度比例与采样换算 */
#define NAV_GYRO_SCALE       0.9807f
#define NAV_GYRO_SENS        16.4f
#define NAV_GYRO_CAL_MS      1000U
#define NAV_LINE_ACTIVE      1U

/* 巡线 PD 参数（对应 main.c） */
#define NAV_LINE_KP          700.0f
#define NAV_LINE_KP2         60.0f
#define NAV_LINE_KD          80.0f
#define NAV_GYRO_DAMP_ZIG    14.0f   /* 锯齿段陀螺仪角速度阻尼 */
#define NAV_YAW_HOLD_KP      140.0f  /* 锁航向直行纠偏增益 (counts/s per deg) */

/* 路口与断路几何参数（单位: mm） */
#define NAV_CROSS_PASS_MM    95.0f   /* 穿十字直行距离 */
#define NAV_T_PASS_MM        90.0f   /* 过 T 口直行距离 */
#define NAV_BROKEN_CLEAR_MM  100.0f  /* 断路直行离端头距离 */
#define NAV_BROKEN_SEEK_MM   480.0f  /* 断路 +15° 斜插距离 */
#define NAV_BROKEN_SWEEP_MM  720.0f  /* 断路最大搜线保护距离 */
#define NAV_BROKEN_ALIGN_MM  70.0f   /* 抓到对面线后回正距离 */
#define NAV_BROKEN_OFFSET_DEG 15.0f  /* 右->左过断路向右(北)偏转 15° */

static TrackState s_state = TRK_START_CLEAR;
static float s_gyro_bias = 0.0f;
static float s_yaw_deg = 0.0f;       /* 全局累计航向角（顺时针为正） */
static float s_yaw_seg = 0.0f;       /* 当前状态累计转角 */
static float s_rate_lpf = 0.0f;      /* 低通角速度（°/s，顺时针为正） */
static float s_seg_mm = 0.0f;        /* 当前状态累计里程（mm） */
static float s_yaw_ref = 0.0f;       /* 参考航向（锁直行/断路/转向用） */
static float s_yaw_west = 0.0f;      /* 过断路前记录的西向基准航向 */
static float s_yaw_smooth = 0.0f;    /* 近期平滑航向（避开进十字瞬间干扰） */

static float s_prev_err = 0.0f;
static uint8_t s_has_prev_err = 0U;
static int8_t s_last_dir = 0;        /* -1 左, +1 右 */
static uint16_t s_lost_ms = 0U;
static uint8_t s_broken_miss_cnt = 0U;
static uint8_t s_cross_debounce = 0U;

static float clamp_f(float v, float lo, float hi) {
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

static void enter_state(TrackState next) {
  s_state = next;
  s_seg_mm = 0.0f;
  s_yaw_seg = 0.0f;
  s_cross_debounce = 0U;
  s_broken_miss_cnt = 0U;
}

void TrackNav_ScanLine(uint8_t active_level, LineScanResult *scan) {
  uint16_t mux_val = 0U;
  uint8_t in_cluster = 0U;
  uint8_t start_ch = 0U;
  int32_t pos_sum = 0;
  uint8_t i;

  MUX_get_value(&mux_val);
  scan->mux_value = mux_val;
  scan->active_count = 0U;
  scan->cluster_count = 0U;
  scan->max_width = 0U;

  for (i = 0U; i < MULTIPLEXER_CHANNEL_NUM; ++i) {
    uint8_t on = (MUX_GET_CHANNEL(mux_val, i) == active_level) ? 1U : 0U;
    if (on) {
      ++scan->active_count;
      if (!in_cluster) {
        in_cluster = 1U;
        start_ch = i;
        pos_sum = 0;
      }
      pos_sum += (int32_t)i * 2 - 11;
    } else if (in_cluster) {
      uint8_t width = (uint8_t)(i - start_ch);
      if (scan->cluster_count < 6U) {
        LineCluster *c = &scan->clusters[scan->cluster_count++];
        c->start_ch = start_ch;
        c->end_ch = (uint8_t)(i - 1U);
        c->width = width;
        c->center = (float)pos_sum / (float)width;
      }
      if (width > scan->max_width) scan->max_width = width;
      in_cluster = 0U;
    }
  }
  if (in_cluster) {
    uint8_t width = (uint8_t)(MULTIPLEXER_CHANNEL_NUM - start_ch);
    if (scan->cluster_count < 6U) {
      LineCluster *c = &scan->clusters[scan->cluster_count++];
      c->start_ch = start_ch;
      c->end_ch = (uint8_t)(MULTIPLEXER_CHANNEL_NUM - 1U);
      c->width = width;
      c->center = (float)pos_sum / (float)width;
    }
    if (width > scan->max_width) scan->max_width = width;
  }
}

static uint8_t is_crossroad(const LineScanResult *scan) {
  return (scan->active_count >= 6U || scan->max_width >= 5U) ? 1U : 0U;
}

static uint8_t is_left_branch(const LineScanResult *scan) {
  if (scan->cluster_count >= 2U) return 1U;
  if (scan->active_count >= 5U && scan->clusters[0].start_ch <= 1U) return 1U;
  return 0U;
}

static uint8_t is_branch_or_cross(const LineScanResult *scan) {
  return (scan->cluster_count >= 2U || scan->active_count >= 5U || scan->max_width >= 4U) ? 1U : 0U;
}

/* 选簇巡线：pick_mode = 0 取最贴近上次/中心簇，-1 取最左簇，+1 取最右簇 */
static void follow_line_pid(const LineScanResult *scan, int8_t pick_mode,
                            float base_target, float gyro_damp, float dt_ms,
                            float *out_l, float *out_r) {
  float err;
  float err_diff;
  float abs_err;
  float abs_diff;
  float severity;
  float scale;
  float base_spd;
  float corr;

  if (scan->cluster_count == 0U) {
    s_has_prev_err = 0U;
    s_lost_ms = (uint16_t)(s_lost_ms + (uint16_t)dt_ms);
    if (s_last_dir != 0 && s_lost_ms <= 800U) {
      *out_l = (s_last_dir > 0) ? 5500.0f : 0.0f;
      *out_r = (s_last_dir < 0) ? 5500.0f : 0.0f;
    } else {
      *out_l = 0.0f;
      *out_r = 0.0f;
    }
    return;
  }

  s_lost_ms = 0U;
  if (pick_mode < 0) {
    /* 若分叉尚未完全裂开成两簇（单簇但变宽），取最左边缘主动左咬 */
    if (scan->cluster_count == 1U && scan->clusters[0].width >= 4U) {
      err = (float)scan->clusters[0].start_ch * 2.0f - 11.0f;
    } else {
      err = scan->clusters[0].center;
    }
  } else if (pick_mode > 0) {
    if (scan->cluster_count == 1U && scan->clusters[0].width >= 4U) {
      err = (float)scan->clusters[0].end_ch * 2.0f - 11.0f;
    } else {
      err = scan->clusters[scan->cluster_count - 1U].center;
    }
  } else {
    /* 默认挑最靠近上一次偏差（或中心）的簇，抑制双线跳变 */
    uint8_t best_idx = 0U;
    float ref = s_has_prev_err ? s_prev_err : 0.0f;
    float best_d = fabsf(scan->clusters[0].center - ref);
    uint8_t i;
    for (i = 1U; i < scan->cluster_count; ++i) {
      float d = fabsf(scan->clusters[i].center - ref);
      if (d < best_d) {
        best_d = d;
        best_idx = i;
      }
    }
    err = scan->clusters[best_idx].center;
  }

  if (err > 0.5f) s_last_dir = 1;
  else if (err < -0.5f) s_last_dir = -1;

  err_diff = s_has_prev_err ? (err - s_prev_err) : 0.0f;
  s_prev_err = err;
  s_has_prev_err = 1U;

  abs_err = fabsf(err);
  corr = NAV_LINE_KP * err + NAV_LINE_KP2 * err * abs_err + NAV_LINE_KD * err_diff;
  if ((err > 0.0f && corr < 0.0f) || (err < 0.0f && corr > 0.0f) || err == 0.0f) {
    corr = 0.0f;
  }
  corr -= gyro_damp * s_rate_lpf;

  abs_diff = fabsf(err_diff);
  severity = abs_err;
  if (abs_diff * 2.5f > severity) severity = abs_diff * 2.5f;

  scale = 1.0f - 0.14f * (severity - 3.5f);
  scale = clamp_f(scale, 0.25f, 1.0f);
  base_spd = base_target * scale;

  *out_l = clamp_f(base_spd + corr, 0.0f, TRACK_MAX_SPEED);
  *out_r = clamp_f(base_spd - corr, 0.0f, TRACK_MAX_SPEED);
}

/* 陀螺仪锁航向直行 */
static void follow_yaw_hold(float target_yaw, float speed, float *out_l, float *out_r) {
  float yaw_err = target_yaw - s_yaw_deg;
  float corr = NAV_YAW_HOLD_KP * yaw_err - 8.0f * s_rate_lpf;
  *out_l = clamp_f(speed + corr, 0.0f, TRACK_MAX_SPEED);
  *out_r = clamp_f(speed - corr, 0.0f, TRACK_MAX_SPEED);
  s_has_prev_err = 0U;
}

/* 差速定角转向（仅外轮正转，不切 DIR 引脚）；返回 1 表示到达目标并捕获线 */
static uint8_t gyro_turn_step(float target_yaw, int8_t dir_sign, float min_turn_deg,
                              const LineScanResult *scan, float *out_l, float *out_r) {
  float rem = (target_yaw - s_yaw_deg) * (float)dir_sign;
  float turned = fabsf(s_yaw_seg);
  s_has_prev_err = 0U;

  if (turned >= min_turn_deg && scan->cluster_count >= 1U) {
    uint8_t idx = (dir_sign < 0) ? 0U : (uint8_t)(scan->cluster_count - 1U);
    if (fabsf(scan->clusters[idx].center) <= 6.5f) {
      return 1U;
    }
  }
  if (rem <= 6.0f) {
    return 1U;
  }

  if (dir_sign > 0) {
    *out_l = TRACK_TURN_OUTER_SPEED;
    *out_r = 0.0f;
  } else {
    *out_l = 0.0f;
    *out_r = TRACK_TURN_OUTER_SPEED;
  }
  return 0U;
}

void TrackNav_Start(void) {
  int32_t sum = 0;
  uint32_t count = 0U;
  uint32_t start_tick;

  s_yaw_deg = 0.0f;
  s_yaw_seg = 0.0f;
  s_rate_lpf = 0.0f;
  s_seg_mm = 0.0f;
  s_yaw_ref = 0.0f;
  s_yaw_west = 0.0f;
  s_yaw_smooth = 0.0f;
  s_prev_err = 0.0f;
  s_has_prev_err = 0U;
  s_last_dir = 0;
  s_lost_ms = 0U;

  if (dodo_BMI270_init() != 0) {
    s_state = TRK_FAULT_STOP;
    return;
  }

  start_tick = HAL_GetTick();
  while ((uint32_t)(HAL_GetTick() - start_tick) < NAV_GYRO_CAL_MS) {
    dodo_BMI270_get_data();
    if (BMI270_last_rslt != 0) {
      s_state = TRK_FAULT_STOP;
      return;
    }
    sum += (int32_t)BMI270_gyro_z;
    ++count;
    HAL_Delay(5U);
  }

  s_gyro_bias = (count > 0U) ? ((float)sum / (float)count) : 0.0f;
  enter_state(TRK_START_CLEAR);
}

uint8_t TrackNav_Step(int32_t left_delta, int32_t right_delta, float dt_s,
                      float *out_l, float *out_r) {
  LineScanResult scan;
  float raw_rate;
  float step_mm;
  float dt_ms = dt_s * 1000.0f;

  if (s_state == TRK_DONE || s_state == TRK_FAULT_STOP) {
    *out_l = 0.0f;
    *out_r = 0.0f;
    return 1U;
  }

  dodo_BMI270_get_data();
  if (BMI270_last_rslt != 0) {
    s_state = TRK_FAULT_STOP;
    *out_l = 0.0f;
    *out_r = 0.0f;
    return 1U;
  }

  raw_rate = -((float)BMI270_gyro_z - s_gyro_bias) / NAV_GYRO_SENS;
  s_yaw_deg += raw_rate * dt_s * NAV_GYRO_SCALE;
  s_yaw_seg += raw_rate * dt_s * NAV_GYRO_SCALE;
  s_rate_lpf = 0.65f * s_rate_lpf + 0.35f * raw_rate;
  s_yaw_smooth = 0.90f * s_yaw_smooth + 0.10f * s_yaw_deg;

  step_mm = 0.5f * (float)(left_delta + right_delta) * 1000.0f / TRACK_COUNTS_PER_METER;
  s_seg_mm += step_mm;

  TrackNav_ScanLine(NAV_LINE_ACTIVE, &scan);

  switch (s_state) {
    /* 1. 发车区起步屏蔽双横线（前 900 mm 不检测十字/终点） */
    case TRK_START_CLEAR:
      follow_line_pid(&scan, 0, TRACK_BASE_SPEED, 0.0f, dt_ms, out_l, out_r);
      if (s_seg_mm >= 900.0f) {
        enter_state(TRK_ZIGZAG_1);
      }
      break;

    /* 2. 右下 2 峰锯齿 + 右下折角升至右中 250 方框十字 */
    case TRK_ZIGZAG_1: {
      float spd = (s_seg_mm < 1550.0f) ? TRACK_ZIGZAG_SPEED : TRACK_BASE_SPEED;
      float damp = (s_seg_mm < 1550.0f) ? NAV_GYRO_DAMP_ZIG : 0.0f;
      follow_line_pid(&scan, 0, spd, damp, dt_ms, out_l, out_r);
      if (s_seg_mm >= 2200.0f && is_crossroad(&scan)) {
        if (++s_cross_debounce >= 2U) {
          s_yaw_ref = s_yaw_smooth;
          enter_state(TRK_SQ_RM_IN);
        }
      } else {
        s_cross_debounce = 0U;
      }
      break;
    }

    /* 3. 右中 250x250 方框（十字直行进 -> 绕圈 -> 十字直行出） */
    case TRK_SQ_RM_IN:
      follow_yaw_hold(s_yaw_ref, TRACK_CROSS_SPEED, out_l, out_r);
      if (s_seg_mm >= NAV_CROSS_PASS_MM) enter_state(TRK_SQ_RM_LOOP);
      break;

    case TRK_SQ_RM_LOOP:
      follow_line_pid(&scan, 0, TRACK_BASE_SPEED, 0.0f, dt_ms, out_l, out_r);
      if (s_seg_mm >= 750.0f && is_crossroad(&scan)) {
        if (++s_cross_debounce >= 2U) {
          s_yaw_ref = s_yaw_smooth;
          enter_state(TRK_SQ_RM_OUT);
        }
      } else {
        s_cross_debounce = 0U;
      }
      break;

    case TRK_SQ_RM_OUT:
      follow_yaw_hold(s_yaw_ref, TRACK_CROSS_SPEED, out_l, out_r);
      if (s_seg_mm >= NAV_CROSS_PASS_MM) enter_state(TRK_APPROACH_HEX_R);
      break;

    /* 4. 右中 200 六边形（30° 左偏入环 -> 绕一圈 -> 左偏出到 330） */
    case TRK_APPROACH_HEX_R:
      follow_line_pid(&scan, 0, TRACK_BASE_SPEED, 0.0f, dt_ms, out_l, out_r);
      if (s_seg_mm >= 260.0f && s_seg_mm <= 560.0f && is_branch_or_cross(&scan)) {
        s_yaw_ref = s_yaw_smooth;
        enter_state(TRK_HEX_R_ENTER);
      }
      break;

    case TRK_HEX_R_ENTER:
      follow_line_pid(&scan, -1, TRACK_HEX_SPEED, 0.0f, dt_ms, out_l, out_r);
      if (s_seg_mm >= 130.0f) enter_state(TRK_HEX_R_LOOP);
      break;

    case TRK_HEX_R_LOOP:
      follow_line_pid(&scan, 0, TRACK_HEX_SPEED, 0.0f, dt_ms, out_l, out_r);
      if (s_seg_mm >= 900.0f && s_yaw_seg <= -245.0f && is_branch_or_cross(&scan)) {
        enter_state(TRK_HEX_R_EXIT);
      }
      break;

    case TRK_HEX_R_EXIT:
      follow_line_pid(&scan, -1, TRACK_HEX_SPEED, 0.0f, dt_ms, out_l, out_r);
      if (s_seg_mm >= 140.0f) enter_state(TRK_APPROACH_OCT);
      break;

    /* 5. 右上八边形（十字直行进 -> 绕一圈 -> 十字直行出） */
    case TRK_APPROACH_OCT:
      follow_line_pid(&scan, 0, TRACK_BASE_SPEED, 0.0f, dt_ms, out_l, out_r);
      if (s_seg_mm >= 1450.0f && is_crossroad(&scan)) {
        if (++s_cross_debounce >= 2U) {
          s_yaw_ref = s_yaw_smooth;
          enter_state(TRK_OCT_IN);
        }
      } else {
        s_cross_debounce = 0U;
      }
      break;

    case TRK_OCT_IN:
      follow_yaw_hold(s_yaw_ref, TRACK_CROSS_SPEED, out_l, out_r);
      if (s_seg_mm >= NAV_CROSS_PASS_MM) enter_state(TRK_OCT_LOOP);
      break;

    case TRK_OCT_LOOP:
      follow_line_pid(&scan, 0, TRACK_HEX_SPEED, 0.0f, dt_ms, out_l, out_r);
      if (s_seg_mm >= 1550.0f && is_crossroad(&scan)) {
        if (++s_cross_debounce >= 2U) {
          s_yaw_ref = s_yaw_smooth;
          enter_state(TRK_OCT_OUT);
        }
      } else {
        s_cross_debounce = 0U;
      }
      break;

    case TRK_OCT_OUT:
      follow_yaw_hold(s_yaw_ref, TRACK_CROSS_SPEED, out_l, out_r);
      if (s_seg_mm >= NAV_CROSS_PASS_MM) enter_state(TRK_APPROACH_BROKEN);
      break;

    /* 6. 前往并跨越顶部断路区域（650 mm 且目标线偏北/右前方 ~130 mm） */
    case TRK_APPROACH_BROKEN:
      follow_line_pid(&scan, 0, TRACK_BASE_SPEED, 0.0f, dt_ms, out_l, out_r);
      if (s_seg_mm >= 900.0f && scan.cluster_count == 0U) {
        if (++s_broken_miss_cnt >= 3U) {
          s_yaw_west = s_yaw_smooth;
          s_yaw_ref = s_yaw_west;
          enter_state(TRK_BROKEN_CLEAR);
        }
      } else {
        s_broken_miss_cnt = 0U;
      }
      break;

    case TRK_BROKEN_CLEAR:
      follow_yaw_hold(s_yaw_west, TRACK_BROKEN_SPEED, out_l, out_r);
      if (s_seg_mm >= NAV_BROKEN_CLEAR_MM) enter_state(TRK_BROKEN_SEEK);
      break;

    case TRK_BROKEN_SEEK: {
      float seek_yaw = s_yaw_west + NAV_BROKEN_OFFSET_DEG;
      if (s_seg_mm > NAV_BROKEN_SEEK_MM) {
        /* 兜底扫线：若尚未压线，做小幅左右摆头扩大横向覆盖 */
        float phase = s_seg_mm - NAV_BROKEN_SEEK_MM;
        seek_yaw = (phase < 120.0f) ? (s_yaw_west - 20.0f) : (s_yaw_west + 28.0f);
      }
      follow_yaw_hold(seek_yaw, TRACK_BROKEN_SPEED, out_l, out_r);
      if (s_seg_mm >= 220.0f && scan.active_count >= 1U) {
        enter_state(TRK_BROKEN_ALIGN);
      } else if (s_seg_mm >= NAV_BROKEN_SWEEP_MM) {
        /* 保护：到上限仍强行进入对齐搜索 */
        enter_state(TRK_BROKEN_ALIGN);
      }
      break;
    }

    case TRK_BROKEN_ALIGN:
      follow_yaw_hold(s_yaw_west, TRACK_BROKEN_SPEED, out_l, out_r);
      if (s_seg_mm >= NAV_BROKEN_ALIGN_MM && scan.cluster_count >= 1U) {
        enter_state(TRK_TL_SQ_PRE_T1);
      }
      break;

    /* 7. 左上 500 下方 250x250 T 型方框（必须绕行完整四边） */
    case TRK_TL_SQ_PRE_T1:
      follow_line_pid(&scan, 0, TRACK_BASE_SPEED, 0.0f, dt_ms, out_l, out_r);
      if (is_left_branch(&scan)) {
        enter_state(TRK_TL_SQ_T1_PASS);
      }
      break;

    case TRK_TL_SQ_T1_PASS:
      follow_yaw_hold(s_yaw_west, TRACK_CROSS_SPEED, out_l, out_r);
      if (s_seg_mm >= NAV_T_PASS_MM) enter_state(TRK_TL_SQ_TO_BL1);
      break;

    case TRK_TL_SQ_TO_BL1:
      follow_line_pid(&scan, 0, TRACK_HEX_SPEED, 0.0f, dt_ms, out_l, out_r);
      if (s_seg_mm >= 260.0f && (s_yaw_deg - s_yaw_west) <= -65.0f && is_branch_or_cross(&scan)) {
        enter_state(TRK_TL_SQ_TURN_E);
      }
      break;

    case TRK_TL_SQ_TURN_E:
      if (gyro_turn_step(s_yaw_west - 180.0f, -1, 65.0f, &scan, out_l, out_r)) {
        enter_state(TRK_TL_SQ_TO_TR2);
      }
      break;

    case TRK_TL_SQ_TO_TR2:
      follow_line_pid(&scan, 0, TRACK_HEX_SPEED, 0.0f, dt_ms, out_l, out_r);
      if (s_seg_mm >= 340.0f && (s_yaw_deg - s_yaw_west) <= -245.0f && is_branch_or_cross(&scan)) {
        enter_state(TRK_TL_SQ_TURN_W);
      }
      break;

    case TRK_TL_SQ_TURN_W:
      if (gyro_turn_step(s_yaw_west - 360.0f, -1, 65.0f, &scan, out_l, out_r)) {
        enter_state(TRK_TL_SQ_TO_BL2);
      }
      break;

    case TRK_TL_SQ_TO_BL2:
      follow_line_pid(&scan, 0, TRACK_HEX_SPEED, 0.0f, dt_ms, out_l, out_r);
      if (s_seg_mm >= 340.0f && (s_yaw_deg - s_yaw_west) <= -425.0f && is_branch_or_cross(&scan)) {
        enter_state(TRK_TL_SQ_EXIT_W);
      }
      break;

    case TRK_TL_SQ_EXIT_W:
      if (gyro_turn_step(s_yaw_west - 360.0f, +1, 65.0f, &scan, out_l, out_r)) {
        enter_state(TRK_LEFT_SQ1_PRE);
      }
      break;

    /* 8. 左侧三个十字方框（350x350 -> 250x250 -> 250x250） */
    case TRK_LEFT_SQ1_PRE:
      follow_line_pid(&scan, 0, TRACK_BASE_SPEED, 0.0f, dt_ms, out_l, out_r);
      if (s_seg_mm >= 140.0f && is_crossroad(&scan)) {
        s_yaw_ref = s_yaw_smooth;
        enter_state(TRK_LEFT_SQ1_IN);
      }
      break;
    case TRK_LEFT_SQ1_IN:
      follow_yaw_hold(s_yaw_ref, TRACK_CROSS_SPEED, out_l, out_r);
      if (s_seg_mm >= NAV_CROSS_PASS_MM) enter_state(TRK_LEFT_SQ1_LOOP);
      break;
    case TRK_LEFT_SQ1_LOOP:
      follow_line_pid(&scan, 0, TRACK_BASE_SPEED, 0.0f, dt_ms, out_l, out_r);
      if (s_seg_mm >= 1050.0f && is_crossroad(&scan)) {
        s_yaw_ref = s_yaw_smooth;
        enter_state(TRK_LEFT_SQ1_OUT);
      }
      break;
    case TRK_LEFT_SQ1_OUT:
      follow_yaw_hold(s_yaw_ref, TRACK_CROSS_SPEED, out_l, out_r);
      if (s_seg_mm >= NAV_CROSS_PASS_MM) enter_state(TRK_LEFT_SQ2_PRE);
      break;

    case TRK_LEFT_SQ2_PRE:
      follow_line_pid(&scan, 0, TRACK_BASE_SPEED, 0.0f, dt_ms, out_l, out_r);
      if (s_seg_mm >= 360.0f && is_crossroad(&scan)) {
        s_yaw_ref = s_yaw_smooth;
        enter_state(TRK_LEFT_SQ2_IN);
      }
      break;
    case TRK_LEFT_SQ2_IN:
      follow_yaw_hold(s_yaw_ref, TRACK_CROSS_SPEED, out_l, out_r);
      if (s_seg_mm >= NAV_CROSS_PASS_MM) enter_state(TRK_LEFT_SQ2_LOOP);
      break;
    case TRK_LEFT_SQ2_LOOP:
      follow_line_pid(&scan, 0, TRACK_BASE_SPEED, 0.0f, dt_ms, out_l, out_r);
      if (s_seg_mm >= 750.0f && is_crossroad(&scan)) {
        s_yaw_ref = s_yaw_smooth;
        enter_state(TRK_LEFT_SQ2_OUT);
      }
      break;
    case TRK_LEFT_SQ2_OUT:
      follow_yaw_hold(s_yaw_ref, TRACK_CROSS_SPEED, out_l, out_r);
      if (s_seg_mm >= NAV_CROSS_PASS_MM) enter_state(TRK_LEFT_SQ3_PRE);
      break;

    case TRK_LEFT_SQ3_PRE:
      follow_line_pid(&scan, 0, TRACK_BASE_SPEED, 0.0f, dt_ms, out_l, out_r);
      if (s_seg_mm >= 180.0f && is_crossroad(&scan)) {
        s_yaw_ref = s_yaw_smooth;
        enter_state(TRK_LEFT_SQ3_IN);
      }
      break;
    case TRK_LEFT_SQ3_IN:
      follow_yaw_hold(s_yaw_ref, TRACK_CROSS_SPEED, out_l, out_r);
      if (s_seg_mm >= NAV_CROSS_PASS_MM) enter_state(TRK_LEFT_SQ3_LOOP);
      break;
    case TRK_LEFT_SQ3_LOOP:
      follow_line_pid(&scan, 0, TRACK_BASE_SPEED, 0.0f, dt_ms, out_l, out_r);
      if (s_seg_mm >= 750.0f && is_crossroad(&scan)) {
        s_yaw_ref = s_yaw_smooth;
        enter_state(TRK_LEFT_SQ3_OUT);
      }
      break;
    case TRK_LEFT_SQ3_OUT:
      follow_yaw_hold(s_yaw_ref, TRACK_CROSS_SPEED, out_l, out_r);
      if (s_seg_mm >= NAV_CROSS_PASS_MM) enter_state(TRK_APPROACH_HEX_BL);
      break;

    /* 9. 左下 200 六边形（沿 460+225 到 60° 交叉 -> 右转 60° 顺时针绕 -> 左转 120° 出） */
    case TRK_APPROACH_HEX_BL:
      /* 约 485mm 处经过顶边 T 汇入时取最左簇防被拉偏 */
      follow_line_pid(&scan, (s_seg_mm >= 440.0f && s_seg_mm < 560.0f) ? -1 : 0,
                      TRACK_HEX_SPEED, 0.0f, dt_ms, out_l, out_r);
      if (s_seg_mm >= 580.0f && is_branch_or_cross(&scan)) {
        s_yaw_ref = s_yaw_smooth;
        enter_state(TRK_HEX_BL_IN);
      }
      break;

    case TRK_HEX_BL_IN:
      if (gyro_turn_step(s_yaw_ref + 60.0f, +1, 40.0f, &scan, out_l, out_r)) {
        enter_state(TRK_HEX_BL_LOOP);
      }
      break;

    case TRK_HEX_BL_LOOP:
      follow_line_pid(&scan, 0, TRACK_HEX_SPEED, 0.0f, dt_ms, out_l, out_r);
      if (s_seg_mm >= 1000.0f && s_yaw_seg >= 285.0f && is_branch_or_cross(&scan)) {
        s_yaw_ref = s_yaw_smooth;
        enter_state(TRK_HEX_BL_EXIT);
      }
      break;

    case TRK_HEX_BL_EXIT:
      if (gyro_turn_step(s_yaw_ref - 120.0f, -1, 95.0f, &scan, out_l, out_r)) {
        enter_state(TRK_APPROACH_HEX_MID);
      }
      break;

    /* 10. 中左 300+200 双六边形 8 字一笔画 */
    case TRK_APPROACH_HEX_MID:
      follow_line_pid(&scan, 0, TRACK_BASE_SPEED, 0.0f, dt_ms, out_l, out_r);
      if (s_seg_mm >= 700.0f && is_branch_or_cross(&scan)) {
        enter_state(TRK_HEX_300_ENTER);
      }
      break;

    case TRK_HEX_300_ENTER:
      follow_line_pid(&scan, -1, TRACK_HEX_SPEED, 0.0f, dt_ms, out_l, out_r);
      if (s_seg_mm >= 130.0f) enter_state(TRK_HEX_300_R);
      break;

    case TRK_HEX_300_R:
      follow_line_pid(&scan, 0, TRACK_HEX_SPEED, 0.0f, dt_ms, out_l, out_r);
      if (s_seg_mm >= 440.0f && is_branch_or_cross(&scan)) {
        s_yaw_ref = s_yaw_smooth;
        enter_state(TRK_HEX_200_IN);
      }
      break;

    case TRK_HEX_200_IN:
      if (gyro_turn_step(s_yaw_ref + 60.0f, +1, 40.0f, &scan, out_l, out_r)) {
        enter_state(TRK_HEX_200_LOOP);
      }
      break;

    case TRK_HEX_200_LOOP:
      follow_line_pid(&scan, 0, TRACK_HEX_SPEED, 0.0f, dt_ms, out_l, out_r);
      if (s_seg_mm >= 980.0f && s_yaw_seg >= 260.0f && is_branch_or_cross(&scan)) {
        s_yaw_ref = s_yaw_smooth;
        enter_state(TRK_HEX_200_CROSS);
      }
      break;

    case TRK_HEX_200_CROSS:
      follow_yaw_hold(s_yaw_ref, TRACK_CROSS_SPEED, out_l, out_r);
      if (s_seg_mm >= 85.0f) enter_state(TRK_HEX_300_L);
      break;

    case TRK_HEX_300_L:
      follow_line_pid(&scan, 0, TRACK_HEX_SPEED, 0.0f, dt_ms, out_l, out_r);
      if (s_seg_mm >= 720.0f && s_yaw_seg <= -90.0f && is_branch_or_cross(&scan)) {
        enter_state(TRK_HEX_300_EXIT);
      }
      break;

    case TRK_HEX_300_EXIT:
      follow_line_pid(&scan, -1, TRACK_HEX_SPEED, 0.0f, dt_ms, out_l, out_r);
      if (s_seg_mm >= 140.0f) enter_state(TRK_ZIGZAG_2);
      break;

    /* 11. 收尾：530+550+240 + 3峰锯齿 + 490 回发车区双横线停车 */
    case TRK_ZIGZAG_2: {
      uint8_t in_zig = (s_seg_mm >= 1150.0f && s_seg_mm <= 2250.0f) ? 1U : 0U;
      follow_line_pid(&scan, 0,
                      in_zig ? TRACK_ZIGZAG_SPEED : TRACK_BASE_SPEED,
                      in_zig ? NAV_GYRO_DAMP_ZIG : 0.0f,
                      dt_ms, out_l, out_r);
      if (s_seg_mm >= 2350.0f) enter_state(TRK_FINISH_ARMED);
      break;
    }

    case TRK_FINISH_ARMED:
      follow_line_pid(&scan, 0, TRACK_BASE_SPEED, 0.0f, dt_ms, out_l, out_r);
      if (is_crossroad(&scan)) {
        enter_state(TRK_FINISH_BAR1);
      }
      break;

    case TRK_FINISH_BAR1:
      follow_line_pid(&scan, 0, TRACK_BASE_SPEED, 0.0f, dt_ms, out_l, out_r);
      if (s_seg_mm >= 320.0f && is_crossroad(&scan)) {
        enter_state(TRK_DONE);
      }
      break;

    case TRK_DONE:
    case TRK_FAULT_STOP:
    default:
      *out_l = 0.0f;
      *out_r = 0.0f;
      return 1U;
  }

  return 0U;
}

TrackState TrackNav_GetState(void) { return s_state; }
float TrackNav_GetYawDeg(void) { return s_yaw_deg; }
float TrackNav_GetSegMm(void) { return s_seg_mm; }
