#include "track_nav.h"
#include "main.h"
#include "multiplexer.h"
#include "dodo_BMI270.h"
#include <math.h>
#include <stddef.h>

/* 陀螺仪角度比例与采样换算 */
#define NAV_GYRO_SCALE           0.9807f
#define NAV_GYRO_SENS            16.4f
#define NAV_GYRO_CAL_MS          800U    /* 配合发车前 200 ms 消抖，总计停 1 s */
#define NAV_LINE_ACTIVE          1U

/* 全赛道模式巡线 PD 参数（D 项按 dt_s 归一化） */
#define NAV_LINE_KP              1150.0f
#define NAV_LINE_KP2             115.0f
#define NAV_LINE_KD              0.55f
#define NAV_SEARCH_SPEED         34000.0f /* 丢线后外轮寻线目标速度 (counts/s) */
#define NAV_SEARCH_REV_SPEED     15000.0f /* 丢线后内轮反转目标速度幅值 (counts/s) */
#define NAV_SEARCH_SWEEP_MS      700U     /* 单向寻线超时后反向扫线 */
#define NAV_SEARCH_SWEEP_DEG     135.0f   /* 单向寻线最大转角后反向扫线 */
#define NAV_GYRO_DAMP_NORM       6.0f     /* 常规巡线轻度角速度阻尼 */
#define NAV_GYRO_DAMP_ZIG        14.0f    /* 锯齿段陀螺仪角速度阻尼 */
#define NAV_YAW_HOLD_KP          160.0f   /* 锁航向直行纠偏增益 (counts/s per deg) */

/* 路口与断路几何参数（单位: mm） */
#define NAV_CROSS_PASS_MM        95.0f    /* 穿十字直行距离 */
#define NAV_T_PASS_MM            90.0f    /* 过 T 口直行距离 */
#define NAV_BROKEN_CLEAR_MM      100.0f   /* 断路直行离端头距离 */
#define NAV_BROKEN_SEEK_MM       480.0f   /* 断路 +15° 斜插距离 */
#define NAV_BROKEN_SWEEP_MM      720.0f   /* 超过后进一步扩大扫线角度 */
#define NAV_BROKEN_ALIGN_MIN_MM  60.0f    /* 抓到对面线后最小巡线对齐距离 */
#define NAV_BROKEN_ALIGN_MAX_MM  180.0f   /* 抓线后超过该距离直接切入下一段 */
#define NAV_BROKEN_OFFSET_DEG    15.0f    /* 右->左过断路向右(北)偏转 15° */
#define NAV_TURN_FALLBACK_MS     2500U    /* 转弯兜底超时：超时自动继续而不锁死停车 */
#define NAV_FINISH_ACTIVE_COUNT  8U       /* 实物横线通常至少 8 路有效 */
#define NAV_FINISH_DEBOUNCE_MS   6U

static TrackState s_state = TRK_START_CLEAR;
static TrackState s_fault_state = TRK_START_CLEAR;
static TrackFault s_fault = TRACK_FAULT_NONE;
static uint16_t s_last_line_mask = 0U;
static uint8_t s_last_line_count = 0U;
static uint32_t s_state_tick = 0U;
static uint16_t s_finish_active_ms = 0U;
static uint16_t s_finish_clear_ms = 0U;
static uint8_t s_finish_released = 0U;
static uint8_t s_gyro_ok = 0U;
static float s_gyro_bias = 0.0f;
static float s_last_raw_rate = 0.0f;
static float s_yaw_deg = 0.0f;       /* 全局累计航向角（顺时针为正） */
static float s_yaw_seg = 0.0f;       /* 当前状态累计转角 */
static float s_rate_lpf = 0.0f;      /* 低通角速度（°/s，顺时针为正） */
static float s_seg_mm = 0.0f;        /* 当前状态累计里程（mm） */
static float s_yaw_ref = 0.0f;       /* 参考航向（锁直行/断路/转向用） */
static float s_yaw_west = 0.0f;      /* 过断路及左上 T 型方框的西向基准航向 */

static float s_prev_err = 0.0f;
static uint8_t s_has_prev_err = 0U;
static int8_t s_last_dir = 0;        /* 持久保留最近有效偏差方向：-1 左, +1 右 */
static int8_t s_search_dir = 0;      /* 本次寻线方向 */
static uint8_t s_search_active = 0U;
static float s_search_start_yaw = 0.0f;
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
  s_state_tick = HAL_GetTick();
  s_seg_mm = 0.0f;
  s_yaw_seg = 0.0f;
  s_cross_debounce = 0U;
  s_broken_miss_cnt = 0U;
  if (next == TRK_FINISH_ARMED || next == TRK_FINISH_BAR1) {
    s_finish_active_ms = 0U;
    s_finish_clear_ms = 0U;
    s_finish_released = 0U;
  }
}

void TrackNav_Abort(TrackFault fault) {
  if (s_state != TRK_FAULT_STOP) {
    s_fault_state = s_state;
    s_fault = fault;
    s_state = TRK_FAULT_STOP;
  }
}

static uint16_t accumulate_ms(uint16_t current, float dt_ms, uint16_t limit) {
  uint32_t add = (dt_ms > 0.0f) ? (uint32_t)(dt_ms + 0.5f) : 0U;
  uint32_t value = (uint32_t)current + add;
  return (value >= limit) ? limit : (uint16_t)value;
}

static uint8_t debounce_cond(uint8_t cond, uint8_t need) {
  if (cond) {
    if (s_cross_debounce < need) {
      ++s_cross_debounce;
    }
    return (s_cross_debounce >= need) ? 1U : 0U;
  }
  s_cross_debounce = 0U;
  return 0U;
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
                            float base_target, float gyro_damp, float dt_s,
                            float *out_l, float *out_r) {
  float err;
  float err_rate;
  float abs_err;
  float abs_diff_5ms;
  float severity;
  float scale;
  float base_spd;
  float corr;
  float dt_ms = dt_s * 1000.0f;

  if (scan->cluster_count == 0U) {
    s_has_prev_err = 0U;
    if (!s_search_active) {
      s_search_dir = (s_last_dir != 0) ? s_last_dir : ((s_rate_lpf < 0.0f) ? -1 : 1);
      s_search_start_yaw = s_yaw_deg;
      s_search_active = 1U;
      s_lost_ms = 0U;
    }
    s_lost_ms = accumulate_ms(s_lost_ms, dt_ms, 60000U);
    if (s_lost_ms >= NAV_SEARCH_SWEEP_MS ||
        fabsf(s_yaw_deg - s_search_start_yaw) >= NAV_SEARCH_SWEEP_DEG) {
      /* 比赛不自杀停车：单方向扫不到线时自动反向来回扫线 */
      s_search_dir = (int8_t)(-s_search_dir);
      s_search_start_yaw = s_yaw_deg;
      s_lost_ms = 0U;
    }
    *out_l = (s_search_dir > 0) ? NAV_SEARCH_SPEED : -NAV_SEARCH_REV_SPEED;
    *out_r = (s_search_dir < 0) ? NAV_SEARCH_SPEED : -NAV_SEARCH_REV_SPEED;
    return;
  }

  s_lost_ms = 0U;
  s_search_active = 0U;
  s_search_dir = 0;

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

  if (err > 0.3f) s_last_dir = 1;
  else if (err < -0.3f) s_last_dir = -1;

  err_rate = s_has_prev_err ? ((err - s_prev_err) / dt_s) : 0.0f;
  s_prev_err = err;
  s_has_prev_err = 1U;

  abs_err = fabsf(err);
  corr = NAV_LINE_KP * err + NAV_LINE_KP2 * err * abs_err +
         NAV_LINE_KD * err_rate - gyro_damp * s_rate_lpf;

  abs_diff_5ms = fabsf(err_rate) * 0.005f;
  severity = abs_err;
  if (abs_diff_5ms * 2.5f > severity) severity = abs_diff_5ms * 2.5f;

  scale = 1.0f - 0.14f * (severity - 3.5f);
  scale = clamp_f(scale, 0.30f, 1.0f);
  base_spd = base_target * scale;

  *out_l = clamp_f(base_spd + corr, -TRACK_MAX_REV_SPEED, TRACK_MAX_SPEED);
  *out_r = clamp_f(base_spd - corr, -TRACK_MAX_REV_SPEED, TRACK_MAX_SPEED);
}

/* 陀螺仪锁航向直行 */
static void follow_yaw_hold(float target_yaw, float speed, float *out_l, float *out_r) {
  float yaw_err = target_yaw - s_yaw_deg;
  float corr = NAV_YAW_HOLD_KP * yaw_err - 8.0f * s_rate_lpf;
  *out_l = clamp_f(speed + corr, -TRACK_MAX_REV_SPEED, TRACK_MAX_SPEED);
  *out_r = clamp_f(speed - corr, -TRACK_MAX_REV_SPEED, TRACK_MAX_SPEED);
  s_has_prev_err = 0U;
}

/* 双向差速定角转向（外轮正转 + 内轮反转）；返回 1 表示到达目标或捕获线 */
static uint8_t gyro_turn_step(float target_yaw, int8_t dir_sign, float min_turn_deg,
                              const LineScanResult *scan, float *out_l, float *out_r) {
  float rem = (target_yaw - s_yaw_deg) * (float)dir_sign;
  float turned = s_yaw_seg * (float)dir_sign;
  uint32_t now = HAL_GetTick();
  s_has_prev_err = 0U;
  s_last_dir = dir_sign;

  if (turned >= min_turn_deg && scan->cluster_count >= 1U) {
    uint8_t idx = (dir_sign < 0) ? 0U : (uint8_t)(scan->cluster_count - 1U);
    if (fabsf(scan->clusters[idx].center) <= 6.5f) {
      return 1U;
    }
  }
  if (rem <= 6.0f || (uint32_t)(now - s_state_tick) >= NAV_TURN_FALLBACK_MS) {
    return 1U;
  }

  if (dir_sign > 0) {
    *out_l = TRACK_TURN_OUTER_SPEED;
    *out_r = -TRACK_TURN_INNER_REV_SPEED;
  } else {
    *out_l = -TRACK_TURN_INNER_REV_SPEED;
    *out_r = TRACK_TURN_OUTER_SPEED;
  }
  return 0U;
}

void TrackNav_Start(void) {
  int32_t sum = 0;
  uint32_t count = 0U;
  uint32_t start_tick;

  s_state = TRK_START_CLEAR;
  s_yaw_deg = 0.0f;
  s_yaw_seg = 0.0f;
  s_rate_lpf = 0.0f;
  s_last_raw_rate = 0.0f;
  s_seg_mm = 0.0f;
  s_yaw_ref = 0.0f;
  s_yaw_west = 0.0f;
  s_prev_err = 0.0f;
  s_has_prev_err = 0U;
  s_last_dir = 0;
  s_search_dir = 0;
  s_search_active = 0U;
  s_lost_ms = 0U;
  s_fault = TRACK_FAULT_NONE;
  s_fault_state = TRK_START_CLEAR;
  s_last_line_mask = 0U;
  s_last_line_count = 0U;
  s_gyro_bias = 0.0f;

  s_gyro_ok = (dodo_BMI270_init() == 0) ? 1U : 0U;
  start_tick = HAL_GetTick();
  while ((uint32_t)(HAL_GetTick() - start_tick) < NAV_GYRO_CAL_MS) {
    if (s_gyro_ok) {
      int16_t gz = 0;
      if (dodo_BMI270_get_gyro_z(&gz) == 0) {
        sum += (int32_t)gz;
        ++count;
      }
    }
    HAL_Delay(2U);
  }
  if (count > 0U) {
    s_gyro_bias = (float)sum / (float)count;
  }
  enter_state(TRK_START_CLEAR);
}

uint8_t TrackNav_Step(int32_t left_delta, int32_t right_delta, float dt_s,
                      float *out_l, float *out_r) {
  LineScanResult scan;
  float raw_rate = s_last_raw_rate;
  float step_mm;

  *out_l = 0.0f;
  *out_r = 0.0f;

  if (s_state == TRK_DONE || s_state == TRK_FAULT_STOP) {
    return 1U;
  }

  if (s_gyro_ok) {
    int16_t gz = 0;
    if (dodo_BMI270_get_gyro_z(&gz) == 0) {
      raw_rate = -((float)gz - s_gyro_bias) / NAV_GYRO_SENS;
      s_last_raw_rate = raw_rate;
    }
  } else {
    raw_rate = 0.0f;
  }

  s_yaw_deg += raw_rate * dt_s * NAV_GYRO_SCALE;
  s_yaw_seg += raw_rate * dt_s * NAV_GYRO_SCALE;
  s_rate_lpf = 0.65f * s_rate_lpf + 0.35f * raw_rate;

  step_mm = 500.0f * ((float)left_delta / TRACK_LEFT_COUNTS_PER_METER +
                      (float)right_delta / TRACK_RIGHT_COUNTS_PER_METER);
  s_seg_mm += step_mm;

  TrackNav_ScanLine(NAV_LINE_ACTIVE, &scan);
  s_last_line_mask = scan.mux_value;
  s_last_line_count = scan.active_count;

  switch (s_state) {
    /* 1. 发车区起步屏蔽双横线（前 900 mm 不检测十字/终点） */
    case TRK_START_CLEAR:
      follow_line_pid(&scan, 0, TRACK_BASE_SPEED, NAV_GYRO_DAMP_NORM, dt_s, out_l, out_r);
      if (s_seg_mm >= 900.0f) {
        enter_state(TRK_ZIGZAG_1);
      }
      break;

    /* 2. 右下 2 峰锯齿 + 右下折角升至右中 250 方框十字 */
    case TRK_ZIGZAG_1: {
      float spd = (s_seg_mm < 1550.0f) ? TRACK_ZIGZAG_SPEED : TRACK_BASE_SPEED;
      float damp = (s_seg_mm < 1550.0f) ? NAV_GYRO_DAMP_ZIG : NAV_GYRO_DAMP_NORM;
      follow_line_pid(&scan, 0, spd, damp, dt_s, out_l, out_r);
      if (debounce_cond((s_seg_mm >= 2200.0f && is_crossroad(&scan)) ? 1U : 0U, 2U)) {
        s_yaw_ref = s_yaw_deg;
        enter_state(TRK_SQ_RM_IN);
      }
      break;
    }

    /* 3. 右中 250x250 方框（十字直行进 -> 绕圈 -> 十字直行出） */
    case TRK_SQ_RM_IN:
      follow_yaw_hold(s_yaw_ref, TRACK_CROSS_SPEED, out_l, out_r);
      if (s_seg_mm >= NAV_CROSS_PASS_MM) enter_state(TRK_SQ_RM_LOOP);
      break;

    case TRK_SQ_RM_LOOP:
      follow_line_pid(&scan, 0, TRACK_BASE_SPEED, NAV_GYRO_DAMP_NORM, dt_s, out_l, out_r);
      if (debounce_cond((s_seg_mm >= 750.0f && is_crossroad(&scan)) ? 1U : 0U, 2U)) {
        s_yaw_ref = s_yaw_deg;
        enter_state(TRK_SQ_RM_OUT);
      }
      break;

    case TRK_SQ_RM_OUT:
      follow_yaw_hold(s_yaw_ref, TRACK_CROSS_SPEED, out_l, out_r);
      if (s_seg_mm >= NAV_CROSS_PASS_MM) enter_state(TRK_APPROACH_HEX_R);
      break;

    /* 4. 右中 200 六边形（30° 左偏入环 -> 绕一圈 -> 左偏出到 330） */
    case TRK_APPROACH_HEX_R:
      follow_line_pid(&scan, 0, TRACK_BASE_SPEED, NAV_GYRO_DAMP_NORM, dt_s, out_l, out_r);
      if (debounce_cond((s_seg_mm >= 260.0f && is_branch_or_cross(&scan)) ? 1U : 0U, 2U)) {
        s_yaw_ref = s_yaw_deg;
        enter_state(TRK_HEX_R_ENTER);
      }
      break;

    case TRK_HEX_R_ENTER:
      follow_line_pid(&scan, -1, TRACK_HEX_R_SPEED, NAV_GYRO_DAMP_NORM, dt_s, out_l, out_r);
      if (s_seg_mm >= 130.0f) enter_state(TRK_HEX_R_LOOP);
      break;

    case TRK_HEX_R_LOOP:
      follow_line_pid(&scan, 0, TRACK_HEX_R_SPEED, NAV_GYRO_DAMP_NORM, dt_s, out_l, out_r);
      if (debounce_cond((s_seg_mm >= 900.0f && s_yaw_seg <= -245.0f && is_branch_or_cross(&scan)) ? 1U : 0U, 2U)) {
        enter_state(TRK_HEX_R_EXIT);
      }
      break;

    case TRK_HEX_R_EXIT:
      follow_line_pid(&scan, -1, TRACK_HEX_R_SPEED, NAV_GYRO_DAMP_NORM, dt_s, out_l, out_r);
      if (s_seg_mm >= 140.0f) enter_state(TRK_APPROACH_OCT);
      break;

    /* 5. 右上八边形（十字直行进 -> 绕一圈 -> 十字直行出） */
    case TRK_APPROACH_OCT:
      follow_line_pid(&scan, 0, TRACK_BASE_SPEED, NAV_GYRO_DAMP_NORM, dt_s, out_l, out_r);
      if (debounce_cond((s_seg_mm >= 1450.0f && is_crossroad(&scan)) ? 1U : 0U, 2U)) {
        s_yaw_ref = s_yaw_deg;
        enter_state(TRK_OCT_IN);
      }
      break;

    case TRK_OCT_IN:
      follow_yaw_hold(s_yaw_ref, TRACK_CROSS_SPEED, out_l, out_r);
      if (s_seg_mm >= NAV_CROSS_PASS_MM) enter_state(TRK_OCT_LOOP);
      break;

    case TRK_OCT_LOOP:
      follow_line_pid(&scan, 0, TRACK_HEX_SPEED, NAV_GYRO_DAMP_NORM, dt_s, out_l, out_r);
      if (debounce_cond((s_seg_mm >= 1550.0f && is_crossroad(&scan)) ? 1U : 0U, 2U)) {
        s_yaw_ref = s_yaw_deg;
        enter_state(TRK_OCT_OUT);
      }
      break;

    case TRK_OCT_OUT:
      follow_yaw_hold(s_yaw_ref, TRACK_CROSS_SPEED, out_l, out_r);
      if (s_seg_mm >= NAV_CROSS_PASS_MM) enter_state(TRK_APPROACH_BROKEN);
      break;

    /* 6. 前往并跨越顶部断路区域（650 mm 且目标线偏北/右前方 ~130 mm） */
    case TRK_APPROACH_BROKEN:
      if (s_seg_mm >= 850.0f && scan.cluster_count == 0U) {
        if (s_broken_miss_cnt == 0U) {
          s_yaw_west = s_yaw_deg;
          s_yaw_ref = s_yaw_west;
        }
        ++s_broken_miss_cnt;
        follow_yaw_hold(s_yaw_west, TRACK_BROKEN_SPEED, out_l, out_r);
        if (s_broken_miss_cnt >= 3U) {
          enter_state(TRK_BROKEN_CLEAR);
        }
      } else {
        s_broken_miss_cnt = 0U;
        follow_line_pid(&scan, 0, TRACK_BASE_SPEED, NAV_GYRO_DAMP_NORM, dt_s, out_l, out_r);
      }
      break;

    case TRK_BROKEN_CLEAR:
      follow_yaw_hold(s_yaw_west, TRACK_BROKEN_SPEED, out_l, out_r);
      if (s_seg_mm >= NAV_BROKEN_CLEAR_MM) enter_state(TRK_BROKEN_SEEK);
      break;

    case TRK_BROKEN_SEEK: {
      float seek_yaw = s_yaw_west + NAV_BROKEN_OFFSET_DEG;
      if (s_seg_mm > NAV_BROKEN_SEEK_MM) {
        uint32_t phase = (uint32_t)((s_seg_mm - NAV_BROKEN_SEEK_MM) / 180.0f);
        if (s_seg_mm > NAV_BROKEN_SWEEP_MM)
          seek_yaw = ((phase & 1U) == 0U) ? (s_yaw_west - 28.0f) : (s_yaw_west + 35.0f);
        else
          seek_yaw = ((phase & 1U) == 0U) ? (s_yaw_west - 20.0f) : (s_yaw_west + 28.0f);
      }
      follow_yaw_hold(seek_yaw, TRACK_BROKEN_SPEED, out_l, out_r);
      if (s_seg_mm >= 220.0f && scan.active_count >= 1U) {
        enter_state(TRK_BROKEN_ALIGN);
      }
      break;
    }

    case TRK_BROKEN_ALIGN:
      /* 抓到线立刻用循迹 PID 回正，不继续盲走 */
      follow_line_pid(&scan, 0, TRACK_BROKEN_SPEED, NAV_GYRO_DAMP_NORM, dt_s, out_l, out_r);
      if (scan.cluster_count >= 1U &&
          ((s_seg_mm >= NAV_BROKEN_ALIGN_MIN_MM && fabsf(s_yaw_deg - s_yaw_west) <= 25.0f) ||
           s_seg_mm >= NAV_BROKEN_ALIGN_MAX_MM)) {
        enter_state(TRK_TL_SQ_PRE_T1);
      } else if (s_search_active && s_lost_ms >= 150U) {
        enter_state(TRK_BROKEN_SEEK);
      }
      break;

    /* 7. 左上 500 下方 250x250 T 型方框（必须绕行完整四边） */
    case TRK_TL_SQ_PRE_T1:
      follow_line_pid(&scan, 0, TRACK_BASE_SPEED, NAV_GYRO_DAMP_NORM, dt_s, out_l, out_r);
      if (scan.cluster_count == 1U && s_seg_mm >= 40.0f && fabsf(s_prev_err) <= 2.5f) {
        /* 沿左上直道时刷新西向基准航向，消除断路前累计漂移 */
        s_yaw_west = s_yaw_deg;
      }
      if (debounce_cond((s_seg_mm >= 120.0f && is_left_branch(&scan)) ? 1U : 0U, 2U)) {
        enter_state(TRK_TL_SQ_T1_PASS);
      }
      break;

    case TRK_TL_SQ_T1_PASS:
      follow_yaw_hold(s_yaw_west, TRACK_CROSS_SPEED, out_l, out_r);
      if (s_seg_mm >= NAV_T_PASS_MM) enter_state(TRK_TL_SQ_TO_BL1);
      break;

    case TRK_TL_SQ_TO_BL1:
      follow_line_pid(&scan, 0, TRACK_HEX_SPEED, NAV_GYRO_DAMP_NORM, dt_s, out_l, out_r);
      if (debounce_cond((s_seg_mm >= 260.0f && (s_yaw_deg - s_yaw_west) <= -65.0f &&
                         is_branch_or_cross(&scan)) ? 1U : 0U, 2U)) {
        enter_state(TRK_TL_SQ_TURN_E);
      }
      break;

    case TRK_TL_SQ_TURN_E:
      if (gyro_turn_step(s_yaw_west - 180.0f, -1, 65.0f, &scan, out_l, out_r)) {
        enter_state(TRK_TL_SQ_TO_TR2);
        follow_line_pid(&scan, 0, TRACK_HEX_SPEED, NAV_GYRO_DAMP_NORM, dt_s, out_l, out_r);
      }
      break;

    case TRK_TL_SQ_TO_TR2:
      follow_line_pid(&scan, 0, TRACK_HEX_SPEED, NAV_GYRO_DAMP_NORM, dt_s, out_l, out_r);
      if (debounce_cond((s_seg_mm >= 340.0f && (s_yaw_deg - s_yaw_west) <= -245.0f &&
                         is_branch_or_cross(&scan)) ? 1U : 0U, 2U)) {
        enter_state(TRK_TL_SQ_TURN_W);
      }
      break;

    case TRK_TL_SQ_TURN_W:
      if (gyro_turn_step(s_yaw_west - 360.0f, -1, 65.0f, &scan, out_l, out_r)) {
        enter_state(TRK_TL_SQ_TO_BL2);
        follow_line_pid(&scan, 0, TRACK_HEX_SPEED, NAV_GYRO_DAMP_NORM, dt_s, out_l, out_r);
      }
      break;

    case TRK_TL_SQ_TO_BL2:
      follow_line_pid(&scan, 0, TRACK_HEX_SPEED, NAV_GYRO_DAMP_NORM, dt_s, out_l, out_r);
      if (debounce_cond((s_seg_mm >= 340.0f && (s_yaw_deg - s_yaw_west) <= -425.0f &&
                         is_branch_or_cross(&scan)) ? 1U : 0U, 2U)) {
        enter_state(TRK_TL_SQ_EXIT_W);
      }
      break;

    case TRK_TL_SQ_EXIT_W:
      if (gyro_turn_step(s_yaw_west - 360.0f, +1, 65.0f, &scan, out_l, out_r)) {
        enter_state(TRK_LEFT_SQ1_PRE);
        follow_line_pid(&scan, 0, TRACK_BASE_SPEED, NAV_GYRO_DAMP_NORM, dt_s, out_l, out_r);
      }
      break;

    /* 8. 左侧三个十字方框（350x350 -> 250x250 -> 250x250） */
    case TRK_LEFT_SQ1_PRE:
      follow_line_pid(&scan, 0, TRACK_BASE_SPEED, NAV_GYRO_DAMP_NORM, dt_s, out_l, out_r);
      if (debounce_cond((s_seg_mm >= 140.0f && is_crossroad(&scan)) ? 1U : 0U, 2U)) {
        s_yaw_ref = s_yaw_deg;
        enter_state(TRK_LEFT_SQ1_IN);
      }
      break;
    case TRK_LEFT_SQ1_IN:
      follow_yaw_hold(s_yaw_ref, TRACK_CROSS_SPEED, out_l, out_r);
      if (s_seg_mm >= NAV_CROSS_PASS_MM) enter_state(TRK_LEFT_SQ1_LOOP);
      break;
    case TRK_LEFT_SQ1_LOOP:
      follow_line_pid(&scan, 0, TRACK_BASE_SPEED, NAV_GYRO_DAMP_NORM, dt_s, out_l, out_r);
      if (debounce_cond((s_seg_mm >= 1050.0f && is_crossroad(&scan)) ? 1U : 0U, 2U)) {
        s_yaw_ref = s_yaw_deg;
        enter_state(TRK_LEFT_SQ1_OUT);
      }
      break;
    case TRK_LEFT_SQ1_OUT:
      follow_yaw_hold(s_yaw_ref, TRACK_CROSS_SPEED, out_l, out_r);
      if (s_seg_mm >= NAV_CROSS_PASS_MM) enter_state(TRK_LEFT_SQ2_PRE);
      break;

    case TRK_LEFT_SQ2_PRE:
      follow_line_pid(&scan, 0, TRACK_BASE_SPEED, NAV_GYRO_DAMP_NORM, dt_s, out_l, out_r);
      if (debounce_cond((s_seg_mm >= 360.0f && is_crossroad(&scan)) ? 1U : 0U, 2U)) {
        s_yaw_ref = s_yaw_deg;
        enter_state(TRK_LEFT_SQ2_IN);
      }
      break;
    case TRK_LEFT_SQ2_IN:
      follow_yaw_hold(s_yaw_ref, TRACK_CROSS_SPEED, out_l, out_r);
      if (s_seg_mm >= NAV_CROSS_PASS_MM) enter_state(TRK_LEFT_SQ2_LOOP);
      break;
    case TRK_LEFT_SQ2_LOOP:
      follow_line_pid(&scan, 0, TRACK_BASE_SPEED, NAV_GYRO_DAMP_NORM, dt_s, out_l, out_r);
      if (debounce_cond((s_seg_mm >= 750.0f && is_crossroad(&scan)) ? 1U : 0U, 2U)) {
        s_yaw_ref = s_yaw_deg;
        enter_state(TRK_LEFT_SQ2_OUT);
      }
      break;
    case TRK_LEFT_SQ2_OUT:
      follow_yaw_hold(s_yaw_ref, TRACK_CROSS_SPEED, out_l, out_r);
      if (s_seg_mm >= NAV_CROSS_PASS_MM) enter_state(TRK_LEFT_SQ3_PRE);
      break;

    case TRK_LEFT_SQ3_PRE:
      follow_line_pid(&scan, 0, TRACK_BASE_SPEED, NAV_GYRO_DAMP_NORM, dt_s, out_l, out_r);
      if (debounce_cond((s_seg_mm >= 180.0f && is_crossroad(&scan)) ? 1U : 0U, 2U)) {
        s_yaw_ref = s_yaw_deg;
        enter_state(TRK_LEFT_SQ3_IN);
      }
      break;
    case TRK_LEFT_SQ3_IN:
      follow_yaw_hold(s_yaw_ref, TRACK_CROSS_SPEED, out_l, out_r);
      if (s_seg_mm >= NAV_CROSS_PASS_MM) enter_state(TRK_LEFT_SQ3_LOOP);
      break;
    case TRK_LEFT_SQ3_LOOP:
      follow_line_pid(&scan, 0, TRACK_BASE_SPEED, NAV_GYRO_DAMP_NORM, dt_s, out_l, out_r);
      if (debounce_cond((s_seg_mm >= 750.0f && is_crossroad(&scan)) ? 1U : 0U, 2U)) {
        s_yaw_ref = s_yaw_deg;
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
                      TRACK_HEX_SPEED, NAV_GYRO_DAMP_NORM, dt_s, out_l, out_r);
      if (debounce_cond((s_seg_mm >= 580.0f && is_branch_or_cross(&scan)) ? 1U : 0U, 2U)) {
        s_yaw_ref = s_yaw_deg;
        enter_state(TRK_HEX_BL_IN);
      }
      break;

    case TRK_HEX_BL_IN:
      if (gyro_turn_step(s_yaw_ref + 60.0f, +1, 40.0f, &scan, out_l, out_r)) {
        enter_state(TRK_HEX_BL_LOOP);
        follow_line_pid(&scan, 0, TRACK_HEX_SPEED, NAV_GYRO_DAMP_NORM, dt_s, out_l, out_r);
      }
      break;

    case TRK_HEX_BL_LOOP:
      follow_line_pid(&scan, 0, TRACK_HEX_SPEED, NAV_GYRO_DAMP_NORM, dt_s, out_l, out_r);
      if (debounce_cond((s_seg_mm >= 1000.0f && s_yaw_seg >= 285.0f &&
                         is_branch_or_cross(&scan)) ? 1U : 0U, 2U)) {
        s_yaw_ref = s_yaw_deg;
        enter_state(TRK_HEX_BL_EXIT);
      }
      break;

    case TRK_HEX_BL_EXIT:
      if (gyro_turn_step(s_yaw_ref - 120.0f, -1, 95.0f, &scan, out_l, out_r)) {
        enter_state(TRK_APPROACH_HEX_MID);
        follow_line_pid(&scan, 0, TRACK_BASE_SPEED, NAV_GYRO_DAMP_NORM, dt_s, out_l, out_r);
      }
      break;

    /* 10. 中左 300+200 双六边形 8 字一笔画 */
    case TRK_APPROACH_HEX_MID:
      follow_line_pid(&scan, 0, TRACK_BASE_SPEED, NAV_GYRO_DAMP_NORM, dt_s, out_l, out_r);
      if (debounce_cond((s_seg_mm >= 700.0f && is_branch_or_cross(&scan)) ? 1U : 0U, 2U)) {
        enter_state(TRK_HEX_300_ENTER);
      }
      break;

    case TRK_HEX_300_ENTER:
      follow_line_pid(&scan, -1, TRACK_HEX_SPEED, NAV_GYRO_DAMP_NORM, dt_s, out_l, out_r);
      if (s_seg_mm >= 130.0f) enter_state(TRK_HEX_300_R);
      break;

    case TRK_HEX_300_R:
      follow_line_pid(&scan, 0, TRACK_HEX_SPEED, NAV_GYRO_DAMP_NORM, dt_s, out_l, out_r);
      if (debounce_cond((s_seg_mm >= 440.0f && is_branch_or_cross(&scan)) ? 1U : 0U, 2U)) {
        s_yaw_ref = s_yaw_deg;
        enter_state(TRK_HEX_200_IN);
      }
      break;

    case TRK_HEX_200_IN:
      if (gyro_turn_step(s_yaw_ref + 60.0f, +1, 40.0f, &scan, out_l, out_r)) {
        enter_state(TRK_HEX_200_LOOP);
        follow_line_pid(&scan, 0, TRACK_HEX_SPEED, NAV_GYRO_DAMP_NORM, dt_s, out_l, out_r);
      }
      break;

    case TRK_HEX_200_LOOP:
      follow_line_pid(&scan, 0, TRACK_HEX_SPEED, NAV_GYRO_DAMP_NORM, dt_s, out_l, out_r);
      if (debounce_cond((s_seg_mm >= 980.0f && s_yaw_seg >= 260.0f &&
                         is_branch_or_cross(&scan)) ? 1U : 0U, 2U)) {
        s_yaw_ref = s_yaw_deg;
        enter_state(TRK_HEX_200_CROSS);
      }
      break;

    case TRK_HEX_200_CROSS:
      follow_yaw_hold(s_yaw_ref, TRACK_CROSS_SPEED, out_l, out_r);
      if (s_seg_mm >= 85.0f) enter_state(TRK_HEX_300_L);
      break;

    case TRK_HEX_300_L:
      follow_line_pid(&scan, 0, TRACK_HEX_SPEED, NAV_GYRO_DAMP_NORM, dt_s, out_l, out_r);
      if (debounce_cond((s_seg_mm >= 720.0f && s_yaw_seg <= -90.0f &&
                         is_branch_or_cross(&scan)) ? 1U : 0U, 2U)) {
        enter_state(TRK_HEX_300_EXIT);
      }
      break;

    case TRK_HEX_300_EXIT:
      follow_line_pid(&scan, -1, TRACK_HEX_SPEED, NAV_GYRO_DAMP_NORM, dt_s, out_l, out_r);
      if (s_seg_mm >= 140.0f) enter_state(TRK_ZIGZAG_2);
      break;

    /* 11. 收尾：530+550+240 + 3峰锯齿 + 490 回发车区双横线停车 */
    case TRK_ZIGZAG_2: {
      uint8_t in_zig = (s_seg_mm >= 1150.0f && s_seg_mm <= 2250.0f) ? 1U : 0U;
      follow_line_pid(&scan, 0,
                      in_zig ? TRACK_ZIGZAG_SPEED : TRACK_BASE_SPEED,
                      in_zig ? NAV_GYRO_DAMP_ZIG : NAV_GYRO_DAMP_NORM,
                      dt_s, out_l, out_r);
      if (s_seg_mm >= 2350.0f) enter_state(TRK_FINISH_ARMED);
      break;
    }

    case TRK_FINISH_ARMED:
      follow_line_pid(&scan, 0, TRACK_BASE_SPEED, NAV_GYRO_DAMP_NORM, dt_s, out_l, out_r);
      if (scan.active_count >= NAV_FINISH_ACTIVE_COUNT)
        s_finish_active_ms = accumulate_ms(s_finish_active_ms, dt_s * 1000.0f, NAV_FINISH_DEBOUNCE_MS);
      else s_finish_active_ms = 0U;
      if (s_finish_active_ms >= NAV_FINISH_DEBOUNCE_MS) enter_state(TRK_FINISH_BAR1);
      break;

    case TRK_FINISH_BAR1:
      follow_line_pid(&scan, 0, TRACK_BASE_SPEED, NAV_GYRO_DAMP_NORM, dt_s, out_l, out_r);
      if (!s_finish_released) {
        if (scan.active_count < NAV_FINISH_ACTIVE_COUNT)
          s_finish_clear_ms = accumulate_ms(s_finish_clear_ms, dt_s * 1000.0f, NAV_FINISH_DEBOUNCE_MS);
        else s_finish_clear_ms = 0U;
        if (s_finish_clear_ms >= NAV_FINISH_DEBOUNCE_MS) s_finish_released = 1U;
      } else if (s_seg_mm >= 320.0f && scan.active_count >= NAV_FINISH_ACTIVE_COUNT) {
        s_finish_active_ms = accumulate_ms(s_finish_active_ms, dt_s * 1000.0f, NAV_FINISH_DEBOUNCE_MS);
        if (s_finish_active_ms >= NAV_FINISH_DEBOUNCE_MS) enter_state(TRK_DONE);
      } else {
        s_finish_active_ms = 0U;
      }
      break;

    case TRK_DONE:
    case TRK_FAULT_STOP:
    default:
      *out_l = 0.0f;
      *out_r = 0.0f;
      return 1U;
  }

  if (s_state == TRK_DONE || s_state == TRK_FAULT_STOP) {
    *out_l = 0.0f;
    *out_r = 0.0f;
    return 1U;
  }
  return 0U;
}

TrackState TrackNav_GetState(void) { return s_state; }
float TrackNav_GetYawDeg(void) { return s_yaw_deg; }
float TrackNav_GetSegMm(void) { return s_seg_mm; }
TrackFault TrackNav_GetFault(void) { return s_fault; }
TrackState TrackNav_GetFaultState(void) { return s_fault_state; }
uint16_t TrackNav_GetLastLineMask(void) { return s_last_line_mask; }
uint8_t TrackNav_GetLastLineCount(void) { return s_last_line_count; }
uint8_t TrackNav_IsSearching(void) { return s_search_active; }
int8_t TrackNav_GetSearchDirection(void) { return s_search_dir; }
