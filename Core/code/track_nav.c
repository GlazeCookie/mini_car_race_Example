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

/* 全赛道模式巡线：三环串级 PID（位置外环 PD -> 目标偏航角速度 -> 陀螺仪角速度 PID -> 轮速环） */
#define NAV_LINE_RATE_KP         48.0f    /* 位置外环线性比例 (°/s per err) */
#define NAV_LINE_RATE_KP2        4.5f     /* 位置外环大偏差二次项 (°/s per err^2) */
#define NAV_LINE_RATE_KD         0.050f   /* 位置外环微分超前 (°/s per (err/s)) */
#define NAV_LINE_RATE_KD_LIM     140.0f   /* 位置外环微分限幅 (°/s)，提升入弯甩尾与出弯制动 */
#define NAV_FOLLOW_MAX_REV_SPEED 26000.0f /* 正常巡线内轮允许最大反转速度 (counts/s, ≈0.70 m/s)，强化急弯甩尾 */
#define NAV_CORNER_MAX_REV_SPEED 38000.0f /* 单侧 90° 直角折角内轮反甩上限 (counts/s, ≈1.02 m/s) */
#define NAV_FOLLOW_MAX_RATE_DPS  495.0f   /* 正常弯道巡线目标角速度上限 (°/s) */
#define NAV_FOLLOW_ZIG_RATE_DPS  429.0f   /* 锯齿急弯目标角速度上限 (°/s)，保持高节奏同时防掉头 */
#define NAV_CORNER_MAX_RATE_DPS  572.0f   /* 90° 直角折角目标角速度上限 (°/s) */

/* 陀螺仪角度外环 + 角速度内环 PID（全局复用于巡线与丢线弯道寻线） */
#define NAV_RATE_FF_GAIN         44.0f    /* 前馈：约 44 counts/s 差速对应 1 °/s 转向 */
#define NAV_RATE_KP              58.0f    /* 角速度环 P */
#define NAV_RATE_KI              260.0f   /* 角速度环 I */
#define NAV_RATE_I_LIMIT         18000.0f /* 定角寻线角速度积分限幅 */
#define NAV_FOLLOW_RATE_I_LIMIT  9000.0f  /* 连续巡线角速度积分限幅 */
#define NAV_TURN_ANGLE_KP        10.0f    /* 角度外环 P (°/s per deg) */

/* 丢线弯道定角寻线（由陀螺仪转向 PID 控速并把单向转角死死卡在 ±135° 内，绝不 180° 掉头） */
#define NAV_SEARCH_TARGET_DEG    135.0f   /* 覆盖 90° 与 120° 弯，低于 180° 防掉头 */
#define NAV_SEARCH_MAX_RATE_DPS  594.0f   /* 寻线最大旋转角速度 (°/s) */
#define NAV_SEARCH_MIN_RATE_DPS  121.0f   /* 接近边界时最小旋转角速度 (°/s) */
#define NAV_SEARCH_FWD_SPEED     4500.0f  /* 普通丢线轻弧线前移速度 (counts/s, ≈0.12 m/s)；直角触发后原地 0 速旋转 */
#define NAV_SEARCH_SWEEP_MS      1500U    /* 兜底超时：超时仍未找到则向对称边界回扫 */
#define NAV_RECAPTURE_MS         70U      /* 重新抓到线后的稳姿窗口 */
#define NAV_RECAPTURE_SPEED      31200.0f /* 抓线瞬间限速 (counts/s, ≈0.84 m/s) */
#define NAV_RECAPTURE_MAX_RATE   396.0f   /* 抓线窗口内目标角速度上限 (°/s) */
#define NAV_YAW_HOLD_KP          260.0f   /* 锁航向直行纠偏增益 (counts/s per deg) */

/* 路口与断路几何参数（单位: mm） */
#define NAV_CROSS_PASS_MM        95.0f    /* 穿十字直行距离 */
#define NAV_BROKEN_CLEAR_MM      100.0f   /* 断路直行离端头距离 */
#define NAV_BROKEN_SEEK_MM       580.0f   /* 断路 +20° 斜插距离（覆盖 650mm 跨距再开始回扫） */
#define NAV_BROKEN_SWEEP_MM      820.0f   /* 超过后进一步扩大扫线角度 */
#define NAV_BROKEN_ALIGN_MIN_MM  60.0f    /* 抓到对面线后最小巡线对齐距离 */
#define NAV_BROKEN_ALIGN_MAX_MM  180.0f   /* 抓线后超过该距离直接切入下一段 */
#define NAV_BROKEN_OFFSET_DEG    20.0f    /* 右->左过断路向右(北)偏转 20° */
#define NAV_FINISH_ACTIVE_COUNT  9U       /* 终点双横线至少 9 路有效且横跨两侧边界 */
#define NAV_FINISH_MAX_RATE_DPS  90.0f    /* 仅在直行姿态判定终点横线，过滤急弯与 90° 角 */
#define NAV_FINISH_DEBOUNCE_MS   6U

static TrackState s_state = TRK_START_CLEAR;
static TrackState s_fault_state = TRK_START_CLEAR;
static TrackFault s_fault = TRACK_FAULT_NONE;
static uint16_t s_last_line_mask = 0U;
static uint8_t s_last_line_count = 0U;
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
static float s_err_rate_lpf = 0.0f;
static uint8_t s_has_prev_err = 0U;
static int8_t s_last_dir = 0;        /* 持久保留最近明显偏差方向：-1 左, +1 右 */
static int8_t s_small_dir_cand = 0;  /* 中心微偏连续多帧才确认方向，滤除单灯闪烁 */
static uint8_t s_small_dir_cnt = 0U;
static int8_t s_search_dir = 0;      /* 本次寻线方向 */
static uint8_t s_search_active = 0U;
static uint8_t s_search_phase = 0U;  /* 0=首次按上次方向坚持寻线，1=兜底反向回扫 */
static float s_search_start_yaw = 0.0f;
static uint16_t s_lost_ms = 0U;
static uint16_t s_recapture_ms = 0U;
static float s_rate_pid_integral = 0.0f;
static uint8_t s_broken_miss_cnt = 0U;
static uint8_t s_cross_debounce = 0U;
static int8_t s_corner_dir = 0;      /* 单侧 90° 折角锁存方向（防离角残留灯触发反向 D 项/假翻向） */
static uint16_t s_corner_latch_ms = 0U;
static uint8_t s_corner_override_prev = 0U;
static float s_search_fwd_spd = NAV_SEARCH_FWD_SPEED;

static float clamp_f(float v, float lo, float hi) {
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

static void gyro_rate_pid_reset(void) {
  s_rate_pid_integral = 0.0f;
}

/* 陀螺仪角速度内环 PID（含物理前馈、反饱和积分与过弯回正泄放），将目标偏航角速度转为左右轮目标差速 */
static void gyro_rate_pid_update(float target_rate_dps, float measured_rate_dps,
                                 float forward_spd, float max_rev_spd,
                                 float i_limit, float dt_s,
                                 float *out_l, float *out_r) {
  float ff = NAV_RATE_FF_GAIN * target_rate_dps;
  float diff;
  if (s_gyro_ok) {
    float err;
    float cand_i;
    float max_diff_fwd = TRACK_MAX_SPEED - fabsf(forward_spd);
    float max_diff_rev = fabsf(forward_spd) + max_rev_spd;
    float max_diff = (max_diff_fwd < max_diff_rev) ? max_diff_fwd : max_diff_rev;

    /* 出弯回直或 S 弯反打时快速泄放旧方向积分，避免惯性反拖出线 */
    if (fabsf(target_rate_dps) < 15.0f) {
      float decay = 1.0f - clamp_f(dt_s / 0.05f, 0.0f, 1.0f);
      s_rate_pid_integral *= decay;
    } else if (target_rate_dps * s_rate_pid_integral < 0.0f &&
               fabsf(target_rate_dps) > 30.0f) {
      float decay = 1.0f - clamp_f(dt_s / 0.01f, 0.0f, 1.0f);
      s_rate_pid_integral *= decay;
    }

    err = target_rate_dps - measured_rate_dps;
    cand_i = clamp_f(s_rate_pid_integral + NAV_RATE_KI * err * dt_s,
                     -i_limit, i_limit);
    diff = ff + NAV_RATE_KP * err + cand_i;
    if ((diff <= max_diff || err < 0.0f) && (diff >= -max_diff || err > 0.0f)) {
      s_rate_pid_integral = cand_i;
    } else {
      diff = ff + NAV_RATE_KP * err + s_rate_pid_integral;
    }
  } else {
    diff = ff;
  }
  *out_l = clamp_f(forward_spd + diff, -max_rev_spd, TRACK_MAX_SPEED);
  *out_r = clamp_f(forward_spd - diff, -max_rev_spd, TRACK_MAX_SPEED);
}

static void enter_state(TrackState next) {
  s_state = next;
  s_seg_mm = 0.0f;
  s_yaw_seg = 0.0f;
  s_cross_debounce = 0U;
  s_broken_miss_cnt = 0U;
  s_corner_dir = 0;
  s_corner_latch_ms = 0U;
  s_corner_override_prev = 0U;
  s_search_fwd_spd = NAV_SEARCH_FWD_SPEED;
  gyro_rate_pid_reset();
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

/* 真正的十字路口必须有足够宽度的连续横线且跨左右两侧（>=9 路、连续>=8 路、ch<=2..ch>=9），过滤 0FF/FF0/1FF/3DF */
static uint8_t is_crossroad(const LineScanResult *scan) {
  if (scan->cluster_count == 0U) return 0U;
  return (scan->active_count >= 9U &&
          scan->max_width >= 8U &&
          scan->clusters[0].start_ch <= 2U &&
          scan->clusters[scan->cluster_count - 1U].end_ch >= 9U) ? 1U : 0U;
}

static uint8_t is_left_branch(const LineScanResult *scan) {
  if (scan->cluster_count >= 2U) return 1U;
  if (scan->active_count >= 5U && scan->clusters[0].start_ch <= 1U) return 1U;
  return 0U;
}

static uint8_t is_branch_or_cross(const LineScanResult *scan) {
  return (scan->cluster_count >= 2U || scan->active_count >= 5U || scan->max_width >= 4U) ? 1U : 0U;
}

/* 六边形 60° 交叉口判据：双簇分离或至少 6 路有效，避免单线弯道(如 0x03C/0x1E0)误判 */
static uint8_t is_hex_junction(const LineScanResult *scan) {
  return (scan->cluster_count >= 2U || scan->active_count >= 6U) ? 1U : 0U;
}

/* 终点横线须同时覆盖传感器最左与最右端(ch<=1..ch>=10)且连续宽度>=9，过滤 0x0FF/0x1FF/0xFF8 等单侧直角弯 */
static uint8_t is_finish_bar(const LineScanResult *scan) {
  if (scan->cluster_count == 0U) return 0U;
  return (scan->active_count >= NAV_FINISH_ACTIVE_COUNT &&
          scan->max_width >= 9U &&
          scan->clusters[0].start_ch <= 1U &&
          scan->clusters[scan->cluster_count - 1U].end_ch >= 10U &&
          (!s_gyro_ok || fabsf(s_rate_lpf) <= NAV_FINISH_MAX_RATE_DPS)) ? 1U : 0U;
}

/* 单侧 90° 直角折角前瞻检测：排除双向十字 (ch<=2..ch>=9) 与常规弯斜线 (width<=5)，仅识别单侧贴边宽横线 (width 6..9) */
static int8_t detect_single_side_corner(const LineScanResult *scan) {
  const LineCluster *c;
  if (scan->cluster_count != 1U) return 0;
  c = &scan->clusters[0];
  if (c->width < 6U || c->width > 9U) return 0;
  if (c->end_ch == (MULTIPLEXER_CHANNEL_NUM - 1U) && c->start_ch >= 3U) return 1;
  if (c->start_ch == 0U && c->end_ch <= 8U) return -1;
  return 0;
}

/* 六边形/八边形 45°~60° 柔和转弯与刚出环汇入直道前不启用 90° 猛反甩锁存 */
static uint8_t allow_90deg_corner_snap(void) {
  switch (s_state) {
    case TRK_HEX_R_ENTER:
    case TRK_HEX_R_LOOP:
    case TRK_HEX_R_EXIT:
    case TRK_OCT_LOOP:
    case TRK_APPROACH_HEX_BL:
    case TRK_HEX_BL_LOOP:
    case TRK_HEX_300_ENTER:
    case TRK_HEX_300_R:
    case TRK_HEX_200_LOOP:
    case TRK_HEX_300_L:
    case TRK_HEX_300_EXIT:
      return 0U;
    case TRK_APPROACH_OCT:
      return (s_seg_mm >= 220.0f) ? 1U : 0U;
    default:
      return 1U;
  }
}

/* 选簇三环串级巡线：pick_mode = 0 取最贴近上次/中心簇，-1 取最左簇，+1 取最右簇 */
static void follow_line_pid(const LineScanResult *scan, int8_t pick_mode,
                            float base_target, float max_rate_dps, float dt_s,
                            float *out_l, float *out_r) {
  float err;
  float raw_err_rate;
  float abs_err;
  float abs_diff_5ms;
  float severity;
  float scale;
  float base_spd;
  float d_rate;
  float target_rate;
  float peak_rate;
  float rate_sev;
  uint8_t corner_active = 0U;
  float dt_ms = dt_s * 1000.0f;

  if (scan->cluster_count == 0U) {
    float target_yaw;
    float rem;
    float rate_mag;
    s_has_prev_err = 0U;
    s_err_rate_lpf = 0.0f;
    s_recapture_ms = 0U;
    if (!s_search_active) {
      uint8_t in_sq_loop = (s_state == TRK_SQ_RM_LOOP ||
                            s_state == TRK_LEFT_SQ1_LOOP ||
                            s_state == TRK_LEFT_SQ2_LOOP ||
                            s_state == TRK_LEFT_SQ3_LOOP) ? 1U : 0U;
      if (s_corner_latch_ms > 0U && s_corner_dir != 0) {
        s_search_dir = s_corner_dir;
        s_search_fwd_spd = 0.0f;
      } else if (in_sq_loop && s_gyro_ok && fabsf(s_yaw_seg) >= 45.0f) {
        s_search_dir = (s_yaw_seg >= 0.0f) ? 1 : -1;
        s_search_fwd_spd = 0.0f;
      } else {
        s_search_dir = (s_last_dir != 0) ? s_last_dir : ((s_rate_lpf < 0.0f) ? -1 : 1);
        s_search_fwd_spd = NAV_SEARCH_FWD_SPEED;
      }
      s_corner_latch_ms = 0U;
      s_corner_dir = 0;
      s_corner_override_prev = 0U;
      s_search_start_yaw = s_yaw_deg;
      s_search_active = 1U;
      s_search_phase = 0U;
      s_lost_ms = 0U;
      gyro_rate_pid_reset();
    }
    s_lost_ms = accumulate_ms(s_lost_ms, dt_ms, 60000U);

    /* 始终围绕丢线时刻航向在 ±135° 内做陀螺仪闭环扫线，绝不转到 180° 掉头 */
    target_yaw = s_search_start_yaw + (float)s_search_dir * NAV_SEARCH_TARGET_DEG;
    rem = (target_yaw - s_yaw_deg) * (float)s_search_dir;

    if (rem <= 4.0f || s_lost_ms >= NAV_SEARCH_SWEEP_MS) {
      s_search_dir = (int8_t)(-s_search_dir);
      s_search_phase = 1U;
      s_lost_ms = 0U;
      gyro_rate_pid_reset();
      target_yaw = s_search_start_yaw + (float)s_search_dir * NAV_SEARCH_TARGET_DEG;
      rem = (target_yaw - s_yaw_deg) * (float)s_search_dir;
    }

    if (s_gyro_ok) {
      rate_mag = clamp_f(NAV_TURN_ANGLE_KP * rem,
                         NAV_SEARCH_MIN_RATE_DPS, NAV_SEARCH_MAX_RATE_DPS);
      gyro_rate_pid_update(rate_mag * (float)s_search_dir, s_rate_lpf,
                           s_search_fwd_spd, TRACK_MAX_REV_SPEED,
                           NAV_RATE_I_LIMIT, dt_s, out_l, out_r);
    } else {
      /* 无陀螺仪兜底时回退到温和差速 */
      *out_l = (s_search_dir > 0) ? 16000.0f : -4000.0f;
      *out_r = (s_search_dir < 0) ? 16000.0f : -4000.0f;
    }
    return;
  }

  s_lost_ms = 0U;
  if (s_search_active) {
    /* 刚从丢线扫回白线时开启稳姿窗口，限速并压低目标角速度防止甩过中心 */
    s_search_active = 0U;
    s_search_dir = 0;
    s_search_phase = 0U;
    s_recapture_ms = NAV_RECAPTURE_MS;
    gyro_rate_pid_reset();
  } else if (s_recapture_ms > 0U) {
    uint16_t dec = (dt_ms > 0.0f) ? (uint16_t)(dt_ms + 0.5f) : 1U;
    s_recapture_ms = (s_recapture_ms > dec) ? (uint16_t)(s_recapture_ms - dec) : 0U;
  }

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

  if (s_corner_latch_ms > 0U) {
    uint16_t dec = (dt_ms > 0.0f) ? (uint16_t)(dt_ms + 0.5f) : 1U;
    s_corner_latch_ms = (s_corner_latch_ms > dec) ? (uint16_t)(s_corner_latch_ms - dec) : 0U;
    if (s_corner_latch_ms == 0U) {
      s_corner_dir = 0;
    }
  }
  if (pick_mode == 0 && allow_90deg_corner_snap()) {
    int8_t cdir = detect_single_side_corner(scan);
    if (cdir != 0) {
      /* 仅在实际踩上宽横线折角或离角瞬间 1~2 帧仅剩单灯残影时顶满外沿，恢复正常多灯巡线立即交还闭环控制 */
      s_corner_dir = cdir;
      s_corner_latch_ms = 36U;
      corner_active = 1U;
      err = (float)cdir * 11.0f;
    } else if (s_corner_latch_ms > 0U && s_corner_dir != 0 &&
               scan->cluster_count == 1U &&
               ((scan->clusters[0].width == 1U && fabsf(scan->clusters[0].center) < 6.0f) ||
                (scan->clusters[0].width >= 4U && scan->clusters[0].width <= 9U))) {
      /* 离角瞬间只剩单灯残影或外侧刚掉灯变成 4~9 路偏边块 (如 0x07C) 时保持外沿锁定不提前放开 */
      corner_active = 1U;
      err = (float)s_corner_dir * 11.0f;
    } else if (scan->cluster_count == 1U && scan->clusters[0].width <= 3U && fabsf(err) <= 3.0f) {
      s_corner_latch_ms = 0U;
      s_corner_dir = 0;
    }
  } else {
    s_corner_latch_ms = 0U;
    s_corner_dir = 0;
  }
  if (corner_active) {
    s_corner_override_prev = 1U;
  } else if (s_corner_override_prev) {
    s_corner_override_prev = 0U;
    s_has_prev_err = 0U;
    s_err_rate_lpf = 0.0f;
  }

  /* 明显偏离中心（>=2.0）立即更新方向；中心附近小偏差需连续 4 帧同向才更新，防止单灯闪烁误翻 */
  if (err >= 2.0f) {
    s_last_dir = 1;
    s_small_dir_cand = 1;
    s_small_dir_cnt = 0U;
  } else if (err <= -2.0f) {
    s_last_dir = -1;
    s_small_dir_cand = -1;
    s_small_dir_cnt = 0U;
  } else if (err >= 0.8f || err <= -0.8f) {
    int8_t d = (err > 0.0f) ? 1 : -1;
    if (s_last_dir == 0) {
      s_last_dir = d;
    } else if (d == s_small_dir_cand) {
      if (++s_small_dir_cnt >= 4U) {
        s_last_dir = d;
        s_small_dir_cnt = 4U;
      }
    } else {
      s_small_dir_cand = d;
      s_small_dir_cnt = 1U;
    }
  } else {
    s_small_dir_cnt = 0U;
  }

  raw_err_rate = (s_has_prev_err && dt_s > 1e-6f) ? ((err - s_prev_err) / dt_s) : 0.0f;
  if (s_has_prev_err) {
    float alpha = (dt_s > 0.0f) ? (dt_s / (0.003f + dt_s)) : 0.40f;
    s_err_rate_lpf += alpha * (raw_err_rate - s_err_rate_lpf);
  } else {
    s_err_rate_lpf = 0.0f;
  }
  s_prev_err = err;
  s_has_prev_err = 1U;

  float eff_base = base_target;
  float eff_max_rate = max_rate_dps;
  float eff_max_rev = NAV_FOLLOW_MAX_REV_SPEED;
  if (corner_active) {
    if (max_rate_dps >= NAV_FOLLOW_MAX_RATE_DPS && eff_max_rate < NAV_CORNER_MAX_RATE_DPS) {
      eff_max_rate = NAV_CORNER_MAX_RATE_DPS;
    }
    eff_max_rev = NAV_CORNER_MAX_REV_SPEED;
  }
  if (s_recapture_ms > 0U) {
    if (eff_base > NAV_RECAPTURE_SPEED) eff_base = NAV_RECAPTURE_SPEED;
    if (eff_max_rate > NAV_RECAPTURE_MAX_RATE) eff_max_rate = NAV_RECAPTURE_MAX_RATE;
  }

  /* 位置外环 PD 直接生成并限幅目标偏航角速度，由角速度内环主动闭环追踪，杜绝急弯甩过 180° */
  abs_err = fabsf(err);
  d_rate = clamp_f(NAV_LINE_RATE_KD * s_err_rate_lpf,
                   -NAV_LINE_RATE_KD_LIM, NAV_LINE_RATE_KD_LIM);
  target_rate = clamp_f(NAV_LINE_RATE_KP * err + NAV_LINE_RATE_KP2 * err * abs_err + d_rate,
                        -eff_max_rate, eff_max_rate);

  abs_diff_5ms = fabsf(s_err_rate_lpf) * 0.005f;
  severity = abs_err;
  if (abs_diff_5ms * 2.5f > severity) severity = abs_diff_5ms * 2.5f;
  peak_rate = fabsf(target_rate);
  if (s_gyro_ok && fabsf(s_rate_lpf) > peak_rate) {
    peak_rate = fabsf(s_rate_lpf);
  }
  rate_sev = peak_rate / 85.0f;
  if (rate_sev > severity) severity = rate_sev;

  scale = 1.0f - 0.085f * (severity - 4.5f);
  if (corner_active) {
    scale = clamp_f(scale, 0.36f, 0.40f);
  } else {
    scale = clamp_f(scale, 0.55f, 1.0f);
  }
  base_spd = eff_base * scale;

  gyro_rate_pid_update(target_rate, s_rate_lpf, base_spd,
                       eff_max_rev, NAV_FOLLOW_RATE_I_LIMIT,
                       dt_s, out_l, out_r);
}

/* 陀螺仪锁航向直行 */
static void follow_yaw_hold(float target_yaw, float speed, float *out_l, float *out_r) {
  float yaw_err = target_yaw - s_yaw_deg;
  float corr = NAV_YAW_HOLD_KP * yaw_err - 8.0f * s_rate_lpf;
  *out_l = clamp_f(speed + corr, -TRACK_MAX_REV_SPEED, TRACK_MAX_SPEED);
  *out_r = clamp_f(speed - corr, -TRACK_MAX_REV_SPEED, TRACK_MAX_SPEED);
  s_has_prev_err = 0U;
  s_err_rate_lpf = 0.0f;
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
  s_err_rate_lpf = 0.0f;
  s_has_prev_err = 0U;
  s_last_dir = 0;
  s_small_dir_cand = 0;
  s_small_dir_cnt = 0U;
  s_search_dir = 0;
  s_search_active = 0U;
  s_search_phase = 0U;
  s_lost_ms = 0U;
  s_recapture_ms = 0U;
  s_corner_dir = 0;
  s_corner_latch_ms = 0U;
  s_corner_override_prev = 0U;
  s_search_fwd_spd = NAV_SEARCH_FWD_SPEED;
  gyro_rate_pid_reset();
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
      follow_line_pid(&scan, 0, TRACK_BASE_SPEED, NAV_FOLLOW_MAX_RATE_DPS, dt_s, out_l, out_r);
      if (s_seg_mm >= 900.0f) {
        enter_state(TRK_ZIGZAG_1);
      }
      break;

    /* 2. 右下 2 峰锯齿 + 右下折角 + 2153/2375 弯道升至右中 250 方框十字（实测约 2890 mm） */
    case TRK_ZIGZAG_1: {
      float spd = (s_seg_mm < 1550.0f) ? TRACK_ZIGZAG_SPEED : TRACK_BASE_SPEED;
      float max_rate = (s_seg_mm < 1550.0f) ? NAV_FOLLOW_ZIG_RATE_DPS : NAV_FOLLOW_MAX_RATE_DPS;
      follow_line_pid(&scan, 0, spd, max_rate, dt_s, out_l, out_r);
      if (debounce_cond((s_seg_mm >= 2600.0f && is_crossroad(&scan)) ? 1U : 0U, 2U)) {
        s_yaw_ref = s_yaw_deg;
        enter_state(TRK_SQ_RM_IN);
      }
      break;
    }

    /* 3. 右中 250x250 方框（十字直行进 -> 绕圈 270° -> 十字直行出到 420） */
    case TRK_SQ_RM_IN:
      follow_yaw_hold(s_yaw_ref, TRACK_CROSS_SPEED, out_l, out_r);
      if (s_seg_mm >= NAV_CROSS_PASS_MM) enter_state(TRK_SQ_RM_LOOP);
      break;

    case TRK_SQ_RM_LOOP:
      follow_line_pid(&scan, 0, TRACK_HEX_SPEED, NAV_FOLLOW_MAX_RATE_DPS, dt_s, out_l, out_r);
      if (debounce_cond((s_seg_mm >= 750.0f &&
                         (!s_gyro_ok || fabsf(s_yaw_seg) >= 210.0f) &&
                         is_crossroad(&scan)) ? 1U : 0U, 2U)) {
        s_yaw_ref = s_yaw_deg;
        enter_state(TRK_SQ_RM_OUT);
      }
      break;

    case TRK_SQ_RM_OUT:
      follow_yaw_hold(s_yaw_ref, TRACK_CROSS_SPEED, out_l, out_r);
      if (s_seg_mm >= NAV_CROSS_PASS_MM) enter_state(TRK_APPROACH_HEX_R);
      break;

    /* 4. 右中 200 六边形（跳过 320mm 处左侧回流线 860..7E0，在 420mm 右侧分叉 0F8/3B8 切入 -> 绕一圈 -> 左偏出到 330） */
    case TRK_APPROACH_HEX_R:
      follow_line_pid(&scan, 0, TRACK_BASE_SPEED, NAV_FOLLOW_MAX_RATE_DPS, dt_s, out_l, out_r);
      if (debounce_cond((s_seg_mm >= 360.0f && is_branch_or_cross(&scan) &&
                         scan.clusters[scan.cluster_count - 1U].end_ch >= 8U) ? 1U : 0U, 2U)) {
        s_yaw_ref = s_yaw_deg;
        enter_state(TRK_HEX_R_ENTER);
      }
      break;

    case TRK_HEX_R_ENTER:
      follow_line_pid(&scan, -1, TRACK_HEX_R_SPEED, NAV_FOLLOW_MAX_RATE_DPS, dt_s, out_l, out_r);
      if (s_seg_mm >= 120.0f) enter_state(TRK_HEX_R_LOOP);
      break;

    case TRK_HEX_R_LOOP:
      follow_line_pid(&scan, 0, TRACK_HEX_R_SPEED, NAV_FOLLOW_MAX_RATE_DPS, dt_s, out_l, out_r);
      if (debounce_cond((s_seg_mm >= 850.0f && s_yaw_seg <= -275.0f &&
                         (scan.cluster_count >= 2U || scan.active_count >= 6U)) ? 1U : 0U, 2U)) {
        enter_state(TRK_HEX_R_EXIT);
      } else if (s_yaw_seg <= -345.0f || s_seg_mm >= 1220.0f) {
        /* 兜底：若已自然转完出环切上 330 直道，直接进入 APPROACH_OCT，防止 HEX_R_EXIT 在下个直角弯错触发 */
        enter_state(TRK_APPROACH_OCT);
      }
      break;

    case TRK_HEX_R_EXIT: {
      int8_t exit_pick = 0;
      if (s_seg_mm < 65.0f) {
        /* 离开 320mm 回流口：左咬切上 420/330 主干道 */
        exit_pick = -1;
      } else if (scan.cluster_count >= 2U ||
                 (scan.cluster_count == 1U && scan.clusters[0].width >= 4U &&
                  (!s_gyro_ok || s_yaw_deg <= s_yaw_ref - 340.0f))) {
        /* 65~200mm 再次经过 420mm 入环分叉 (如 0xF0C/0x3B8) 时，改取最右簇直走 330，防止再次左拐入环 */
        exit_pick = 1;
      }
      follow_line_pid(&scan, exit_pick, TRACK_HEX_R_SPEED, NAV_FOLLOW_MAX_RATE_DPS, dt_s, out_l, out_r);
      if (s_seg_mm >= 195.0f &&
          (scan.cluster_count <= 1U || s_seg_mm >= 240.0f) &&
          (!s_gyro_ok || s_yaw_deg <= s_yaw_ref - 335.0f || s_seg_mm >= 240.0f)) {
        enter_state(TRK_APPROACH_OCT);
      }
      break;
    }

    /* 5. 右上八边形（十字直行进 -> 绕一圈 -> 十字直行出） */
    case TRK_APPROACH_OCT:
      follow_line_pid(&scan, 0, TRACK_BASE_SPEED, NAV_FOLLOW_MAX_RATE_DPS, dt_s, out_l, out_r);
      if (debounce_cond((s_seg_mm >= 1350.0f && is_crossroad(&scan)) ? 1U : 0U, 2U)) {
        s_yaw_ref = s_yaw_deg;
        enter_state(TRK_OCT_IN);
      }
      break;

    case TRK_OCT_IN:
      follow_yaw_hold(s_yaw_ref, TRACK_CROSS_SPEED, out_l, out_r);
      if (s_seg_mm >= NAV_CROSS_PASS_MM) enter_state(TRK_OCT_LOOP);
      break;

    case TRK_OCT_LOOP:
      follow_line_pid(&scan, 0, TRACK_HEX_SPEED, NAV_FOLLOW_MAX_RATE_DPS, dt_s, out_l, out_r);
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
        follow_line_pid(&scan, 0, TRACK_BASE_SPEED, NAV_FOLLOW_MAX_RATE_DPS, dt_s, out_l, out_r);
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
          seek_yaw = ((phase & 1U) == 0U) ? (s_yaw_west - 28.0f) : (s_yaw_west + 40.0f);
        else
          seek_yaw = ((phase & 1U) == 0U) ? (s_yaw_west - 20.0f) : (s_yaw_west + 32.0f);
      }
      follow_yaw_hold(seek_yaw, TRACK_BROKEN_SPEED, out_l, out_r);
      if (s_seg_mm >= 220.0f && scan.active_count >= 1U) {
        enter_state(TRK_BROKEN_ALIGN);
      }
      break;
    }

    case TRK_BROKEN_ALIGN:
      /* 抓到线立刻用循迹 PID 回正，不继续盲走 */
      follow_line_pid(&scan, 0, TRACK_BROKEN_SPEED, NAV_FOLLOW_MAX_RATE_DPS, dt_s, out_l, out_r);
      if (scan.cluster_count >= 1U &&
          ((s_seg_mm >= NAV_BROKEN_ALIGN_MIN_MM && fabsf(s_yaw_deg - s_yaw_west) <= 25.0f) ||
           s_seg_mm >= NAV_BROKEN_ALIGN_MAX_MM)) {
        enter_state(TRK_TL_SQ_PRE_T1);
      } else if (s_search_active && s_lost_ms >= 150U) {
        enter_state(TRK_BROKEN_SEEK);
      }
      break;

    /* 7. 左上 500 折角南下 -> 250x250 T 型方框（穿十字下探 -> 逆时针绕完整四边 -> 向西出至 350 方框） */
    case TRK_TL_SQ_PRE_T1: {
      float spd = (s_seg_mm >= 280.0f) ? TRACK_HEX_SPEED : TRACK_BASE_SPEED;
      follow_line_pid(&scan, 0, spd, NAV_FOLLOW_MAX_RATE_DPS, dt_s, out_l, out_r);
      if (scan.cluster_count == 1U && s_seg_mm >= 40.0f && s_seg_mm < 280.0f &&
          fabsf(s_prev_err) <= 2.5f) {
        /* 沿左上直道时刷新西向基准航向，过折角转南前停止刷新 */
        s_yaw_west = s_yaw_deg;
      }
      if ((s_seg_mm >= 200.0f && s_gyro_ok && (s_yaw_deg - s_yaw_west) <= -65.0f &&
           scan.cluster_count >= 1U && fabsf(s_prev_err) <= 4.0f) ||
          (!s_gyro_ok && debounce_cond((s_seg_mm >= 360.0f && is_left_branch(&scan)) ? 1U : 0U, 2U))) {
        enter_state(TRK_TL_SQ_TO_BL1);
      }
      break;
    }

    case TRK_TL_SQ_T1_PASS:
      follow_yaw_hold(s_yaw_ref, TRACK_CROSS_SPEED, out_l, out_r);
      if (s_seg_mm >= NAV_CROSS_PASS_MM) enter_state(TRK_TL_SQ_TO_TR2);
      break;

    case TRK_TL_SQ_TO_BL1:
      follow_line_pid(&scan, 0, TRACK_HEX_SPEED, NAV_FOLLOW_MAX_RATE_DPS, dt_s, out_l, out_r);
      if (debounce_cond((s_seg_mm >= 80.0f && (!s_gyro_ok || (s_yaw_deg - s_yaw_west) >= -125.0f) &&
                         is_crossroad(&scan)) ? 1U : 0U, 2U)) {
        s_yaw_ref = s_yaw_deg;
        enter_state(TRK_TL_SQ_T1_PASS);
      } else if (s_gyro_ok && (s_yaw_deg - s_yaw_west) <= -145.0f) {
        enter_state(TRK_TL_SQ_TO_TR2);
      }
      break;

    case TRK_TL_SQ_TURN_E:
      enter_state(TRK_TL_SQ_TO_TR2);
      follow_line_pid(&scan, 0, TRACK_HEX_SPEED, NAV_FOLLOW_MAX_RATE_DPS, dt_s, out_l, out_r);
      break;

    case TRK_TL_SQ_TO_TR2:
      follow_line_pid(&scan, 0, TRACK_HEX_SPEED, NAV_FOLLOW_MAX_RATE_DPS, dt_s, out_l, out_r);
      if (s_gyro_ok) {
        if (s_seg_mm >= 300.0f && (s_yaw_deg - s_yaw_west) <= -335.0f &&
            scan.cluster_count == 1U && fabsf(s_prev_err) <= 3.5f) {
          enter_state(TRK_TL_SQ_TO_BL2);
        }
      } else if (debounce_cond((s_seg_mm >= 900.0f && is_crossroad(&scan)) ? 1U : 0U, 2U)) {
        s_yaw_ref = s_yaw_deg;
        enter_state(TRK_TL_SQ_EXIT_W);
      }
      break;

    case TRK_TL_SQ_TURN_W:
      enter_state(TRK_TL_SQ_TO_BL2);
      follow_line_pid(&scan, 0, TRACK_HEX_SPEED, NAV_FOLLOW_MAX_RATE_DPS, dt_s, out_l, out_r);
      break;

    case TRK_TL_SQ_TO_BL2:
      follow_line_pid(&scan, 0, TRACK_HEX_SPEED, NAV_FOLLOW_MAX_RATE_DPS, dt_s, out_l, out_r);
      if (debounce_cond((s_seg_mm >= 60.0f &&
                         (is_crossroad(&scan) || is_hex_junction(&scan))) ? 1U : 0U, 2U)) {
        s_yaw_ref = s_yaw_deg;
        enter_state(TRK_TL_SQ_EXIT_W);
      }
      break;

    case TRK_TL_SQ_EXIT_W:
      follow_yaw_hold(s_yaw_ref, TRACK_CROSS_SPEED, out_l, out_r);
      if (s_seg_mm >= NAV_CROSS_PASS_MM) enter_state(TRK_LEFT_SQ1_PRE);
      break;

    /* 8. 左侧三个十字方框（350x350 -> 250x250 -> 250x250） */
    case TRK_LEFT_SQ1_PRE:
      follow_line_pid(&scan, 0, TRACK_BASE_SPEED, NAV_FOLLOW_MAX_RATE_DPS, dt_s, out_l, out_r);
      if (debounce_cond((s_seg_mm >= 75.0f && is_crossroad(&scan)) ? 1U : 0U, 2U)) {
        s_yaw_ref = s_yaw_deg;
        enter_state(TRK_LEFT_SQ1_IN);
      }
      break;
    case TRK_LEFT_SQ1_IN:
      follow_yaw_hold(s_yaw_ref, TRACK_CROSS_SPEED, out_l, out_r);
      if (s_seg_mm >= NAV_CROSS_PASS_MM) enter_state(TRK_LEFT_SQ1_LOOP);
      break;
    case TRK_LEFT_SQ1_LOOP:
      follow_line_pid(&scan, 0, TRACK_BASE_SPEED, NAV_FOLLOW_MAX_RATE_DPS, dt_s, out_l, out_r);
      if (debounce_cond((s_seg_mm >= 950.0f &&
                         (!s_gyro_ok || fabsf(s_yaw_seg) >= 210.0f) &&
                         is_crossroad(&scan)) ? 1U : 0U, 2U)) {
        s_yaw_ref = s_yaw_deg;
        enter_state(TRK_LEFT_SQ1_OUT);
      }
      break;
    case TRK_LEFT_SQ1_OUT:
      follow_yaw_hold(s_yaw_ref, TRACK_CROSS_SPEED, out_l, out_r);
      if (s_seg_mm >= NAV_CROSS_PASS_MM) enter_state(TRK_LEFT_SQ2_PRE);
      break;

    case TRK_LEFT_SQ2_PRE:
      follow_line_pid(&scan, 0, TRACK_BASE_SPEED, NAV_FOLLOW_MAX_RATE_DPS, dt_s, out_l, out_r);
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
      follow_line_pid(&scan, 0, TRACK_BASE_SPEED, NAV_FOLLOW_MAX_RATE_DPS, dt_s, out_l, out_r);
      if (debounce_cond((s_seg_mm >= 650.0f &&
                         (!s_gyro_ok || fabsf(s_yaw_seg) >= 210.0f) &&
                         is_crossroad(&scan)) ? 1U : 0U, 2U)) {
        s_yaw_ref = s_yaw_deg;
        enter_state(TRK_LEFT_SQ2_OUT);
      }
      break;
    case TRK_LEFT_SQ2_OUT:
      follow_yaw_hold(s_yaw_ref, TRACK_CROSS_SPEED, out_l, out_r);
      if (s_seg_mm >= NAV_CROSS_PASS_MM) enter_state(TRK_LEFT_SQ3_PRE);
      break;

    case TRK_LEFT_SQ3_PRE:
      follow_line_pid(&scan, 0, TRACK_BASE_SPEED, NAV_FOLLOW_MAX_RATE_DPS, dt_s, out_l, out_r);
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
      follow_line_pid(&scan, 0, TRACK_BASE_SPEED, NAV_FOLLOW_MAX_RATE_DPS, dt_s, out_l, out_r);
      if (debounce_cond((s_seg_mm >= 650.0f &&
                         (!s_gyro_ok || fabsf(s_yaw_seg) >= 210.0f) &&
                         is_crossroad(&scan)) ? 1U : 0U, 2U)) {
        s_yaw_ref = s_yaw_deg;
        enter_state(TRK_LEFT_SQ3_OUT);
      }
      break;
    case TRK_LEFT_SQ3_OUT:
      follow_yaw_hold(s_yaw_ref, TRACK_CROSS_SPEED, out_l, out_r);
      if (s_seg_mm >= NAV_CROSS_PASS_MM) enter_state(TRK_APPROACH_HEX_BL);
      break;

    /* 9. 左下 200 六边形（沿 460+225 到 60° 交叉 -> 直行切入顺时针绕 6 边 -> 直行越过交叉口出） */
    case TRK_APPROACH_HEX_BL:
      follow_line_pid(&scan, (s_seg_mm >= 440.0f && s_seg_mm < 560.0f) ? -1 : 0,
                      TRACK_HEX_SPEED, NAV_FOLLOW_MAX_RATE_DPS, dt_s, out_l, out_r);
      if (debounce_cond((s_seg_mm >= 560.0f && is_hex_junction(&scan)) ? 1U : 0U, 2U)) {
        s_yaw_ref = s_yaw_deg;
        enter_state(TRK_HEX_BL_IN);
      }
      break;

    case TRK_HEX_BL_IN:
      follow_yaw_hold(s_yaw_ref, TRACK_CROSS_SPEED, out_l, out_r);
      if (s_seg_mm >= 85.0f) enter_state(TRK_HEX_BL_LOOP);
      break;

    case TRK_HEX_BL_LOOP:
      follow_line_pid(&scan, 0, TRACK_HEX_SPEED, NAV_FOLLOW_MAX_RATE_DPS, dt_s, out_l, out_r);
      if (debounce_cond((s_seg_mm >= 1020.0f && (!s_gyro_ok || s_yaw_seg >= 270.0f) &&
                         is_hex_junction(&scan)) ? 1U : 0U, 2U)) {
        s_yaw_ref = s_yaw_deg;
        enter_state(TRK_HEX_BL_EXIT);
      }
      break;

    case TRK_HEX_BL_EXIT:
      follow_yaw_hold(s_yaw_ref, TRACK_CROSS_SPEED, out_l, out_r);
      if (s_seg_mm >= 85.0f) enter_state(TRK_APPROACH_HEX_MID);
      break;

    /* 10. 中左 300+200 双六边形 8 字一笔画 */
    case TRK_APPROACH_HEX_MID:
      follow_line_pid(&scan, 0, TRACK_BASE_SPEED, NAV_FOLLOW_MAX_RATE_DPS, dt_s, out_l, out_r);
      if (debounce_cond((s_seg_mm >= 700.0f && is_hex_junction(&scan)) ? 1U : 0U, 2U)) {
        enter_state(TRK_HEX_300_ENTER);
      }
      break;

    case TRK_HEX_300_ENTER:
      follow_line_pid(&scan, -1, TRACK_HEX_SPEED, NAV_FOLLOW_MAX_RATE_DPS, dt_s, out_l, out_r);
      if (s_seg_mm >= 130.0f) enter_state(TRK_HEX_300_R);
      break;

    case TRK_HEX_300_R:
      follow_line_pid(&scan, 0, TRACK_HEX_SPEED, NAV_FOLLOW_MAX_RATE_DPS, dt_s, out_l, out_r);
      if (debounce_cond((s_seg_mm >= 360.0f && (!s_gyro_ok || s_yaw_seg <= -35.0f) &&
                         is_hex_junction(&scan)) ? 1U : 0U, 2U)) {
        s_yaw_ref = s_yaw_deg;
        enter_state(TRK_HEX_200_IN);
      }
      break;

    case TRK_HEX_200_IN:
      follow_yaw_hold(s_yaw_ref, TRACK_CROSS_SPEED, out_l, out_r);
      if (s_seg_mm >= 85.0f) enter_state(TRK_HEX_200_LOOP);
      break;

    case TRK_HEX_200_LOOP:
      follow_line_pid(&scan, 0, TRACK_HEX_SPEED, NAV_FOLLOW_MAX_RATE_DPS, dt_s, out_l, out_r);
      if (debounce_cond((s_seg_mm >= 1000.0f && (!s_gyro_ok || s_yaw_seg >= 270.0f) &&
                         is_hex_junction(&scan)) ? 1U : 0U, 2U)) {
        s_yaw_ref = s_yaw_deg;
        enter_state(TRK_HEX_200_CROSS);
      }
      break;

    case TRK_HEX_200_CROSS:
      follow_yaw_hold(s_yaw_ref, TRACK_CROSS_SPEED, out_l, out_r);
      if (s_seg_mm >= 85.0f) enter_state(TRK_HEX_300_L);
      break;

    case TRK_HEX_300_L:
      follow_line_pid(&scan, 0, TRACK_HEX_SPEED, NAV_FOLLOW_MAX_RATE_DPS, dt_s, out_l, out_r);
      if (debounce_cond((s_seg_mm >= 720.0f && (!s_gyro_ok || s_yaw_seg <= -90.0f) &&
                         is_hex_junction(&scan)) ? 1U : 0U, 2U)) {
        enter_state(TRK_HEX_300_EXIT);
      }
      break;

    case TRK_HEX_300_EXIT:
      follow_line_pid(&scan, -1, TRACK_HEX_SPEED, NAV_FOLLOW_MAX_RATE_DPS, dt_s, out_l, out_r);
      if (s_seg_mm >= 140.0f) enter_state(TRK_ZIGZAG_2);
      break;

    /* 11. 收尾：530+550+240 + 3峰锯齿 + 490 回发车区双横线停车 */
    case TRK_ZIGZAG_2: {
      uint8_t in_zig = (s_seg_mm >= 1150.0f && s_seg_mm <= 2250.0f) ? 1U : 0U;
      follow_line_pid(&scan, 0,
                      in_zig ? TRACK_ZIGZAG_SPEED : TRACK_BASE_SPEED,
                      in_zig ? NAV_FOLLOW_ZIG_RATE_DPS : NAV_FOLLOW_MAX_RATE_DPS,
                      dt_s, out_l, out_r);
      if (s_seg_mm >= 2450.0f) enter_state(TRK_FINISH_ARMED);
      break;
    }

    case TRK_FINISH_ARMED:
      follow_line_pid(&scan, 0, TRACK_BASE_SPEED, NAV_FOLLOW_MAX_RATE_DPS, dt_s, out_l, out_r);
      if (is_finish_bar(&scan))
        s_finish_active_ms = accumulate_ms(s_finish_active_ms, dt_s * 1000.0f, NAV_FINISH_DEBOUNCE_MS);
      else s_finish_active_ms = 0U;
      if (s_finish_active_ms >= NAV_FINISH_DEBOUNCE_MS) enter_state(TRK_FINISH_BAR1);
      break;

    case TRK_FINISH_BAR1:
      follow_line_pid(&scan, 0, TRACK_BASE_SPEED, NAV_FOLLOW_MAX_RATE_DPS, dt_s, out_l, out_r);
      if (!s_finish_released) {
        if (!is_finish_bar(&scan))
          s_finish_clear_ms = accumulate_ms(s_finish_clear_ms, dt_s * 1000.0f, NAV_FINISH_DEBOUNCE_MS);
        else s_finish_clear_ms = 0U;
        if (s_finish_clear_ms >= NAV_FINISH_DEBOUNCE_MS) s_finish_released = 1U;
      } else if (s_seg_mm >= 320.0f && is_finish_bar(&scan)) {
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
