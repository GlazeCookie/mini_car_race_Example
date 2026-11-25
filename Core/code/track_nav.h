#ifndef TRACK_NAV_H
#define TRACK_NAV_H

#include <stdint.h>

/* 每米编码器计数；右轮尚未单独标定，暂用左轮值。 */
#define TRACK_LEFT_COUNTS_PER_METER  37188.0f
#define TRACK_RIGHT_COUNTS_PER_METER 37188.0f

/* 赛道巡线与竞速参数（实车支持双向调速与反接制动） */
#define TRACK_BASE_SPEED           110000.0f  /* 常规/直道巡线目标轮速 (counts/s, ≈2.37 m/s) */
#define TRACK_ZIGZAG_SPEED         67000.0f  /* 锯齿急弯段目标轮速 (counts/s, ≈1.43 m/s) */
#define TRACK_SQ_SPEED             75500.0f  /* 左侧 250×250 方框绕圈目标轮速 (counts/s, ≈1.74 m/s) */
#define TRACK_HEX_SPEED            75000.0f  /* 六边形/八边形/紧方框环岛速度 (counts/s, ≈1.77 m/s) */
#define TRACK_HEX_R_SPEED          75500.0f  /* 右中 200 六边形速度 (counts/s, ≈1.66 m/s) */
#define TRACK_CROSS_SPEED          75000.0f  /* 过十字/T口直行速度 (counts/s, ≈1.74 m/s) */
#define TRACK_BROKEN_SPEED         67500.0f  /* 过断路区域盲走速度 (counts/s, ≈1.58 m/s) */
#define TRACK_TURN_OUTER_SPEED     34000.0f  /* 无陀螺仪兜底时定角转向外轮速度 (counts/s, ≈0.91 m/s) */
#define TRACK_TURN_INNER_REV_SPEED 18000.0f  /* 无陀螺仪兜底时定角转向内轮反转速度 (counts/s, ≈0.49 m/s) */
#define TRACK_MAX_SPEED            140000.0f  /* 正向最高目标速度 (counts/s, ≈2.82 m/s) */
#define TRACK_MAX_REV_SPEED        52000.0f  /* 反向差速/反接制动最高速度幅值 (counts/s, ≈1.40 m/s) */

/* 12 路光电管单簇信息 */
typedef struct {
  uint8_t start_ch;
  uint8_t end_ch;
  uint8_t width;
  float center; /* -11.0 ~ +11.0（左负右正） */
} LineCluster;

typedef struct {
  uint16_t mux_value;
  uint8_t active_count;
  uint8_t cluster_count;
  uint8_t max_width;
  LineCluster clusters[6];
} LineScanResult;

typedef enum {
  TRK_START_CLEAR = 0,     /* 发车区起步屏蔽双横线 */
  TRK_ZIGZAG_1,            /* 右下 2 峰锯齿 + 上行至右中十字 */
  TRK_SQ_RM_IN,            /* 穿入右中 250 方框 */
  TRK_SQ_RM_LOOP,          /* 绕右中 250 方框 */
  TRK_SQ_RM_OUT,           /* 穿出右中 250 方框到 420+330 */
  TRK_APPROACH_HEX_R,      /* 沿 420 前往右中 200 六边形切点 */
  TRK_HEX_R_ENTER,         /* 左偏 30° 切入右中 200 六边形 */
  TRK_HEX_R_LOOP,          /* 逆时针绕右中 200 六边形 */
  TRK_HEX_R_EXIT,          /* 出右中 200 六边形回 330 */
  TRK_APPROACH_OCT,        /* 330+500+650+250 前往右上八边形 */
  TRK_OCT_IN,              /* 穿入右上八边形 */
  TRK_OCT_LOOP,            /* 绕右上八边形 */
  TRK_OCT_OUT,             /* 穿出右上八边形到 400 */
  TRK_APPROACH_BROKEN,     /* 400+250+500 前往顶部断路区域 */
  TRK_BROKEN_CLEAR,        /* 断路阶段1：锁航向直行离开线端 */
  TRK_BROKEN_SEEK,         /* 断路阶段2：+15°斜插平移找对面线（含扫线兜底） */
  TRK_BROKEN_ALIGN,        /* 断路阶段3：抓到线后回正航向 */
  TRK_TL_SQ_PRE_T1,        /* 左上 500 找第一个 T 口（右上 T 口） */
  TRK_TL_SQ_T1_PASS,       /* 直行越过右上 T 口 */
  TRK_TL_SQ_TO_BL1,        /* 走左上 250 方框顶边+左边到左下 T 口 */
  TRK_TL_SQ_TURN_E,        /* 左转 90° 进方框底边向东 */
  TRK_TL_SQ_TO_TR2,        /* 走底边+右边回到右上 T 口 */
  TRK_TL_SQ_TURN_W,        /* 左转 90° 回顶边向西 */
  TRK_TL_SQ_TO_BL2,        /* 再走顶边+左边到左下 T 口 */
  TRK_TL_SQ_EXIT_W,        /* 右转 90° 出到 230 横线向西 */
  TRK_LEFT_SQ1_PRE,        /* 沿 230 到左上 350 方框十字 */
  TRK_LEFT_SQ1_IN,         /* 穿入 350 方框 */
  TRK_LEFT_SQ1_LOOP,       /* 绕 350 方框 */
  TRK_LEFT_SQ1_OUT,        /* 穿出 350 方框到 490 */
  TRK_LEFT_SQ2_PRE,        /* 沿 490 到左中 250 方框十字 */
  TRK_LEFT_SQ2_IN,         /* 穿入左中 250 方框 */
  TRK_LEFT_SQ2_LOOP,       /* 绕左中 250 方框 */
  TRK_LEFT_SQ2_OUT,        /* 穿出左中 250 方框到 300 */
  TRK_LEFT_SQ3_PRE,        /* 沿 300 到最左 250 方框十字 */
  TRK_LEFT_SQ3_IN,         /* 穿入最左 250 方框 */
  TRK_LEFT_SQ3_LOOP,       /* 绕最左 250 方框 */
  TRK_LEFT_SQ3_OUT,        /* 穿出最左 250 方框到 460 */
  TRK_APPROACH_HEX_BL,     /* 460+225 到左下 60° 交叉口 */
  TRK_HEX_BL_IN,           /* 右转 60° 进左下 200 六边形 */
  TRK_HEX_BL_LOOP,         /* 顺时针绕左下 200 六边形 */
  TRK_HEX_BL_EXIT,         /* 左转 120° 出左下 60° 交叉口上斜线 */
  TRK_APPROACH_HEX_MID,    /* 经 120°+590 到中左 300 六边形切点 */
  TRK_HEX_300_ENTER,       /* 左偏 30° 进 300 六边形右半圈 */
  TRK_HEX_300_R,           /* 沿 300 右半圈北上至 60° 交点 */
  TRK_HEX_200_IN,          /* 右转 60° 切入 200 小六边形 */
  TRK_HEX_200_LOOP,        /* 顺时针绕 200 小六边形 */
  TRK_HEX_200_CROSS,       /* 直行穿过 60° 交点回 300 左半圈 */
  TRK_HEX_300_L,           /* 沿 300 左半圈回到 590/530 切点 */
  TRK_HEX_300_EXIT,        /* 左偏 30° 出环到 530 向东 */
  TRK_ZIGZAG_2,            /* 530+550+240+3峰锯齿+490 回发车区 */
  TRK_FINISH_ARMED,        /* 进入底部发车区等待第一条终点横线 */
  TRK_FINISH_BAR1,         /* 越过第一条横线等待第二条横线 */
  TRK_DONE,                /* 完赛停车锁定 */
  TRK_FAULT_STOP           /* 陀螺仪或异常保护停车 */
} TrackState;

typedef enum {
  TRACK_FAULT_NONE = 0,
  TRACK_FAULT_GYRO_INIT,
  TRACK_FAULT_GYRO_READ,
  TRACK_FAULT_GYRO_MOVED,
  TRACK_FAULT_GAP_NOT_FOUND,   /* 预留；比赛策略继续扫线 */
  TRACK_FAULT_GAP_ALIGN,       /* 预留；比赛策略返回扫线 */
  TRACK_FAULT_TURN_TIMEOUT,
  TRACK_FAULT_TURN_NO_PROGRESS,
  TRACK_FAULT_TURN_WRONG_DIRECTION,
  TRACK_FAULT_BRANCH_MISSED,   /* 预留；比赛策略继续找分叉 */
  TRACK_FAULT_STATE_LIMIT,     /* 预留；比赛策略不设通用状态距离上限 */
  TRACK_FAULT_LEFT_ENCODER,
  TRACK_FAULT_RIGHT_ENCODER
} TrackFault;

/* 初始化并静止标定 BMI270 零偏（请在发车延时后或发车前静止调用） */
void TrackNav_Start(void);

/* 12 路簇解析工具函数（可供其他模式复用） */
void TrackNav_ScanLine(uint8_t active_level, LineScanResult *scan);

/* 每个控制步调用：输出左右轮目标速度（counts/s），返回 1 表示应停车归零 PWM */
uint8_t TrackNav_Step(int32_t left_delta, int32_t right_delta, float dt_s,
                      float *out_left_target, float *out_right_target);

TrackState TrackNav_GetState(void);
float TrackNav_GetYawDeg(void);
float TrackNav_GetSegMm(void);
TrackFault TrackNav_GetFault(void);
TrackState TrackNav_GetFaultState(void);
uint16_t TrackNav_GetLastLineMask(void);
uint8_t TrackNav_GetLastLineCount(void);
uint8_t TrackNav_IsSearching(void);
int8_t TrackNav_GetSearchDirection(void);
void TrackNav_Abort(TrackFault fault);

#endif /* TRACK_NAV_H */
