/*
 * pid_helper_protocol.h  —  PID Helper 上位机串口协议（可移植版）
 *
 * 与官方示例 User/pid_helper_protocol.h 的协议行为完全一致，但去掉了
 * stm32f1xx_hal.h 依赖，改为纯 C99 + 一个发送回调，因此可以放进任何
 * 具备 UART 发送能力的工程（STM32 / GD32 / APM32 / ESP32 / 沁恒 / AVR 等）。
 *
 * 协议要点（务必与上位机一致，不要改）：
 *   命令帧 17 字节: AA 55 cmd seq arg0(f32) arg1(f32) arg2(f32) sum
 *     - arg0/arg1/arg2 为 float32 小端
 *     - sum = 前 16 字节逐字节累加后取低 8 位
 *   下行命令号: 1 PID(kp,ki,kd) / 2 MODE(0闭环 1开环) / 3 OUT(开环输出)
 *              4 SP(目标值)     / 5 APPMODE(0速度环 1位置环)
 *
 *   应答 ACK 20 字节: cmd(f32) status(f32) value(f32) seq(f32) + 00 00 80 7F
 *     - status: 1 成功 / 0 失败；cmd 和 seq 必须原样回显，否则上位机匹配不上
 *
 *   遥测 36 字节: timestamp(u32) actual target input error p i d (f32) + 00 00 80 7F
 *     - 全部小端；建议 20ms 发一次（上位机按 30s/600 点画图）
 *
 * 移植三步：
 *   1) 实现 tx_callback：把 data/length 通过串口发出去，成功返回 0
 *   2) 在 UART 接收中断/DMA 回调里，把收到的每个字节喂给
 *      PIDHelperProtocol_InputByte()，或用 Input() 整块喂
 *   3) 主循环里每 20ms 调一次 PIDHelperProtocol_SendTelemetry()
 *   然后实现 on_pid / on_mode / on_output / on_setpoint / on_appmode 五个回调，
 *   在回调里改你自己控制器的参数，返回 MakeResult(1, 0.0f) 表示接受。
 *
 * 注意：回调是在解析上下文同步执行的。如果 tx_callback 是阻塞发送，建议把
 * PIDHelperProtocol_InputByte() 放在主循环（而不是中断里）调用，避免长时间占用中断。
 */

#ifndef PID_HELPER_PROTOCOL_H
#define PID_HELPER_PROTOCOL_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stddef.h>
#include <stdint.h>

#define PID_HELPER_PROTOCOL_COMMAND_FRAME_SIZE    17u
#define PID_HELPER_PROTOCOL_ACK_FRAME_SIZE        20u
#define PID_HELPER_PROTOCOL_TELEMETRY_FRAME_SIZE  36u

#define PID_HELPER_PROTOCOL_HEADER0               0xAAu
#define PID_HELPER_PROTOCOL_HEADER1               0x55u

/* 发送回调返回值约定 */
#define PID_HELPER_TX_OK      0
#define PID_HELPER_TX_FAIL   -1

typedef enum {
    PID_HELPER_COMMAND_PID = 1u,
    PID_HELPER_COMMAND_MODE = 2u,
    PID_HELPER_COMMAND_OUT = 3u,
    PID_HELPER_COMMAND_SP = 4u,
    PID_HELPER_COMMAND_APPMODE = 5u
} PIDHelperProtocolCommandId;

typedef enum {
    PID_HELPER_MODE_CLOSED = 0u,
    PID_HELPER_MODE_OPEN = 1u
} PIDHelperProtocolMode;

typedef struct {
    uint32_t timestamp_ms;
    float actual;   /* 当前反馈：速度环 rad/s，位置环 degree */
    float target;   /* 目标值   ：速度环 rad/s，位置环 degree */
    float input;    /* 控制器输出 / Uq */
    float error;    /* 误差 */
    float p;
    float i;
    float d;
} PIDHelperProtocolTelemetry;

typedef struct {
    uint8_t command_id;
    uint8_t seq;
    float arg0;
    float arg1;
    float arg2;
} PIDHelperProtocolCommand;

typedef struct {
    uint8_t command_id;
    uint8_t status_code;   /* 1 成功 / 0 失败 */
    float value;
    uint8_t seq;
} PIDHelperProtocolAck;

typedef struct {
    uint8_t accepted;      /* 1 接受 / 0 拒绝，会体现在 ACK 的 status 里 */
    float value;           /* 回传值，一般填 0.0f */
} PIDHelperProtocolCommandResult;

typedef struct PIDHelperProtocol PIDHelperProtocol;

/* 发送回调：返回 PID_HELPER_TX_OK(0) 表示发送成功 */
typedef int (*PIDHelperProtocolTxFn)(void *tx_ctx, const uint8_t *data, uint16_t length);

typedef PIDHelperProtocolCommandResult (*PIDHelperProtocolOnPidFn)(void *user_arg,
                                                                  float p, float i, float d,
                                                                  uint8_t seq);
typedef PIDHelperProtocolCommandResult (*PIDHelperProtocolOnModeFn)(void *user_arg,
                                                                   uint8_t mode, uint8_t seq);
typedef PIDHelperProtocolCommandResult (*PIDHelperProtocolOnOutputFn)(void *user_arg,
                                                                     float output, uint8_t seq);
typedef PIDHelperProtocolCommandResult (*PIDHelperProtocolOnSetpointFn)(void *user_arg,
                                                                       float setpoint, uint8_t seq);
typedef PIDHelperProtocolCommandResult (*PIDHelperProtocolOnAppModeFn)(void *user_arg,
                                                                      uint8_t appmode, uint8_t seq);
typedef void (*PIDHelperProtocolOnUnknownCommandFn)(void *user_arg,
                                                    const PIDHelperProtocolCommand *command);
typedef void (*PIDHelperProtocolOnChecksumErrorFn)(void *user_arg,
                                                   const uint8_t frame[PID_HELPER_PROTOCOL_COMMAND_FRAME_SIZE]);

typedef struct {
    void *tx_ctx;
    PIDHelperProtocolTxFn tx_callback;              /* 必须实现 */
    void *user_arg;                                 /* 下面回调的上下文，一般填你的控制器结构体 */
    PIDHelperProtocolOnPidFn on_pid;                /* cmd=1 */
    PIDHelperProtocolOnModeFn on_mode;              /* cmd=2 */
    PIDHelperProtocolOnOutputFn on_output;          /* cmd=3 */
    PIDHelperProtocolOnSetpointFn on_setpoint;      /* cmd=4 */
    PIDHelperProtocolOnAppModeFn on_appmode;        /* cmd=5 */
    PIDHelperProtocolOnUnknownCommandFn on_unknown_command;
    PIDHelperProtocolOnChecksumErrorFn on_checksum_error;
} PIDHelperProtocolConfig;

struct PIDHelperProtocol {
    PIDHelperProtocolConfig config;
    uint8_t rx_frame[PID_HELPER_PROTOCOL_COMMAND_FRAME_SIZE];
    uint8_t rx_index;
    uint8_t rx_state;
};

void PIDHelperProtocol_Init(PIDHelperProtocol *protocol, const PIDHelperProtocolConfig *config);

void PIDHelperProtocol_ResetRx(PIDHelperProtocol *protocol);

void PIDHelperProtocol_SetUserArg(PIDHelperProtocol *protocol, void *user_arg);

uint8_t PIDHelperProtocol_CalcChecksum(const uint8_t *frame16);

PIDHelperProtocolCommandResult PIDHelperProtocol_MakeResult(uint8_t accepted, float value);

int32_t PIDHelperProtocol_DecodeCommandFrame(
    const uint8_t frame[PID_HELPER_PROTOCOL_COMMAND_FRAME_SIZE],
    PIDHelperProtocolCommand *command);

int32_t PIDHelperProtocol_BuildAckFrame(
    const PIDHelperProtocolAck *ack,
    uint8_t out_frame[PID_HELPER_PROTOCOL_ACK_FRAME_SIZE]);

int32_t PIDHelperProtocol_BuildTelemetryFrame(
    const PIDHelperProtocolTelemetry *telemetry,
    uint8_t out_frame[PID_HELPER_PROTOCOL_TELEMETRY_FRAME_SIZE]);

/* 返回 PID_HELPER_TX_OK / PID_HELPER_TX_FAIL */
int32_t PIDHelperProtocol_SendAck(PIDHelperProtocol *protocol, const PIDHelperProtocolAck *ack);

int32_t PIDHelperProtocol_SendTelemetry(PIDHelperProtocol *protocol,
                                        const PIDHelperProtocolTelemetry *telemetry);

int32_t PIDHelperProtocol_DispatchCommand(PIDHelperProtocol *protocol,
                                          const PIDHelperProtocolCommand *command);

/*
 * 输入一个字节，返回值：
 *   1: 收到并处理了一帧有效命令，且 ACK 发送成功
 *   0: 还没拼满一帧
 *  -1: 收到一帧但校验失败
 *  -2: 命令有效但 ACK 发送失败
 */
int32_t PIDHelperProtocol_InputByte(PIDHelperProtocol *protocol, uint8_t byte);

/* 整块喂字节，返回实际消耗的字节数 */
size_t PIDHelperProtocol_Input(PIDHelperProtocol *protocol, const uint8_t *data, size_t length);

#ifdef __cplusplus
}
#endif

#endif /* PID_HELPER_PROTOCOL_H */
