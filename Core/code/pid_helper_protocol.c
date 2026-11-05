#include "pid_helper_protocol.h"

#include <string.h>

#if defined(__STDC_VERSION__) && (__STDC_VERSION__ >= 201112L)
_Static_assert(sizeof(float) == 4u, "PID helper protocol requires 32-bit float.");
_Static_assert(sizeof(uint32_t) == 4u, "PID helper protocol requires 32-bit uint32_t.");
#else
typedef char pid_helper_float_must_be_4_bytes[(sizeof(float) == 4u) ? 1 : -1];
typedef char pid_helper_u32_must_be_4_bytes[(sizeof(uint32_t) == 4u) ? 1 : -1];
#endif

enum {
    PID_HELPER_RX_STATE_WAIT_HEADER0 = 0u,
    PID_HELPER_RX_STATE_WAIT_HEADER1 = 1u,
    PID_HELPER_RX_STATE_WAIT_BODY = 2u
};

/* ACK 和遥测共用的 4 字节帧尾: 00 00 80 7F */
static const uint8_t pid_helper_frame_tail[4] = {0x00u, 0x00u, 0x80u, 0x7Fu};

static uint32_t pid_helper_read_u32_le(const uint8_t *data)
{
    return ((uint32_t)data[0]) |
           ((uint32_t)data[1] << 8) |
           ((uint32_t)data[2] << 16) |
           ((uint32_t)data[3] << 24);
}

static void pid_helper_write_u32_le(uint8_t *data, uint32_t value)
{
    data[0] = (uint8_t)(value & 0xFFu);
    data[1] = (uint8_t)((value >> 8) & 0xFFu);
    data[2] = (uint8_t)((value >> 16) & 0xFFu);
    data[3] = (uint8_t)((value >> 24) & 0xFFu);
}

static float pid_helper_read_f32_le(const uint8_t *data)
{
    uint32_t raw = pid_helper_read_u32_le(data);
    float value = 0.0f;

    /* 协议固定 float32 小端传输，按位还原，避免结构体对齐/padding 问题 */
    memcpy(&value, &raw, sizeof(value));
    return value;
}

static void pid_helper_write_f32_le(uint8_t *data, float value)
{
    uint32_t raw = 0u;

    memcpy(&raw, &value, sizeof(raw));
    pid_helper_write_u32_le(data, raw);
}

static int32_t pid_helper_write(PIDHelperProtocol *protocol, const uint8_t *data, uint16_t length)
{
    if ((protocol == NULL) || (data == NULL) || (protocol->config.tx_callback == NULL)) {
        return PID_HELPER_TX_FAIL;
    }

    return (protocol->config.tx_callback(protocol->config.tx_ctx, data, length) == PID_HELPER_TX_OK)
               ? PID_HELPER_TX_OK
               : PID_HELPER_TX_FAIL;
}

void PIDHelperProtocol_Init(PIDHelperProtocol *protocol, const PIDHelperProtocolConfig *config)
{
    if (protocol == NULL) {
        return;
    }

    memset(protocol, 0, sizeof(*protocol));
    if (config != NULL) {
        protocol->config = *config;
    }
    protocol->rx_state = PID_HELPER_RX_STATE_WAIT_HEADER0;
}

void PIDHelperProtocol_ResetRx(PIDHelperProtocol *protocol)
{
    if (protocol == NULL) {
        return;
    }

    protocol->rx_index = 0u;
    protocol->rx_state = PID_HELPER_RX_STATE_WAIT_HEADER0;
}

void PIDHelperProtocol_SetUserArg(PIDHelperProtocol *protocol, void *user_arg)
{
    if (protocol == NULL) {
        return;
    }

    protocol->config.user_arg = user_arg;
}

uint8_t PIDHelperProtocol_CalcChecksum(const uint8_t *frame16)
{
    uint16_t i = 0u;
    uint32_t sum = 0u;

    if (frame16 == NULL) {
        return 0u;
    }

    /* 校验和规则：前 16 字节逐字节累加，只保留低 8 位 */
    for (i = 0u; i < 16u; ++i) {
        sum += frame16[i];
    }

    return (uint8_t)(sum & 0xFFu);
}

PIDHelperProtocolCommandResult PIDHelperProtocol_MakeResult(uint8_t accepted, float value)
{
    PIDHelperProtocolCommandResult result;

    result.accepted = (accepted != 0u) ? 1u : 0u;
    result.value = value;
    return result;
}

int32_t PIDHelperProtocol_DecodeCommandFrame(
    const uint8_t frame[PID_HELPER_PROTOCOL_COMMAND_FRAME_SIZE],
    PIDHelperProtocolCommand *command)
{
    if ((frame == NULL) || (command == NULL)) {
        return -1;
    }

    if ((frame[0] != PID_HELPER_PROTOCOL_HEADER0) ||
        (frame[1] != PID_HELPER_PROTOCOL_HEADER1)) {
        return -1;
    }

    if (PIDHelperProtocol_CalcChecksum(frame) != frame[16]) {
        return -1;
    }

    command->command_id = frame[2];
    command->seq = frame[3];
    command->arg0 = pid_helper_read_f32_le(&frame[4]);
    command->arg1 = pid_helper_read_f32_le(&frame[8]);
    command->arg2 = pid_helper_read_f32_le(&frame[12]);
    return 0;
}

int32_t PIDHelperProtocol_BuildAckFrame(
    const PIDHelperProtocolAck *ack,
    uint8_t out_frame[PID_HELPER_PROTOCOL_ACK_FRAME_SIZE])
{
    if ((ack == NULL) || (out_frame == NULL)) {
        return -1;
    }

    /* ACK 的 4 个字段都按 float32 小端编码 */
    pid_helper_write_f32_le(&out_frame[0], (float)ack->command_id);
    pid_helper_write_f32_le(&out_frame[4], (ack->status_code != 0u) ? 1.0f : 0.0f);
    pid_helper_write_f32_le(&out_frame[8], ack->value);
    pid_helper_write_f32_le(&out_frame[12], (float)ack->seq);
    memcpy(&out_frame[16], pid_helper_frame_tail, sizeof(pid_helper_frame_tail));
    return 0;
}

int32_t PIDHelperProtocol_BuildTelemetryFrame(
    const PIDHelperProtocolTelemetry *telemetry,
    uint8_t out_frame[PID_HELPER_PROTOCOL_TELEMETRY_FRAME_SIZE])
{
    if ((telemetry == NULL) || (out_frame == NULL)) {
        return -1;
    }

    pid_helper_write_u32_le(&out_frame[0], telemetry->timestamp_ms);
    pid_helper_write_f32_le(&out_frame[4], telemetry->actual);
    pid_helper_write_f32_le(&out_frame[8], telemetry->target);
    pid_helper_write_f32_le(&out_frame[12], telemetry->input);
    pid_helper_write_f32_le(&out_frame[16], telemetry->error);
    pid_helper_write_f32_le(&out_frame[20], telemetry->p);
    pid_helper_write_f32_le(&out_frame[24], telemetry->i);
    pid_helper_write_f32_le(&out_frame[28], telemetry->d);
    memcpy(&out_frame[32], pid_helper_frame_tail, sizeof(pid_helper_frame_tail));
    return 0;
}

int32_t PIDHelperProtocol_SendAck(PIDHelperProtocol *protocol, const PIDHelperProtocolAck *ack)
{
    uint8_t frame[PID_HELPER_PROTOCOL_ACK_FRAME_SIZE];

    if (PIDHelperProtocol_BuildAckFrame(ack, frame) != 0) {
        return PID_HELPER_TX_FAIL;
    }

    return pid_helper_write(protocol, frame, (uint16_t)sizeof(frame));
}

int32_t PIDHelperProtocol_SendTelemetry(PIDHelperProtocol *protocol,
                                        const PIDHelperProtocolTelemetry *telemetry)
{
    uint8_t frame[PID_HELPER_PROTOCOL_TELEMETRY_FRAME_SIZE];

    if (PIDHelperProtocol_BuildTelemetryFrame(telemetry, frame) != 0) {
        return PID_HELPER_TX_FAIL;
    }

    return pid_helper_write(protocol, frame, (uint16_t)sizeof(frame));
}

int32_t PIDHelperProtocol_DispatchCommand(PIDHelperProtocol *protocol,
                                          const PIDHelperProtocolCommand *command)
{
    PIDHelperProtocolCommandResult result;
    PIDHelperProtocolAck ack;
    uint8_t known_command = 1u;

    if ((protocol == NULL) || (command == NULL)) {
        return PID_HELPER_TX_FAIL;
    }

    result = PIDHelperProtocol_MakeResult(0u, 0.0f);

    /* 协议层只负责分发和回 ACK，具体控制动作交给用户回调 */
    switch (command->command_id) {
    case PID_HELPER_COMMAND_PID:
        if (protocol->config.on_pid != NULL) {
            result = protocol->config.on_pid(protocol->config.user_arg,
                                             command->arg0, command->arg1, command->arg2,
                                             command->seq);
        }
        break;

    case PID_HELPER_COMMAND_MODE:
        if (protocol->config.on_mode != NULL) {
            uint8_t mode = (command->arg0 >= 0.5f) ? PID_HELPER_MODE_OPEN
                                                   : PID_HELPER_MODE_CLOSED;
            result = protocol->config.on_mode(protocol->config.user_arg, mode, command->seq);
        }
        break;

    case PID_HELPER_COMMAND_OUT:
        if (protocol->config.on_output != NULL) {
            result = protocol->config.on_output(protocol->config.user_arg, command->arg0, command->seq);
        }
        break;

    case PID_HELPER_COMMAND_SP:
        if (protocol->config.on_setpoint != NULL) {
            result = protocol->config.on_setpoint(protocol->config.user_arg, command->arg0, command->seq);
        }
        break;

    case PID_HELPER_COMMAND_APPMODE:
        if (protocol->config.on_appmode != NULL) {
            /* 注意：APPMODE 即使回调没实现也必须回 ACK，
               否则上位机"打开串口"时会一直卡在等待 ACK 超时 */
            uint8_t appmode = (command->arg0 >= 0.5f) ? 1u : 0u;
            result = protocol->config.on_appmode(protocol->config.user_arg, appmode, command->seq);
        } else {
            result = PIDHelperProtocol_MakeResult(1u, 0.0f);
        }
        break;

    default:
        known_command = 0u;
        if (protocol->config.on_unknown_command != NULL) {
            protocol->config.on_unknown_command(protocol->config.user_arg, command);
        }
        break;
    }

    if (known_command == 0u) {
        return PID_HELPER_TX_OK;
    }

    /* ACK 必须原样回显命令号和 seq，上位机据此匹配 */
    ack.command_id = command->command_id;
    ack.status_code = (result.accepted != 0u) ? 1u : 0u;
    ack.value = result.value;
    ack.seq = command->seq;

    return PIDHelperProtocol_SendAck(protocol, &ack);
}

int32_t PIDHelperProtocol_InputByte(PIDHelperProtocol *protocol, uint8_t byte)
{
    PIDHelperProtocolCommand command;
    int32_t status = 0;
    int32_t tx_status = PID_HELPER_TX_OK;

    if (protocol == NULL) {
        return -1;
    }

    switch (protocol->rx_state) {
    case PID_HELPER_RX_STATE_WAIT_HEADER0:
        /* 找第一个帧头 0xAA */
        if (byte == PID_HELPER_PROTOCOL_HEADER0) {
            protocol->rx_frame[0] = byte;
            protocol->rx_index = 1u;
            protocol->rx_state = PID_HELPER_RX_STATE_WAIT_HEADER1;
        }
        return 0;

    case PID_HELPER_RX_STATE_WAIT_HEADER1:
        /* 第二字节必须是 0x55；若又收到 0xAA 则当作新起点重同步 */
        if (byte == PID_HELPER_PROTOCOL_HEADER1) {
            protocol->rx_frame[1] = byte;
            protocol->rx_index = 2u;
            protocol->rx_state = PID_HELPER_RX_STATE_WAIT_BODY;
        } else if (byte == PID_HELPER_PROTOCOL_HEADER0) {
            protocol->rx_frame[0] = byte;
            protocol->rx_index = 1u;
            protocol->rx_state = PID_HELPER_RX_STATE_WAIT_HEADER1;
        } else {
            PIDHelperProtocol_ResetRx(protocol);
        }
        return 0;

    case PID_HELPER_RX_STATE_WAIT_BODY:
    default:
        protocol->rx_frame[protocol->rx_index] = byte;
        ++protocol->rx_index;

        /* 收满固定 17 字节后再统一校验，避免半包问题 */
        if (protocol->rx_index < PID_HELPER_PROTOCOL_COMMAND_FRAME_SIZE) {
            return 0;
        }

        status = PIDHelperProtocol_DecodeCommandFrame(protocol->rx_frame, &command);
        PIDHelperProtocol_ResetRx(protocol);

        if (status != 0) {
            if (protocol->config.on_checksum_error != NULL) {
                protocol->config.on_checksum_error(protocol->config.user_arg, protocol->rx_frame);
            }
            return -1;
        }

        tx_status = PIDHelperProtocol_DispatchCommand(protocol, &command);
        if (tx_status != PID_HELPER_TX_OK) {
            return -2;
        }
        return 1;
    }
}

size_t PIDHelperProtocol_Input(PIDHelperProtocol *protocol, const uint8_t *data, size_t length)
{
    size_t i = 0u;

    if ((protocol == NULL) || (data == NULL)) {
        return 0u;
    }

    for (i = 0u; i < length; ++i) {
        (void)PIDHelperProtocol_InputByte(protocol, data[i]);
    }

    return length;
}
