#include "pid_helper_app.h"
#include "pid_helper_protocol.h"

#include <string.h>

/* PID 增益：默认取 main.c 传进来的编译期参数，上位机下发后覆盖。
   这几项放在开关外面，因为关闭调参功能时 SpeedPid_Update 仍然通过
   PidHelper_Kp()/Ki()/Kd() 取值。 */
static float s_kp = 0.0f;
static float s_ki = 0.0f;
static float s_kd = 0.0f;

#if PID_HELPER_ENABLE

#define PH_TELEMETRY_PERIOD_MS 20u
#define PH_TX_TIMEOUT_MS       20u
/* 接管期间如果连续这么久没收到任何命令，就把目标速度和开环输出清零（停车），
   但保持接管状态，避免突然把控制权还给循迹导致车子自己跑出去。 */
#define PH_TUNING_TIMEOUT_MS   15000u

static PIDHelperProtocol s_proto;
static UART_HandleTypeDef *s_huart;
static uint8_t s_rx_byte;

/* 限幅 */
static float s_max_target;
static float s_max_pwm;

/* 上位机状态 */
static uint8_t s_tuning;      /* 1 = 上位机接管，循迹让位 */
static uint8_t s_open_mode;   /* 1 = 开环，OUT 直接当 PWM */
static float   s_target_speed;
static float   s_open_output;

/* 上报给上位机的量（由 PidHelper_Report 更新） */
static float s_actual;
static float s_target;
static float s_input;

static uint32_t s_last_tx_tick;
static uint32_t s_last_cmd_tick;

static float ph_clamp(float value, float minimum, float maximum)
{
    if (value < minimum) return minimum;
    if (value > maximum) return maximum;
    return value;
}

static int ph_tx(void *ctx, const uint8_t *data, uint16_t length)
{
    UART_HandleTypeDef *huart = (UART_HandleTypeDef *)ctx;

    if (huart == NULL) {
        return PID_HELPER_TX_FAIL;
    }

    /* 阻塞发送：36 字节约 3.1 ms（115200），会占用主循环时间；
       目标控制周期为 5 ms 时需实测时序，同时需保证 ACK 及时发送。 */
    return (HAL_UART_Transmit(huart, (uint8_t *)data, length, PH_TX_TIMEOUT_MS) == HAL_OK)
               ? PID_HELPER_TX_OK
               : PID_HELPER_TX_FAIL;
}

/* cmd=1：下发 PID。只改增益，不触发接管，避免调增益时车子突然停下 */
static PIDHelperProtocolCommandResult ph_on_pid(void *user_arg,
                                                float p, float i, float d,
                                                uint8_t seq)
{
    (void)user_arg;
    (void)seq;

    s_last_cmd_tick = HAL_GetTick();
    s_kp = ph_clamp(p, 0.0f, 1000.0f);
    s_ki = ph_clamp(i, 0.0f, 1000.0f);
    s_kd = ph_clamp(d, 0.0f, 1000.0f);
    return PIDHelperProtocol_MakeResult(1u, 0.0f);
}

/* cmd=2：闭环 / 开环 */
static PIDHelperProtocolCommandResult ph_on_mode(void *user_arg, uint8_t mode, uint8_t seq)
{
    (void)user_arg;
    (void)seq;

    s_last_cmd_tick = HAL_GetTick();
    s_open_mode = (mode == PID_HELPER_MODE_OPEN) ? 1u : 0u;
    if (s_open_mode != 0u) {
        s_open_output = 0.0f; /* 进开环先把输出清零，等 OUT 下发 */
    }
    return PIDHelperProtocol_MakeResult(1u, 0.0f);
}

/* cmd=3：开环输出。收到即接管 */
static PIDHelperProtocolCommandResult ph_on_output(void *user_arg, float output, uint8_t seq)
{
    (void)user_arg;
    (void)seq;

    s_last_cmd_tick = HAL_GetTick();
    s_open_output = ph_clamp(output, 0.0f, s_max_pwm);
    s_open_mode = 1u;
    s_tuning = 1u;
    return PIDHelperProtocol_MakeResult(1u, 0.0f);
}

/* cmd=4：目标速度。收到即接管，两轮同目标 */
static PIDHelperProtocolCommandResult ph_on_setpoint(void *user_arg, float setpoint, uint8_t seq)
{
    (void)user_arg;
    (void)seq;

    /* 本车 PWM 单向，负速度按 0 处理 */
    s_last_cmd_tick = HAL_GetTick();
    s_target_speed = ph_clamp(setpoint, 0.0f, s_max_target);
    s_open_mode = 0u;
    s_tuning = 1u;
    return PIDHelperProtocol_MakeResult(1u, 0.0f);
}

/* cmd=5：应用背景。本车只有速度环，位置环也按速度环处理，但必须回 ACK，
   否则上位机「打开串口」会卡在等待 ACK 超时（1.5 s）然后自动断开。 */
static PIDHelperProtocolCommandResult ph_on_appmode(void *user_arg, uint8_t appmode, uint8_t seq)
{
    (void)user_arg;
    (void)appmode;
    (void)seq;

    s_last_cmd_tick = HAL_GetTick();
    return PIDHelperProtocol_MakeResult(1u, 0.0f);
}

void PidHelper_Init(UART_HandleTypeDef *huart,
                    float kp,
                    float ki,
                    float kd,
                    float max_target,
                    float max_pwm)
{
    PIDHelperProtocolConfig config;

    s_huart = huart;
    s_kp = kp;
    s_ki = ki;
    s_kd = kd;
    s_max_target = (max_target > 1.0f) ? max_target : 1.0f;
    s_max_pwm = (max_pwm > 1.0f) ? max_pwm : 1.0f;

    s_tuning = 0u;
    s_open_mode = 0u;
    s_target_speed = 0.0f;
    s_open_output = 0.0f;

    s_actual = 0.0f;
    s_target = 0.0f;
    s_input = 0.0f;

    memset(&config, 0, sizeof(config));
    config.tx_ctx = huart;
    config.tx_callback = ph_tx;
    config.user_arg = NULL;
    config.on_pid = ph_on_pid;
    config.on_mode = ph_on_mode;
    config.on_output = ph_on_output;
    config.on_setpoint = ph_on_setpoint;
    config.on_appmode = ph_on_appmode;

    PIDHelperProtocol_Init(&s_proto, &config);

    s_last_tx_tick = HAL_GetTick();
    s_last_cmd_tick = s_last_tx_tick;

    if (huart != NULL) {
        (void)HAL_UART_Receive_IT(huart, &s_rx_byte, 1u);
    }
}

/* 每收一个字节立刻重新挂一次中断接收，保证 17 字节命令帧连续到达时不丢字节 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if ((s_huart == NULL) || (huart != s_huart)) {
        return;
    }

    (void)PIDHelperProtocol_InputByte(&s_proto, s_rx_byte);
    (void)HAL_UART_Receive_IT(huart, &s_rx_byte, 1u);
}

void PidHelper_Poll(void)
{
    uint32_t now;

    if (s_huart == NULL) {
        return;
    }

    now = HAL_GetTick();

    /* 20 ms 一帧遥测，上位机按 30 s / 600 点画图 */
    if ((uint32_t)(now - s_last_tx_tick) >= PH_TELEMETRY_PERIOD_MS) {
        PIDHelperProtocolTelemetry telemetry;

        s_last_tx_tick = now;

        telemetry.timestamp_ms = now;
        telemetry.actual = s_actual;
        telemetry.target = s_target;
        telemetry.input = s_input;
        telemetry.error = s_target - s_actual;
        telemetry.p = s_kp;
        telemetry.i = s_ki;
        telemetry.d = s_kd;

        (void)PIDHelperProtocol_SendTelemetry(&s_proto, &telemetry);
    }

    /* 接管期间长时间收不到命令：停车，但保持接管状态，
       不把控制权还给循迹，避免车子自己跑出去 */
    if ((s_tuning != 0u) && ((uint32_t)(now - s_last_cmd_tick) >= PH_TUNING_TIMEOUT_MS)) {
        s_target_speed = 0.0f;
        s_open_output = 0.0f;
    }

    /* 溢出/帧错误会让 HAL 停在错误态，这里兜底重新挂上接收 */
    if ((s_huart->RxState != HAL_UART_STATE_READY) &&
        (s_huart->RxState != HAL_UART_STATE_BUSY_RX)) {
        (void)HAL_UART_AbortReceive(s_huart);
        (void)HAL_UART_Receive_IT(s_huart, &s_rx_byte, 1u);
    }
}

void PidHelper_Report(float actual, float target, float input)
{
    s_actual = actual;
    s_target = target;
    s_input = input;
}

uint8_t PidHelper_IsTuning(void) { return s_tuning; }
uint8_t PidHelper_IsOpenMode(void) { return s_open_mode; }
float PidHelper_TargetSpeed(void) { return s_target_speed; }
float PidHelper_OpenOutput(void) { return s_open_output; }

#else /* —— 关闭调参功能时全部退化成空实现，main.c 不必再改 —— */

void PidHelper_Init(UART_HandleTypeDef *huart,
                    float kp,
                    float ki,
                    float kd,
                    float max_target,
                    float max_pwm)
{
    s_kp = kp;
    s_ki = ki;
    s_kd = kd;
    (void)huart; (void)max_target; (void)max_pwm;
}

void PidHelper_Poll(void) {}

void PidHelper_Report(float actual, float target, float input)
{
    (void)actual; (void)target; (void)input;
}

uint8_t PidHelper_IsTuning(void) { return 0u; }
uint8_t PidHelper_IsOpenMode(void) { return 0u; }
float PidHelper_TargetSpeed(void) { return 0.0f; }
float PidHelper_OpenOutput(void) { return 0.0f; }

#endif /* PID_HELPER_ENABLE */

float PidHelper_Kp(void) { return s_kp; }
float PidHelper_Ki(void) { return s_ki; }
float PidHelper_Kd(void) { return s_kd; }
