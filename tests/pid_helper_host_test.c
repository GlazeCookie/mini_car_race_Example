#include "pid_helper_app.h"
#include "pid_helper_protocol.h"
#include <assert.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart);
static uint32_t tick;
static uint8_t *rx_destination;
static uint8_t *tx_source;
static uint16_t tx_length;
static unsigned int rx_arms, tx_starts, tx_aborts;

uint32_t HAL_GetTick(void) { return tick; }
void HAL_Delay(uint32_t delay_ms) { tick += delay_ms; }
uint32_t __get_PRIMASK(void) { return 0; }
void __disable_irq(void) {}
void __set_PRIMASK(uint32_t value) { (void)value; }
int HAL_UART_Receive_IT(UART_HandleTypeDef *uart, uint8_t *data, uint16_t length) {
  assert(length == 1U); rx_destination = data; uart->RxState = HAL_UART_STATE_BUSY_RX; ++rx_arms;
  return HAL_OK;
}
int HAL_UART_Transmit_IT(UART_HandleTypeDef *uart, uint8_t *data, uint16_t length) {
  if (uart->gState != HAL_UART_STATE_READY) return HAL_BUSY;
  tx_source = data; tx_length = length; uart->gState = HAL_UART_STATE_BUSY_TX; ++tx_starts;
  return HAL_OK;
}
int HAL_UART_AbortTransmit(UART_HandleTypeDef *uart) { uart->gState = HAL_UART_STATE_READY; ++tx_aborts; return HAL_OK; }
int HAL_UART_AbortReceive(UART_HandleTypeDef *uart) { uart->RxState = HAL_UART_STATE_READY; return HAL_OK; }

static void command(UART_HandleTypeDef *uart, uint8_t id, uint8_t seq, float a, float b, float c) {
  uint8_t frame[PID_HELPER_PROTOCOL_COMMAND_FRAME_SIZE] = {0xAA, 0x55, id, seq};
  memcpy(&frame[4], &a, sizeof(a)); memcpy(&frame[8], &b, sizeof(b)); memcpy(&frame[12], &c, sizeof(c));
  frame[16] = PIDHelperProtocol_CalcChecksum(frame);
  for (unsigned int i = 0; i < sizeof(frame); ++i) {
    assert(rx_destination != NULL);
    *rx_destination = frame[i];
    uart->RxState = HAL_UART_STATE_READY;
    HAL_UART_RxCpltCallback(uart);
  }
}

int main(void) {
  UART_HandleTypeDef uart = { HAL_UART_STATE_READY, HAL_UART_STATE_READY };
  float ack_cmd, ack_status;
  PidHelper_Init(&uart, 1.0f, 2.0f, 3.0f, 30000.0f, 5000.0f);
  assert(rx_arms == 1 && tx_starts == 0);
  command(&uart, PID_HELPER_COMMAND_PID, 7, 4, 5, 6);
  assert(rx_arms == 18 && tx_starts == 0 && PidHelper_Kp() == 1.0f);
  tick = 1; PidHelper_Poll();
  assert(PidHelper_Kp() == 4.0f && PidHelper_Ki() == 5.0f && PidHelper_Kd() == 6.0f);
  assert(tx_starts == 1 && tx_length == PID_HELPER_PROTOCOL_ACK_FRAME_SIZE);
  memcpy(&ack_cmd, &tx_source[0], sizeof(ack_cmd));
  memcpy(&ack_status, &tx_source[4], sizeof(ack_status));
  assert(ack_cmd == 1.0f && ack_status == 1.0f);
  tick = 20; PidHelper_Poll();
  assert(tx_starts == 1); /* UART 仍忙，不阻塞、不覆盖 ACK 缓冲。 */
  uart.gState = HAL_UART_STATE_READY; tick = 21; PidHelper_Poll();
  command(&uart, PID_HELPER_COMMAND_PID, 8, NAN, 5, 6);
  tick = 22; PidHelper_Poll();
  assert(PidHelper_Kp() == 4.0f && tx_starts == 2);
  memcpy(&ack_status, &tx_source[4], sizeof(ack_status)); assert(ack_status == 0.0f);
  tick = 123; PidHelper_Poll(); assert(tx_aborts == 1); /* TX 异常不会永久占队列。 */
  uart.RxState = HAL_UART_STATE_READY;
  HAL_UART_ErrorCallback(&uart); tick = 124; PidHelper_Poll();
  command(&uart, PID_HELPER_COMMAND_SP, 9, 12000, 0, 0);
  tick = 125; PidHelper_Poll();
  assert(PidHelper_IsTuning() && PidHelper_TargetSpeed() == 12000.0f);
  unsigned int old_rx = rx_arms, old_tx = tx_starts;
  PidHelper_Init(NULL, 7, 8, 9, 30000, 5000);
  tick += 200; PidHelper_Poll();
  assert(PidHelper_Kp() == 7 && PidHelper_Ki() == 8 && PidHelper_Kd() == 9);
  assert(rx_arms == old_rx && tx_starts == old_tx);
  puts("PID Helper nonblocking host tests: OK");
  return 0;
}
