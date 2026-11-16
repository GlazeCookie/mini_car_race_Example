#ifndef TEST_MAIN_H
#define TEST_MAIN_H
#include <stdint.h>
typedef enum {
  HAL_UART_STATE_READY = 0,
  HAL_UART_STATE_BUSY_TX = 1,
  HAL_UART_STATE_BUSY_RX = 2
} HAL_UART_StateTypeDef;
typedef struct {
  volatile HAL_UART_StateTypeDef gState, RxState;
} UART_HandleTypeDef;
#define HAL_OK 0
#define HAL_BUSY 1
uint32_t HAL_GetTick(void);
void HAL_Delay(uint32_t delay_ms);
int HAL_UART_Receive_IT(UART_HandleTypeDef *huart, uint8_t *data, uint16_t length);
int HAL_UART_Transmit_IT(UART_HandleTypeDef *huart, uint8_t *data, uint16_t length);
int HAL_UART_AbortTransmit(UART_HandleTypeDef *huart);
int HAL_UART_AbortReceive(UART_HandleTypeDef *huart);
uint32_t __get_PRIMASK(void);
void __disable_irq(void);
void __set_PRIMASK(uint32_t value);
#endif
