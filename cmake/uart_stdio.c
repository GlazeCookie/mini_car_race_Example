/* GNU newlib 的 _write 使用 __io_putchar；复用工程已有的 USART3 printf 输出。 */
#include "main.h"

extern UART_HandleTypeDef huart3;

int __io_putchar(int ch)
{
    uint8_t byte = (uint8_t)ch;
    (void)HAL_UART_Transmit(&huart3, &byte, 1, 0xffff);
    return ch;
}
