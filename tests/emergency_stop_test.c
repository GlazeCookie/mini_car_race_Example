#include "../Core/Inc/main.h"
#include <assert.h>
#include <stdio.h>

TestRcc test_rcc;
TestTim test_tim1;

int main(void) {
  test_tim1.CCER = TIM_CCER_CC1E | TIM_CCER_CC2E | (1U << 8);
  test_tim1.BDTR = TIM_BDTR_MOE | 3U;
  test_tim1.CCR1 = 5000U; test_tim1.CCR2 = 5000U;
  MOTOR_EMERGENCY_STOP(); /* 时钟未开时不访问 TIM1。 */
  assert(test_tim1.CCR1 == 5000U && test_tim1.CCR2 == 5000U);
  test_rcc.APB2ENR = RCC_APB2ENR_TIM1EN;
  MOTOR_EMERGENCY_STOP();
  assert((test_tim1.CCER & (TIM_CCER_CC1E | TIM_CCER_CC2E)) == 0U);
  assert((test_tim1.CCER & (1U << 8)) != 0U);
  assert((test_tim1.BDTR & TIM_BDTR_MOE) == 0U);
  assert(test_tim1.CCR1 == 0U && test_tim1.CCR2 == 0U);
  puts("emergency motor disable host test: OK");
  return 0;
}
