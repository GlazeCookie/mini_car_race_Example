#ifndef TEST_STM32F1XX_HAL_H
#define TEST_STM32F1XX_HAL_H
#include <stdint.h>
typedef struct { volatile uint32_t APB2ENR; } TestRcc;
typedef struct { volatile uint32_t CCER, BDTR, CCR1, CCR2; } TestTim;
typedef struct { int unused; } TIM_HandleTypeDef;
extern TestRcc test_rcc;
extern TestTim test_tim1;
#define RCC (&test_rcc)
#define TIM1 (&test_tim1)
#define RCC_APB2ENR_TIM1EN (1U << 11)
#define TIM_CCER_CC1E (1U << 0)
#define TIM_CCER_CC2E (1U << 4)
#define TIM_BDTR_MOE (1U << 15)
#endif
