#ifndef TURN_TEST_H
#define TURN_TEST_H

/* 发车延时结束后开始零偏校准；Poll 需要在主循环中持续调用。 */
void TurnTest_Start(void);
void TurnTest_Poll(void);

#endif
