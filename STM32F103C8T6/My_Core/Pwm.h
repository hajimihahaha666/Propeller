#ifndef PWM_H
#define PWM_H

#include "main.h"

/* Debug variables */
extern volatile uint32_t debug_pwm_ccr[8];
extern volatile uint8_t debug_pwm_initialized;
extern volatile uint8_t debug_pwm_test_active;

void Pwm_Init(void); /* PWM初始化 */
void Pwm_SetChannel(uint8_t channel, uint32_t pulse_us); /* 设置单个通道PWM */
void Pwm_Test_Sweep(void); /* PWM扫描测试 */
void Pwm_Test_Fixed(uint32_t pulse_us); /* 固定值测试 */
void DEBUG_Toggle_PC13(void); /* 翻转PC13 LED */

#endif // PWM_H
