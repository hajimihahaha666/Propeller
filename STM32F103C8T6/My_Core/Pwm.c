#include "Pwm.h"
#include "tim.h"
#include "gpio.h"

/* Debug: expose last written CCR values for each ESC channel (1..8) */
volatile uint32_t debug_pwm_ccr[8] = {0};
volatile uint8_t debug_pwm_initialized = 0;
volatile uint8_t debug_pwm_test_active = 0;

/* Utility: toggle PC13 (debug LED) to indicate ESC_ApplyToPWM activity */
void DEBUG_Toggle_PC13(void)
{
    HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
}

/********************************************************************************************************
Function Name: Pwm_Init  
Author       : ZFY
Date         : 2025-01-05
Description  :
Outputs      : void
Notes        : 
********************************************************************************************************/
void Pwm_Init()
{
	debug_pwm_initialized = 0;
	
	/* Enable GPIO clocks for PA and PB explicitly */
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();
	
	/* Set initial pulse to mid (1500us) before starting PWM */
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 1500);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 1500);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 1500);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 1500);
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 1500);
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, 1500);
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 1500);
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, 1500);
	
	/* Enable advanced timer main output BEFORE starting channels */
	__HAL_TIM_MOE_ENABLE(&htim1);

	/* 8 路 PWM：TIM1 CH1-CH4 + TIM4 CH1-CH4 */
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);

	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);
	
	/* Store initial CCR values for debug */
	debug_pwm_ccr[0] = htim1.Instance->CCR1;
	debug_pwm_ccr[1] = htim1.Instance->CCR2;
	debug_pwm_ccr[2] = htim1.Instance->CCR3;
	debug_pwm_ccr[3] = htim1.Instance->CCR4;
	debug_pwm_ccr[4] = htim4.Instance->CCR1;
	debug_pwm_ccr[5] = htim4.Instance->CCR2;
	debug_pwm_ccr[6] = htim4.Instance->CCR3;
	debug_pwm_ccr[7] = htim4.Instance->CCR4;
	
	debug_pwm_initialized = 1;
}

/********************************************************************************************************
Function Name: Pwm_SetChannel  
Description  : 设置单个通道的PWM脉宽
Inputs       : channel: 0-7, pulse_us: 1000-2000
Outputs      : void
********************************************************************************************************/
void Pwm_SetChannel(uint8_t channel, uint32_t pulse_us)
{
	if (channel >= 8) return;
	if (pulse_us < 1000) pulse_us = 1000;
	if (pulse_us > 2000) pulse_us = 2000;
	
	debug_pwm_ccr[channel] = pulse_us;
	
	if (channel < 4) {
		uint32_t ch = (channel == 0) ? TIM_CHANNEL_1 : 
		              (channel == 1) ? TIM_CHANNEL_2 : 
		              (channel == 2) ? TIM_CHANNEL_3 : TIM_CHANNEL_4;
		__HAL_TIM_SET_COMPARE(&htim1, ch, pulse_us);
	} else {
		uint32_t ch = (channel == 4) ? TIM_CHANNEL_1 : 
		              (channel == 5) ? TIM_CHANNEL_2 : 
		              (channel == 6) ? TIM_CHANNEL_3 : TIM_CHANNEL_4;
		__HAL_TIM_SET_COMPARE(&htim4, ch, pulse_us);
	}
}

/********************************************************************************************************
Function Name: Pwm_Test_Sweep  
Description  : 测试PWM输出，从1000us到2000us来回扫描
Outputs      : void
********************************************************************************************************/
void Pwm_Test_Sweep(void)
{
	debug_pwm_test_active = 1;
	
	/* Set all channels to 1000us */
	for (int i = 0; i < 8; i++) {
		Pwm_SetChannel(i, 1000);
	}
	HAL_Delay(500);
	
	/* Sweep up to 2000us */
	for (uint32_t pulse = 1000; pulse <= 2000; pulse += 10) {
		for (int i = 0; i < 8; i++) {
			Pwm_SetChannel(i, pulse);
		}
		HAL_Delay(10);
	}
	
	/* Sweep down to 1000us */
	for (uint32_t pulse = 2000; pulse >= 1000; pulse -= 10) {
		for (int i = 0; i < 8; i++) {
			Pwm_SetChannel(i, pulse);
		}
		HAL_Delay(10);
	}
	
	/* Back to mid */
	for (int i = 0; i < 8; i++) {
		Pwm_SetChannel(i, 1500);
	}
	
	debug_pwm_test_active = 0;
}

/********************************************************************************************************
Function Name: Pwm_Test_Fixed  
Description  : 设置固定的测试值，便于用示波器测量
Inputs       : pulse_us: 脉宽值
Outputs      : void
********************************************************************************************************/
void Pwm_Test_Fixed(uint32_t pulse_us)
{
	debug_pwm_test_active = 1;
	
	for (int i = 0; i < 8; i++) {
		Pwm_SetChannel(i, pulse_us);
	}
}






























