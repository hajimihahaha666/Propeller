/*********************************************************************************************************/
/*
                   _ooOoo_
                  o8888888o
                  88" . "88
                  (| -_- |)
                  O\  =  /O
               ____/`---'\____
             .'  \\|     |//  `.
            /  \\|||  :  |||//  \
           /  _||||| -:- |||||-  \
           |   | \\\  -  /// |   |
           | \_|  ''\---/''  |   |
            \  .-\__  `-`  ___/-. /
         ___`. .'  /--.--\  `. . __
      ."" '<  `.___\_<|>_/___.'  >'"".
     | | :  `- \`.;`\ _ /`;.`/ - ` : | |
      \  \ `-.   \_ __\ /__ _/   .-` /  /
======`-.____`-.___\_____/___.-`____.-'======
                   `=---='
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
         佛祖保佑       永无BUG
*/
/*********************************************************************************************************/
#include "App.h"
#include "esc_spi.h"
#include "spi_slave.h"
#include "tim.h"
#include "stm32f1xx_hal.h"

/* 调试变量统一放在 App.c，方便在调试器中查看。
   这些变量由 ESC SPI 帧解析器和 SPI 从机回调更新，
   用于观察接收状态、最后一帧信息、CRC 校验结果和超时保护状态。 */
volatile uint8_t debug_app_esc_spi_initialized = 0;   /* ESC 解析器是否已初始化 */
volatile uint8_t debug_app_spi_slave_initialized = 0; /* SPI 从机是否已初始化 */
volatile uint32_t debug_esc_last_frame_ms = 0;       /* 上次有效 ESC 帧的时间戳 */
volatile uint16_t debug_esc_last_frame_len = 0;      /* 上次解析的 ESC 帧长度 */
volatile uint8_t debug_esc_last_cmd = 0;             /* 上次 ESC 帧的命令字 */
volatile uint8_t debug_esc_last_crc = 0;             /* 上次 ESC 帧的 CRC 字节 */
volatile uint8_t debug_esc_last_crc_ok = 0;          /* CRC 校验是否通过 */
volatile uint8_t debug_esc_last_frame_valid = 0;     /* 上一次是否解析出有效帧 */
volatile int16_t debug_esc_throttles[ESC_CHANNELS] = {0}; /* 解码后的油门值 */
volatile uint8_t debug_esc_timeout_active = 0;       /* 超时保护是否激活 */

volatile uint16_t debug_spi_frame_len = 0;           /* SPI 当前接收缓冲区字节数 */
volatile uint32_t debug_spi_frame_count = 0;         /* 已接收完整帧数量 */
volatile uint8_t debug_spi_last_byte = 0;            /* 最近接收到的 SPI 原始字节 */
volatile uint8_t debug_spi_frame_ready = 0;          /* 是否已拼接出完整帧 */
volatile uint8_t debug_spi_error = 0;                /* SPI 接收错误标志 */

/* TIM4 debug variables */
volatile uint8_t debug_tim4_test_active = 0;
volatile uint16_t debug_tim4_pulse = 0;

/* Set to 1 to run a brief PWM self-test at boot */
#define PWM_SELF_TEST 1

/* Simple test for TIM4 only - bypasses other code */
static void Test_TIM4_Only(void)
{
    debug_tim4_test_active = 1;

    /* Configure PB6-PB9 as GPIO push-pull first for testing */
    __HAL_RCC_GPIOB_CLK_ENABLE();
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* Toggle pins to test GPIO works */
    for (int i = 0; i < 10; i++) {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_RESET);
        HAL_Delay(100);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_SET);
        HAL_Delay(100);
    }

    /* Now reconfigure for AF_PP (PWM) */
    GPIO_InitStruct.Pin = GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* Ensure no remapping */
    __HAL_RCC_AFIO_CLK_ENABLE();
    __HAL_AFIO_REMAP_TIM4_DISABLE();

    /* Set compare values */
    debug_tim4_pulse = 1500;
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, debug_tim4_pulse);
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, debug_tim4_pulse);
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, debug_tim4_pulse);
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, debug_tim4_pulse);

    /* Start TIM4 counter */
    __HAL_TIM_ENABLE(&htim4);

    /* Start PWM channels */
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);

    /* Sweep test for TIM4 only */
    for (uint16_t pulse = 1000; pulse <= 2000; pulse += 50) {
        debug_tim4_pulse = pulse;
        __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, pulse);
        __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, pulse);
        __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, pulse);
        __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, pulse);
        HAL_Delay(200);
    }

    /* Back to mid */
    debug_tim4_pulse = 1500;
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, debug_tim4_pulse);
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, debug_tim4_pulse);
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, debug_tim4_pulse);
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, debug_tim4_pulse);
}

void System_Init(void)
{
    /* Initialize PC13 LED for debug */
    __HAL_RCC_GPIOC_CLK_ENABLE();
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = GPIO_PIN_13;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);

#if PWM_SELF_TEST
    /* First test TIM4 alone */
    Test_TIM4_Only();
#endif

    /* Start 8 PWM outputs (TIM1+TIM4) */
    ESC_PWM_Init();

    /* Initialize ESC SPI handling (throttles default to mid/zero) */
    ESC_SPI_Init();
    debug_app_esc_spi_initialized = 1;

    /* Initialize SPI slave */
    SPI_Slave_Init();
    debug_app_spi_slave_initialized = 1;

    /* Force one-time PWM update to ensure outputs are driven immediately */
    ESC_ApplyToPWM();

#if PWM_SELF_TEST
    /* Run PWM self-test before entering main loop */
    HAL_Delay(100);

    /* Fixed value test: 1500us (mid) -> 1000us (low) -> 2000us (high) -> back to 1500us */
    ESC_PWM_Test_Fixed(1500);
    HAL_Delay(1000);

    ESC_PWM_Test_Fixed(1000);
    HAL_Delay(1000);

    ESC_PWM_Test_Fixed(2000);
    HAL_Delay(1000);

    ESC_PWM_Test_Fixed(1500);
#endif /* PWM_SELF_TEST */
}

/********************************************************************************************************
Function Name: Mymain  自定义的main函数
Author       : ZFY
Date         : 2025-01-05
Description  :
Outputs      : void
Notes        :
*********************************************************************************************************/

void Mymain(void)
{
    System_Init();

    /* 主循环仅保持固件运行。
       实际控制逻辑由定时器中断和周期任务执行。 */
    for (;;)
    {
        /* 最小化主循环：兜底重启 SPI 接收 */
        SPI_Slave_Poll();

        /* Toggle LED to show we're alive */
        HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
        HAL_Delay(500);
    }
}

/********************************************************************************************************
Function Name: Millisecond_Task  毫秒级任务
Author       : ZFY
Date         : 2025-01-05
Description  :
Outputs      : void
Notes        :
*********************************************************************************************************/

void Millisecond_Task(void)
{
    /* DEBUG: disabled during SPI bring-up */
}

/********************************************************************************************************
FunctionName: Millisecond_50_Task  50毫秒级任务
Author       : ZFY
Date         : 2025-01-05
Description
Outputs      : void
Notes        :
*********************************************************************************************************/

int clawtime = 0;

void Millisecond_50_Task(void)
{
    /* DEBUG: disabled during SPI bring-up */
}

/********************************************************************************************************
Function Name: HAL_TIM_PeriodElapsedCallback  定时器回调函数
Author       : ZFY
Date         : 2024-01-05
Description
Outputs      : void
Notes        :
*********************************************************************************************************/

int pluse = 0;

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    /* DEBUG: disable periodic tasks during SPI bring-up */
    (void)htim;
    return;
}
