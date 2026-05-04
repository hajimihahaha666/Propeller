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
         ���汣��      ����BUG
*/
/*********************************************************************************************************/
#include "App.h"
#include "esc_spi.h"
#include "spi_slave.h"
#include "Pwm.h"
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

/* 如果 Pwm.c 没有加入 Keil 工程，提供一个 weak 版本避免链接错误。
   一旦 Pwm.c 被编译进来，它会覆盖这个 weak 实现。 */
__weak void Pwm_Init(void)
{
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);

	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);
}

/* Real implementations live in My_Core/esc_spi.c and My_Core/spi_slave.c */
/********************************************************************************************************
Function Name: System_Init  
Author       : ZFY
Date         : 2025-01-05
Description  :
Outputs      : void
Notes        : 
********************************************************************************************************/


		
void System_Init()
{
	
//	HAL_Delay(5000);//�ȴ�5S��ϵͳ����
//	
	
	/*
	 * DEBUG MINIMAL BOOT:
	 * 为了排查“启动奇怪/模块阻碍 SPI”，这里临时关闭所有与 SPI 无关的模块。
	 * 等 SPI 正常后，再逐个恢复这些模块。
	 */
	// HAL_TIM_Base_Start_IT(&htim4); // 1ms 中断（隔离阶段先关闭）

	/* System_Init 负责应用层启动：
	   - 启动 1ms 定时器中断，用于循环控制
	   - 初始化步进电机、舵机、IBUS 等子系统
	   - 启动 8 路 ESC PWM 输出
	   - 初始化 ESC SPI 解析器和 SPI 从机 */

	// Step_Motor_Init();// 步进电机（隔离阶段关闭）
	// Step_Pwm_Fre(5000);  // 步进 PWM 频率（隔离阶段关闭）
	// Servo_Init();// 舵机（隔离阶段关闭）
	// Ibus_Init();// IBUS（隔离阶段关闭）

	/* Start PWM outputs for ESC channels (隔离阶段关闭) */
	// HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	// HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
	// HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
	// HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
	// HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
	// HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
	// HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
	// HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
	// HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
	// HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);

	/* Start 8 PWM outputs (TIM1+TIM2) */
	Pwm_Init();

	/* Initialize ESC SPI handling (throttles default to mid/zero) */
	ESC_SPI_Init();
	debug_app_esc_spi_initialized = 1;
	/* Initialize SPI slave to receive frames from Raspberry Pi */
	SPI_Slave_Init();
	debug_app_spi_slave_initialized = 1;
	

	
	
	
}





/********************************************************************************************************
Function Name: Mymain  �Զ����main����
Author       : ZFY
Date         : 2023-06-09
Description  :
Outputs      : void
Notes        : 
********************************************************************************************************/
									
			
											
void Mymain()
{
	
	System_Init();

	 /* 主循环仅保持固件运行。
		 实际控制逻辑由定时器中断和周期任务执行。 */

			for(;;)
			{
				/* 最小化主循环：兜底重启 SPI 接收 */
				SPI_Slave_Poll();
			}

	
	
}




/********************************************************************************************************
Function Name: Millisecond_Task  ���뼶����
Author       : ZFY
Date         : 2025-01-05
Description  :
Outputs      : void
Notes        : 
********************************************************************************************************/


void Millisecond_Task()      //1msһ������
{
	/* DEBUG: disabled during SPI bring-up */
}

/********************************************************************************************************
Function Name: Millisecond_10_Task  10���뼶����
Author       : ZFY
Date         : 2025-01-05
Description  :
Outputs      : void
Notes        : 
********************************************************************************************************/

int clawtime=0;

void Millisecond_50_Task()      //50msһ������
{
	/* DEBUG: disabled during SPI bring-up */
}

/********************************************************************************************************
Function Name: HAL_TIM_PeriodElapsedCallback  ��ʱ���ص�����
Author       : ZFY
Date         : 2024-01-05
Description  :
Outputs      : void
Notes        :
********************************************************************************************************/

int pluse=0;

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
 /* DEBUG: disable periodic tasks during SPI bring-up */
 (void)htim;
 return;

 /* NOTE:
  * 原工程在这里会处理 1ms 时基并调用 Millisecond_50_Task 等。
  * bring-up 阶段我们直接 return，避免其它模块打断 SPI 调试。
  * 需要恢复时，请把原逻辑搬回并确保括号匹配。
  */

}















