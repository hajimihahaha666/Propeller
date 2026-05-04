#include "UART.h"



/********************************************************************************************************
Function Name: USER_UART_IRQHandler 
Author       : ZFY
Date         : 2025-10-10
Description  :
Outputs      : void
Notes        : 空闲中断
********************************************************************************************************/
void USER_UART_IRQHandler(UART_HandleTypeDef *huart)
{
 if(huart->Instance==USART1)
		{
        if(RESET != __HAL_UART_GET_FLAG(&huart1, UART_FLAG_IDLE))   //判断是否是空闲中断
        {
            __HAL_UART_CLEAR_IDLEFLAG(&huart1);                     //清楚空闲中断标志（否则会一直不断进入中断）
            USAR_UART1_IDLECallback();                              //调用中断处理函数    Ibus
        }				
		
		}
		
		
		
		
		
		
}

























