#ifndef SBUS_H
#define SBUS_H





#include "main.h"


#define  SBUS_SIZE  (50)
#define SBUS_CH_NUM 16

extern DMA_HandleTypeDef hdma_uart7_rx;
void Sbus_Init(void);
void USAR_UART7_IDLECallback(void);
//extern uint16_t Sbus_CH[CH_NUM];  // 遥控器通道值      针对8通道的遥控器

extern float __DTCM Sbus_Value[SBUS_CH_NUM];

#endif //SBUS_H
