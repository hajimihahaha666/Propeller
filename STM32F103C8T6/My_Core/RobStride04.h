#ifndef ROBSTRIDE04_H
#define ROBSTRIDE04_H

#include "main.h"
//extern FDCAN_FilterTypeDef R04_sFilterConfig;
//extern FDCAN_TxHeaderTypeDef R04_TxHeader;
//extern FDCAN_RxHeaderTypeDef R04_RxHeader;





#define P_MIN -12.57f //0.4.0.5及之前为12.5，之后为12.57
#define P_MAX 12.57f //0.4.0.5及之前为12.5，之后为12.57
#define V_MIN -15.0f
#define V_MAX 15.0f
#define KP_MIN 0.0f
#define KP_MAX 5000.0f
#define KD_MIN 0.0f
#define KD_MAX 100.0f
#define T_MIN -120.0f
#define T_MAX 120.0f


/**
* @brief    电机命令枚举
**/
typedef enum 
{
    RS_ENABLE = 0,     //使能
    RS_DISABLE,        //失能
    RS_CLEAR,          //清除错误
    RS_ZERO,           //保存为零点
}MotorCmdEnum;



/*Functions------------------------------------------------------------------*/
uint8_t FDCAN1_Send_Msg(uint8_t* msg,uint32_t len,uint32_t CAN_ID);
void R04_filter(FDCAN_HandleTypeDef* fdcanhandle,uint32_t FilterBank_Num);

void Motor_Cmd(uint32_t motor_id, MotorCmdEnum cmd);
void Motor_Ctrl(uint32_t motor_id,float torque, float MechPosition, float speed, float kp, float kd); //设置电机


typedef struct 
{
	uint8_t id;
	float position,speed,torque,temp;
	uint16_t status;
}Rs_Motor;   //定义TMOTOR电机结构体


void Motor_DataTransform(Rs_Motor *motor,FDCAN_RxHeaderTypeDef *rx_header,uint8_t *rxData);







#endif //ROBSTRIDE04_H
