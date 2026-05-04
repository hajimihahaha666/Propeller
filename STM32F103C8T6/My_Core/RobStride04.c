
#include "RobStride04.h"


FDCAN_FilterTypeDef R04_sFilterConfig;
FDCAN_TxHeaderTypeDef R04_TxHeader;
FDCAN_RxHeaderTypeDef R04_RxHeader;

void R04_filter(FDCAN_HandleTypeDef* fdcanhandle,uint32_t FilterBank_Num)
{
	
	R04_sFilterConfig.IdType=FDCAN_EXTENDED_ID;//扩展ID
	R04_sFilterConfig.FilterIndex=FilterBank_Num;//滤波器索引   
	R04_sFilterConfig.FilterType=FDCAN_FILTER_RANGE;      //滤波器类型
	R04_sFilterConfig.FilterConfig=FDCAN_FILTER_TO_RXFIFO0;//过滤器0关联到FIFO0       CAN1关联到了FIFO0 
	R04_sFilterConfig.FilterID1 =0x00000000; 
	R04_sFilterConfig.FilterID2 =0x00000000;
	
	R04_TxHeader.IdType=FDCAN_EXTENDED_ID;                  //扩展ID
	R04_TxHeader.TxFrameType=FDCAN_DATA_FRAME;              //数据帧
	R04_TxHeader.ErrorStateIndicator=FDCAN_ESI_ACTIVE;            
	R04_TxHeader.BitRateSwitch=FDCAN_BRS_OFF;               //关闭速率切换
	R04_TxHeader.FDFormat=FDCAN_CLASSIC_CAN;                //传统的CAN模式
	R04_TxHeader.TxEventFifoControl=FDCAN_NO_TX_EVENTS;     //无发送事件
	R04_TxHeader.MessageMarker=0;  

	 
	
	if(HAL_FDCAN_ConfigFilter(fdcanhandle,&R04_sFilterConfig)!=HAL_OK)//滤波器初始化
	HAL_FDCAN_ConfigGlobalFilter(fdcanhandle,FDCAN_REJECT, FDCAN_REJECT, DISABLE, DISABLE);//FDCAN_ACCEPT_IN_RX_FIFO0
	HAL_FDCAN_ActivateNotification(fdcanhandle,FDCAN_IT_RX_FIFO0_NEW_MESSAGE,0);
	HAL_FDCAN_Start(fdcanhandle);                               //开启FDCAN

}



uint8_t FDCAN1_Send_Msg(uint8_t* msg,uint32_t len,uint32_t CAN_ID)
{	
	R04_TxHeader.Identifier=CAN_ID;                           //32位ID
	R04_TxHeader.DataLength=len<<16;                            //数据长度
	if(HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1,&R04_TxHeader,msg)!=HAL_OK) {
		return 1;}//发送}
    return 0;	
}




/**
* @brief        浮点转整形
* @param        
* @ref          
* @author       ZFY
* @note         
**/
uint16_t Float2Uint(float val,float min,float max,uint8_t bits)
{

		float span = max - min;
		float offset = min;
		if(val > max) val=max;
		else if(val < min) val= min;
    return (int) ((val-offset)*((float)((1<<bits)-1))/span);
}

/**
* @brief        整形转浮点
* @param        
* @ref          
* @author       ZFY
* @note         
**/
float Uint2Float(uint16_t uint, float min,float max,uint8_t bits)
{
    float temp = 0.f;
    if(bits != 0)
        temp = ((float)uint)*(max-min)/((float)((1<<bits)-1)) + min;
    return temp;
}


/**
* @brief        关节电机控制
* @param        
* @ref          
* @author       ZFY
* @note         
**/
void Motor_Cmd(uint32_t motor_id, MotorCmdEnum cmd)
{
    uint8_t buff[8]={0};

    //发送控制命令
    switch(cmd)
    {
        case RS_ENABLE:    motor_id += (3<<24);   break;
        case RS_DISABLE:   motor_id += (4<<24);   break;
        case RS_ZERO:      motor_id += (6<<24);   buff[0]=1;   break;
        case RS_CLEAR:     motor_id += (4<<24);   buff[0]=1;   break;
    }
    FDCAN1_Send_Msg(buff,8,motor_id);
}



/**
* @brief        电机力位混合控制（MIT模式）
* @param        pos:    位置rad/弧度
*               vel：   速度rad/s
*               kp：    位置PD比例系数
*               kd：    位置PD微分系数
*               torque：力矩n.m           
* @ref          
* @author       Bling
* @note         
**/
void Motor_Ctrl(uint32_t motor_id,float torque, float MechPosition, float speed, float kp, float kd)
{    
		motor_id += (1<<24); //电机运控指令
    uint8_t buff[8];

    //数值转换     
    uint16_t position_16   = Float2Uint(MechPosition,P_MIN, P_MAX, 16);     //位置
    uint16_t velocity_16   = Float2Uint(speed, V_MIN, V_MAX, 16);     //速度
    uint16_t torque_16     = Float2Uint(torque, T_MIN, T_MAX, 16);     //力矩
    uint16_t Kp_16       = Float2Uint(kp, KP_MIN,KP_MAX, 16);               //位置KP比例系数
    uint16_t Kd_16       = Float2Uint(kd,KD_MIN,KD_MAX, 16);               //位置KD比例系数
	
	  motor_id+=(torque_16<<8); //设置电机扭矩
    buff[0] = (position_16>>8);
    buff[1] = (position_16);
    buff[2] = (velocity_16>>8);
    buff[3] = (velocity_16);
    buff[4] = (Kp_16>>8);
    buff[5] = (Kp_16);
	  buff[6] = (Kd_16>>8);
    buff[7] = (Kd_16);
    //发送控制命令
    FDCAN1_Send_Msg(buff,8,motor_id);
}



/**
* @brief        电机反馈数据处理
* @param        
* @ref          
* @author       ZFY
* @note         
**/




void Motor_DataTransform(Rs_Motor *motor,FDCAN_RxHeaderTypeDef *rx_header,uint8_t *rxData)
{
    static uint16_t position_16=32768;     //位置
    static uint16_t velocity_16=32768;     //速度
    static uint16_t torque_16=32768;     //力矩
	  static uint16_t temp=32768;     //力矩
	  if((rx_header->Identifier>>24)==0x02)   //判断是否为电机返回的数据帧
		{
		
				uint8_t id = (rx_header->Identifier>>8)& 0x000F;  //电机ID
				//电机故障代码：16/欠压 17/过流 18/过温 19/磁编码故障 20/堵转过载 21/未标定 22-23/工作模式
				uint16_t status = (rx_header->Identifier >>16)& 0x00FF;
	
				position_16= (rxData[0]<<8)|rxData[1];
        velocity_16= (rxData[2]<<8)|rxData[3];				
		    torque_16  = (rxData[4]<<8)|rxData[5];	
				temp  = (rxData[6]<<8)|rxData[7];	
				
				float posReal = Uint2Float(position_16,P_MIN,P_MAX,16);
				float velReal = Uint2Float(velocity_16,V_MIN,V_MAX,16);
				float torReal = Uint2Float(torque_16,T_MIN,T_MAX,16);
				float tempReal = temp/10.f;		
			
				motor->id=id;
				motor->position=posReal;
				motor->speed=motor->speed*0.5f+velReal*0.5f;  //滤波
				motor->torque=torReal;
				motor->status=status;
				motor->temp=tempReal;
		
		}
	
	

   
}






















