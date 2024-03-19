#include "user_can.h"
#include "string.h"
#include "yaw_task.h"
extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

CAN_TxHeaderTypeDef can_tx_message;
uint8_t can_send_data[8];
motor_info motor[8];
up_data UpData;
int Up_ins_yaw = 0;

//void CAN1_Init()
//{
//	CAN_FilterTypeDef  can_filter;

//  can_filter.FilterBank = 0;
//  can_filter.FilterMode =  CAN_FILTERMODE_IDMASK;
//  can_filter.FilterScale = CAN_FILTERSCALE_32BIT;
//  can_filter.FilterIdHigh = 0; 
//  can_filter.FilterIdLow  = 0; 
//  can_filter.FilterMaskIdHigh = 0;
//  can_filter.FilterMaskIdLow  = 0;
//  can_filter.FilterFIFOAssignment = CAN_RX_FIFO1;
//  can_filter.FilterActivation = ENABLE;
//  can_filter.SlaveStartFilterBank  = 14;          
//   
//  HAL_CAN_ConfigFilter(&hcan1, &can_filter);
//  HAL_CAN_ActivateNotification(&hcan1,CAN_IT_RX_FIFO1_MSG_PENDING);
//  HAL_CAN_Start(&hcan1);
//}

void CAN1_Init() //CAN1过滤器配置
{
	CAN_FilterTypeDef  can_filter;

  can_filter.FilterBank = 0;
  can_filter.FilterMode =  CAN_FILTERMODE_IDMASK;
  can_filter.FilterScale = CAN_FILTERSCALE_32BIT;
  can_filter.FilterIdHigh = 0; 
  can_filter.FilterIdLow  = 0; 
  can_filter.FilterMaskIdHigh = 0;
  can_filter.FilterMaskIdLow  = 0;
  can_filter.FilterFIFOAssignment = CAN_RX_FIFO0;
  can_filter.FilterActivation = ENABLE;
  can_filter.SlaveStartFilterBank  = 14;          
   
  HAL_CAN_ConfigFilter(&hcan1, &can_filter);
  HAL_CAN_ActivateNotification(&hcan1,CAN_IT_RX_FIFO0_MSG_PENDING);
  HAL_CAN_Start(&hcan1);
}

void CAN2_Init() //CAN2过滤器配置
{
	CAN_FilterTypeDef  can_filter;

  can_filter.FilterBank = 14;
  can_filter.FilterMode =  CAN_FILTERMODE_IDMASK;
  can_filter.FilterScale = CAN_FILTERSCALE_32BIT;
  can_filter.FilterIdHigh = 0; 
  can_filter.FilterIdLow  = 0; 
  can_filter.FilterMaskIdHigh = 0;
  can_filter.FilterMaskIdLow  = 0;
  can_filter.FilterFIFOAssignment = CAN_RX_FIFO0;
  can_filter.FilterActivation = ENABLE;
  can_filter.SlaveStartFilterBank  = 14;          
   
  HAL_CAN_ConfigFilter(&hcan2, &can_filter);
  HAL_CAN_ActivateNotification(&hcan2,CAN_IT_RX_FIFO0_MSG_PENDING);
  HAL_CAN_Start(&hcan2);
}


/****************************************************CAN1发送函数***************************************************/
void can_remote(uint8_t sbus_buf[],uint8_t can_send_id)
{
	CAN_TxHeaderTypeDef can_remote_message;
	uint32_t remote_mail_box = (uint32_t)CAN_TX_MAILBOX1; //邮箱2
	can_remote_message.StdId = can_send_id;
	can_remote_message.IDE = CAN_ID_STD;
	can_remote_message.RTR = CAN_RTR_DATA;
	can_remote_message.DLC = 0x08;
	
	HAL_CAN_AddTxMessage(&hcan1,&can_remote_message,sbus_buf,&remote_mail_box);
}
/*******************************************************************************************************************/


/****************************************************CAN2发送函数***************************************************/
void can_cmd_send_3508(int motor1,int motor2,int motor3,int motor4) //can2发送 控制3508四个电机（motor[]:0-3）
{
	uint32_t send_mail_box = (uint32_t)CAN_TX_MAILBOX0;
	can_tx_message.StdId = 0x200;
	can_tx_message.IDE = CAN_ID_STD;
	can_tx_message.RTR = CAN_RTR_DATA;
	can_tx_message.DLC = 0x08;
	can_send_data[0] = (motor1>>8)&0xff;
	can_send_data[1] = motor1&0xff;
	can_send_data[2] = (motor2>>8)&0xff;
	can_send_data[3] = motor2&0xff;
	can_send_data[4] = (motor3>>8)&0xff;
	can_send_data[5] = motor3&0xff;
	can_send_data[6] = (motor4>>8)&0xff;
	can_send_data[7] = motor4&0xff;

	HAL_CAN_AddTxMessage(&hcan2,&can_tx_message,can_send_data,&send_mail_box);
}

void can_cmd_send_6020(int motor1,int motor2,int motor3,int motor4) //can2发送 控制6020四个电机(motor[]:4-7)
{
	uint32_t send_mail_box = (uint32_t)CAN_TX_MAILBOX0;
	can_tx_message.StdId = 0x1FF;
	can_tx_message.IDE = CAN_ID_STD;
	can_tx_message.RTR = CAN_RTR_DATA;
	can_tx_message.DLC = 0x08;
	can_send_data[0] = (motor1>>8)&0xff;
	can_send_data[1] = motor1&0xff;
	can_send_data[2] = (motor2>>8)&0xff;
	can_send_data[3] = motor2&0xff;
	can_send_data[4] = (motor3>>8)&0xff;
	can_send_data[5] = motor3&0xff;
	can_send_data[6] = (motor4>>8)&0xff;
	can_send_data[7] = motor4&0xff;

	HAL_CAN_AddTxMessage(&hcan2,&can_tx_message,can_send_data,&send_mail_box);
}
void can_cmd_send_6020_2(int motor1,int motor2,int motor3,int motor4) //can2发送 控制6020四个电机(motor[]:4-7)
{
	uint32_t send_mail_box = (uint32_t)CAN_TX_MAILBOX0;
	can_tx_message.StdId = 0x2FF;
	can_tx_message.IDE = CAN_ID_STD;
	can_tx_message.RTR = CAN_RTR_DATA;
	can_tx_message.DLC = 0x08;
	can_send_data[0] = (motor1>>8)&0xff;
	can_send_data[1] = motor1&0xff;
	can_send_data[2] = (motor2>>8)&0xff;
	can_send_data[3] = motor2&0xff;
	can_send_data[4] = (motor3>>8)&0xff;
	can_send_data[5] = motor3&0xff;
	can_send_data[6] = (motor4>>8)&0xff;
	can_send_data[7] = motor4&0xff;

	HAL_CAN_AddTxMessage(&hcan1,&can_tx_message,can_send_data,&send_mail_box);
}
/*******************************************************************************************************************/


/**************************************************尝试双can双fifo**************************************************/
//void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan) //can1发送3508数据 FIFO1
//{
//	if(hcan->Instance == CAN1){ 
//		CAN_RxHeaderTypeDef can_rx_message;
//		uint8_t can_receive_data[8];
//	  HAL_CAN_GetRxMessage(hcan,CAN_RX_FIFO1,&can_rx_message,can_receive_data);
//	  if((can_rx_message.StdId >= 0x201) && (can_rx_message.StdId <= 0x204)){
//			uint8_t index = can_rx_message.StdId - 0x201 + 4;
//			motor[index].angle = ((can_receive_data[0] << 8) | can_receive_data[1]);
//			motor[index].speed = ((can_receive_data[2] << 8) | can_receive_data[3]);
//			motor[index].tor_current = ((can_receive_data[4] << 8) | can_receive_data[5]);
//			motor[index].temperture = can_receive_data[6];
//		}
//	}
//}
/*******************************************************************************************************************/
extern uint16_t yaw_code_val;

/*********************************************CAN通信接收回调函数 fifo0*********************************************/
int error = 0;
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	if(hcan->Instance == CAN1) //接收上C板数据
	{
		CAN_RxHeaderTypeDef can_rx_message;
		uint8_t can_recevie_data[8];
		HAL_CAN_GetRxMessage(hcan,CAN_RX_FIFO0,&can_rx_message,can_recevie_data);
		if(can_rx_message.StdId == 0x47)
		{
			memcpy((void*)(&yaw_code_val),(const void*)(&can_recevie_data[0]),2);
		}
		if(can_rx_message.StdId == 0x404){
			memcpy(&UpData.yaw_up,&can_recevie_data,4); //接收上C板yaw(float)
		}
				if(can_rx_message.StdId==0x55)//上C向下C传IMU数据
		{
				if(can_recevie_data[0] == 8)	//校验位
				{
					Up_ins_yaw = can_recevie_data[1] | (can_recevie_data[2] << 8);			
					//memcpy(&Up_ins_yaw,&rx_data[1],4);
					//memcpy(&frame_id,&rx_data[5],1);
				}
		}
				if ((can_rx_message.StdId >= 0x209)//201-207
   && (can_rx_message.StdId <  0x211))                  // 判断标识符，标识符为0x200+ID
  {
    uint8_t index = can_rx_message.StdId - 0x209;                  // get motor index by can_id

      motor_gimbal[index].bmz    = ((can_recevie_data[0] << 8) | can_recevie_data[1]);
      motor_gimbal[index].speed    = ((can_recevie_data[2] << 8) | can_recevie_data[3]);
      motor_gimbal[index].torque_current = ((can_recevie_data[4] << 8) | can_recevie_data[5]);
      motor_gimbal[index].temp           =   can_recevie_data[6];

     

  }
		  if((can_rx_message.StdId >= 0x201) && (can_rx_message.StdId <= 0x208)){
			uint8_t index = can_rx_message.StdId - 0x201;
			motor[index].angle = ((can_recevie_data[0] << 8) | can_recevie_data[1]);
			motor[index].speed = ((can_recevie_data[2] << 8) | can_recevie_data[3]);
			motor[index].tor_current = ((can_recevie_data[4] << 8) | can_recevie_data[5]);
			motor[index].temperture = can_recevie_data[6];
		}
	}
	
	if(hcan->Instance == CAN2) //can2发送6020、3508数据 FIFO0
	{
		CAN_RxHeaderTypeDef can_rx_message;
		uint8_t can_receive_data[8];
	  HAL_CAN_GetRxMessage(hcan,CAN_RX_FIFO0,&can_rx_message,can_receive_data);
	  if((can_rx_message.StdId >= 0x201) && (can_rx_message.StdId <= 0x208)){
			uint8_t index = can_rx_message.StdId - 0x201;
			motor[index].angle = ((can_receive_data[0] << 8) | can_receive_data[1]);
			motor[index].speed = ((can_receive_data[2] << 8) | can_receive_data[3]);
			motor[index].tor_current = ((can_receive_data[4] << 8) | can_receive_data[5]);
			motor[index].temperture = can_receive_data[6];
		}
						if(can_rx_message.StdId==0x55)//上C向下C传IMU数据
		{
				if(can_receive_data[0] == 8)	//校验位
				{
					Up_ins_yaw = can_receive_data[1] | (can_receive_data[2] << 8);			
					//memcpy(&Up_ins_yaw,&rx_data[1],4);
					//memcpy(&frame_id,&rx_data[5],1);
				}
		}
	}
}
/*******************************************************************************************************************/
