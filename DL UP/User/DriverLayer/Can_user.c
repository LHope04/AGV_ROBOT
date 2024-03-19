#include "Can_user.h"
#include "remote_control.h"

//	Can ��һЩ�û�׫д�Ľ��պ���
extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;
extern int16_t Rotate_w;
extern int16_t Down_pitch;
extern int8_t Update_yaw_flag;
extern fp32 init_yaw;
int16_t yaw_s,pitch_s;
int8_t errt;
int8_t shoot = 0;
float yaw_ss,pitch_ss;
extern uint16_t muzzle_heat;
extern uint16_t muzzle_heat_limit;
void can_1_user_init(CAN_HandleTypeDef* hcan )
{
  CAN_FilterTypeDef  can_filter;

  can_filter.FilterBank = 0;                       // filter 0
  can_filter.FilterMode =  CAN_FILTERMODE_IDMASK;  // ��ʶ������λģʽ
  can_filter.FilterScale = CAN_FILTERSCALE_32BIT;		//������λ��Ϊ����32λ
  can_filter.FilterIdHigh = 0;//��ʶ���Ĵ��� 
  can_filter.FilterIdLow  = 0;//��ʶ���Ĵ��� 
  can_filter.FilterMaskIdHigh = 0;//���μĴ���
  can_filter.FilterMaskIdLow  = 0;       //���μĴ���   set mask 0 to receive all can id
  can_filter.FilterFIFOAssignment = CAN_RX_FIFO0; // assign to fifo0����������FIFO0
  can_filter.FilterActivation = ENABLE;           // enable can filter
  can_filter.SlaveStartFilterBank  = 14;          // only meaningful in dual can mode
   
  HAL_CAN_ConfigFilter(hcan, &can_filter);        // init can filter
	HAL_CAN_Start(&hcan1);//����can����װ��can_user_init()����
	HAL_CAN_ActivateNotification(&hcan1,CAN_IT_RX_FIFO0_MSG_PENDING);		//ʹ��can��FIFO0�жϣ�Ҳ��װ��can_user_init()����
}

void can_2_user_init(CAN_HandleTypeDef* hcan )
{
  CAN_FilterTypeDef  can_filter;

  can_filter.FilterBank = 14;                       // filter 14
  can_filter.FilterMode =  CAN_FILTERMODE_IDMASK;  // ��ʶ������λģʽ
  can_filter.FilterScale = CAN_FILTERSCALE_32BIT;		//������λ��Ϊ����32λ
  can_filter.FilterIdHigh = 0;//��ʶ���Ĵ��� 
  can_filter.FilterIdLow  = 0;//��ʶ���Ĵ��� 
  can_filter.FilterMaskIdHigh = 0;//���μĴ���
  can_filter.FilterMaskIdLow  = 0;       //���μĴ���   set mask 0 to receive all can id
  can_filter.FilterFIFOAssignment = CAN_RX_FIFO0; // assign to fifo0����������FIFO0
  can_filter.FilterActivation = ENABLE;           // enable can filter
  can_filter.SlaveStartFilterBank  = 14;          // only meaningful in dual can mode
   
  HAL_CAN_ConfigFilter(hcan, &can_filter);        // init can filter
	HAL_CAN_Start(&hcan2);//����can����װ��can_user_init()����
	HAL_CAN_ActivateNotification(&hcan2,CAN_IT_RX_FIFO0_MSG_PENDING);		//ʹ��can��FIFO0�жϣ�Ҳ��װ��can_user_init()����
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)//�����жϻص�����
{
  CAN_RxHeaderTypeDef rx_header;
  if(hcan->Instance == CAN1)
  {
		HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data); //receive can1 data
		
		//YAWУ������
		if(rx_header.StdId==0x66)
		{
			if(rx_data[0] == 0xff)
			{
				Update_yaw_flag = 1;
				HAL_GPIO_TogglePin(GPIOH,GPIO_PIN_12);
				init_yaw = 0;//��������̨��ֵ
			}
		}
		
	//	HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data); //receive can1 data
		if ((rx_header.StdId >= 0x201)//201-207
   && (rx_header.StdId <  0x207))                  // �жϱ�ʶ������ʶ��Ϊ0x200+ID
  {
    uint8_t index = rx_header.StdId - 0x201;                  // get motor index by can_id
    motor_info[index].rotor_angle    = ((rx_data[0] << 8) | rx_data[1]);
    motor_info[index].rotor_speed    = ((rx_data[2] << 8) | rx_data[3]);
    motor_info[index].torque_current = ((rx_data[4] << 8) | rx_data[5]);
    motor_info[index].temp           =   rx_data[6];
		if(index==0)
		{can_cnt_1 ++;}
  }
	
	//YAW
	  if(rx_header.StdId == 0x209)
	{
		motor_info[5].rotor_angle    = ((rx_data[0] << 8) | rx_data[1]);
    motor_info[5].rotor_speed    = ((rx_data[2] << 8) | rx_data[3]);
    motor_info[5].torque_current = ((rx_data[4] << 8) | rx_data[5]);
    motor_info[5].temp           =   rx_data[6];
	}
	  if(rx_header.StdId == 0x210)
	{
		motor_info[6].rotor_angle    = ((rx_data[0] << 8) | rx_data[1]);
    motor_info[6].rotor_speed    = ((rx_data[2] << 8) | rx_data[3]);
    motor_info[6].torque_current = ((rx_data[4] << 8) | rx_data[5]);
    motor_info[6].temp           =   rx_data[6];
	}
	
  }
	 if(hcan->Instance == CAN2)
  {		uint8_t             rx_data[8];
    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data); //receive can2 data
		if ((rx_header.StdId >= FEEDBACK_ID_BASE)//201-207
   && (rx_header.StdId <  FEEDBACK_ID_BASE + MOTOR_MAX_NUM))                  // �жϱ�ʶ������ʶ��Ϊ0x200+ID
  {
    uint8_t index = rx_header.StdId - FEEDBACK_ID_BASE;                  // get motor index by can_id
    motor_info_can_2[index].rotor_angle    = ((rx_data[0] << 8) | rx_data[1]);
    motor_info_can_2[index].rotor_speed    = ((rx_data[2] << 8) | rx_data[3]);
    motor_info_can_2[index].torque_current = ((rx_data[4] << 8) | rx_data[5]);
    motor_info_can_2[index].temp           =   rx_data[6];

  }
		  if(rx_header.StdId == 0x210)
	{
		motor_info[6].rotor_angle    = ((rx_data[0] << 8) | rx_data[1]);
    motor_info[6].rotor_speed    = ((rx_data[2] << 8) | rx_data[3]);
    motor_info[6].torque_current = ((rx_data[4] << 8) | rx_data[5]);
    motor_info[6].temp           =   rx_data[6];
	}
	  if ((rx_header.StdId >= FEEDBACK_ID_BASE_6020)//205-211,ע���ID���ɴ���3,��Ȼ�ͻ�Ͷ�ȡ3508�ĺ���������ͻ
   && (rx_header.StdId <  FEEDBACK_ID_BASE_6020 + MOTOR_MAX_NUM))                  // �жϱ�ʶ������ʶ��Ϊ0x204+ID
  {
    uint8_t index = rx_header.StdId - FEEDBACK_ID_BASE_6020;                  // get motor index by can_id
    motor_info_can_2[index].rotor_angle    = ((rx_data[0] << 8) | rx_data[1]);
    motor_info_can_2[index].rotor_speed    = ((rx_data[2] << 8) | rx_data[3]);
    motor_info_can_2[index].torque_current = ((rx_data[4] << 8) | rx_data[5]);
    motor_info_can_2[index].temp           =   rx_data[6];

  }
	if(rx_header.StdId==0x33)//˫C�崫��ң�����źŵĽӿڱ�ʶ��
		{
			can_cnt_2++;
			if (can_cnt_2 == 100)//��˸���ƴ���ң�ؽ�������ͨ��
			{
				can_cnt_2 = 0;
				HAL_GPIO_TogglePin(GPIOH,GPIO_PIN_10);
			}
			rc_ctrl.rc.ch[0] = (rx_data[0] | (rx_data[1] << 8)) & 0x07ff;        //!< Channel 0  ��ֵΪ1024�����ֵ1684����Сֵ364��������Χ��660
			rc_ctrl.rc.ch[1] = (((rx_data[1] >> 3)&0xff) | (rx_data[2] << 5)) & 0x07ff; //!< Channel 1
			rc_ctrl.rc.ch[2] = (((rx_data[2] >> 6)&0xff) | (rx_data[3] << 2) |          //!< Channel 2
                         (rx_data[4] << 10)) &0x07ff;
			rc_ctrl.rc.ch[3] = (((rx_data[4] >> 1)&0xff) | (rx_data[5] << 7)) & 0x07ff; //!< Channel 3
			rc_ctrl.rc.s[0] = ((rx_data[5] >> 4) & 0x0003);                  //!< Switch left����������������
			rc_ctrl.rc.s[1] = ((rx_data[5] >> 4) & 0x000C) >> 2;    		//!< Switch right�������������
			rc_ctrl.mouse.x = rx_data[6] | (rx_data[7] << 8);                    //!< Mouse X axis
			}
		if(rx_header.StdId==0x34)//˫C�崫��ң�����źŵĽӿڱ�ʶ��
		{
			rc_ctrl.mouse.y = rx_data[0] | (rx_data[1] << 8);                    //!< Mouse Y axis
			rc_ctrl.mouse.z = rx_data[2] | (rx_data[3] << 8);                  //!< Mouse Z axis
			rc_ctrl.mouse.press_l = rx_data[4];                                  //!< Mouse Left Is Press ?
			rc_ctrl.mouse.press_r = rx_data[5];                                  //!< Mouse Right Is Press ?
			rc_ctrl.key.v = rx_data[6] | (rx_data[7] << 8); 
/*			//!< KeyBoard value	
    rc_ctrl.rc.ch[4] = rx_data[16] | (rx_data[17] << 8);                 //NULL
			*/	}
		if(rx_header.StdId==0x35)//˫C�崫��ң�����źŵĽӿڱ�ʶ��
		{
			
               //NULL
			memcpy(&pitch_s,&rx_data[0],2);
			memcpy(&yaw_s,&rx_data[2],2);
			yaw_ss = (float)yaw_s / 100;
			pitch_ss = (float)pitch_s / 100;
		//���յ�����ת��

			
		//���յ���imu��Pitch����
			errt = (int8_t) rx_data[4];
			shoot =  (int8_t) rx_data[5];
			//memcpy((void*)(&muzzle_heat),(const void*)(&rx_data[6]),2);
		}

    if(rx_header.StdId==0x36)
    {
      memcpy((void*)(&muzzle_heat),(const void*)(&rx_data[0]),2);
      memcpy((void*)(&muzzle_heat_limit),(const void*)(&rx_data[2]),2);
    }
		
  }

  if (can_cnt_1 == 500)//��˸��ƴ���can����ͨ��
  {
    can_cnt_1 = 0;
		HAL_GPIO_TogglePin(GPIOH,GPIO_PIN_12);

  }
}

void can_remote(uint8_t sbus_buf[],uint32_t id)//����can������ң��������
{
  CAN_TxHeaderTypeDef tx_header;  
  tx_header.StdId = id;
  tx_header.IDE   = CAN_ID_STD;//��׼֡
  tx_header.RTR   = CAN_RTR_DATA;//����֡
  tx_header.DLC   = 8;		//�������ݳ��ȣ��ֽڣ�

  HAL_CAN_AddTxMessage(&hcan2, &tx_header, sbus_buf,(uint32_t*)CAN_TX_MAILBOX0);
}
