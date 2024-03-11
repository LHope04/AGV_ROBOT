
#include "exchange.h"
#include "ins_task.h"
extern INS_t INS;

ins_data_t ins_data;
#include "struct_typedef.h"
#include "exchange.h"
#include "ins_task.h"
#include "miniPC_process.h"

extern INS_t INS;
extern UART_HandleTypeDef huart1;

static Vision_Recv_s *vision_recv_data;

ins_data_t ins_data;

void exchangeangle_task(void const *pvParameters)
{
    Vision_Init_Config_s config = {
        .recv_config = {
            .header = VISION_RECV_HEADER,
        },
        .send_config = {
            .header        = VISION_SEND_HEADER,
            .detect_color  = VISION_DETECT_COLOR_BLUE,
            .reset_tracker = VISION_RESET_TRACKER_NO,
            .is_shoot      = VISION_SHOOTING,
            .tail          = VISION_SEND_TAIL,
        },
        .usart_config = {
            .recv_buff_size = VISION_RECV_SIZE,
            .usart_handle   = &huart6,
        },
    };
    vision_recv_data = VisionInit(&config);

    while (1) {
        ins_data.angle[0] = INS.Yaw;
        ins_data.angle[1] = INS.Roll;
        ins_data.angle[2] = INS.Pitch;
				ins_data.gyro[0]=INS.Gyro[2];
        VisionSetAltitude(INS.Yaw, INS.Pitch, INS.Roll);
        VisionSend();
				
//			int senddata  = 10000;
//			    HAL_UART_Transmit(&huart1,(uint8_t *)&senddata,sizeof(senddata),0xFFFF);
			// vision_instance

        osDelay(1);
    }
}
//void exchangeangle_task(void const *pvParameters)
//{
//	while(1){
//	ins_data.angle[0]=INS.Yaw;
//	ins_data.angle[1]=INS.Roll;
//	ins_data.angle[2]=INS.Pitch;

//	
//	osDelay(1);}
//}