#include "miniPC_process.h"
#include "string.h"

float vision_yaw;
float vision_pitch;

static Vision_Instance *vision_instance; // ????????????

/**
 * @brief ?????????
 *
 * @param recv
 * @param rx_buff
 */
static void RecvProcess(Vision_Recv_s *recv, uint8_t *rx_buff)
{
    /* ??memcpy??????? */
    memcpy(&recv->yaw, &rx_buff[1], 4);
    memcpy(&recv->pitch, &rx_buff[5], 4);

    /* ????? */
    memcpy(&recv->checksum, &rx_buff[9], 2);
}

/**
 * @brief ????,?????????????
 *
 */
static void DecodeVision(void)
{
    if (vision_instance->usart->recv_buff[0] == vision_instance->recv_data->header)
    {
        // ??????
        RecvProcess(vision_instance->recv_data, vision_instance->usart->recv_buff);

        vision_yaw = vision_instance->recv_data->yaw;
        vision_pitch = vision_instance->recv_data->pitch;
    }
    else
    {
    }
}

/**
 * @brief ???????????????,???????????????
 *
 * @param recv_config
 * @return Vision_Recv_s*
 */
Vision_Recv_s *VisionRecvRegister(Vision_Recv_Init_Config_s *recv_config)
{
    Vision_Recv_s *recv_data = (Vision_Recv_s *)malloc(sizeof(Vision_Recv_s));
    memset(recv_data, 0, sizeof(Vision_Recv_s));

    recv_data->header = recv_config->header;

    return recv_data;
}

/**
 * @brief ???????????????,???????????????
 *
 * @param send_config
 * @return Vision_Send_s*
 */
Vision_Send_s *VisionSendRegister(Vision_Send_Init_Config_s *send_config)
{
    Vision_Send_s *send_data = (Vision_Send_s *)malloc(sizeof(Vision_Send_s));
    memset(send_data, 0, sizeof(Vision_Send_s));

    send_data->header = send_config->header;
    send_data->detect_color = send_config->detect_color;
    send_data->reset_tracker = send_config->reset_tracker;
    send_data->is_shoot = send_config->is_shoot;
    send_data->tail = send_config->tail;
    return send_data;
}

/**
 * @brief ??????????????,???????????????
 *
 * @param init_config
 * @return Vision_Recv_s*
 */
Vision_Recv_s *VisionInit(Vision_Init_Config_s *init_config)
{
    vision_instance = (Vision_Instance *)malloc(sizeof(Vision_Instance));
    memset(vision_instance, 0, sizeof(Vision_Instance));

    init_config->usart_config.module_callback = DecodeVision;

    vision_instance->usart = USARTRegister(&init_config->usart_config);
    vision_instance->recv_data = VisionRecvRegister(&init_config->recv_config);
    vision_instance->send_data = VisionSendRegister(&init_config->send_config);
    return vision_instance->recv_data;
}

/**
 * @brief ????????
 *
 * @param send ?????
 * @param tx_buff ?????
 *
 */
#include "CRC.h"
extern uint16_t CRC_INIT ;
//const uint16_t wCRC_Table[256];
uint16_t checknm = 0;
static void SendProcess(Vision_Send_s *send, uint8_t *tx_buff)
{
    /* ????,????,??????? */
    tx_buff[0] = send->header;
    tx_buff[1] = send->detect_color;
    tx_buff[2] = send->reset_tracker;
    tx_buff[3] = send->is_shoot;

    /* ??memcpy??????? */
    memcpy(&tx_buff[4], &send->roll, 4);
    memcpy(&tx_buff[8], &send->yaw, 4);
    memcpy(&tx_buff[12], &send->pitch, 4);

    /* ????? */
		send->checksum = Get_CRC16_Check_Sum(tx_buff,16,CRC_INIT);
	checknm =send->checksum;
    memcpy(&tx_buff[16], &send->checksum, 2);
    memcpy(&tx_buff[18], &send->tail, 1);
	
}


/**
 * @brief ????
 *
 * @param send ?????
 *
 */
void VisionSend()
{
    static uint8_t send_buff[VISION_SEND_SIZE];
    SendProcess(vision_instance->send_data, send_buff);
	  HAL_UART_Transmit(&huart1,(uint8_t *)&send_buff,sizeof(send_buff),100);
	CDC_Transmit_FS((uint8_t *)&send_buff, sizeof(send_buff));
    //USARTSend(vision_instance->usart, send_buff, VISION_SEND_SIZE, USART_TRANSFER_BLOCKING);
}

/**
 * @brief ????????IMU??
 *
 * @param yaw
 * @param pitch
 * @param roll
 */
void VisionSetAltitude(float yaw, float pitch, float roll)
{
    vision_instance->send_data->yaw = yaw;
    vision_instance->send_data->pitch = pitch;
    vision_instance->send_data->roll = roll;
}