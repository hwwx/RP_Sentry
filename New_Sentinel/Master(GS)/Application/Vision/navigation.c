#include "navigation.h"
#include "vision.h"
#include "bmi.h"
#include "can_protocol.h"
#include "rp_shoot.h"
#include "rp_gimbal.h"
#include "judge.h"
#include "Car.h"

/*ʹ�ô���1*/
extern UART_HandleTypeDef huart1;//ʹ�ô���1

extern judge_t          judge;
extern Master_Head_t    Master_Head_structure;
extern bmi_t            bmi_structure;
extern car_t            car_structure;
extern vision_t         vision_structure;
navigation_rx_packet_t  navigation_rx_pack;
navigation_tx_packet_t  navigation_tx_pack;
navigation_t            navigation_structure;


/*����Buffer*/
uint8_t                 navigation_txBuf[80]; 


/**
  * @Name    Vision_Init
  * @brief   �Ӿ����ֽṹ���ʼ��
  * @param   None
  * @retval  
  * @author  HWX
  * @Date    2022-10-21
**/
void Navigation_Init(void)
{
	/*�ṹ���ʼ��*/
	navigation_structure.rx_pack = &navigation_rx_pack;
	navigation_structure.tx_pack = &navigation_tx_pack;
	
	/*���ݳ�ʼ��*/
	navigation_structure.tx_pack->FrameHeader.sof   = VISION_SEND_ID;
	navigation_structure.tx_pack->FrameHeader.cmd_id= CMD_PATROL;
	navigation_structure.tx_pack->TxData.navi_enable= 1;//Ĭ�Ͽ���
	navigation_structure.tx_pack->TxData.omni_enable= 0;//Ĭ�Ϲر�
	
	
	
	/*����״̬��ʼ��*/
	navigation_structure.state.offline_max_cnt = VISION_OFFLINE_MAX_CNT;
	navigation_structure.state.offline_cnt     = VISION_OFFLINE_MAX_CNT;
	navigation_structure.state.work_state      = VISION_OFFLINE;
	
}


/**
  * @Name    Vision_SendData
  * @brief   ��С����ͨѶ��CRCУ�����ݿ϶�Ҫ��
  * @param   None                              
  * @retval
  * @author  HWX
  * @Date    2022-10-21
**/
bool Navigation_SendData(void)
{
	/*�ж��Ƿ����*/
	if(navigation_structure.state.offline_cnt++ >= navigation_structure.state.offline_max_cnt)
	{
		navigation_structure.state.offline_cnt--;
		navigation_structure.state.work_state = VISION_OFFLINE;
	}
	else
	{
		navigation_structure.state.work_state = VISION_ONLINE;
	}
	
	/*���ݷ���*/	
	memcpy(navigation_txBuf, &navigation_tx_pack, sizeof(navigation_tx_pack));
	
	/*����CRCУ��λ*/
	Append_CRC8_Check_Sum(navigation_txBuf, LEN_NAVIGATION_FRAME_HEADER);
	Append_CRC16_Check_Sum(navigation_txBuf, LEN_NAVIGATION_TX_PACKET);
	
	
	if(HAL_UART_Transmit_DMA(&huart1,navigation_txBuf,sizeof(navigation_tx_pack)) == HAL_OK)
	{
		memset(&navigation_tx_pack,0,sizeof(navigation_tx_pack));
		return true;
	}
	else
	{
		memset(&navigation_tx_pack,0,sizeof(navigation_tx_pack));
		return false;
	}
}


/**
  * @Name    Vision_GetData
  * @brief   �������ݣ�һ����CRCУ��Ĳ�������Ҫ��
  * @param   None
  * @retval  
  * @author  HWX
  * @Date    2022-10-21
**/
bool Navigation_GetData(uint8_t *rxBuf)
{

	
	if(rxBuf[0] == 0xA5)
	{
		if(Verify_CRC8_Check_Sum(rxBuf, LEN_NAVIGATION_FRAME_HEADER) == true)
		{
			if(Verify_CRC16_Check_Sum(rxBuf,LEN_NAVIGATION_RX_PACKET) == true)
			{
				memcpy(&navigation_rx_pack, rxBuf, LEN_NAVIGATION_RX_PACKET);

				/*�쳣�������*/
				if(navigation_rx_pack.RxData.chassis_front >= 3200)
				{
					navigation_rx_pack.RxData.chassis_front = 3200; 
				}
				else if(navigation_rx_pack.RxData.chassis_front <= -3200)
				{
					navigation_rx_pack.RxData.chassis_front = -3200; 
				}
				
				if(navigation_rx_pack.RxData.chassis_right >= 3200)
				{
					navigation_rx_pack.RxData.chassis_right = 3200; 
				}
				else if(navigation_rx_pack.RxData.chassis_right <= -3200)
				{
					navigation_rx_pack.RxData.chassis_right = -3200; 
				}
				
				/*���߼���������*/
				navigation_structure.state.offline_cnt = 0;
				return true;
			}
		}
	}
	return false;
}

/**
  * @Name    Vision_Update
  * @brief   ��������
  * @param   None
  * @retval
  * @author  HWX
  * @Date    2022-10-21
**/
void Navigation_Update(void)
{
	//���ݽ����Ѿ���У�����ʱ�������
		
}

/**
  * @Name    USART1_rxDataHandler
  * @brief   ����1�����жϻص�����
  * @param   None
  * @retval
  * @author  HWX
  * @Date    2022-10-21
**/
void USART1_rxDataHandler(uint8_t *rxBuf)
{	
	Navigation_GetData(rxBuf);
}


void Navigation_Heart()
{
	if(navigation_structure.state.offline_cnt++ >= navigation_structure.state.offline_max_cnt)
	{
		navigation_structure.state.offline_cnt = navigation_structure.state.offline_max_cnt;
		navigation_structure.state.work_state = VISION_OFFLINE;
	}
	else
	{
		navigation_structure.state.work_state = VISION_ONLINE;
	}
}

/**
  * @Name    vision_task
  * @brief  
  * @param   None
  * @retval
  * @author  HWX
  * @Date    2022-10-24
**/
#define X_LIMIIT 14.0f
void Navigation_task(void)
{	
	
	
	/*�������ݴ���*/
		if(car_structure.mode == aim_CAR)
		{
			navigation_structure.tx_pack->FrameHeader.cmd_id   = CMD_CHECK;
			
		}
		else
		{
			if(judge.base_info->robot_commond == 'Y')//Y
			{
				navigation_structure.tx_pack->FrameHeader.cmd_id   = CMD_POINT_A;
			}
			else if(judge.base_info->robot_commond == 'K')//K
			{
				navigation_structure.tx_pack->FrameHeader.cmd_id   = CMD_POINT_B;
			}
			else if(judge.base_info->robot_commond == 'I')//I
			{
				navigation_structure.tx_pack->FrameHeader.cmd_id   = CMD_POINT_D;
			}
			else if(judge.base_info->robot_commond == 'V')//V
			{
				navigation_structure.tx_pack->FrameHeader.cmd_id   = CMD_POINT_E;
			}
			else if(judge.base_info->robot_commond == 'H')//H
			{
				navigation_structure.tx_pack->FrameHeader.cmd_id   = CMD_STOP;
			}
			else if(judge.base_info->robot_commond == 'O')//O
			{
				navigation_structure.tx_pack->FrameHeader.cmd_id   = CMD_POINT_G;
			}
			else if(judge.base_info->robot_commond == 'L')//L
			{
				navigation_structure.tx_pack->FrameHeader.cmd_id   = CMD_POINT_H;
			}
			else if(judge.base_info->robot_commond == 'T')//T
			{
				navigation_structure.tx_pack->FrameHeader.cmd_id   = CMD_R3;
			}
			else if(judge.base_info->robot_commond == 'G')//G
			{
				navigation_structure.tx_pack->FrameHeader.cmd_id   = CMD_POINT_J;
			}
			else if(judge.base_info->robot_commond == 'F')//F
			{
				navigation_structure.tx_pack->FrameHeader.cmd_id   = CMD_POINT_K;
			}
			else if(judge.base_info->robot_commond == 'Q')//Q
			{
				navigation_structure.tx_pack->FrameHeader.cmd_id   = CMD_OUTPOS;
			}
			else if(judge.base_info->robot_commond == 'N')//N
			{
				navigation_structure.tx_pack->FrameHeader.cmd_id   = CMD_R4;
			}
			else if(judge.base_info->robot_commond == 'Z')//Z
			{
				navigation_structure.tx_pack->FrameHeader.cmd_id   = CMD_GOGOGO;
				navigation_structure.tx_pack->TxData.target_x = judge.data->ext_robot_command.target_x;
				navigation_structure.tx_pack->TxData.target_y = judge.data->ext_robot_command.target_y;
			}
			else if(judge.base_info->robot_commond == 'J')//J
			{
				navigation_structure.tx_pack->FrameHeader.cmd_id   = CMD_FUCK_OUTPOS;
			}
			else
			{
				navigation_structure.tx_pack->FrameHeader.cmd_id   = CMD_STOP;
			}
			
		}

		if(judge.base_info->robot_commond == 'R')//R
		{
			if(judge.data->ext_robot_command.target_x >= X_LIMIIT)
			{
				navigation_structure.tx_pack->TxData.navi_enable = 1;
			}	
			else
			{
				navigation_structure.tx_pack->TxData.navi_enable = 0;
			}
			
		}
		

	navigation_structure.tx_pack->TxData.B_yaw_angle   = bmi_structure.yaw_angle;
	navigation_structure.tx_pack->TxData.game_progress = judge.base_info->game_progress;
	navigation_structure.tx_pack->TxData.uwb_x         = judge.data->game_robot_pos.x;
	navigation_structure.tx_pack->TxData.uwb_y         = judge.data->game_robot_pos.y;
	navigation_structure.tx_pack->TxData.my_color      = judge.base_info->car_color;	
	navigation_structure.tx_pack->TxData.ammor_size_3  = BIGGG_ARMOR;		
	navigation_structure.tx_pack->TxData.ammor_size_4  = BIGGG_ARMOR;
	navigation_structure.tx_pack->TxData.ammor_size_5  = BIGGG_ARMOR;
	navigation_structure.tx_pack->TxData.nuc_flag1     = vision_structure.rx_pack->RxData.nuc_flag1;
	/* ����5�뵹��ʱ�׶� */
	if (judge.data->game_status.game_progress == 3)
	{
		/* �췽 */
		if (judge.base_info->car_color == 0)
		{

			if (judge.data->game_robot_HP.blue_3_robot_HP == BALANCE_HP)
			{
				navigation_structure.tx_pack->TxData.ammor_size_3 = BIGGG_ARMOR;
			}
			else
			{
				navigation_structure.tx_pack->TxData.ammor_size_3 = SMALL_ARMOR;
			}
			
			if (judge.data->game_robot_HP.blue_4_robot_HP == BALANCE_HP)
			{
				navigation_structure.tx_pack->TxData.ammor_size_4 = BIGGG_ARMOR;
			}
			else
			{
				navigation_structure.tx_pack->TxData.ammor_size_4 = SMALL_ARMOR;
			}
			
			if (judge.data->game_robot_HP.blue_5_robot_HP == BALANCE_HP)
			{
				navigation_structure.tx_pack->TxData.ammor_size_5 = BIGGG_ARMOR;
			}
			else
			{
				navigation_structure.tx_pack->TxData.ammor_size_5 = SMALL_ARMOR;
			}
		}
		/* ���� */
		if (judge.base_info->car_color == 1)
		{
			if (judge.data->game_robot_HP.red_3_robot_HP == BALANCE_HP)
			{
				navigation_structure.tx_pack->TxData.ammor_size_3 = BIGGG_ARMOR;
			}
			else
			{
				navigation_structure.tx_pack->TxData.ammor_size_3 = SMALL_ARMOR;
			}
			
			if (judge.data->game_robot_HP.red_4_robot_HP == BALANCE_HP)
			{
				navigation_structure.tx_pack->TxData.ammor_size_4 = BIGGG_ARMOR;
			}
			else
			{
				navigation_structure.tx_pack->TxData.ammor_size_4 = SMALL_ARMOR;
			}
			
			if (judge.data->game_robot_HP.red_5_robot_HP == BALANCE_HP)
			{
				navigation_structure.tx_pack->TxData.ammor_size_5 = BIGGG_ARMOR;
			}
			else
			{
				navigation_structure.tx_pack->TxData.ammor_size_5 = SMALL_ARMOR;
			}
		}
	}
	/*�Ӿ����ߴ���*/
	if(navigation_structure.state.work_state == VISION_OFFLINE)	
	{
		navigation_structure.rx_pack->RxData.chassis_front = 0;
		navigation_structure.rx_pack->RxData.chassis_right = 0;
		
		navigation_structure.rx_pack->RxData.enemy1_num    = 0;
		navigation_structure.rx_pack->RxData.enemy1_x      = 0;
		navigation_structure.rx_pack->RxData.enemy1_y      = 0;
		navigation_structure.rx_pack->RxData.enemy1_z      = 0;
		
		navigation_structure.rx_pack->RxData.enemy2_num    = 0;
		navigation_structure.rx_pack->RxData.enemy2_x      = 0;
		navigation_structure.rx_pack->RxData.enemy2_y      = 0;
		navigation_structure.rx_pack->RxData.enemy2_z      = 0;
		
		navigation_structure.rx_pack->RxData.enemy3_num    = 0;
		navigation_structure.rx_pack->RxData.enemy3_x      = 0;
		navigation_structure.rx_pack->RxData.enemy3_y      = 0;
		navigation_structure.rx_pack->RxData.enemy3_z      = 0;
		
		navigation_structure.rx_pack->RxData.enemy4_num    = 0;
		navigation_structure.rx_pack->RxData.enemy4_x      = 0;
		navigation_structure.rx_pack->RxData.enemy4_y      = 0;
		navigation_structure.rx_pack->RxData.enemy4_z      = 0;
	}
	
	
	Navigation_SendData();

}




