#include "vision.h"
#include "navigation.h"
#include "bmi.h"
#include "can_protocol.h"
#include "rp_shoot.h"
#include "rp_gimbal.h"
#include "judge.h"
#include "Car.h"

/*使用串口3*/
extern UART_HandleTypeDef huart3;//使用串口3

extern judge_t          judge;
extern Master_Head_t    Master_Head_structure;
extern bmi_t            bmi_structure;
extern car_t            car_structure;
extern navigation_t     navigation_structure;

vision_rx_packet_t      vision_rx_pack;
vision_tx_packet_t      vision_tx_pack;
vision_t                vision_structure;

uint8_t                 must_patrol_enable = 0;


/*发送Buffer*/
uint8_t                 vision_txBuf[80]; 


/**
  * @Name    Vision_Init
  * @brief   视觉部分结构体初始化
  * @param   None
  * @retval  
  * @author  HWX
  * @Date    2022-10-21
**/
void Vision_Init(void)
{
	/*结构体初始化*/
	vision_structure.rx_pack = &vision_rx_pack;
	vision_structure.tx_pack = &vision_tx_pack;
	
	/*数据初始化*/
	vision_structure.tx_pack->FrameHeader.sof                   = VISION_SEND_ID;
	vision_structure.tx_pack->FrameHeader.cmd_id                = CMD_PATROL;
	vision_structure.tx_pack->TxData.flag.bit.engine_hit_enable = 1;
	vision_structure.tx_pack->TxData.flag.bit.hero_hit_enable   = 0;
	vision_structure.tx_pack->TxData.flag.bit.only_outpose      = 0;
	vision_structure.tx_pack->TxData.only_sentry                = 0;
	
	
	/*工作状态初始化*/
	vision_structure.state.offline_max_cnt = VISION_OFFLINE_MAX_CNT;
	vision_structure.state.offline_cnt     = VISION_OFFLINE_MAX_CNT;
	vision_structure.state.work_state      = VISION_OFFLINE;
	
	
	
	
	
}


/**
  * @Name    Vision_SendData
  * @brief   与小电脑通讯，CRC校验数据肯定要改
  * @param   None                              
  * @retval
  * @author  HWX
  * @Date    2022-10-21
**/
bool Vision_SendData(void)
{
	/*判断是否掉线*/
	if(vision_structure.state.offline_cnt++ >= vision_structure.state.offline_max_cnt)
	{
		vision_structure.state.offline_cnt--;
		vision_structure.state.work_state = VISION_OFFLINE;
	}
	else
	{
		vision_structure.state.work_state = VISION_ONLINE;
	}
	
	/*数据发送*/	
	memcpy(vision_txBuf, &vision_tx_pack, sizeof(vision_tx_packet_t));
	
	/*增加CRC校验位*/
	Append_CRC8_Check_Sum(vision_txBuf, LEN_FRAME_HEADER);
	Append_CRC16_Check_Sum(vision_txBuf, LEN_VISION_TX_PACKET);
	
	
	if(HAL_UART_Transmit_DMA(&huart3,vision_txBuf,sizeof(vision_tx_packet_t)) == HAL_OK)
	{
		memset(&vision_tx_pack,sizeof(vision_tx_pack),0);
		return true;
	}
	else
	{
		memset(&vision_tx_pack,sizeof(vision_tx_pack),0);
		return false;
	}
}


/**
  * @Name    Vision_GetData
  * @brief   接收数据，一样的CRC校验的参数可能要改
  * @param   None
  * @retval  
  * @author  HWX
  * @Date    2022-10-21
**/
bool Vision_GetData(uint8_t *rxBuf)
{

	
	if(rxBuf[0] == 0xA5)
	{
		if(Verify_CRC8_Check_Sum(rxBuf, LEN_FRAME_HEADER) == true)
		{
			if(Verify_CRC16_Check_Sum(rxBuf,LEN_VISION_RX_PACKET) == true)
			{
				memcpy(&vision_rx_pack, rxBuf, LEN_VISION_RX_PACKET);
//				/*超热量测试代码*/
//				vision_structure.rx_pack->RxData.L_is_find_target = 1;
//				vision_structure.rx_pack->RxData.L_shoot_speed = 250;
//				vision_structure.rx_pack->RxData.L_shoot_cnt += 3;
				
				/*异常数据清除*/
				
				if(vision_rx_pack.RxData.L_shoot_speed >= 250)
				{
					vision_rx_pack.RxData.L_shoot_speed = 250; 
				}

				if(vision_rx_pack.RxData.R_shoot_speed >= 250)
				{
					vision_rx_pack.RxData.R_shoot_speed = 250; 
				}
				
				/*没干死广工的秘密武器*/
				if(vision_rx_pack.RxData.R_pitch_angle <= P_MOT_LOW_LIMIT  || vision_rx_pack.RxData.R_pitch_angle >= 4000)
				{
					vision_rx_pack.RxData.R_pitch_angle = P_MOT_LOW_LIMIT; 
				}
				else if(vision_rx_pack.RxData.R_pitch_angle >= P_MOT_UPP_LIMIT &&  vision_rx_pack.RxData.R_pitch_angle <= 4000)
				{
					vision_rx_pack.RxData.R_pitch_angle = P_MOT_UPP_LIMIT;
				}
				
				if(vision_rx_pack.RxData.L_pitch_angle <= P_MOT_LOW_LIMIT_ &&  vision_rx_pack.RxData.L_pitch_angle >= 3000)
				{
					vision_rx_pack.RxData.L_pitch_angle = P_MOT_LOW_LIMIT_;
				}
				else if(vision_rx_pack.RxData.L_pitch_angle >= P_MOT_UPP_LIMIT_ ||  vision_rx_pack.RxData.L_pitch_angle <= 3000)
				{
					vision_rx_pack.RxData.L_pitch_angle = P_MOT_UPP_LIMIT_;
				}
				/*掉线计数器清零*/
				vision_structure.state.offline_cnt = 0;
				return true;
			}
		}
	}
	return false;
}

/**
  * @Name    Vision_Update
  * @brief   解析数据
  * @param   None
  * @retval
  * @author  HWX
  * @Date    2022-10-21
**/
void Vision_Update(void)
{
	//数据接收已经在校验完成时便结束了
		
}

/**
  * @Name    USART3_rxDataHandler
  * @brief   串口3接收中断回调函数
  * @param   None
  * @retval
  * @author  HWX
  * @Date    2022-10-21
**/
void USART3_rxDataHandler(uint8_t *rxBuf)
{	
	Vision_GetData(rxBuf);
}


void Vision_Heart()
{
	if(vision_structure.state.offline_cnt++ >= vision_structure.state.offline_max_cnt)
	{
		vision_structure.state.offline_cnt = vision_structure.state.offline_max_cnt;
		vision_structure.state.work_state = VISION_OFFLINE;
	}
	else
	{
		vision_structure.state.work_state = VISION_ONLINE;
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
void vision_task(void)
{	
	
	
	/*发送数据处理*/
		if(car_structure.mode == aim_CAR)
		{
			vision_structure.tx_pack->FrameHeader.cmd_id   = CMD_CHECK;
			
		}
		else
		{
			if(judge.base_info->robot_commond == 'Y')//Y
			{
				vision_structure.tx_pack->FrameHeader.cmd_id   = CMD_POINT_A;
			}
			else if(judge.base_info->robot_commond == 'K')//K
			{
				vision_structure.tx_pack->FrameHeader.cmd_id   = CMD_POINT_B;
			}
			else if(judge.base_info->robot_commond == 'I')//I
			{
				vision_structure.tx_pack->FrameHeader.cmd_id   = CMD_POINT_D;
			}
			else if(judge.base_info->robot_commond == 'V')//V
			{
				vision_structure.tx_pack->FrameHeader.cmd_id   = CMD_POINT_E;
			}
			else if(judge.base_info->robot_commond == 'H')//H
			{
				vision_structure.tx_pack->FrameHeader.cmd_id   = CMD_STOP;
			}
			else if(judge.base_info->robot_commond == 'O')//O
			{
				vision_structure.tx_pack->FrameHeader.cmd_id   = CMD_POINT_G;
			}
			else if(judge.base_info->robot_commond == 'L')//L
			{
				vision_structure.tx_pack->FrameHeader.cmd_id   = CMD_POINT_H;
			}
			else if(judge.base_info->robot_commond == 'T')//T
			{
				vision_structure.tx_pack->FrameHeader.cmd_id   = CMD_R3;
			}
			else if(judge.base_info->robot_commond == 'G')//G
			{
				vision_structure.tx_pack->FrameHeader.cmd_id   = CMD_POINT_J;
			}
			else if(judge.base_info->robot_commond == 'F')//F
			{
				vision_structure.tx_pack->FrameHeader.cmd_id   = CMD_POINT_K;
			}
			else if(judge.base_info->robot_commond == 'Q')//Q
			{
				vision_structure.tx_pack->FrameHeader.cmd_id   = CMD_OUTPOS;
			}
			else if(judge.base_info->robot_commond == 'N')//N
			{
				vision_structure.tx_pack->FrameHeader.cmd_id   = CMD_R4;
			}
			else if(judge.base_info->robot_commond == 'Z')//Z
			{
				vision_structure.tx_pack->FrameHeader.cmd_id   = CMD_GOGOGO;
			}
			else if(judge.base_info->robot_commond == 'J')//J
			{
				vision_structure.tx_pack->FrameHeader.cmd_id   = CMD_FUCK_OUTPOS;
			}
			else
			{
				vision_structure.tx_pack->FrameHeader.cmd_id   = CMD_STOP;
			}
			
		}

		if(judge.base_info->robot_commond == 'E')//E
		{
			if(judge.data->ext_robot_command.target_x >= X_LIMIIT)
			{
				vision_structure.tx_pack->TxData.flag.bit.engine_hit_enable = 1;
			}	
			else
			{
				vision_structure.tx_pack->TxData.flag.bit.engine_hit_enable = 0;
			}
			
		}
		
		if(judge.base_info->robot_commond == 'C')//C
		{
			
			if(judge.data->ext_robot_command.target_x >= X_LIMIIT)
			{
				vision_structure.tx_pack->TxData.flag.bit.hero_hit_enable = 1;
			}	
			else
			{  
				vision_structure.tx_pack->TxData.flag.bit.hero_hit_enable = 0;
			}
			
		}
					
		if(judge.base_info->robot_commond == 'X')//X
		{
			
			if(judge.data->ext_robot_command.target_x >= X_LIMIIT)
			{
				must_patrol_enable = 1;
			}	
			else
			{  
			    must_patrol_enable = 0;
			}
			
		}
		
		if(judge.base_info->robot_commond == 'B')//B
		{
			
			if(judge.data->ext_robot_command.target_x >= X_LIMIIT)
			{
				vision_structure.tx_pack->TxData.only_sentry = 1;
			}	
			else
			{  
			    vision_structure.tx_pack->TxData.only_sentry = 0;
			}
			
		}
		
		if(vision_structure.tx_pack->FrameHeader.cmd_id == CMD_POINT_A || vision_structure.tx_pack->FrameHeader.cmd_id == CMD_FUCK_OUTPOS) 
		{
			vision_structure.tx_pack->TxData.flag.bit.only_outpose = 1;
		}	
		else
		{  
			vision_structure.tx_pack->TxData.flag.bit.only_outpose = 0;
		}

		
		vision_structure.tx_pack->TxData.R_pitch_angle           = Master_Head_structure.From_R_Head.measure_pit;
		vision_structure.tx_pack->TxData.R_yaw_angle             = Master_Head_structure.From_R_Head.measure_yaw;
		vision_structure.tx_pack->TxData.L_pitch_angle           = Master_Head_structure.From_L_Head.measure_pit;
		vision_structure.tx_pack->TxData.L_yaw_angle             = Master_Head_structure.From_L_Head.measure_yaw;
		vision_structure.tx_pack->TxData.B_yaw_angle             = bmi_structure.yaw_angle;
		vision_structure.tx_pack->TxData.flag.bit.my_color       = judge.base_info->car_color;
		vision_structure.tx_pack->TxData.flag.bit.game_progress  = (judge.base_info->game_progress == 4)? 1:0;
		
		vision_structure.tx_pack->TxData.remain_bullet           = judge.base_info->remain_bullte;
		vision_structure.tx_pack->TxData.friendly_outpose_HP     = judge.base_info->friendly_outposts_HP;
		vision_structure.tx_pack->TxData.my_HP                   = judge.base_info->remain_HP/10;
		vision_structure.tx_pack->TxData.enemy_hero_HP           = judge.base_info->enemy_hero_HP;
		vision_structure.tx_pack->TxData.enemy_engening_HP       = judge.base_info->enemy_engeing;
		vision_structure.tx_pack->TxData.enemy_infantry3_HP      = judge.base_info->enemy_infanry3_HP;
		vision_structure.tx_pack->TxData.enemy_infantry4_HP      = judge.base_info->enemy_infanry4_HP;
		vision_structure.tx_pack->TxData.enemy_infantry5_HP      = judge.base_info->enemy_infanry5_HP;
		vision_structure.tx_pack->TxData.enemy_sentry_HP         = judge.base_info->enemy_Shaobing_HP;
		vision_structure.tx_pack->TxData.enemy_outpose_HP        = judge.base_info->enemy_outposts_HP;
		
		vision_structure.tx_pack->TxData.enemy1_num              = navigation_structure.rx_pack->RxData.enemy1_num;
		vision_structure.tx_pack->TxData.enemy1_x                = navigation_structure.rx_pack->RxData.enemy1_x;
		vision_structure.tx_pack->TxData.enemy1_y                = navigation_structure.rx_pack->RxData.enemy1_y;
		vision_structure.tx_pack->TxData.enemy1_z                = navigation_structure.rx_pack->RxData.enemy1_z;


		vision_structure.tx_pack->TxData.enemy2_num              = navigation_structure.rx_pack->RxData.enemy2_num;
		vision_structure.tx_pack->TxData.enemy2_x                = navigation_structure.rx_pack->RxData.enemy2_x;
		vision_structure.tx_pack->TxData.enemy2_y                = navigation_structure.rx_pack->RxData.enemy2_y;
		vision_structure.tx_pack->TxData.enemy2_z                = navigation_structure.rx_pack->RxData.enemy2_z;
		
		vision_structure.tx_pack->TxData.enemy3_num              = navigation_structure.rx_pack->RxData.enemy3_num;
		vision_structure.tx_pack->TxData.enemy3_x                = navigation_structure.rx_pack->RxData.enemy3_x;
		vision_structure.tx_pack->TxData.enemy3_y                = navigation_structure.rx_pack->RxData.enemy3_y;
		vision_structure.tx_pack->TxData.enemy3_z                = navigation_structure.rx_pack->RxData.enemy3_z;
		
		vision_structure.tx_pack->TxData.enemy4_num              = navigation_structure.rx_pack->RxData.enemy4_num;
		vision_structure.tx_pack->TxData.enemy4_x                = navigation_structure.rx_pack->RxData.enemy4_x;
		vision_structure.tx_pack->TxData.enemy4_y                = navigation_structure.rx_pack->RxData.enemy4_y;
		vision_structure.tx_pack->TxData.enemy4_z                = navigation_structure.rx_pack->RxData.enemy4_z;
		
		vision_structure.tx_pack->TxData.arrive_flag             = navigation_structure.rx_pack->RxData.arrive_flag;
		
		/* 比赛5秒倒计时阶段 */
		if (judge.data->game_status.game_progress == 3)
		{
			/* 红方 */
			if (judge.base_info->car_color == 0)
			{

				if (judge.data->game_robot_HP.blue_3_robot_HP == BALANCE_HP)
				{
					vision_structure.tx_pack->TxData.flag.bit.ammor_size_3 = BIGGG_ARMOR;
				}
				else
				{
					vision_structure.tx_pack->TxData.flag.bit.ammor_size_3 = SMALL_ARMOR;
				}
				
				if (judge.data->game_robot_HP.blue_4_robot_HP == BALANCE_HP)
				{
					vision_structure.tx_pack->TxData.flag.bit.ammor_size_4 = BIGGG_ARMOR;
				}
				else
				{
					vision_structure.tx_pack->TxData.flag.bit.ammor_size_4 = SMALL_ARMOR;
				}
				
				if (judge.data->game_robot_HP.blue_5_robot_HP == BALANCE_HP)
				{
					vision_structure.tx_pack->TxData.flag.bit.ammor_size_5 = BIGGG_ARMOR;
				}
				else
				{
					vision_structure.tx_pack->TxData.flag.bit.ammor_size_5 = SMALL_ARMOR;
				}
			}
			/* 蓝方 */
			if (judge.base_info->car_color == 1)
			{
				if (judge.data->game_robot_HP.red_3_robot_HP == BALANCE_HP)
				{
					vision_structure.tx_pack->TxData.flag.bit.ammor_size_3 = BIGGG_ARMOR;
				}
				else
				{
					vision_structure.tx_pack->TxData.flag.bit.ammor_size_3 = SMALL_ARMOR;
				}
				
				if (judge.data->game_robot_HP.red_4_robot_HP == BALANCE_HP)
				{
					vision_structure.tx_pack->TxData.flag.bit.ammor_size_4 = BIGGG_ARMOR;
				}
				else
				{
					vision_structure.tx_pack->TxData.flag.bit.ammor_size_4 = SMALL_ARMOR;
				}
				
				if (judge.data->game_robot_HP.red_5_robot_HP == BALANCE_HP)
				{
					vision_structure.tx_pack->TxData.flag.bit.ammor_size_5 = BIGGG_ARMOR;
				}
				else
				{
					vision_structure.tx_pack->TxData.flag.bit.ammor_size_5 = SMALL_ARMOR;
				}
			}
		}
		

		/*视觉掉线处理*/
		if(vision_structure.state.work_state == VISION_OFFLINE)	
		{
			vision_structure.rx_pack->RxData.L_shoot_speed = 0;
			vision_structure.rx_pack->RxData.L_is_find_target = 0;
			
			vision_structure.rx_pack->RxData.R_shoot_speed = 0;
			vision_structure.rx_pack->RxData.R_is_find_target = 0;
			
			vision_structure.rx_pack->FrameHeader.cmd_id = 0;
			
		}
	
	
	Vision_SendData();

}




