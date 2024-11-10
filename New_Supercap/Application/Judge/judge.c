/**
  ************************************* Copyright ******************************   
  *                 (C) Copyright 2022, hwx, China, SZU.
  *                            N0   Rights  Reserved
  *                              
  *                   
  * @FileName   : rp_chassis.c   
  * @Version    : v1.1		
  * @Author     : hwx			
  * @Date       : 2023年2月28日         
  * @Description:    
  *
  *
  ******************************************************************************
 */
#include <string.h>
#include "crc.h"
#include "judge_protocol.h"
#include "judge.h"
#include "drv_can.h"



judge_frame_header_t judge_frame_header;

drv_judge_info_t drv_judge_info = {
	.frame_header = &judge_frame_header,
};

judge_rawdata_t judge_rawdata;

judge_info_t judge_info = 
{
	.offline_cnt_max = 1000,
	
};

judge_t judge = 
{
	.info      = &judge_info,
	.data      = &judge_rawdata,

	.heart	   = judge_heart,
	.update    = judge_recive,
};


void judge_heart()
{
	if(judge.info->offline_cnt ++ >= judge.info->offline_cnt_max)
	{
		judge.info->offline_cnt = judge.info->offline_cnt_max;
		judge.info->status = JUDGE_OFFLINE;
	}
	else
	{
		judge.info->status = JUDGE_ONLINE;
	}
	
}

/*其中的所有函数，若为连接到裁判系统或裁判系统不发送该指令，则不会运行*/
void judge_update(uint16_t id, uint8_t *rxBuf)
{
	/*掉线计数器清零*/
	judge.info->offline_cnt = 0;
	
	switch(id)
	{
		case ID_rfid_status:
			/*数据接收-----------------------------------------------------------------------------------------*/
			memcpy(&judge.data->rfid_status, rxBuf, LEN_rfid_status);
			break;
		case ID_game_state:
			/*数据接收-----------------------------------------------------------------------------------------*/
			memcpy(&judge.data->game_status, rxBuf, LEN_game_state);
			break;
		
		case ID_power_heat_data:
			/*数据接收-----------------------------------------------------------------------------------------*/
			memcpy(&judge.data->power_heat_data, rxBuf, LEN_power_heat_data);
			break;
		case ID_game_robot_state:
			/*数据接收-----------------------------------------------------------------------------------------*/			
			memcpy(&judge.data->game_robot_status, rxBuf, LEN_game_robot_state);
			break;
		case ID_shoot_data:
			/*数据接收-----------------------------------------------------------------------------------------*/
			memcpy(&judge.data->shoot_data, rxBuf, LEN_shoot_data);
			break;
		case ID_game_robot_pos:
			/*数据接收-----------------------------------------------------------------------------------------*/
			memcpy(&judge.data->game_robot_pos, rxBuf, LEN_game_robot_pos);
			break;
		case ID_robot_hurt:
			/*数据接收-----------------------------------------------------------------------------------------*/
			memcpy(&judge.data->ext_robot_hurt, rxBuf, LEN_robot_hurt);
			break;
		case ID_aerial_data:
			/*数据接收-----------------------------------------------------------------------------------------*/
			memcpy(&judge.data->ext_robot_command, rxBuf, LEN_aerial_data);
			break;
		case ID_game_robot_HP:
			/*数据接收-----------------------------------------------------------------------------------------*/
			memcpy(&judge.data->game_robot_HP, rxBuf, LEN_game_robot_HP);
			break;
		case ID_bullet_remaining:
			/*数据接收-----------------------------------------------------------------------------------------*/
			memcpy(&judge.data->bullet_remaining, rxBuf, LEN_bullet_remaining);
			break;
		case ID_ground_robot_position:
			/*数据接收-----------------------------------------------------------------------------------------*/
			memcpy(&judge.data->robot_position, rxBuf, LEN_ground_robot_position);
		default:
			break;
	}
}

void judge_recive(uint8_t *rxBuf)
{
	uint16_t frame_length;
	if( rxBuf == NULL )
	{
		return;
	}
	drv_judge_info.frame_header->SOF = rxBuf[0];
	if(drv_judge_info.frame_header->SOF == 0xA5)
	{
		memcpy(&drv_judge_info.frame_header->data_length, rxBuf + 1, 4);
		if(Verify_CRC8_Check_Sum(rxBuf, 5) == 1)
		{
			frame_length = 5 + 2 + drv_judge_info.frame_header->data_length + 2;
			if(Verify_CRC16_Check_Sum(rxBuf, frame_length) == 1)
			{
				memcpy(&drv_judge_info.cmd_id, rxBuf + 5, 2);
				judge_update(drv_judge_info.cmd_id, rxBuf + 7);
			}
			memcpy(&drv_judge_info.frame_tail, rxBuf + 5 + 2 + drv_judge_info.frame_header->data_length, 2);
		}
	}

	/* 如果一个数据包出现了多帧数据就再次读取 */
	if(rxBuf[frame_length] == 0xA5)
	{
		judge_recive( &rxBuf[frame_length] );
	}
}


void USART1_rxDataHandler(uint8_t *rxBuf)
{	
	judge_recive(rxBuf);
}