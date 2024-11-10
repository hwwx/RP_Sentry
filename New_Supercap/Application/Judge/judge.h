/**
  ******************************************************************************
  * @file           : judge.h
  * @brief          : 
  * @update         : finish 2022-2-13 20:10:12
  ******************************************************************************
  */

#ifndef __JUDGE_H
#define __JUDGE_H

#include "stm32f3xx_hal.h"
#include "judge_protocol.h"


#define JUDGE_ONLINE 1
#define JUDGE_OFFLINE 0

typedef struct
{
	uint16_t offline_cnt_max;
	uint8_t status;
	uint16_t offline_cnt;
}judge_info_t;


typedef struct 
{
	ext_rfid_status_t       rfid_status;
	ext_game_status_t       game_status;
	ext_game_robot_status_t game_robot_status;
	ext_power_heat_data_t   power_heat_data;
	ext_shoot_data_t        shoot_data;
	ext_game_robot_pos_t    game_robot_pos;
	ext_robot_hurt_t        ext_robot_hurt;
	ext_aerial_data_t       ext_aerial_data;
	ext_robot_command_t     ext_robot_command;
	ext_game_robot_HP_t     game_robot_HP;
	ext_bullet_remaining_t  bullet_remaining;
	ground_robot_position_t robot_position;
	
}judge_rawdata_t;


typedef struct 
{
	judge_info_t *info;
	judge_rawdata_t *data;

	void (*heart)(void);
	void (*update)(uint8_t *rxBuf);

}judge_t;
/*--------------------裁判系统发送内容-----------------------*/
typedef struct __packed
{
	uint8_t  			    sof;		// 同步头
	uint16_t  	            data_length;// 命令码
	uint8_t                 seq;
	uint8_t  			    crc8;		// CRC8校验码
	uint16_t                cmd_id;
} Judge_frame_header_t;

/* 帧尾格式 */
typedef struct __packed 
{
	uint16_t crc16;					// CRC16校验码
} Judge_frame_tailer_t;

/* 发送包格式 */
typedef struct __packed
{
	Judge_frame_header_t   FrameHeader;	// 帧头
	map_sentry_data_t	   TxData;		    // 数据
	Judge_frame_tailer_t   FrameTailer;	// 帧尾	
}Judge_tx_packet_t;

extern judge_t judge;

void judge_recive(uint8_t *data);
void judge_heart();


#endif
