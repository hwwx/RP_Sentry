/**
  ************************************* Copyright ******************************   
  *                 (C) Copyright 2022, hwx, China, SZU.
  *                            N0   Rights  Reserved
  *                              
  *                   
  * @FileName   : rp_chassis.c   
  * @Version    : v2.0		
  * @Author     : hwx			
  * @Date       : 2023-7-06         
  * @Description:    
  *
  *
  ******************************************************************************
 */
 
/* Includes ------------------------------------------------------------------*/
#include "stm32F4xx_hal.h"
#include "rp_shoot.h"
#include "remote.h"
#include "vision.h"
#include "Car.h"
#include "judge.h" 
#include "can_protocol.h"
#include <stdlib.h>
/* Private function prototypes -----------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
extern judge_t                       judge;
extern car_t                         car_structure;
extern shoot_t                        L_shoot_structure;
extern shoot_t                       R_shoot_structure;
extern motor_2006_t                  L_motor_2006_PLUCK_structure;
extern motor_2006_t                  R_motor_2006_PLUCK_structure;
extern vision_t                      vision_structure;
extern rc_t                          rc_structure;
extern Master_Head_t                 Master_Head_structure;
extern pid_t                         L_pid_p_stir_sin; /*单发速度环*/
/*超射速处理*/
extern uint8_t                       shoot_1_over_speed_flag;
extern uint8_t                       shoot_2_over_speed_flag;

/*防止同时打蛋*/
extern float                         elapsed_seconds;

/* Private macro -------------------------------------------------------------*/

#define AERIAL_SHOOT      (1)
#define AERIAL_STOP_SHOOT (0)
#define SIMULATAN_TIME    (0.0005f)
/* Private variables ---------------------------------------------------------*/

/*停止转动计数器*/
uint16_t no_shoot_time = 0;

float   L_shoot_speed       = 0;
float   R_shoot_speed       = 0;

/*视觉计数器*/
uint8_t L_shoot_cnt         = 0;
uint8_t R_shoot_cnt         = 0;
/*视觉计数器-上次*/
uint8_t L_shoot_cnt_last    = 0;
uint8_t R_shoot_cnt_last    = 0;

uint8_t shoot_aerial_cmd     = AERIAL_SHOOT;
uint8_t Simultaneous_fire    = 0;


void shoot_work(shoot_t* shoot)
{

	/*比赛开始视觉*/
	if(rc_structure.info->status == REMOTE_ONLINE )
	{
		if(judge.base_info->game_progress == 4) //若比赛开始则强制将打蛋控制权给视觉
		{
			L_shoot_structure.status = Visin_Shoot;
			R_shoot_structure.status = Visin_Shoot;
			
		}
		else if(judge.base_info->game_progress == 5)//比赛结束强制关闭发射机构
		{
			L_shoot_structure.status = Stop_Shoot;
			R_shoot_structure.status = Stop_Shoot;
		}
	}
	else
	{
		L_shoot_structure.status = Stop_Shoot;
		R_shoot_structure.status = Stop_Shoot;
	}
	
					

	
	/*视觉射速处理*/
	L_shoot_speed     = -((vision_structure.rx_pack->RxData.L_shoot_speed/10.0)/8.0)*60*36;
	L_shoot_cnt_last  =    L_shoot_cnt;
	L_shoot_cnt       =    vision_structure.rx_pack->RxData.L_shoot_cnt;
	
	R_shoot_speed     = -((vision_structure.rx_pack->RxData.R_shoot_speed/10.0)/8.0)*60*36;
	R_shoot_cnt_last  =    R_shoot_cnt;
	R_shoot_cnt       =    vision_structure.rx_pack->RxData.R_shoot_cnt;
	
	/*自动打蛋*/
	if(L_shoot_structure.status == Visin_Shoot)
	{
		int32_t L_delta_angle = abs(L_shoot_structure.stir_wheel->base_info->target_angle_sum - L_shoot_structure.stir_wheel->base_info->angle_sum);
		
		/*累计未发弹量不超3发*/
		if( L_shoot_cnt != L_shoot_cnt_last && L_delta_angle <= 36864 * 3)
		{
			L_shoot_structure.stir_wheel->base_info->target_angle_sum -= 36864;
		}
		
		/*限制丢失目标之后还在打*/
		if(vision_structure.rx_pack->RxData.L_is_find_target != 1)
		{
			/*速度清零*/
			L_shoot_speed = 0;
			/*角度和清零*/
			L_shoot_structure.stir_wheel->base_info->target_angle_sum = 0;	
			L_shoot_structure.stir_wheel->base_info->angle_sum = 0;
		}
		 
		/*限制位置环最大速度，相当于限制射频*/
		L_shoot_structure.pid_p_stir_sin->MaxOutput = (-1)*L_shoot_speed;
	}

	if(R_shoot_structure.status == Visin_Shoot)
	{
		int32_t R_delta_angle = abs(R_shoot_structure.stir_wheel->base_info->target_angle_sum - R_shoot_structure.stir_wheel->base_info->angle_sum);
		
		/*累计未发弹量不超3发*/
		if( R_shoot_cnt != R_shoot_cnt_last && R_delta_angle <= 36864 * 3)
		{
			R_shoot_structure.stir_wheel->base_info->target_angle_sum -= 36864;
		}
		
		if(vision_structure.rx_pack->RxData.R_is_find_target != 1 )
		{
			/*速度清零*/
			R_shoot_speed = 0;
			/*角度和清零*/
			R_shoot_structure.stir_wheel->base_info->target_angle_sum = 0;
			R_shoot_structure.stir_wheel->base_info->angle_sum = 0;
		}
		
		/*限制位置环最大速速，相当于限制射速*/
		R_shoot_structure.pid_p_stir_sin->MaxOutput = (-1)*R_shoot_speed;
	}

	
	/*枪管热量限制*/
	/*双枪未连服务器只有一根枪管热量*/
	 judge.data->game_robot_status.shooter_id2_17mm_cooling_limit = 240;
	 judge.data->game_robot_status.shooter_id1_17mm_cooling_limit = 240;
	
	if(judge.base_info->shooter_id2_cooling_heat >= judge.data->game_robot_status.shooter_id2_17mm_cooling_limit- 80 )
	{
		if(L_shoot_speed <= -2400)
		{
			L_shoot_speed = -2400;
		}
		if(judge.base_info->shooter_id2_cooling_heat >= judge.data->game_robot_status.shooter_id2_17mm_cooling_limit- 45 )
		{
			L_shoot_speed = 0;
			L_shoot_structure.stir_wheel->base_info->target_angle_sum = 0;
			L_shoot_structure.stir_wheel->base_info->angle_sum = 0;
		}
	}
	
	if(judge.base_info->shooter_id1_cooling_heat >= judge.data->game_robot_status.shooter_id1_17mm_cooling_limit- 80 )
	{
		if(R_shoot_speed <= -2400)
		{
			R_shoot_speed = -2400;
		}
		if(judge.base_info->shooter_id1_cooling_heat >= judge.data->game_robot_status.shooter_id1_17mm_cooling_limit- 45 )
		{
			R_shoot_speed = 0;
			R_shoot_structure.stir_wheel->base_info->target_angle_sum = 0;
			R_shoot_structure.stir_wheel->base_info->angle_sum = 0;
		}
		
	}
	
	
	#if 0
	/*防止双头同时打蛋*/
	if(R_shoot_speed != 0 && L_shoot_speed != 0 && elapsed_seconds<= SIMULATAN_TIME && Simultaneous_fire != 1)//双头都在打蛋 并且 时间特别近
	{
		Simultaneous_fire = 1;//防止连续减
		R_shoot_speed += 200;//减速
	}
	
	if(elapsed_seconds >= SIMULATAN_TIME)
	{
		Simultaneous_fire = 0;//未发生同时射击
	}
	#endif
	
	/*保证小于0*/
	if(R_shoot_speed >= 0)
	{
		R_shoot_speed = 0;
	}
	if(L_shoot_speed >= 0)
	{
		L_shoot_speed = 0;
	}
	
	/*掉线检测*/
	Shoot_Heart(&L_shoot_structure);
	Shoot_Heart(&R_shoot_structure);
	
	/*堵转检测*/
	Done_Check(&L_shoot_structure);
	Done_Check(&R_shoot_structure);
	

	/*左拨蛋轮*/
	/*连发模式*/
	if(L_shoot_structure.status == Running_Shoot)
	{
		if(L_shoot_structure.flag.locked == 0)
		{
			if(judge.base_info->shooter_id2_cooling_heat >= judge.data->game_robot_status.shooter_id2_17mm_cooling_limit- 50)
			{
				L_shoot_structure.base.shoot_speed = 0;
			}
			else
			{
				L_shoot_structure.base.shoot_speed = -700;
			}
			/*连发测试*/
			Running_Fire(&L_shoot_structure);

			L_shoot_structure.cnt.deal_time = 0;
		}
		else
		{
			DONE_WORK(&L_shoot_structure);
		}
	}
	/*单发模式*/
	else if (L_shoot_structure.status == Single_Shoot)
	{
		

		if(L_shoot_structure.flag.locked == 0)
		{
			//Car.C中处理
			//L_shoot_structure.stir_wheel->base_info->target_angle_sum += 36864;
			
			/*角度控制*/
			Single_Fire(&L_shoot_structure);
			
			L_shoot_structure.cnt.deal_time = 0;
		}
		else
		{
			DONE_WORK(&L_shoot_structure);
		}
	}
	/*视觉控制*/
	else if(L_shoot_structure.status == Visin_Shoot )
	{

		
		if(L_shoot_structure.flag.locked == 0)
		{
			#if 0
			/*数据预处理*/
			L_shoot_structure.base.last_shoot_speed = L_shoot_structure.base.shoot_speed;
			L_shoot_structure.base.shoot_speed = L_shoot_speed;
			
			Running_Fire(&L_shoot_structure);
			#endif
			
			#if 1
			
			/*角度控制*/
			Single_Fire(&L_shoot_structure);
			
			#endif
			
			/*堵转清零*/
			L_shoot_structure.cnt.deal_time = 0;
		}
		else
		{
			DONE_WORK(&L_shoot_structure);
		}
	}
	/*发射机构关闭模式*/
	else
	{	
		/*停止转动*/
		L_shoot_structure.base.shoot_speed = 0;
		Running_Fire(&L_shoot_structure);
		
		/*角度和清零*/
		L_shoot_structure.stir_wheel->base_info->target_angle_sum = 0;
		L_shoot_structure.stir_wheel->base_info->angle_sum = 0;
		/*堵转标志位清零*/
		L_shoot_structure.cnt.done_time = 0;
		L_shoot_structure.flag.locked = 0;
	}
		
		

	/*右拨蛋轮*/
	/*连发模式*/
	if(R_shoot_structure.status == Running_Shoot)
	{
		if(R_shoot_structure.flag.locked == 0)
		{
			if(judge.base_info->shooter_id1_cooling_heat >= judge.data->game_robot_status.shooter_id1_17mm_cooling_limit- 50)
			{
				R_shoot_structure.base.shoot_speed = 0;
			}
			else
			{
				R_shoot_structure.base.shoot_speed = -700;
			}

			/*连发测试*/
			Running_Fire(&R_shoot_structure);

			R_shoot_structure.cnt.deal_time = 0;
		}
		else
		{
			DONE_WORK(&R_shoot_structure);
		}
	}
	/*单发模式*/
	else if (R_shoot_structure.status == Single_Shoot)
	{
		
		if(R_shoot_structure.flag.locked == 0)
		{
			//Car.C中处理
			//R_shoot_structure.stir_wheel->base_info->target_angle_sum += 36864;
			/*角度控制*/
			Single_Fire(&R_shoot_structure);
			
			R_shoot_structure.cnt.deal_time = 0;
		}
		else
		{
			DONE_WORK(&R_shoot_structure);
		}
	}
	/*视觉模式*/
	else if(R_shoot_structure.status == Visin_Shoot)
	{
		
		
		if(R_shoot_structure.flag.locked == 0)
		{		
			#if 0
			/*数据预处理*/
			R_shoot_structure.base.last_shoot_speed = R_shoot_structure.base.shoot_speed;
			R_shoot_structure.base.shoot_speed = R_shoot_speed;
			
			Running_Fire(&R_shoot_structure);
			#endif
			
			#if 1
			
			/*角度控制*/
			Single_Fire(&R_shoot_structure);
			#endif
			
			/*堵转清零*/
			R_shoot_structure.cnt.deal_time = 0;
		}
		else
		{
			DONE_WORK(&R_shoot_structure);
		}
	}
	/*发射机构关闭模式*/
	else
	{
		/*停止射击*/
		R_shoot_structure.base.shoot_speed = 0;
		Running_Fire(&R_shoot_structure);
		/*角度和清零*/
		R_shoot_structure.stir_wheel->base_info->target_angle_sum = 0;
		R_shoot_structure.stir_wheel->base_info->angle_sum = 0;
		/*堵转标志位清零*/
		R_shoot_structure.cnt.done_time = 0;
		R_shoot_structure.flag.locked = 0;
	}

	
	/*防止堵转处理导致电机过热，防止单次堵转次数过高*/
	/*警钟长鸣！！！！！！！！！！！！！！！！！！！！*/
	if(L_shoot_structure.cnt.deal_done_cnt >= DEAL_TIME_MAX)
	{
		L_shoot_structure.cnt.deal_done_cnt = DEAL_TIME_MAX;
		L_shoot_structure.stir_wheel->output_current = 0;
	}
	
	if(R_shoot_structure.cnt.deal_done_cnt >= DEAL_TIME_MAX)
	{
		R_shoot_structure.cnt.deal_done_cnt = DEAL_TIME_MAX;
		R_shoot_structure.stir_wheel->output_current = 0;
	}
	/*警钟长鸣！！！！！！！！！！！！！！！！！！！！*/
	
	/*DJI展览模式*/
	//L_shoot_structure.status = Stop_Shoot;
	//R_shoot_structure.status = Stop_Shoot;
	/*摩擦轮状态更新*/
	if(L_shoot_structure.status == Visin_Shoot || \
	   L_shoot_structure.status == Single_Shoot || \
	   L_shoot_structure.status == Running_Shoot)
	{
		Master_Head_structure.Send_L_Head.shoot_mode = 1 + shoot_2_over_speed_flag;
	}
	else
	{
		Master_Head_structure.Send_L_Head.shoot_mode = 0;
	}
	
	if(R_shoot_structure.status == Visin_Shoot || \
	   R_shoot_structure.status == Single_Shoot || \
	   R_shoot_structure.status == Running_Shoot)
	{
		Master_Head_structure.Send_R_Head.shoot_mode = 1 + shoot_1_over_speed_flag;
	}
	else
	{
		Master_Head_structure.Send_R_Head.shoot_mode = 0;
	}
	/*电机数据发送（单个电机掉线单独清零，不能一个掉线另一个也不发射）*/
	
	/*掉线保护*/
	
	
	if(rc_structure.info->status == REMOTE_OFFLINE)
	{
		int16_t L_current = L_shoot_structure.stir_wheel->output_current;
		int16_t R_current = R_shoot_structure.stir_wheel->output_current;
		
		if(L_shoot_structure.stir_wheel->info->status  == _DEV_OFFLINE)
		{
			/*状态更新*/
			L_current = 0;
			L_shoot_structure.base.status = Shoot_Offline;
		}
			
		if(R_shoot_structure.stir_wheel->info->status  == _DEV_OFFLINE)
		{
			/*状态更新*/
			R_current = 0;
			R_shoot_structure.base.status = Shoot_Offline;
		}
		
		/*卸力*/
		L_current = 0;
		R_current = 0;
		
		/*卸力*/
		Motor_3508_Can2SentData(L_current,R_current,0,0);
			
	}
	/*正常工作*/
	else 
	{
		/*DJI展览模式*/
		//L_shoot_structure.stir_wheel->output_current = 0;
		//R_shoot_structure.stir_wheel->output_current = 0;
		
		
		if(Master_Head_structure.L_Head_status.status != M2H_ONLINE)
		{
			L_shoot_structure.stir_wheel->output_current = 0;
		}
		if(Master_Head_structure.R_Head_status.status != M2H_ONLINE)
		{
			R_shoot_structure.stir_wheel->output_current = 0;
		}
		/*数据发送*/
		Motor_3508_Can2SentData(L_shoot_structure.stir_wheel->output_current,\
								R_shoot_structure.stir_wheel->output_current,\
								  0,\
								  0);
		/*状态更新*/
		L_shoot_structure.base.status = Shoot_Online;
		R_shoot_structure.base.status = Shoot_Online;
	}
	
	
	

}	


