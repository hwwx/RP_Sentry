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
extern pid_t                         L_pid_p_stir_sin; /*�����ٶȻ�*/
/*�����ٴ���*/
extern uint8_t                       shoot_1_over_speed_flag;
extern uint8_t                       shoot_2_over_speed_flag;

/*��ֹͬʱ��*/
extern float                         elapsed_seconds;

/* Private macro -------------------------------------------------------------*/

#define AERIAL_SHOOT      (1)
#define AERIAL_STOP_SHOOT (0)
#define SIMULATAN_TIME    (0.0005f)
/* Private variables ---------------------------------------------------------*/

/*ֹͣת��������*/
uint16_t no_shoot_time = 0;

float   L_shoot_speed       = 0;
float   R_shoot_speed       = 0;

/*�Ӿ�������*/
uint8_t L_shoot_cnt         = 0;
uint8_t R_shoot_cnt         = 0;
/*�Ӿ�������-�ϴ�*/
uint8_t L_shoot_cnt_last    = 0;
uint8_t R_shoot_cnt_last    = 0;

uint8_t shoot_aerial_cmd     = AERIAL_SHOOT;
uint8_t Simultaneous_fire    = 0;


void shoot_work(shoot_t* shoot)
{

	L_shoot_structure.status = Running_Shoot;
	R_shoot_structure.status = Running_Shoot;
	
					

	
	/*�Ӿ����ٴ���*/
	L_shoot_speed     = -((vision_structure.rx_pack->RxData.L_shoot_speed/10.0)/8.0)*60*36;
	L_shoot_cnt_last  =    L_shoot_cnt;
	L_shoot_cnt       =    vision_structure.rx_pack->RxData.L_shoot_cnt;
	
	R_shoot_speed     = -((vision_structure.rx_pack->RxData.R_shoot_speed/10.0)/8.0)*60*36;
	R_shoot_cnt_last  =    R_shoot_cnt;
	R_shoot_cnt       =    vision_structure.rx_pack->RxData.R_shoot_cnt;
	
	/*�Զ���*/
	if(L_shoot_structure.status == Visin_Shoot)
	{
		int32_t L_delta_angle = abs(L_shoot_structure.stir_wheel->base_info->target_angle_sum - L_shoot_structure.stir_wheel->base_info->angle_sum);
		
		/*�ۼ�δ����������3��*/
		if( L_shoot_cnt != L_shoot_cnt_last && L_delta_angle <= 36864 * 3)
		{
			L_shoot_structure.stir_wheel->base_info->target_angle_sum -= 36864;
		}
		
		/*���ƶ�ʧĿ��֮���ڴ�*/
		if(vision_structure.rx_pack->RxData.L_is_find_target != 1)
		{
			/*�ٶ�����*/
			L_shoot_speed = 0;
			/*�ǶȺ�����*/
			L_shoot_structure.stir_wheel->base_info->target_angle_sum = 0;	
			L_shoot_structure.stir_wheel->base_info->angle_sum = 0;
		}
		 
		/*����λ�û�����ٶȣ��൱��������Ƶ*/
		L_shoot_structure.pid_p_stir_sin->MaxOutput = (-1)*L_shoot_speed;
	}

	if(R_shoot_structure.status == Visin_Shoot)
	{
		int32_t R_delta_angle = abs(R_shoot_structure.stir_wheel->base_info->target_angle_sum - R_shoot_structure.stir_wheel->base_info->angle_sum);
		
		/*�ۼ�δ����������3��*/
		if( R_shoot_cnt != R_shoot_cnt_last && R_delta_angle <= 36864 * 3)
		{
			R_shoot_structure.stir_wheel->base_info->target_angle_sum -= 36864;
		}
		
		if(vision_structure.rx_pack->RxData.R_is_find_target != 1 )
		{
			/*�ٶ�����*/
			R_shoot_speed = 0;
			/*�ǶȺ�����*/
			R_shoot_structure.stir_wheel->base_info->target_angle_sum = 0;
			R_shoot_structure.stir_wheel->base_info->angle_sum = 0;
		}
		
		/*����λ�û�������٣��൱����������*/
		R_shoot_structure.pid_p_stir_sin->MaxOutput = (-1)*R_shoot_speed;
	}

	
	/*ǹ����������*/
	/*˫ǹδ��������ֻ��һ��ǹ������*/
	 judge.data->game_robot_status.shooter_id2_17mm_cooling_limit = 240;
	 judge.data->game_robot_status.shooter_id1_17mm_cooling_limit = 240;
	

	
	#if 0
	/*��ֹ˫ͷͬʱ��*/
	if(R_shoot_speed != 0 && L_shoot_speed != 0 && elapsed_seconds<= SIMULATAN_TIME && Simultaneous_fire != 1)//˫ͷ���ڴ� ���� ʱ���ر��
	{
		Simultaneous_fire = 1;//��ֹ������
		R_shoot_speed += 200;//����
	}
	
	if(elapsed_seconds >= SIMULATAN_TIME)
	{
		Simultaneous_fire = 0;//δ����ͬʱ���
	}
	#endif
	
	
	
	/*��ת���*/
	Done_Check(&L_shoot_structure);
	Done_Check(&R_shoot_structure);
	

	/*�󲦵���*/
	/*����ģʽ*/
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
				L_shoot_structure.base.shoot_speed = 6750;
			}
			/*��������*/
			Running_Fire(&L_shoot_structure);

			L_shoot_structure.cnt.deal_time = 0;
		}
		else
		{
			DONE_WORK(&L_shoot_structure);
		}
	}
	/*����ģʽ*/
	else if (L_shoot_structure.status == Single_Shoot)
	{
		

		if(L_shoot_structure.flag.locked == 0)
		{
			//Car.C�д���
			//L_shoot_structure.stir_wheel->base_info->target_angle_sum += 36864;
			
			/*�Ƕȿ���*/
			Single_Fire(&L_shoot_structure);
			
			L_shoot_structure.cnt.deal_time = 0;
		}
		else
		{
			DONE_WORK(&L_shoot_structure);
		}
	}
	/*�Ӿ�����*/
	else if(L_shoot_structure.status == Visin_Shoot )
	{

		
		if(L_shoot_structure.flag.locked == 0)
		{
			#if 0
			/*����Ԥ����*/
			L_shoot_structure.base.last_shoot_speed = L_shoot_structure.base.shoot_speed;
			L_shoot_structure.base.shoot_speed = L_shoot_speed;
			
			Running_Fire(&L_shoot_structure);
			#endif
			
			#if 1
			
			/*�Ƕȿ���*/
			Single_Fire(&L_shoot_structure);
			
			#endif
			
			/*��ת����*/
			L_shoot_structure.cnt.deal_time = 0;
		}
		else
		{
			DONE_WORK(&L_shoot_structure);
		}
	}
	/*��������ر�ģʽ*/
	else
	{	
		/*ֹͣת��*/
		L_shoot_structure.base.shoot_speed = 0;
		Running_Fire(&L_shoot_structure);
		
		/*�ǶȺ�����*/
		L_shoot_structure.stir_wheel->base_info->target_angle_sum = 0;
		L_shoot_structure.stir_wheel->base_info->angle_sum = 0;
		/*��ת��־λ����*/
		L_shoot_structure.cnt.done_time = 0;
		L_shoot_structure.flag.locked = 0;
	}
		
		

	/*�Ҳ�����*/
	/*����ģʽ*/
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
				R_shoot_structure.base.shoot_speed = 6750;
			}

			/*��������*/
			Running_Fire(&R_shoot_structure);

			R_shoot_structure.cnt.deal_time = 0;
		}
		else
		{
			DONE_WORK(&R_shoot_structure);
		}
	}
	/*����ģʽ*/
	else if (R_shoot_structure.status == Single_Shoot)
	{
		
		if(R_shoot_structure.flag.locked == 0)
		{
			//Car.C�д���
			//R_shoot_structure.stir_wheel->base_info->target_angle_sum += 36864;
			/*�Ƕȿ���*/
			Single_Fire(&R_shoot_structure);
			
			R_shoot_structure.cnt.deal_time = 0;
		}
		else
		{
			DONE_WORK(&R_shoot_structure);
		}
	}
	/*�Ӿ�ģʽ*/
	else if(R_shoot_structure.status == Visin_Shoot)
	{
		
		
		if(R_shoot_structure.flag.locked == 0)
		{		
			#if 0
			/*����Ԥ����*/
			R_shoot_structure.base.last_shoot_speed = R_shoot_structure.base.shoot_speed;
			R_shoot_structure.base.shoot_speed = R_shoot_speed;
			
			Running_Fire(&R_shoot_structure);
			#endif
			
			#if 1
			
			/*�Ƕȿ���*/
			Single_Fire(&R_shoot_structure);
			#endif
			
			/*��ת����*/
			R_shoot_structure.cnt.deal_time = 0;
		}
		else
		{
			DONE_WORK(&R_shoot_structure);
		}
	}
	/*��������ر�ģʽ*/
	else
	{
		/*ֹͣ���*/
		R_shoot_structure.base.shoot_speed = 0;
		Running_Fire(&R_shoot_structure);
		/*�ǶȺ�����*/
		R_shoot_structure.stir_wheel->base_info->target_angle_sum = 0;
		R_shoot_structure.stir_wheel->base_info->angle_sum = 0;
		/*��ת��־λ����*/
		R_shoot_structure.cnt.done_time = 0;
		R_shoot_structure.flag.locked = 0;
	}

	
	/*��ֹ��ת�����µ�����ȣ���ֹ���ζ�ת��������*/
	/*���ӳ�������������������������������������������*/
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
	/*���ӳ�������������������������������������������*/
	
	/*Ħ����״̬����*/
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
	/*������ݷ��ͣ�����������ߵ������㣬����һ��������һ��Ҳ�����䣩*/
	
	/*���߱���*/
	
	
	/*���ݷ���*/
	Motor_3508_Can2SentData(L_shoot_structure.stir_wheel->output_current,\
							  R_shoot_structure.stir_wheel->output_current,\
							  0,\
							  0);
	/*״̬����*/
	L_shoot_structure.base.status = Shoot_Online;
	R_shoot_structure.base.status = Shoot_Online;
	
	
	

}	


