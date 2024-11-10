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
#include "rp_gimbal.h"
#include "remote.h"
#include "Car.h"
#include "vision.h"
#include "bmi.h"
#include "judge.h" 

/* Private function prototypes -----------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
extern bmi_t                  bmi_structure;
extern gimbal_t               Gimbal;
extern rc_t                   rc_structure;
extern car_t                  car_structure;
extern motor_9015_t           motor_9015_structure;
extern vision_t               vision_structure;
extern judge_t                judge;

/* Private macro -------------------------------------------------------------*/
uint16_t                      first2vision   = 0;
uint16_t                      first2follow   = 0;
uint16_t                      first2spin     = 0;
uint16_t                      first2machine  = 0;
uint16_t                      last_encoder   = 0;
uint32_t                      motor_9015_offline_cnt = 0;
uint32_t                      spin_time      = 0;
uint32_t                      spin_time_L    = 0;
float                         spin_angular   = 0;

/* Function  body --------------------------------------------------------*/
void gimbal_work()
{
	/*������߼��*/
	Motor_9015_HeartBeat();
	
	/*��̨���߼��*/
	if (Gimbal.Yaw_9015->info->status == _9015_ONLINE)
	{
		Gimbal.staus.work_sate = GIMBAL_ONLINE;
	}
	else
	{
		Gimbal.staus.work_sate = GIMBAL_OFFLINE;
	}
	
	
	/*ģʽ�л�*/
	if(car_structure.ctrl == RC_CAR)
	{
		if(car_structure.mode == machine_CAR  )//��еģʽ
		{
			if(first2machine == 0)
			{
				Gimbal.target_yaw_angle = Gimbal.Yaw_9015->base_info->encoder;
			}
			first2machine = 1;
			if(car_structure.mode == machine_CAR)
			{
				Gimbal.target_yaw_angle = MOROT_9015_MIDDLE;   
			}
			
			Gimbal_BigYawAngleCheck(&Gimbal);
			Gimbal_BigYawPosition(&Gimbal);

			
		}
		else if(car_structure.mode == follow_CAR  || car_structure.mode == two_CAR)//����ģʽ
		{
			if(car_structure.mode == follow_CAR)
			{
				Gimbal.target_yaw_angle += (int32_t)(rc_structure.base_info->ch0*(-0.09));
			}
			
			if(first2follow == 0 )
			{
				Gimbal.target_yaw_angle = bmi_structure.yaw_angle * 8;
			}
			first2follow = 1;
			
			Gimbal_BigYawAngleCheck(&Gimbal);
			Gimbal_BigYawBmiPosition(&Gimbal);
		}
		else if(car_structure.mode == vision_CAR ||car_structure.mode == aim_CAR || car_structure.mode ==  shake_CAR)//�Ӿ�ģʽ
		{
			/*��������*/
			#if 1
			if(vision_structure.state.work_state == VISION_OFFLINE)
			{
				if(first2vision == 0)
				{
					Gimbal.target_yaw_angle = bmi_structure.yaw_angle;
				}
				first2vision = 1;
				motor_9015_structure.base_info->target_angle = Gimbal.target_yaw_angle;
				Motor_9015_VisionBmiPosition();
			}
			else
			{
				if(first2vision == 0)
				{
					Gimbal.target_yaw_angle = bmi_structure.yaw_angle;
				}
				first2vision = 1;
				Gimbal.target_yaw_angle = vision_structure.rx_pack->RxData.B_yaw_angle;
				motor_9015_structure.base_info->target_angle = Gimbal.target_yaw_angle;
				Motor_9015_VisionBmiPosition();//???
			}
			#endif
			
			/*���Դ���*/
			#if 0
			if(first2vision == 0)
			{
				Gimbal.target_yaw_angle = Gimbal.Yaw_9015->base_info->encoder;
			}
			first2vision = 1;
			Gimbal_BigYawPosition(&Gimbal);
			
			#endif

		}
		else if(car_structure.mode == spin_CAR ||car_structure.mode == navigation_CAR || car_structure.mode == patrol_CAR)//С����ģʽ
		{
			#if 1
			/*��������*/
				if(first2spin == 0 )
				{
					Gimbal.target_yaw_angle = bmi_structure.yaw_angle * 8;
				}
				first2spin = 1;
				
				Gimbal.target_yaw_angle = Hurt_And_Find();
				
				Gimbal.target_yaw_angle += (int32_t)(rc_structure.base_info->ch0*(-0.09));
				
				
				Gimbal_BigYawAngleCheck(&Gimbal);
				Gimbal_BigYawBmiPosition(&Gimbal);
				
			
			#endif
			
			#if 0
			
			Gimbal.target_yaw_angle = MOROT_9015_MIDDLE;   
			Gimbal_BigYawAngleCheck(&Gimbal);
			Gimbal_BigYawPosition(&Gimbal);
			
			#endif
			
		}
		else if(car_structure.mode == offline_CAR)//����ģʽ
		{
			Motor_9015_Off();
		}
		
		/*��־λ����*/
		if(car_structure.mode != follow_CAR &&  car_structure.mode != two_CAR)
		{
			first2follow = 0;
		}
		if(car_structure.mode != spin_CAR && car_structure.mode != navigation_CAR && car_structure.mode != patrol_CAR)
		{
			first2spin = 0;
		}
		if(car_structure.mode != vision_CAR && car_structure.mode != shake_CAR && car_structure.mode != aim_CAR)
		{
			first2vision = 0;
		}
		if(car_structure.mode != machine_CAR )
		{
			first2machine = 0;
		}
		
	}
	else if(car_structure.ctrl == MINIPC_CAR)
	{
		
	}
	else if(car_structure.ctrl == TEST_CAR)
	{

		
	}
	

}

/*����Ѱ��*/
int16_t Hurt_And_Find()
{
	int16_t target_angle = Gimbal.target_yaw_angle;
	
	/*δѰ������*/
	if(vision_structure.rx_pack->RxData.L_is_find_target == 0 && vision_structure.rx_pack->RxData.R_is_find_target == 0)
	{
		if(judge.base_info->armor_id == 0)
		{
			target_angle = MOTOR_9015_TO_BMI(&Gimbal,AMMO_1_ANGLE);
		}
		else if(judge.base_info->armor_id == 1)
		{
			target_angle = MOTOR_9015_TO_BMI(&Gimbal,AMMO_2_ANGLE);
		}
		else if(judge.base_info->armor_id == 2)
		{
			target_angle = MOTOR_9015_TO_BMI(&Gimbal,AMMO_3_ANGLE);
		}
		else if(judge.base_info->armor_id == 3)
		{
			target_angle = MOTOR_9015_TO_BMI(&Gimbal,AMMO_4_ANGLE);
		}
		
		/*�����ʾ�������*/
		judge.base_info->armor_id = 6;

	}
	
	return target_angle;
}