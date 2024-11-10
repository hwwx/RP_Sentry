/**
 * @file        drv_tim.c
 * @author      RobotPilots@2024
 * @Version     V1.0
 * @date        23-August-2020
 * @brief       TIMER Driver Package(Based on HAL).
 */

/* Includes ------------------------------------------------------------------*/
#include "drv_tim.h"
#include "hrtim.h"

extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim2;

/* Private macro -------------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
/* Private functions ---------------------------------------------------------*/


/* Exported functions --------------------------------------------------------*/
void PWM_Init(void)
{


}
/*��main.c�Ķ�ʱ���ж��е���*/
void TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance == TIM2)
	{
		
	}

}
void PWM_output(int16_t OutDutyCycle)
{
	int32_t PWM_PER_0_5	= 0.5f * hhrtim1.Instance->sTimerxRegs[0].PERxR;
   uint32_t	buck_duty,boost_duty;
   
   if(OutDutyCycle > 0.9f)  //����ռ�ձȴ���90%ʱ������BOOSTģʽ��
   {
      boost_duty = (OutDutyCycle - 0.8f) * PWM_PER_0_5;//����boost��ռ�ձȣ�����ģ�����ĶԳƵ�PWM��ע��Ҫ��ȥbuck_duty��ռ�ձ�: 0.8*PWM_PER_0_5
   }
	else
   {
		boost_duty = 0.1f * PWM_PER_0_5;	//����ռ�ձȲ�����90%ʱ������BUCKģʽ,boost_duty���̶�ռ�ձ�
   }

	if(OutDutyCycle > 0.9f)
   {
      buck_duty = 0.9f * PWM_PER_0_5; //����ռ�ձȴ���90%ʱ������BOOSTģʽ,buck_duty���̶�ռ�ձ�
   }
	else
   {
		buck_duty = OutDutyCycle * PWM_PER_0_5;	//����buck��ռ�ձȣ�����ģ�����ĶԳƵ�PWM��
   }

   hhrtim1.Instance->sTimerxRegs[0].CMP1xR = PWM_PER_0_5 - buck_duty;//TimerA PWM??1?????????????????PWM????PWM_PER_0_5?????
	  
   hhrtim1.Instance->sTimerxRegs[0].CMP2xR = PWM_PER_0_5 + buck_duty;//TimerA PWM??0???????????  ??400-50????400+50??????????400	

   hhrtim1.Instance->sTimerxRegs[3].CMP1xR = PWM_PER_0_5 + boost_duty;//TimerD PWM??1?????????????????PWM????PWM_PER_0_5?????
	  
   hhrtim1.Instance->sTimerxRegs[3].CMP2xR = PWM_PER_0_5 - boost_duty;//TimerD PWM??0???????????  ??400-50????400+50??????????400	
	  
}

void PWM_Off(void)
{
   HAL_HRTIM_WaveformOutputStop(&hhrtim1, HRTIM_OUTPUT_TA1|HRTIM_OUTPUT_TA2|HRTIM_OUTPUT_TD1|HRTIM_OUTPUT_TD2); //ͨ���ر�
}


