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

#ifndef __9015_MOTOR_H
#define __9015_MOTOR_H

 /* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "drv_can.h"
#include "can.h"
#include "dji_pid.h"
/* Private macro -------------------------------------------------------------*/
#define MOTOR_9015_SENT_ID 0x141
#define MOTOR_9015_CAN     (hcan1)
#define _9015_OFFLINE        0
#define _9015_ONLINE         1
#define _9015_ERRO           2
#define OFFLINE_TIME_MAX     25

/*bmi轴速度环*/
#define S_PIT_PID_P (5.0f)		//1
#define S_PIT_PID_I 0.01
#define S_PIT_PID_D 0
#define S_PIT_LIM_I 0    //积分上限
#define S_PIT_LIM_O 2000   //输出上限

/*Bmi轴位置环*/
#define B_PIT_PID_P (4.5f)    //0.2
#define B_PIT_PID_I 0.001
#define B_PIT_PID_D 0
#define B_PIT_LIM_I 0      //积分上限
#define B_PIT_LIM_O 4000   //输出上限

/*Yaw轴位置环*/
#define P_PIT_PID_P (0.90f)    //0.2
#define P_PIT_PID_I 0.001
#define P_PIT_PID_D 1
#define P_PIT_LIM_I 0      //积分上限
#define P_PIT_LIM_O 4000   //输出上限

/*Yaw轴速度环*/
#define Y_PIT_PID_P (6.0)    //0.2
#define Y_PIT_PID_I 0.1
#define Y_PIT_PID_D 0
#define Y_PIT_LIM_I 500      //积分上限
#define Y_PIT_LIM_O 2000   //输出上限
/* Private function prototypes -----------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/*9015电机命令*/
typedef enum motor_kt9025_command_e
{
	PID_RX_ID				   = 0x30, 
	PID_TX_RAM_ID			   = 0x31,  //断电失效
	PID_TX_ROM_ID			   = 0x32,  //断电有效
	ACCEL_RX_ID				   = 0x33,
	ACCEL_TX_ID				   = 0x34,
	ENCODER_RX_ID		       = 0x90,
	ZERO_ENCODER_TX_ID		   = 0x91,	
	ZERO_POSNOW_TX_ID		   = 0x19,	
	
	MOTOR_ANGLE_ID			   = 0x92,
	CIRCLE_ANGLE_ID			   = 0x94,
	STATE1_ID				   = 0x9A,
	CLEAR_ERROR_State_ID	   = 0x9B,
	STATE2_ID				   = 0x9C,
	STATE3_ID				   = 0x9D,
	MOTOR_CLOSE_ID			   = 0x80,
	MOTOR_STOP_ID			   = 0x81,
	MOTOR_RUN_ID			   = 0x88,
	
	TORQUE_OPEN_LOOP_ID        = 0xA0,
	TORQUE_CLOSE_LOOP_ID       = 0xA1,
	SPEED_CLOSE_LOOP_ID        = 0XA2,
	POSI_CLOSE_LOOP_ID1        = 0XA3,   
	POSI_CLOSE_LOOP_ID2        = 0XA4,
	POSI_CLOSE_LOOP_ID3        = 0XA5,
	POSI_CLOSE_LOOP_ID4        = 0XA6,
	POSI_CLOSE_LOOP_ID5        = 0XA7,
	POSI_CLOSE_LOOP_ID6        = 0XA8,
	
	
}motor_kt9025_command_e;



/* 9015结构体*/
typedef struct
{
	uint32_t  angle;             //电机单圈角度   0.01°/LSB  （ 0~36000 * 减速比-1） 以编码零点作为起始点，顺时针增加，再次到达零点时数值变为0
	int16_t   speed;            //电机转速（RPM）（max = 7600）
	uint32_t  target_angle;     //目标角度，云台电机
	int32_t   target_speed;     //目标转速
	int16_t   target_iq;
	int8_t    temperature;
	int16_t   iq;
	uint16_t  encoder;           //0~65536
}motor_9015_base_info_t; 

typedef struct 
{
	uint8_t          offline_cnt;
	uint8_t          offline_cnt_max;
	uint8_t          status;
}motor_9015_info_t;

typedef struct KT_motor_pid_tx_info_t
{
	uint8_t angleKp;
	uint8_t angleKi;
	uint8_t speedKp;
	uint8_t speedKi;
	uint8_t iqKp;  //转矩环
	uint8_t iqKi; 
	
}motor_9015_pid_t;


typedef struct motor_9015_t
{
	motor_9015_base_info_t *base_info;
	motor_9015_info_t      *info;
	motor_9015_pid_t       *pid;
	uint8_t                 tx_buff[8];
	
}motor_9015_t;

/* Function  body --------------------------------------------------------*/
/*初始化*/
void Motor_9015_Init(void);

/*速度控制*/
void Motor_9015_Speed(void);

/*PID参数设置*/
void Motor_9015_SetPID(void);

/*掉线检测*/
void Motor_9015_HeartBeat(void);

/*主动读取数据*/
void Motor_9015_Data(uint8_t command);

/*接受中断函数*/
void Motor_9015_Updata(uint8_t *rxBuf);

/*电机关闭函数*/
void Motor_9015_Off(void);

/*电机开启函数*/
void Motor_9015_Star(void);



/*电机睡觉*/
void Motor_9015_Stop(void);
	
/*陀螺仪角度 0~8192*/
void Motor_9015_VisionBmiPosition(void);
/*陀螺仪角度 */
void Motor_9015_BmiPosition(void);


/*单圈位置控制*/
void Motor_9015_Position(void);

#endif
