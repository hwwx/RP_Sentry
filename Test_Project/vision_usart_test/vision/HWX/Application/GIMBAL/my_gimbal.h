#ifndef __MY_GIMBAL_H
#define __MY_GIMBAL_H



#include "stm32F4xx_hal.h"
#include "6020_motor.h"


#define HALF_CIRLE         (4096)
#define HALF_HALF_CIRLE    (1024)
#define YAW_MIDDLE_FOUNT   (2100)
#define YAW_MIDDLE_BACK    ((2100) + HALF_CIRLE)
#define PIT_MIDDLE         (6550)
#define PIT_RANGE          (750)
#define PIT_M_UP_RANGE     (PIT_MIDDLE - PIT_RANGE)
#define PIT_M_DOWN_RANGE   (PIT_MIDDLE + PIT_RANGE)
#define PIT_B_UP_RANGE     (HALF_CIRLE + PIT_RANGE)
#define PIT_B_DOWN_RANGE   (HALF_CIRLE - PIT_RANGE)
#define MOTOR_GAP_BMI      (PIT_MIDDLE - HALF_CIRLE)

/*��Ҫ��PITCH�ı任*/
#define motor2bmi(m,b)     (b=(m)-MOTOR_GAP_BMI)
#define bmi2motor(b,m)     (m=(b)+MOTOR_GAP_BMI)

#define sgn(x)             (((x)>0)?1:((x)<0?-1:0))

/****************************************����ģʽö��****************************************/

/* ��̨ģ�� */
typedef enum 
{
	G_G_offline,  //ʧ��
	G_G_init,     //��ʼ��
	G_G_follow,   //����
	G_G_shoot,    //����
	G_G_check,    //��鲹��
	G_G_lock,
	G_G_spin,
	G_G_vision,
}gimbal_work_status_e;

/* ��̨pitch���� */
typedef enum 
{
	G_P_offline,  //ʧ��
	G_P_stop,     //ֹͣ
	G_P_speed,    //�ٶȻ�
	G_P_lock,     //�ٶȻ���ת
	G_P_angle,    //�ǶȻ�
	G_P_done,     //�ǶȻ���תor��λ
	G_P_normal,   //����
	G_P_slow,     //���٣�����ʱ��
	G_P_vision,   //�Ӿ�����
}s_pitch_work_status_e;

/* ��̨yaw���� */
typedef enum 
{
	G_Y_offline,  //ʧ��
	G_Y_stop,     //ֹͣ
	G_Y_angle,    //�ǶȻ�
	G_Y_done,     //�ǶȻ���תor��λ
	G_Y_normal,   //����
	G_Y_slow,     //���٣�����ʱ��
	G_Y_lock,
	G_Y_spin,
	G_Y_vision,   //�Ӿ�����
}s_yaw_work_status_e;

/* ��̨��Զ����� */
typedef enum 
{
	G_W_offline,  //ʧ��
	G_W_stop,     //ֹͣ
	G_W_speed,    //�ٶȻ�
	G_W_lock,     //�ٶȻ���ת
	G_W_angle,    //�ǶȻ�
	G_W_done,     //�ǶȻ���תor��λ
	G_W_retract,  //�վ�
	G_W_open,     //����
}watch_work_status_e;

/****************************************������Ϣ�ṹ��****************************************/

/* ��̨ģ�� */
typedef struct
{
	uint8_t command;
	uint8_t status;
	uint8_t init_Y_O_N;
	uint8_t work_status;
}gimbal_work_info_t;

/* ��̨pitch���� */
typedef struct
{
	uint8_t status;
	int16_t cnt;
}s_pitch_work_info_t;

/* ��̨yaw���� */
typedef struct
{
	uint8_t status;
	int16_t cnt;
}s_yaw_work_info_t;

/* ��̨��Զ����� */
typedef struct
{
	uint8_t status;
	int16_t cnt;
}watch_work_info_t;

/****************************************ģ��ṹ��****************************************/

/* ��̨���� */
typedef struct 
{
	int16_t watch_init_out_max;     //��Զ����ʼ��ʱ������ֵ�������ٶȻ���ת��������
	int16_t watch_normal_out_max;   //��Զ������ʱ������ֵ
	int16_t watch_init_speed;       //��Զ����ʼ���ٶ�
	int32_t watch_up_angle;         //��Զ���϶˽Ƕ�
	int16_t pitch_init_speed;       //pitch���ʼ���ٶ�
	int16_t pitch_up_angle;         //pitch���϶˽Ƕ�
	int16_t yaw_check_angle;        //yaw����鲹���Ƕ�
	int16_t pitch_check_angle;      //pitch����鲹���Ƕ�
	int16_t yaw_middle_angle;       //yaw��Ƕ���ֵ
	int16_t lock_cnt_max;           //��ת��������
	int16_t pitch_angle_permit_max; //pitch��Ƕ���������
	int16_t pitch_angle_permit_min; //pitch��Ƕ���������
}gimbal_config_t;

/* ��̨��Ϣ */
typedef struct
{
	int16_t  pitch_speed;        //ƫ�����ٶ�
	int16_t  target_pitch_speed; //Ŀ��ƫ�����ٶ�
	int32_t  pitch_angle;        //ƫ����
	int32_t  target_pitch_angle; //Ŀ��ƫ����
	int16_t  yaw_speed;          //
	int16_t  target_yaw_speed;   //
	int16_t  yaw_angle;          //
	int16_t  target_yaw_angle;   //
	int16_t  watch_speed;        //
	int16_t  target_watch_speed; 
	int32_t  watch_angle;
	int32_t  target_watch_angle;
	int16_t  return_yaw_angle;
	int32_t  return_pitch_angle;
	uint8_t  shoot_watch_flag;
}gimbal_info_t;

/* ��̨ */
typedef struct 
{
	motor_6020_t    *pitch;         //������
	motor_6020_t    *yaw;           //ƫ����
	motor_6020_t    *watch;         //������ ��ʱ��6020����
	
	gimbal_work_info_t *work;       //��̨ģ��
	s_pitch_work_info_t *pitch_work;//��̨pitch����
	s_yaw_work_info_t *yaw_work;    //��̨yaw����
  watch_work_info_t *watch_work;
	
	gimbal_config_t *config;        //��̨����
	gimbal_info_t   *info;          //��̨��Ϣ
	
	int8_t           front_or_back;	//��ͷ��־λ,1������ͷ��-1�Ƿ���ͷ
	bool             angle_45;
}gimbal_t;

/****************************************�ⲿ����****************************************/
extern gimbal_t gimbal_structure;

/****************************************����****************************************/

/* ��ʼ�� */
void gimbal_init(gimbal_t *gimbal);
void gimbal_info_init(gimbal_info_t *info);
void gimbal_commond_init(gimbal_t *gimbal);

/* �����빤������ */
void gimbal_ctrl_task(gimbal_t *gimbal);
void gimbal_mode_update(gimbal_t *gimbal);
void gimbal_commond_respond(gimbal_t *gimbal);
void gimbal_work(gimbal_t *gimbal);
void gimbal_pitch_work(gimbal_t *gimbal);
void gimbal_yaw_work(gimbal_t *gimbal);
void gimbal_yaw_angle_lock_ctrl(gimbal_t *gimbal);
void gimbal_yaw_lock_speed_ctrl(gimbal_t *gimbal);
//void gimbal_watch_work(gimbal_t *gimbal);

/* can���� */
void gimbal_yaw_can_update(gimbal_t *gimbal);
void gimbal_pitch_can_update(gimbal_t *gimbal);
void gimbal_watch_can_update(gimbal_t *gimbal);

/* ��� */
void gimbal_pitch_angle_check(gimbal_t *gimbal);
void gimbal_yaw_angle_check(gimbal_t *gimbal);

/* ������ */
void gimbal_yaw_speed_ctrl(gimbal_t *gimbal);
void gimbal_yaw_angle_ctrl(gimbal_t *gimbal);
void gimbal_pitch_speed_ctrl(gimbal_t *gimbal);
void gimbal_pitch_angle_ctrl(gimbal_t *gimbal);
void gimbal_watch_speed_ctrl(gimbal_t *gimbal);
void gimbal_watch_angle_ctrl(gimbal_t *gimbal);

#endif
