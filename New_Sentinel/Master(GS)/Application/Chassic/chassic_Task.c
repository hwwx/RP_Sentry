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
#include "rp_chassis.h"
#include "remote.h"
#include "Car.h"
#include "vision.h"
#include "bmi.h"
#include "rp_gimbal.h"
#include "judge.h"
#include "judge_protocol.h"
#include "math.h"
#include "rp_math.h"
#include "navigation.h"

/* Private macro -------------------------------------------------------------*/
#define change_p              (1.0f)
#define pi                    (3.14159265354f)

#define UNDER_SMALL_HIT       (1)
#define UNDER_BIGGG_HIT       (2)
#define NO_HIT                (0)

#define AERIAL_W              (1)
#define AERIAL_A              (2)
#define AERIAL_S              (3)
#define AERIAL_D              (4)
#define AERIAL_NO             (5)
#define AERIAL_MOVE_TIME      (1700)
#define X_LIMIIT              (14.0f)


/* Private function prototypes -----------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/

/* Exported variables --------------------------------------------------------*/
extern chassis_t              Chassis; 
extern rc_t                   rc_structure;
extern car_t                  car_structure;
extern vision_t               vision_structure;
extern navigation_t           navigation_structure;
extern bmi_t                  bmi_structure;
extern gimbal_t               Gimbal;
extern judge_t                judge;
extern pid_t                  chassis_pid_follow_structure;
extern pid_t                  chassis_pid_speed_structure[4];
/* Private variables ---------------------------------------------------------*/
float                         cita        = 0;  

uint32_t                      sin_cnt     = 0;
uint32_t                      sin_tick    = 0; 
uint32_t                      cos_tick    = 0; 
uint32_t                      hurt_cnt    = 0;

uint16_t                      right_cnt   = 0;
uint16_t                      fornt_cnt   = 0;
uint16_t                      right_time  = 0;
uint16_t                      fornt_time  = 0;
uint16_t                      aerial_time = 0; //��̨��ָ������ʱ��

int32_t                       hurt_change = 0;
int32_t                       vision_spin_time = 0;

int16_t                       spin_speed  = 0;
int16_t                       vision_spin_speed = 0;

uint8_t                       game_star   = 0;
uint8_t                       position_ok = 0;
uint8_t                       hurt_mode   = 0;

uint8_t                       chassis_aerial_cmd = AERIAL_NO;

Max_Acc_Fliter                Front_Acc_Fliter;
Max_Acc_Fliter                Right_Acc_Fliter;
/* Exported macro ------------------------------------------------------------*/

/* Function  body --------------------------------------------------------*/

void chassis_cycle_speed_change(void)
{
	if(vision_spin_time >= 300000)
	{
		vision_spin_time = 0;
	}
	if(sin_tick >= 1300000)
	{
		sin_tick = 0;
	}
	/*ת�ٱ仯*/
	if(vision_spin_time++ %20000 <= 7000)
	{
		vision_spin_speed = 4000;
	}
	else if(vision_spin_time++ %30000 >= 14000)
	{
		vision_spin_speed = 4000 +  1000*sin( ((sin_tick)%1300)*2*3.14f );
	}
	else
	{
		vision_spin_speed = -4000;
	}
	
	
	/*����С����*/
	if(judge.base_info->game_progress == 4 && judge.base_info->friendly_outposts_HP == 0)
	{
		Chassis.base_info.target.cycle_speed = vision_spin_speed + 0*sin( ((sin_tick++)%1000)*2*3.14f ) + hurt_change;
	}
	else
	{
		/*�����жϣ�����Ѱ�ҵ����ˣ���ͣת*/
		if(vision_structure.rx_pack->RxData.L_is_find_target || vision_structure.rx_pack->RxData.R_is_find_target)
		{
			Chassis.base_info.target.cycle_speed = 0;
		}
		else
		{
			Chassis.base_info.target.cycle_speed = 300;
		}
		
	}

	/*״̬�ı�*/
	if(judge.base_info->last_HP - judge.base_info->remain_HP >= 9 && hurt_mode == NO_HIT)
	{
		hurt_mode = UNDER_SMALL_HIT;
	}
	else if(judge.base_info->last_HP - judge.base_info->remain_HP >= 99 && hurt_mode == NO_HIT)
	{
		hurt_mode = UNDER_BIGGG_HIT;
	}
	
	/*״̬����*/
	if(hurt_mode == UNDER_SMALL_HIT)
	{
		if(hurt_cnt++ > 4000)
		{
			hurt_cnt  = 0;
			hurt_mode = NO_HIT;
		}
		
		if(hurt_cnt%1700 >= 900)
		{
			if(vision_spin_speed>= 0 )
			{
				hurt_change = -7500;
			}
			else
			{
				hurt_change = -700;
			}						
		}
		else
		{
			hurt_change = 700;
		}
	}
	else if(hurt_mode == UNDER_BIGGG_HIT)
	{
		if(hurt_cnt++ > 6000)
		{
			hurt_cnt  = 0;
			hurt_mode = NO_HIT;
		}
		
		if(hurt_cnt%3000 >= 2000)
		{
			if(vision_spin_speed>= 0 )
			{
				hurt_change = -8000;
			}
			else
			{
				hurt_change = -1000;
			}						
		}
		else if(hurt_cnt%3000 <= 1000)
		{
			if(vision_spin_speed>= 0 )
			{
				hurt_change = 900;
			}
			else
			{
				hurt_change = -800;
			}	
		}
		else
		{
			hurt_change = 1000;
		}
	}
	else
	{
		hurt_change = 0;
	}
}
/**
  * @Name    chassis_work
  * @brief   ��������
  * @param   ���� 
  * @retval
  * @author  HWX
  * @Date    2022-11-06
**/
void chassis_work()
{
	
	if(car_structure.ctrl == RC_CAR)
	{

		/*��еģʽ*/
		if(car_structure.mode == machine_CAR)
		{
			/*���̿��Ʋ���*/
			Chassis.base_info.target.front_speed = (-1)*(float)rc_structure.base_info->ch3 * (float)Chassis.work_info.config.speed_max / 660.f;
			Chassis.base_info.target.right_speed = (-1)*(float)rc_structure.base_info->ch2 * (float)Chassis.work_info.config.speed_max / 660.f;
			Chassis.base_info.target.cycle_speed = (0.2)*(float)rc_structure.base_info->ch0 / 660.f * Chassis.work_info.config.speed_max;
			
			/*״̬λ���*/
			Chassis.config_top_calc(CHASSIS_TOP_CALC_OFF);
			sin_cnt = 0;
			
			/*�����ٶ�����*/
			if(rc_structure.base_info->thumbwheel.value == -660)
			{
				if(Chassis.work_info.config.speed_max ++ >= 10000)
				{
					Chassis.work_info.config.speed_max = 10000;
				}
			}
			else if(rc_structure.base_info->thumbwheel.value == 660)
			{
				if(Chassis.work_info.config.speed_max -- <= 0)
				{
					Chassis.work_info.config.speed_max = 0;
				}					
			}	
		}
		else if(car_structure.mode == navigation_CAR)
		{	
			
			Chassis.base_info.target.front_speed = (-1)*(float)navigation_structure.rx_pack->RxData.chassis_front*change_p;//chassis_front �Ӿ��� ǰ��
			Chassis.base_info.target.right_speed = (-1)*(float)navigation_structure.rx_pack->RxData.chassis_right*change_p; //�Ӿ��� ����
			
			Chassis.base_info.target.cycle_speed = 500;
			
						
			Chassis.base_info.measure.top_detal_angle = (float)((MOROT_9015_MIDDLE -  Gimbal.Yaw_9015->base_info->encoder )/65536.0)*2*PI;
			
			Chassis_Top_Speed_Calculating(&Chassis);
			Chassis.config_top_calc(CHASSIS_TOP_CALC_ON);
			
			
		}
		else if(car_structure.mode == spin_CAR )
		{
			Chassis.base_info.target.front_speed = (-1)*(float)rc_structure.base_info->ch3 * (float)CHASSIS_SPEED_MAX / 660.f;
			Chassis.base_info.target.right_speed = (-1)*(float)rc_structure.base_info->ch2 * (float)CHASSIS_SPEED_MAX / 660.f;
			Chassis.base_info.target.cycle_speed = spin_speed;
			
			
			Chassis.base_info.measure.top_detal_angle = (float)((MOROT_9015_MIDDLE -  Gimbal.Yaw_9015->base_info->encoder )/65536.0)*2*PI;
			
			Chassis_Top_Speed_Calculating(&Chassis);
			Chassis.config_top_calc(CHASSIS_TOP_CALC_ON);
			
			/*�����ٶ�����*/
			if(rc_structure.base_info->thumbwheel.value == -660)
			{
				if(spin_speed ++ >= 7000)
				{
					spin_speed = 7000;
				}
			}
			else if(rc_structure.base_info->thumbwheel.value == 660)
			{
				if(spin_speed -- <= -7000)
				{
					spin_speed = -7000;
				}					 
			}
			
		}
		else if(car_structure.mode == two_CAR)
		{
			Chassis.base_info.target.front_speed = 0;
			Chassis.base_info.target.right_speed = 0;
			Chassis.base_info.target.cycle_speed = 0;

			
		}
		else if(car_structure.mode == follow_CAR)
		{
			/*���̿��Ʋ���*/
			Chassis.base_info.target.front_speed = (-1)*(float)rc_structure.base_info->ch3 * (float)Chassis.work_info.config.speed_max / 660.f;
			Chassis.base_info.target.right_speed = (-1)*(float)rc_structure.base_info->ch2 * (float)Chassis.work_info.config.speed_max / 660.f;
			Chassis.base_info.target.cycle_speed = pid_calc_err_9015(&chassis_pid_follow_structure,
																	  Gimbal.Yaw_9015->base_info->encoder,\
																	  MOROT_9015_MIDDLE); //��pid�������
			
			Chassis.base_info.measure.top_detal_angle = (float)((MOROT_9015_MIDDLE -  Gimbal.Yaw_9015->base_info->encoder )/65536.0)*2*PI;
			Chassis_Top_Speed_Calculating(&Chassis);
			
			if(Gimbal.Yaw_9015->info->status == _9015_OFFLINE)
			{
				Chassis.base_info.target.cycle_speed = 0;	
			}
			Chassis.config_top_calc(CHASSIS_TOP_CALC_OFF);
			

			
		}
		else if(car_structure.mode == patrol_CAR || car_structure.mode ==  shake_CAR || car_structure.mode == vision_CAR)
		{
			/*��ת����ֵ*/
			chassis_cycle_speed_change();
			
			/*��ת�⸳ֵ*/
			Chassis.base_info.target.front_speed = (-1)*(float)navigation_structure.rx_pack->RxData.chassis_front*change_p;//chassis_front �Ӿ��� ǰ��
			Chassis.base_info.target.right_speed = (-1)*(float)navigation_structure.rx_pack->RxData.chassis_right*change_p; //�Ӿ��� ����
			Chassis.base_info.measure.top_detal_angle = (float)((MOROT_9015_MIDDLE -  Gimbal.Yaw_9015->base_info->encoder )/65536.0)*2*PI;
			
			Chassis_Top_Speed_Calculating(&Chassis);
			Chassis.config_top_calc(CHASSIS_TOP_CALC_ON);
			
			//���Դ���
			//Chassis.base_info.target.cycle_speed  = 0;
		}
		else 
		{
			Chassis.base_info.target.front_speed = 0;
			Chassis.base_info.target.right_speed = 0;
			Chassis.base_info.target.cycle_speed = 0;
		}
		
	}
	
	/*��̨��ָ��*/
	if(judge.info->status == JUDGE_ONLINE)
	{
		if(judge.base_info->robot_commond == 'W') //'W'
		{
			chassis_aerial_cmd = AERIAL_W;
		}
		else if(judge.base_info->robot_commond == 'A') //'A'
		{
			chassis_aerial_cmd = AERIAL_A;
		}
		else if(judge.base_info->robot_commond == 'S') //'S'
		{
			chassis_aerial_cmd = AERIAL_S;
		} 
		else if(judge.base_info->robot_commond == 'D') //'D'
		{
			chassis_aerial_cmd = AERIAL_D;
		}
	}
	else
	{
		chassis_aerial_cmd = AERIAL_NO;
	}
	
	/*��ͼ���������Ͻ�
	
		W
	A	S	D
	
	
	
	*/
	
	
	/*��̨��ָ����Ӧ*/
	//uint16_t Suppose_Middle_Angle = MOROT_9015_MIDDLE - 8*(bmi_structure.first_yaw_angle - bmi_structure.yaw_angle);
	uint16_t Suppose_Middle_Angle = MOROT_9015_MIDDLE - 8*(4096 - navigation_structure.rx_pack->RxData.yaw_angle);
	if(chassis_aerial_cmd == AERIAL_W)
	{
		if(aerial_time++ >= AERIAL_MOVE_TIME)
		{
			chassis_aerial_cmd = AERIAL_NO;
			judge.base_info->robot_commond = 0;
			aerial_time = 0;
		}
		else
		{
			if(judge.data->ext_robot_command.target_x >= X_LIMIIT)
			{
				Chassis.base_info.target.front_speed = 0;
				Chassis.base_info.target.right_speed = 900 ;
			}
			else
			{
				Chassis.base_info.target.front_speed = 0;
				Chassis.base_info.target.right_speed = 2500 ;
			}
				
			/*����С���ݽ���*/
			Chassis.base_info.measure.top_detal_angle = (float)((Suppose_Middle_Angle -  Gimbal.Yaw_9015->base_info->encoder )/65536.0)*2*PI;
			Chassis_Top_Speed_Calculating(&Chassis);
			
			
			Chassis.config_top_calc(CHASSIS_TOP_CALC_ON);
		}
	}
	else if(chassis_aerial_cmd == AERIAL_A)
	{
		if(aerial_time++ >= AERIAL_MOVE_TIME)
		{
			chassis_aerial_cmd = AERIAL_NO;
			judge.base_info->robot_commond = 0;
			aerial_time = 0;
		}
		else
		{
			if(judge.data->ext_robot_command.target_x >= X_LIMIIT)
			{
				Chassis.base_info.target.front_speed = 900;
				Chassis.base_info.target.right_speed = 0 ;
			}
			else
			{
				Chassis.base_info.target.front_speed = 2500;
				Chassis.base_info.target.right_speed = 0 ;
			}
				
			/*����С���ݽ���*/
			Chassis.base_info.measure.top_detal_angle = (float)((Suppose_Middle_Angle -  Gimbal.Yaw_9015->base_info->encoder )/65536.0)*2*PI;
			Chassis_Top_Speed_Calculating(&Chassis);
			
			Chassis.config_top_calc(CHASSIS_TOP_CALC_ON);
		}
	}
	else if(chassis_aerial_cmd == AERIAL_D)
	{
		if(aerial_time++ >= AERIAL_MOVE_TIME)
		{
			chassis_aerial_cmd = AERIAL_NO;
			judge.base_info->robot_commond = 0;
			aerial_time = 0;
		}
		else
		{
			if(judge.data->ext_robot_command.target_x >= X_LIMIIT)
			{
				Chassis.base_info.target.front_speed = -900;
				Chassis.base_info.target.right_speed = 0 ;
			}
			else
			{
				Chassis.base_info.target.front_speed = -2500;
				Chassis.base_info.target.right_speed = 0;
			}
			/*����С���ݽ���*/
			Chassis.base_info.measure.top_detal_angle = (float)((Suppose_Middle_Angle -  Gimbal.Yaw_9015->base_info->encoder )/65536.0)*2*PI;
			Chassis_Top_Speed_Calculating(&Chassis);
			
			
			Chassis.config_top_calc(CHASSIS_TOP_CALC_ON);
		}
	}
	else if(chassis_aerial_cmd == AERIAL_S)
	{
		if(aerial_time++ >= AERIAL_MOVE_TIME)
		{
			chassis_aerial_cmd = AERIAL_NO;
			judge.base_info->robot_commond = 0;
			aerial_time = 0;
		}
		else
		{
			if(judge.data->ext_robot_command.target_x >= X_LIMIIT)
			{
				Chassis.base_info.target.front_speed = 0;
				Chassis.base_info.target.right_speed = -900 ;
			}
			else
			{
				Chassis.base_info.target.front_speed = 0;
				Chassis.base_info.target.right_speed = -2500 ;
			}
			/*����С���ݽ���*/
			/*�ı�MIDDLE�������9015MIDDLE�����������ʼ��ֵ*8��*/
			Chassis.base_info.measure.top_detal_angle = (float)((Suppose_Middle_Angle -  Gimbal.Yaw_9015->base_info->encoder )/65536.0)*2*PI;
			Chassis_Top_Speed_Calculating(&Chassis);
			
			
			
			Chassis.config_top_calc(CHASSIS_TOP_CALC_ON);
		}
	}
	
	/*��������--���ٶ�����*/
	Front_Acc_Fliter.target_speed     = Chassis.base_info.target.front_speed;
	Front_Acc_Fliter.current_speed    = Chassis.base_info.measure.front_speed;
	Front_Acc_Fliter.max_acceleration = 120;
	
	Right_Acc_Fliter.target_speed     = Chassis.base_info.target.right_speed;
	Right_Acc_Fliter.current_speed    = Chassis.base_info.measure.right_speed;
	Right_Acc_Fliter.max_acceleration = 120;
	
	Chassis.base_info.target.front_speed = SmoothAccelerationUpdate(&Front_Acc_Fliter);
	Chassis.base_info.target.right_speed = SmoothAccelerationUpdate(&Right_Acc_Fliter);
	
	Chassis_Work(&Chassis);

}


/***********************�����㷨*****************************************************************************************************************/

#define DISTANCE_MAX   21.0f
#define RADIUS         1.0f
#define STARING_YAW    4096

/*����С���ݽ���*/

void Depart(float Engi_x ,float Engi_y ,int16_t *speed_x, int16_t *speed_y, float first_yaw_angle)
{
		/*��ȡ����λ��*/
	float Shao_x = judge.data->game_robot_pos.x;
	float Shao_y = judge.data->game_robot_pos.y;
	
	float dx = Engi_x - Shao_x;
	float dy = Engi_y - Shao_y;
	
	
	/*�������*/
	int16_t distance = sqrt(pow((dx),2) + pow((dy),2));
	
	if(distance <= DISTANCE_MAX && distance >= RADIUS)
	{
		// ����Ƕȣ��Ի���Ϊ��λ��
		float angle_rad = atan2(dy, dx);

		// ���Ƕ�ת��Ϊ�� x ��������Ϊ����˳ʱ��Ƕȣ��Զ�Ϊ��λ��
		float angle_deg = angle_rad * 180.0 / PI ;
		angle_deg = angle_deg * 22.7555 + first_yaw_angle;
		if (angle_deg < 0) 
		{
			angle_deg += 8192.0;
		}
		else if(angle_deg > 8191)
		{
			angle_deg -= 8192.0;
		}
		
		/*�ٶȼ���*/
		*speed_x = dx*100 + signbit(dx) * 500;
		*speed_y = dy*100 + signbit(dy) * 500;
		
		/*��Χ����*/
		if(*speed_y >= 3000)
		{
			*speed_y = 3000;
		}
		else if(*speed_y <= -3000)
		{
			*speed_y = -3000;
		}
		if(*speed_x >= 3000)
		{
			*speed_x = 3000;
		}
		else if(*speed_x <= -3000)
		{
			*speed_x = -3000;
		}
		
		/*�������������ϵ�¹���������ҵĽǶ�*/
		/*С���ݽ���*/
		Chassis.base_info.measure.top_detal_angle = (float)((first_yaw_angle - angle_deg )/8192.0)*2*PI;
		Chassis_Top_Speed_Calculating(&Chassis);
		

	}
	else if(distance <= RADIUS)
	{
		*speed_x = 0;
		*speed_y = 0;
	}
}


void Follow_Engineer(int16_t *speed_x, int16_t *speed_y, float first_yaw_angle)
{
	/*��ȡ����λ��*/
	float Engi_x = judge.data->robot_position.engineer_x;
	float Engi_y = judge.data->robot_position.engineer_y;
	
	Depart(Engi_x,Engi_y,speed_x,speed_y,first_yaw_angle);

}


#if 0
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>

#define INFINITY 9999
#define MAX_NODES 100

// ͼ�Ľڵ�ṹ
typedef struct {
    int weight;
    bool visited;
    int previous;
} Node;

// Dijkstra�㷨
void dijkstra(int graph[MAX_NODES][MAX_NODES], int source, int num_nodes) {
    Node nodes[MAX_NODES];

    // ��ʼ���ڵ�
    for (int i = 0; i < num_nodes; i++) {
        nodes[i].weight = INFINITY;
        nodes[i].visited = false;
        nodes[i].previous = -1;
    }

    // ����Դ�ڵ�
    nodes[source].weight = 0;

    // Ѱ�����·��
    for (int count = 0; count < num_nodes - 1; count++) {
        int min_weight = INFINITY;
        int min_index = -1;

        // ѡ��Ȩ����С��δ���ʹ��Ľڵ�
        for (int i = 0; i < num_nodes; i++) {
            if (!nodes[i].visited && nodes[i].weight < min_weight) {
                min_weight = nodes[i].weight;
                min_index = i;
            }
        }

        // ��Ǹýڵ�Ϊ�ѷ���
        nodes[min_index].visited = true;

        // �������ڽڵ��Ȩ��
        for (int i = 0; i < num_nodes; i++) {
            if (!nodes[i].visited && graph[min_index][i] != 0 && nodes[min_index].weight + graph[min_index][i] < nodes[i].weight) {
                nodes[i].weight = nodes[min_index].weight + graph[min_index][i];
                nodes[i].previous = min_index;
            }
        }
    }

    // ��ӡ���·����Ȩ��
    for (int i = 0; i < num_nodes; i++) {
        printf("�ڵ� %d ��Դ�ڵ�����·����", i);
        int path[MAX_NODES];
        int path_length = 0;
        int current = i;

        while (current != source) {
            path[path_length++] = current;
            current = nodes[current].previous;
        }

        printf("%d", source);

        for (int j = path_length - 1; j >= 0; j--) {
            printf(" -> %d", path[j]);
        }

        printf("\tȨ�أ�%d\n", nodes[i].weight);
    }
}

int main() {
    int graph[MAX_NODES][MAX_NODES];
    int num_nodes, source;

    printf("������ڵ���Ŀ��");
    scanf("%d", &num_nodes);

    printf("������ͼ���ڽӾ���\n");

    for (int i = 0; i < num_nodes; i++) {
        for (int j = 0; j < num_nodes; j++) {
            scanf("%d", &graph[i][j]);
        }
    }

    printf("������Դ�ڵ㣺");
    scanf("%d", &source);

    dijkstra(graph, source, num_nodes);

    return 0;
}

#endif