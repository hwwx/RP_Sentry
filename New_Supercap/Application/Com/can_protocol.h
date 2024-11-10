#ifndef __CAN_PROTOCOL_H
#define __CAN_PROTOCOL_H

#include "stm32f3xx_hal.h"
#include "drv_can.h"
#include "rp_math.h"

/*超电数据接收*/
#define CAP_CAN_DATA_E 0x02E
#define CAP_CAN_DATA_F 0x02F
#define CAP_CAN_DATA_3 0x030

extern CAN_HandleTypeDef hcan;


#define DEV_OFFLINE            0
#define DEV_ONLINE             1
#define CAN_OFFLINE_TIME_MAX   25
#define DRV_CAN_USE            (hcan)

typedef union
{
    uint16_t state;
    struct
    {
        uint16_t warning : 1;   //报警
        uint16_t cap_v_over : 1;    //电容过压
        uint16_t cap_i_over : 1;    //电容过流
        uint16_t cap_v_low : 1;     //电容欠压
        uint16_t bat_v_low : 1;     //裁判系统欠压
        uint16_t can_receive_miss : 1;    //未读到CAN通信数据
    }bit;
}cap_state_t;

typedef struct
{
    uint16_t chassis_power_buffer;  //底盘功率缓冲
    uint16_t chassis_power_limit;   //机器人底盘功率限制上限
    int16_t  output_power_limit;    //电容放电功率限制
    uint16_t input_power_limit;     //电容充电功率限制
    uint16_t chassis_volt;			//裁判系统底盘电压
    uint16_t chassis_current;		//裁判系统底盘电流

    union{
        uint16_t all;
        struct
        {
            uint16_t cap_switch : 1;    //电容开关
            uint16_t cap_record : 1;    //记录功能开关
            uint16_t gamegoing : 1;     //比赛进行中为1，否则为0
        }bit;
    }cap_control;
}CAN_Recive_t;

typedef struct
{
    uint16_t    cap_current;  /*电容电流*/
    uint16_t    cap_volt;     /*电容电压*/
    cap_state_t cap_state;    /*电容状态*/
}CAN_Transimit_t;

typedef struct
{
    uint16_t offline_cnt;     /*掉线计数器*/
    uint16_t offline_cnt_max; /*掉线计数器最大值*/
    uint16_t status;          /*状态*/

}CAN_work_t;


typedef struct Cap_com_
{
    CAN_Recive_t    recive;
    CAN_Transimit_t transimit;
    CAN_work_t      E_state;
    CAN_work_t      F_state;

    void (*send)(struct Cap_com_ * pack);
    void (*get)(uint32_t id ,struct Cap_com_ * pack , uint8_t *rxBuf);
    void (*heart)(struct Cap_com_ * pack);
}Cap_communication_t;



extern Cap_communication_t CAN_Data;


void CAP_CanGetData( uint32_t id ,Cap_communication_t* pack , uint8_t *rxBuf);
void CAP_CanHeartBeat(Cap_communication_t * pack);
void CAP_CanSendData(Cap_communication_t * pack);
#endif

