#ifndef __CAN_PROTOCOL_H
#define __CAN_PROTOCOL_H

#include "stm32f3xx_hal.h"
#include "drv_can.h"
#include "rp_math.h"

/*�������ݽ���*/
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
        uint16_t warning : 1;   //����
        uint16_t cap_v_over : 1;    //���ݹ�ѹ
        uint16_t cap_i_over : 1;    //���ݹ���
        uint16_t cap_v_low : 1;     //����Ƿѹ
        uint16_t bat_v_low : 1;     //����ϵͳǷѹ
        uint16_t can_receive_miss : 1;    //δ����CANͨ������
    }bit;
}cap_state_t;

typedef struct
{
    uint16_t chassis_power_buffer;  //���̹��ʻ���
    uint16_t chassis_power_limit;   //�����˵��̹�����������
    int16_t  output_power_limit;    //���ݷŵ繦������
    uint16_t input_power_limit;     //���ݳ�繦������
    uint16_t chassis_volt;			//����ϵͳ���̵�ѹ
    uint16_t chassis_current;		//����ϵͳ���̵���

    union{
        uint16_t all;
        struct
        {
            uint16_t cap_switch : 1;    //���ݿ���
            uint16_t cap_record : 1;    //��¼���ܿ���
            uint16_t gamegoing : 1;     //����������Ϊ1������Ϊ0
        }bit;
    }cap_control;
}CAN_Recive_t;

typedef struct
{
    uint16_t    cap_current;  /*���ݵ���*/
    uint16_t    cap_volt;     /*���ݵ�ѹ*/
    cap_state_t cap_state;    /*����״̬*/
}CAN_Transimit_t;

typedef struct
{
    uint16_t offline_cnt;     /*���߼�����*/
    uint16_t offline_cnt_max; /*���߼��������ֵ*/
    uint16_t status;          /*״̬*/

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

