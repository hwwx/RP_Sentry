#ifndef __NAVIGATION_H
#define __NAVIGATION_H

#include "stm32f4xx_hal.h"
#include <string.h>
#include <stdbool.h>
#include "crc.h"
#include "vision.h"
/* �궨����ȫ�ֱ��� ------------------------------------------------------------*/


/* ָ�������� ------------------------------------------------------------*/



/* ���ݳ��� */
typedef enum {
	/* Std */
	LEN_NAVIGATION_RX_PACKET	= 31 + 5,	// ���հ���������
	LEN_NAVIGATION_TX_PACKET	= 26 + 5,	// ���Ͱ���������

	LEN_NAVIGATION_RX_DATA 			= LEN_VISION_RX_PACKET - 5,	// �������ݶγ���
	LEN_NAVIGATION_TX_DATA 			= LEN_VISION_TX_PACKET - 5,	// �������ݶγ���
	
	LEN_NAVIGATION_FRAME_HEADER 	  	= 3,	// ֡ͷ����
	LEN_NAVIGATION_FRAME_TAILER 		= 2,	// ֡βCRC16
	
}navigation_data_length_t;


/* ���ݰ���ʽ ------------------------------------------------------------*/


/* �������ݶθ�ʽ */
typedef struct __packed 
{
	/*��YAW��*/
	int16_t     B_yaw_angle;
	
	float   uwb_x;
	float   uwb_y;
	uint8_t game_progress;
	uint8_t my_color;
	uint8_t ammor_size_3;
	uint8_t ammor_size_4;
	uint8_t ammor_size_5;
	
	uint8_t nuc_flag1;    //�Ƿ��������״̬
	uint8_t navi_enable;  //�Ƿ�������
	uint8_t omni_enable;  //�Ƿ���ȫ���֪
	
	
	float target_x;
	float target_y;

}navigation_tx_data_t;/*�����Ӿ�����ͻȻ���ߵ�׼��*/


typedef struct __packed 
{
	int16_t   chassis_front; //����ǰ���ٶ� 
	int16_t   chassis_right; //����ƽ���ٶ�
	
	int16_t  yaw_angle;     //��ǰYaw��Ƕ�
	
	
	/*˫��ͨѶ*/
	uint8_t enemy1_num;
	int16_t enemy1_x;
	int8_t  enemy1_y;
	int16_t enemy1_z;
	
	uint8_t enemy2_num;
	int16_t enemy2_x;
	int8_t  enemy2_y;
	int16_t enemy2_z;
	
	uint8_t enemy3_num;
	int16_t enemy3_x;
	int8_t  enemy3_y;
	int16_t enemy3_z;
	
	uint8_t enemy4_num;
	int16_t enemy4_x;
	int8_t  enemy4_y;
	int16_t enemy4_z;
	
	uint8_t arrive_flag;  //�����־λ
	
}navigation_rx_data_t;

/* ���Ͱ���ʽ */
typedef struct __packed
{
	vision_frame_header_t   FrameHeader;	// ֡ͷ
	navigation_tx_data_t	TxData;		    // ����
	vision_frame_tailer_t   FrameTailer;	// ֡β	
}navigation_tx_packet_t;

/* ���հ���ʽ */
typedef struct __packed 
{
	vision_frame_header_t   FrameHeader;	// ֡ͷ
	navigation_rx_data_t	RxData;		    // ����
	vision_frame_tailer_t   FrameTailer;	// ֡β	
} navigation_rx_packet_t;


/* ����ģʽ ------------------------------------------------------------*/
/* ���� ------------------------------------------------------------*/

typedef  struct 
{
	navigation_rx_packet_t *rx_pack;
	navigation_tx_packet_t *tx_pack;
	vision_state_t          state;
	
	
}navigation_t;

/* ��غ��� ------------------------------------------------------------*/

void Navigation_Init(void);
void Navigation_Update(void);

bool Navigation_SendData(void);
bool Navigation_GetData(uint8_t *rxBuf);

void Navigation_task(void);

#endif
