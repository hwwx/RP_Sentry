#ifndef __VISION_H
#define __VISION_H

#include "stm32f4xx_hal.h"
#include <string.h>
#include <stdbool.h>
#include "crc.h"
/* �궨����ȫ�ֱ��� ------------------------------------------------------------*/

/*��ʼ�ֽڣ�Э��̶�β0xA5*/
#define VISION_SEND_ID             (0xA5)
#define VISION_OFFLINE_MAX_CNT     (300)
#define VISION_OFFLINE             (0)
#define VISION_ONLINE              (1)
#define BALANCE_HP                 (300)
#define HIT                        (1)
#define NO_HIT                     (0)

#define BIGGG_ARMOR                (1)
#define SMALL_ARMOR                (0)
/* ָ�������� ------------------------------------------------------------*/
typedef enum {
	/*�ر���Ҫ�ఴһ��*/
	/*�Ӿ�����ģʽ*/
	CMD_PATROL		        = 0,	// Ѳ��ģʽ 
	CMD_GOGOGO              = 7,    // ָ������       Z
	
	CMD_R3                  = 8,    // ǰ��R3�ߵ�     T  ����
	CMD_R4                  = 9,    // ǰ��R4�ߵ�     N   ()
	CMD_OUTPOS              = 10,   // ǰ������       Q   
	
	CMD_POINT_A             = 11,   // ǰ���ҷ����ŵ� Y ����ǰ�ڣ��Զ�������ǰ�ڣ�
	CMD_FUCK_OUTPOS         = 13,   // ��Դ����ǰ��   J ����Դ����ǰ�ڣ��Զ�������ǰ�ڣ�
	//ֻ�ܰ�һ��J
	CMD_POINT_B             = 12,   // ǰ���ҷ�ǰ��ս K ����Ӣ�۴�ǰ��ս��
	
	CMD_POINT_D             = 14,   // ǰ��Ѳ�������� I �����ٻؼ�2��
	CMD_POINT_E             = 15,   // ǰ������ǰ��ս V ������1��
	CMD_STOP                = 16,   // ֹͣ           H
	//CMD_POINT_F             = 17,   // ǰ������һ�վ B ������2��
	CMD_POINT_G             = 18,   // ǰ��������Դ�� O �����ִ򹤳�)
	CMD_POINT_H             = 19,   // ǰ������Ѳ���� L ������3��
	CMD_POINT_J             = 21,   // Ѳ����Ѳ��ģʽ G	
	CMD_POINT_K             = 22,   // ǰ��������ŵ� F
	
	//E ���򹤳� Ĭ�Ͽ��� �Ұ벿�ֿ���
	//C ֻ��Ӣ�� Ĭ�Ϲر� �Ұ벿�ֿ��� 
	//R �������� Ĭ�Ͽ��� �Ұ벿�ֿ��� (�ؼҹ����б���,�ǵ��ٴ�ѡ��Ŀ�ĵ�)
	////U ֻ��ǰ�� Ĭ�Ϲر� �Ұ벿�ֿ��� (���ִ�ǰ��)
	//X ǿ��Ѳ�� Ĭ�Ϲر� �Ұ벿�ֿ��� (��ֹ��ʶ��)
	//B ֻ���ڱ� Ĭ�Ϲر� �Ұ벿�ֿ���
	
	//WASD �ұ��� ��߿�
	
	/*��ط���ģʽ*/
	CMD_CHECK               = 2,    // �������ģʽ

} vision_cmd_id_t;


/* ���ݳ��� */
typedef enum {
	/* Std */
	LEN_VISION_RX_PACKET	= 17 + 5,	// ���հ���������
	LEN_VISION_TX_PACKET	= 55 + 5,	// ���Ͱ���������

	LEN_RX_DATA 			= LEN_VISION_RX_PACKET - 5,	// �������ݶγ���
	LEN_TX_DATA 			= LEN_VISION_TX_PACKET - 5,	// �������ݶγ���
	
	LEN_FRAME_HEADER 	  	= 3,	// ֡ͷ����
	LEN_FRAME_TAILER 		= 2,	// ֡βCRC16
	
} vision_data_length_t;


/* ���ݰ���ʽ ------------------------------------------------------------*/
/* ֡ͷ��ʽ */
typedef struct __packed
{
	uint8_t  			    sof;		// ͬ��ͷ
	vision_cmd_id_t  	    cmd_id;	    // ������
	uint8_t  			    crc8;		// CRC8У����
} vision_frame_header_t;

/* ֡β��ʽ */
typedef struct __packed 
{
	uint16_t crc16;					// CRC16У����
} vision_frame_tailer_t;


/* �������ݶθ�ʽ */
typedef struct __packed 
{
	/*����̨*/
    int16_t     L_yaw_angle;
    int16_t     L_pitch_angle;
	
	/*����̨*/
	int16_t     R_yaw_angle;
    int16_t     R_pitch_angle;
	
	/*��YAW��*/
	int16_t     B_yaw_angle;
	
	/*��������*/
	uint8_t remain_bullet;
	uint8_t friendly_outpose_HP;
	uint8_t my_HP;
	uint8_t enemy_hero_HP;
	uint8_t enemy_engening_HP;
	uint8_t enemy_infantry3_HP;
	uint8_t enemy_infantry4_HP;
	uint8_t enemy_infantry5_HP;
	uint8_t enemy_sentry_HP;
	uint8_t enemy_outpose_HP;
	
	 __packed   union
    {
        uint8_t flag; 
        struct __packed
        {
            uint8_t engine_hit_enable : 1;    //ֻ��Ӣ��
            uint8_t hero_hit_enable : 1;      //�Ƿ�򹤳�
            uint8_t my_color : 1;             //�ҵ���ɫ
            uint8_t game_progress : 1;        //��������
			uint8_t ammor_size_3 : 1;         //���Ų���װ�װ��С
			uint8_t ammor_size_4 : 1;         //�ĺŲ���װ�װ��С
			uint8_t ammor_size_5 : 1;         //��Ų���װ�װ��С
			uint8_t only_outpose : 1;         //ֻ��ǰ��
		}bit;
    }flag;
	
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
	
	float uwb_x;
	float uwb_y;
	
	uint8_t arrive_flag;
	uint8_t only_sentry;

}vision_tx_data_t;

/*�����Ӿ�����ͻȻ���ߵ�׼��*/
typedef struct __packed 
{
	/*����̨*/
	int16_t      L_yaw_angle;
    int16_t      L_pitch_angle;
	
	uint8_t      L_shoot_speed;
    uint8_t      L_is_find_target;
	uint8_t      L_shoot_cnt;
	
	/*����̨*/
	uint16_t     R_yaw_angle;
    uint16_t     R_pitch_angle;
	
	uint8_t      R_shoot_speed;
    uint8_t      R_is_find_target;
	uint8_t      R_shoot_cnt;
	
	uint16_t     B_yaw_angle;
	
	uint8_t      nuc_flag1;
	
}vision_rx_data_t;

/* ���Ͱ���ʽ */
typedef struct __packed
{
	vision_frame_header_t   FrameHeader;	// ֡ͷ
	vision_tx_data_t	    TxData;		    // ����
	vision_frame_tailer_t   FrameTailer;	// ֡β	
}vision_tx_packet_t;

/* ���հ���ʽ */
typedef struct __packed 
{
	vision_frame_header_t   FrameHeader;	// ֡ͷ
	vision_rx_data_t	    RxData;		    // ����
	vision_frame_tailer_t   FrameTailer;	// ֡β	
} vision_rx_packet_t;


/* ����ģʽ ------------------------------------------------------------*/

/*����ģʽ*/
typedef enum
{
	VISION_MODE_MANUAL		  = 0,	// �ֶ�ģʽ
	VISION_MODE_AUTO		  = 1,	// ����ģʽ
	VISION_MODE_BIG_BUFF	  = 2,	// ����ģʽ
	VISION_MODE_SMALL_BUFF	  = 3,	// ��С��ģʽ
} Vision_Mode_t;

/* ������ʶ���� */
typedef struct
{
	uint8_t 		  my_color;			 // ��0/1��ʾ��ɫ
	Vision_Mode_t	  mode;				 // �Ӿ�ģʽ
	uint8_t  		  rx_data_valid;     // �������ݵ���ȷ��
	uint16_t 		  rx_err_cnt;		 // �������ݵĴ���ͳ��
	uint32_t		  rx_cnt;		     // �������ݰ���ͳ��
	bool		      rx_data_update;    // ���������Ƿ����
                                                                                                                                                                                   	uint32_t 		  rx_time_prev;	     // �������ݵ�ǰһʱ��
	uint32_t 		  rx_time_now;	     // �������ݵĵ�ǰʱ��
	uint16_t 		  rx_time_fps;	     // ֡��
	
	uint8_t           work_state;
	int16_t		      offline_cnt;
	int16_t		      offline_max_cnt;
	
} vision_state_t;
/* ���� ------------------------------------------------------------*/

typedef  struct 
{
	vision_rx_packet_t *rx_pack;
	vision_tx_packet_t *tx_pack;
	vision_state_t      state;
	
	
}vision_t;

/* ��غ��� ------------------------------------------------------------*/

void Vision_Init(void);
void Vision_Update(void);

bool Vision_SendData(void);
bool Vision_GetData(uint8_t *rxBuf);

void vision_task(void);

#endif
