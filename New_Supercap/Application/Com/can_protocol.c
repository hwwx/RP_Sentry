/*
 * @Author: hwwx 93569047+hwwx@users.noreply.github.com
 * @Date: 2023-10-17 14:49:49
 * @LastEditors: hwwx 93569047+hwwx@users.noreply.github.com
 * @LastEditTime: 2023-10-22 21:21:48
 * @FilePath: \MDK-ARMc:\Users\HWX\Documents\GitHub\2023_HWX\New_Supercap\Application\Com\can_protocol.c
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#include "can_protocol.h"
#include <string.h>
Cap_communication_t CAN_Data =
{
	.E_state.offline_cnt = CAN_OFFLINE_TIME_MAX,
	.E_state.offline_cnt_max = CAN_OFFLINE_TIME_MAX,
	.E_state.status = DEV_OFFLINE,

	.F_state.offline_cnt = CAN_OFFLINE_TIME_MAX,
	.F_state.offline_cnt_max = CAN_OFFLINE_TIME_MAX,
	.F_state.status = DEV_OFFLINE,

	.send = CAP_CanSendData,
	.get = CAP_CanGetData,
	.heart = CAP_CanHeartBeat,
};

/**
 *	@brief	掉线检测
 */
void CAP_CanHeartBeat(Cap_communication_t * pack)
{
	if(pack->E_state.offline_cnt ++ >= CAN_OFFLINE_TIME_MAX)
	{
		pack->E_state.status = DEV_OFFLINE;
		pack->E_state.offline_cnt = CAN_OFFLINE_TIME_MAX;
	}
	else
	{
		pack->E_state.status = DEV_ONLINE;
	}

	if(pack->F_state.offline_cnt ++ >= CAN_OFFLINE_TIME_MAX)
	{
		pack->F_state.status = DEV_OFFLINE;
		pack->F_state.offline_cnt = CAN_OFFLINE_TIME_MAX;
	}
	else
	{
		pack->F_state.status = DEV_ONLINE;
	}
}



/**
*	@brief	数据信息发送
 */
void CAP_CanSendData(Cap_communication_t * pack)
{

	uint32_t txMailBox;//发送邮箱
    CAN_TxHeaderTypeDef txFrameHeader;
    uint8_t data[8];
	
    /*数据处理*/
	int16_t temp_u = float_to_int16(pack->transimit.cap_volt, 30, 0, 32000, -32000);
	int16_t temp_i = float_to_int16(pack->transimit.cap_current, 30, -20, 32000, -32000);
	int16_t temp_s = pack->transimit.cap_state.state;
	
	data[0] = (uint8_t)(temp_u>> 8);
	data[1] = (uint8_t)(temp_u);

	data[2] = (uint8_t)(temp_i>> 8);
	data[3] = (uint8_t)(temp_i);

	data[4] = (uint8_t)(temp_s>> 8);
	data[5] = (uint8_t)(temp_s);

    /*数据帧开头*/
    txFrameHeader.StdId = CAP_CAN_DATA_3;
    txFrameHeader.IDE   = CAN_ID_STD;
    txFrameHeader.RTR   = CAN_RTR_DATA;
    txFrameHeader.DLC   = 0x08;

    /*默认使用CAN*/
	HAL_CAN_AddTxMessage(&DRV_CAN_USE, &txFrameHeader, data, &txMailBox);

    memset(data,0,sizeof(data));

}

/**
*	@brief	数据信息接收解析
 */
void CAP_CanGetData( uint32_t id ,Cap_communication_t * pack , uint8_t *rxBuf)
{
	switch (id)
	{
		case CAP_CAN_DATA_E:
		/*清除掉线标志位*/
		pack->E_state.offline_cnt = 0;
		/*数据解析*/
		pack->recive.chassis_power_buffer 	= ((uint16_t)rxBuf[0] << 8| rxBuf[1]);
		pack->recive.chassis_volt 			= ((uint16_t)rxBuf[2] << 8| rxBuf[3]);
		pack->recive.chassis_current 		= ((uint16_t)rxBuf[4] << 8| rxBuf[5]);
		pack->recive.cap_control.all 		= ((uint16_t)rxBuf[6] << 8| rxBuf[7]);

		break;
		case CAP_CAN_DATA_F:
		/*清除掉线标志位*/
		pack->F_state.offline_cnt = 0;
		/*数据解析*/
		pack->recive.chassis_power_limit 	= ((uint16_t)rxBuf[0] << 8| rxBuf[1]);
		pack->recive.output_power_limit 	= ((uint16_t)rxBuf[2] << 8| rxBuf[3]);
		pack->recive.input_power_limit 		= ((uint16_t)rxBuf[4] << 8| rxBuf[5]);

		break;
	}
}




