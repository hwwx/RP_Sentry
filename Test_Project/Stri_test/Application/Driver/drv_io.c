/**
 * @file        drv_io.c
 * @author      RobotPilots@2020
 * @Version     V1.0
 * @date        15-September-2020
 * @brief       Systick driver
 */
 
/* Includes ------------------------------------------------------------------*/
#include "drv_io.h"
#include "cmsis_os.h"
/* Private macro -------------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/
// ���������Ч������
#define MIN_BRIGHTNESS 0   // �������
#define MAX_BRIGHTNESS 10 // �������
#define BREATHE_STEP 1     // ���ȱ仯����

void Blue_setLedBrightness(uint8_t brightness) {
    // ���ݵ�ǰ��������LED���ŵĸߵ͵�ƽ
    if (brightness > 0) {
        BLUE_ON();
        osDelay(brightness);
    }
    if (brightness < 100) {
        BLACK_ON();
        osDelay(MAX_BRIGHTNESS - brightness);
    }
}
void Red_setLedBrightness(uint8_t brightness) {
    // ���ݵ�ǰ��������LED���ŵĸߵ͵�ƽ
    if (brightness > 0) {
        RED_ON();
        osDelay(brightness);
    }
    if (brightness < 100) {
        BLACK_ON();
        osDelay(MAX_BRIGHTNESS - brightness);
    }
}
uint8_t breatheDirection =1;
uint8_t currentBrightness = MIN_BRIGHTNESS;
uint8_t breatheStep = BREATHE_STEP;
void LED_breath()
{
	if (breatheDirection == 1) {
		currentBrightness += breatheStep;
		if (currentBrightness >= MAX_BRIGHTNESS) {
			currentBrightness = MAX_BRIGHTNESS;
			breatheDirection = -1;
		}
	} else {
		currentBrightness -= breatheStep;
		if (currentBrightness <= MIN_BRIGHTNESS) {
			currentBrightness = MIN_BRIGHTNESS;
			breatheDirection = 1;
		}
	}
}

void BLUE_LED_PWM()
{
	Blue_setLedBrightness(currentBrightness);
	
	LED_breath();
}

void RED_LED_PWM()
{
	Red_setLedBrightness(currentBrightness);
	
	LED_breath();
}
