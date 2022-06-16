#include "bsp.h"

void uart2_send_by485(uint8_t *pData, uint16_t Size)
{
	HAL_GPIO_WritePin(uart2_en_GPIO_Port,uart2_en_Pin,GPIO_PIN_SET);
	HAL_UART_Transmit(&huart2,pData,Size,1000);
	HAL_GPIO_WritePin(uart2_en_GPIO_Port,uart2_en_Pin,GPIO_PIN_RESET);
}

float get_temper(void)
{
	HAL_ADC_Start(&hadc1);	
	HAL_ADC_PollForConversion(&hadc1,10);
	uint16_t AD_Value = HAL_ADC_GetValue(&hadc1);
	float Vol_Value = AD_Value*(3.3/4096);
	return (1.43 - Vol_Value)/0.0043 + 25;
}
