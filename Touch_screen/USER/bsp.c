#include "bsp.h"

void uart2_send_by485(uint8_t *pData, uint16_t Size)
{
	HAL_GPIO_WritePin(uart2_en_GPIO_Port,uart2_en_Pin,GPIO_PIN_SET);
	HAL_UART_Transmit(&huart2,pData,Size,1000);
	HAL_GPIO_WritePin(uart2_en_GPIO_Port,uart2_en_Pin,GPIO_PIN_RESET);
}
