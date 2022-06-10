#include "bsp.h"

void uart2_send_by485(uint8_t *pData, uint16_t Size)
{
	HAL_GPIO_WritePin(uart2_en_GPIO_Port,uart2_en_Pin,GPIO_PIN_SET);
	HAL_UART_Transmit(&huart2,pData,Size,1000);
	HAL_GPIO_WritePin(uart2_en_GPIO_Port,uart2_en_Pin,GPIO_PIN_RESET);
}

//4字节大小端转换
void transfer_4bytes(uint32_t* x)
{	
	uint32_t buff = 0;
	buff = ((*x & 0x0000ffff) << 16) | ((*x & 0xffff0000) >> 16);
	*x = buff;
}
//2字节大小端转换
void transfer_2bytes(uint16_t* x)
{
	uint16_t buff = 0;
	buff = (*x << 8) | (*x >> 8);
	*x = buff;
}
