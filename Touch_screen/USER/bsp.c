#include "bsp.h"

uint8_t RS485_RX_BUF[64];
uint8_t RS485_RX_CNT=0;  
uint8_t rx_byte;

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

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance == USART2)
	{
		if(RS485_RX_CNT<64)
		{
			RS485_RX_BUF[RS485_RX_CNT] = rx_byte;		//记录接收到的值
			RS485_RX_CNT++;						//接收数据增加1 
		} 
		HAL_UART_Receive_IT(&huart2,&rx_byte,1);
	}
}

void RS485_Receive_Data(uint8_t *buf,uint8_t *len)
{
	uint8_t rxlen=RS485_RX_CNT;
	uint8_t i=0;
	*len=0;				//默认为0
	HAL_Delay(10);		//等待10ms,连续超过10ms没有接收到一个数据,则认为接收结束
	if(rxlen==RS485_RX_CNT&&rxlen)//接收到了数据,且接收完成了
	{
		for(i=0;i<rxlen;i++)
		{
			buf[i]=RS485_RX_BUF[i];	
		}		
		*len=RS485_RX_CNT;	//记录本次数据长度
		RS485_RX_CNT=0;		//清零
	}
}

