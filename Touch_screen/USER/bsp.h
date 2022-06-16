
#ifndef __BSP_H
#define __BSP_H

#include "usart.h"
#include "adc.h"

void uart2_send_by485(uint8_t *pData, uint16_t Size);
float get_temper(void);

#endif
