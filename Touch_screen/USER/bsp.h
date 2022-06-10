#ifndef __BSP_H
#define __BSP_H

#include "usart.h"

void uart2_send_by485(uint8_t *pData, uint16_t Size);
void transfer_4bytes(uint32_t* x);
void transfer_2bytes(uint16_t* x);

#endif

