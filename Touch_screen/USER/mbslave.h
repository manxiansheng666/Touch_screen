#ifndef __MBSLAVE_H
#define __MBSLAVE_H

#include "main.h"
#include "bsp.h"

void modbus_process(uint8_t *buf,uint8_t len,uint8_t station_addr);
#endif


