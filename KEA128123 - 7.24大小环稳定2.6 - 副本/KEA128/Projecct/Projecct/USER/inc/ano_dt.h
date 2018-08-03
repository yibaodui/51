#ifndef __ANO_DT_H
#define __ANO_DT_H 			   
#include "common.h" 


void nrf_send_ANO(uint8 fun,uint8*data,uint8 len);
void usart_send_ANO(uint8 fun,uint8*data,uint8 len);

void ANO_send_data(short aacx,short aacy,short aacz,short gyrox,short gyroy,short gyroz,short roll,short pitch,short yaw);
void ano_dt(short roll,short pitch,short yaw,int high);

#endif



