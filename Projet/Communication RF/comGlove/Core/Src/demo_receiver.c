/*
 * demo_receiver.c
 *
 *  Created on: May 11, 2023
 *      Author: farimathomas
 */


#include "nrf24l01p.h"
#include "spi.h"
#include "main.h"
#include "usart.h"
#include "demo_receiver.h"
#include <stdio.h>


// data array to be read
uint8_t rx_data[NRF24L01P_PAYLOAD_LENGTH] = { 0, };
int i;


// for tx interrupt
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);



void receiver()
{
	nrf24l01p_rx_init(2500, _1Mbps,pipe0);
	printf("***Debut de la reception***\r\n");

	while (1)
	    {

		for (i=0;i<8;i++)
				{
					nrf24l01p_rx_receive(rx_data[i]); // read data when data ready flag is set
					printf("data received : %d\r\n",rx_data[i]);
				}
	        HAL_Delay(100);
	    }
}




void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if(GPIO_Pin == NRF24L01P_IRQ_PIN_NUMBER){
		// Nothing to do

	}
}

