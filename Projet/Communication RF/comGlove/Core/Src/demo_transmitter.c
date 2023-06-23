/*
 * demo_transmitter.c
 *
 *  Created on: May 10, 2023
 *      Author: farimathomas
 */

#include "nrf24l01p.h"
#include "spi.h"
#include "main.h"
#include "usart.h"
#include "demo_transmitter.h"


uint8_t tx_data[NRF24L01P_PAYLOAD_LENGTH] = {0, 1, 2, 3, 4, 5, 6, 7};

//void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);

void transmitter (void)
{
	nrf24l01p_tx_init(2500,_1Mbps,pipe0);
	printf("***Debut de la transmission***\r\n");


    while (1)
    {
        //Transmission de packets d'un octet
        for(int i= 0; i < 8; i++)
        {
        	tx_data[i]++;
        	nrf24l01p_tx_transmit(tx_data[i]);
        	printf("data sent : %d \r\n",tx_data[i]);
        	printf("status fifo : %d \r\n",nrf24l01p_get_fifo_status());

        }

        //Transmission de packets plusieurs octets
       /* nrf24l01p_tx_transmitMB((uint8_t *)tx_data,8);
        for(int i= 0; i < 8; i++)
                {
                	//tx_data[i]++;
                	//nrf24l01p_tx_transmit(tx_data[i]);
                	printf("data sent : %d \r\n",tx_data[i]);

                }*/


        // transmit
        HAL_Delay(100);
    }
}

/*
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if(GPIO_Pin == NRF24L01P_IRQ_PIN_NUMBER)
		nrf24l01p_tx_irq(); // clear interrupt flag
}
*/


