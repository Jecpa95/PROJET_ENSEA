#include "nrf24l01p.h"
#include "spi.h"
#include "main.h"
#include <stdio.h>


//-----------*Fonctions des PINS et Registres*---------------

//SPI Chip select sans l'activation de Cs le module nrf ne répond pas
static void cs_high()
{

	HAL_GPIO_WritePin(NRF24L01P_SPI_CS_PIN_PORT,NRF24L01P_SPI_CS_PIN_NUMBER,GPIO_PIN_SET);
}


static void cs_low()
{

	HAL_GPIO_WritePin(NRF24L01P_SPI_CS_PIN_PORT,NRF24L01P_SPI_CS_PIN_NUMBER,GPIO_PIN_RESET);
}

//CE sert à activer le RX/TX mode
static void ce_high()
{

	HAL_GPIO_WritePin(NRF24L01P_SPI_CE_PIN_PORT,NRF24L01P_SPI_CE_PIN_NUMBER,GPIO_PIN_SET);
}


static void ce_low()
{

	HAL_GPIO_WritePin(NRF24L01P_SPI_CE_PIN_PORT,NRF24L01P_SPI_CE_PIN_NUMBER,GPIO_PIN_RESET);
}

//fonction qui permet d'aller lire dans un registre
/* - reg : numero du registre à lire
 * - return value : valeur du registre*/
static uint8_t read_register (uint8_t reg)
{

	uint8_t command = NRF24L01P_CMD_R_REGISTER | reg;
	uint8_t status, read_val;

	cs_low();
	HAL_SPI_TransmitReceive(NRF24L01P_SPI,&command,&status,1,HAL_MAX_DELAY);
	HAL_SPI_Receive(NRF24L01P_SPI,&read_val,1,2000);
	cs_high();

	return read_val;
}

//fonction qui permet d'écrire dans un registre
/* - reg : numero du registre où écrire
 * - val : valeur à écrire*/
static void write_register (uint8_t reg,uint8_t val)
{

	uint8_t command = NRF24L01P_CMD_W_REGISTER | reg;
	uint8_t status;
	uint8_t write_val = val;

	cs_low();
	HAL_SPI_TransmitReceive(NRF24L01P_SPI,&command,&status,1,HAL_MAX_DELAY);
	HAL_SPI_Transmit(NRF24L01P_SPI,&write_val,1,2000);
	cs_high();

	//return status;
}

//fonction qui permet de lire un multi-octet d'un registre
/* - reg : numero du registre où écrire
 * - pBuf : pointeur de la data à écrire
 * - count : nombre d'octets à écrire*/
static void write_MBRegister(uint8_t reg,uint8_t* pBuf, uint8_t count)
{
	while(count--)
	{
		write_register(reg,*pBuf++);
	}

}


//fonction qui permet d'écrire un multi-octet d'un registre
/* - reg : numero du registre où lire
 * - pBuf : pointeur de la data à lire
 * - count : nombre d'octets à lire*/
static void read_MBRegister(uint8_t reg,uint8_t* pBuf, uint8_t count)
{
	while(count--)
	{
		*pBuf++ =read_register(reg);
	}

}


//-----------*Sous-Fonctions des Registres, de Debug*---------------

//bit 0 (PRIM_RX) du registre CONFIG à mettre à 1 pour Rx control : PRX = 1
void nrf24l01p_prx_mode()
{
	uint8_t new_conf = read_register(NRF24L01P_REG_CONFIG);
	new_conf |= 1 << 0;        //la valeur du registre + 1 pour garder le contenu précédent

	write_register(NRF24L01P_REG_CONFIG, new_conf);
}

//bit 0 (PRIM_RX) du registre CONFIG à mettre à 0 pour Tx control : PTX = 0
void nrf24l01p_ptx_mode()
{
	uint8_t new_conf = read_register(NRF24L01P_REG_CONFIG);
	new_conf &= 0xFE;       //la valeur du registre x 0b11111110 pour garder le contenu précédent

	write_register(NRF24L01P_REG_CONFIG, new_conf);
}


void nrf24l01p_flush_rx_fifo()
{
	uint8_t command = NRF24L01P_CMD_FLUSH_RX;
	uint8_t status;

	cs_low();
	HAL_SPI_TransmitReceive(NRF24L01P_SPI,&command,&status,1,HAL_MAX_DELAY);
	cs_high();
}

void nrf24l01p_flush_tx_fifo()
{
	uint8_t command = NRF24L01P_CMD_FLUSH_TX;
	uint8_t status;

	cs_low();
	HAL_SPI_TransmitReceive(NRF24L01P_SPI,&command,&status,1,HAL_MAX_DELAY);
	cs_high();
}

//fonction de reset du module nrf24l01
void nrf24l01p_reset()
{
	uint8_t buffer[5] = {0xE7,0xE7,0xE7,0xE7,0xE7};
	uint8_t buf[5] = {0xC2,0xC2,0xC2,0xC2,0xC2};
	//reset des pins
	cs_high();
	ce_low();

	//reset des registres
	write_register(NRF24L01P_REG_CONFIG,0x08);
	write_register(NRF24L01P_REG_EN_AA,0x6F);
	write_register(NRF24L01P_REG_EN_RXADDR,0x03);
	write_register(NRF24L01P_REG_SETUP_AW,0x03);
	write_register(NRF24L01P_REG_SETUP_RETR,0x03);
	write_register(NRF24L01P_REG_RF_CH,0x02);
	write_register(NRF24L01P_REG_RF_SETUP,0x0E);
	write_register(NRF24L01P_REG_STATUS,0x0E);
	write_MBRegister(NRF24L01P_REG_RX_ADDR_P0,buffer,5);
	write_MBRegister(NRF24L01P_REG_RX_ADDR_P1,buf,5);
	write_register(NRF24L01P_REG_RX_ADDR_P2,0xC3);
	write_register(NRF24L01P_REG_RX_ADDR_P3,0xC4);
	write_register(NRF24L01P_REG_RX_ADDR_P4,0xC5);
	write_register(NRF24L01P_REG_RX_ADDR_P5,0xC6);
	write_register(NRF24L01P_REG_RX_PW_P0,0x00);
	write_register(NRF24L01P_REG_RX_PW_P1,0x00);
	write_register(NRF24L01P_REG_RX_PW_P2,0x00);
	write_register(NRF24L01P_REG_RX_PW_P3,0x00);
	write_register(NRF24L01P_REG_RX_PW_P4,0x00);
	write_register(NRF24L01P_REG_RX_PW_P5,0x00);
	write_register(NRF24L01P_REG_FIFO_STATUS,0x11);
	write_register(NRF24L01P_REG_DYNPD,0x00);
	write_register(NRF24L01P_REG_FEATURE,0x00);

	//reset des FIFOs
	nrf24l01p_flush_tx_fifo();
	nrf24l01p_flush_rx_fifo();


}


// Check if the nRF24L01 present
// return:
//   1 - nRF24L01 is online and responding
//   0 - received sequence differs from original
uint8_t nrf24l01p_Check(void) {
	uint8_t rxbuf[5];
	uint8_t i;
	uint8_t *ptr = (uint8_t *)nRF24_TEST_ADDR;

	// Write test TX address and read TX_ADDR register
	write_MBRegister(NRF24L01P_REG_TX_ADDR, ptr, 5);
	read_MBRegister(NRF24L01P_REG_TX_ADDR, rxbuf, 5);

	// Compare buffers, return error on first mismatch
	for (i = 0; i < 5; i++) {
		if (rxbuf[i] != *ptr++) return 0;
	}

	return 1;
}



//On du module nrf : PWR_UP à 1 dans le registre CONFIG
void nrf24l01p_pwr_up()
{
	uint8_t new_conf = read_register(NRF24L01P_REG_CONFIG);
	new_conf |= 1 << 1;
	write_register(NRF24L01P_REG_CONFIG,new_conf);
}



//Off du module nrf : PWR_DOWN à 0 dans le registre CONFIG
void nrf24l01p_pwr_down()
{
	uint8_t new_conf = read_register(NRF24L01P_REG_CONFIG);
	new_conf &= 0xFD;
	write_register(NRF24L01P_REG_CONFIG,new_conf);
}


//Ecriture dans la FIFO :  en cas de réception d'un message
/**/
/*Quand le module nrf est en réception, l'opération
 * qui s'exécute sur le SPI est une lecture.
 * La donnée reçue se trouve sur le bus MISO*/
uint8_t nrf24l01p_read_rx_fifo (uint8_t rx_payload)
{
	uint8_t command = NRF24L01P_CMD_R_RX_PAYLOAD;
	uint8_t status;

	cs_low();
	HAL_SPI_TransmitReceive(NRF24L01P_SPI,&command,&status,1,HAL_MAX_DELAY);
	HAL_SPI_Receive(NRF24L01P_SPI,&rx_payload,NRF24L01P_PAYLOAD_LENGTH,HAL_MAX_DELAY);
	cs_high();

	return status;
}

//Lecture de la FIFO : en cas de transmission d'un message
/*Quand le module nrf fait une transmission, l'opération
 * qui s'exécute sur le SPI est une écriture.
 * La donnée envoyée se trouve sur le bus MOSI*/
uint8_t nrf24l01p_write_tx_fifo (uint8_t tx_payload)
{
	uint8_t command = NRF24L01P_CMD_W_TX_PAYLOAD;
	uint8_t status;

	cs_low();
	HAL_SPI_TransmitReceive(NRF24L01P_SPI,&command,&status,1,HAL_MAX_DELAY);
	HAL_SPI_Transmit(NRF24L01P_SPI,&tx_payload,NRF24L01P_PAYLOAD_LENGTH,HAL_MAX_DELAY);
	cs_high();

	return status;

}

//Fonction permettant d'avoir le status du module nrf
uint8_t nrf24l01p_get_status()
{
	uint8_t command = NRF24L01P_CMD_NOP;
	uint8_t status;

	cs_low();
	HAL_SPI_TransmitReceive(NRF24L01P_SPI,&command,&status,1,HAL_MAX_DELAY);
	cs_high();

	return status;
}

//Fonction permettant d'avoir le status de la FIFO
uint8_t nrf24l01p_get_fifo_status()
{
	return read_register(NRF24L01P_REG_FIFO_STATUS);
}



void nrf24l01p_rx_set_payload_widths(uint8_t reg_Pipe, widths bytes)
{
	write_register(reg_Pipe, bytes);
}



/*Data ready Rx FIFO interrupt :confirmation d'interruption quand une nouvelle donnée arrive.
Bit 6 du registre status à mettre à 1 pour l'initialiser*/
void nrf24l01p_clear_rx_dr()
{
	uint8_t new_status = nrf24l01p_get_status();
	new_status |= 0x40;

	write_register(NRF24L01P_REG_STATUS, new_status);
}



/*Data sent Tx FIFO interrupt :confirmation quand un paquet est transmis.
Bit 5 du registre status à mettre à 1 pour l'initialiser*/
void nrf24l01p_clear_tx_ds()
{
	uint8_t new_status = nrf24l01p_get_status();
	new_status |= 0x20;

	write_register(NRF24L01P_REG_STATUS, new_status);
}


/*Maximum number of Tx retransmits interruprt.
 * Bit 4 du registre status à mettre à 1 pour permettre d'autres communications si Max_RT est confirmé*/
void nrf24l01p_clear_max_rt()
{
	uint8_t new_status = nrf24l01p_get_status();
	new_status |= 0x10;

	write_register(NRF24L01P_REG_STATUS, new_status);
}


//CRC encoding scheme. Bit 2 du registre config pour déterminer la taille d'octets à vérifier
void nrf24l01p_set_crc_length(length bytes)
{
	uint8_t new_conf = read_register(NRF24L01P_REG_CONFIG);

	switch (bytes)
	{
	//CRCO à mettre à 0 quand la taille est d'un octet
	case 1 : new_conf &= 0xFB; break;

	//CRCO à mettre à 1 quand la taille est de 2 octets
	case 2 : new_conf |= 1<<2; break;
	}

	write_register(NRF24L01P_REG_CONFIG,new_conf);
}


//Registre SETUP_AW : setup of address widths. Les valeurs possibles : 3, 4 ou 5 octets
void nrf24l01p_set_adr_widths(widths bytes)
{
	write_register(NRF24L01P_REG_SETUP_AW,bytes - 2);
}


//Comptage du nombre de auto-retransmission en cas d'échec
void nrf24l01p_auto_retransmit_count(count cnt)
{
	uint8_t new_setup_rtr = read_register(NRF24L01P_REG_SETUP_RETR);
	new_setup_rtr |= 0xF0;
	new_setup_rtr |= cnt;

	write_register(NRF24L01P_REG_SETUP_RETR,new_setup_rtr);

}

//delai d'auto-retransmission (sur 4 bits)
void nrf24l01p_auto_retransmit_delay(delay us)
{
	uint8_t new_setup_rtr = read_register(NRF24L01P_REG_SETUP_RETR);
	new_setup_rtr |= 0x0F;
	new_setup_rtr |= (us/250 - 1)<<4;

	write_register(NRF24L01P_REG_SETUP_RETR,new_setup_rtr);
}

//Set frequency channel for nrf operations
void nrf24l01p_set_rf_channel(channel Mhz)
{
	uint16_t new_rf_ch = Mhz - 2400; //F =2400 + rf_ch

	write_register(NRF24L01P_REG_RF_CH,new_rf_ch);
}



//Set output power for Tx Mode
void nrf24l01p_set_rf_tx_output_pwr(output_power dBm)
{
	uint8_t new_rf_setup = read_register(NRF24L01P_REG_RF_SETUP);
	new_rf_setup &= 0xF9;
	new_rf_setup |= dBm<<1;

	write_register(NRF24L01P_REG_RF_SETUP,new_rf_setup);

}

//Set data transmission speed
void nrf24l01p_set_rf_data_rate(data_rate bps)
{
	uint8_t new_rf_setup = read_register(NRF24L01P_REG_RF_SETUP);
	new_rf_setup &= 0xD7;

	switch(bps)
	{
	case _1Mbps : break;

	case _2Mbps : new_rf_setup |= 1 <<3; break;

	case _250kbps : new_rf_setup |= 1 <<5; break;
	}

	write_register(NRF24L01P_REG_RF_SETUP,new_rf_setup);
}

// Configure a specified RX pipe
// input:
//   pipe - number of the RX pipe, value from 0 to 5
//   aa_state - state of auto acknowledgment, one of nRF24_AA_xx values
//   payload_len - payload length in bytes
void nrf24l01p_set_Rx_Pipe(uint8_t pipe, uint8_t aa_state, uint8_t payload_len) {
	uint8_t reg;

	// Enable the specified pipe (EN_RXADDR register)
	reg = read_register(NRF24L01P_REG_EN_RXADDR);
	reg &= 0x3F;
	reg |= 1 << pipe;
	write_register(NRF24L01P_REG_EN_RXADDR, reg);

	// Set RX payload length (RX_PW_Px register,length)
	nrf24l01p_rx_set_payload_widths(NRF24L01P_RX_PW_PIPE[pipe],payload_len);

	// Set auto acknowledgment for a specified pipe (EN_AA register)
	reg = read_register(NRF24L01P_REG_EN_AA);
	if (aa_state == 1) {
		reg |=  (1 << pipe);
	} else {
		reg &= ~(1 << pipe);
	}
	write_register(NRF24L01P_REG_EN_AA, reg);
}

// Disable specified RX pipe
// input:
//   PIPE - number of RX pipe, value from 0 to 5
void nrf24l01p_close_Pipe(uint8_t pipe) {
	uint8_t reg;

	reg  = read_register(NRF24L01P_REG_EN_RXADDR);
	reg &= 0x3F;
	reg &= ~(1 << pipe);

	write_register(NRF24L01P_REG_EN_RXADDR, reg);
}

// Enable the auto retransmit (a.k.a. enhanced ShockBurst) for the specified RX pipe
// input:
//   pipe - number of the RX pipe, value from 0 to 5
void nrf24l01p_EnableAA(uint8_t pipe) {
	uint8_t reg;

	// Set bit in EN_AA register
	reg  = read_register(NRF24L01P_REG_EN_AA);
	reg |= (1 << pipe);
	write_register(NRF24L01P_REG_EN_AA, reg);
}

// Disable the auto retransmit (a.k.a. enhanced ShockBurst) for one or all RX pipes
// input:
//   pipe - number of the RX pipe, value from 0 to 5, any other value will disable AA for all RX pipes
void nrf24l01p_DisableAA(uint8_t pipe) {
	uint8_t reg;

	if (pipe > 5) {
		// Disable Auto-ACK for ALL pipes
		write_register(NRF24L01P_REG_EN_AA, 0x00);
	} else {
		// Clear bit in the EN_AA register
		reg  = read_register(NRF24L01P_REG_EN_AA);
		reg &= ~(1 << pipe);
		write_register(NRF24L01P_REG_EN_AA, reg);
	}
}


//----------------------*Main Functions*-----------------------
//Initialisation de la réception
void nrf24l01p_rx_init(channel MHz, data_rate bps, uint8_t pipe)
{
	//Reset du module nrf
	nrf24l01p_reset();

	//Activation du mode réception
	nrf24l01p_prx_mode();
	nrf24l01p_pwr_up();

	//taille du message
	nrf24l01p_rx_set_payload_widths(NRF24L01P_RX_PW_PIPE[pipe],NRF24L01P_PAYLOAD_LENGTH);

	//Configuration de RF Channel
	nrf24l01p_set_rf_channel(MHz);
	nrf24l01p_set_rf_data_rate(bps);
	nrf24l01p_set_rf_tx_output_pwr(_0dBm);

	//Taille du message (en octet)
	nrf24l01p_set_crc_length(1);
	nrf24l01p_set_adr_widths(5);


	//autorisation de fonctionnement
	ce_high();
}


//Initialisation de la transmission
void nrf24l01p_tx_init(channel MHz, data_rate bps,uint8_t pipe)
{
	//Reset du module nrf
	nrf24l01p_reset();

	//Activation du mode transmission
	nrf24l01p_ptx_mode();
	nrf24l01p_pwr_up();

	//taille du message
	nrf24l01p_rx_set_payload_widths(NRF24L01P_RX_PW_PIPE[pipe],NRF24L01P_PAYLOAD_LENGTH);

	//Configuration de RF Channel
	nrf24l01p_set_rf_channel(MHz);
	nrf24l01p_set_rf_data_rate(bps);
	nrf24l01p_set_rf_tx_output_pwr(_0dBm);

	//Taille du message (en octet)
	nrf24l01p_set_crc_length(1);
	nrf24l01p_set_adr_widths(5);

	//désactivation de la retransmission automatique pour toutes les pipes
	nrf24l01p_DisableAA(6);

	//autorisation de fonctionnement
	ce_high();

}


//réception
void nrf24l01p_rx_receive(uint8_t rx_payload)
{
	nrf24l01p_read_rx_fifo(rx_payload);
	nrf24l01p_clear_rx_dr();

	HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_1);
}


//transmission
void nrf24l01p_tx_transmit(uint8_t rx_payload)
{
	//uint8_t status;

	nrf24l01p_write_tx_fifo(rx_payload);
	//printf("Status : %d \r\n",status);
}

void nrf24l01p_tx_transmitMB(uint8_t* rx_tab,uint8_t n)
{
	while (n--)
	{
		nrf24l01p_write_tx_fifo(*rx_tab++);
	}

}

void nrf24l01p_tx_irq()
{
	uint8_t tx_ds = nrf24l01p_get_status();
	tx_ds &= 0x20;

	if(tx_ds)
	{
		// TX_DS
		HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_0);
		nrf24l01p_clear_tx_ds();
	}

	else
	{
		// MAX_RT
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, SET);
		nrf24l01p_clear_max_rt();
	}
}
