/*
 * nrf24l01p.h
 *
 *  Created on: Mar 30, 2023
 *      Author: farimathomas
 */

#ifndef INC_NRF24L01P_H_
#define INC_NRF24L01P_H_

#include "spi.h"    // header from stm32cubemx code generate
#include <stdbool.h>

/* User Configurations */
#define NRF24L01P_SPI                     &hspi1

#define NRF24L01P_SPI_CS_PIN_PORT         GPIOB
#define NRF24L01P_SPI_CS_PIN_NUMBER       GPIO_PIN_9

#define NRF24L01P_SPI_CE_PIN_PORT             GPIOB
#define NRF24L01P_SPI_CE_PIN_NUMBER           GPIO_PIN_8

#define NRF24L01P_IRQ_PIN_PORT            GPIOA
#define NRF24L01P_IRQ_PIN_NUMBER          GPIO_PIN_3

#define NRF24L01P_PAYLOAD_LENGTH          8     // 1 - 32bytes

// Fake address to test transceiver presence (5 bytes long)
#define nRF24_TEST_ADDR            "nRF24"

/* nRF24L01+ typedefs */
typedef uint8_t count;
typedef uint8_t widths;
typedef uint8_t length;
typedef uint16_t delay;
typedef uint16_t channel;

typedef  enum {
	_0dBm = 3,
	_6dBm = 2,
	_12dBm = 1,
	_18dBm =0
}output_power;

typedef enum {
	_1Mbps = 0,
	_2Mbps = 1,
	_250kbps = 2
}data_rate;

// Enumeration of RX pipe addresses and TX address
enum {
	pipe0  = (uint8_t)0x00, // pipe0
	pipe1  = (uint8_t)0x01, // pipe1
	pipe2  = (uint8_t)0x02, // pipe2
	pipe3  = (uint8_t)0x03, // pipe3
	pipe4  = (uint8_t)0x04, // pipe4
	pipe5  = (uint8_t)0x05, // pipe5
	pipe6  = (uint8_t)0x06  // TX address (not a pipe in fact)
};

/* nRF24L01+ Commands */
#define NRF24L01P_CMD_R_REGISTER                  0x000AAAAA
#define NRF24L01P_CMD_W_REGISTER                  0x001AAAAA
#define NRF24L01P_CMD_R_RX_PAYLOAD                0b01100001
#define NRF24L01P_CMD_W_TX_PAYLOAD                0b10100000
#define NRF24L01P_CMD_FLUSH_TX                    0b11100001
#define NRF24L01P_CMD_FLUSH_RX                    0b11100010
#define NRF24L01P_CMD_REUSE_TX_PL                 0b11100011
#define NRF24L01P_CMD_R_RX_PL_WID                 0b01100000
#define NRF24L01P_CMD_W_ACK_PAYLOAD               0b10101000
#define NRF24L01P_CMD_W_TX_PAYLOAD_NOACK          0b10110000
#define NRF24L01P_CMD_NOP                         0b11111111

/* nRF24L01+ Registers */
#define NRF24L01P_REG_CONFIG            0x00
#define NRF24L01P_REG_EN_AA             0x01
#define NRF24L01P_REG_EN_RXADDR         0x02
#define NRF24L01P_REG_SETUP_AW          0x03
#define NRF24L01P_REG_SETUP_RETR        0x04
#define NRF24L01P_REG_RF_CH             0x05
#define NRF24L01P_REG_RF_SETUP          0x06
#define NRF24L01P_REG_STATUS            0x07
#define NRF24L01P_REG_OBSERVE_TX        0x08    // Read-Only
#define NRF24L01P_REG_RPD               0x09    // Read-Only
#define NRF24L01P_REG_RX_ADDR_P0        0x0A
#define NRF24L01P_REG_RX_ADDR_P1        0x0B
#define NRF24L01P_REG_RX_ADDR_P2        0x0C
#define NRF24L01P_REG_RX_ADDR_P3        0x0D
#define NRF24L01P_REG_RX_ADDR_P4        0x0E
#define NRF24L01P_REG_RX_ADDR_P5        0x0F
#define NRF24L01P_REG_TX_ADDR           0x10
#define NRF24L01P_REG_RX_PW_P0          0x11
#define NRF24L01P_REG_RX_PW_P1          0x12
#define NRF24L01P_REG_RX_PW_P2          0x13
#define NRF24L01P_REG_RX_PW_P3          0x14
#define NRF24L01P_REG_RX_PW_P4          0x15
#define NRF24L01P_REG_RX_PW_P5          0x16
#define NRF24L01P_REG_FIFO_STATUS       0x17
#define NRF24L01P_REG_DYNPD             0x1C
#define NRF24L01P_REG_FEATURE           0x1D






// Addresses of the RX_PW_P# registers
static const uint8_t NRF24L01P_RX_PW_PIPE[6] = {
		NRF24L01P_REG_RX_PW_P0,
		NRF24L01P_REG_RX_PW_P1,
		NRF24L01P_REG_RX_PW_P2,
		NRF24L01P_REG_RX_PW_P3,
		NRF24L01P_REG_RX_PW_P4,
		NRF24L01P_REG_RX_PW_P5
};

void nrf24l01p_prx_mode();
void nrf24l01p_ptx_mode();
void nrf24l01p_flush_rx_fifo();
void nrf24l01p_flush_tx_fifo();
void nrf24l01p_reset();
void nrf24l01p_pwr_up();
void nrf24l01p_pwr_down();
uint8_t nrf24l01p_read_rx_fifo (uint8_t rx_payload);
uint8_t nrf24l01p_write_tx_fifo (uint8_t tx_payload);
uint8_t nrf24l01p_get_status();
uint8_t nrf24l01p_get_fifo_status();
void nrf24l01p_rx_set_payload_widths(uint8_t reg_Pipe, widths bytes);
void nrf24l01p_clear_rx_dr();
void nrf24l01p_clear_tx_ds();
void nrf24l01p_clear_max_rt();
void nrf24l01p_set_crc_length(length bytes);
void nrf24l01p_set_adr_widths(widths bytes);
void nrf24l01p_auto_retransmit_count(count cnt);
void nrf24l01p_auto_retransmit_delay(delay us);
void nrf24l01p_set_rf_channel(channel Mhz);
void nrf24l01p_set_rf_tx_output_pwr(output_power dBm);
void nrf24l01p_set_rf_data_rate(data_rate bps);
void nrf24l01p_rx_init(channel MHz, data_rate bps,uint8_t pipe);
void nrf24l01p_tx_init(channel MHz, data_rate bps,uint8_t pipe);
void nrf24l01p_tx_transmit(uint8_t rx_payload);
void nrf24l01p_tx_transmitMB(uint8_t* rx_tab,uint8_t n);
void nrf24l01p_tx_irq();
void nrf24l01p_set_Rx_Pipe(uint8_t pipe, uint8_t aa_state, uint8_t payload_len);
void nrf24l01p_close_Pipe(uint8_t pipe);
void nrf24l01p_EnableAA(uint8_t pipe);
void nrf24l01p_DisableAA(uint8_t pipe);

#endif /* INC_NRF24L01P_H_ */
