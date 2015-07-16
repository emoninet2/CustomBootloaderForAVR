/*
 * nrf24l01p.h
 *
 * Created: 4/19/2015 7:11:59 PM
 *  Author: Emon
 */ 






#ifndef NRF24L01P_H_
#define NRF24L01P_H_

#include <util/delay.h>


#include <stdio.h>
#include <stdlib.h>



#define set_bit(port,bit) port|= (1<<bit);
#define clr_bit(port,bit) port&= ~(1<<bit);
#define tgl_bit(port,bit) port^= (1<<bit);


#include "spi.h"


// #define CSN_DDR DDRB
// #define CSN_PORT PORTB
// #define CSN_PIN_BIT	1
// 
// #define CE_DDR	DDRB
// #define CE_PORT	PORTB
// #define CE_PIN_BIT	0

//////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////MBED DEFINTIONS
#define _NRF24L01P_TX_FIFO_COUNT   3
#define _NRF24L01P_RX_FIFO_COUNT   3

#define _NRF24L01P_TX_FIFO_SIZE   32
#define _NRF24L01P_RX_FIFO_SIZE   32

#define _NRF24L01P_SPI_MAX_DATA_RATE     10000000

#define _NRF24L01P_SPI_CMD_RD_REG            0x00
#define _NRF24L01P_SPI_CMD_WR_REG            0x20
#define _NRF24L01P_SPI_CMD_RD_RX_PAYLOAD     0x61
#define _NRF24L01P_SPI_CMD_WR_TX_PAYLOAD     0xa0
#define _NRF24L01P_SPI_CMD_FLUSH_TX          0xe1
#define _NRF24L01P_SPI_CMD_FLUSH_RX          0xe2
#define _NRF24L01P_SPI_CMD_REUSE_TX_PL       0xe3
#define _NRF24L01P_SPI_CMD_R_RX_PL_WID       0x60
#define _NRF24L01P_SPI_CMD_W_ACK_PAYLOAD     0xa8
#define _NRF24L01P_SPI_CMD_W_TX_PYLD_NO_ACK  0xb0
#define _NRF24L01P_SPI_CMD_NOP               0xff


#define _NRF24L01P_REG_CONFIG                0x00
#define _NRF24L01P_REG_EN_AA                 0x01
#define _NRF24L01P_REG_EN_RXADDR             0x02
#define _NRF24L01P_REG_SETUP_AW              0x03
#define _NRF24L01P_REG_SETUP_RETR            0x04
#define _NRF24L01P_REG_RF_CH                 0x05
#define _NRF24L01P_REG_RF_SETUP              0x06
#define _NRF24L01P_REG_STATUS                0x07
#define _NRF24L01P_REG_OBSERVE_TX            0x08
#define _NRF24L01P_REG_RPD                   0x09
#define _NRF24L01P_REG_RX_ADDR_P0            0x0a
#define _NRF24L01P_REG_RX_ADDR_P1            0x0b
#define _NRF24L01P_REG_RX_ADDR_P2            0x0c
#define _NRF24L01P_REG_RX_ADDR_P3            0x0d
#define _NRF24L01P_REG_RX_ADDR_P4            0x0e
#define _NRF24L01P_REG_RX_ADDR_P5            0x0f
#define _NRF24L01P_REG_TX_ADDR               0x10
#define _NRF24L01P_REG_RX_PW_P0              0x11
#define _NRF24L01P_REG_RX_PW_P1              0x12
#define _NRF24L01P_REG_RX_PW_P2              0x13
#define _NRF24L01P_REG_RX_PW_P3              0x14
#define _NRF24L01P_REG_RX_PW_P4              0x15
#define _NRF24L01P_REG_RX_PW_P5              0x16
#define _NRF24L01P_REG_FIFO_STATUS           0x17
#define _NRF24L01P_REG_DYNPD                 0x1c
#define _NRF24L01P_REG_FEATURE               0x1d

#define _NRF24L01P_REG_ADDRESS_MASK          0x1f

// CONFIG register:
#define _NRF24L01P_CONFIG_PRIM_RX        (1<<0)
#define _NRF24L01P_CONFIG_PWR_UP         (1<<1)
#define _NRF24L01P_CONFIG_CRC0           (1<<2)
#define _NRF24L01P_CONFIG_EN_CRC         (1<<3)
#define _NRF24L01P_CONFIG_MASK_MAX_RT    (1<<4)
#define _NRF24L01P_CONFIG_MASK_TX_DS     (1<<5)
#define _NRF24L01P_CONFIG_MASK_RX_DR     (1<<6)

#define _NRF24L01P_CONFIG_CRC_MASK       (_NRF24L01P_CONFIG_EN_CRC|_NRF24L01P_CONFIG_CRC0)
#define _NRF24L01P_CONFIG_CRC_NONE       (0)
#define _NRF24L01P_CONFIG_CRC_8BIT       (_NRF24L01P_CONFIG_EN_CRC)
#define _NRF24L01P_CONFIG_CRC_16BIT      (_NRF24L01P_CONFIG_EN_CRC|_NRF24L01P_CONFIG_CRC0)

// EN_AA register:
#define _NRF24L01P_EN_AA_NONE            0

// EN_RXADDR register:
#define _NRF24L01P_EN_RXADDR_NONE        0

// SETUP_AW register:
#define _NRF24L01P_SETUP_AW_AW_MASK      (0x3<<0)
#define _NRF24L01P_SETUP_AW_AW_3BYTE     (0x1<<0)
#define _NRF24L01P_SETUP_AW_AW_4BYTE     (0x2<<0)
#define _NRF24L01P_SETUP_AW_AW_5BYTE     (0x3<<0)

// SETUP_RETR register:
#define _NRF24L01P_SETUP_RETR_NONE       0

// RF_SETUP register:
#define _NRF24L01P_RF_SETUP_RF_PWR_MASK          (0x3<<1)
#define _NRF24L01P_RF_SETUP_RF_PWR_0DBM          (0x3<<1)
#define _NRF24L01P_RF_SETUP_RF_PWR_MINUS_6DBM    (0x2<<1)
#define _NRF24L01P_RF_SETUP_RF_PWR_MINUS_12DBM   (0x1<<1)
#define _NRF24L01P_RF_SETUP_RF_PWR_MINUS_18DBM   (0x0<<1)

#define _NRF24L01P_RF_SETUP_RF_DR_HIGH_BIT       (1 << 3)
#define _NRF24L01P_RF_SETUP_RF_DR_LOW_BIT        (1 << 5)
#define _NRF24L01P_RF_SETUP_RF_DR_MASK           (_NRF24L01P_RF_SETUP_RF_DR_LOW_BIT|_NRF24L01P_RF_SETUP_RF_DR_HIGH_BIT)
#define _NRF24L01P_RF_SETUP_RF_DR_250KBPS        (_NRF24L01P_RF_SETUP_RF_DR_LOW_BIT)
#define _NRF24L01P_RF_SETUP_RF_DR_1MBPS          (0)
#define _NRF24L01P_RF_SETUP_RF_DR_2MBPS          (_NRF24L01P_RF_SETUP_RF_DR_HIGH_BIT)

// STATUS register:
#define _NRF24L01P_STATUS_TX_FULL        (1<<0)
#define _NRF24L01P_STATUS_RX_P_NO        (0x7<<1)
#define _NRF24L01P_STATUS_MAX_RT         (1<<4)
#define _NRF24L01P_STATUS_TX_DS          (1<<5)
#define _NRF24L01P_STATUS_RX_DR          (1<<6)

// RX_PW_P0..RX_PW_P5 registers:
#define _NRF24L01P_RX_PW_Px_MASK         0x3F

#define _NRF24L01P_TIMING_Tundef2pd_us     100000   // 100mS
#define _NRF24L01P_TIMING_Tstby2a_us          130   // 130uS
#define _NRF24L01P_TIMING_Thce_us              10   //  10uS
#define _NRF24L01P_TIMING_Tpd2stby_us        4500   // 4.5mS worst case
#define _NRF24L01P_TIMING_Tpece2csn_us          4   //   4uS


#define NRF24L01P_TX_PWR_ZERO_DB         0
#define NRF24L01P_TX_PWR_MINUS_6_DB     -6
#define NRF24L01P_TX_PWR_MINUS_12_DB   -12
#define NRF24L01P_TX_PWR_MINUS_18_DB   -18

#define NRF24L01P_DATARATE_250_KBPS    250
#define NRF24L01P_DATARATE_1_MBPS     1000
#define NRF24L01P_DATARATE_2_MBPS     2000

#define NRF24L01P_CRC_NONE               0
#define NRF24L01P_CRC_8_BIT              8
#define NRF24L01P_CRC_16_BIT            16

#define NRF24L01P_MIN_RF_FREQUENCY    2400
#define NRF24L01P_MAX_RF_FREQUENCY    2525

#define NRF24L01P_PIPE_P0                0
#define NRF24L01P_PIPE_P1                1
#define NRF24L01P_PIPE_P2                2
#define NRF24L01P_PIPE_P3                3
#define NRF24L01P_PIPE_P4                4
#define NRF24L01P_PIPE_P5                5

/**
* Default setup for the nRF24L01+, based on the Sparkfun "Nordic Serial Interface Board"
*  for evaluation (http://www.sparkfun.com/products/9019)
*/
#define DEFAULT_NRF24L01P_ADDRESS       ((unsigned long long) 0xE7E7E7E7E7 )
#define DEFAULT_NRF24L01P_ADDRESS_WIDTH  5
#define DEFAULT_NRF24L01P_CRC            NRF24L01P_CRC_8_BIT
#define DEFAULT_NRF24L01P_RF_FREQUENCY  (NRF24L01P_MIN_RF_FREQUENCY + 2)
#define DEFAULT_NRF24L01P_DATARATE       NRF24L01P_DATARATE_1_MBPS
#define DEFAULT_NRF24L01P_TX_PWR         NRF24L01P_TX_PWR_ZERO_DB
#define DEFAULT_NRF24L01P_TRANSFER_SIZE  4



//////////////////////////////////////////////////////////////////////////
#define dummybyte 0x65
#define CONFIG      0x00
#define EN_AA       0x01
#define EN_RXADDR   0x02
#define SETUP_AW    0x03
#define SETUP_RETR  0x04
#define RF_CH       0x05
#define RF_SETUP    0x06
#define NRF_STATUS		0x07
#define OBSERVE_TX  0x08
#define RPD         0x09
#define RX_ADDR_P0  0x0A
#define RX_ADDR_P1  0x0B
#define RX_ADDR_P2  0x0C
#define RX_ADDR_P3  0x0D
#define RX_ADDR_P4  0x0E
#define RX_ADDR_P5  0x0F
#define TX_ADDR     0x10
#define RX_PW_P0    0x11
#define RX_PW_P1    0x12
#define RX_PW_P2    0x13
#define RX_PW_P3    0x14
#define RX_PW_P4    0x15
#define RX_PW_P5    0x16
#define FIFO_STATUS 0x17
#define DYNPD		0x1C
#define FEATURE		0x1D

/* Bit Mnemonics */
#define MASK_RX_DR  6
#define MASK_TX_DS  5
#define MASK_MAX_RT 4
#define EN_CRC      3
#define CRCO        2
#define PWR_UP      1
#define PRIM_RX     0

#define ENAA_P5     5
#define ENAA_P4     4
#define ENAA_P3     3
#define ENAA_P2     2
#define ENAA_P1     1
#define ENAA_P0     0

#define ERX_P5      5
#define ERX_P4      4
#define ERX_P3      3
#define ERX_P2      2
#define ERX_P1      1
#define ERX_P0      0

#define AW          0

#define ARD         4
#define ARC         0

#define CONT_WAVE	7
#define RF_DR_LOW	6
#define PLL_LOCK    4
#define RF_DR_HIGH  3
#define RF_PWR      1

#define LNA_HCURR   0
#define RX_DR       6
#define TX_DS       5
#define MAX_RT      4
#define RX_P_NO     1
#define TX_FULL     0
#define PLOS_CNT    4
#define ARC_CNT     0
#define TX_REUSE    6
#define FIFO_FULL   5
#define TX_EMPTY    4
#define RX_FULL     1
#define RX_EMPTY    0

#define EN_DPL		2
#define EN_ACK_PAY	1
#define EN_DYN_ACK	0

/* Instruction Mnemonics */
#define R_REGISTER    0x00
#define W_REGISTER    0x20
#define REGISTER_MASK 0x1F
#define R_RX_PAYLOAD  0x61
#define W_TX_PAYLOAD  0xA0
#define FLUSH_TX      0xE1
#define FLUSH_RX      0xE2
#define REUSE_TX_PL   0xE3
#define R_RX_PL_WID	  0x60
#define NOP           0xFF

//////////////////////////////////////////////////////////////////////////
#define _NRF24L01P_TIMING_Tundef2pd_us     100000   // 100mS
#define _NRF24L01P_TIMING_Tstby2a_us          130   // 130uS
#define _NRF24L01P_TIMING_Thce_us              10   //  10uS
#define _NRF24L01P_TIMING_Tpd2stby_us        4500   // 4.5mS worst case
#define _NRF24L01P_TIMING_Tpece2csn_us          4   //   4uS

typedef enum {
	_NRF24L01P_MODE_UNKNOWN,
	_NRF24L01P_MODE_POWER_DOWN,
	_NRF24L01P_MODE_STANDBY,
	_NRF24L01P_MODE_RX,
	_NRF24L01P_MODE_TX,
} nRF24L01P_Mode_Type;


//////////////////////////////////////////////////////////////////////////
uint8_t ce_value = 0;
uint8_t csn_value = 0;
uint8_t mode;

uint8_t nrf24l01p_config;
uint8_t nrf24l01p_en_aa;
uint8_t nrf24l01p_en_rxaddr;
uint8_t nrf24l01p_setup_aw;
uint8_t nrf24l01p_setup_retr;
uint8_t nrf24l01p_rf_ch;
uint8_t nrf24l01p_rf_setup;
uint8_t nrf24l01p_status;
uint8_t nrf24l01p_observe_tx;
uint8_t nrf24l01p_rpd;
uint8_t nrf24l01p_rx_addr_p0[5];
uint8_t nrf24l01p_rx_addr_p1[5];
uint8_t nrf24l01p_rx_addr_p2[1];
uint8_t nrf24l01p_rx_addr_p3[1];
uint8_t nrf24l01p_rx_addr_p4[1];
uint8_t nrf24l01p_rx_addr_p5[1];
uint8_t nrf24l01p_tx_addr[5];
uint8_t nrf24l01p_rx_pw[6];
uint8_t nrf24l01p_fifo_status;
uint8_t nrf24l01p_dynpd;
uint8_t nrf24l01p_feature;

//////////////////////////////////////////////////////////////////////////

void nrf24l01p_init()
{
	spi_master_initialize();
// 	PORTC_DIRSET = (1<<4); //CS pin as output
// 	PORTA_DIRSET = (1<<0); //CE pin as output
}
void ce_pin(int state){
// 	if(state){PORTA_OUTSET = (1<<0);}
// 	else {PORTA_OUTCLR = (1<<0);}
	ce_value = state;
}
void csn_pin(int state){
// 	if(state){PORTC_OUTSET = (1<<4);}
// 	else {PORTC_OUTCLR = (1<<4);}
	csn_value = state;
}
void nrf24l01p_read_register(uint8_t address, uint8_t *dataout, int len)
{
	csn_pin(0);
	
	//spi_master_transmit_byte(nrf24l01p_spi, address&(0x1F));
	spi_master_transmit_byte_val(address&(REGISTER_MASK));
	//uint8_t data[5];
	
	for(int i=0;i<len;i++)
	{
		//spi_master_receive_byte(nrf24l01p_spi,&dataout[i]);
		spi_master_receive_byte_ref(dataout+ i);
		//usarte0_send(data[i]);
	}
	
	csn_pin(1);
}
void nrf24l01p_write_register(uint8_t address, uint8_t *datain, int len)
{
	csn_pin(0);
	

	spi_master_transmit_byte_val(( W_REGISTER | (address&(REGISTER_MASK))));
	//uint8_t data[5];
	
	for(int i=0;i<len;i++)
	{
		spi_master_transmit_byte_ref(datain+i);
	}
	
	csn_pin(1);
}
void nrf24l01p_read_payload(uint8_t *dataout, int pay_len)
{
	csn_pin(0);
	
	//spi_master_transmit_byte(nrf24l01p_spi, address&(0x1F));
	spi_master_transmit_byte_val(R_RX_PAYLOAD);
	//uint8_t data[5];
	
	for(int i=0;i<pay_len;i++)
	{
		//spi_master_receive_byte(nrf24l01p_spi,&dataout[i]);
		spi_master_receive_byte_ref(dataout+ i);
		//usarte0_send(data[i]);
	}
	
	csn_pin(1);
}
void nrf24l01p_write_payload(uint8_t *datain, int pay_len)
{
	csn_pin(0);
	

	spi_master_transmit_byte_val(W_TX_PAYLOAD);
	//uint8_t data[5];
	
	for(int i=0;i<pay_len;i++)
	{
		spi_master_transmit_byte_ref(datain+i);
	}
	
	csn_pin(1);
}
void nrf24l01p_flush_tx()
{
	csn_pin(0);
	spi_master_transmit_byte_val(FLUSH_TX);
	csn_pin(1);
}
void nrf24l01p_flush_rx()
{
	csn_pin(0);
	spi_master_transmit_byte_val(FLUSH_RX);
	csn_pin(1);
}
void nrf24l01p_reuse_tx_payload()
{
	csn_pin(0);
	spi_master_transmit_byte_val(REUSE_TX_PL);
	csn_pin(1);
}
void nrf24l01p_read_rx_payload_width(uint8_t *data)
{
	csn_pin(0);
	spi_master_transmit_byte_val(R_RX_PL_WID);
	spi_master_receive_byte_ref(data);
	csn_pin(1);
}

//////////////////////////////////////////////////////////////////////////
						/*REGISTER DESCRIPTIONS*/
//////////////////////////////////////////////////////////////////////////CONFIG
void nrf24l01p_write_config(uint8_t data){
	nrf24l01p_config = data;
	nrf24l01p_write_register(CONFIG,&nrf24l01p_config,sizeof(nrf24l01p_config));
}
uint8_t nrf24l01p_read_config(){
	nrf24l01p_read_register(CONFIG,&nrf24l01p_config,sizeof(nrf24l01p_config));
	return nrf24l01p_config;
}
void nrf24l01p_power_up(){
	nrf24l01p_read_register(CONFIG,&nrf24l01p_config,sizeof(nrf24l01p_config));
	set_bit(nrf24l01p_config,PWR_UP);
	nrf24l01p_write_register(CONFIG,&nrf24l01p_config,sizeof(nrf24l01p_config));
}
void nrf24l01p_power_down(){
	nrf24l01p_read_register(CONFIG,&nrf24l01p_config,sizeof(nrf24l01p_config));
	clr_bit(nrf24l01p_config,PWR_UP);
	nrf24l01p_write_register(CONFIG,&nrf24l01p_config,sizeof(nrf24l01p_config));
}
void nrf24l01p_rx_mode(){
	nrf24l01p_read_register(CONFIG,&nrf24l01p_config,sizeof(nrf24l01p_config));
	set_bit(nrf24l01p_config,PRIM_RX);
	nrf24l01p_write_register(CONFIG,&nrf24l01p_config,sizeof(nrf24l01p_config));
}
void nrf24l01p_tx_mode(){
	nrf24l01p_read_register(CONFIG,&nrf24l01p_config,sizeof(nrf24l01p_config));
	clr_bit(nrf24l01p_config,PRIM_RX);
	nrf24l01p_write_register(CONFIG,&nrf24l01p_config,sizeof(nrf24l01p_config));
}
void nrf24l01p_enable_crc()
{
	nrf24l01p_read_register(CONFIG,&nrf24l01p_config,sizeof(nrf24l01p_config));
	set_bit(nrf24l01p_config,EN_CRC);
	nrf24l01p_write_register(CONFIG,&nrf24l01p_config,sizeof(nrf24l01p_config));
}
void nrf24l01p_disable_crc()
{
	nrf24l01p_read_register(CONFIG,&nrf24l01p_config,sizeof(nrf24l01p_config));
	clr_bit(nrf24l01p_config,EN_CRC);
	nrf24l01p_write_register(CONFIG,&nrf24l01p_config,sizeof(nrf24l01p_config));
}
void nrf24l01p_crc_encoding_scheme(uint8_t scheme)
{
	nrf24l01p_read_register(CONFIG,&nrf24l01p_config,sizeof(nrf24l01p_config));
	if(scheme){set_bit(nrf24l01p_config,CRCO);}
	else{clr_bit(nrf24l01p_config,CRCO);}
	nrf24l01p_write_register(CONFIG,&nrf24l01p_config,sizeof(nrf24l01p_config));
}
//////////////////////////////////////////////////////////////////////////EN_AA
void nrf24l01p_write_en_aa(uint8_t data){
	nrf24l01p_en_aa = data;
	nrf24l01p_write_register(EN_AA,&nrf24l01p_en_aa,sizeof(nrf24l01p_en_aa));
}
uint8_t nrf24l01p_read_en_aa(){
	nrf24l01p_read_register(EN_AA,&nrf24l01p_en_aa,sizeof(nrf24l01p_en_aa));
	return nrf24l01p_en_aa;
}
void nrf24l01p_enable_auto_acknowledge(uint8_t pipe)
{
	nrf24l01p_read_register(EN_AA,&nrf24l01p_en_aa,sizeof(nrf24l01p_en_aa));
	set_bit(nrf24l01p_en_aa,pipe);
	nrf24l01p_write_register(EN_AA,&nrf24l01p_en_aa,sizeof(nrf24l01p_en_aa));
}
void nrf24l01p_disable_auto_acknowledge(uint8_t pipe)
{
	nrf24l01p_read_register(EN_AA,&nrf24l01p_en_aa,sizeof(nrf24l01p_en_aa));
	clr_bit(nrf24l01p_en_aa,pipe);
	nrf24l01p_write_register(EN_AA,&nrf24l01p_en_aa,sizeof(nrf24l01p_en_aa));
}
//////////////////////////////////////////////////////////////////////////EN_RXADDR
void nrf24l01p_write_en_rxaddr(uint8_t data){
	nrf24l01p_en_rxaddr = data;
	nrf24l01p_write_register(EN_RXADDR,&nrf24l01p_en_rxaddr,sizeof(nrf24l01p_en_rxaddr));
}
uint8_t nrf24l01p_read_en_rxaddr(){
	nrf24l01p_read_register(EN_RXADDR,&nrf24l01p_en_rxaddr,sizeof(nrf24l01p_en_rxaddr));
	return nrf24l01p_en_rxaddr;
}
void nrf24l01p_enable_rx_pipe(uint8_t pipe){
	nrf24l01p_read_register(EN_RXADDR,&nrf24l01p_en_rxaddr,sizeof(nrf24l01p_en_rxaddr));
	set_bit(nrf24l01p_en_rxaddr,pipe);
	nrf24l01p_write_register(EN_RXADDR,&nrf24l01p_en_rxaddr,sizeof(nrf24l01p_en_rxaddr));
}
void nrf24l01p_disable_rx_pipe(uint8_t pipe){
	nrf24l01p_read_register(EN_RXADDR,&nrf24l01p_en_rxaddr,sizeof(nrf24l01p_en_rxaddr));
	clr_bit(nrf24l01p_en_rxaddr,pipe);
	nrf24l01p_write_register(EN_RXADDR,&nrf24l01p_en_rxaddr,sizeof(nrf24l01p_en_rxaddr));
}

//////////////////////////////////////////////////////////////////////////SETUP_AW
void nrf24l01p_write_setup_aw(uint8_t data){
	nrf24l01p_setup_aw = data;
	nrf24l01p_write_register(SETUP_RETR,&nrf24l01p_setup_aw,sizeof(nrf24l01p_setup_aw));
}
uint8_t nrf24l01p_read_setup_aw(){
	nrf24l01p_read_register(SETUP_RETR,&nrf24l01p_setup_aw,sizeof(nrf24l01p_setup_aw));
	return nrf24l01p_setup_aw;
}
//////////////////////////////////////////////////////////////////////////SETUP_RETR
void nrf24l01p_write_setup_retr(uint8_t data){
	nrf24l01p_setup_retr = data;
	nrf24l01p_write_register(SETUP_RETR,&nrf24l01p_setup_retr,sizeof(nrf24l01p_setup_retr));
}
uint8_t nrf24l01p_read_setup_retr(){
	nrf24l01p_read_register(SETUP_RETR,&nrf24l01p_setup_retr,sizeof(nrf24l01p_setup_retr));
	return nrf24l01p_setup_retr;
}
void nrf24l01p_set_auto_retransmit_delay(uint8_t delay_times_250us)
{
	nrf24l01p_read_register(SETUP_RETR,&nrf24l01p_setup_retr,sizeof(nrf24l01p_setup_retr));
	nrf24l01p_setup_retr = (nrf24l01p_setup_retr& 0x0F)| (delay_times_250us<<ARD);
	nrf24l01p_write_register(SETUP_RETR,&nrf24l01p_setup_retr,sizeof(nrf24l01p_setup_retr));
}
void nrf24l01p_set_auto_retransmit_count(uint8_t count)
{
	nrf24l01p_read_register(SETUP_RETR,&nrf24l01p_setup_retr,sizeof(nrf24l01p_setup_retr));
	nrf24l01p_setup_retr = (nrf24l01p_setup_aw&0xF0) | (count<<ARC);
	nrf24l01p_write_register(SETUP_RETR,&nrf24l01p_setup_retr,sizeof(nrf24l01p_setup_retr));
}

//////////////////////////////////////////////////////////////////////////RF_CH
void nrf24l01p_write_rf_ch(uint8_t data){
	nrf24l01p_rf_ch = data;
	nrf24l01p_write_register(RF_CH,&nrf24l01p_rf_ch,sizeof(nrf24l01p_rf_ch));
}
uint8_t nrf24l01p_read_rf_ch(){
	nrf24l01p_read_register(RF_CH,&nrf24l01p_rf_ch,sizeof(nrf24l01p_rf_ch));
	return nrf24l01p_rf_ch;
}
//////////////////////////////////////////////////////////////////////////RF_SETUP
void nrf24l01p_write_rf_setup(uint8_t data){
	nrf24l01p_rf_setup = data;
	nrf24l01p_write_register(RF_SETUP,&nrf24l01p_rf_setup,sizeof(nrf24l01p_rf_setup));
}
uint8_t nrf24l01p_read_rf_setup(){
	nrf24l01p_read_register(RF_SETUP,&nrf24l01p_rf_setup,sizeof(nrf24l01p_rf_setup));
	return nrf24l01p_rf_setup;
}
//////////////////////////////////////////////////////////////////////////STATUS
void nrf24l01p_write_status(uint8_t data){
	nrf24l01p_status = data;
	nrf24l01p_write_register(NRF_STATUS,&nrf24l01p_status,sizeof(nrf24l01p_status));
}
uint8_t nrf24l01p_read_status(){
	nrf24l01p_read_register(NRF_STATUS,&nrf24l01p_status,sizeof(nrf24l01p_status));
	return nrf24l01p_status;
}
uint8_t nrf24l01p_tx_full_flag(){
	nrf24l01p_read_register(NRF_STATUS,&nrf24l01p_status,sizeof(nrf24l01p_status));
	if(nrf24l01p_status&(1<<TX_FULL)) return 1;
	else return 0;
}
uint8_t nrf24l01p_rx_pipe_no(){
	nrf24l01p_read_register(NRF_STATUS,&nrf24l01p_status,sizeof(nrf24l01p_status));
	return ((nrf24l01p_status>>RX_P_NO)&0x07 );
}
uint8_t nrf24l01p_max_rt_flag(){
	nrf24l01p_read_register(NRF_STATUS,&nrf24l01p_status,sizeof(nrf24l01p_status));
	if(nrf24l01p_status&(1<<MAX_RT)) return 1;
	else return 0;
}
void nrf24l01p_clear_max_rt_flag(){
	nrf24l01p_read_register(NRF_STATUS,&nrf24l01p_status,sizeof(nrf24l01p_status));
	set_bit(nrf24l01p_status,MAX_RT);
	nrf24l01p_write_register(NRF_STATUS,&nrf24l01p_status,sizeof(nrf24l01p_status));
}
uint8_t nrf24l01p_data_sent_flag(){
	nrf24l01p_read_register(NRF_STATUS,&nrf24l01p_status,sizeof(nrf24l01p_status));
	if(nrf24l01p_status&(1<<TX_DS)) return 1;
	else return 0;
}
void nrf24l01p_clear_data_sent_flag(){
	nrf24l01p_read_register(NRF_STATUS,&nrf24l01p_status,sizeof(nrf24l01p_status));
	set_bit(nrf24l01p_status,TX_DS);
	nrf24l01p_write_register(NRF_STATUS,&nrf24l01p_status,sizeof(nrf24l01p_status));
}
uint8_t nrf24l01p_data_ready_flag(){
	nrf24l01p_read_register(NRF_STATUS,&nrf24l01p_status,sizeof(nrf24l01p_status));
	if(nrf24l01p_status&(1<<RX_DR)) return 1;
	else return 0;
}
void nrf24l01p_clear_data_ready_flag(){
	nrf24l01p_read_register(NRF_STATUS,&nrf24l01p_status,sizeof(nrf24l01p_status));
	set_bit(nrf24l01p_status,RX_DR);
	nrf24l01p_write_register(NRF_STATUS,&nrf24l01p_status,sizeof(nrf24l01p_status));
}
//////////////////////////////////////////////////////////////////////////OBSERVE_TX
uint8_t nrf24l01p_read_observe_tx(){
	nrf24l01p_read_register(OBSERVE_TX,&nrf24l01p_observe_tx,sizeof(nrf24l01p_observe_tx));
	return nrf24l01p_observe_tx;
}
uint8_t nrf24l01p_lost_packets_count(){
	nrf24l01p_read_register(OBSERVE_TX,&nrf24l01p_observe_tx,sizeof(nrf24l01p_observe_tx));
	return ((nrf24l01p_observe_tx& 0xF0)>>PLOS_CNT);
}
uint8_t nrf24l01p_retransmitted_packet_count(){
	nrf24l01p_read_register(OBSERVE_TX,&nrf24l01p_observe_tx,sizeof(nrf24l01p_observe_tx));
	return ((nrf24l01p_observe_tx& 0x0F)>>ARC_CNT);
}
//////////////////////////////////////////////////////////////////////////RPD
uint8_t nrf24l01p_read_rpd(){
	nrf24l01p_read_register(RPD,&nrf24l01p_rpd,sizeof(nrf24l01p_rpd));
	return nrf24l01p_rpd;
}
//////////////////////////////////////////////////////////////////////////RX_ADDR_P0
void nrf24l01p_write_rx_addr_p0(){
	nrf24l01p_write_register(RX_ADDR_P0,nrf24l01p_rx_addr_p0,sizeof(nrf24l01p_rx_addr_p0));
}
void nrf24l01p_read_rx_addr_p0(){
	nrf24l01p_read_register(RX_ADDR_P0,nrf24l01p_rx_addr_p0,sizeof(nrf24l01p_rx_addr_p0));
}
//////////////////////////////////////////////////////////////////////////RX_ADDR_P1
void nrf24l01p_write_rx_addr_p1(){
	nrf24l01p_write_register(RX_ADDR_P1,nrf24l01p_rx_addr_p1,sizeof(nrf24l01p_rx_addr_p1));
}
void nrf24l01p_read_rx_addr_p1(){
	nrf24l01p_read_register(RX_ADDR_P1,nrf24l01p_rx_addr_p1,sizeof(nrf24l01p_rx_addr_p1));
}
//////////////////////////////////////////////////////////////////////////RX_ADDR_P2
void nrf24l01p_write_rx_addr_p2(){
	nrf24l01p_write_register(RX_ADDR_P2,nrf24l01p_rx_addr_p2,sizeof(nrf24l01p_rx_addr_p2));
}
void nrf24l01p_read_rx_addr_p2(){
	nrf24l01p_read_register(RX_ADDR_P2,nrf24l01p_rx_addr_p2,sizeof(nrf24l01p_rx_addr_p2));
}
//////////////////////////////////////////////////////////////////////////RX_ADDR_P3
void nrf24l01p_write_rx_addr_p3(){
	nrf24l01p_write_register(RX_ADDR_P3,nrf24l01p_rx_addr_p3,sizeof(nrf24l01p_rx_addr_p3));
}
void nrf24l01p_read_rx_addr_p3(){
	nrf24l01p_read_register(RX_ADDR_P3,nrf24l01p_rx_addr_p3,sizeof(nrf24l01p_rx_addr_p3));
}
//////////////////////////////////////////////////////////////////////////RX_ADDR_P4
void nrf24l01p_write_rx_addr_p4(){
	nrf24l01p_write_register(RX_ADDR_P4,nrf24l01p_rx_addr_p4,sizeof(nrf24l01p_rx_addr_p4));
}
void nrf24l01p_read_rx_addr_p4(){
	nrf24l01p_read_register(RX_ADDR_P4,nrf24l01p_rx_addr_p4,sizeof(nrf24l01p_rx_addr_p4));
}
//////////////////////////////////////////////////////////////////////////RX_ADDR_P5
void nrf24l01p_write_rx_addr_p5(){
	nrf24l01p_write_register(RX_ADDR_P5,nrf24l01p_rx_addr_p5,sizeof(nrf24l01p_rx_addr_p5));
}
void nrf24l01p_read_rx_addr_p5(){
	nrf24l01p_read_register(RX_ADDR_P5,nrf24l01p_rx_addr_p5,sizeof(nrf24l01p_rx_addr_p5));
}
//////////////////////////////////////////////////////////////////////////TX_ADDR
void nrf24l01p_write_tx_addr(){
	nrf24l01p_write_register(TX_ADDR,nrf24l01p_tx_addr,sizeof(nrf24l01p_tx_addr));
}
void nrf24l01p_read_tx_addr(){
	nrf24l01p_read_register(TX_ADDR,nrf24l01p_tx_addr,sizeof(nrf24l01p_tx_addr));
}
//////////////////////////////////////////////////////////////////////////RX_PW[5:0]
void nrf24l01p_write_rx_pw(uint8_t pipe, uint8_t data){
	nrf24l01p_rx_pw[pipe] = data;
	nrf24l01p_write_register(RX_PW_P0+pipe,&nrf24l01p_rx_pw[pipe],sizeof(nrf24l01p_rx_pw[pipe]));
}
uint8_t nrf24l01p_read_rx_pw(uint8_t pipe){
	nrf24l01p_read_register(RX_PW_P0+pipe,&nrf24l01p_rx_pw[pipe],sizeof(nrf24l01p_rx_pw[pipe]));
	return nrf24l01p_rx_pw[pipe];
}
//////////////////////////////////////////////////////////////////////////FIFO_STATUS
uint8_t nrf24l01p_read_fifo_status(){
	nrf24l01p_read_register(FIFO_STATUS,&nrf24l01p_fifo_status,sizeof(nrf24l01p_fifo_status));
	return nrf24l01p_fifo_status;
}
uint8_t nrf24l01p_read_rx_fifo_empty_flag(){
	nrf24l01p_read_fifo_status();
	return nrf24l01p_fifo_status&(1<<RX_EMPTY);
}
//////////////////////////////////////////////////////////////////////////DYNPD
void nrf24l01p_write_dynpd(uint8_t data){
	nrf24l01p_dynpd = data;
	nrf24l01p_write_register(DYNPD,&nrf24l01p_dynpd,sizeof(nrf24l01p_dynpd));
}
uint8_t nrf24l01p_read_dynpd(){
	nrf24l01p_read_register(DYNPD,&nrf24l01p_dynpd,sizeof(nrf24l01p_dynpd));
	return nrf24l01p_dynpd;
}
void nrf24l01p_enable_dynamic_payload(uint8_t pipe)
{
	nrf24l01p_read_register(DYNPD,&nrf24l01p_dynpd,sizeof(nrf24l01p_dynpd));
	set_bit(nrf24l01p_dynpd,pipe);
	nrf24l01p_write_register(DYNPD,&nrf24l01p_dynpd,sizeof(nrf24l01p_dynpd));
}
void nrf24l01p_disable_dynamic_payload(uint8_t pipe)
{
	nrf24l01p_read_register(DYNPD,&nrf24l01p_dynpd,sizeof(nrf24l01p_dynpd));
	clr_bit(nrf24l01p_dynpd,pipe);
	nrf24l01p_write_register(DYNPD,&nrf24l01p_dynpd,sizeof(nrf24l01p_dynpd));
}
//////////////////////////////////////////////////////////////////////////FEATURE
void nrf24l01p_write_feature(uint8_t data){
	nrf24l01p_feature = data;
	nrf24l01p_write_register(FEATURE,&nrf24l01p_feature,sizeof(nrf24l01p_feature));
}
uint8_t nrf24l01p_read_feature(){
	nrf24l01p_read_register(FEATURE,&nrf24l01p_feature,sizeof(nrf24l01p_feature));
	return nrf24l01p_feature;
}
void nrf24l01p_enable_feature_dynamic_payload()
{
	nrf24l01p_read_register(FEATURE,&nrf24l01p_feature,sizeof(nrf24l01p_feature));
	set_bit(nrf24l01p_feature,EN_DPL);
	nrf24l01p_write_register(FEATURE,&nrf24l01p_feature,sizeof(nrf24l01p_feature));
}
void nrf24l01p_disable_feature_dynamic_payload()
{
	nrf24l01p_read_register(FEATURE,&nrf24l01p_feature,sizeof(nrf24l01p_feature));
	clr_bit(nrf24l01p_feature,EN_DPL);
	nrf24l01p_write_register(FEATURE,&nrf24l01p_feature,sizeof(nrf24l01p_feature));
}
void nrf24l01p_enable_feature_payload_with_ack()
{
	nrf24l01p_read_register(FEATURE,&nrf24l01p_feature,sizeof(nrf24l01p_feature));
	set_bit(nrf24l01p_feature,EN_ACK_PAY);
	nrf24l01p_write_register(FEATURE,&nrf24l01p_feature,sizeof(nrf24l01p_feature));
}
void nrf24l01p_disable_feature_payload_with_ack()
{
	nrf24l01p_read_register(FEATURE,&nrf24l01p_feature,sizeof(nrf24l01p_feature));
	clr_bit(nrf24l01p_feature,EN_ACK_PAY);
	nrf24l01p_write_register(FEATURE,&nrf24l01p_feature,sizeof(nrf24l01p_feature));
}
void nrf24l01p_enable_w_tx_payload_noack()
{
	nrf24l01p_read_register(FEATURE,&nrf24l01p_feature,sizeof(nrf24l01p_feature));
	set_bit(nrf24l01p_feature,EN_DYN_ACK);
	nrf24l01p_write_register(FEATURE,&nrf24l01p_feature,sizeof(nrf24l01p_feature));
}
void nrf24l01p_disable_w_tx_payload_noack()
{
	nrf24l01p_read_register(FEATURE,&nrf24l01p_feature,sizeof(nrf24l01p_feature));
	clr_bit(nrf24l01p_feature,EN_DYN_ACK);
	nrf24l01p_write_register(FEATURE,&nrf24l01p_feature,sizeof(nrf24l01p_feature));
}

//////////////////////////////////////////////////////////////////////////


uint8_t readable(int pipe) {

	if ( ( pipe < NRF24L01P_PIPE_P0 ) || ( pipe > NRF24L01P_PIPE_P5 ) ) {
		return 0;
	}

	nrf24l01p_read_status();

	return ( ( nrf24l01p_status & _NRF24L01P_STATUS_RX_DR ) && ( ( ( nrf24l01p_status & _NRF24L01P_STATUS_RX_P_NO ) >> 1 ) == ( pipe & 0x7 ) )    );

}

void power_up(){
	nrf24l01p_power_up();
	_delay_us(_NRF24L01P_TIMING_Tpd2stby_us);
	mode = _NRF24L01P_MODE_STANDBY;
}

void power_down(){
	nrf24l01p_power_down();
	mode = _NRF24L01P_MODE_POWER_DOWN;
}


void set_transmit_mode(){
	if ( _NRF24L01P_MODE_POWER_DOWN == mode ) power_up();
	nrf24l01p_tx_mode();
	mode = _NRF24L01P_MODE_TX;
}

void set_receive_mode(){
	if ( _NRF24L01P_MODE_POWER_DOWN == mode ) power_up();
	nrf24l01p_rx_mode();
	mode = _NRF24L01P_MODE_RX;
}


int write(int pipe, char *data, int count) {

	// Note: the pipe number is ignored in a Transmit / write

	//
	// Save the CE state
	//
	int originalCe = ce_value;
	ce_pin(0);//disable();

	if ( count <= 0 ) return 0;

	if ( count > _NRF24L01P_TX_FIFO_SIZE ) count = _NRF24L01P_TX_FIFO_SIZE;

	// Clear the Status bit
	nrf24l01p_clear_data_sent_flag();
	//setRegister(_NRF24L01P_REG_STATUS, _NRF24L01P_STATUS_TX_DS);
	
	csn_pin(0);//nCS_ = 0;

	spi_master_transmit_byte_val(W_TX_PAYLOAD);

	for ( int i = 0; i < count; i++ ) {
		spi_master_transmit_byte_val(*data++);
		//spi.write(*data++);
	}

	csn_pin(1);

	int originalMode = mode;
	set_transmit_mode();//setTransmitMode();

	ce_pin(1);//enable();
	_delay_us(_NRF24L01P_TIMING_Thce_us);
	ce_pin(0);

	while ( !( nrf24l01p_read_status() & _NRF24L01P_STATUS_TX_DS ) ) {//while ( !( getStatusRegister() & _NRF24L01P_STATUS_TX_DS ) ) {

		// Wait for the transfer to complete

	}

	// Clear the Status bit
	nrf24l01p_clear_data_sent_flag();
	//setRegister(_NRF24L01P_REG_STATUS, _NRF24L01P_STATUS_TX_DS);

	if ( originalMode == _NRF24L01P_MODE_RX ) {

	set_receive_mode();//setReceiveMode();

	}

	//ce_ = originalCe;
	ce_pin(originalCe);
	
	_delay_us( _NRF24L01P_TIMING_Tpece2csn_us );

	return count;

}


int read(int pipe, char *data, int count) {

	if ( ( pipe < NRF24L01P_PIPE_P0 ) || ( pipe > NRF24L01P_PIPE_P5 ) ) {

		//error( "nRF24L01P: Invalid read pipe number %d\r\n", pipe );
		return -1;

	}

	if ( count <= 0 ) return 0;

	if ( count > _NRF24L01P_RX_FIFO_SIZE ) count = _NRF24L01P_RX_FIFO_SIZE;

	if ( readable(pipe) ) {

		csn_pin(0);//nCS_ = 0;

		//int status = spi.write(_NRF24L01P_SPI_CMD_R_RX_PL_WID);
		spi_master_transmit_byte_val(_NRF24L01P_SPI_CMD_R_RX_PL_WID);
		
		//int rxPayloadWidth = spi.write(_NRF24L01P_SPI_CMD_NOP);
		int rxPayloadWidth = spi_master_transmit_byte_val(_NRF24L01P_SPI_CMD_NOP);
		
		csn_pin(1);//nCS_ = 1;

		if ( ( rxPayloadWidth < 0 ) || ( rxPayloadWidth > _NRF24L01P_RX_FIFO_SIZE ) ) {
			
			// Received payload error: need to flush the FIFO

			csn_pin(0);//nCS_ = 0;
			
			//int status = spi.write(_NRF24L01P_SPI_CMD_FLUSH_RX);
			spi_master_transmit_byte_val(_NRF24L01P_SPI_CMD_FLUSH_RX);
			
			//int rxPayloadWidth = spi.write(_NRF24L01P_SPI_CMD_NOP);
			//int rxPayloadWidth = spi_master_transmit_byte_val(_NRF24L01P_SPI_CMD_NOP);
			
			csn_pin(1);//nCS_ = 1;
			
		

			} else {

			if ( rxPayloadWidth < count ) count = rxPayloadWidth;

			csn_pin(0);//nCS_ = 0;
			
			//int status = spi.write(_NRF24L01P_SPI_CMD_RD_RX_PAYLOAD);
			spi_master_transmit_byte_val(_NRF24L01P_SPI_CMD_RD_RX_PAYLOAD);
			for ( int i = 0; i < count; i++ ) {
				
				//*data++ = spi.write(_NRF24L01P_SPI_CMD_NOP);
				*data++ = spi_master_transmit_byte_val(_NRF24L01P_SPI_CMD_NOP);
			}

			csn_pin(1);//nCS_ = 1;

			// Clear the Status bit
			if(nrf24l01p_read_rx_fifo_empty_flag())
			{
				nrf24l01p_clear_data_ready_flag();
				//setRegister(_NRF24L01P_REG_STATUS, _NRF24L01P_STATUS_RX_DR);
			}
			
			return count;
		}

		} else {

		return 0;

	}

	return -1;

}


void nrf_startup(){
		
	#define TRANSFER_SIZE 1
	nrf24l01p_write_rf_setup(0b00000111);
	nrf24l01p_write_en_aa(0);
	
	nrf24l01p_write_rx_pw(0, TRANSFER_SIZE);
	power_up();
	_delay_us(_NRF24L01P_TIMING_Tpd2stby_us);
	
	set_receive_mode();
	ce_pin(1);
}



//////////////////////////////////////////////////////////////////////////
//NRF FILE STREAM
// 
// 
// 
// int nrf_putchar(char ch, FILE *stream)
// {
// 	if(ch=='\n')
// 	nrf_putchar('\r',stream);
// 	write(0,&ch,sizeof( ch ));
// 	//nrf_send_function(&ch);
// 	//_delay_us(10);
// 	return 0;
// }
// 
// int nrf_getchar(FILE *stream)
// {
// 	char ch;
//  	read(0, &ch, sizeof( ch ));
// 	asm("nop");
// // 	char ch;
// // 	nrf_receive_function(&ch);
// // 	/* Echo the output back to the terminal */
// 
// 	if(ch != NULL) 
// 	{
// 		nrf_putchar(ch,stream);
// 	}
//  	
// 	return ch;
// }
// 
// FILE nrf_std = FDEV_SETUP_STREAM(nrf_putchar, nrf_getchar, _FDEV_SETUP_RW);


#endif /* NRF24L01P_H_ */

