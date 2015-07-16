/*
 * usart.h
 *
 * Created: 3/9/2015 2:23:43 AM
 *  Author: Emon
 */ 


#ifndef USART_H_
#define USART_H_



#define use_USART	1

#define usart_parity_mode_none		0
#define usart_parity_mode_even		2
#define usart_parity_mode_odd		3

#define usart_stop_bit_1bit			0
#define usart_stop_bit_2bit			1

#define usart_character_size_5bit	0
#define usart_character_size_6bit	1
#define usart_character_size_7bit	2
#define usart_character_size_8bit	3
#define usart_character_size_9bit	3


#define usart_clk_pol_rise_tx_fall_rx 0
#define usart_clk_pol_rise_rx_fall_tx 1
#define default_usart_clk_pol 0



void USART0_Init( unsigned long baud)
{
	unsigned int ubrr = (F_CPU/16/baud)-1;
	UBRR0H = (unsigned char)(ubrr>>8);
	UBRR0L = (unsigned char)ubrr;
	UCSR0B = (1<<RXEN0)|(1<<TXEN0);
	UCSR0C |= (1 << UCSZ01) | (1 << UCSZ00);

}

#if (use_USART==1)

#ifndef USART_baud_rate
#warning "USART baud rate not defined. Default baud rate 9600 selected"
#define USART_baud_rate 9600
#endif

#define USART_use_rxc_interrupt 0
#define USART_use_txc_interrupt 0
#define USART_enable_rx 1
#define USART_enable_tx 1
#define USART_use_9bits				0
#define USART_mode					0 //asynchronous
#define USART_parity_mode			usart_parity_mode_none
#define USART_stop_bits			usart_stop_bit_1bit
#define USART_character_size		usart_character_size_8bit
#define USART_clock_polarity		default_usart_clk_pol

#define USART_initialize(BAUD,MODE,PARITY,STOP_BITS,SIZE,POL){\
	UBRR0H = ((F_CPU/BAUD/16)-1)>>8;\
	UBRR0L = ((F_CPU/BAUD/16)-1);\
	UCSR0B = (USART_use_rxc_interrupt<<RXCIE0) | (USART_use_txc_interrupt<<TXCIE0)|\
	(USART_enable_rx<<RXEN0)| (USART_enable_tx<<TXEN0) | (USART_use_9bits<<UCSZ02);\
	UCSR0C = (MODE<<UMSEL) | (PARITY<<UPM00) | (STOP_BITS<<USBS0) |(SIZE<<UCSZ00) | (POL<<UCPOL0);\
}

#define USART0_transmit(data) {while (!(UCSR0A & (1<<UDRE0)));UDR0 = data;}
#define USART0_receive(data) {while (!(UCSR0A & (1<<RXC0)));*data = UDR0;}
#define USART0_transmit_string(data,datasize) {for(int i=0;i<datasize;i++){USART0_transmit(*(data + i));} }
#endif



#endif /* USART_H_ */