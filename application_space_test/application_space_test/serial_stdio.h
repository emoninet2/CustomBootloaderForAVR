/*
 * serial_stdio.h
 *
 * Created: 3/9/2015 4:35:08 AM
 *  Author: Emon
 */ 


#ifndef SERIAL_STDIO_H_
#define SERIAL_STDIO_H_

#include <stdio.h>
#include <stdlib.h>
#include "usart.h"


#define stdio_serial_send_function		USART0_transmit
#define stdio_serial_receive_function	USART0_receive

int uart_putchar(char ch, FILE *stream)
{
	if(ch=='\n')
	uart_putchar('\r',stream);
	stdio_serial_send_function(ch);
	return 0;
}

int uart_getchar(FILE *stream)
{
	char ch;
	stdio_serial_receive_function(&ch);
	
	/* Echo the output back to the terminal */
	uart_putchar(ch,stream);

	return ch;
}


FILE uart_str = FDEV_SETUP_STREAM(uart_putchar, uart_getchar, _FDEV_SETUP_RW);


void stdio_serial_initialize()
{
	stdout = stdin = &uart_str;
}


// void clear_screen()
// {
// 	printf("%c[2J",27);
// }
// 
// void cursor_home()
// {
// 	printf("%c[H",27);
// }

#endif /* SERIAL_STDIO_H_ */