/*
 * wireless_bootloader.c
 *
 * Created: 7/15/2015 1:11:03 PM
 *  Author: Emon
 */ 
#define F_CPU 8000000UL


#include <avr/io.h>
#include <util/delay.h>
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <avr/interrupt.h>
#include <avr/boot.h>
#include <avr/wdt.h>

#include "nrf24l01p.h"
#include "usart.h"
#include "serial_stdio.h"


#define soft_reset()        \
do                          \
{                           \
	wdt_enable(WDTO_15MS);  \
	for(;;)                 \
	{                       \
	}                       \
} while(0)


void execute_command(char *cmd,char **args){
	if(!strcmp(cmd, "test") ) {
		PORTB^=(1<<PORTB0);
	}
	else if(!strcmp(cmd, "bfp") ) {
		boot_page_fill_safe(args[1], args[2]);
	}
	else if(!strcmp(cmd, "bpw") ) {
		boot_page_write_safe(args[1]);
	}
	else if(!strcmp(cmd, "bre") ) {
		boot_rww_enable_safe();
	}
// 	else if(!strcmp(cmd, "blb") ) {
// 		boot_lock_bits_set_safe(args[1]);
// 	}
	else if(!strcmp(cmd, "bpe") ) {
		boot_page_erase_safe(args[1]);
	}
	else if(!strcmp(cmd, "rst") ) {//boot reset to application //0x7a
		//GICR = (1<<IVCE);
		//GICR = 0;
		
		asm("ldi r30, 0");
		asm("ldi r31, 0");
		asm("IJMP");
	}
	
	else if(!strcmp(cmd, "reset") )  {
		soft_reset();
		
	}
	else if(!strcmp(cmd, "write_reg") )  {
		_SFR_IO16(atoi(args[1]))= atoi(args[2]);

	}
	
	

	//else printf("unknown command\n");
	#define UNKNOWN_STRING "\rBL:unknown\n\r"
	else USART0_transmit_string(UNKNOWN_STRING,sizeof(UNKNOWN_STRING));
	
}


int uart_putc(char ch)
{
	if(ch=='\n')
	uart_putc('\r');
	stdio_serial_send_function(ch);
	return 0;
}

int uart_getc()
{
	char ch;
	stdio_serial_receive_function(&ch);
	
	/* Echo the output back to the terminal */
	uart_putc(ch);

	return ch;
}




int main(void)
{
	DDRB |= (1<<DDB0);
	_delay_ms(500);

	USART0_Init(9600);
	//stdio_serial_initialize();

   
   char startup_string[] = "\nentered bootloader\n\r";
	char cmdstr[25];
	
	
  
   
   
	int count = 0;
	_delay_ms(500);
	USART0_transmit_string(startup_string,sizeof(startup_string));
	
   while(1){
	    int arg_index = 0;
	    char *cmd;
	    char *pch;
	    char *args[ 11];
	   cmd = cmdstr;
	   for(int i =0;i<25;i++){cmdstr[i] = 0;}

		count = 0;
		do{
			char charin;
			charin = uart_getc();
			if(charin == '\r') break;
			else{
				cmdstr[count] = charin;
				count++;
			}

		}while(1);
	

	   pch = strtok(cmdstr, " ,;\r\n");
	   while(pch != NULL) {
	   		   args[arg_index] = pch;
	   		   arg_index++;
	   		   if(arg_index >=11) break;
	   		   pch = strtok (NULL, " ,;\r\n");
	   	   }

	   execute_command(cmd,args);
	  


   }
}

/*
int main(void)
{
	DDRB|= (1<<DDB1);
	
	while(1)
	{
		//TODO:: Please write your application code
		PORTB^= (1<<PORTB1);
		_delay_ms(100);
		
	}
}*/