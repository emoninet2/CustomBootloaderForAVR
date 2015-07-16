/*
 * application_space_test.c
 *
 * Created: 7/16/2015 3:20:35 AM
 *  Author: Emon
 */ 

#define F_CPU 8000000UL
#include <avr/io.h>
#include <util/delay.h>
#include <avr/wdt.h>

#include "usart.h"
#include "serial_stdio.h"

uint8_t execute_command(char *cmd,char **args){
	if(!strcmp(cmd, "test") ) {
		PORTB^=(1<<PORTB1);
	}
	else if(!strcmp(cmd, "bootload") )  {	
		//3800
		//392f
		
		asm("ldi r30, 0x00");
		asm("ldi r31, 0x38");
		asm("IJMP");
		
	}
	else printf("\nAPP: unknown command\n\r");
	//else USART0_transmit_string("\runknown\n\r",sizeof("\nunknown\n\r"));
	
	return 0;
}







int main(void)
{

	
	_delay_ms(500);
	
	USART0_Init(9600);
	stdio_serial_initialize();
	
	
	
	
	
	DDRB|= (1<<DDB1);
	
	printf("entered application space\n");
	
	
	
	
	
    while(1)
    {
        char cmdstr[25];
		int arg_index = 0;
		char *cmd;
		char *pch;
		char *args[ 10];
        cmd = cmdstr;
        int count = 0;
		
		
        scanf("%s",&cmdstr[0]);

		pch = strtok(cmdstr, " ,;");
		while(pch != NULL) {
			args[arg_index] = pch;
			arg_index++;
			if(arg_index >=10) break;
			pch = strtok (NULL, " ,;");
		}
		
		execute_command(cmd,args);
		
		
		
    }
}