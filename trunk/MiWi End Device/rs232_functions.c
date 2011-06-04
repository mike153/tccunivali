#include "prototypes.h"
#include <Console.h>
#include <stdio.h>

void get_rs232_cmd() 
{
	char buff[16];
	char *p = NULL;
	int i = 0;
	
	buff[0] = 0;
	
	while(buff[i] != 'f') 
	{
		print(ConsoleGet());
		printf("\r\n");
	//	while(PIR1bits.RCIF == 1);
	}
	
	if(strlen(buff) > 0)
		printf(buff);
	p = buff;
	
	switch(*p++) 
	{
		case 'a':
		{
			BYTE i = *p++;
			if(i == 'c')
				printf("OK\r\n");
			else
				printf("NOK\r\n");
			buff[0] = 0;
			break;
		}	
	}
}

char *get_valor(char *str)
{
	
}