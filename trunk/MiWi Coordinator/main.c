/********************************************************************
 * FileName:		main.c
 * Dependencies: none   
 * Processor:	PIC18, PIC24F, PIC24H, dsPIC30, dsPIC33
 *               tested with 18F4620, dsPIC33FJ256GP710	
 * Hardware:		PICDEM Z, Explorer 16
 * Complier:     Microchip C18 v3.04 or higher
 *				Microchip C30 v2.03 or higher	
 * Company:		Microchip Technology, Inc.
 *
 * Copyright and Disclaimer Notice for MiWi Software:
 *
 * Copyright © 2007-2008 Microchip Technology Inc.  All rights reserved.
 *
 * Microchip licenses to you the right to use, modify, copy and distribute 
 * Software only when embedded on a Microchip microcontroller or digital 
 * signal controller and used with a Microchip radio frequency transceiver, 
 * which are integrated into your product or third party product (pursuant 
 * to the terms in the accompanying license agreement). 
 *
 * You should refer to the license agreement accompanying this Software for 
 * additional information regarding your rights and obligations.
 *
 * SOFTWARE AND DOCUMENTATION ARE PROVIDED “AS IS” WITHOUT WARRANTY OF ANY 
 * KIND, EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT LIMITATION, ANY 
 * WARRANTY OF MERCHANTABILITY, TITLE, NON-INFRINGEMENT AND FITNESS FOR A 
 * PARTICULAR PURPOSE. IN NO EVENT SHALL MICROCHIP OR ITS LICENSORS BE 
 * LIABLE OR OBLIGATED UNDER CONTRACT, NEGLIGENCE, STRICT LIABILITY, 
 * CONTRIBUTION, BREACH OF WARRANTY, OR OTHER LEGAL EQUITABLE THEORY ANY 
 * DIRECT OR INDIRECT DAMAGES OR EXPENSES INCLUDING BUT NOT LIMITED TO 
 * ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE OR CONSEQUENTIAL DAMAGES, 
 * LOST PROFITS OR LOST DATA, COST OF PROCUREMENT OF SUBSTITUTE GOODS, 
 * TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES (INCLUDING BUT 
 * NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.
 *
 *********************************************************************
 * File Description:
 *
 *  This is the main example file.  Since there are only 2 buttons 
 *   available on the PICDEM Z, we have created several defintions
 *   in the definitions section of this file to select between
 *   several different demos showing off different ways to use features
 *   of the stack.  Please read each explination carefully.
 *
 * Change History:
 *  Rev   Date         Author	Description
 *  0.1   11/09/2006   df        Initial revision
 *  1.0   01/09/2007   yfy       Initial release
 ********************************************************************/

/************************ HEADERS **********************************/
#include ".\Common\Console.h"
#include ".\MiWi\MiWi.h"
#include ".\MiWi\MRF24J40.h"
#include ".\MiWi\SymbolTime.h"
#include <p18f4520.h>
#include <stdlib.h>

/************************ VARIABLES ********************************/
#pragma romdata longAddressLocation = 0x0E
ROM unsigned char myLongAddress[8] = {EUI_0,EUI_1,EUI_2,EUI_3,EUI_4,EUI_5,EUI_6,EUI_7};
#pragma romdata

#ifdef SUPPORT_SECURITY
#pragma romdata securityKey = 0x2A
ROM unsigned char mySecurityKey[16] = {SECURITY_KEY_00, SECURITY_KEY_01, SECURITY_KEY_02, SECURITY_KEY_03, SECURITY_KEY_04,
	SECURITY_KEY_05, SECURITY_KEY_06, SECURITY_KEY_07, SECURITY_KEY_08, SECURITY_KEY_09, SECURITY_KEY_10, SECURITY_KEY_11, 
	SECURITY_KEY_12, SECURITY_KEY_13, SECURITY_KEY_14, SECURITY_KEY_15};
#pragma romdata
ROM unsigned char mySecurityLevel = SECURITY_LEVEL;
ROM unsigned char myKeySequenceNumber = KEY_SEQUENCE_NUMBER;
#endif


ROM BYTE availableChannels[AVAILABLE_CHANNELS_SIZE]={ALLOWED_CHANNELS};
ROM unsigned char myManufacturerString[]="Company ABC";

/********************** FUNCTION PROTOTYPES *************************/
void BoardInit(void);

/********************** CONSTANTS/DEFINITIONS ***********************/

#define MAX_DISCOVERY_ATTEMPTS 3
#define DEBOUNCE_TIME 0x00008FFF
/*
#define PIC32MX_SPI1_SDO_SCK_MASK_VALUE      (0x00000140)

#define PIC32MX_SPI1_SDI_MASK_VALUE          (0x00000080)

#define PIC32MX_INT1_MASK_VALUE              (0x00000008)
*/
/* MAX SPI CLOCK FREQ SUPPORTED FOR MIWI TRANSCIEVER */
#define MAX_SPI_CLK_FREQ_FOR_MIWI            (20000000)

/* FUNCOES DE INTERFACE COM PC */

void send_nodes_list(void) 
{
	BYTE i=0;
	printf("<INI>\r\n");
	for(i=0;i<NETWORK_TABLE_SIZE;i++) {
		if((networkStatus[i].bits.isValid) && (networkTable[i].ShortAddress.Val != 0x0000)) {
			PrintChar(networkTable[i].ShortAddress.v[1]);
			PrintChar(networkTable[i].ShortAddress.v[0]);
		//	PrintChar(networkTable[i].info.LongAddress[7]);
		//	PrintChar(networkTable[i].info.LongAddress[6]);
		//	PrintChar(networkTable[i].info.LongAddress[5]);
		//	PrintChar(networkTable[i].info.LongAddress[4]);
		//	PrintChar(networkTable[i].info.LongAddress[3]);
		//	PrintChar(networkTable[i].info.LongAddress[2]);
		//	PrintChar(networkTable[i].info.LongAddress[1]);
		//	PrintChar(networkTable[i].info.LongAddress[0]);
			printf("\r\n");
			//PrintChar (myShortAddress.v[1]);
			//PrintChar (myShortAddress.v[0]);
		}	
	}	
	printf("<FIM>\r\n");
}



void turn_on(WORD node)
{
		WORD_VAL no, panid;
        TxPayLoad();
        
     	WriteData(USER_REPORT_TYPE);
        WriteData(0x87); //TODO: Definir comando
        //SendReportByLongAddress(node);
      //  SendReportByHandle(node, FALSE);
      	panid.Val = 0x2247;
      	no.Val = node+0x30;
     	SendReportByShortAddress(panid, no, FALSE);
}

void turn_off(WORD node)
{
		WORD_VAL no, panid;
        TxPayLoad();
        WriteData(USER_REPORT_TYPE);
        WriteData(0x86); //TODO: Definir comando
        //SendReportByLongAddress(node);
        //SendReportByHandle(node, FALSE);
        panid.Val = 0x2247;
      	no.Val = node+0x30;
     	SendReportByShortAddress(panid, no, FALSE);

        
}

void dimmer_node(WORD node, WORD_VAL value)
{
		WORD_VAL no, panid;
        TxPayLoad();
        WriteData(USER_REPORT_TYPE);
        WriteData(0x88); //TODO: Definir comando
        WriteData(value.v[0]);
        WriteData(value.v[1]);
       // SendReportByLongAddress(node);
       // SendReportByHandle(atoi(node), FALSE);
 	    panid.Val = 0x2247;
      	no.Val = node+0x30;
     	SendReportByShortAddress(panid, no, FALSE);
}

WORD get_node(char *ptr) 
{
	int i;
	char aux[4] = {0};
	WORD node;
	
	for(i = 0; i < 4; i++) 
		aux[i] = *ptr++;

	return atoi(aux);
}

void get_rs232_cmd() 
{
	char buff[16] = {0};
	char *p = NULL;
	
	p = buff;

	ConsoleGetString(&buff, 7);
	
	switch(*p++) 
	{
		case '0':
		{
			turn_off(get_node(p));
			break;
		}
		case '1':
		{
			turn_on(get_node(p));
			break;
		}
		case '2':
		{
			WORD node;
			WORD_VAL dimmer;
			dimmer.v[1] = *p++;
			dimmer.v[0] = *p++;
			node = get_node(p);
			
			dimmer_node(node, dimmer);
		
			break;
		}
		case '3': 
		{
			send_nodes_list();
			break;
		}
	}
	buff[0] = 0;
}

int pega_cmd() {

	BYTE *pRxData2;
	BYTE i;
//	ConsolePutROMString((ROM char*)"DEBUG> RxPacket TRUE ...\r\n");             
	//insert user code here for processing packets as they come in. 
	//This is a light example
	pRxData2 = pRxData;
	*pRxData++;     //take off the seq number
	
	//DumpNetworkTable();
	switch(*pRxData++)      //report type
	{
		case USER_REPORT_TYPE:

			switch(*pRxData++)      //report id
			{
				case 0x86: 
				//	ConsolePutROMString((ROM char*)"0x86\r\n");
					break;
				case 0x87: 
				//	ConsolePutROMString((ROM char*)"0x87\r\n");
					break;	
				case 0x88: {
					BYTE a, b;
					a = *pRxData++;
					b = *pRxData++;
				//	ConsolePutROMString((ROM char*)"0x88\r\n");
				//	PrintChar(a-0x30);
				//	PrintChar(b-0x30);
					//ConsolePutROMString((ROM char*)"\r\n<FIM>\r\n");
					break;	
				}	

				case SERIAL_TYPE:  	//report Type
					break;
			}
			break;
		case MIWI_STACK_REPORT_TYPE: // NÃO APagar do stack !!
			switch(*pRxData)
			{
				case ACK_REPORT_TYPE:
			//		ConsolePutROMString((ROM char*)"Got MiWi ACK for my packet\r\n");
					break;
			}
			break;

		default:
		//	ConsolePutROMString((ROM char*)"DEBUG> RxPacket Desconhecido TYPE... \r\n");	
			break;

	} // fecha switch...

	//need to discard this packet before we are able to receive any other packets
	DiscardPacket();
}

/*********************************************************************
 * Function:         void main(void)
 *
 * PreCondition:     none
 *
 * Input:		    none
 *
 * Output:		    none
 *
 * Side Effects:	    none
 *
 * Overview:		    This is the main function that runs the demo.  
 *                   The device will first search for an existing
 *                   network.  If a network exists and the parameters 
 *                   of the network are acceptable to the device (in 
 *                   this example simple RSSI minimum) then the device
 *                   will join the existing network.  If the device 
 *                   does not find an acceptable network and is a 
 *                   coordinator then the device will perform an 
 *                   energy scan on all of the channels available and 
 *                   determine which channel has the lowest noise.  
 *                   It will form a new network on this channel as 
 *                   the PAN coordinator.
 *
 * Note:			    This header only applies if P2P_ONLY is not
 *                   defined.  If it is then the appropriate header
 *                   is located below
 ********************************************************************/


void main(void)
{   
	BYTE i;
	BYTE numDiscoveryAttempts;
	BYTE myFriend = 0xFF;
	BYTE tmpRxSize = 0;

	numDiscoveryAttempts = 0;

	//initialize the system
	BoardInit();  

	ConsoleInit();  

	MiWiInit();

//	ConsolePutROMString((ROM char*)"Sistema Inicializado Ok...\r\n"); 

	INTCONbits.GIEH = 1;

	while(1)
	{
		LED_3 = 1;
		MiWiTasks();
		
		
		if(PIR1bits.RCIF) 			
			get_rs232_cmd();
		//if we are a memeber of a network
		
		if(MemberOfNetwork())
		{
			LED_3 = 1; //acende led sinalizando que está numa rede
			if(RxPacket())
			{
			//	ConsolePutROMString((ROM char*)"Chegou novo pacote, tratando...\r\n"); 
				pega_cmd();
			}
		}
		else
		{  
			LED_3 = 0; //apaga led sinalizando que Não está numa rede

			//If I don't have a network yet and I am not currently trying to join a network
			if((!SearchingForNetworks()) && (!AttemptingToJoinNetwork()) && (!AttemptingToRejoinNetwork()))
			{
		//		ConsolePutROMString((ROM char*)"Criando nova rede...\r\n"); 
				//form a network with the specified PANID
				FormNetwork(0x2247); // era 0xFFFF pra RANDOM
				//clear all of the network entries
				ClearNetworkTable(CLEAR_NETWORKS);
			}
		}
		LED_3 = 0;
	}
}



/*********************************************************************
 * Function:        void BoardInit( void )
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    Board is initialized for MiWi usage
 *
 * Overview:        This function configures the board for the PICDEM-z
 *                  MRF24J40 usage 
 *
 * Note:            This routine needs to be called before the function 
 *                  to initialize MiWi stack or any other function that
 *                  operates on the stack
 ********************************************************************/

void BoardInit(void)
{

	WDTCONbits.SWDTEN = 0; //disable WDT

	// Switches S2 and S3 are on RB5 and RB4 respectively. We want interrupt-on-change
	INTCON = 0x00;

	// There is no external pull-up resistors on S2 and S3. We will use internal pull-ups.
	// The MRF24J40 is using INT0 for interrupts
	// Enable PORTB internal pullups
	INTCON2 = 0x00;
	INTCON3 = 0x00;

	// Make PORTB as input - this is the RESET default
	TRISB = 0x01;
	LATB = 0x00;

	// Set PORTC control signal direction and initial states
	// disable chip select
	LATC = 0xfd;
	// Set the SPI module for use by Stack
	TRISC = 0xD0;

	LATD = 0x00;	// PORTD como saÌda e ...
	TRISD = 0x00;	// nÌvel 0 apaga displays

	// Set the SPI module
	SSPSTAT = 0xC0;
	SSPCON1 = 0x20;

	ADCON1 = 0x0D; // PA 0 & 1 como AN restante digital

	LATA = 0x00;
	TRISA = 0b11011111;  // bit 5 em zero liga display 2 do PIc CUBE Æ

	PHY_CS = 1;             //deselect the MRF24J40
	PHY_CS_TRIS = 0;        //make chip select an output   

	RFIF = 0;               //clear the interrupt flag

	RCONbits.IPEN = 1;

	INTCON2bits.INTEDG0 = 0;

}
