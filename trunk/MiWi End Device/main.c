/*****************************************************************************
 *
 *              Main.c -- user main program
 *
 *****************************************************************************
 * FileName:        main.c
 * Dependencies:
 * Processor:       PIC18F
 * Compiler:        C18 02.20.00 or higher
 * Linker:          MPLINK 03.40.00 or higher
 * Company:         Microchip Technology Incorporated
 *
 * Software License Agreement
 *
 * Copyright � 2007-2008 Microchip Technology Inc.  All rights reserved.
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
 * SOFTWARE AND DOCUMENTATION ARE PROVIDED �AS IS� WITHOUT WARRANTY OF ANY 
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
 *****************************************************************************
 *
 * Change History:
 *  Rev   Date         Author	Description
 *  0.1   11/09/2006   df       Initial revision
 *  1.0   01/09/2007   yfy      Initial release
 *****************************************************************************/

#include "../Microchip/Include/Common/Console.h"
#include "../Microchip/Include/MiWi/MiWi.h"
#include "../Microchip/Include/MiWi/MRF24J40.h"
#include "../Microchip/Include/MiWi/SymbolTime.h"
#include <delays.h>
#include <p18f4520.h>
#include <math.h>
#include <timers.h>

// long address do m�dulo para qual  vai transmitir quando usar este tipo de endere�amento
#pragma romdata eedata2_scn=0xf00000
rom unsigned char eedata2_values[8] = {0x93, 0x78, 0x56, 0x34, 0x12, 0xA3, 0x04, 0x00};
#pragma romdata

// short address do m�dulo para o qual ir� transmitir quando usar este tipo de endere�amento
#pragma romdata eedata3_scn = 0xf00010
rom unsigned char eedata3_values [2] = {0x00, 0x00};
#pragma romdata

// EUI / long address deste M�dulo localizado na mem�ria flash rom
// a definini��o do EUI / long address deste m�dulo fica no arquivo MiWiDefs.h
#pragma romdata longAddressLocation = 0x0E
ROM unsigned char myLongAddress[8] = {EUI_0,EUI_1,EUI_2,EUI_3,EUI_4,EUI_5,EUI_6,EUI_7};
#pragma romdata


#ifdef SUPPORT_SECURITY
#pragma romdata securityKey = 0x2A
ROM unsigned char mySecurityKey[16] = {SECURITY_KEY_00, SECURITY_KEY_01, SECURITY_KEY_02, SECURITY_KEY_03, SECURITY_KEY_04,
	SECURITY_KEY_05, SECURITY_KEY_06, SECURITY_KEY_07, SECURITY_KEY_08, SECURITY_KEY_09, SECURITY_KEY_10, SECURITY_KEY_11, 
	SECURITY_KEY_12, SECURITY_KEY_13, SECURITY_KEY_14, SECURITY_KEY_15};
#pragma romdata
#endif

#if defined(SUPPORT_SECURITY)
ROM unsigned char mySecurityLevel = SECURITY_LEVEL;
ROM unsigned char myKeySequenceNumber = KEY_SEQUENCE_NUMBER;
#endif

ROM BYTE availableChannels[AVAILABLE_CHANNELS_SIZE]={ALLOWED_CHANNELS};
ROM unsigned char myManufacturerString[]="Company ABC";


#define MAX_PA_OUTPUT 0x00	// 0dB output
//all values between 0x00 and 0x1F are valid and results in a -1.25dB output per unit above 0
#define MIN_PA_OUTPUT 0x1F	// -38.75dB output
#define MRF24J40PAOutputAdjust(a) {PHYSetLongRAMAddr(RFCTRL3,(a<<3));}

unsigned short carga_dispositivo = 0;

/********************** GLOBALS ****************************************/
unsigned short carga_disp(int valor)
{
	if (valor > 63)
		valor = 63;
	else if (valor < 33)
		valor = 33;
		
	return (unsigned short)valor*1000;
}

void liga()
{
	INTCON3bits.INT1IE = 0;
	CloseTimer1();
	carga_dispositivo = 0;
	PORTBbits.RB3 = 1;
}

void desliga()
{
	INTCON3bits.INT1IE = 0;
	CloseTimer1();
	carga_dispositivo = 0;
	PORTBbits.RB3 = 0;
}

/********************** FUNCTION (&)PROTOTYPES *************************/


void BoardInit(void);
int pega_cmd();
void send2coord();


unsigned char Le_E2Prom (unsigned char Endereco) 
{
	EECON1bits.EEPGD = 0;  /* READ step #1 */
	EEADR = Endereco;      /* READ step #2 */
	EECON1bits.RD = 1;     /* READ step #3 */
	return EEDATA;    	 /* READ step #4 */
}

unsigned char Flag_Envio_ID = 0;

void Envio_ID (void)
{
	if (Flag_Envio_ID == 0) //se n�o enviou ainda envia por pan coord o id de volta em modo serial
	{	
		signed char ii;
		TxPayLoad();//tem definido:#define TxPayLoad() TxData = 11   
		WriteData(USER_REPORT_TYPE);
		WriteData(SERIAL_TYPE);
		WriteData ('<');
		WriteData (myShortAddress.v [1]);
		WriteData (myShortAddress.v [0]);
		for (ii = 0 ; ii < 8; ii++)
		{
			WriteData (myLongAddress[ii]);	
		}
		WriteData ('>');
		tempShortAddress.Val = 0x0000; // shjort address do pan coord.
		SendReportByShortAddress(myPANID, tempShortAddress, FALSE);
		ConsolePutROMString((ROM char*)"envio short addr pro pan coord...\r\n");
		Flag_Envio_ID =  1;

	}
}

void send2coord() 
{
	tempShortAddress.Val = 0x0000; // shjort address do pan coord.
	SendReportByShortAddress(myPANID, tempShortAddress, FALSE);
	//ConsolePutROMString((ROM char*)"envio short addr pro pan coord...\r\n");
}
void trata_int_e_timer() 
{
	if(INTCON3bits.INT1IF) //Verifica se a INT1 aconteceu
    {
	    INTCON3bits.INT1IF = 0;
	    
	  // carga_dispositivo = 37000;
	    if (carga_dispositivo > 0) 
	    {
		    if (T1CONbits.TMR1ON == 0) {
		    	T1CONbits.TMR1ON = 1;
		    	PIE1bits.TMR1IE = 1;
		    }	
        	WriteTimer1(carga_dispositivo);
     	}   
    }
        
    if (PIR1bits.TMR1IF && T1CONbits.TMR1ON && PIE1bits.TMR1IE) 
    {
	    PIR1bits.TMR1IF = 0;
		//CloseTimer1();
        PORTBbits.RB3 = 1;
        Delay10TCYx(30);
        PORTBbits.RB3 = 0;        
    }
}
/********************************************************************/
/********************** CONSTANTS ***********************************/
/**********************  M A I N ************************************/
/********************************************************************/
/********************************************************************/
/********************************************************************/

#define MAX_DISCOVERY_ATTEMPTS 3
#define DEBOUNCE_TIME 0x00000C35


int pega_cmd() {

	BYTE *pRxData2;
	BYTE i;

	pRxData2 = pRxData;
	*pRxData++;     //take off the seq number

	switch(*pRxData++)      //report type
	{
		case USER_REPORT_TYPE:
			switch(*pRxData++)      //report id
			{
				case 0x86:
					desliga();
					TxPayLoad();
					WriteData(USER_REPORT_TYPE);
					WriteData(0x86); //TODO: Definir comando
					send2coord();
				break;
				case 0x87:
					liga();
					TxPayLoad();
					WriteData(USER_REPORT_TYPE);
					WriteData(0x87); //TODO: Definir comando
					send2coord();
					
				break;
				case 0x88: 
				{
					char tmp[3];
				
					int carga = 0;
					
					tmp[1] = *pRxData++;
					tmp[0] = *pRxData++;
					tmp[2] = 0;
					
					carga = atoi(tmp);

					
					if (carga) {
						carga_dispositivo = carga_disp(carga);
						INTCON3bits.INT1IE = 1;
						T1CONbits.TMR1ON = 1;
						PIE1bits.TMR1IE = 1;
					
					}
					ConsolePutROMString((ROM char*)"Ajustou\r\n");
					TxPayLoad();
					WriteData(USER_REPORT_TYPE);
					WriteData(0x88); //TODO: Definir comando
					WriteData(tmp[1]);
					WriteData(tmp[0]);
					
					send2coord();
				}
				break;
			}
			break;
		case MIWI_STACK_REPORT_TYPE:
			switch(*pRxData)
			{
				case ACK_REPORT_TYPE:
					//ConsolePutROMString((ROM char*)"Got MiWi ACK for my packet\r\n");
					break;
			}
			break;
	}

	//need to discard this packet before we are able to receive any other packets
	DiscardPacket();
}

void main(void)
{   
	BYTE i;
	BYTE numDiscoveryAttempts;
	BOOL requestedSocket;
	BOOL PUSH_BUTTON_1_pressed;
	TICK PUSH_BUTTON_1_press_time;
	TICK tickDifference;
	BYTE myFriend = 0xFF;

	requestedSocket = FALSE;

	numDiscoveryAttempts = 0;

	ConsoleInit();  
	BoardInit(); 
	ConsolePutROMString((ROM char*)"DEBUG> [sub Main] BoardInit OK ...\n\r ");
	MiWiInit();
  
	while(1)
	{
		LED_3 = 1;
		MiWiTasks();
		LED_3 = 0;

		
		if(MemberOfNetwork())
		{

			Envio_ID();		//v� se j� enviou back ID pro PAN COORd

			//If I am already part of the network
			if(RxPacket())
			{
				pega_cmd();
			}

				for (i = 0; i < 8; i++) {
					tempLongAddress[i] = Le_E2Prom (i);
					PrintChar ( tempLongAddress[i] );
				}

#if defined(I_AM_RFD)


			if(CheckForDataComplete()) {
				MRF24J40Sleep();
				while( !ConsoleIsPutReady() );

				WDTCONbits.SWDTEN = 1;      //enable the WDT to wake me up
				INTCONbits.RBIF = 0;
				INTCONbits.RBIE = 1;

		//		Sleep();            //goto sleep

				INTCONbits.RBIE = 0;
				WDTCONbits.SWDTEN = 0;      //disable WDT

				MRF24J40Wake();

				CheckForData();     //when I wake up do a data request
			}
#endif

			//END OF THE CODE AFTER YOU BECAME A MEMBER
		}
		else
		{  
			//If I don't have a network yet and I am not currently trying to join a network
			if((!SearchingForNetworks()) && (!AttemptingToJoinNetwork()) && (!AttemptingToRejoinNetwork()))
			{
				//I am not actively searching for a network so lets choise one from the list
				BYTE handleOfBestNetwork;
				BYTE i;
				//I will show the example of picking a network based on RSSI
				BYTE maxRSSI;

				//initialize the handle to none(0xFF)
				handleOfBestNetwork = 0xFF;
				//RSSI example
				maxRSSI = 0x00;

				for(i=0;i<NETWORK_TABLE_SIZE;i++)
				{
					if(networkStatus[i].bits.isValid)
					{
						if(networkStatus[i].bits.NeighborOrNetwork == NETWORK)
						{
							//make your PAN choice here
							if(networkTable[i].info.networkInfo.Protocol == MIWI_PROTOCOL_ID)
							{
								//make sure they are allowing joining
								if(networkTable[i].info.networkInfo.flags.bits.associationPermit == 1)
								{
									//first I want to make sure it is a MiWi network
									if(networkTable[i].info.networkInfo.sampleRSSI >= maxRSSI)
									{
										handleOfBestNetwork = i;
										maxRSSI = networkTable[i].info.networkInfo.sampleRSSI;
									}
								}
							}
						}
					}
				}

				//now that I picked a network let me join to it
				if(handleOfBestNetwork == 0xFF)
				{
					//I couldn't find a suitable network
	//				ConsolePutROMString((ROM char*)"Trying to find a suitable network\r\n");

					if(numDiscoveryAttempts++ > MAX_DISCOVERY_ATTEMPTS)
					{
						//clear all of the network entries
						ClearNetworkTable(CLEAR_NETWORKS);
					}
					else
					{
						//clear all of the network entries
						ClearNetworkTable(CLEAR_NETWORKS);
						//and start a new search
						DiscoverNetworks();
					}
				}
				else
				{
					//I found a network I would like to join                   
					//Join the network that you found to be the best
	//				ConsolePutROMString((ROM char*)"Trying to join network: ");
	//				PrintChar(handleOfBestNetwork);
	//				ConsolePutROMString((ROM char*)"\r\n");
					JoinNetwork(handleOfBestNetwork);

					{
						WORD i;

						LED_2 = 1;

						for(i=0;i<20000;i++)
						{
						}

						LED_2 = 0;

						for(i=0;i<20000;i++)
						{
						}

						LED_2 = 1;

						for(i=0;i<20000;i++)
						{
						}

						LED_2 = 0;                            
					} 
				}
			}
			//if I am searching for a network then I will leave it alone so I can continue searching
			//or if I am trying to join a network then I will let that join proccess finish
		}
		
	Delay10KTCYx(100);
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
	//PORTAS
		
	// D1 and D2 are on RA0 and RA1 respectively, and CS of TC77 is on RA2.
	// Make PORTA as digital I/O.
	//na~o The TC77 temp sensor CS is on RA2.
	//n�o Deselect TC77 (RA2)
	LATA = 0b11011111; // liga DSP2 PIC CUBE

	// n�o Make RA0, RA1, RA2 and RA4 as outputs.
	TRISA = 0b11011111;
	// Make PORTB as input - this is the RESET default
	TRISB = 0x03; //RB1 como input - INT1
	LATB = 0x00;
	PORTB = 0x00;

	TRISD = 0x00;
	LATD = 0x00;
	PORTD = 0x00;
	
	// Set PORTC control signal direction and initial states
	// disable chip select
	LATC = 0xfd;
	// Set the SPI module for use by Stack
	TRISC = 0xD0;

	//SERIAL
	// Set the SPI module
	SSPSTAT = 0xC0;
	SSPCON1 = 0x20;

	//MiWi
	PHY_CS = 1;             //deselect the MRF24J40
	PHY_CS_TRIS = 0;        //make chip select an output   

	//INTERRUPCOES
	RFIF = 0;               //clear the interrupt flag - Interrupt flag do MiWi
	WDTCONbits.SWDTEN = 0; //disable WDT
	// Switches S2 and S3 are on RB5 and RB4 respectively. We want interrupt-on-change
	INTCON = 0x00;
	// There is no external pull-up resistors on S2 and S3. We will use internal pull-ups.
	// The MRF24J40 is using INT0 for interrupts
	// Enable PORTB internal pullups
	INTCON2 = 0x00;
	INTCON3 = 0x00;
	
	RCONbits.IPEN = 1;
	INTCONbits.PEIE = 1;
	
	INTCONbits.GIEH = 1;
	INTCON2bits.INTEDG0 = 0; //INT0 ativa na borda de descida
	INTCON2bits.RBPU = 1; //Desativa todos os pull-ups, a int1 sera acionada por um pulso, logo, precisa estar em 0.
	
	
	/* int 1 */	
	INTCON3bits.INT1IE = 1; //Enable INT1
	INTCON3bits.INT1IP = 1; //Alta prioridade
	INTCON2bits.INTEDG1 = 0; //Boarda de subida
	INTCON3bits.INT1IF = 0;	// Int1 flag bit
	/* timer 1 */
	
	PORTBbits.RB3 = 0;
	
	OpenTimer1(TIMER_INT_OFF
			   &T1_16BIT_RW
			   &T1_SOURCE_INT
			   &T1_OSC1EN_OFF
			   &T1_SYNC_EXT_OFF
			   &T1_PS_1_1);
			  
//	T1CONbits.TMR1ON = 1;
//	PIE1bits.TMR1IE = 1;
	IPR1bits.TMR1IP = 1;
	PIR1bits.TMR1IF = 0; //Flag bit
	
	
/*
	//ADC
	ADCON1 = 0x0D; // PIC CUBE A0 e A1 como Anal. restante digital
	OpenADC(ADC_FOSC_16
			&ADC_RIGHT_JUST
			&ADC_4_TAD,
			ADC_CH0
			&ADC_INT_OFF
			&ADC_VREFPLUS_EXT
			&ADC_VREFMINUS_VSS,
			ADC_2ANA);
		*/
}
