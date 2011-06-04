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
#include "Common\Console.h"
#include "Microchip\MiWi\MiWi.h"
#include "MiWi\MRF24J40.h"
#include "MiWi\SymbolTime.h"
#include <usart.h>
#include <adc.h>
#include <delays.h>
#include <p18f4520.h>

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


#define Dig_0 0b11101110;
#define Dig_1 0b00101000;
#define Dig_2 0b11001101;
#define Dig_3 0b01101101;
#define Dig_4 0b00101011;
#define Dig_5 0b01100111;
#define Dig_6 0b11100111;
#define Dig_7 0b00101100;
#define Dig_8 0b11101111;
#define Dig_9 0b01101111;
#define Dig_C 0b11000110;
#define Dig_A 0b10101111;


/********************** FUNCTION (&)PROTOTYPES *************************/


void BoardInit(void);
unsigned int media_amostra();
int Converte_BCD2_7Seg (unsigned char bcd_value);
float le_temperatura();
int pega_cmd();


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

	//insert user code here for processing packets as they come in.

	pRxData2 = pRxData;
	*pRxData++;     //take off the seq number

	switch(*pRxData++)      //report type
	{
		case USER_REPORT_TYPE:
			switch(*pRxData++)      //report id
			{
				case LIGHT_REPORT:
					switch(*pRxData++)      //first byte of payload
					{
						case LIGHT_ON:
							LED_2 = 1;
							break;
						case LIGHT_OFF:
							LED_2 = 0;
							break;
						case LIGHT_TOGGLE:
							LED_2 ^= 1;
							ConsolePutROMString((ROM char*)"Received Report to Toggle Light\r\n");
							break;
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

unsigned int media_amostra()
{
	unsigned int i;
	unsigned long valor_canal = 0;

	for(i = 0; i < 2; i++) {
		ConvertADC();
		while(BusyADC()); //Espera a leitura
		valor_canal += ReadADC();
	}
	return valor_canal >> 1; //media de 256
}

float le_temperatura()
{
	//unsigned char ch;
	//long tlong;
	//char temperatura[3];
	float tensao, temp;
	unsigned int lido = media_amostra();
/*
	tlong = (long)lido * 5000;           // covert adc reading to milivolts
    tlong = tlong / 1023;
	ch     = tlong / 1000; 
	ch    = (tlong / 10) % 10; // Unidade       
	temperatura[0]  = Converte_BCD2_7Seg(ch);  
	ch    = (tlong / 100) % 10; //Dezena
	temperatura[1]  = Converte_BCD2_7Seg(ch);
*/
	tensao = (lido*5000)/1023;
	temp = tensao/10;
	return temp;
}

int Converte_BCD2_7Seg (unsigned char bcd_value) {

     switch (bcd_value){
         case 1:
         	return Dig_1;
         case 2:
	         return Dig_2;
         case 3:
         	return Dig_3;
         case 4:
         	return Dig_4;
         case 5:
         	return Dig_5;
         case 6:
         	return Dig_6;
         case 7:
         	return Dig_7;
         case 8:
         	return Dig_8;
         case 9:
         	return Dig_9;
         case 0:
	         return Dig_0;
         default:
         	return 0x00;
     }
}

void main(void)
{   
	BYTE i;
	BYTE numDiscoveryAttempts;
	BOOL requestedSocket;
	BOOL PUSH_BUTTON_1_pressed;
	BOOL PUSH_BUTTON_2_pressed;
	TICK PUSH_BUTTON_1_press_time;
	TICK PUSH_BUTTON_2_press_time;
	TICK tickDifference;
	BYTE myFriend = 0xFF;
//	char *temperatura = NULL;
	float temp = 0;

	requestedSocket = FALSE;

	numDiscoveryAttempts = 0;

	ConsoleInit();  

	BoardInit(); 
	ConsolePutROMString((ROM char*)"DEBUG> [sub Main] BoardInit OK ...\n\r ");

	MiWiInit();
	ConsolePutROMString((ROM char*)"DEBUG> [sub Main] MiWiInit OK ...\n\r ");

	INTCONbits.GIEH = 1;

	/*    if(PUSH_BUTTON_1 == 0)
	      {
	      ConsolePutROMString((ROM char*)"DEBUG> [sub Main] PUSH_BUTTON_1 pressionado ...\n\r ");
	      DumpNetworkTable();
	      while(PUSH_BUTTON_1==0){}
	      RejoinNetwork();
	      }
	      */
	while(1)
	{
		//temperatura = NULL;
		LED_3 = 1;
		MiWiTasks();
		LED_3 = 0;
		SetChanADC(ADC_CH0);
		//temperatura = le_temperatura();
		temp = le_temperatura();
		ConsolePutROMString((ROM char*) temp);
		if(MemberOfNetwork())
		{

			Envio_ID();		//v� se j� enviou back ID pro PAN COORd

			//If I am already part of the network
			if(RxPacket())
			{
				pega_cmd();
			}

#if defined(SUPPORT_CLUSTER_SOCKETS) || defined(SUPPORT_P2P_SOCKETS)
			if(requestedSocket == TRUE)
			{
				if(OpenSocketComplete())
				{
					requestedSocket = FALSE;

					if(OpenSocketSuccessful())
					{
						LED_1 = 1;
						ConsolePutROMString((ROM char*)"Found a socket: ");
						myFriend = OpenSocketHandle();
						PrintChar(myFriend);
						ConsolePutROMString((ROM char*)"\r\n");
					}
					else
					{
						LED_1 = 0;
						myFriend = 0xFF;
						ConsolePutROMString((ROM char*)"socket request failed\r\n");
					}
				}
			}
#endif


			//************ pressionou bot�o 2 ********************* 
			if(PUSH_BUTTON_2 == 0)            
			{
				if(PUSH_BUTTON_2_pressed == FALSE)                
				{
					static BYTE transmitMode = 0;

					PUSH_BUTTON_2_pressed = TRUE;
					ConsolePutROMString((ROM char*)"DEBUG> [sub Main] PUSH_BUTTON_2_pressed ...\n\r ");                   
					TxPayLoad();
					WriteData(USER_REPORT_TYPE);
					//    WriteData(LIGHT_REPORT);
					//   WriteData(LIGHT_TOGGLE);
					WriteData(0x87);
					WriteData(0x86);

					for (i = 0; i < 8; i++)
					{
						tempLongAddress[i] = Le_E2Prom (i);
						PrintChar ( tempLongAddress[i] );
					}

					SendReportByLongAddress(tempLongAddress);
					ConsolePutROMString((ROM char*)"Send Report by Long Address\r\n");
				}
				PUSH_BUTTON_2_press_time = TickGet();
			}
			else
			{
				TICK t = TickGet();

				tickDifference.Val = TickGetDiff(t,PUSH_BUTTON_2_press_time);

				if(tickDifference.Val > DEBOUNCE_TIME)
				{
					PUSH_BUTTON_2_pressed = FALSE;
				}
			}

			//************ pressionou bot�o 1 *********************                              
			if(PUSH_BUTTON_1 == 0)            
			{
				if(PUSH_BUTTON_1_pressed == FALSE)
				{
					PUSH_BUTTON_1_pressed = TRUE;                    
					Flag_Envio_ID = 0;
					ConsolePutROMString((ROM char*)"REENVIO do short addr pro pan coord...\r\n");
				}
				PUSH_BUTTON_1_press_time = TickGet();
			}	
			else
			{
				TICK t = TickGet();    
				tickDifference.Val = TickGetDiff(t,PUSH_BUTTON_1_press_time);
				if(tickDifference.Val > DEBOUNCE_TIME)
				{
					PUSH_BUTTON_1_pressed = FALSE;
				}
			}

#if defined(I_AM_RFD)


			if(CheckForDataComplete()) {
				MRF24J40Sleep();
				while( !ConsoleIsPutReady() );

				WDTCONbits.SWDTEN = 1;      //enable the WDT to wake me up
				INTCONbits.RBIF = 0;
				INTCONbits.RBIE = 1;

				Sleep();            //goto sleep

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
					ConsolePutROMString((ROM char*)"Trying to find a suitable network\r\n");

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
					ConsolePutROMString((ROM char*)"Trying to join network: ");
					PrintChar(handleOfBestNetwork);
					ConsolePutROMString((ROM char*)"\r\n");
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
	WDTCONbits.SWDTEN = 0; //disable WDT

	// Switches S2 and S3 are on RB5 and RB4 respectively. We want interrupt-on-change
	INTCON = 0x00;

	// There is no external pull-up resistors on S2 and S3. We will use internal pull-ups.
	// The MRF24J40 is using INT0 for interrupts
	// Enable PORTB internal pullups
	INTCON2 = 0x00;
	INTCON3 = 0x00;

	// Make PORTB as input - this is the RESET default
	LATB = 0x00;
	TRISB = 0x01;

	TRISD = 0x00;
	LATD = 0x00;

	// Set PORTC control signal direction and initial states
	// disable chip select
	LATC = 0xfd;

	// Set the SPI module for use by Stack
	TRISC = 0xD0;

	// Set the SPI module
	SSPSTAT = 0xC0;
	SSPCON1 = 0x20;

	// D1 and D2 are on RA0 and RA1 respectively, and CS of TC77 is on RA2.
	// Make PORTA as digital I/O.
	//na~o The TC77 temp sensor CS is on RA2.
	ADCON1 = 0x0D; // PIC CUBE A0 e A1 como Anal. restante digital

	//n�o Deselect TC77 (RA2)
	LATA = 0b11011111; // liga DSP2 PIC CUBE

	// n�o Make RA0, RA1, RA2 and RA4 as outputs.
	TRISA = 0b11011111;

	PHY_CS = 1;             //deselect the MRF24J40
	PHY_CS_TRIS = 0;        //make chip select an output   

	RFIF = 0;               //clear the interrupt flag

	RCONbits.IPEN = 1;

	INTCON2bits.INTEDG0 = 0;
	
	//ADC
	OpenADC(ADC_FOSC_16
			&ADC_RIGHT_JUST
			&ADC_4_TAD,
			ADC_CH0
			&ADC_INT_OFF
			&ADC_VREFPLUS_VDD
			&ADC_VREFMINUS_VSS,
			ADC_2ANA);
}
