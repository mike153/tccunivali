#include "prototypes.h"
#include ".\Common\Console.h"
#include ".\MiWi\MiWi.h"
#include ".\MiWi\MRF24J40.h"
#include ".\MiWi\SymbolTime.h"
#include <adc.h>
#include <p18f4520.h>

// long address do módulo para qual  vai transmitir quando usar este tipo de endereçamento
#pragma romdata eedata2_scn=0xf00000
rom unsigned char eedata2_values[8] = {0x93, 0x78, 0x56, 0x34, 0x12, 0xA3, 0x04, 0x00};
#pragma romdata

// short address do módulo para o qual irá transmitir quando usar este tipo de endereçamento
#pragma romdata eedata3_scn = 0xf00010
rom unsigned char eedata3_values [2] = {0x00, 0x00};
#pragma romdata

// EUI / long address deste Módulo localizado na memória flash rom
// a defininição do EUI / long address deste módulo fica no arquivo MiWiDefs.h
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

#define MAX_DISCOVERY_ATTEMPTS 3
#define DEBOUNCE_TIME 0x00000C35

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
	if (Flag_Envio_ID == 0) //se não enviou ainda envia por pan coord o id de volta em modo serial
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

void miwi_core()
{
	BYTE i;
	BYTE numDiscoveryAttempts;
	BOOL requestedSocket;
	BOOL PUSH_BUTTON_1_pressed;
	TICK PUSH_BUTTON_1_press_time;
	TICK tickDifference;
	BYTE myFriend = 0xFF;
	float temp = 0, ultima_temp = 0;
	
	
	requestedSocket = FALSE;

	numDiscoveryAttempts = 0;
	
	
		LED_3 = 1;
		MiWiTasks();
		LED_3 = 0;
		SetChanADC(ADC_CH0);
		
		temp = le_temperatura();
		
		if(MemberOfNetwork())
		{
			Envio_ID();		//vê se já enviou back ID pro PAN COORd

			//If I am already part of the network
			if(RxPacket())
				pega_cmd();
			
			if (temp != ultima_temp) { //Soh envia reporte quando ocorre alteracao de temperatura
				TxPayLoad();
				WriteData(USER_REPORT_TYPE);
				WriteData(0x87); //TODO: Definir comando
				WriteData(temp);
				ultima_temp = temp;

				for (i = 0; i < 8; i++) {
					tempLongAddress[i] = Le_E2Prom (i);
					PrintChar ( tempLongAddress[i] );
				}
				tempShortAddress.Val = 0x0000; // shjort address do pan coord.
				SendReportByShortAddress(myPANID, tempShortAddress, FALSE);
				ConsolePutROMString((ROM char*)"envio short addr pro pan coord...\r\n");

				//SendReportByLongAddress(tempLongAddress);
				//ConsolePutROMString((ROM char*)"Send Report by Long Address\r\n");
				
			}
			//************ pressionou botão 1 *********************                              
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

}