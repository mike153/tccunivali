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
 *****************************************************************************
 *
 * Change History:
 *  Rev   Date         Author	Description
 *  0.1   11/09/2006   df       Initial revision
 *  1.0   01/09/2007   yfy      Initial release
 *****************************************************************************/
#include ".\Common\Console.h"
#include ".\MiWi\MiWi.h"
#include ".\MiWi\MRF24J40.h"
#include ".\MiWi\SymbolTime.h"
#include <usart.h>

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


/********************** FUNCTION (&)PROTOTYPES *************************/


void BoardInit(void);



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
/********************************************************************/
/********************** CONSTANTS ***********************************/
/**********************  M A I N ************************************/
/********************************************************************/
/********************************************************************/
/********************************************************************/

#define MAX_DISCOVERY_ATTEMPTS 3
#define DEBOUNCE_TIME 0x00000C35

#if !defined(P2P_ONLY)
#if defined(__18CXX)
void main(void)
#else
int main(void)
#endif
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
   
    requestedSocket = FALSE;
    
    numDiscoveryAttempts = 0;
        
    ConsoleInit();  

    BoardInit(); 
		ConsolePutROMString((ROM char*)"DEBUG> [sub Main] BoardInit OK ...\n\r ");

    MiWiInit();
		ConsolePutROMString((ROM char*)"DEBUG> [sub Main] MiWiInit OK ...\n\r ");
   
	#if defined(PICDEMZ)
    INTCONbits.GIEH = 1;
    #elif defined(EXPLORER16)
		#error "definida explorer16"
    #else
        #error "Unknown board.  Please initialize board as required."
    #endif

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
		LED_3 = 1;
        MiWiTasks();
		LED_3 = 0;

        if(MemberOfNetwork())
        {

			Envio_ID();		//vê se já enviou back ID pro PAN COORd

            //If I am already part of the network
            if(RxPacket())
            {
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
            

//************ pressionou botão 2 ********************* 
            if(PUSH_BUTTON_2 == 0)            
            {
                if(PUSH_BUTTON_2_pressed == FALSE)                
                {
                    static BYTE transmitMode = 0;
                    
                    PUSH_BUTTON_2_pressed = TRUE;
                    ConsolePutROMString((ROM char*)"DEBUG> [sub Main] PUSH_BUTTON_2_pressed ...\n\r ");                   
                    TxPayLoad();
                    WriteData(USER_REPORT_TYPE);
                    WriteData(LIGHT_REPORT);
                    WriteData(LIGHT_TOGGLE);
					
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
                
                #ifdef __PIC32MX__
                
                if(CheckForDataComplete() && (PUSH_BUTTON_1 != 0) && (PUSH_BUTTON_2 != 0))
                
                #else
                
                if(CheckForDataComplete())
                
                #endif
                {
                    MRF24J40Sleep();
                    while( !ConsoleIsPutReady() );
                    
                    #if defined(__18CXX)
                        WDTCONbits.SWDTEN = 1;      //enable the WDT to wake me up
                        INTCONbits.RBIF = 0;
                        INTCONbits.RBIE = 1;
                    #elif defined(__dsPIC30F__) || defined(__dsPIC33F__) || defined(__PIC24F__) || defined(__PIC24H__)
                        ClrWdt();
                        RCONbits.SWDTEN = 1;        //enable the WDT to wake me up
                        IEC1bits.CNIE = 1;
                    #elif defined(__PIC32MX__)
                        /* Enable Change Notice Push Button 1 & 2 interrupt, so that on pushing 
                           button 1 or 2 the controller wakes up. Disabling of Change notice
                           module & interrupt is done in ISR */
                        Enable_PB_1_2_Interrupts();
                        
                        EnableWDT();   // Enable Watchdog
                        ClearWDT();   // Clear the Watchdog Timer
                    #endif
                    
                    Sleep();            //goto sleep
                    
                    #if defined(__18CXX)
                        INTCONbits.RBIE = 0;
                        WDTCONbits.SWDTEN = 0;      //disable WDT
                    #elif defined(__dsPIC30F__) || defined(__dsPIC33F__) || defined(__PIC24F__) || defined(__PIC24H__)
                        RCONbits.SWDTEN = 0;        //disable WDT
                        IEC1bits.CNIE = 0;
                    #elif defined(__PIC32MX__)
                        RCONCLR = RCON_SLEEP_MASK | RCON_IDLE_MASK;   // clear the Sleep and Idle bits
                        /* Disable Watchdog Timer */
                        DisableWDT();
                    #endif

                    MRF24J40Wake();
                    
                    CheckForData();     //when I wake up do a data request
                }
            #endif
                        
            //END OF THE CODE AFTER YOU BECAME A MEMBER
        }
        else
        {  
            #if !defined(P2P_ONLY)
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
                        #if defined(I_AM_COORDINATOR_CAPABLE)
                            //form a network with the specified PANID
                            FormNetwork(0xFFFF);
                            {
                                WORD i;
                                
                                LED_1 = 1;
                                
                                for(i=0;i<20000;i++)
                                {
                                }
                                
                                LED_1 = 0;
                                
                                for(i=0;i<20000;i++)
                                {
                                }
                                
                                LED_1 = 1;
    
                                for(i=0;i<20000;i++)
                                {
                                }
                                
                                LED_1 = 0;                            
                            }
                        #endif

                        #if !defined(P2P_ONLY)
                        //clear all of the network entries
                            ClearNetworkTable(CLEAR_NETWORKS);
                        #endif
                        
                    }
                    else
                    {
                        #if !defined(P2P_ONLY)	
                            //clear all of the network entries
                            ClearNetworkTable(CLEAR_NETWORKS);
                        #endif
                        
                        //and start a new search
                        DiscoverNetworks();
                    }
                }
                else
                {
                    #if !defined(P2P_ONLY)
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
                    #endif
                }
            }
            #endif //P2P_ONLY
            //if I am searching for a network then I will leave it alone so I can continue searching
            //or if I am trying to join a network then I will let that join proccess finish
        }
    }
}

#else
#error "definido P2P only!! "
/**********************************************************************************
*
*                      DEMO code for P2P only devices
*
***********************************************************************************/
//DEMO for P2P Only devices
void main(void)
{
    BOOL PUSH_BUTTON_1_pressed;
    TICK PUSH_BUTTON_1_press_time, tickDifference;
    BOOL FoundSocket;
    BYTE myFriend;
    
    ConsoleInit();
    BoardInit();  
    MiWiInit();

    #if defined(PICDEMZ)
    INTCONbits.GIEH = 1;
    #elif defined(EXPLORER16)
    #else
        #error "Unknown board.  Please initialize board as required."
    #endif
    
    SetChannel(CHANNEL_25);

    FoundSocket = FALSE;
    
    while(1)
    {
        MiWiTasks();
        
        if(FoundSocket == TRUE)
        {
            if(RxPacket())
            {
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
            
            if(PUSH_BUTTON_1 == 0)
            {
                if(PUSH_BUTTON_1_pressed == FALSE)
                {                   
                    PUSH_BUTTON_1_pressed = TRUE;

					TxPayLoad();
                    WriteData(USER_REPORT_TYPE);
                    WriteData(LIGHT_REPORT);
                    WriteData(LIGHT_TOGGLE);

                    SendReportByLongAddress(openSocketInfo.LongAddress1);

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
            
        }
        else
        {
            if(OpenSocketComplete())
            {                
                if(OpenSocketSuccessful())
                {
                    FoundSocket = TRUE;
                    LED_1 = 1;
                    ConsolePutROMString((ROM char*)"Found a socket: ");
                    ConsolePutROMString((ROM char*)"\r\n");
                }
                else
                {
                    ConsolePutROMString((ROM char*)"sending request\r\n");
                    //else we need to try again (or for the first time)
                    OpenSocket(P2P_SOCKET);
                }
            }
        }
    }
}
#endif

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
 #if defined(PICDEMZ)
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

    //não Deselect TC77 (RA2)
    LATA = 0b11011111; // liga DSP2 PIC CUBE

    // não Make RA0, RA1, RA2 and RA4 as outputs.
    TRISA = 0b11011111;
    
    PHY_CS = 1;             //deselect the MRF24J40
    PHY_CS_TRIS = 0;        //make chip select an output   
    
    RFIF = 0;               //clear the interrupt flag

    RCONbits.IPEN = 1;
    
    INTCON2bits.INTEDG0 = 0;
}
#elif defined(EXPLORER16)
void BoardInit(void)
{
    #ifdef __PIC32MX__
#error "definido explorer16 "
       unsigned int pbFreq;

       /* Clear SPI1CON register */
       SPI1CONCLR = 0xFFFFFFFF;

       /* Enable SPI1, Set to Master Mode & Set CKE bit : Serial output data changes on transition 
          from active clock state to Idle clock state */
       SPI1CON = 0x00008120;
       
       /* Peripheral Bus Frequency = System Clock / PB Divider */
       pbFreq = (DWORD)CLOCK_FREQ / (1 << mOSCGetPBDIV() );

       /* PB Frequency can be maximum 40 MHz */
       if( pbFreq > ( 2 * MAX_SPI_CLK_FREQ_FOR_MIWI) )
       {
           {
               unsigned int SPI_Clk_Freq;
               
               unsigned char SPI_Brg = 1;
               
               /* Continue the loop till you find SPI Baud Rate Register Value */
               while(1)
               {
                  /* SPI Clock Calculation as per PIC32 Manual */
                  SPI_Clk_Freq = pbFreq / ( 2 * ( SPI_Brg + 1) );
                  
                  if( SPI_Clk_Freq <= MAX_SPI_CLK_FREQ_FOR_MIWI )
                  {
                      break;
                  }
               
                  SPI_Brg++;
               }
       
              mSpiChnSetBrg(1,SPI_Brg);
           
           }
       }
       else
       {
           /* Set SPI1 Baud Rate */
           mSpiChnSetBrg(1,0);
       }

       /* Set the Port Directions of SDO, SDI, Clock & Slave Select Signal */
       mPORTFSetPinsDigitalOut(PIC32MX_SPI1_SDO_SCK_MASK_VALUE);
       mPORTFSetPinsDigitalIn(PIC32MX_SPI1_SDI_MASK_VALUE);

       /* Set the INT1 port pin to input */
       mPORTESetPinsDigitalIn(PIC32MX_INT1_MASK_VALUE);
       
       /* Set the Interrupt Priority */
       mINT1SetIntPriority(4);

       /* Set Interrupt Subpriority Bits for INT2 */
       mINT1SetIntSubPriority(2);

       /* Set INT2 to falling edge */
       mINT1SetEdgeMode(0);

       /* Set the Interrupt Priority for Push Button 2 & 3 as '5' */
       mCNSetIntPriority(5);
       
       /* Set the Interrupt Priority for Push Button 2 & 3 as '1' */
       mCNSetIntSubPriority(1);

       /* Enable Multi Vectored Interrupts */
       INTEnableSystemMultiVectoredInt();    
    
    #else

       SPI1CON1 = 0b0000000100111110;
       SPI1STAT = 0x8000;
       
       CNEN1bits.CN15IE = 1;
       CNEN2bits.CN16IE = 1;
       INTCON2bits.INT1EP = 1;
    #endif
    
/*    PHY_RESETn = 0;
    PHY_RESETn_TRIS = 0;
    PHY_CS = 1;
    PHY_CS_TRIS = 0;
    PHY_WAKE = 1;
    PHY_WAKE_TRIS = 0;
    
    LED_1_TRIS = 0;
    LED_2_TRIS = 0;
    PUSH_BUTTON_1_TRIS = 1;
    PUSH_BUTTON_2_TRIS = 1;*/

    PHY_RESETn = 0;
    PHY_RESETn_TRIS = 0;
    PHY_CS = 1;
    PHY_CS_TRIS = 0;
    PHY_WAKE = 1;
    PHY_WAKE_TRIS = 0;
    LED_1_TRIS = 0;
    LED_2_TRIS = 0;
	LED_3_TRIS = 0;

    PUSH_BUTTON_1_TRIS = 1;
    PUSH_BUTTON_2_TRIS = 1;

    RFIF = 0;
    RFIE = 1;

}
#else
    #error "Unknown demo board.  Please properly initialize the part for the board."
#endif


/*********************************************************************
 * Function:        void Enable_PB_1_2_Interrupts()
 *
 * PreCondition:    None
 * 
 * Input:           None
 * 
 * Output:          None
 * 
 * Side Effects:    None
 * 
 * Overview:        Configure/Enable PB 1 & 2 interrupts.
 *  
 * Note:            None
 ********************************************************************/

#ifdef __PIC32MX__

static void Enable_PB_1_2_Interrupts(void)
{
    unsigned int value;

    CNCON = 0x8000;  // Enable Change Notice module

    /* Configure Change Notice Registers for Push Button 1 & 2. These buttons can be 
       used to wake the controller from sleep state. */
    CNEN  = 0x00018000; // Enable CN15 and CN16 pins

    /* Read port to clear mismatch on change notice pins */
    value = PORTD;

    mCNClearIntFlag(); // Clear the CN interrupt flag status bit

    mCNIntEnable(1); // Enable Change Notice interrupts
}

#endif

/*********************************************************************
 * Function:        void _CN_Interrupt_ISR(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None.
 *
 * Overview:        This is the interrupt handler for push button 1 & 2 to 
 *                  wake up the controller in Sleep State.
 ********************************************************************/
#if defined(__dsPIC30F__) || defined(__dsPIC33F__) || defined(__PIC24F__) || defined(__PIC24H__)
    void _ISRFAST __attribute__((interrupt, auto_psv)) _CNInterrupt(void) 
    {
        IFS1bits.CNIF = 0;
    }
#elif defined(__PIC32MX__)
    
void __ISR(_CHANGE_NOTICE_VECTOR, ipl5) _CN_Interrupt_ISR(void)
{
    /* Disable Watchdog Timer to make sure that reset doesn't happen */
    DisableWDT();
    
    {
        unsigned int value1;

        value1 = PORTD; // Read PORTD to clear CN15/16 mismatch condition

        if(value1 & 0x80 )
        {
           Pushbutton_1_Wakeup_Pressed = 1;
        }
        else
        {
           Pushbutton_2_Wakeup_Pressed = 1;
        }
    }
    

    mCNIntEnable(0); // Disable the CN Interrupt 

    CNCON = 0x0000; // Disable CN Module 

    CNEN = 0x0000;
    
    mCNClearIntFlag(); // Clear the CN interrupt flag status bit

}

#endif

