/*********************************************************************
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
 *                                                                    
 *********************************************************************/

// Created by ZENA(TM) Version 1.2.1.0, 8/17/2006, 8:38:24

#define ENABLE_TABLE_DUMP

#define ENABLE_CONSOLE

#ifndef _MIWI_DEFS_H_
#define _MIWI_DEFS_H_

//#define P2P_ONLY

#if defined(__18CXX)
    #define PICDEMZ
#endif

#if defined(__dsPIC30F__) || defined(__dsPIC33F__) || defined(__PIC24F__) || defined(__PIC24H__) || defined(__PIC32MX__)
    #define EXPLORER16
#endif

#if defined(PICDEMZ)
    #if defined(__dsPIC30F__) || defined(__dsPIC33F__) || defined(__PIC24F__) || defined(__PIC24H__) || defined(__PIC32MX__)
        #error "Incorrect board/processor combination."
    #endif
#endif

#if defined(EXPLORER16)
    #if defined(__18CXX)
        #error "Incorrect board/processor combination."
    #endif
#endif

//IEEE EUI - globally unique number
#define EUI_7 0x54
#define EUI_6 0x01
#define EUI_5 0x02
#define EUI_4 0x03
#define EUI_3 0x04
#define EUI_2 0x05
#define EUI_1 0x06
#define EUI_0 0x07

#if defined(PICDEMZ)

    #define CLOCK_FREQ 20000000
    // Transceiver Configuration
    #define RFIF INTCONbits.INT0IF
    #define RFIE INTCONbits.INT0IE

    
	#define PHY_CS LATCbits.LATC2
    #define PHY_CS_TRIS TRISCbits.TRISC2
    #define PHY_RESETn LATCbits.LATC0
    #define PHY_RESETn_TRIS TRISCbits.TRISC0
    #define PHY_WAKE LATCbits.LATC1
    #define PHY_WAKE_TRIS TRISCbits.TRISC1
    
    #define PUSH_BUTTON_1 PORTAbits.RA2
    #define PUSH_BUTTON_2 PORTAbits.RA3
    #define LED_1 PORTDbits.RD7
    #define LED_2 PORTDbits.RD1
    #define LED_3 PORTDbits.RD5     //******************
	#define LED_5 PORTDbits.RD6		//defs de acordo com
	#define LED_6 PORTDbits.RD0		// PIC CUBE !!!
	#define LED_7 PORTDbits.RD2		//******************

    #define PUSH_BUTTON_1_TRIS TRISA2
    #define PUSH_BUTTON_2_TRIS TRISA3
//	#define PUSH_BUTTON_3_TRIS TRISA3
//	#define PUSH_BUTTON_4_TRIS TRISA4
    
	#define LED_1_TRIS TRISD7
    #define LED_2_TRIS TRISD1

    #define LED_3_TRIS TRISD5

    #define LED_5_TRIS TRISD6
	#define LED_6_TRIS TRISD0
	#define LED_7_TRIS TRISD2
	
    #define RF_INT_PIN PORTBbits.RB0

    #define TMRL TMR0L

#elif defined(EXPLORER16)
    #error "definido explorer 16 !!!"
    #ifdef __PIC32MX__
        #define CLOCK_FREQ 64000000
    #else
        #define CLOCK_FREQ 8000000
    #endif
    
    #if defined(__PIC24F__) || defined(__PIC32MX__)    
		#error "DEFINIDO __PIC24F__ !!!!"
        #define RF_INT_PIN PORTEbits.RE8    
    #elif defined(__dsPIC33F__) || defined(__PIC24H__)
		#error "DEFINIDO __dsPIC33F__ !!!!"
        #define RF_INT_PIN PORTAbits.RA12
    #endif
    
    #define TMRL TMR2
// Transceiver Configuration

    #ifdef __PIC32MX__
		#error "DEFINIDO __PIC32MX__ !!!!"
        #define RFIF IFS0bits.INT1IF
        #define RFIE IEC0bits.INT1IE
    #else    
        #define RFIF IFS1bits.INT1IF
        #define RFIE IEC1bits.INT1IE
    #endif

    #define PHY_CS LATBbits.LATB2
    #define PHY_CS_TRIS TRISBbits.TRISB2
    #define PHY_RESETn LATGbits.LATG2
    #define PHY_RESETn_TRIS TRISGbits.TRISG2
    #define PHY_WAKE LATGbits.LATG3
    #define PHY_WAKE_TRIS TRISGbits.TRISG3
    
    #define PUSH_BUTTON_1 PORTDbits.RD6
    #define PUSH_BUTTON_2 PORTDbits.RD7
    #define LED_1 LATAbits.LATA7
    #define LED_2 LATAbits.LATA6
    
    #define PUSH_BUTTON_1_TRIS TRISDbits.TRISD6
    #define PUSH_BUTTON_2_TRIS TRISDbits.TRISD7
    #define LED_1_TRIS TRISAbits.TRISA7
    #define LED_2_TRIS TRISAbits.TRISA6

#else
    #error "Unknown demo board.  Please select on in MiWiDefs.h"
#endif

#define SUPPORT_CLUSTER_SOCKETS
#define SUPPORT_P2P_SOCKETS
#define SUPPORT_EUI_ADDRESS_SEARCH

// PICmicro Information

#define BAUD_RATE 19200

// Device Information
//#define I_AM_FFD
#define I_AM_RFD
#define DEVICE_TYPE 0       // RFD
//#define I_AM_COORDINATOR_CAPABLE
#define ALTERNATE_PAN_COORDINATOR 0
#define RX_ON_WHEN_IDLE 0
#define POWER_SOURCE 0      // Not Mains
#define SECURITY_CAPABLE 0
#define ALLOCATE_ADDRESS 1
#define CAP_INFO ( (((BYTE)ALLOCATE_ADDRESS)<<7) | (((BYTE)SECURITY_CAPABLE)<<6) | (((BYTE)RX_ON_WHEN_IDLE)<<3) | (((BYTE)POWER_SOURCE)<<2) | (((BYTE)DEVICE_TYPE)<<1) | ALTERNATE_PAN_COORDINATOR ) // 0x8B


#define PA_LEVEL 0x00  // -0.00 dBm
#define FREQUENCY_BAND 2400
#define ALLOWED_CHANNELS CHANNEL_25
#define AVAILABLE_CHANNELS_SIZE 1

// Message Buffers
#define TX_BUFFER_SIZE 127
#define RX_BUFFER_SIZE 127

// Timeouts
#define NETWORK_DISCOVERY_TIMEOUT 0x00007A12
#define OPEN_CLUSTER_SOCKET_TIMEOUT 0x000061A8

#define RFD_DATA_WAIT 0x00000C35

// Indirect Buffer Management
#define INDIRECT_BUFFER_SIZE 127
#define INDIRECT_BUFFER_TIMEOUT 0x0002DC6C

// Additional NWK/MAC Constants
#define NETWORK_TABLE_SIZE 10
#define MAX_HOPS 4

//#define SUPPORT_SECURITY
#define SECURITY_KEY_00 0x00
#define SECURITY_KEY_01 0x01
#define SECURITY_KEY_02 0x02
#define SECURITY_KEY_03 0x03
#define SECURITY_KEY_04 0x04
#define SECURITY_KEY_05 0x05
#define SECURITY_KEY_06 0x06
#define SECURITY_KEY_07 0x07
#define SECURITY_KEY_08 0x08
#define SECURITY_KEY_09 0x09
#define SECURITY_KEY_10 0x0a
#define SECURITY_KEY_11 0x0b
#define SECURITY_KEY_12 0x0c
#define SECURITY_KEY_13 0x0d
#define SECURITY_KEY_14 0x0e
#define SECURITY_KEY_15 0x0f
#define KEY_SEQUENCE_NUMBER 0x00
#define SECURITY_LEVEL 0x04

// Constants Validation

// ZENA(TM) will automatically range check the entered values.  These range
// checks are included here in cases the application developer manually
// adjusts the values.

#if (RX_BUFFER_SIZE > 127)
    #error RX BUFFER SIZE too large. Must be <= 127.
#endif

#if (TX_BUFFER_SIZE > 127)
    #error TX BUFFER SIZE too large. Must be <= 127.
#endif

#if (RX_BUFFER_SIZE < 25)
    #error RX BUFFER SIZE too small. Must be >= 25.
#endif

#if (TX_BUFFER_SIZE < 25)
    #error TX BUFFER SIZE too small. Must be >= 25.
#endif

#if (NETWORK_TABLE_SIZE == 0)
    #error NETWORK TABLE SIZE too small.
#endif

#if (NETWORK_TABLE_SIZE > 0xFE)
    #error NETWORK TABLE SIZE too large.  Must be < 0xFF.
#endif

#if (INDIRECT_BUFFER_SIZE > 0xFE)
    #error INDIRECT BUFFER SIZE too large.  Must be < 0xFF.
#endif

#endif
