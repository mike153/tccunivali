// *****************************************************************************
//                                                                              
//  Software License Agreement                                                  
//                                                                              
//  Copyright (c) 2007-2008 Microchip Technology Inc.                           
//  All rights reserved.                                                        
//                                                                              
//  Microchip licenses to you the right to use, modify, copy and distribute     
//  Software only when embedded on a Microchip microcontroller or digital       
//  signal controller and used with a Microchip radio frequency transceiver,    
//  which are integrated into your product or third party product (pursuant     
//  to the sublicense terms in the accompanying license agreement).             
//                                                                              
//  You should refer to the license agreement accompanying this Software for    
//  additional information regarding your rights and obligations.               
//                                                                              
//  SOFTWARE AND DOCUMENTATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY     
//  KIND, EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT LIMITATION, ANY          
//  WARRANTY OF MERCHANTABILITY, TITLE, NON-INFRINGEMENT AND FITNESS FOR A      
//  PARTICULAR PURPOSE. IN NO EVENT SHALL MICROCHIP OR ITS LICENSORS BE         
//  LIABLE OR OBLIGATED UNDER CONTRACT, NEGLIGENCE, STRICT LIABILITY,           
//  CONTRIBUTION, BREACH OF WARRANTY, OR OTHER LEGAL EQUITABLE THEORY ANY       
//  DIRECT OR INDIRECT DAMAGES OR EXPENSES INCLUDING BUT NOT LIMITED TO ANY     
//  INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE OR CONSEQUENTIAL DAMAGES, LOST      
//  PROFITS OR LOST DATA, COST OF PROCUREMENT OF SUBSTITUTE GOODS,              
//  TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES (INCLUDING BUT NOT     
//  LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.                    
//                                                                              
//  08/20/08                                                                    
//                                                                              
// *****************************************************************************

// Created by ZENA(TM) Version 3.0.0.0, 6/28/2011, 20:06:25

#ifndef _MIWIDEFS_H_
#define _MIWIDEFS_H_


// Build Configuration
#define ENABLE_CONSOLE
#define PICDEMZ

// MAC Address
#define EUI_7 0x00
#define EUI_6 0x04
#define EUI_5 0xA3
#define EUI_4 0x12
#define EUI_3 0x34
#define EUI_2 0x56
#define EUI_1 0x78
#define EUI_0 0x94
//#define EUI_0 0x95
// PIC Information
#define CLOCK_FREQ 4000000
#define BAUD_RATE 19200

// Device Information
#define I_AM_RFD
#define DEVICE_TYPE 0       // RFD
#define ALTERNATE_PAN_COORDINATOR 0
#define RX_ON_WHEN_IDLE 0
#define POWER_SOURCE 0      // Not Mains
#define ALLOCATE_ADDRESS 1

// Security Configuration
#define SECURITY_CAPABLE 0
#define CAP_INFO ( (((BYTE)ALLOCATE_ADDRESS)<<7) | (((BYTE)SECURITY_CAPABLE)<<6) | (((BYTE)RX_ON_WHEN_IDLE)<<3) | (((BYTE)POWER_SOURCE)<<2) | (((BYTE)DEVICE_TYPE)<<1) | ALTERNATE_PAN_COORDINATOR ) // 0x80

// Transceiver Configuration
#define TMRL TMR0L
#define RFIF INTCONbits.INT0IF
#define RFIE INTCONbits.INT0IE
#define RF_INT_PIN PORTBbits.RB0
#define PHY_CS LATCbits.LATC0
#define PHY_CS_TRIS TRISCbits.TRISC0
#define PHY_RESETn LATCbits.LATC2
#define PHY_RESETn_TRIS TRISCbits.TRISC2
#define PHY_WAKE LATCbits.LATC1
#define PHY_WAKE_TRIS TRISCbits.TRISC1

#define PA_LEVEL 0x00  // -0.00 dBm
#define FREQUENCY_BAND 2400
#define ALLOWED_CHANNELS CHANNEL_25
#define AVAILABLE_CHANNELS_SIZE 1

// Device Configuration Definitions
#define SUPPORT_CLUSTER_SOCKETS
#define SUPPORT_EUI_ADDRESS_SEARCH

// Message Buffers
#define TX_BUFFER_SIZE 127
#define RX_BUFFER_SIZE 127

// Timeouts
#define NETWORK_DISCOVERY_TIMEOUT 0x00007A12
#define OPEN_CLUSTER_SOCKET_TIMEOUT 0x0002DC6C

// Additional NWK/MAC Constants
#define RFD_DATA_WAIT 0x00007A12
#define NETWORK_TABLE_SIZE 10
#define MAX_HOPS 4
#define RSSI_SAMPLES_PER_CHANNEL 5

// Additional Hardware Definitions
#define PUSH_BUTTON_1 PORTBbits.RB5
#define PUSH_BUTTON_2 PORTBbits.RB4
#define LED_1 LATAbits.LATA0
#define LED_2 LATAbits.LATA1
#define PUSH_BUTTON_1_TRIS TRISBbits.TRISB5
#define PUSH_BUTTON_2_TRIS TRISBbits.TRISB4
#define LED_1_TRIS TRISAbits.TRISA0
#define LED_2_TRIS TRISAbits.TRISA1


// Validation

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

#if defined( PICDEMZ) || defined( CUSTOM_PIC18 )
    #if defined(__dsPIC30F__) || defined(__dsPIC33F__) || defined(__PIC24F__) || defined(__PIC24H__)
        #error "Incorrect board/processor combination."
    #endif
#endif

#if defined( EXPLORER16 ) || defined( CUSTOM_16BIT )
    #if defined(__18CXX)
        #error "Incorrect board/processor combination."
    #endif
#endif


#endif
