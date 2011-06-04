/*********************************************************************
 *
 *                Microchip C18 Firmware - Serial Communications Demo
 *
 *********************************************************************
 * FileName:        user_serial.c
 * Dependencies:    See INCLUDES section below
 * Processor:       PIC18
 * Compiler:        C18
 * Company:         Lakeview Research LLC (www.Lvr.com)
 *
 * Software License Agreement
 *
 * Licensor grants any person obtaining a copy of this software ("You") 
 * a worldwide, royalty-free, non-exclusive license, for the duration of 
 * the copyright, free of charge, to store and execute the Software in a 
 * computer system and to incorporate the Software or any portion of it 
 * in computer programs You write.   

 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 *
 * Author               Date        Comment
 *~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 * Jan Axelson           10/20/07   Original
 ********************************************************************/

/*
This application demonstrates asynchronous serial communications.
On receiving a lower-case character code on the serial port,
the PIC converts the character to upper case and sends it back.
On receiving any other character, the PIC sends the character code back unchanged.

The port uses hardware flow control.

You can communicate with the PIC using a terminal emulator such as 
HyperTerminal or a custom application. 

This application and PC applications to communicate with the PIC are available
from www.Lvr.com

I tested the application on a PICDEM 2 Plus board with a PIC18F4520. 
*/

/** I N C L U D E S **********************************************************/

#include <p18cxxx.h>
#include <usart.h>
#include "GenericTypeDefs.h"
#include "user_serial.h"

/** V A R I A B L E S ********************************************************/
#pragma udata

// Flow control bits. Use any spare port bits.

#define flow_control_input   PORTBbits.RB4
#define flow_control_output  PORTBbits.RB5

BYTE data_ready_to_send = 0;
unsigned char serial_in;
unsigned char serial_out;

/** P R I V A T E  P R O T O T Y P E S ***************************************/

void transmit_serial_data(void);
void receive_serial_data(void);

/** D E C L A R A T I O N S **************************************************/

#pragma code

/******************************************************************************
 * Function:        void configure_serial_port(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        Configures the UART for asynchronous communications.
 *                  RX = Port C, bit 7 
 *					TX = Port C, bit 6
 *
 * Note:            None
 *****************************************************************************/

void configure_serial_port(void)
{
	// 9600 bps with a 4MHz oscillator

	OpenUSART (USART_TX_INT_OFF & 
 			   USART_RX_INT_OFF &
	           USART_ASYNCH_MODE & 
			   USART_EIGHT_BIT &
			   USART_CONT_RX & 
		 	   USART_BRGH_HIGH, 
			   25);

	// Set the direction of the flow control bits.

	TRISBbits.TRISB4 = 1;
	TRISBbits.TRISB5 = 0;

	// Assert the flow-control output to enable receiving data on the serial port.

	flow_control_output = 0 ;
}

/******************************************************************************
 * Function:        void receive_serial_data(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          serial_out = a character to send on the serial port.
 *					data_ready_to_send = 1 if a byte is ready to send on the 
 *					serial port.
 *
 * Side Effects:    None
 *
 * Overview:  		Check for a received byte on the serial port.
 *					On receiving a byte, prepare to send a byte back.
 *					If the received byte is a lower-case character code, 
 *					convert it to upper case.              
 *
 * Note:           
 *****************************************************************************/

void receive_serial_data(void)
{
	if (PIR1bits.RCIF == 1)
		{  
			// A byte has been received. Read it.

		   	serial_in = getcUSART(); 

			// Tell the remote computer to stop sending data .

			flow_control_output = 1;

			// Set serial_out to a character code to send on the serial port.

			if ((serial_in > 96) & (serial_in < 123))
			{
				// The received character code was lower case (a-z).
				// Send back the character in upper case.

				serial_out = serial_in - 32;
			}
			else
			{
				// For other characters, send back the same character.

				serial_out = serial_in;
			}
			// Set a variable to indicate that a byte is ready for transmitting.
	
			data_ready_to_send = 1;
		}
	}

/******************************************************************************
 * Function:        void transmit_serial_data(void) 
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        If a byte is ready to send and the flow-control input is asserted, send the byte.
 *                  
 *
 * Note:            None
 *****************************************************************************/

void transmit_serial_data(void)
{
	BYTE transmit_data = 1;

	if (data_ready_to_send == 1) 
	{
		// A byte is ready to send.
		// If the flow control input is asserted, send the byte.
		// Otherwise wait until the routine is called again.

		if (flow_control_input == 0)
		{
			data_ready_to_send = 0;
			while(BusyUSART());
			putcUSART(serial_out);

			// Tell the remote computer it's OK to send more data .			
		
			flow_control_output = 0;
		}
	}
}

