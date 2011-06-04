/*********************************************************************
 *
 *                Microchip C18 Firmware - Serial Communications Demo
 *
 *********************************************************************
 * FileName:        user_serial.h
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

#ifndef USER_H
#define USER_H

/** P U B L I C  P R O T O T Y P E S *****************************************/

void configure_serial_port(void);
void receive_serial_data(void);
void transmit_serial_data(void);
     
#endif //USER_H
