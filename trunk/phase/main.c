
#include <delays.h>
#include <p18f4520.h>
#include <math.h>
#include <timers.h>


void HighISR(void);



#pragma code HighVector=0x08
void HighVector (void)
{
    _asm GOTO HighISR _endasm
}
#pragma code /* return to default code section */
#pragma interrupt HighISR

void HighISR(void) 
{
	if(INTCON3bits.INT1IF) //Verifica se a INT1 aconteceu
    {
	    INTCON3bits.INT1IF = 0;
        PIE1bits.TMR1IE = 1;
	    T1CONbits.TMR1ON = 1;
	    WriteTimer1(33000);
	 }
	  if (PIR1bits.TMR1IF) 
    {
	    PIR1bits.TMR1IF = 0;
	    CloseTimer1();
        //PORTD ^= 0b11111111;
        PORTBbits.RB4 = 1;
        Delay10TCYx(30);
        PORTBbits.RB4 = 0;        
    }
    
}

void main() {

	LATA = 0b11011111; // liga DSP2 PIC CUBE
	TRISA = 0b11011111;

	TRISB = 0x03; //RB1 como input - INT1
	LATB = 0x00;
	PORTB = 0x00;
	
	TRISD = 0x00;
	LATD = 0x00;
	PORTD = 0x00;
	
	RCONbits.IPEN = 1;
	INTCONbits.GIEH = 1; //Habilita interrupcoes de alta
//	INTCONbits.PEIE = 1;
	INTCON2bits.RBPU = 1; //Desativa todos os pull-ups, a int1 sera acionada por um pulso, logo, precisa estar em 0.
	//INTCONbits.INT0IE = 0;
	
	/* int 1 */	
	INTCON3bits.INT1IE = 1; //Enable INT1
	INTCON3bits.INT1IP = 1; //Alta prioridade
	INTCON2bits.INTEDG1 = 0; //Boarda de subida
	INTCON3bits.INT1IF = 0;	// Int1 flag bit
	/* timer 1 */
	
	OpenTimer1(TIMER_INT_ON
			   &T1_16BIT_RW
			   &T1_SOURCE_INT
			   &T1_OSC1EN_OFF
			   &T1_SYNC_EXT_OFF
			   &T1_PS_1_1);
			  
	
	//WriteTimer1(23869);
//	T1CONbits.TMR1ON = 1; //Jah configurei, agora posso desativar o timer1 e ativa-lo na int1		   
//	T1CONbits.T1RUN = 0;
	PIR1bits.TMR1IF = 0; //Flag bit
//	IPR1bits.TMR1IP = 1; //Alta prioridade
//	PIE1bits.TMR1IE = 1; //Habilita interrupcao tmr1

	PORTBbits.RB4 = 0;
	PORTD = 0;
	while(1) {
		int a = 0;
	};
}