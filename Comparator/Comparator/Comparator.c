/*
 * GccApplication1.c
 *
 * Created: 5/07/2016 4:16:21 PM
 *  Author: LJ
 */ 

#define F_CPU 16000000UL
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>


#define gateOn	(PORTD |=(1<<2))
#define gateOff (PORTD &= ~(1<<2))
#define IRledon  (PORTD |= (1<<4)) 
#define IRledoff  (PORTD &= ~(1<<4)) 
#define ledOn	(PORTC |= (1<<7))
#define ledOff	(PORTC &= ~(1<<7))
#define  PD6ON  (PORTD |= (1<<6))



ISR(TIMER1_COMPA_vect);

volatile int a;

void setup(void)
{
	DDRD = 0b00010100;//PD2 PWM signal PD4 IR LED.
	PORTD = 0x00;
	DDRC = 0b00000000;
	
	_delay_ms(1000);
	////Timer 1
	TCCR1A |= (1<<COM1A1)|(1<<WGM11); // Fast PWM clear on match set at top
	TCCR1B |= (1<<WGM13)|(1<<CS10)|(1<<WGM12);  // Fast PWM 16MHz
	TIMSK1 |= (1<<OCIE1A);
	
	ICR1 = 1599;
	OCR1A = 32; // 2us on 18us off.
	
	sei();
}


int main(void)
{
	setup();
    while(1)
    {
		while(a < 20)
		{
			ICR1 = 3200;
			OCR1A = 32;
		}
			ICR1 = 1599;
			OCR1A = 32;
		
       if (bit_is_set(PIND,PIND6)) // REACH 100v
       {
		   a=0;
		   TCCR1B = 0;
	       TCCR1A = 0; // Fast PWM clear on match set at top
	       TIMSK1 = 0;
	       PORTD=0x00;
	       //ledOn;
	       //gateOff
	       _delay_ms(50);
	       IRledon;   //Discharge SCR on.
	       _delay_ms(50);
	       IRledoff;
	       //Turn the timer back on  
		   _delay_ms(1000);
	       TCCR1A |= (1<<COM1A1)|(1<<WGM11); // Fast PWM clear on match set at top
		TCCR1B |= (1<<WGM13)|(1<<CS10)|(1<<WGM12);  // Fast PWM 16MHz
	   TIMSK1 |= (1<<OCIE1A);
	   ICR1 = 3200;
	   OCR1A = 32; // 2us on 18us off.
	   //TCCR1B |= (1<<WGM13)|(1<<CS10)|(1<<WGM12);  // Fast PWM 16MHz
       }
	   
    }
}

ISR(TIMER1_COMPA_vect)
{
	a++;
}

