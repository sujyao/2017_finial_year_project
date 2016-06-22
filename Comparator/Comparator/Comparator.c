	/*
 * GccApplication1.c
 *
 * Created: 2/05/2016 1:59:53 PM
 * Author : jkq0427
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


//global variable
volatile uint8_t count,mcriosec;
volatile uint8_t second;
volatile int a;

//ISR(TIMER1_COMPA_vect);
ISR(ANACOMP1_vect);
ISR(TIMER1_COMPA_vect);
//Timer0
//ISR(TIMER0_COMPA_vect);

void setup(void);

int abc;

int main(void)
{
	setup();
    /* Replace with your application code */
	
    while (1) 
    {
		//ledOn;

		if (bit_is_set(PIND,PIND6)) // REACH 100v
		{
			a=0;	
			//count = 0;
			//abc = 0;
			ledOn;
			gateOff;
			//TCCR1B = 0x00;
			_delay_ms(50);
			IRledon;   //Discharge SCR on.
			_delay_ms(50);
			IRledoff;	
			//Turn the timer back on 
			_delay_ms(1000);
			TCCR1A |= (1<<COM1A1)|(1<<COM1A0); // Set OC1A/OC1B on compare match when upcounting.Clear OC1A/OC1B on compare match when down counting.
			TCCR1B |= (1<<WGM13)|(1<<CS10);
			TIMSK1 |= (1<<OCIE1A);
			ICR1 = 160;
			OCR1A = 144;
		}
		
		  
    }
}
void setup(void)
{
	
	
	DDRD = 0b00010100;//PD2 PWM signal PD4 IR LED.
	PORTD = 0x00;
	DDRC = 0b10000000;
	//PORTC = 0b10000000;
	
	_delay_ms(1000);
	////Timer 1
	TCCR1A |= (1<<COM1A1)|(1<<COM1A0); // Set OC1A/OC1B on compare match when upcounting.Clear OC1A/OC1B on compare match when down counting.
	TCCR1B |= (1<<WGM13)|(1<<CS10);  // PWM, phase and frequency correct. 8MHz.
	TIMSK1 |= (1<<OCIE1A);
	
	ICR1 = 160;
	OCR1A = 144; // 2us on.48us off.
	sei();

	
	//Comparator setup
	ADCSRA &=~(1<<ADEN); // Disable ADC
	AC0CON |= (1<<AC0EN)|(1<<AC0M2)|(1<<AC0M1);//Analog comparator negative input (ACMPM pin);Analog Comparator 0 Enable
	
	AC1CON |=  (1<<AC1EN)|(1<<AC1IE)|(1<<AC1M2)|(1<<AC1M1)|(1<<AC1IS1); // enable interrupt on falling 
	
	ACSR &=(~(1<<AC0O))|(~(1<<AC1O)) ;// set ACO 0
	
	//_delay_us(1);
	a = 0;
}
ISR(ANACOMP1_vect)
{
		//count++;
		if (a >= 1000 && bit_is_clear(PIND,PIND6))
		{
			//count = 0;
			TCCR1A = 0x00;
			TCCR1B = 0x00;
			gateOn;
			_delay_us(4);
			gateOff;
			ledOn;
		}
		
	
}



ISR(TIMER1_COMPA_vect)
{
	a++;
}