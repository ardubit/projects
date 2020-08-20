/*
 * Tamagochi_LED_backlight.c
 *
 * Created: 29.12.2015 21:10:46
 *  Author: Owner
 */ 

#define F_CPU 1200000UL 
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>

#define LED_ON			PORTB |= (1 << PB0)
#define LED_OFF			PORTB &=~(1 << PB0)
#define LED_TOG			PORTB ^= (1 << PB0)
#define BACKLIGHT		30000

int latch;

void setup () {
	// *** Port Setup ***
	
	// PB1 - Button
	// 0 input
	
	// PB0 - LED
	// 1 output
	
	DDRB  |= (0 << PB5) | (0 << PB4) | (0 << PB3) | (0 << PB2) | (0 << PB1) | (1 << PB0);
	PORTB |= (1 << PB5) | (1 << PB4) | (1 << PB3) | (1 << PB2) | (1 << PB1) | (0 << PB0);
	
	// Analog Comparator Disable
	ACSR |= (1 << ACD);
	
	// ********** Interrupts Setup ********** //
	
	// Enable External Interrupt on INT0
	GIMSK |= (1 << INT0);
	
	// The low level will generate an interrupt (H to L)
	MCUCR = (0 << ISC00) | (0 << ISC01);
	
	// *********** Sleep Mode Setup ********** //
	
	// Enable Sleep Mode: Power-down
	set_sleep_mode(SLEEP_MODE_PWR_DOWN);
	
	// ********** Timer Setup ********** // 
	
	// Start timer T0 with prescaler 64.
	TCCR0B |= (1<<CS01) | (1<<CS00);
	
	// Enable Fast PWM
	TCCR0A |= (1 << COM0A0) | (1 << COM0A1) | (1 << WGM01) | (1 << WGM00);
	
	// Reset timer T0 flags.
	TIFR0 = 0;
}

ISR (INT0_vect) {
	//Disable external interrupts (only need one to wake up)
	//GIMSK = 0;
}

void go_to_sleep_mode () {
	//MCUCR |= (1 << SM1);
	//MCUCR |= (1 << SE);
	//asm("sleep");
	
	sleep_enable();
	sei();
	sleep_cpu();
	sleep_disable(); 
}

void fade_out () {
	unsigned int i;
	for (i=0; i<255; i++)
	{
		OCR0A=i;
		_delay_ms(10);
	}
}

void fade_in () {
	unsigned int i;
	for (i=255; i>0; i--)
	{
		OCR0A=i;
		_delay_ms(10);
	}
}

int main(void)
{
	setup();
	sei();
	
    while(1)
    {
		// Please sleep now
		go_to_sleep_mode();
	    
		if (!(PINB & (1 << PB1)) && (latch == 0))
		{
			cli();
			latch=1;
			
			fade_in();
			_delay_ms(BACKLIGHT);
			fade_out();
			
			sei();
		}
		
		if ((PINB & (1 << PB1)) && (latch == 1))
		{
			latch=0;
		}
	}
}
