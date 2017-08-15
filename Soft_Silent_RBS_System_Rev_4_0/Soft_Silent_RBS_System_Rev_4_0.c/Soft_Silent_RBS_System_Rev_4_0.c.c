/*
 * Soft_Silent_RBS_System_Rev_4_0.c
 *
 * Created: 21.07.2016 14:02:14
 * Author: Aleksey
 */ 


#define F_CPU 1200000UL
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <util/atomic.h>

uint8_t flags;
volatile uint8_t ISR_flags;

volatile uint16_t timer_Drive;
volatile uint16_t timer_Ultra;

#define SET_BIT(p,m)	((p) |=(1<<(m)))
#define CLR_BIT(p,m)	((p) &=~(1<<(m)))
#define CHK_BIT(p,m)	((p) & (1<<(m)))
#define TOG_BIT(p,m)	((p) ^=(1<<(m)))

// Bits in the ISR_flags
#define TIMER_LONGPRESS_DRIVE	0	// Bit 0		
#define TIMER_LONGPRESS_ULTRA	1	// Bit 1

// Bits in the general variable flags
#define DRIVE_BUTTON_LATCH 		0		// Bit 0
#define DRIVE_LATCH				1		// Bit 2
#define BOOST_BUTTON_LATCH		2		// Bit 3
#define MOMENTARY_MODE_LATCH	3		// Bit 5
#define DRIVE					4		// Bit 1
#define BOOST					5		// Bit 4
#define A						6		// Bit 6
#define B						7		// Bit 7

#define BYPASS_BUTTON_PRESSED	!(PINB & (1<<PB1))
#define BOOST_BUTTON_PRESSED	!(PINB & (1<<PB0))

#define BYPASS_BUTTON_RELEASED	(PINB & (1<<PB1))
#define BOOST_BUTTON_RELEASED	(PINB & (1<<PB0))

#define MUTE_DELAY			8		// ms
#define RELAY_DELAY			20		// ms
#define LONG_PRESS_TIME		600		// ms
#define DEBOUNCE			45		// ms
#define DEBOUNCE_FAST		25		// ms

#define MUTE_ON				PORTB |= (1<<PB4)
#define MUTE_OFF			PORTB &=~(1<<PB4)

#define RELAY_ON			PORTB |= (1<<PB3)
#define RELAY_OFF			PORTB &=~(1<<PB3)

#define BOOST_ON			PORTB |= (1<<PB2)
#define BOOST_OFF			PORTB &=~(1<<PB2)

void MCU_Setup();
ISR (TIM0_COMPA_vect);
void Drive_ON();
void Drive_OFF();
void Boost_ON();
void Boost_OFF();
void Flags_Handler();
void Drive_Button_Handler();
void Boost_Button_Handler();
void Drive_Long_Click_Button_Handler();
void Ultra_Long_Click_Button_Handler();
void Clear_Timer_Drive();
void Clear_Timer_Ultra();

void MCU_Setup() {
	
	// PB0 - Boost Button (Ultra)
	// PB1 - Bypass Button
	// PB2 - Ultra, LED Boost
	// PB3 - Relay, LED Drive
	// PB4 - Mute MOSFETs
	// PB5 - Reset (Pull-up 10k resistor to +Vcc)
	
	// 0 - input, 1 - output
	DDRB = (0<<PB0)|(0<<PB1)|(1<<PB2)|(1<<PB3)|(1<<PB4)|(0<<PB5);
	
	// Pullup resistors
	PORTB = (1<<PB0)|(1<<PB1)|(1<<PB5);
	
	// Analog comparator OFF
	ACSR |= (1<<ACD);
	
	// Start timer T0 with prescaler 8
	TCCR0B |= (1<<CS01);
	
	// Enable time T0 overflow interrupt
	TIMSK0 |= (1<<OCIE0A);
	
	// Enable CTC Mode.
	TCCR0A |= (1<<WGM01);
	
	// T0 will overflow each 1 ms
	OCR0A = 150;
	
	// Reset timer T0 flags
	TIFR0 = 0;
	
}

ISR (TIM0_COMPA_vect) {
	
	// DRIVE. Long press handler timer
	timer_Drive++;
	if (timer_Drive >= LONG_PRESS_TIME)
	{
		timer_Drive=0;
		SET_BIT(ISR_flags,TIMER_LONGPRESS_DRIVE);
	}
	
	// ULTRA. Long press handler timer
	timer_Ultra++;
	if (timer_Ultra >= LONG_PRESS_TIME)
	{
		timer_Ultra=0;
		SET_BIT(ISR_flags,TIMER_LONGPRESS_ULTRA);
	}
	
}

void Drive_ON() {	
	MUTE_ON;
	_delay_ms(MUTE_DELAY);
	RELAY_ON;
	_delay_ms(RELAY_DELAY);
	MUTE_OFF;	
}

void Drive_OFF() {	
	MUTE_ON;
	_delay_ms(MUTE_DELAY);
	RELAY_OFF;
	_delay_ms(RELAY_DELAY);
	MUTE_OFF;	
}

void Boost_ON() {
	BOOST_ON;
}

void Boost_OFF() {
	BOOST_OFF;
}

void Flags_Handler() {
	
	// **********************************************
	// Bypass
	// **********************************************
	
	if (CHK_BIT(flags,DRIVE) && (!CHK_BIT(flags,DRIVE_LATCH)))
	{
		SET_BIT(flags,DRIVE_LATCH);
		Drive_ON();
	}
	
	if (!CHK_BIT(flags,DRIVE) && (CHK_BIT(flags,DRIVE_LATCH)))
	{
		CLR_BIT(flags,DRIVE_LATCH);
		Drive_OFF();
	}
	
	// **********************************************
	// Boost
	// **********************************************
	
	if (CHK_BIT(flags,BOOST))
	{
		Boost_ON();
	}
	else
	{
		Boost_OFF();
	}
	
}

void Drive_Button_Handler() {
	
	if (BYPASS_BUTTON_PRESSED){
		_delay_ms(DEBOUNCE);
		
		if (BYPASS_BUTTON_PRESSED && (!CHK_BIT(flags,DRIVE_BUTTON_LATCH)) ){
			
			SET_BIT(flags,DRIVE_BUTTON_LATCH);
			TOG_BIT(flags,DRIVE);
			Clear_Timer_Drive();	
			}		
		}
	
	// Button release
	if ((BYPASS_BUTTON_RELEASED) && (CHK_BIT(flags,DRIVE_BUTTON_LATCH)) )
	{
		CLR_BIT(flags,DRIVE_BUTTON_LATCH);
	}
	
}

void Boost_Button_Handler() {
	
	if (BOOST_BUTTON_PRESSED){
		_delay_ms(DEBOUNCE);
		
		if (BOOST_BUTTON_PRESSED && (!CHK_BIT(flags,BOOST_BUTTON_LATCH)) ){
			
			SET_BIT(flags,BOOST_BUTTON_LATCH);
			TOG_BIT(flags,BOOST);
			Clear_Timer_Ultra();
		}
	}
	
	// Button release
	if ((BOOST_BUTTON_RELEASED) && (CHK_BIT(flags,BOOST_BUTTON_LATCH)) )
	{
		CLR_BIT(flags,BOOST_BUTTON_LATCH);
	}
	
}

void Drive_Long_Click_Button_Handler() {
	
	if (BYPASS_BUTTON_PRESSED && (!CHK_BIT(flags,MOMENTARY_MODE_LATCH)) && (CHK_BIT(ISR_flags,TIMER_LONGPRESS_DRIVE)))
	{
		SET_BIT(flags,MOMENTARY_MODE_LATCH);
		TOG_BIT(flags,DRIVE);
			
		while (BYPASS_BUTTON_PRESSED) {
		Boost_Button_Handler();// Do nothing
		asm("nop");	
		}
		
	}
	
	// Button release
	if ((BYPASS_BUTTON_RELEASED) && (CHK_BIT(flags,MOMENTARY_MODE_LATCH)) )
	{
		CLR_BIT(flags,MOMENTARY_MODE_LATCH);
	}
	
	if (BYPASS_BUTTON_RELEASED)
	{
		ATOMIC_BLOCK(ATOMIC_FORCEON)
		{
			CLR_BIT(ISR_flags,TIMER_LONGPRESS_DRIVE);
		}
	}	
}

void Ultra_Long_Click_Button_Handler() {
	
	if (BOOST_BUTTON_PRESSED && (!CHK_BIT(flags,MOMENTARY_MODE_LATCH)) && (CHK_BIT(ISR_flags,TIMER_LONGPRESS_ULTRA)))
	{
		SET_BIT(flags,MOMENTARY_MODE_LATCH);
		TOG_BIT(flags,BOOST);
		
		while (BOOST_BUTTON_PRESSED) {
			// Do nothing
			Drive_Button_Handler();
			asm("nop");
		}
		
	}
	
	// Button release
	if ((BOOST_BUTTON_RELEASED) && (CHK_BIT(flags,MOMENTARY_MODE_LATCH)) )
	{
		CLR_BIT(flags,MOMENTARY_MODE_LATCH);
	}
	
	if (BOOST_BUTTON_RELEASED)
	{
		ATOMIC_BLOCK(ATOMIC_FORCEON)
		{
			CLR_BIT(ISR_flags,TIMER_LONGPRESS_ULTRA);
		}
	}
}

void Clear_Timer_Drive() {
	
	// Clear "One Click" long press handling timer
	ATOMIC_BLOCK(ATOMIC_FORCEON)
	{
		timer_Drive = 0;
		CLR_BIT(ISR_flags,TIMER_LONGPRESS_DRIVE);
	}
	
}

void Clear_Timer_Ultra() {
	
	// Clear "One Click" long press handling timer
	ATOMIC_BLOCK(ATOMIC_FORCEON)
	{
		timer_Ultra = 0;
		CLR_BIT(ISR_flags,TIMER_LONGPRESS_ULTRA);
	}
	
}

int main(void)
{
	MCU_Setup();
	sei();
    while(1)
    {
        Drive_Button_Handler();
		Boost_Button_Handler();
        Flags_Handler();
        Drive_Long_Click_Button_Handler();
		Ultra_Long_Click_Button_Handler();
    }
}