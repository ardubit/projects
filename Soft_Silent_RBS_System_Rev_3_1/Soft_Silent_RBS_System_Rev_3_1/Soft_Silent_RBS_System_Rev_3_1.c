/*
 * Soft_Silen__RBS_System_Rev_3_1.c
 * Soft_&_Silen_ RBS_System
 * Created: 03.10.2015 21:51:34
 * Author: Aleksey
 * The system is running on MCU ATTiny13
 *
 */

#define F_CPU 1200000UL
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <util/atomic.h>

uint8_t flags;
volatile uint8_t ISR_flags;

volatile uint16_t timer_2_counter_long_press;		// Long press handling timer
volatile uint16_t timer_3_counter_double_click; // Double click handling timer

uint8_t click_counter;

#define SET_BIT(p, m) ((p) |= (1 << (m)))
#define CLR_BIT(p, m) ((p) &= ~(1 << (m)))
#define CHK_BIT(p, m) ((p) & (1 << (m)))
#define TOG_BIT(p, m) ((p) ^= (1 << (m)))

// Bits in the ISR_flags
#define TIMER_2_LONG_PRESS 1	 // Bit 1
#define TIMER_3_DOUBLE_CLICK 2 // Bit 1

// Bits in the general variable flags
#define BUTTON_LATCH 0				// Bit 0
#define DRIVE_LATCH 1					// Bit 2
#define ONECLICK_MODE_LATCH 2 // Bit 3
#define DRIVE_MODE_LATCH 3		// Bit 5
#define DRIVE 4								// Bit 1
#define BOOST 5								// Bit 4
#define CLICK_TO_BOOST 6			// Bit 6
#define TIP_MODE 7						// Bit 7

#define BUTTON_PRESSED !(PINB & (1 << PB1))

// Click - 160 ms
#define LONG_PRESS_TIME 600		// ms
#define DOUBLE_CLICK_TIME 400 // ms
#define DEBOUNCE 45						// ms
#define DEBOUNCE_FAST 25			// ms
#define MUTE_DELAY 8					// ms
#define RELAY_DELAY 20				// ms

#define LED_GREEN_B_TOG PORTB ^= (1 << PB2)
#define LED_GREEN_B_ON PORTB |= (1 << PB2)
#define LED_GREEN_B_OFF PORTB &= ~(1 << PB2)

#define LED_RED_D_TOG PORTB ^= (1 << PB3)
#define LED_RED_D_ON PORTB |= (1 << PB3)
#define LED_RED_D_OFF PORTB &= ~(1 << PB3)

#define RELAY_TOG PORTB ^= (1 << PB0)
#define RELAY_ON PORTB |= (1 << PB0)
#define RELAY_OFF PORTB &= ~(1 << PB0)

#define RELAY_ON PORTB |= (1 << PB0)
#define RELAY_OFF PORTB &= ~(1 << PB0)

#define MUTE_ON PORTB |= (1 << PB4)
#define MUTE_OFF PORTB &= ~(1 << PB4)

#define DRIVE_LED_ON PORTB |= (1 << PB3)
#define DRIVE_LED_OFF PORTB &= ~(1 << PB3)

#define BOOST_ON PORTB |= (1 << PB2)
#define BOOST_OFF PORTB &= ~(1 << PB2)

ISR(TIM0_COMPA_vect)
{

	// Long press handling timer "One Click"
	timer_2_counter_long_press++;
	if (timer_2_counter_long_press >= LONG_PRESS_TIME)
	{
		timer_2_counter_long_press = 0;
		SET_BIT(ISR_flags, TIMER_2_LONG_PRESS);
	}

	// Double click handling timer
	timer_3_counter_double_click++;
	if (timer_3_counter_double_click >= DOUBLE_CLICK_TIME)
	{
		timer_3_counter_double_click = 0;
		SET_BIT(ISR_flags, TIMER_3_DOUBLE_CLICK);
	}
}

void MCU_Setup()
{

	// PB0 - Relay
	// PB1 - Button
	// PB2 - LED Boost
	// PB3 - LED Drive
	// PB4 - Mute MOSFETs
	// PB5 - Reset (Pull-up 10k resistor to +Vcc)

	DDRB = (1 << PB0) | (0 << PB1) | (1 << PB2) | (1 << PB3) | (1 << PB4) | (0 << PB5);
	PORTB = (1 << PB1) | (1 << PB5);

	// Analog comparator OFF
	ACSR |= (1 << ACD);

	// Start timer T0 with prescaler 8
	TCCR0B |= (1 << CS01);

	// Enable time T0 overflow interrupt
	TIMSK0 |= (1 << OCIE0A);

	// Enable CTC Mode.
	TCCR0A |= (1 << WGM01);

	// T0 will overflow each 1 ms
	OCR0A = 150;

	// Reset timer T0 flags
	TIFR0 = 0;
}

void Blinking()
{
	for (int i = 5; i >= 0; i--)
	{
		LED_RED_D_TOG;
		LED_GREEN_B_TOG;
		_delay_ms(200);
	}
}

void Bypass_Mode_Checker()
{
	if (BUTTON_PRESSED)
	{

		_delay_ms(DEBOUNCE_FAST);

		if (BUTTON_PRESSED)
		{

			_delay_ms(800);
			SET_BIT(flags, CLICK_TO_BOOST);
			Blinking();

			while (!(PINB & (1 << PB1)))
			{
				// Do nothing
				asm("nop");
			}
		}
	}
}

void Drive_ON()
{
	MUTE_ON;
	_delay_ms(MUTE_DELAY);
	RELAY_ON;
	_delay_ms(RELAY_DELAY);
	DRIVE_LED_ON;
	MUTE_OFF;
}

void Drive_OFF()
{
	MUTE_ON;
	_delay_ms(MUTE_DELAY);
	RELAY_OFF;
	_delay_ms(RELAY_DELAY);
	DRIVE_LED_OFF;
	MUTE_OFF;
}

void Clear_Timer_2_Long_Press()
{
	// Clear "One Click" long press handling timer
	ATOMIC_BLOCK(ATOMIC_FORCEON)
	{
		timer_2_counter_long_press = 0;
		CLR_BIT(ISR_flags, TIMER_2_LONG_PRESS);
	}
}

void Clear_Timer_3_Double_Click()
{
	// Clear double click handling timer
	ATOMIC_BLOCK(ATOMIC_FORCEON)
	{
		timer_3_counter_double_click = 0;
		CLR_BIT(ISR_flags, TIMER_3_DOUBLE_CLICK);
	}
}

void Flags_Handler()
{

	// **********************************************
	// Bypass
	// **********************************************

	if (CHK_BIT(flags, DRIVE) && (!CHK_BIT(flags, DRIVE_LATCH)))
	{
		SET_BIT(flags, DRIVE_LATCH);
		Drive_ON();
	}

	if (!CHK_BIT(flags, DRIVE) && (CHK_BIT(flags, DRIVE_LATCH)))
	{
		CLR_BIT(flags, DRIVE_LATCH);
		Drive_OFF();
	}

	// **********************************************
	// Boost
	// **********************************************

	if (CHK_BIT(flags, BOOST))
	{
		BOOST_ON;
	}
	else
	{
		BOOST_OFF;
	}
}

void Long_Click_Button_Handler()
{
	if (!(PINB & (1 << PB1)) && (!CHK_BIT(flags, ONECLICK_MODE_LATCH)) && (CHK_BIT(ISR_flags, TIMER_2_LONG_PRESS)))
	{
		SET_BIT(flags, ONECLICK_MODE_LATCH);

		// **********************************************
		// Section "Click To Boost"
		// **********************************************

		// Default Mode
		if (!CHK_BIT(flags, CLICK_TO_BOOST))
		{
			TOG_BIT(flags, DRIVE);
		}

		// Click To Boost Mode
		if (CHK_BIT(flags, CLICK_TO_BOOST))
		{

			if (!CHK_BIT(flags, TIP_MODE))
			{
				TOG_BIT(flags, DRIVE);
				CLR_BIT(flags, DRIVE_MODE_LATCH);
			}

			/* 
			
			if (CHK_BIT(flags,DRIVE_MODE_LATC) && CHK_BIT(flags,DRIVE) )
			{
				TOG_BIT(flags,BOOST);
			}
			
			*/

			if (CHK_BIT(flags, TIP_MODE) && CHK_BIT(flags, DRIVE))
			{
				TOG_BIT(flags, BOOST);
			}
		}

		while (!(PINB & (1 << PB1)))
		{

			// Do nothing
			asm("nop");
		}
	}

	// Button release
	if ((PINB & (1 << PB1)) && (CHK_BIT(flags, ONECLICK_MODE_LATCH)))
	{
		CLR_BIT(flags, ONECLICK_MODE_LATCH);
	}

	if ((PINB & (1 << PB1)))
	{
		ATOMIC_BLOCK(ATOMIC_FORCEON)
		{
			CLR_BIT(ISR_flags, TIMER_2_LONG_PRESS);
		}
	}
}

void Double_Click_Protection()
{

	// Clear double click handling timer
	ATOMIC_BLOCK(ATOMIC_FORCEON)
	{
		timer_3_counter_double_click = 0;
		CLR_BIT(ISR_flags, TIMER_3_DOUBLE_CLICK);
	}
}

void Button_Handler()
{

	if (BUTTON_PRESSED)
	{
		_delay_ms(DEBOUNCE);

		if (!(PINB & (1 << PB1)) && (!CHK_BIT(flags, BUTTON_LATCH)))
		{

			SET_BIT(flags, BUTTON_LATCH);

			// **********************************************
			// Bypassed Mode Section
			// **********************************************

			if (!CHK_BIT(flags, DRIVE_MODE_LATCH))
			{

				CLR_BIT(flags, TIP_MODE);

				// **********************************************
				// Section "Click To Boost"
				// **********************************************

				// Default Mode and Click To Boost Mode is the same

				TOG_BIT(flags, DRIVE);

				//Launch double click protection
				Double_Click_Protection();

				// Clear "One Click" long press handling timer
				Clear_Timer_2_Long_Press();
			}

			// **********************************************
			// Drive Mode Section
			// **********************************************

			if (CHK_BIT(flags, DRIVE_MODE_LATCH))
			{

				SET_BIT(flags, TIP_MODE);

				// Clear "One Click" long press timer
				Clear_Timer_2_Long_Press();

				// Reset clicks counter if reached max
				if (click_counter == 2)
				{
					click_counter = 0;
				}

				// Lets start count clicks
				click_counter++;

				// If it's first click toggle a Drive bit
				if (click_counter == 1)
				{

					// Start count time for double click
					Clear_Timer_3_Double_Click();

					// **********************************************
					// Section "Click To Boost"
					// **********************************************

					// Default Mode
					if (!CHK_BIT(flags, CLICK_TO_BOOST))
					{
						TOG_BIT(flags, DRIVE);
					}

					// Click To Boost Mode
					if (CHK_BIT(flags, CLICK_TO_BOOST))
					{
						TOG_BIT(flags, BOOST);
					}
				}
			}
		}
	}

	// **********************************************
	// Bypassed Mode Section
	// **********************************************

	// Set Drive Mode latch if protection time is lost
	if ((CHK_BIT(flags, DRIVE)) && (CHK_BIT(ISR_flags, TIMER_3_DOUBLE_CLICK)))
	{
		SET_BIT(flags, DRIVE_MODE_LATCH);
	}

	// Button release
	if ((PINB & (1 << PB1)) && (CHK_BIT(flags, BUTTON_LATCH)))
	{
		CLR_BIT(flags, BUTTON_LATCH);
	}

	// **********************************************
	// Analyzer
	// **********************************************

	if (CHK_BIT(flags, DRIVE_MODE_LATCH))
	{

		if ((click_counter == 1) && (CHK_BIT(ISR_flags, TIMER_3_DOUBLE_CLICK)))
		{
			ATOMIC_BLOCK(ATOMIC_FORCEON)
			{
				CLR_BIT(ISR_flags, TIMER_3_DOUBLE_CLICK);
			}

			click_counter = 0;

			// Section "Click To Boost"
			if (!CHK_BIT(flags, CLICK_TO_BOOST))
			{
				CLR_BIT(flags, DRIVE_MODE_LATCH);
			}
		}

		if ((click_counter == 2) && (!CHK_BIT(ISR_flags, TIMER_3_DOUBLE_CLICK)))
		{

			// Got two clicks in time, so it's a double click action
			TOG_BIT(flags, BOOST);

			// Turn back relay
			TOG_BIT(flags, DRIVE);

			ATOMIC_BLOCK(ATOMIC_FORCEON)
			{
				CLR_BIT(ISR_flags, TIMER_3_DOUBLE_CLICK);
			}

			click_counter = 0;

			// Section "Click To Boost"

			if (CHK_BIT(flags, CLICK_TO_BOOST))
			{
				CLR_BIT(flags, DRIVE_MODE_LATCH);
			}
		}
	}

	// Analyzer End

	if (CHK_BIT(flags, DRIVE_MODE_LATCH))
	{
		// Keep double click timer bit clear
		if (click_counter == 0)
		{
			ATOMIC_BLOCK(ATOMIC_FORCEON)
			{
				CLR_BIT(ISR_flags, TIMER_3_DOUBLE_CLICK);
			}
		}
	}
}

int main(void)
{
	MCU_Setup();
	Bypass_Mode_Checker();
	sei();

	while (1)
	{
		Button_Handler();
		Flags_Handler();
		Long_Click_Button_Handler();
	}
}
