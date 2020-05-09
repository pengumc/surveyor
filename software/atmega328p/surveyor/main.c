/*
 * surveyor.c
 *
 * Created: 26 Apr 2020 17:30:32
 * Author : michiel
 */ 

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/wdt.h>

#include "smbus_slave.h"

// Max ADC clock is 200 kHz. We're running on 8 MHz
// So a minimum of 40 is needed.
// selected: 64 for a ADC clock of 125 kHz
#define ADC_PRESCALER ((1 << ADPS2) | (1 << ADPS1) | (0 << ADPS0))
#define ADC_COUNT (3)
#define PWM_COUNT_MSK (0x3F)

static uint8_t adc_mux;
static uint8_t comms_mem[10];
static uint8_t pwm_count;

void smbus_slave_command_hook(uint8_t cmd)
{
	PORTD &= ~(1<<PIND6);
	wdt_reset();
}

void smbus_slave_block_write_done()
{
	
}

int main(void)
{
	// SMBus setup
	comms_mem[0] = 0; // ADC 0 result L
	comms_mem[1] = 0; // ADC 0 result H
	comms_mem[2] = 0; // ADC 1 result L
	comms_mem[3] = 0; // ADC 1 result H
	comms_mem[4] = 0; // ADC 2 result L
	comms_mem[5] = 0; // ADC 2 result H
	comms_mem[6] = 0; // last received cmd
	comms_mem[7] = 0; // left speed
	comms_mem[8] = 0; // right speed
	smbus_slave_init(0x48, 56, 0b00, comms_mem, comms_mem); // baudrate 62.5 kHz
	
	// Motor control outputs
	DDRD = (1 << PIND2) | (1 << PIND3) | (1 << PIND4) | (1 << PIND5) | (1 << PIND6);

	// error led and watchdog
	PORTD = (1 << PIND6);
	wdt_enable(WDTO_500MS);
	// ADC setup
	// max clock is 200 kHz
	adc_mux = 0;
	ADMUX = (1 << REFS0) | adc_mux;
	ADCSRA = (1 << ADEN) | (1 << ADSC) | (0 << ADATE) | (1 << ADIE) | ADC_PRESCALER;
	ADCSRB = (1 << ADC0D) | (1 << ADC1D) | (1 << ADC2D);

	// pwm setup
	pwm_count = 0;
	TCCR0A = (1 << WGM01); // CTC mode
	TCCR0B = (0 << CS02) | (1 << CS01) | (1 << CS00); // clk / 64 = 125 kHz
	TIMSK0 = (1 << OCIE0A); // comp A interrupt
	OCR0A = 0x08;

	sei();
    while (1) 
    {
		// comms_mem[6]
		//	bit0 = right track backward_ena
		//  bit1 = left track backward_ena

		// right track
		if (comms_mem[6] & 0x01)
		{
			// forward
			if (comms_mem[7] > pwm_count)
			{
				PORTD &= ~(1 << PIND4);
				PORTD |= (1 << PIND5);
			}
			else
			{
				PORTD &= ~((1 << PIND4) | (1<< PIND5));
			}
		}
		else
		{
			// backward
			if (comms_mem[7] > pwm_count)
			{
				PORTD &= ~(1 << PIND5);
				PORTD |= (1 << PIND4);
			}
			else
			{
				PORTD &= ~((1 << PIND4) | (1<< PIND5));
			}
		}

		// left track
		if (comms_mem[6] & 0x02)
		{
			// forward
			if (comms_mem[8] > pwm_count)
			{
				PORTD &= ~(1 << PIND2);
				PORTD |= (1 << PIND3);
			}
			else
			{
				PORTD &= ~((1 << PIND2) | (1<< PIND3));
			}
		}
		else
		{
			// backward
			if (comms_mem[8] > pwm_count)
			{
				PORTD &= ~(1 << PIND3);
				PORTD |= (1 << PIND2);
			}
			else
			{
				PORTD &= ~((1 << PIND2) | (1<< PIND3));
			}
		}
    }
}

ISR(TIMER0_COMPA_vect)
{
	pwm_count = (pwm_count+1) & PWM_COUNT_MSK; // counting 0..63
}

ISR(ADC_vect)
{
	// store result
	comms_mem[adc_mux<<1] = ADCL;
	comms_mem[(adc_mux<<1)|1] = ADCH;
	if (++adc_mux >= ADC_COUNT)
	{
		adc_mux = 0;
	}
	ADMUX = (1 << REFS0) | adc_mux;
	ADCSRA |= (1 << ADSC);
}
