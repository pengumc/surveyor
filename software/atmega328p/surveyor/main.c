/*
 * surveyor.c
 *
 * Created: 26 Apr 2020 17:30:32
 * Author : michiel
 */ 

#include <avr/io.h>
#include <avr/interrupt.h>

#include "smbus_slave.h"

// Max ADC clock is 200 kHz. We're running on 8 MHz
// So a minimum of 40 is needed.
// selected: 64 for a ADC clock of 125 kHz
#define ADC_PRESCALER ((1 << ADPS2) | (1 << ADPS1) | (0 << ADPS0))
#define ADC_COUNT (3)

static uint8_t adc_mux;
static uint8_t comms_mem[10];

void smbus_slave_block_write_done()
{
	// hex commands:
	// track A forward/backward: 0x04 0x08
	// track B forward/backward: 0x10 0x20
	PORTD = comms_mem[6] & ((1 << PIND2) | (1 << PIND3) | (1 << PIND4) | (1 << PIND5));
	
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
	smbus_slave_init(0x48, 56, 0b00, comms_mem, comms_mem); // baudrate 62.5 kHz
	
	// Motor control outputs
	DDRD = (1 << PIND2) | (1 << PIND3) | (1 << PIND4) | (1 << PIND5);

	// ADC setup
	// max clock is 200 kHz
	adc_mux = 0;
	ADMUX = (1 << REFS0) | adc_mux;
	ADCSRA = (1 << ADEN) | (1 << ADSC) | (0 << ADATE) | (1 << ADIE) | ADC_PRESCALER;
	ADCSRB = (1 << ADC0D) | (1 << ADC1D) | (1 << ADC2D);
	sei();
    while (1) 
    {
    }
}

ISR(TIMER0_COMPA_vect)
{
	

}

ISR(ADC_vect)
{
	// store result
	comms_mem[adc_mux++] = ADCL;
	comms_mem[adc_mux++] = ADCH;
	if (adc_mux >= ADC_COUNT)
	{
		adc_mux = 0;
	}
	ADMUX = (1 << REFS0) | adc_mux;
	ADCSRA |= (1 << ADSC);
}
