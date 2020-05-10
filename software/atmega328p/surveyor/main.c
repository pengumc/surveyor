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
#define ADC_COUNT (4)
#define ADC_SAMPLE_COUNT (64)
#define PWM_COUNT_MSK (0x3F)

static uint8_t comms_mem[20];
static uint8_t pwm_count;

typedef struct ADCData
{
	uint8_t sample_count;
	uint16_t sample_sum[ADC_COUNT];
	uint8_t admux[ADC_COUNT];
	uint8_t adc_index;
} adc_data_t;

static adc_data_t adc_data;

void smbus_slave_command_hook(uint8_t cmd)
{
	PORTD &= ~(1<<PIND6);
	wdt_reset();
}

void smbus_slave_block_write_done()
{
	
}

void adc_update()
{
	uint8_t result[2];
	result[0] = ADCL;
	result[1] = ADCH;
	adc_data.sample_sum[adc_data.adc_index] += *(uint16_t*)&result;
	if (++adc_data.adc_index >= ADC_COUNT)
	{
		adc_data.adc_index = 0;
		if (++adc_data.sample_count >= ADC_SAMPLE_COUNT)
		{
			adc_data.sample_count = 0;
			uint8_t i;
			register uint8_t v;
			uint8_t* addr = (uint8_t*)&adc_data.sample_sum[0];
			uint8_t checksum = 0xFF;
			for (i = 0; i < ADC_COUNT*2; ++i)
			{
				v = *addr;
				checksum += v;
				comms_mem[i] = v;
				*addr = 0;
				++addr;
			}
			comms_mem[ADC_COUNT*2] = checksum;
		}
	}
	ADMUX = (1 << REFS0) | adc_data.admux[adc_data.adc_index];
	ADCSRA |= (1 << ADIF) | (1 << ADSC);
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
	comms_mem[6] = 0; // temperature 3 L
	comms_mem[7] = 0; // temperature 3 H
	comms_mem[8] = 0; // analog checksum
	comms_mem[9] = 0; // track directions
	comms_mem[10] = 0; // right speed
	comms_mem[11] = 0; // left speed
	smbus_slave_init(0x48, 56, 0b00, comms_mem, comms_mem); // baudrate 62.5 kHz
	
	// Motor control outputs
	DDRD = (1 << PIND2) | (1 << PIND3) | (1 << PIND4) | (1 << PIND5) | (1 << PIND6);

	// error led and watchdog
	PORTD = (1 << PIND6);
	wdt_enable(WDTO_500MS);
	// ADC setup
	adc_data.adc_index = 0;
	adc_data.admux[0] = 0;
	adc_data.admux[1] = 1;
	adc_data.admux[2] = 2;
	adc_data.admux[3] = 0b1111; // temperature
	adc_data.sample_sum[0] = 0;
	adc_data.sample_sum[1] = 0;
	adc_data.sample_sum[2] = 0;
	adc_data.sample_sum[3] = 0;
	adc_data.sample_count = 0;
	ADMUX = (1 << REFS0) | adc_data.admux[0];
	ADCSRA = (1 << ADEN) | (1 << ADSC) | (0 << ADATE) | (0 << ADIE) | ADC_PRESCALER;
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
		//wdt_reset();

		if (ADCSRA & (1 << ADIF))
		{
			adc_update();
		}

		// right track
		if (comms_mem[9] & 0x01)
		{
			// forward
			if (comms_mem[10] > pwm_count)
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
			if (comms_mem[10] > pwm_count)
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
		if (comms_mem[9] & 0x02)
		{
			// forward
			if (comms_mem[11] > pwm_count)
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
			if (comms_mem[11] > pwm_count)
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

ISR(TIMER0_COMPA_vect, ISR_NOBLOCK)
{
	pwm_count = (pwm_count+1) & PWM_COUNT_MSK; // counting 0..63
}
