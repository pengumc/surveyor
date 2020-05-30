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

// Max ADC clock is 200 kHz. We're running on 16 MHz
// So a minimum of 80 is needed.
// selected: 128 for a ADC clock of 125 kHz
#define ADC_PRESCALER ((1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0))
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
	comms_mem[12] = 90; // servo control
	comms_mem[13] = 0; // control register. bit0 = HC-SR04 trigger
	comms_mem[14] = 0; // last ICP1L
	comms_mem[15] = 0; // last ICP1H
	smbus_slave_init(0x48, 120, 0b00, comms_mem, comms_mem); // baudrate 62.5 kHz
	
	// 2..5 = motor control
	// 6 = led
	// 7 = servo control 
	
	DDRD = (1 << PIND2) | (1 << PIND3) | (1 << PIND4) | (1 << PIND5) | (1 << PIND6) | (1 << PIND7);

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

	// Motor PWM setup
	pwm_count = 0;
	TCCR0A = (1 << WGM01); // CTC mode
	TCCR0B = (0 << CS02) | (1 << CS01) | (1 << CS00); // clk / 64 = 125 kHz
	TIMSK0 = (1 << OCIE0A); // comp A interrupt
	OCR0A = 0x08;

	// servo control timer setup
	// run timer with clk div 1024 till overflow, set B1 high
	// then switch to CTC mode and clk div 256 so we can control up time 
	// with OCR2A. On compare match, clear B1 and reset to clk div 1024 and overflow
	DDRB |= (1 << PINB1);
	TCCR2A = 0;
	OCR2A = comms_mem[12];
	TCCR2B = (1 << CS22) | (1 << CS21) | (1 << CS20);
	TIMSK2 = (1 << TOIE2);

	// Ultrasonic timer setup
	// timer 1. input capture on PB0
	// clk div 8. gives us 32 ms max
	TCCR1A = 0;
	TCCR1B = (1 << ICES1) | (0 << CS12) | (1 << CS11) | (0 << CS10);
	
	DDRB &= ~(0<< PINB0);
	TIMSK1 = (1 << ICIE1);

	sei();
    while (1) 
    {
		
		wdt_reset();

		if (comms_mem[13] & 0x01)
		{
			PORTD |= (1 << PIND7);
		}
		else
		{
			PORTD &= ~(1 << PIND7);
		}
		
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

ISR(TIMER2_OVF_vect)
{
	PORTB |= (1 << PINB1);
	TCCR2B = 0; // turn off timer
	OCR2A = comms_mem[12];
	TCCR2A = (1 << WGM21);
	TIFR2 = 0xFF;
	TIMSK2 = (1 << OCIE2A);
	TCCR2B = (1 << CS22) | (1 << CS21) | (0 << CS20);
}

ISR(TIMER2_COMPA_vect)
{
	PORTB &= ~(1 << PINB1);
	TIMSK2 = (1 << TOIE2);
	TCCR2A = 0;
	TCCR2B = (1 << CS22) | (1 << CS21) | (1 << CS20);
}

ISR(TIMER1_CAPT_vect)
{
	if (TCCR1B & (1 <<  ICES1))
	{
		// captured rising edge of echo response
		TCNT1 = 0;
		TCCR1B &= ~(1 << ICES1);
	}
	else
	{
		// falling edge capture
		*((uint16_t*)&comms_mem[14]) = ICR1;
		TCCR1B |= (1 << ICES1);
	}
}

ISR(TIMER1_OVF_vect)
{
	// timer overflown. reset to rising edge capture
	TCCR1B |= (1 << ICES1);
}