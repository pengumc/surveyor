/*
 * surveyor.c
 *
 * Created: 26 Apr 2020 17:30:32
 * Author : michiel
 */ 

#include <avr/io.h>
#include <avr/interrupt.h>

#include "smbus_slave.h"

void smbus_slave_block_write_done()
{
	PORTD ^= (1 << PIND3);
}

void smbus_slave_command_hook(uint8_t cmd)
{
	PORTD ^= (1 << PIND2);
}

int main(void)
{
	uint8_t comms_mem[10];
	smbus_slave_init(0x48, 56, 0b00, comms_mem, comms_mem); // baudrate 62.5 kHz
	
	TCCR0A = (1 << WGM01); // CTC mode
	TCCR0B = (1 << CS02) | (1 << CS00); // clk / 1024 (8e6 / 1024 = 7812.5 Hz)
	TIMSK0 = (1 << OCIE0A); // overflow interrupt
	OCR0A = 200; // 40 Hz
	// D2 and D3 as output
	DDRD = (1 << PIND2) | (1 << PIND3);
	sei();
    /* Replace with your application code */
    while (1) 
    {
    }
}

ISR(TIMER0_COMPA_vect)
{
	

}

