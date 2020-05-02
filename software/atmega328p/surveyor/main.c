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
	
}

void smbus_slave_command_hook(uint8_t cmd)
{
	// hex commands: 
	// track A forward/backward: 0x04 0x08
	// track B forward/backward: 0x10 0x20
	PORTD = cmd & ((1 << PIND2) | (1 << PIND3) | (1 << PIND4) | (1 << PIND5));
}

int main(void)
{
	uint8_t comms_mem[10];
	smbus_slave_init(0x48, 56, 0b00, comms_mem, comms_mem); // baudrate 62.5 kHz
	
	// D2 and D3 as output
	DDRD = (1 << PIND2) | (1 << PIND3) | (1 << PIND4) | (1 << PIND5);
	sei();
    while (1) 
    {
    }
}

ISR(TIMER0_COMPA_vect)
{
	

}

