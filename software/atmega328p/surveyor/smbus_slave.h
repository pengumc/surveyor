/*
 * smbus_slave.h
 *
 * Created: 27 Apr 2020 12:25:44
 *  Author: michiel
 */ 


#ifndef SMBUS_SLAVE_H_
#define SMBUS_SLAVE_H_

typedef struct SMBusSlave
{
	uint8_t data_count;
	uint8_t addr;
	uint8_t response;
} smbus_slave_t;

extern smbus_slave_t* global_smbus_slave;
void smbus_slave_init(uint8_t i2c_addr, uint8_t twbr, uint8_t prescaler);

// User implemented:
void smbus_slave_command_hook(uint8_t cmd);
void smbus_slave_block_write_done();




#endif /* SMBUS_SLAVE_H_ */