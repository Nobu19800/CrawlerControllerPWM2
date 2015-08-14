#include <stdio.h>
#include <unistd.h>
#include <sys/time.h>
#include <mraa.h>
#include <stdlib.h>
#include <algorithm>
#include <math.h>
#include <iostream>
#include "L3GD20_I2C.h"



L3GD20_I2C::L3GD20_I2C(mraa_result_t &response, mraa::I2c *i2c, i2c_smf *smf, uint8_t addr, uint8_t scale, double r) :
L3GD20(smf,addr,scale,r)
{
	_i2c = i2c;


	if(!sensor_Exist())
	{
		response = MRAA_ERROR_UNSPECIFIED;
		return;
	}

	reset();

	response = MRAA_SUCCESS;
}

L3GD20_I2C::~L3GD20_I2C() {
	//delete _smf;
}


void L3GD20_I2C::writeByte(uint8_t Address, uint8_t Register, uint8_t Data) {

	_smf->sem_lock();
	_i2c->address(Address);
	
	_i2c->writeReg(Register, Data);
	
	_smf->sem_unlock();

  	
}

void L3GD20_I2C::readByte(uint8_t Address, uint8_t Register, uint8_t Nbytes, uint8_t* Data) {
	
	_smf->sem_lock();

	_i2c->address(Address);
	_i2c->writeByte(Register);

	
	_i2c->read(Data,Nbytes);
	
	_smf->sem_unlock();
}