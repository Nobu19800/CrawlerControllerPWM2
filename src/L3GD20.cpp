#include <stdio.h>
#include <unistd.h>
#include <sys/time.h>
#include <mraa.h>
#include <stdlib.h>
#include <algorithm>
#include <math.h>
#include <iostream>
#include "L3GD20.h"

L3GD20::L3GD20(mraa::I2c *i2c, i2c_smf *smf, uint8_t addr, double r) {
	_i2c = i2c;
	
	
	_r = r;
	
	
	
	_addr = addr;
	

	_smf = smf;

	reset();
}

L3GD20::~L3GD20() {
	//delete _smf;
}



void L3GD20::setScale(uint8_t scale)
{
	_scale = scale;
	writeByte(_addr, CTRL_REG4, _scale);
	
	
}




void L3GD20::reset(void) {
	
	
	
	writeByte(_addr,CTRL_REG1,0x0f);

	


	
	setScale(Range_250dps);
	

  	
  	
  	
 
  	
  	

	const double count = 10;
	double avx,avy,avz;
	lastX = 0;
	lastY = 0;
	lastZ = 0;
	
	for(int i=0;i < count;i++)
	{
		getGyroData(avx,avy,avz);

		lastX += avx/count;
		lastY += avy/count;
		lastZ += avz/count;
	}


	

	

	
}



void L3GD20::getGyro(double &avx, double &avy, double &avz)
{
	getGyroData(avx,avy,avz);

	lastX = _r*avx + (1-_r)*lastX;
	lastY = _r*avy + (1-_r)*lastY;
	lastZ = _r*avz + (1-_r)*lastZ;
	

}



void L3GD20::getGyroData(double &avx, double &avy, double &avz) {
	//uint8_t Buf[6];

	//readByte(_addr, OUT_X_L | (1 << 7), 6, Buf);

	_smf->sem_lock();


	_i2c->address(_addr);

	
	

	short GyroRaw_x = _i2c->readReg(OUT_X_H);
	GyroRaw_x = (GyroRaw_x << 8) | _i2c->readReg(OUT_X_L);
	short GyroRaw_y = _i2c->readReg(OUT_Y_H);
	GyroRaw_y = (GyroRaw_y << 8) | _i2c->readReg(OUT_Y_L);
	short GyroRaw_z = _i2c->readReg(OUT_Z_H);
	GyroRaw_z = (GyroRaw_z << 8) | _i2c->readReg(OUT_Z_L);

	_smf->sem_unlock();





	double gRes;
	if(_scale == Range_250dps)
		gRes = 2*0.00875*3.141592/180;
	else if(_scale == Range_500dps)
		gRes = 2*0.0175*3.141592/180;
	else
		gRes = 2*0.07*3.141592/180;

	avx = (double)GyroRaw_x * gRes;
	avy = (double)GyroRaw_y * gRes;
	avz = (double)GyroRaw_z * gRes;

	
	
}



void L3GD20::writeByte(uint8_t Address, uint8_t Register, uint8_t Data) {

	_smf->sem_lock();
	_i2c->address(Address);
	
	_i2c->writeReg(Register, Data);
	
	_smf->sem_unlock();

  	
}

void L3GD20::readByte(uint8_t Address, uint8_t Register, uint8_t Nbytes, uint8_t* Data) {
	
	_smf->sem_lock();

	_i2c->address(Address);
	_i2c->writeByte(Register);

	
	_i2c->read(Data,Nbytes);
	
	_smf->sem_unlock();
}