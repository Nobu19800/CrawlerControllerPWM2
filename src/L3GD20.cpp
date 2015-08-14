#define _USE_MATH_DEFINES
#include <stdio.h>
#include <unistd.h>
#include <sys/time.h>
#include <mraa.h>
#include <stdlib.h>
#include <algorithm>
#include <math.h>
#include <iostream>
#include "L3GD20.h"

#define L3GD20_WHO_ID  0xD4

L3GD20::L3GD20(i2c_smf *smf, uint8_t addr, uint8_t scale, double r) {

	_r = r;
	
	_scale = scale;
	
	_addr = addr;
	

	_smf = smf;

}

L3GD20::~L3GD20() {
	//delete _smf;
}

bool L3GD20::sensor_Exist() {
	uint8_t Buf[2];
	readByte(_addr,WHO_AM_I,1,Buf);

	//std::cout << (int)Buf[0] << "\t" << (int)L3GD20_WHO_ID << std::endl;
	if(Buf[0] != L3GD20_WHO_ID)return false;
	
	return true;
}

mraa_result_t L3GD20::setAddr(uint8_t addr)
{
	if(_addr != addr)
	{
		_addr = addr;
		if(!sensor_Exist())return MRAA_ERROR_UNSPECIFIED;
		reset();
	}
	return MRAA_SUCCESS;
}

void L3GD20::setScale(uint8_t scale)
{
	if(_scale != scale)
	{
		_scale = scale;
		reset();
	}
}

void L3GD20::setCoefficient(double r)
{
	_r = r;
}


void L3GD20::setRange(uint8_t scale)
{
	_scale = scale;
	writeByte(_addr, CTRL_REG4, _scale);
	
	
}




void L3GD20::reset(void) {
	
	
	
	writeByte(_addr,CTRL_REG1,0x0f);

	


	
	setRange(_scale);
	

  	
  	
  	
 
  	
  	

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
		usleep(10000);
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



	
	uint8_t x_hi, x_lo, y_hi, y_lo, z_hi, z_lo;

	uint8_t Buf[2];
	readByte(_addr,OUT_X_H,1,Buf);
	x_hi = Buf[0];

	readByte(_addr,OUT_X_L,1,Buf);
	x_lo = Buf[0];

	readByte(_addr,OUT_Y_H,1,Buf);
	y_hi = Buf[0];

	readByte(_addr,OUT_Y_L,1,Buf);
	y_lo = Buf[0];

	readByte(_addr,OUT_Z_H,1,Buf);
	z_hi = Buf[0];

	readByte(_addr,OUT_Z_L,1,Buf);
	z_lo = Buf[0];

	short GyroRaw_x = x_hi;
	GyroRaw_x = (GyroRaw_x << 8) | x_lo;
	short GyroRaw_y = y_hi;
	GyroRaw_y = (GyroRaw_y << 8) | y_lo;
	short GyroRaw_z = z_hi;
	GyroRaw_z = (GyroRaw_z << 8) | z_lo;





	double gRes;
	if(_scale == Range_250dps)
		gRes = 2*0.00875*M_PI/180;
	else if(_scale == Range_500dps)
		gRes = 2*0.0175*M_PI/180;
	else
		gRes = 2*0.07*M_PI/180;

	avx = (double)GyroRaw_x * gRes;
	avy = (double)GyroRaw_y * gRes;
	avz = (double)GyroRaw_z * gRes;

	
	
}


void L3GD20::readByte(uint8_t Address, uint8_t Register, uint8_t Nbytes, uint8_t* Data)
{

}

void L3GD20::writeByte(uint8_t Address, uint8_t Register, uint8_t Data)
{

}