#ifndef L3GD20_I2C_H
#define L3GD20_I2C_H

#include <stdio.h>
#include <mraa.hpp>


#include "L3GD20.h"


 

class L3GD20_I2C : public L3GD20
{
public:
	L3GD20_I2C(mraa_result_t &response, mraa::I2c *i2c, i2c_smf *smf, uint8_t addr = 0x6a, uint8_t scale = 0, double r = 0.2);
	~L3GD20_I2C();
	
private:
	
	//int _i2channel;

	void readByte(uint8_t Address, uint8_t Register, uint8_t Nbytes, uint8_t* Data);
	void writeByte(uint8_t Address, uint8_t Register, uint8_t Data);

	
	mraa::I2c *_i2c;
	
};





#endif