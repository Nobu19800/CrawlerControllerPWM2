#ifndef TA8428K_H
#define TA8428K_H

#include <stdio.h>
#include <mraa.hpp>




class TA8428K
{
public:
	TA8428K(mraa_result_t &response, int pwm_pin0 = 3, int pwm_pin1 = 5);
	~TA8428K();
	void setValue(double vol);
	
private:
	mraa::Pwm* pwm0;
	mraa::Pwm* pwm1;
};





#endif