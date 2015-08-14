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
	mraa_result_t setPinNum(int pwm_pin0 = 3, int pwm_pin1 = 5);
private:
	mraa::Pwm* pwm0;
	mraa::Pwm* pwm1;
	int m_pin0;
	int m_pin1;
};





#endif