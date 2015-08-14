#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <iostream>

#include "TA8428K.h"

TA8428K::TA8428K(mraa_result_t &response, int pwm_pin0, int pwm_pin1) {
	m_pin0 = pwm_pin0;
	pwm0 = new mraa::Pwm(pwm_pin0);
	if (pwm0 == NULL) {
        	response = MRAA_ERROR_UNSPECIFIED;
		return;
    	}
	pwm0->enable(true);
	//pwm0->period_ms(10);
	m_pin1 = pwm_pin1;
	pwm1 = new mraa::Pwm(pwm_pin1);
	if (pwm1 == NULL) {
        	response = MRAA_ERROR_UNSPECIFIED;
		return;
    	}
	pwm1->enable(true);
	//pwm1->period_ms(10);

	response = MRAA_SUCCESS;

	
}

TA8428K::~TA8428K() {
	if(pwm0)
	{
		pwm0->write(0);
		pwm0->enable(false);
		delete pwm0;
	}
	if(pwm1)
	{
		pwm1->write(0);
		pwm1->enable(false);
		delete pwm1;
	}

	
	
}

mraa_result_t TA8428K::setPinNum(int pwm_pin0, int pwm_pin1)
{
	if(m_pin0 != pwm_pin0)
	{
		if(pwm0)
		{
			pwm0->write(0);
			pwm0->enable(false);
			delete pwm0;
		}
		pwm0 = new mraa::Pwm(pwm_pin0);
		if (pwm0 == NULL) {
	        	return MRAA_ERROR_UNSPECIFIED;
	    	}
		pwm0->enable(true);
	}
	m_pin0 = pwm_pin0;


	if(m_pin1 != pwm_pin1)
	{
		if(pwm1)
		{
			pwm1->write(0);
			pwm1->enable(false);
			delete pwm1;
		}
		pwm1 = new mraa::Pwm(pwm_pin1);
		if (pwm1 == NULL) {
	        	return MRAA_ERROR_UNSPECIFIED;
	    	}
		pwm1->enable(true);
	}
	m_pin1 = pwm_pin1;

	return MRAA_SUCCESS;
}


void TA8428K::setValue(double vol) {

	//std::cout << vol << std::endl;
	if(vol < 0)
	{
		pwm0->write(-vol);
		pwm1->write(0);
	}
	else
	{
		pwm0->write(0);
		pwm1->write(vol);
	}
}
