#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <iostream>

#include "TA8428K.h"

TA8428K::TA8428K(mraa_result_t &response, int pwm_pin0, int pwm_pin1) {
	
	pwm0 = new mraa::Pwm(pwm_pin0);
	if (pwm0 == NULL) {
        	response = MRAA_ERROR_UNSPECIFIED;
		return;
    	}
	pwm0->enable(true);
	//pwm0->period_ms(10);

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
	pwm0->write(0);
	pwm0->enable(false);
	pwm1->write(0);
	pwm1->enable(false);

	delete pwm0;
	delete pwm1;
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
