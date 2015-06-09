#ifndef GP2Y0A21YK_H
#define GP2Y0A21YK_H

#include <stdio.h>
#include <mraa.hpp>




class GP2Y0A21YK
{
public:
	GP2Y0A21YK(mraa_result_t &response, int pin = 0, double r = 0.2);
	~GP2Y0A21YK();
	double getDistance();
	double getDistanceData();
	double voltage2distance(double dVolt);
	
private:
	mraa::Aio* a;
	double lastDistance;
	double _r;
};





#endif