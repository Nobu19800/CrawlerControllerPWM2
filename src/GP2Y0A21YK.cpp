#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>

#include "GP2Y0A21YK.h"

GP2Y0A21YK::GP2Y0A21YK(mraa_result_t &response, int pin, double r) {
	
	a = new mraa::Aio(pin);
	if (a == NULL) {
        	response = MRAA_ERROR_UNSPECIFIED;
		return;
    	}
	response = MRAA_SUCCESS;

	lastDistance = 0;
	_r = r;
}

GP2Y0A21YK::~GP2Y0A21YK() {
	
	delete a;
}

double GP2Y0A21YK::getDistance() {
	float vol = a->readFloat();
	double distance = voltage2distance(vol*5)/100;

	lastDistance = _r*distance + (1-_r)*lastDistance;

	return lastDistance;
}


double GP2Y0A21YK::voltage2distance(double dVolt) {
  
  	double dDist;
  
  
	if(dVolt < 0.384321){
	    dDist = 999.0;                        /*out of range*/

	  }else if(dVolt < 0.430210){
	    dDist = dVolt * (-217.916667) + 163.750000;      /*80 - 70*/
	    
	  }else if (dVolt < 0.493308){
	    dDist = dVolt * (-158.484848) + 138.181818;    /*70 - 60*/
	      
	  }else if (dVolt < 0.590822){
	    dDist = dVolt * (-102.549020) + 110.588235;  /*60 - 50*/
	        
	  }else if (dVolt < 0.722753){
	    dDist = dVolt * (-75.797101) + 94.782609;  /*50 - 40*/
	          
	  }else if (dVolt < 0.894837){
	    dDist = dVolt * (-58.111111) + 82.000000;  /*40 - 30*/
	            
	  }else if (dVolt < 1.284895){
	    dDist = dVolt * (-25.637255) + 52.941176;  /*30 - 20*/
	              
	  }else if (dVolt < 1.623327){
	    dDist = dVolt * (-14.774011) + 38.983051;  /*20 - 15*/
	                
	  }else if (dVolt < 2.294455){
	    dDist = dVolt * (-7.450142) + 27.094017;  /*15 - 10*/
	                  
	  }else if (dVolt < 3.131931){
	    dDist = dVolt * (-5.970320) + 23.698630;  /*10 - 5*/
	  }

	  else{
	    dDist = 0.0;                              /*error*/
	}

	return dDist;
}