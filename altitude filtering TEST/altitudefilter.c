#include "altitudefilter.h"

// computed altitude 
// loop can be inserted in the 1ms quaternions determination 
// result is given by alt->estimated_altitude and can be read 
// by altitude threading control looping

 void compute_altitude (struct alt_filter * alt,float dt, float Z_acc, float baroalt)
	{



	if (alt->inizialized == false ){

		alt->inizialized = true;
		alt->estimated_altitude = baroalt;
		alt->estimated_velocity = 0;
		alt->altitude_error_i = 0;
	printf("baor:%f alt %f init:%d to baro\n",baroalt, alt->estimated_altitude, alt->inizialized);

	}


        // Estimation Error
        alt->altitude_error = baroalt - alt->estimated_altitude;
        alt->altitude_error_i = alt->altitude_error_i + alt->altitude_error;
        // use contraits see later on 
	alt->altitude_error_i = min(25.0, max(-25.0, alt->altitude_error_i));
   
	alt->inst_acceleration = Z_acc * G_i*10 + alt->altitude_error_i *
	ALT_Ki;

	// Integrators
        alt->delta = (alt->inst_acceleration*dt) + (KP1*dt)*alt->altitude_error;

        alt->estimated_altitude = alt->estimated_altitude+
        (alt->estimated_velocity/5.0 + alt->delta)*(dt / 2) + (KP2 * dt) *
        alt->altitude_error;

        alt->estimated_velocity = alt->estimated_velocity + alt->delta*10.0; 
    
        // READ estimated_altitude from external 

	}
       
 

// struct inizializer 
void initializeAlt (struct alt_filter* alt)
	{
	
    	alt->estimated_altitude = 0.0;
	alt->altitude = 0.0;
	alt->inst_acceleration = 0.0;
	alt->compensated_acceleration = 0.0;
	alt->delta = 0.0;
	alt->inizialized = false;
	}

void printAlt (struct alt_filter* alt, float dt, float acc, float baro)
	{
	printf("t: %.3f acc %.3f Delt: %.3f ",dt,acc,alt->delta);		
	printf("A.Ins:%.2f ", alt->inst_acceleration);	
	printf("A.Est:%.2f ", alt->estimated_altitude);
	printf ("BA:%.2f ",baro);
	printf("A.EsV:%.2f ", alt->estimated_velocity);
	printf("A.Er:%.2f ", alt->altitude_error);
	printf("A.ErI:%.2f \n",alt->altitude_error_i );
	}




