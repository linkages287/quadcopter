
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <unistd.h>
#include <stdint.h>

#define max(a,b) (((a) > (b)) ? (a) : (b))
#define min(a,b) (((a) < (b)) ? (a) : (b))
#define ALT_Ki 0.0005
#define KP1 0.65f //PI observer velocity gain 
#define KP2 1.0f // PI observer position gain
#define G_i 9.80665f


struct alt_filter
	{
	float estimated_altitude;
	float altitude;
	float altitude_error;
	float altitude_error_i;
	float inst_acceleration;
	float compensated_acceleration;
	float estimated_velocity;
	float delta;
	bool  inizialized;
	} alt_filter;


void compute_altitude (struct alt_filter * ,float , float , float );

// struct inizializer 
void initializeAlt (struct alt_filter*,float,float);

//print structure
void printAlt (struct alt_filter*,float,float,float );


