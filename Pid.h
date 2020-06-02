
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <math.h>
#include <curses.h>

//#include <gsl/gsl_multifit.h>


#define COMPENSATIONFACTOR 0.01
#define SDT 1.0 // standart dT 
#define SIZEB 3
#define PI_Q 180
#define PI_QO 360

#define PGAIN 1.0
#define DGAIN 0.1
#define IGAIN 1.0

#define PGAINPX 1.0
#define DGAINPX 0.0
#define IGAINPX 1.0

#define ALPHAFACTOR 0.001
#define MINALPHA 0.002
#define MAXALPHA 0.998
#define LOWFREQ 5
#define BETAFILTER 0.09


#define MINOUTYAWLIMIT -15
#define MAXOUTYAWLIMIT 15
#define MINDERIVYAW 0.01
#define MINPROPYAW 0
#define MAXIYAW 10

#define MAXIPITCH 15.0
#define MINOUTPITCHLIMIT -15.0
#define MAXOUTPITCHLIMIT 15.0
#define MINPROPPITCH 0
#define MINDERIVPITCH 0.01
#define TIMELEAP 0.0015

#define MAXIROLL 15.0
#define MINOUTROLLLIMIT -15.0
#define MAXOUTROLLLIMIT 15.0
#define MINPROPROLL 0
#define MINDERIVROLL 0.01

// THIS IS APPLIED TO ROLL
#define MINPROPX 7
#define MAXIX 7
#define MINDERIVX 0
#define MINOUTXLIMIT -10 // total min out
#define MAXOUTXLIMIT 10 // total max out

// THIS IS APPLIED TO ROLL
#define MINPROPY 7 // max min prortional part
#define MAXIY 7 // max min integrative part
#define MINDERIVY 0
#define MINOUTYLIMIT -10 // total min out
#define MAXOUTYLIMIT 10 // total max out

// THIS IS APPLIED TO ALT
#define MINPROPALT 5.5 // max min proportional part
#define MAXIALT 8.0  // min max for Integrative part
#define MINDERIVALT 2.5
#define MINOUTALTLIMIT -15
#define MAXOUTALTLIMIT 15

#define constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))
#define PIV2 M_PI+M_PI
#define C360 360.000

struct pid
{
	float SetPoint; 	//what we want to achieve 
	float InputPoint;	//what we have
	float OutputSignal;	//what signal we need to give to achieve 
				//the result
	float Kp,Ki,Kd;		//Control factors p , i , d
	float errorP;		//Error proportional
	float errorI;		//Error Integrative
	float errorD;		//Error Derivative
	float lasterrorD;	//Last error D
	float lastError;
	int   counter;
	float lowfreqDt;
	float smoother;	//smootherlvl1
	float oldD;	
	// LIMITS
	float outMinLimit;	//minimum out limit
	float outMaxLimit;	//maximum out limit
	float Dcon;
	float Icon;
	float Pcon;
	
	bool  startPid;		//start output only if true
	bool  authKal;		//kalman filter flag TRUE kalman applied
	bool  authFilter;
	
	float delta; // F cut off for Deriv
	float deltaS; // F cut off for basic signal
	float buff[5]; // define buffering elements
	//convergence lock
	bool convLocking;
	float alphap; // alpha for convergence
	float betap;  // beta for convergence
	float lockedVal; // lockingval
	bool resetLocking;
	

}pid;



typedef struct {
  float q; //process noise covariance
  float r; //measurement noise covariance
  float x; //value
  float p; //estimation error covariance
  float k; //kalman gain
} kalman_state;


typedef struct {

	float sum;
	short int samples;
	float out;
	} pid_av;

void pidAV_init ( pid_av * , int samples);

void pid_a (pid_av* ma, float newdata);

void pidInitNULL (struct pid *); // init specified pit to NULL

void SetPIDlimitations (struct pid*  , float  , float  , float  , float  , float , bool , bool);

void pidPrint (struct pid* );

void pidPrintKs (struct pid*);

//input pid kp ki kd , timer , output the
//result in pid->feedBAck struct
float pid_calc( struct pid* , float); 

//float pid_calc (struct pid* pid , float dt , pid_av * av);
/*
@retun Set P-I-D parameters multiplied by P-I-D gain
*/
void setKpid (struct pid* , float  ,float  ,float,float,float,float ); //setnew Kpid values


void printComponents (struct pid*);

float difangdeg(float x, float y);


float angleFromOffset ( float , float ); // given an angle and an offset report the next angle 
			       // if i need to move te quad X degrees on the left/right 
			       // i need to know the new angle

void printPNCURSES (struct pid *, struct pid *,struct pid *, int , int);

kalman_state kalman_init(float q, float r, float p, float intial_value);

void kalman_update(kalman_state* state, float measurement);

void quickSort( float[], int, int);

float partition( float[], int, int);




