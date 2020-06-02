#include "clock.h"

//--------------------------------------------------------------------------
int timerInitQ ()

	{


	int  fd; //file descriptor
		

	
	if ( (fd=open("/dev/mem", O_RDWR|O_SYNC) ) < 0)
		{
			printf ("error opening /dev/mem\n");

			return 0;
		}

	return fd;

	
	};


//--------------------------------------------------------------------------
void tickInit (struct Qclock* clockQuad)

	{
		clockQuad->tick_100ms = 1900000.0f;	//10 Hz
		clockQuad->tick_50ms  =  950000.0f;	//50 Hz
		clockQuad->tick_10ms  =  190000.0f;	//100 Hz
		clockQuad->tick_1ms   =   19000.0f;	//1Khz
		printClock (clockQuad);
	}

//--------------------------------------------------------------------------
void printClock (struct Qclock* clockQuad)
		{

		printf ("\nsaved timer 1: %d\n", clockQuad->savedtimer1);
		printf ("saved timer 2: %d\n", clockQuad->savedtimer2);
		printf ("saved timer 3: %d\n", clockQuad->savedtimer3);

		printf ("value 100ms: %f\n", clockQuad->tick_100ms);
		printf ("value 10ms: %f\n", clockQuad->tick_10ms);
		printf ("value 1ms: %f\n", clockQuad->tick_1ms);
		printf ("timer /dev/ fd: %d\n\n", clockQuad->filedescriptor);


		}


//--------------------------------------------------------------------------
void initClock (struct Qclock* clockQuad)
	{
		
		clockQuad->actualtimer=0;
		clockQuad->savedtimer1=0;
		clockQuad->savedtimer2=0;
		clockQuad->savedtimer3=0;
		clockQuad->dmpTimestamp=0; // dmp time stamp
		clockQuad->dmpsavedTimer1=0;//
		clockQuad->dmpsavedTimer2=0;//		

	}

float timedifference_msec(struct timeval t0, struct timeval t1)
{
    return (t1.tv_sec - t0.tv_sec) * 1000.0f + (t1.tv_usec - t0.tv_usec) / 1000.0f;
}


