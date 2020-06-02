
#include <stdio.h>
#include <sys/mman.h>
#include <stdint.h>
#include <inttypes.h>
#include <fcntl.h>   /* File control definitions */
#include <time.h>
#include <stdlib.h>
#include <unistd.h> // solve th implicit declaration of sleep call

#define MAP_SIZE 4096UL   

#define MAP_MASK (MAP_SIZE - 1)

#define MEMDEV "/dev/mem" // map on memory device

#define BASEMEM 0x48040000 // map clock on base memory address

#define OFFSET 0x3C 

#define DMT  * (int32_t*) ( gpt_regs + 0x3C)


struct Qclock
	{
		uint32_t actualtimer; // var dedicated to actual time tick
		uint32_t partial1; // var dedicated to a partial time count given by a difference of elpsed time
		uint32_t partial2;
		uint32_t savedtimer1; // general purpoise
		uint32_t savedtimer2; // general purpoise 
		uint32_t savedtimer3;
		float tick_100ms;	//10 HZ  
		float tick_50ms;	//50 Hz 
		float tick_10ms;	//100 HZ  
		float tick_1ms; 	//1Khz
		int	 filedescriptor;

		unsigned long int dmpTimestamp; // dmp time stamp
		unsigned long int dmpsavedTimer1;//
		unsigned long int dmpsavedTimer2;//


	} Qclock;

int timerInitQ (); // initializie FD open /dev/mem 

void tickInit (struct Qclock* ); // tick settings

void printClock (struct Qclock*); // print struct values

void initClock (struct Qclock*); //initializie values to 0 no FD initialized

float timedifference_msec(struct timeval t0, struct timeval t1);
