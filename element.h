/*file elemento.h
*/

#include <stdio.h>
#include <stdlib.h>
#include <limits.h>
#include <pthread.h>


/*dichiarazione dei prototipi di funzione*/
/*e progettazione struttura*/
#define DIAGTIME 5000

typedef struct set_element 
  
  {  
	long int seq;	
	float ReadanglePitch;
	float ReadangleRoll;
	float ReadangleYaw;
	float PP , PD , PI , RP , RD , RI , YP , YI , YD;
	float totCorrectionP , totCorrectionR , totCorrectionY;
	float SetanglePitch;
	float SetangleRoll;
	float SetangleYaw;
	float dT;
	struct set_element *next;
	struct set_element *first;
  }     dataSet;




dataSet* add_element (  dataSet *last, long int seq , float RaP , float RaR , float RaY , float SaP , float SaR , float SaY,  float
pitchP , float pitchI , float pitchD , float rollP , float rollI , float rollD ,float yawP , float yawI , float yawD,float totCP, float totCR, float totCY , float dT);

void print_set (dataSet *);

int writeFile (dataSet *);
void *writeFilePT (void *);

void pthreatWriteFile (dataSet *);


