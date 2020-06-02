// hereby defined a structure for Q
// al the function to be working with quad should be defined here
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <curses.h>
#include <stdint.h>

#define SCALEUPDOWN 89 //scale variable from IPHONE
#define SCALELEFTRIGHT 89 
#define GAINUPDOWN 0.2
#define GAINLEFTRIGHT 0.2 //-89 +89 scaling factor
#define GAINYAW 1
#define GAINANGLE 10 //value is sent divided by 10
#define STEERING 36
#define MINPOWNOTSCALED 0.0
#define MAXPOWNOTSCALED 100.0
#define MINTOTPOWER 0.0
#define MAXTOTPOWER 100.0
#define MINPOWSCALED 5.1
#define MAXPOWESCALED 10.0
#define DIMBUFFER 20 // dimension of serial buffer to be written
#define PIDPOWER 30 //power percentage PIDs on
#define CONVERGENCEPOWER 50
#define PERCENT 0.01

#define MAXALT 220 // altitude in CM not to pass
#define SAFEALT 150 // altitude in CM to go is MAX is passed

struct Quad {
	
	//--------------------------	
	float power;
	float savedPowerState;
	float saveval;
	int updown;
	int updownSavedVal;
	int leftright;
	int leftrightSavedVal;
	int direction;
	int oldDirection;
	int directionSavedVal;
	float relativeDirection;
	bool armdisarm;
	bool powerCheck; // verify flag for power more than PIDPOWER
	bool newDirection;
	int IMUstatus;
	int IMUsavedVal;
	int counter;

	//px4flow
	float px4_comp_x;
	float px4_comp_y;
	float px4_alt;
	int px4_image_quality;
	uint64_t px4_timer;
	uint64_t px4_dtimer;
	float px4_pidX_out;
	float px4_pidY_out;
	float px4_pidAltout;
	float px4OutX;
	float px4OutY;
	float px4OutA;
	bool pxReady;
	int posMaintain;
	//--------------------------

	//baro altitude 
	float baroAlt;

	//---offsetts---------------
	float offsetUD;
	float savedOffsetStateUD;
	float offsetLR;
	float savedOffsetStateLR;
	float offsetDir;
	//--------------------------	

	// motor values------------
	float motorUL;
	float motorUR;
	float motorDL;
	float motorDR;
	// -------------------------
	bool enableUL;
	bool enableUR;
	bool enableDL;
	bool enableDR;
	//--------------------------

	// angles data
	float angleX;
	float angleY;
	float angleZ;

	// TRASMISSION BUFFERS
	char StringBuf[DIMBUFFER];
	char StringTestBuf1[DIMBUFFER];
	char StringTestBuf2[DIMBUFFER];
	//--------------------------

	// PIDs
	float kpp;
	float kdp;
	float kip;
	float kpr;
	float kdr;
	float kir;
	float kpy;
	float kdy;
	float kiy;
	//--------------------------
	// used in the trasmission setup 
	// testMode = 0 doesnt trasmit
	// testMode = 1 trasmit data for testing: Ks, angles errors
	// testMode = 2 trasmit data for running: Angles
	int testMode;
	bool diagnostic;
	bool KsSaveFlag;
	bool settingFlag;
	};


int init(struct Quad *);

int printval(struct Quad *);

int printval2 (struct Quad *);

int printKpidS (struct Quad *);

int resetval(struct Quad *);

int printAngles(struct Quad *);

void printQNCURSES (struct Quad *,  int , int, float);

int readKsFromFile (struct Quad *);

int readOffsetFromFile (struct Quad *);

int savePidtoFile (struct Quad *);

int saveOffsetToFile (struct Quad *quad);

float map(float , float , float , float , float );


