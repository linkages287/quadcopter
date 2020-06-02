#include <stdlib.h>  /* Standard lib definitions*/
#include <stdio.h>   /* Standard input/output definitions */
#include <stdint.h>
#include <string.h>  /* String function definitions */
#include <unistd.h>  /* UNIX standard function definitions */
#include <fcntl.h>   /* File control definitions */
#include <errno.h>   /* Error number definitions */
#include <termios.h> /* POSIX terminal control definitions */
#include <stdbool.h> /* std bool for boolean result */
#include <ctype.h>   /* is alpha or number */
#include <pthread.h> /* pthread library */
#include <prussdrv.h> /*pruss driver */
#include <pruss_intc_mapping.h> /* pruss memory management */
#include <linux/i2c-dev.h> /*i2c drivers*/
#include <math.h> /*math library abs , sqrt ...*/
#include <time.h> /*time library get time from the system*/
#include <signal.h> /*handle signals */
#include <sys/time.h> // time of the day functions
#include <sys/resource.h>  // process priority
#include "toserial.h" /*serial connection*/
#include "clock.h" /*rt clock lib*/
#include "Pid.h" /*pid handler functions*/
#include "element.h" /*add dignostic support*/
#include "imufilters.h" /*filters to be use for imu AHRS*/
#include "imuLib.h" /*read write i2c imu REGS and settings*/
#include "prulib.h" /*pru and pwm library*/
#include "PXtoserial.h" /*PXflow serial write read*/
#include "altitudeKF.h" /*altitude fusion filters*/

#define DEBUG 0 // set debug to 0 or to 10 for NCURSES 
#define CALIB 0 // calibration load variables
// ----------------------------------------------------------------------------
// global variables------------------------------------------------------------
struct Quad quad; // contain Quad information and status
struct Qclock qclock; // clock 
struct pid pidYaw; // pid yaw
struct pid pidPitch; // pid pitch
struct pid pidRoll; // pid roll

//-----------------------------------------------------------------------------
// general globals
int fd; // file descriptor BLE
int fd1; // file descriptor /dev/mem for timer mapping purpoise
int fd2; // file descriptor PXFLOW
int fd3; // file descriptor ALTITUDE

// ----------------------------------------------------------------------------
// FUNCTIONS DECLARATION
// ----------------------------------------------------------------------------

//------MPU loop process variables---------------------------------------------
void IMU_loop (int dt); // looping IMU func to get angles
float pitchBuff[5];
float rollBuff[5];
float yawBuff[5];
int buffc = 0;
//-----------------------------------------------------------------------------

//------PID loop process-------------------------------------------------------
void PID_loop (struct pid* , struct pid* , struct pid* , float dT);
// ----------------------------------------------------------------------------

//------PWM--------------------------------------------------------------------
void PWM_loop(struct pid* , struct pid* , struct pid* );
unsigned int percents[4]; // number of motors
int savedVal[4]; //
//-----------------------------------------------------------------------------

//short unsigned int *rate;

//------THREADING---FUNC-------------------------------------------------------
void *threadFuncBLERX(void *arg); //define thread function serial RX
void *threadFuncBLETX(void *arg); //define thread function serial TX
void *threadALT_POS(void *arg); // define thread read PX4 data and PID
pthread_mutex_t lock; // mutex lock definition 


//-----------------------------------------------------------------------------
// IMU SETTLED VARIABLES
//-----------------------------------------------------------------------------
int i2c_fd = I2C_BUS2; // i2c fd var
unsigned char buff[MAX_WRITE_LEN + 1];
quaternion_t q; // qauternioni con G
attitude_t v; // valore di quota
calibration c; // valori di calibrazione
attQ  att; // valori di attitude pich roll yaw
int counter = 5; // filtering counter
//raw Data
float acc[3]; // raw acc
float gyro[3]; // raw gyro
float mag[3]; // raw mag
float noG_Zacc; // linear acc
mov_av gyrox , gyroy , gyroz; // gyro averaging
mov_av accx , accy ,accz; // acc averaging
mov_av z_acc; // Z moving average
pid_av dp , dr , dy;
//altitude structure container 
struct alt_filter alt;
//signal handler CONTROL-C exit procedure
static volatile int keepRunning = 1;

// signal handler for control C
void intHandler(int signalNum) {

	printf ("Intercepted Interrupt SIGNAL, exiting\n");
    	
	keepRunning = 0;
}

// ----------------------------------------------------------------------------
// -------------------------- main program-------------------------------------
//-----------------------------------------------------------------------------
int main (void) 
	{
	
	int errorFD = 0; // error files closure variable
	// timer variables -- volatile need to be declared in the same scope
	unsigned char* gpt_regs; // timer base register
	int datacounter = 0;
	float dt = 0.0f;
	register uint32_t partial;
	initQuaterion (q , 1.0f , 0.0f , 0.0f, 0.0f); // quaterion initializ
	
	//-------define mpu variables filterings ---------------------------	
	ma_init(&gyrox, FLOOP);
	ma_init(&gyroy, FLOOP);
	ma_init(&gyroz, FLOOP);

	ma_init(&accx, FLOOP);
	ma_init(&accy, FLOOP);
	ma_init(&accz, FLOOP);

	ma_init(&z_acc, FLOOP); // Z acc MA

	// load calibration data from file to be applied to IMU sensor
	if(CALIB){loadCalibration (&c);}
	//-----------------------------------------------------------------
	int row,col;	// to store the number of rows and 
			// the number of colums of the screen
	//-----------------------------------------------------------------
	// DECLARE and INITIALIZE PIDs
	//-----------------------------------------------------------------
	//struct pid pidPitch, pidYaw, pidRoll;
	pidInitNULL (&pidPitch);
	pidInitNULL (&pidYaw);
	pidInitNULL (&pidRoll);
	//-----------------------------------------------------------------

	//------PTHREAD----------------------------------------------------
	pthread_t pthBLE;	// thread identifiers
	pthread_t pthPX4;	// pthPx4 thread for reading
	//-----------------------------------------------------------------
	// DECLARE and INITIALIZE QUAD
	//-----------------------------------------------------------------
	
	//init quad variables to 0
	init (&quad);
	initAltStruct(&alt, 0.1f , 0.1f);
	
	// read Ks from external file and assign to quad-------------------
	readKsFromFile (&quad);
	// pass pids Ks initial value
	setKpid (&pidPitch, quad.kpp, quad.kip, quad.kdp, PGAIN, IGAIN, DGAIN);
	setKpid (&pidRoll, quad.kpr, quad.kir, quad.kdr, PGAIN, IGAIN, DGAIN);
	setKpid (&pidYaw, quad.kpy, quad.kiy, quad.kdy, PGAIN, IGAIN, DGAIN);

	// set pid limitations and filters auth. defined in Pid.h
	//struct pid* myPID , float Pcon , float Icon , float Dcon , float
	//MinOut , float MaxOut , bool filtering , bool lockingAuth
	SetPIDlimitations (&pidPitch, MINPROPPITCH , MAXIPITCH , MINDERIVPITCH,
	MINOUTPITCHLIMIT , MAXOUTPITCHLIMIT , true , true);

	SetPIDlimitations (&pidRoll , MINPROPROLL , MAXIROLL , MINDERIVROLL ,
	MINOUTROLLLIMIT , MAXOUTROLLLIMIT , true ,  true);

	SetPIDlimitations (&pidYaw , MINPROPYAW , MAXIROLL , MINDERIVYAW ,
	MINOUTYAWLIMIT , MAXOUTYAWLIMIT , true , true);
	
	//----------------------------------------------------------------------
	//read offsets to ZEROLEVEL platform files contain pitch and roll level
	//settings
	readOffsetFromFile(&quad);

	// other program variables
	//DIAGNOSTIC PURPOISE
	dataSet *diagnose = NULL;
	dataSet *savedDiagnose = NULL;

	// Ncurses --initialize IF DEBUG IN PROGRESS----------------------------
	if (DEBUG == 10)
		{
		 	 
		WINDOW * mainwin;

		 if ( (mainwin = initscr()) == NULL ) 
		{
			printf("Error initialising ncurses.\n");
			return 0;
		}
			initscr(); // Initialize the window
			getmaxyx(stdscr,row,col); //get the number of rows and columns 
			noecho(); // Don't echo any keypresses
			curs_set(FALSE); // Don't display a cursor
    		}
	
	// DECLARE and INITIALIZE CLOCK DEVICE
	//----------------------------------------------------------------------
	qclock.filedescriptor = timerInitQ(); // file descr. timer device
	// mapping on /dev/mem on a file descriptor
	gpt_regs = (unsigned char*) mmap(0, MAP_SIZE,PROT_READ|PROT_WRITE,
	MAP_SHARED, qclock.filedescriptor, BASEMEM & ~MAP_MASK);
	// test timer for tick definitions, device read from 24Mhz clock
	initClock (&qclock); 
	tickInit (&qclock); // initialize clock with 24mhz values RTC
	//---------------------------------------------------------------------

	//----PXFLOW OPEN/INIT-------------------------------------------------
	fd2 = serial_open(PX4DEVICE); 
	if (configure_px4flow(fd2) == false) {keepRunning=false; 
	printf ("PX4FLOW config error\n");}
	//---------------------------------------------------------------------	

	//----BLUETOOTH INIT --------------------------------------------------
	fd = connectSerial (DEVICE); //  connect to the serial device
	
	//----ALTITUDE INIT ---------------------------------------------------
	fd3 = connectSerial (ALTITUDE); //  connect to the serial device
	//SETTING VALUES: K1,K2,K3,KALMANFLAG,MILLISEC CYCLE
	if (initAltimeter(fd3, 0.085,1,0.015,1,12)>0)
	{printf("Barometer initialized\n");} else
	{
	printf ("Baro error: exiting program\n");	
	keepRunning=false;}

	//----MPU INIT---------------------------------------------------------
	i2c_fd = i2c_open(); // open user space device
	imuInit(i2c_fd); // initialize imu with choosen parameters

	// starting signal sent to iphone to start controlling
	if (bleAliveSignal(fd)<0) {printf ("BEALIVE SIGNAL TXRX error\n");}
	
	//----------------------------------------------------------------------
	//----PWM INIT--- LOAD DUMP AND INITIALIZE PRU WITH PWM
	PWM_init(); // load and initialize PWM
	//--------------------------------------void PX4_loop (struct Quad *quad)------------------------
		

	//---pthread TXRX create--------------------------------------------------------------------------
	pthread_create(&pthBLE,NULL,threadFuncBLERX,"Start processing... BLE RX thread"); //  thread serial RX
	pthread_create(&pthBLE,NULL,threadFuncBLETX,"Start processing... BLE TX thread"); // thread serial TX
	pthread_create(&pthPX4,NULL,threadALT_POS,"Start processing... PX4 flow thread"); // thread serial RX PX4
	pthread_mutex_init (&lock,NULL);// define a lock for multithreading safety
	//----------------------------------------------------------------------

	//while (quad.baroAlt<=0){printf("wait baro data\n");usleep(700);};
	//signal handler catch if CTRL-C
	signal(SIGINT, intHandler);

	printf ("-> STARTING MAIN LOOP <-\n");

	while (keepRunning) // start main looping for pid calc	
		{
		//---------------------- main PID algorithm -------------------------------------
		qclock.actualtimer = DMT; // save actual timer in a global variable
		// calculate partial timer elpsed at the beginning of each cycle
		partial = abs(qclock.savedtimer1 - qclock.actualtimer);

		if ((partial) >= qclock.tick_1ms) // 1 KHZ CYCLE

			{
				
			pthread_mutex_lock(&lock); //--------------LOCK

			dt = ((float)partial / (float)(qclock.tick_1ms))/1000; // value should be around 0.001
				
			IMU_loop(dt); // get imu data acc,gyr,mag

			//DEBUG
			//printf ("x:%.2f y:%.2f z:%.2f dt:%.5f \n",quad.angleX,
			//quad.angleY,quad.angleZ,dt);
				
			// PID -> compute output , it will be mapped to PWM to engines
			PID_loop (&pidPitch,&pidRoll,&pidYaw ,dt);

			// PWM UPDATE motors computer with pid output
			PWM_loop(&pidPitch,&pidRoll, &pidYaw);

			pthread_mutex_unlock(&lock); //-------------UNLOCK
				
			//----------------------------------------------------------
			// diagnostic and others -accessories-----------------------
			// NB usage of NCURSES will incresase looping time
			//----------------------------------------------------------
			//print NCURSES RESULT
			//warning NCURSE WILL SLOW DOWN ACQUIRING DATA 
			//ANGLES WILL BE NOT RELIABLE
			if (DEBUG == 10) 
				{

				printQNCURSES (&quad ,row, col, dt);
					
				printPNCURSES (&pidPitch,&pidRoll,&pidYaw,row,col);

				refresh();				
					
				}
				// write information data on a file RECORD 5 SECONDS
				// FOR TESTING PURPOUSE
				if (quad.diagnostic == true)
					{
						//printf ("enter diag %d\n", datacounter);						
						if (quad.diagnostic == true && datacounter == 0)
						{
							// reset diagnose for new reading
						diagnose = NULL;
						diagnose = (dataSet *)malloc (sizeof(dataSet)); // define 1 element
						savedDiagnose = diagnose;	
						}
						
						// 5000 * 1 ms = 5 sec data recording
						if (datacounter <=DIAGTIME) 
						{
						datacounter++;						
						// end of data save 
						diagnose = add_element(diagnose
						, datacounter , quad.angleX ,
						quad.angleY , pidYaw.InputPoint ,
						quad.updown , quad.leftright ,
						quad.direction , pidPitch.errorP
						, pidPitch.errorI ,
						pidPitch.errorD , pidRoll.errorP
						, pidRoll.errorI ,
						pidRoll.errorD , pidYaw.errorP ,
						pidYaw.errorI , pidYaw.errorD ,
						pidPitch.OutputSignal ,
						pidRoll.OutputSignal ,
						pidYaw.OutputSignal , dt); 

						} 

						if (datacounter>DIAGTIME)
						{//pthread save start
						pthreatWriteFile (savedDiagnose); 
						datacounter= 0; 
						quad.diagnostic = false;
						}
					}// end diganostic routine
				//----------------------------------------------------------
				//----------------------------------------------------------

				//save the timer as last loop step				
				qclock.savedtimer1 = DMT; // save new timer before looping
				
			}
			sched_yield();	// pass to another thread 
	}// end looping

	// contrl - C catched-------------------------------------------------- 
	// CLOSING UP----------------------------------------------------------
	pthread_join(pthBLE, NULL);
	pthread_join(pthPX4, NULL);
	PWM_close();
	//close all mapped devices
	
	errorFD = close(fd); 
	errorFD += close(fd1);
	errorFD += close(fd2);
	errorFD += close(fd3);
	printf ("program terminated, file descriptors errors: %d\n",errorFD);
	return 0;//program terminated normal exit
	}//end of main

//-----------------------------------------------------------------------------
//--------------END MAIN-------------------------------------------------------
//-----------------------------------------------------------------------------


//-----------------------------------------------------------------------------
// ----------------------------------------------------------------------------
// ---- pthread functions: POSITION PX4 and serial trasmission
//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------


void *threadALT_POS(void *arg)
	{

	struct timeval t1, t2 , t3 , t4; // timer for thread
	float elapsedTimePX4 = 0.0f; // PX4 timer elapsed 65-66 ms
	float elapsedTimeBaro = 0.0f; // baro timer elapsed 10 ms
	bool negAlt = false; // negative altitude 
	
	quad.baroAlt = 0.0f; // barometric altitude in main quad
	
	float baroBuff = 0.0f; // barometric altitude buff
	char buff ='K'; // buffer for read from altitude
	bool altFlag = true; // altitude regulation when alt control is active
	int fd3RX= 0; // serial reading output
	bool startreading = false; // reading control flag

	//px4 dt
	float dt = 0.0f;

	//-------------------------------
	float newBaroAlt = 0.0f; // new data for delta
	float newPX4Alt = 0.0f; // new data for delta
	//-------------------------------
	// pid definitions	
	struct pid Xpitch; // pid X lateral
	struct pid Yroll; // pid Y lateral
	struct pid Alt; // pid Altitude
	
	
	// pid init 0
	pidInitNULL (&Xpitch); // inizialize X pid
	pidInitNULL (&Yroll); // inizialize Y pid
	pidInitNULL (&Alt); // initialize altitude pid

	
	//-----------------------------------------------------------------
	// pid INITIALIZE PXFLOW X,Y,ALT PID
	setKpid (&Xpitch , 5.33 , 0.23  , 0, PGAINPX , IGAINPX , DGAINPX);
	setKpid (&Yroll ,  5.33 , 0.23 , 0, PGAINPX , IGAINPX , DGAINPX);
	setKpid (&Alt , 5.0 , 0.00175 , 0.7,   PGAINPX , IGAINPX , DGAINPX);

	//SAMPLE for pid initialization
	//SetPIDlimitations (&pidPitch, MINPROPPITCH , MAXIPITCH , MINDERIVPITCH,
	//MINOUTPITCHLIMIT , MAXOUTPITCHLIMIT , true , true)
	// SET PIDS LIMITATIONS false: derivative true:pid funcioning
	// in the case no filter and no derivative is consifered in the process
	SetPIDlimitations (&Xpitch, MINPROPX , MAXIX , MINDERIVX,
	MINOUTXLIMIT , MAXOUTXLIMIT , false , false);

	SetPIDlimitations (&Yroll , MINPROPY , MAXIY , MINDERIVY ,
	MINOUTYLIMIT , MAXOUTYLIMIT , false , false);

	SetPIDlimitations (&Alt , MINPROPALT, MAXIALT, MINDERIVALT ,
	MINOUTALTLIMIT , MAXOUTALTLIMIT , true , true);

	// for testing purposes set to true
	// those need to be set sia remote
	Xpitch.startPid = true;
	Yroll.startPid = true;
	Alt.startPid = true;
	
	// setpointS to have XY no movement
	// set point for alitutde need to be done by USER REMOTE
	Xpitch.SetPoint = 0;
	Yroll.SetPoint = 0;

	// reset all timer to current time
	gettimeofday(&t1, NULL); // start timer
	gettimeofday(&t2, NULL); // start timer
	gettimeofday(&t3, NULL); // start timer
	gettimeofday(&t4, NULL); // start timer

// ---------------------------------------------------------------------
// ---------------------------------------------------------------------
//THREAD SPINNING run stop on control-C
// ---------------------------------------------------------------------
// ---------------------------------------------------------------------
while(keepRunning) {// keepRunning false on control C

	gettimeofday(&t1, NULL); // start timer 
	elapsedTimePX4 = timedifference_msec(t2, t1); // dt math Px4
	elapsedTimeBaro = timedifference_msec(t3, t1); // dt math baro
	
//----------------------------------------------------------------------
//PX4 DATA PARSER 
// if new data present apply PD loop	
//----------------------------------------------------------------------
if (elapsedTimePX4 > REFRESH_PX4 ) // check if REFRESH_PX4 ms are ELAPSED 
				   // REFRESH_PX4 defined in PXtoserial.h
{ 

  // READ ALL DATA FROM PX4 POSITION AND ALTITUDE
  if (readDataPX4 (fd2, &quad)) 
  {

	gettimeofday(&t2, NULL); // save timer t2	

//----------------------------------------------------
//DEFINE NEW ALTITUDE FOR OLDING IF 01 - 11
//----------------------------------------------------
if ((altFlag == true) && ((quad.posMaintain == 1) || (quad.posMaintain== 11))) 
	{
	altFlag = false; // SWITCH FLAG
	newBaroAlt = quad.baroAlt; // NEW BARO ALT
	newPX4Alt = quad.px4_alt*100; // NEW PX4 ALT
	}
// TURN OFF ALTITUDE AND POSITION HOLDING
if (quad.posMaintain == 0 && altFlag == false) 
	{	
	//FLAG IS RESETTED ONLY IF BOTH ALT AND POS ARE OFF 
	altFlag = true; // reset flag to 0 for next update		
	} 

	
  } // end PX4 READING LOOP ROUTINE

	// POSITION DETERMINATION PID FROM PX4
	//POSITION DETERMINATION DOES NOT REQUIRE A START POINT
	if(quad.posMaintain == 10 || quad.posMaintain == 11 )
	{
	//internal timer is more precise for PID math 
	dt = (float)(quad.px4_dtimer)/65000; // proper Dt calc from Px4 reading
	Xpitch.InputPoint =  quad.px4_comp_x; // X PID input
	Yroll.InputPoint = quad.px4_comp_y; // Y PID input 
	quad.px4OutX = pid_calc (&Xpitch, dt); //pitch fix	
	quad.px4OutY = pid_calc (&Yroll, dt); // roll fix
  	}

} // end REFRESH_PX4 ms delay time


// CHECK BARO SERIAL EACH REFRESH_ATL MS FOR NEW DATA
// TIMER NEED TO BE LESS THAN BARO SET UP TIMER
// REFRESH_ATL is defined in toserial.h
if (elapsedTimeBaro>=REFRESH_ATL) {
	
	// read barometric altitude
	// READ serial 1 BYTE AT A TIME 
	// this is faster than SONAR ALT
	fd3RX = read (fd3, &buff, 1);
//------------------------------------------------------------------------------
//  BARO DATA PARSER
//------------------------------------------------------------------------------
if (fd3RX>0){ // start data evaluation if buffering

	gettimeofday(&t3, NULL); // save timer t3
	//printf ("BARO dt %f\n",elapsedTimeBaro);
	 //start serial reading alt data
	if (buff=='a' && startreading == false)  {startreading = true;}

	while(startreading){// finish reading stream untill data ready

	fd3RX = read (fd3, &buff, 1);

	if (fd3RX>0){

	if  (buff=='-'){ negAlt=true;}
		
	if (isdigit(buff) && startreading == true) 
	{
		baroBuff=atoi(&buff)+10*baroBuff;
	}

	// last section of reading is about 12 ms cycle
	//
	if (buff=='b' && startreading == true) 
		{
		startreading = false; // reading end character

		if(negAlt){ // check negative altitude
		quad.baroAlt = -baroBuff; // altitude in cm
		} else quad.baroAlt = baroBuff; // altitude in cm
		
		baroBuff = 0.0; // reset baro buff for next reafing
		negAlt = false;

		Altitude_KF_update(&alt,quad.baroAlt);
		//printf ("baro timer: %f\n", elapsedTimeBaro);
		} 
		   } // end IF serial reading 	
	} // END WHILE PARSING DATA 				
}// END BARO DATA PARSER 



//----------------------------------------------------
// altitude and position are false
// DATA BYPASSED
// dont apply correction if not engaged
//----------------------------------------------------
if (quad.posMaintain == 0) 
	{		
	//FLAG IS RESETTED ONLY IF BOTH ALT AND POS ARE OFF 
	altFlag = true; // reset flag to 0 for next update			

	// reset integratives errors
	Alt.InputPoint = 0;
	Alt.errorI = 0;
	Xpitch.errorI = 0;
	Yroll.errorI = 0;
	// reset quad output PID
	quad.px4OutX = 0; // pid X
	quad.px4OutY = 0; // pid Y
	quad.px4OutA = 0; // pid altitude 	
	} 

// ----------------------------------------------------
// this is HOLD altitude setting only
// position need to be filtered and set to 0
//-----------------------------------------------------
if (quad.posMaintain == 1)
	{
	//reset XY components value 
	Xpitch.errorI = 0; // reset integrative section
	Yroll.errorI = 0;
	quad.px4OutX = 0; // reset out PID values
	quad.px4OutY = 0;
	}

}// end infinite looping for thread 

// this is HOLD POSITION settings only
// altitude need to be filtered and set to 0
if (quad.posMaintain == 10)
	{			
	Alt.errorI = 0; // erase integrative
	quad.px4OutA = 0; // PID out 0
	}



// HOLDING ALTITUDE DETERMINATION
// IF FLAG IS FALSE THEN HOLDING ALT IS ACQUIRED
if ((altFlag == false) && ((quad.posMaintain == 1) || (quad.posMaintain== 11)))
{
Alt.InputPoint = evaluateAltitude (newBaroAlt , alt.h ,newPX4Alt,quad.px4_alt*100);


quad.px4OutA = pid_calc (&Alt, elapsedTimeBaro); 


//DEBUG
printf ("alt.input : %.2f Alt_pid_out: %.2f\n" , Alt.InputPoint,quad.px4OutA);

} 

}// end timer baro reading 




	//----------------------------------------------------------------------
	//-------EXITING -------------------------------------------------------
	//----------------------------------------------------------------------
	printf ("EXITing Pthread PX4\n");
	return NULL;
} // END THREAD ALTIMETER AND POSITIONING SYSTEM


void *threadFuncBLERX(void *arg)
	{
	// thread visibility variables
	// define control values-------------------------
	int updown = 0;
	int leftright = 0;
	float power = 0;	
	int direction = 0;
	bool armdisarm = false;
	int selector = 0;
	bool EndSerial = false;
	bool startreading = false; // running loop
	bool readnumber = false;
	bool readletter = false;
	char buff ='K';
	
	// define settings vars (variables used for settings purpoise)
	bool EndTesting = false;
	bool starttesting = false; // testing loop
	int enableMotorUL = 0; // motors enable flags
	int enableMotorUR = 0;
	int enableMotorDL = 0;
	int enableMotorDR = 0;
	int holdingPos = 0; //position holding flag
	float offSUD = 0.0; // offset values
	float offSLR = 0.0;
	float kpp = 0.0; //Ks values
	float kdp = 0.0;
	float kip = 0.0;
	float kpr = 0.0;
	float kdr = 0.0;
	float kir = 0.0;
	float kpy = 0.0;
	float kdy = 0.0;
	float kiy = 0.0;
	// ---------------------------------------
	
	while (keepRunning)
	{
	
	// save new Ks values when requested
	if (quad.KsSaveFlag == true)
			{
			//save val
			savePidtoFile(&quad);
			saveOffsetToFile(&quad);
			//reset flag
			quad.KsSaveFlag = false;
			}
	//----------------------------------------------------------------------
		
	//----------------------------------------------------------------------
	//--------------------- START COMMAND SECTION --------------------------
	//----------------------------------------------------------------------
	if(EndSerial==true){// if end serial string reset all data
			    // and pass all retrieved data to Quad struct

		// trasmission is set in testmode == 1
		quad.testMode = 2; // trasmission from BBB will STOP 

		quad.power = power;
		quad.updown = -updown + SCALEUPDOWN; // value given by joystick ios Qmodel.h
		quad.leftright = -leftright + SCALELEFTRIGHT; // value given by joystick ios

		quad.direction = direction;// pass Q direction

		// if direction is new recalculate pid
		if (quad.direction != quad.oldDirection)
		{
		  quad.newDirection = true; // NEW DIRECTION FLAG
		}

		quad.oldDirection = quad.direction; // NEW DIRECTION DATA

		quad.armdisarm = armdisarm; // ARM DISARM ROTORS FLAG

		quad.posMaintain = holdingPos; // HOLD POSITION OR ALTITUDE FLAG

		if (quad.armdisarm == 1) {
						quad.enableUL = 1;
						quad.enableUR = 1;
						quad.enableDR = 1;
						quad.enableDL = 1;	
					} else {
						quad.enableUL = 0;
						quad.enableUR = 0;
						quad.enableDR = 0;
						quad.enableDL = 0;
						quad.power = 0;
						}
		//--------------------------------------------------------------
		// reset all variables after string parsing	
		buff ='K'; // set the buff to not existing command
		updown = 0.0;
	 	leftright = 0.0;
	 	power = 0.0;
	 	direction = 0.0;
	 	armdisarm = false;
 	 	selector = 0;
	 	EndSerial = false;
	 	startreading = false;
	 	readnumber = false;
	 	readletter = false;
		holdingPos = 0;
		// -----------------------
	}

	// end testing
	if (EndTesting==true){

		// set SETTINGS mode 
		// from that point the machine will start to send 
		// pitch roll and yaw data via BLE
		quad.testMode = 1; 
		//-------------------------------------
		
		// ENDTESTING SETTINGS UPDATE CICLE
		// CALLED ONLY AT THEN END OF THE TRASMISSION COMMAND
		quad.power = power;
		quad.kpp = kpp;
		quad.kdp = kdp;
		quad.kip = kip;
		quad.kpr = kpr;
		quad.kdr = kdr;
		quad.kir = kir;
		quad.kpy = kpy;
		quad.kdy = kdy;
		quad.kiy = kiy;	
		quad.posMaintain = holdingPos;	
		//------------------------------------------------------

		//setting new Ks value read from BLE-----------------
		//reverse the multiply process in TX from iphone by 100
		setKpid (&pidPitch , quad.kpp ,quad.kip ,quad.kdp, PGAIN/100 ,
		IGAIN/100, DGAIN/100);
		setKpid (&pidRoll , quad.kpr ,quad.kir ,quad.kdr, PGAIN/100 ,
		IGAIN/100 , DGAIN/100);
		setKpid (&pidYaw , quad.kpy ,quad.kiy ,quad.kdy, PGAIN/100 ,
		IGAIN/100 , DGAIN/100);
		//---------------------------------------------------

		// ENABLE-DISABLE MOTORS flags 
		//-------------------------------------
		quad.enableUL = enableMotorUL;
		quad.enableUR = enableMotorUR;
		quad.enableDL = enableMotorDL;
		quad.enableDR = enableMotorDR;

		// offset settings different to 0 angle
		// accelerometer need to be in a flat surface
		// when calibrtithat value.
		// check min max values on iphone screen to set offset values
		quad.offsetUD = (offSUD/10)-6; // scaling value avoid negative tx
		quad.offsetLR = (offSLR/10)-6; // scaling value avoid negative tx

		// check armed if one of the motors is enabled
		if ((quad.enableUL) || (quad.enableUR) || (quad.enableDL) || (quad.enableDR))
		{
		
		quad.armdisarm = 1;

		} else { quad.armdisarm = 0;}

		//Reset variables for the next read
		enableMotorUL = 0;
		enableMotorUR = 0;
		enableMotorDL = 0;
		enableMotorDR = 0;
		kpp = 0;
		kdp = 0;
		kip = 0;
		kpr = 0;
		kdr = 0;
		kir = 0;
		kpy = 0;
		kdy = 0;
		kiy = 0;
		power = 0;
		offSUD = 0.0;
		offSLR = 0.0;
		buff ='K';
		holdingPos = 0;
		// flow selector variables
		selector = 0;
		starttesting = false;
		EndTesting = false;
		}
	
	//RED 1 BYTE AT A TIME 
	int fdRX = read (fd, &buff, 1);
	
	// read from file descriptor if there are some data in the 
	// buffer then do the parsing
	if (fdRX >0)
	{		
	// -------------------------------------------- 
	pthread_mutex_lock(&lock); //--------------LOCK
	// --------------------------------------------
	if (buff == '*')
		{
		 
		printf ("Program terminated system rebooting\n");
		system ("reboot");
		return 0;
		}
	// is digit or letter?		
	if (isdigit(buff))	{ 
				readletter = false;
				readnumber = true;
				}

			else 	{
				readnumber = false;
				readletter = true;
				}

	//if letter assign the selector
	if (readletter==true)
				{
				// running section
				
				if (buff =='A') 

				{
				startreading = true;					
				selector = 1;
				}

				if (buff == 'B') { selector = 2;}//power 

				if (buff =='C') { selector = 3;}//UPDOWN

				if (buff =='D') { selector = 4; }//LEFTRIGHT

				if (buff =='E') { selector = 5;}//motor safe

				if (buff =='F') {selector = 100;}//hold position
				
				if (buff =='Z') 
					{ //end of serial 
				EndSerial = true; 
  				startreading = false; }

				

				// end running section


				//------------------------------------
				//------------------------------------
				// testing section
				//------------------------------------
				//------------------------------------

				if (buff =='a')
					{
					starttesting = true;
					selector = 6;
					}
				
				if (buff =='b') {selector = 7;}

				if (buff =='c') {selector = 8;}

				if (buff =='d') {selector = 9;}

				if (buff =='e') {selector = 10;}

				if (buff =='f') {selector = 11;}

				if (buff =='g') {selector = 12;}
				
				if (buff =='h') {selector = 13;}
				
				if (buff =='i') {selector = 14;}
				
				if (buff =='l') {selector = 15;}
				
				if (buff =='m') {selector = 16;}
				
				if (buff =='n') {selector = 17;}
				
				if (buff =='o') {selector = 18;}
				
				if (buff =='p') {selector = 19;}

				if (buff =='s') {selector = 20;}

				if (buff =='r') {selector = 21;}
			
				// diagnostic prevent start and end reading
				// read just a letter
				if (buff =='q') {quad.diagnostic = true;}
				
				//save new Ks
				if (buff =='t') { 
						//save Ks settings command
						quad.KsSaveFlag = true;
						} 
				if (buff =='u')
						{
						//load Ks settings command
						quad.settingFlag = true;
						}				

				if (buff =='v')
						{
						//positions maintain flag on
						selector = 22;
				
						}	

				if (buff =='z') 
						{
						starttesting=false;
						EndTesting=true;		 
						}

				//------------------------------------
				// end testing section
				//------------------------------------

				}// end readletter
	
		//--------------------------------------------
		// reading numbers : running session
		//--------------------------------------------
		// enter number reading section if a number is received 
		// slector is not 0 and a reading started.
		if (startreading == true && readnumber == true && selector !=0)

		{
			// readnumber
		if (selector==1){//read power
		power = atoi(&buff)+10*power;			
				}
		if (selector==2){//read updown
		updown =atoi(&buff)+10*updown;
				}
		if (selector==3){
		leftright=atoi(&buff)+10*leftright;						
				}
		if (selector==4){// read direction
		direction =atoi(&buff)+10*direction;
				}
		if (selector==5){// read disarm buttom
		armdisarm =atoi(&buff);
				}
		if (selector==100)//holding position
				{
		holdingPos= atoi(&buff)+10*holdingPos;
				}

		}
		//--------------------------------------------
		// reading numbers : settings session 
		//--------------------------------------------
		// enter number reading section if a number is received 
		// slector is not 0 and a reading started.
		if (starttesting == true && readnumber == true && selector !=0)

		{
			
			if (selector==6){ // motor UL -> 1
				
					enableMotorUL = atoi(&buff);
					
					}

			if (selector==7){// motoro UR -> 2

					enableMotorUR = atoi(&buff);

					}

			if (selector==8){ // motor DL -> 3

					enableMotorDL = atoi(&buff);

					}

			if (selector==9){ // motor DR -> 4

					enableMotorDR = atoi(&buff);
					
					}
			
			if (selector==10){// set kpp

					kpp = atoi(&buff) + 10*kpp; 				
					
					}
			
			if (selector==11){// set kdp

					kdp = atoi(&buff) + 10*kdp;
					
					}

			if (selector==12){

					kip = atoi(&buff) + 10*kip;

					}
			
			if (selector==13){

					kpr = atoi(&buff) + 10*kpr;

					}

			if (selector==14){

					kdr = atoi(&buff) + 10*kdr;

					}

			if (selector==15){

					kir = atoi(&buff) + 10*kir;
			
					}
			
			if (selector==16){

					kpy = atoi(&buff) + 10*kpy;
					
					}
		
			if (selector==17){

					kdy = atoi(&buff) + 10*kdy;
						
					}
			
			if (selector==18){

					kiy = atoi(&buff) + 10*kiy;

					}

			if (selector==19){

					power = atoi(&buff) + 10*power;

					}

			if (selector==20){//offset UD

					offSUD = atoi(&buff) + 10*offSUD;

					}

			if (selector==21){//offset LR

					offSLR = atoi(&buff) + 10*offSLR;

					}

			if (selector==22)//holding position
					{
					holdingPos= atoi(&buff)+10*holdingPos;
					}
			
			
		} // end  testing parsing
		 // --------------------------------------------
	 	 pthread_mutex_unlock(&lock); //------------UNLOCK
		 // --------------------------------------------

	} //end if read fd	
		else {sched_yield();} // resched

	}//end while reading cycle
	//exiting procedure
	printf ("Exit Pthread BLE RX\n");
	return NULL;

	}// end of thread function


//-----------------------------------------------------------------------------
//----------TX over BLE -------------------------------------------------------
//-----------------------------------------------------------------------------
void *threadFuncBLETX(void *arg)
	{

	//parzializer bool flags
	bool scale1 = false;
	bool scale2 = false;
	bool scale3 = false;
	quad.StringBuf[0] = '\0';
	
	//infinite loop for receving data
	while(keepRunning){

	qclock.partial1 = abs(qclock.savedtimer2 - qclock.actualtimer);//check partial timing
	
	// 100 ms timer send data 
	if ((qclock.partial1)>= qclock.tick_100ms) // 20Hz cycle
	{
	// -------------------------------------------- 
	pthread_mutex_lock(&lock); //--------------LOCK
	// --------------------------------------------

	//pass saved setting values to set-up page on ios
	//values are passed scaled each 100 ms 
	//max payload trasmission for ble is 20byte
	//so the total payload need to be spread into 
	//different packets
	if (scale3)
		{
		//write Kpitch
		snprintf(quad.StringBuf, DIMBUFFER , "I,%.0f,%.1f,%.1f,U\n",
		quad.power, quad.offsetUD, quad.offsetLR);
		//WRITE to serial FD
		if ((write (fd,quad.StringBuf , DIMBUFFER)) <0)
			{
			printf ("Serial Write BLE failed\n");
			return 0;			
			}
		scale3 = false;
		int tc = tcdrain(fd); // wait for data to be sent
		tcflush(fd,TCOFLUSH); // flush channel
		if (tc<0){printf ("TC_Drain error\n");}


		}

	if (scale2)
		{
		//write Kyaw
		snprintf(quad.StringBuf, DIMBUFFER , "H,%.2f,%.3f,%.2f,T\n",
		quad.kpy, quad.kiy, quad.kdy);
		//WRITE to serial FD
		if ((write (fd,quad.StringBuf , DIMBUFFER)) <0)
			{
			printf ("Serial Write BLE failed\n");
			return 0;			
			}
		scale2 = false;
		scale3 = true; // activate the next scale
		int tc = tcdrain(fd); // wait for data to be sent
		tcflush(fd,TCOFLUSH); // flush channel
		if (tc<0){printf ("TC_Drain error\n");}
		}

	if (scale1)
		{
		//write Kroll
		snprintf(quad.StringBuf, DIMBUFFER , "G,%.2f,%.3f,%.2f,S\n",
		quad.kpr, quad.kir, quad.kdr);
		//WRITE to serial FD
		if ((write (fd,quad.StringBuf , DIMBUFFER)) <0)
			{
			printf ("Serial Write BLE failed\n");
			return 0;			
			}
		scale1 = false;
		scale2 = true; // activate the next scale
		int tc = tcdrain(fd); // wait for data to be sent
		tcflush(fd,TCOFLUSH); // flush channel
		if (tc<0){printf ("TC_Drain error\n");}
		}

	if (quad.settingFlag) // activate by controller via BLE
		{
		printf ("Reloading Ks settings from file\n");

		// read Ks from external file and assign to quad-------------------
		readKsFromFile (&quad);
		// pass pids Ks initial value
		setKpid (&pidPitch , quad.kpp  , quad.kip  , quad.kdp, PGAIN , IGAIN , DGAIN);
		setKpid (&pidRoll ,  quad.kpr  , quad.kir  , quad.kdr, PGAIN , IGAIN , DGAIN);
		setKpid (&pidYaw ,  quad.kpy  , quad.kiy  , quad.kdy, PGAIN , IGAIN , DGAIN);

		printf ("sending Ks setting to IOS\n");
		//write Kpitch
		
		snprintf(quad.StringBuf, DIMBUFFER , "F,%.2f,%.3f,%.2f,R\n",
		quad.kpp, quad.kip, quad.kdp);
		//WRITE to serial FD
		if ((write (fd,quad.StringBuf , DIMBUFFER)) <0)
			{
			printf ("Serial Write BLE failed\n");
			return 0;			
			}
		quad.settingFlag = false; // reset flag reload settings 
		scale1 = true; // send other pack scaled by DT
		int tc = tcdrain(fd); // wait for data to be sent
		tcflush(fd,TCOFLUSH); // flush channel
		if (tc<0){printf ("TC_Drain error\n");}
		}
	

	// Quad in normal working state will send angular data to IOS
	// data are sent only if in test mode 2
	if (quad.testMode == 2){
	//check if anyone of the tramission values is different from the previous
	if (( (int)(quad.angleX)!=savedVal[0] ) || ((int)(quad.angleY)
	!=savedVal[1]) || ((int)(quad.angleZ) !=savedVal[2]))
		{

		quad.StringBuf[0] = '\0';//ending value to be added
		//save XYZ values, i want to to trasmit bbbvalues 
		//only if they are different
		savedVal[0] = (int)(quad.angleX);
		savedVal[1] = (int)(quad.angleY);
		savedVal[2] = (int)(quad.angleZ);

		// example for field quad.StringBuf = "X,-100,-200,-300,Q";
		// X,x-val,y-val,z-val,Q
		// parsing string to be trasmitted
		snprintf(quad.StringBuf, DIMBUFFER , "X,%d,%d,%d,Q\n",

		(int)(quad.angleX),(int)(quad.angleY),(int)(quad.angleZ));
		
		//WRITE to serial FD for trasmission
		if ((write (fd,quad.StringBuf , DIMBUFFER-1)) <0)
			{
			printf ("Serial Write BLE failed\n");
			return 0;			
			}
	}

	

	qclock.savedtimer2 = qclock.actualtimer; // save actual timer

	int tc = tcdrain(fd); // wait for data to be sent
	tcflush(fd,TCOFLUSH); // flush channel-*
	if (tc<0){printf ("TC_Drain error\n");}

	if (DEBUG == 4) // DEBUG STRING level 2
		{
		printf ("string : %s size: %d timer: %f tcdrain: %d \n", quad.StringBuf,
		sizeof(quad.StringBuf), ((float)qclock.partial1/(float)qclock.tick_100ms), tc);
		}

	} // end IF

	// ------------------------------------------------- 
	pthread_mutex_unlock(&lock); //--------------UNLOCK
	// -------------------------------------------------	

	} // end 10Hz check
	
	sched_yield(); // pass control to anothre thread
	
	} //end while loop

	printf ("Exit Pthread BLE TX\n");
	return NULL;

	} // end thread 
			

//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------
void PWM_loop (struct pid* pidp, struct pid* pidr, struct pid* pidy) 
	{

	//if  TODO write a start control process if just posted at the end
	// EXAMPLE CHECK CONNECTED, LANDED TRASMITTING
	
	//if armdisarm == true exec 
	if (quad.armdisarm)
	{

	// APPLY PID TODO YAW AND SCALE 
	// MIN and MAX are defined in Qmodel.h
	if (quad.enableUL){//check motor enabled
	quad.motorUL = map(constrain(quad.power + pidp->OutputSignal +
	pidr->OutputSignal - pidy->OutputSignal +
	quad.px4OutA,MINTOTPOWER,MAXTOTPOWER),MINPOWNOTSCALED , MAXPOWNOTSCALED
	, MINPOWSCALED , MAXPOWESCALED);
	} else {quad.motorUL = MINPOWSCALED;}

	if (quad.enableUR){//check motor enabled	
	quad.motorUR = map(constrain(quad.power + pidp->OutputSignal -
	pidr->OutputSignal + pidy->OutputSignal + quad.px4OutA,MINTOTPOWER,MAXTOTPOWER),
	MINPOWNOTSCALED , MAXPOWNOTSCALED , MINPOWSCALED , MAXPOWESCALED);
	} else {quad.motorUR = MINPOWSCALED;}

	if (quad.enableDL){//check motor enabled
	quad.motorDL = map(constrain(quad.power - pidp->OutputSignal +
	pidr->OutputSignal + pidy->OutputSignal + quad.px4OutA,MINTOTPOWER,MAXTOTPOWER),
	MINPOWNOTSCALED , MAXPOWNOTSCALED , MINPOWSCALED , MAXPOWESCALED);
	} else {quad.motorDL = MINPOWSCALED;}

	if (quad.enableDR){ // check motor enabled
	quad.motorDR = map(constrain(quad.power - pidp->OutputSignal -
	pidr->OutputSignal - pidy->OutputSignal + quad.px4OutA,MINTOTPOWER,MAXTOTPOWER),
	MINPOWNOTSCALED , MAXPOWNOTSCALED , MINPOWSCALED , MAXPOWESCALED);
	} else {quad.motorDR = MINPOWSCALED;}
	} else
		{ // if armdisarm flag is sibaled SET all motors to minimum speed
		  // this is a safety check
			quad.motorUL = MINPOWSCALED;
			quad.motorUR = MINPOWSCALED;
			quad.motorDL = MINPOWSCALED;
			quad.motorDR = MINPOWSCALED;
		}

	
	//SCALE OUTPUT IN PWM FREQUENCY RANGE 
	//assign a freq to each motor
	int dcUL = (int) (quad.motorUL * BASE_FREQ * PERCENT);
	int dcDL = (int) (quad.motorDL * BASE_FREQ * PERCENT);

	int dcUR = (int) (quad.motorUR * BASE_FREQ * PERCENT);

	int dcDR = (int) (quad.motorDR * BASE_FREQ * PERCENT);

	//APPLY VALUES TO THE ENGINES ARRAY
	percents[0]= dcUL; // P9_31 -->>> go to motorUL

	percents[1]= dcDL; // P9_27 -->>> go to motorDL

	percents[2]= dcUR; // P8_12 -->>> go to motorUR

	percents[3]= dcDR; // P8_11 -->>> go to motorDR
	
//DEBUG
//printf ("UL %d DL %d UR %d DR %d OUTALTSENS %f\n",dcUL, dcDL, dcUR, dcDR , quad.px4OutA);
	//WRITE VALUES TO PRUSS to generate PWM AND HAVE ROTATIONS 
	prussdrv_pru_write_memory(PRUSS0_PRU0_DATARAM, 0, percents, 16);
	
	

	}


//-----------------------------------------------------------------------------
// ----------------------------------------------------------------------------
// ------------------------- function IMU DATA LOOPING ------------------------
//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------

void IMU_loop (int dt){

		// read acc and gyro and apply calibration
		get_acc_gyro (acc, gyro, i2c_fd, buff, c);
		
		// moving average filter acc from vibes
		filterAcc (acc, &accx , &accy, &accz);

		// read mag and apply calibration
		get_mag (mag, i2c_fd , buff , c);

		// FUSION FILTER ALL DATA ACC GYRO MAG CONVERTED TO QUADRIONS
		// put results in Q
		MadgwickAHRSupdate(gyro[0]*DEGREE_TO_RAD, gyro[1]*DEGREE_TO_RAD,
		gyro[2]*DEGREE_TO_RAD, acc[0], acc[1], acc[2], mag[0], mag[1],
		mag[2], dt , q );

		//computer compensated linear acceleration
   		noG_Zacc= compute_compensated_acc(q, acc);

		//apply contraints no Z linear acceleration
		//if (abs(noG_Zacc)<ZACC_MIN) noG_Zacc=0.0;
		
		//debug
		//printf("Z: %.2f\n",noG_Zacc);

		//filter noG_ZAcc
		moving_average(&z_acc,noG_Zacc);// result in z_acc.out 

		Altitude_KF_propagate(&alt,-z_acc.out,dt);

		//COMPUTER PITCH ROLL YAW FROM QUATERNIONS AND WRITE RESULTS IN
		//&att
		computePRY ( q , &att);


			quad.angleX = -att.pitch ; 
			quad.angleY = att.roll ;
			quad.angleZ = att.yaw ;
			

	// CONVERT YAW FROM 0 TO 360
	// normalize angle from 0 to 360 
	if (quad.angleZ < 0) quad.angleZ = 360 + quad.angleZ; 
 

	//debug
	//printf("dt: %.3f %.2f %.2f
	//%.2f\n",dt,quad.angleX,quad.angleY,quad.angleZ);
	}


void PID_loop (struct pid* pidP, struct pid* pidR, struct pid* pidY , float dT)

	{
	//printf ("DT %.4f\n",dT);
	//----------------------------------------------------------------------
	// SET POINT________pid_calc___OUTPUT-->CORRECT MOTORs
	// INPUT POINT__________|		 	  |
	//   |____________________________________________|			

	//SET POINT-------------------------------------------------------------
	// GAIN IS DECLARED IN Qmodel.h: respect to the joystic values
	// give a real angle for the offset

	// es. GAINUPDOWN = 6 GENERATE A 48/6 = 8 DEG max angle correction
	// quad.updown and quad.leftright are JUST CORRECTED WITH 24 OFFSET 
	//TO AVOID NEGATIVE VALUE DURING TRASMISSION IN BLE
	// set point is the joysticj vaue corrected by the 
	// inclination offset from an incorrect acc mounting


	//apply PX4 corrections if required
	//if (quad.posMaintain == false) 
	//{
	//	quad.px4_comp_x = 0;		
	//	quad.px4_comp_y = 0;
	//}

	// SETPOINT = READ VALUE + OFFSET VALUE + PX4corrections
	pidP->SetPoint = quad.updown*GAINUPDOWN + quad.offsetUD + quad.px4OutX;

	pidR->SetPoint = quad.leftright*GAINLEFTRIGHT + quad.offsetLR - quad.px4OutY;
	
	//printf ("SetPointP %f  SetPointR %f  \n",pidP->SetPoint,pidR->SetPoint);
	//----------------------------------------------------------------------

	//PID INPUT     --------------------------------------------------------
	pidP->InputPoint = quad.angleX ;

	pidR->InputPoint = quad.angleY ;

	//printf ("angle_readX : %.2f , angle_readY : %.2f\n", quad.angleX , quad.angleY);
	
	
	// pids need to start in a power relation  TO TAKEOFF
	// for example if power is more than 30% than start pid
	// this value need to be verified during the testing phase
	// PIDPOWER is declared in Qmodel.h
	// this cycle is performed ONE TIME
	// PIDPOWER is set in Qmodel.h
	if ((quad.power >= PIDPOWER) && (quad.powerCheck == false)) // TODO add check power flag to loop 1 time 
	{
	pidY->startPid = true; 
	pidP->startPid = true; 
	pidR->startPid = true;
	
	quad.powerCheck = true; // quad power flag 
	quad.newDirection = true; // set take off direction 
	
	// reset pidYaw
	//quad.directionSavedVal = 0;
	//pidY->InputPoint = 0;
	//pidY->OutputSignal = 0;
	}

	
	if (quad.newDirection == true )
	{
	// new relative direction is given each time a new angle is set on iphone
	quad.directionSavedVal = (quad.direction - STEERING)*GAINANGLE; // -180<-->+180
	quad.newDirection = false; // reset the new direction flag
	quad.relativeDirection = angleFromOffset (quad.angleZ, quad.directionSavedVal);
	pidY->SetPoint = 0; // reset the set point
	pidY->errorI = 0; // reset integrative error
	pidY->InputPoint = 0; // reset the input point
	pidY->OutputSignal = 0; 	
	}

	pidY->InputPoint = (difangdeg ( quad.angleZ , quad.relativeDirection
	))*GAINYAW;	
	

	//----------------------------------------------------------------------

	//OUTPUT PID->OutputSignal----------------------------------------------
	pidP->OutputSignal = pid_calc (pidP,dT);
	
	pidR->OutputSignal = pid_calc (pidR,dT);
	
	pidY->OutputSignal = pid_calc (pidY, dT); 	

	
	//printf ( "prop: %.2f deriv : %.2f alpha : %.2f out : %.2f\n",
	//pidP->errorP , pidP->errorD , pidP->alpha, pidP->OutputSignal);

	
	//printf (" read BRG: %0.2f wanted BRG:%d input angle: %0.2f output
	//signal: %0.2f\n", quad.angleZ , quad.direction, pidY->InputPoint,
	//pidY->OutputSignal);
	//----------------------------------------------------------------------

	}



