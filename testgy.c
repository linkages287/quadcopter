
#include <stdio.h>
#include <time.h>
#include <sys/time.h>
#include <errno.h>
#include <string.h>
#include <unistd.h>
#include <stdint.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <math.h>
#include "imufilters.h"
#include "imuLib.h"
//#include "clock.h"
/*
#define	I2C_BUFFER_LEN 8
#define MAX_WRITE_LEN 511
*/







	//int I2C_bus_write(int ,  char *, int );
	//int I2C_bus_read (int ,  char *, int );
	//void delay_msek(long int msek);
	//int i2c_open();
	//void i2cSetAddress(int , int);



//---------------------------------------
int i2c_bus = 2; // i2c_1
int i2c_fd;
int current_slave;
char buff[MAX_WRITE_LEN + 1];
//----------------------------------------
quaternion_t q;
attitude_t v;
calibration c;
//Qclock qclock; // clock 
attQ  att;


int main () 

{

	//unsigned char* gpt_regs; // timer base register	
	//register uint32_t partial;

	c.counter0 = 1000;
	c.counter1 = 1000;
	c.calibdone = false;

	
	struct timeval lastSample;
   	struct timeval thisSample;
    	struct timeval dtSample;
    	//gettimeofday(&lastSample, NULL);
	q[0] = 1.0f, q[1] = 0.0f, q[2] = 0.0f, q[3] = 0.0f;
	

//float pitchGyr = 0;
//float rollGyr = 0;
//float yawGyr = 0;
//float pitchAccel = 0;
//float rollAccel = 0;
//float yaw = 0;
//float pitch = 0;
//float roll = 0;
float dt = 0;
//float suppTimer = 0;
int counter = 10;
mov_av x , y , z;
mov_av accx , accy ,accz;

ma_init(&x, 5);
ma_init(&y, 5);
ma_init(&z, 5);

ma_init(&accx, 5);
ma_init(&accy, 5);
ma_init(&accz, 5);

float bearing = 0;

 //raw Data
   int rawAcc[3];
   int rawGyro[3];
   // int rawMag[3];
 //raw Data
    float acc[3];
    float gyro[3];
    float mag[3];

//int comresponse = 0;


// DECLARE and INITIALIZE CLOCK DEVICE
	//-----------------------------------------------------------------
	//qclock.filedescriptor = timerInitQ(); // file desc timer device
	//mapping /dev/mem
	//gpt_regs = (unsigned char*) mmap(0, MAP_SIZE,PROT_READ|PROT_WRITE, MAP_SHARED, qclock.filedescriptor, BASEMEM & ~MAP_MASK);
	// test timer for tick definitions, device read from 24Mhz clock
	//initClock (&qclock);
	
	//tickTest (gpt_regs, &qclock);
	//tick definitions are saved on average of 5 sec-------------------
	//printClock(&qclock);
	//-----------------------------------------------------------------




		// open i2c device
		i2c_fd = i2c_open(); // open user space device

		imuInit(i2c_fd);

		// set trasmission address
		i2cSetAddress (MCU6050_ADDR, i2c_fd); // set device address

		while (c.calibdone !=true)
		{

			I2C_bus_read (ACCEL_XOUT_H , buff, 14 ,i2c_fd, buff);
			
			rawAcc[0]= ((signed short)(buff[0]*256+buff[1]));
        		rawAcc[1]= ((signed short)(buff[2]*256+buff[3]));
        		rawAcc[2]= ((signed short)(buff[4]*256+buff[5]));

			

			rawGyro[0]= ((signed short)(buff[8]*256+buff[9]));
        		rawGyro[1]= ((signed short)(buff[10]*256+buff[11]));
        		rawGyro[2]= ((signed short)(buff[12]*256+buff[13]));	


			c.rawGyroX = rawGyro[0];
			c.rawGyroY = rawGyro[1];
			c.rawGyroZ = rawGyro[2];
			Cal (&c);

			//printf ("Xo: %f Yo: %f  Zo: %f , counter: %d\n", c.offsetGyroX, c.offsetGyroY ,c.offsetGyroZ, c.counter0);						

		}


		//qclock.savedtimer1 = DMT;
		gettimeofday(&lastSample, NULL);
		
		while (true)
		{
		
		//usleep(10);

		//qclock.actualtimer = DMT; // save actual timer in a global variable

		//partial = abs(qclock.savedtimer1 - qclock.actualtimer);

	
		gettimeofday(&thisSample, NULL);

		timersub(&thisSample, &lastSample, &dtSample);

		dt = ((float)dtSample.tv_usec/1000000.0f) ;
		//printf ("partial : %d\n" , partial);

		if (dt >= 0.001) {

		//suppTimer = (float)partial/1000;
		//dt=dt*10;
		

		//i2cSetAddress (i2c_fd , MCU6050_ADDR , i2c_fd); // set device address
	        //I2C_bus_read (ACCEL_XOUT_H , buff, 14 , i2c_fd, buff);

		//gettimeofday(&thisSample, NULL);
 		//rawAcc[0]= ((signed short)(buff[0]*256+buff[1]));
        	//rawAcc[1]= ((signed short)(buff[2]*256+buff[3]));
        	//rawAcc[2]= ((signed short)(buff[4]*256+buff[5]));
        	//acc[0]= (rawAcc[0])/2048.0f*9.81f;
        	//acc[1]= (rawAcc[1])/2048.0f*9.81f;
        	//acc[2]= (rawAcc[2])/2048.0f*9.81f; 
		//rawGyro[0]= ((signed short)(buff[8]*256+buff[9]));
        	//rawGyro[1]= ((signed short)(buff[10]*256+buff[11]));
        	//rawGyro[2]= ((signed short)(buff[12]*256+buff[13]));

//-------------

//			moving_average (&x , rawAcc[0]);
//			moving_average (&y , rawAcc[1]);
//			moving_average (&z , rawAcc[2]);

			//printf ("XAcc:%d YAcc:%d ZAcc:%d ", rawAcc[0],rawAcc[1],rawAcc[2]);
			

//			rawAcc[0] = x.out;
//			rawAcc[1] = y.out;
//			rawAcc[2] = z.out;
			//printf ("MAXAcc:%d MAYAcc:%d MAZAcc:%d \n", rawAcc[0],rawAcc[1],rawAcc[2]);
//----------------------



//	  	gyro[0]= (rawGyro[0])/65.5  - c.offsetGyroX;
//       	  	gyro[1]= (rawGyro[1])/65.5  - c.offsetGyroY;
//          	gyro[2]= (rawGyro[2])/65.5  - c.offsetGyroZ; 

		// 1KHz / 5 -> 200Hz sensor update on moving average
		if (counter >= 5) 
		{

		get_acc_gyro (acc, gyro, i2c_fd, buff, c);

		filterGyro ( gyro , &x , &y , &z);

		filterAcc (acc, &accx , &accy, &accz);

		get_mag (mag, i2c_fd , buff);


		counter = 0;
		}
		counter++;


		//i2cSetAddress (i2c_fd , HMC5883L_ADDR, i2c_fd); // set device address
		//I2C_bus_read (MAG_XOUT_H , buff, 6 , i2c_fd , buff);
		//rawMag[0]= ((signed short)(buff[0]*256+buff[1]));
         	//rawMag[2] = ((signed short)(buff[2]*256+buff[3]));
         	//rawMag[1] = ((signed short)(buff[4]*256+buff[5]));

		//mag[0] = rawMag[0] * 0.92;
		//mag[1] = rawMag[1] * 0.92;
		//mag[2] = rawMag[2] * 0.92;
         	
		//bearing = atan2 (mag[1],mag[0]) * RAD_TO_DEGREE;
		

		//printf ("g3 %f\n",gyro[2]);
		//printf ("bearing %f\n",bearing);
		//printf("RAW GURO Pitch = %05.5f : Roll = %05.5f\n ",  gyro[0],  gyro[1]);
        	
        	
		//printf ("X:%f Y:%f Z:%f DT:%f \n", x_accel,y_accel,z_accel,(float)dtSample.tv_usec/1000000);
		//printf ("X:%f Y:%f Z:%f DT:%f \n", acc[0],acc[1],acc[2],(float)dtSample.tv_usec/1000000);

		//pitchGyr += ((gyro[0])* ((float)dtSample.tv_usec / 1000000));
        	//rollGyr -= ((gyro[1])* ((float)dtSample.tv_usec / 1000000));
		//yawGyr += ((gyro[2])* ((float)dtSample.tv_usec / 1000000));

		//printf ("Xmax:%d Ymag:%d Zmag:%d DT:%f \n", rawMag[0],rawMag[1],rawMag[2],(float)dtSample.tv_usec/1000000);
		//printf ("Xmax:%f Ymag:%f Zmag:%f B:%f \n", mag[0],mag[1],mag[2],bearing);

		//printf ("gyro sun %f\n", yawGyr);

		//MadgwickIMUupdate(gyro[0]*DEGREE_TO_RAD, gyro[1]*DEGREE_TO_RAD, gyro[2]*DEGREE_TO_RAD,  acc[0],  acc[1],  acc[2] , (float)dtSample.tv_usec/1000000 , q);

		
		//MadgwickIMUupdate(gyro[0]*DEGREE_TO_RAD, gyro[1]*DEGREE_TO_RAD, gyro[2]*DEGREE_TO_RAD,  acc[0],  acc[1],  acc[2] , (float)dtSample.tv_usec/1000000 , q);

		MadgwickAHRSupdate(gyro[0]*DEGREE_TO_RAD, gyro[1]*DEGREE_TO_RAD,
		gyro[2]*DEGREE_TO_RAD, acc[0], acc[1], acc[2], mag[0], mag[1],
		mag[2], dt , q );


		//MadgwickAHRSupdate(gyro[0]*DEGREE_TO_RAD, gyro[1]*DEGREE_TO_RAD, gyro[2]*DEGREE_TO_RAD,  acc[0],  acc[1],  acc[2], mag[0], mag[1], mag[2], dt , q );
		
		//filterAtt ( &att , &x , &y , &z);
		computePRY ( q ,  &att);
		

		//printf ("q1: %f q2: %f q3: %f q4: %f\n",q[0],q[1],q[2],q[3]);
		

		//yaw   = atan2(2.0f * (q[1] * q[2] + q[0] * q[3]), q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3]);   
    		//pitch = -asin(2.0f * (q[1] * q[3] - q[0] * q[2]));
    		//roll  = atan2(2.0f * (q[0] * q[1] + q[2] * q[3]), q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3]);
    		//pitch *= 180.0f / PI;
    		//yaw   *= 180.0f / PI - 13.8f; // Declination at Danville, California is 13 degrees 48 minutes and 47 seconds on 2014-04-04
    		//roll  *= 180.0f / PI;

		//if (yaw <0) yaw = yaw + 360;

		//quaternionToEuler(q , v);

		//printf ("v1: %f v2: %f v3: %f\n",v[0],v[1],v[2]);




		printf ("pitch: %.2f roll: %.2f yaw: %.2f dT: %.5f\n" ,att.pitch,att.roll,att.yaw, dt);


		//qclock.savedtimer1 = DMT; // save new timer before looping
		
		gettimeofday(&lastSample, NULL);
		//printf ("v1: %.2f v2: %.2f v3: %.2f dT: %f  Dtmachine: %d\n",pitch,roll,yaw, (float)dtSample.tv_usec/1000000 , partial);
		
		} //end if

		//printf("Pitch = %05.5f : Roll = %05.5f\n ", pitchGyr, rollGyr);

		//printf ("gyro sun %f\n", yawGyr);

		// compleemntary filter 

		//pitchAccel = 57.295*atan(acc[1]/ (sqrt(pow(acc[2],2)+pow(acc[0],2))));
		//rollAccel = 57.295*atan(-acc[0]/ (sqrt(pow(acc[2],2)+pow(acc[1],2))));	


		//pitchGyr = (pitchGyr * 0.99) + (pitchAccel * 0.01);
        	//rollGyr = (rollGyr * 0.99) + (rollAccel * 0.01);

		//printf("PitchA = %03.3f : RollA = %03.3f dT: %f\n ", pitchAccel, rollAccel, (float)dtSample.tv_usec/1000000);
		//printf("Pitch = %05.1f : Roll = %05.1f DT: %f\n ", pitchGyr, rollGyr,(float)dtSample.tv_usec/1000000);
		}


}


