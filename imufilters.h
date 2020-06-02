
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <unistd.h>
#include <stdint.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <fcntl.h>
#include <linux/i2c-dev.h>

// System constants

#define PI 3.14159265358979f

#define beta 0.12f

#define GyroMeasDrift  PI * (5.0f / 180.0f)

#define	DEGREE_TO_RAD	((float)M_PI / 180.0f)

#define	RAD_TO_DEGREE	(180.0f / (float)M_PI)

#define TWO_PI	(2.0f * (float)M_PI)

#define FLOOP 5

#define CONV_FACTOR 1 // raw acc to G conversion factor 

#define ZACC_MIN 0.001f // Zacc constraints

typedef struct {

	float sum;
	short int samples;
	float out;
	} mov_av;

typedef struct {

		float pitch;
		float roll;
		float yaw;
	} attQ;


typedef float quaternion_t[4]; 

typedef float attitude_t[3];

void initQuaterion (quaternion_t q , float q0 , float q1 , float q2 , float q3);

void ma_init ( mov_av * , int samples);

void moving_average (mov_av* ma, float newdata);

//main fusion filter by Madwick
void MadgwickAHRSupdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz, float deltat ,quaternion_t q);

void MahonyAHRSupdateIMU(float gx, float gy, float gz, float ax, float ay, float az, float deltat ,quaternion_t q );

float invSqrt(float x);

void filterGyro ( float *gyro, mov_av *x , mov_av *y , mov_av *z);

void filterAcc ( float *acc , mov_av *x , mov_av *y , mov_av *z);

void filterAtt ( attQ  *att , mov_av *x , mov_av *y , mov_av *z);

void computePRY (quaternion_t q , attQ * att);

float compute_compensated_acc (quaternion_t q , float *acc);






