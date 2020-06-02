#include "imufilters.h"

//------------------------------------------------------------------------------
// Variable definitions
//
volatile float twoKp = (2.0f * 0.5f);// 2 * proportional gain (Kp)
volatile float twoKi =  (2.0f * 0.001f);// 2 * integral gain (Ki)
volatile float integralFBx = 0.0f,  integralFBy = 0.0f, integralFBz = 0.0f;// integral error terms scaled by Ki


float lv1=0;
float lv2=0;
float lv3=0;
float la1 = 0;
float la2 = 0;
float la3 = 0;
float g0 = 0;
float g1 = 0;
float g2 = 0;

void MadgwickAHRSupdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz, float deltat ,quaternion_t q ) {

	float q0 = q[0], q1 = q[1], q2 = q[2], q3 = q[3]; // local quaternions

	float sampleFreq = 1/deltat;
	float recipNorm;
	float s0, s1, s2, s3;
	float qDot1, qDot2, qDot3, qDot4;
	float hx, hy;
	float _2q0mx, _2q0my, _2q0mz, _2q1mx, _2bx, _2bz, _4bx, _4bz, _2q0,
	_2q1, _2q2, _2q3, _2q0q2, _2q2q3, q0q0, q0q1, q0q2, q0q3, q1q1, q1q2,
	q1q3, q2q2, q2q3, q3q3;

	// Rate of change of quaternion from gyroscope
	qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
	qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
	qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
	qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);

	// Compute feedback only if accelerometer measurement valid 
	// (avoids NaN in accelerometer normalisation)
	if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) 
		{

		// Normalise accelerometer measurement
		recipNorm = invSqrt(ax * ax + ay * ay + az * az);
		ax *= recipNorm;
		ay *= recipNorm;
		az *= recipNorm;   

		// Normalise magnetometer measurement

		recipNorm = invSqrt(mx * mx + my * my + mz * mz);
		mx *= recipNorm;
		my *= recipNorm;
		mz *= recipNorm;

		// Auxiliary variables to avoid repeated arithmetic

		_2q0mx = 2.0f * q0 * mx;
		_2q0my = 2.0f * q0 * my;
		_2q0mz = 2.0f * q0 * mz;
		_2q1mx = 2.0f * q1 * mx;
		_2q0 = 2.0f * q0;
		_2q1 = 2.0f * q1;
		_2q2 = 2.0f * q2;
		_2q3 = 2.0f * q3;
		_2q0q2 = 2.0f * q0 * q2;
		_2q2q3 = 2.0f * q2 * q3;
		q0q0 = q0 * q0;
		q0q1 = q0 * q1;
		q0q2 = q0 * q2;
		q0q3 = q0 * q3;
		q1q1 = q1 * q1;
		q1q2 = q1 * q2;
		q1q3 = q1 * q3;
		q2q2 = q2 * q2;
		q2q3 = q2 * q3;
		q3q3 = q3 * q3;

		// Reference direction of Earth's magnetic field
		hx = mx * q0q0 - _2q0my * q3 + _2q0mz * q2 + mx * q1q1 + _2q1 *
		my * q2 + _2q1 * mz * q3 - mx * q2q2 - mx * q3q3; hy = _2q0mx *
		q3 + my * q0q0 - _2q0mz * q1 + _2q1mx * q2 - my * q1q1 + my *
		q2q2 + _2q2 * mz * q3 - my * q3q3; _2bx = sqrt(hx * hx + hy *
		hy); _2bz = -_2q0mx * q2 + _2q0my * q1 + mz * q0q0 + _2q1mx * q3
		- mz * q1q1 + _2q2 * my * q3 - mz * q2q2 + mz * q3q3; _4bx =
		2.0f * _2bx; _4bz = 2.0f * _2bz;

		// Gradient decent algorithm corrective step
		s0 = -_2q2 * (2.0f * q1q3 - _2q0q2 - ax) + _2q1 * (2.0f * q0q1 +
		_2q2q3 - ay) - _2bz * q2 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz *
		(q1q3 - q0q2) - mx) + (-_2bx * q3 + _2bz * q1) * (_2bx * (q1q2 -
		q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * q2 * (_2bx * (q0q2 +
		q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
		s1 = _2q3 * (2.0f * q1q3 - _2q0q2 - ax) + _2q0 * (2.0f * q0q1 +
		_2q2q3 - ay) - 4.0f * q1 * (1 - 2.0f * q1q1 - 2.0f * q2q2 - az)
		+ _2bz * q3 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 -
		q0q2) - mx) + (_2bx * q2 + _2bz * q0) * (_2bx * (q1q2 - q0q3) +
		_2bz * (q0q1 + q2q3) - my) + (_2bx * q3 - _4bz * q1) * (_2bx *
		(q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
		s2 = -_2q0 * (2.0f * q1q3 - _2q0q2 - ax) + _2q3 * (2.0f * q0q1 +
		_2q2q3 - ay) - 4.0f * q2 * (1 - 2.0f * q1q1 - 2.0f * q2q2 - az)
		+ (-_4bx * q2 - _2bz * q0) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz
		* (q1q3 - q0q2) - mx) + (_2bx * q1 + _2bz * q3) * (_2bx * (q1q2
		- q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * q0 - _4bz * q2) *
		(_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
		s3 = _2q1 * (2.0f * q1q3 - _2q0q2 - ax) + _2q2 * (2.0f * q0q1 +
		_2q2q3 - ay) + (-_4bx * q3 + _2bz * q1) * (_2bx * (0.5f - q2q2 -
		q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * q0 + _2bz * q2) *
		(_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * q1 *
		(_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);

		recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude

		s0 *= recipNorm;
		s1 *= recipNorm;
		s2 *= recipNorm;
		s3 *= recipNorm;

		// Apply feedback step
		qDot1 -= beta * s0;
		qDot2 -= beta * s1;
		qDot3 -= beta * s2;
		qDot4 -= beta * s3;
	}

	// Integrate rate of change of quaternion to yield quaternion
	q0 += qDot1 * (1.0f / sampleFreq);
	q1 += qDot2 * (1.0f / sampleFreq);
	q2 += qDot3 * (1.0f / sampleFreq);
	q3 += qDot4 * (1.0f / sampleFreq);

	// Normalise quaternion
	recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
	q0 *= recipNorm;
	q1 *= recipNorm;
	q2 *= recipNorm;
	q3 *= recipNorm;

	q[0] = q0;
        q[1] = q1;
        q[2] = q2;
        q[3] = q3;

	
}	

//---------------------------------------------------------------------------------------------------
// IMU algorithm update
// quaternion_t is defined as global variable

void MahonyAHRSupdateIMU(float gx, float gy, float gz, float ax, float ay, float az, float deltat ,quaternion_t q ) {
	
	 float q0 = q[0], q1 = q[1], q2 = q[2], q3 = q[3];         // short name local variable for readability

	float recipNorm;
	float halfvx, halfvy, halfvz;
	float halfex, halfey, halfez;
	float qa, qb, qc;

	// Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
	if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

		// Normalise accelerometer measurement
		recipNorm = invSqrt(ax * ax + ay * ay + az * az);
		ax *= recipNorm;
		ay *= recipNorm;
		az *= recipNorm;        

		// Estimated direction of gravity and vector perpendicular to magnetic flux
		halfvx = q1 * q3 - q0 * q2;
		halfvy = q0 * q1 + q2 * q3;
		halfvz = q0 * q0 - 0.5f + q3 * q3;
	
		// Error is sum of cross product between estimated and measured direction of gravity
		halfex = (ay * halfvz - az * halfvy);
		halfey = (az * halfvx - ax * halfvz);
		halfez = (ax * halfvy - ay * halfvx);

		// Compute and apply integral feedback if enabled
		if(twoKi > 0.0f) {
			integralFBx += twoKi * halfex * (1.0f * deltat );	// integral error scaled by Ki
			integralFBy += twoKi * halfey * (1.0f * deltat);
			integralFBz += twoKi * halfez * (1.0f * deltat);
			gx += integralFBx;	// apply integral feedback
			gy += integralFBy;
			gz += integralFBz;
		}
		else {
			integralFBx = 0.0f;	// prevent integral windup
			integralFBy = 0.0f;
			integralFBz = 0.0f;
		}

		// Apply proportional feedback
		gx += twoKp * halfex;
		gy += twoKp * halfey;
		gz += twoKp * halfez;
	}
	
	// Integrate rate of change of quaternion
	gx *= (0.5f * (1.0f * deltat));		// pre-multiply common factors
	gy *= (0.5f * (1.0f * deltat));
	gz *= (0.5f * (1.0f * deltat));
	qa = q0;
	qb = q1;
	qc = q2;
	q0 += (-qb * gx - qc * gy - q3 * gz);
	q1 += (qa * gx + qc * gz - q3 * gy);
	q2 += (qa * gy - qb * gz + q3 * gx);
	q3 += (qa * gz + qb * gy - qc * gx); 
	
	// Normalise quaternion
	recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
	q0 *= recipNorm;
	q1 *= recipNorm;
	q2 *= recipNorm;
	q3 *= recipNorm;

	q[0] = q0;
        q[1] = q1;
        q[2] = q2;
        q[3] = q3;
}




//---------------------------------------------------------------------------------------------------
// Fast inverse square-root
// See: http://en.wikipedia.org/wiki/Fast_inverse_square_root
float invSqrt(float x) 
		{
	 	int32_t i = 0x5F1F1412 - (*(int32_t*)&x >> 1);
   		float tmp = *(float*)&i;
   		return tmp * (1.69000231f - 0.714158168f * x * tmp * tmp);
		}


//compute pitch roll yaw from quaternion
void computePRY (quaternion_t q , attQ * att)
	{	
		att->yaw = -atan2(2.0f * (q[1] * q[2] + q[0] * q[3]), q[0] *
		q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3]); att->pitch =
		-asin(2.0f * (q[1] * q[3] - q[0] * q[2])); att->roll =
		atan2(2.0f * (q[0] * q[1] + q[2] * q[3]), q[0] * q[0] - q[1] *
		q[1] - q[2] * q[2] + q[3] * q[3]);

		//degrees conversion
    		att->roll *= 180.0f / PI;
    		att->yaw   *= 180.0f / PI; 
    		att->pitch  *= 180.0f / PI;

	}

//initialize quaternions
void initQuaterion (quaternion_t q , float q0 , float q1 , float q2 , float q3)
	{
		
		q[0] = q0;
		q[1] = q1;
		q[2] = q2;
		q[3] = q3;
	}



// moving average general filter 
void moving_average (mov_av* ma, float newdata)
	{
	
	//MA*[i]= MA*[i-1] +X[i] - MA*[i-1]/N
	ma->sum = ma->sum + newdata - ma->sum/ma->samples ; 
	//MA[i]= MA*[i]/N
	ma->out = ma->sum / ma->samples; 

	}


//initialize moving average filtering
void ma_init ( mov_av *mv, int samples)
	{

		mv->out = 0;
		mv->sum = 0;
		mv-> samples = samples;

	}


//moving average filtering for gyro.
void filterGyro ( float *gyro , mov_av *x , mov_av *y , mov_av *z)
	{

		moving_average (x , gyro[0]);
		moving_average (y , gyro[1]);
		moving_average (z , gyro[2]);
			
		gyro[0] = x->out;
		gyro[1] = y->out;
		gyro[2] = z->out;

	}


// moving average filtering for accelerometer
void filterAcc ( float *acc , mov_av *x , mov_av *y , mov_av *z)
	{

		moving_average (x , acc[0]);
		moving_average (y , acc[1]);
		moving_average (z , acc[2]);

		acc[0] = x->out;
		acc[1] = y->out;
		acc[2] = z->out;
	}


// coputer filtered  data 
// general moving average general filtering
void filterAtt ( attQ  *att , mov_av *x , mov_av *y , mov_av *z)
	{

		moving_average (x , att->pitch);
		moving_average (y , att->roll);
		moving_average (z , att->yaw);
			
		att->pitch = x->out;
		att->roll = y->out;
		att->yaw = z->out;
			
	}


// computer compensated Z linear acceleration
// data computed is required for altitude compensation 
float compute_compensated_acc (quaternion_t q , float *acc)

	{
		float gx, gy, gz; // estimated gravity direction

		gx = 2 * (q[1]*q[3] - q[0]*q[2]);
  		gy = 2 * (q[0]*q[1] + q[2]*q[3]);
  		gz = q[0]*q[0] - q[1]*q[1] - q[2]*q[2] + q[3]*q[3];

	  // return vertical acceleration without gravity
  	 // (A dot G) / |G| - 1G (|G| = 1) -> (A dot G) - 1G
  		return ((acc[0]*gx + acc[1]*gy + acc[2]*gz) - 1.0);
	
	}


