#include "Pid.h"

void pidInitNULL (struct pid* myPID) 
	{
	myPID->SetPoint = 0.0; // the point we want to achieve
	myPID->InputPoint = 0.0;	
	myPID->OutputSignal =0.0;	
	myPID->Kp =0.0 ;
	myPID->Ki =0.0 ;
	myPID->Kd =0.0;
	myPID->errorP =0.0 ;		
	myPID->errorI =0.0;		
	myPID->errorD =0.0;	
	myPID->lasterrorD =0.0f;
	myPID->smoother =0.0f;
	myPID->counter = 0;
	myPID->lowfreqDt = 0.0f;
	//limitations-------------
	myPID->outMinLimit =0.0;	
	myPID->outMaxLimit =0.0;
	myPID->Dcon =0.0;
	myPID->Icon =0.0;
	myPID->Pcon =0.0;
	myPID->authKal = false;
	myPID->authFilter = false;
	
	myPID->startPid =0;	// WARNING: PID INIT STOP PID FROM EXEC
				// STARTING NEED TO BE DONE MANUALLY	

	myPID->delta = 0.0f;
	myPID->deltaS = 0.0f;
	//----------------------------------
	myPID->alphap = 0.001;
	myPID->betap = 1- myPID->alphap;
	myPID->lockedVal = 0.0;
	myPID->convLocking = false;
	myPID->resetLocking = true;
	myPID->oldD = 0.0;
	}

//settle contraint and limitations.
void SetPIDlimitations (struct pid* myPID , float Pcon , float Icon , float Dcon , float MinOut , float MaxOut , bool authFilter , bool lockingAuth)
	{
	myPID->outMinLimit =MinOut; // INFERIOR SIGNAL LIMIT
	myPID->outMaxLimit =MaxOut; // SUPERIOR SEGNAL LIMIT
	myPID->Dcon =Dcon;
	myPID->Icon =Icon;
	myPID->Pcon =Pcon;
	myPID->authFilter = authFilter;
	myPID->convLocking = lockingAuth;
	}

void printComponents (struct pid* myPID)
	{

	printf ("P:%f	I:%f	D:%f	Input:%f	Output:%f\n", myPID->errorP,myPID->errorI,myPID->errorD,myPID->InputPoint,myPID->OutputSignal);

	}

	
void pidPrint (struct pid* myPID) 
	{
	printf("------------------------->>> setpoint: %f\n",  myPID->SetPoint); 
	printf("------------------------->>> input: %f\n", myPID->InputPoint);
	printf("------------------------->>> raw out: %f\n",myPID->OutputSignal);
	printf("Kp: %f ", myPID->Kp);
	printf("Ki: %f ", myPID->Ki);
	printf("Kd: %f\n", myPID->Kd);
	printf("errorP: %f ", myPID->errorP);
	printf("errorI: %f ", myPID->errorI); 		
	printf("errorD: %f\n", myPID->errorD); 	
	printf("lastError: %f\n", myPID->lasterrorD);
	printf("MinLimit: %f ", myPID->outMinLimit); 	
	printf("MaxLimit: %f\n", myPID->outMaxLimit); 	
	printf("startPidFlag: %d\n",myPID->startPid); 
	printf("***************************************\n");
	}

void pidPrintKs (struct pid* myPID) 
	{
	printf("Kp: %f ", myPID->Kp);
	printf("Ki: %f ", myPID->Ki);
	printf("Kd: %f\n", myPID->Kd);
	}


void printPNCURSES (struct pid * pidP, struct pid * pidR ,struct pid * pidY, int row , int col)
	{
		 
		 mvprintw (6,1, "------------PIDs auth (PRY):%d%d%d---------",pidP->startPid,pidR->startPid,pidY->startPid);
		 mvprintw (7,1, "Pp: %.3f ", pidP->Kp);
		 mvprintw (7,20, "Dp: %.3f ", pidP->Kd);
		 mvprintw (7,40, "Ip: %.3f", pidP->Ki);
		 mvprintw (8,1, "Pr: %.3f ", pidR->Kp);
		 mvprintw (8,20, "Dr: %.3f ", pidR->Kd);
		 mvprintw (8,40, "Ir: %.3f", pidR->Ki);
		 mvprintw (9,1, "Py: %.3f ", pidY->Kp);
		 mvprintw (9,20, "Dy: %.3f ", pidY->Kd);
		 mvprintw (9,40, "Iy: %.3f", pidY->Ki);


		 mvprintw (12,1, "------------SET Points (what we want)-------------------");
		 mvprintw (13,1, "X : %.1f	Y: %.1f	Z: %.1f" , pidP->SetPoint,
		 pidR->SetPoint, pidY->SetPoint);
		 mvprintw (14,1, "------------Input Points (what we have in angle)--------");

		 mvprintw (15,1, "X : %.1f" , pidP->InputPoint);
		 mvprintw (15,15, "Y : %.1f" , pidR->InputPoint);
		 mvprintw (15,30, "Z : %.1f" , pidY->InputPoint);

		 mvprintw (16,1, "------------PID OUT Sig--(correction to achieve it)-----");

		 mvprintw (17,1, "P : %.1f" , pidP->OutputSignal);
		 mvprintw (17,15, "R : %.1f" , pidR->OutputSignal);
		 mvprintw (17,30, "Y : %.1f" , pidY->OutputSignal);

		 mvprintw (18,1, "PP : %.1f" , pidP->errorP);
		 mvprintw (18,15, "PD : %.1f" , pidP->errorD);
		 mvprintw (18,30, "PI : %.1f" , pidP->errorI);
	}


float pid_calc (struct pid* pid , float dt)

	{
	// pid start only if flag is active
	if (pid->startPid == true) 
	{
	float outVal = 0.0f;	


	// declare compensation factors
	float KiCompensation = IGAIN*pid->Ki; // apply compensation if required

	float KdCompensation = DGAIN*pid->Kd; // apply compensation echo "compiling Q-Model"

	//present error Setted Val - Wanted Val
	float diff = pid->SetPoint - pid->InputPoint; 



	//PROPORTIONAL----------------------------------------------------
	// time invariant
	pid->errorP = pid->Kp*(diff); 
	//----------------------------------------------------------------
	


	//DERIVATIVE------------------------------------------------------
	// derivative part: error variation in dT
	// compensation may vary with timing 
	// DGAIN defined in pid.h = 2
	//use input derivative to avoid drivative Kick
	// discart all time values more than 1.5 ms
	if (dt<=TIMELEAP){
	//APPLY K FILTER ON DERIVATIVE PART OR WILL BE TOO NOISY
	pid->errorD = -KdCompensation*((pid->InputPoint - pid->lasterrorD) / (dt)) ; 
	//pid->errorD = KdCompensation*((diff - pid->lasterrorD) / (dt)) ;
	} 


	
	if (pid->authFilter == true)
		{

		pid->oldD = (1-BETAFILTER)*pid->oldD  + BETAFILTER*pid->errorD;
		pid->errorD = pid->oldD;
		}	
		
	// save last error
	pid->lasterrorD = pid->InputPoint;
	


	//INTEGRATIVE-----------------------------------------------------
	pid->errorI += KiCompensation * dt * diff; // compensate the KI
 
	//WINDUP CORRECTION
	// apply constraint to error I 
	// we dont want errorI build up too much
	// or it will lead to instability
	pid->errorI = constrain (pid->errorI , -pid->Icon , pid->Icon);

	// no Ki no SUM
	// reset when 0 
	if (pid->Ki == 0.0) 
			{ // if D is set 0 then also SUM is reset to 0
			  // we need to null the integrative part is K is set to 0
				
				pid->errorI = 0;
				
			 }	
	// total supposed output applied with limits
	outVal = constrain((pid->errorP+pid->errorI+pid->errorD), pid->outMinLimit , pid->outMaxLimit);

	//--------------------------------------------
	return outVal ;
	//--------------------------------------------

	//--------------------------------------------
	// if pid not enabled output 0
	}
	else {
		pid->errorI = 0.0; // reset integrative build up
		return 0;
		} // if pid is not started return 0
	}	


// reset the K's
void setKpid (struct pid* pid, float kp ,float ki ,float kd)
	{

	pid->Kp =kp ;
	pid->Ki =ki ;
	pid->Kd =kd ;

	}

float difangdeg(float x, float y)
	{
	float arg;
	arg = fmod(y-x, C360);
	if (arg < 0 )  arg  = arg + C360;
	if (arg > 180) arg  = arg - C360;
	return (-arg);
	}



// return a new angle given an OFFset angle 
float angleFromOffset ( float angle , float offSetAngle ) 

	{
		
		float CoAngle = 0;

		//set limits from 0 to 360 
		if ((angle < 0 || angle > PI_QO))
		{

			if (angle < 0) 

				{angle = angle + PI_QO ;}
			else

				{angle = angle - PI_QO ;}
		
		}



		// banal case if offset = 0 than just return angle
		if (offSetAngle == 0 ) return angle;


		// offset check
		//-----------------------------------------
		CoAngle = angle + offSetAngle;

		if (CoAngle >= 0 && CoAngle <=PI_QO)
			{
				
				return CoAngle;

			} 
		else

		{

			if (CoAngle<0) CoAngle = CoAngle + PI_QO;
	
			if (CoAngle>PI_QO) CoAngle = CoAngle - PI_QO;

			return CoAngle;

		}

	}



//kalman init funcion parameters
kalman_state kalman_init(float q, float r, float p, float intial_value)
{
  kalman_state result;
  result.q = q;
  result.r = r;
  result.p = p;
  result.x = intial_value;

  return result;
}

//kalman update function
// kalman filter for only 1 variable
void kalman_update(kalman_state* state, float measurement)
{
  //prediction update
  //omit x = x
  state->p = state->p + (state->q);

  //measurement update
  state->k = state->p / (state->p + state->r);
  state->x = state->x + state->k * (measurement - state->x);
  state->p = (1 - state->k) * state->p;
  
}


void pid_a (pid_av* ma, float newdata)
{

	ma->sum = ma->sum + newdata - ma->sum/ma->samples ; //MA*[i]= MA*[i-1] +X[i] - MA*[i-1]/N
	ma->out = ma->sum / ma->samples; //MA[i]= MA*[i]/N


}

void pidAV_init ( pid_av *mv, int samples)
	{


		mv->out = 0;
		mv->sum = 0;
		mv-> samples = samples;

	}


void quickSort( float a[], int l, int r)
{
   float j;

   if( l < r ) 
   {
   	// divide and conquer
        j = partition( a, l, r);
       quickSort( a, l, j-1);
       quickSort( a, j+1, r);
   }
	
}



float partition( float a[], int l, int r) {
   int i, j;
   float t, pivot;
   pivot = a[l];
   i = l; j = r+1;
		
   while( 1)
   {
   	do ++i; while( a[i] <= pivot && i <= r );
   	do --j; while( a[j] > pivot );
   	if( i >= j ) break;
   	t = a[i]; a[i] = a[j]; a[j] = t;
   }
   t = a[l]; a[l] = a[j]; a[j] = t;
   return j;
}









