// implementation Quad


#include "Qmodel.h"


int init(struct Quad *quad)
		{

		quad->power = 0;
		quad->savedPowerState = 0;
		quad->powerCheck = false;
		quad->saveval = 0;
		quad->updown = 0;
		quad->leftright = 0;
		quad->leftrightSavedVal = 0;
		quad->updownSavedVal = 0;
		quad->direction = 0; // direction and old direction relative 0
		quad->oldDirection = 0;
		quad->directionSavedVal = 0;
		quad->armdisarm = false;
		quad->offsetUD = 0;
		quad->offsetLR = 0;
		quad->offsetDir = 0;
		quad->motorUL = 0;
		quad->motorUR = 0;
		quad->motorDL = 0;
		quad->motorDR = 0;
		quad->angleX = 0;
		quad->angleY = 0;
		quad->angleZ = 0;
		quad->pxReady = false; // flag data ready PX4
		quad->newDirection = true; // start true in order to read the initial 
					   // take off value
		quad->relativeDirection = 0;
		quad->IMUstatus = 0;
		quad->IMUsavedVal = 0;
		// PX4 VARIABLE INIT
		quad->px4_comp_x =0 ;
		quad->px4_comp_y = 0;
		quad->px4_alt = 0;
		quad->px4_image_quality = 0;
		quad->posMaintain = 0;
		quad->px4OutX = 0;
		quad->px4OutY = 0;
		quad->px4OutA = 0;
		quad->baroAlt = 0;
		//k'S PID SETTINGS
		quad->kpp = 0;
		quad->kdp = 0;
		quad->kip = 0;
		quad->kpr = 0;
		quad->kdr = 0;
		quad->kir = 0;
		quad->kpy = 0;
		quad->kdy = 0;
		quad->kiy = 0;
		// testing variables 
		quad->testMode = 0;
		quad->diagnostic = false;
		quad->KsSaveFlag = false;
		quad->settingFlag = false;
		return 0;

		}

int printval(struct Quad *quad)
		{
		printf ("Power: %f ",quad->power);
		printf ("UD: %d ",quad->updown);
		printf ("LR: %d ",quad->leftright);
		printf ("Dir: %d ",quad->direction);
		printf ("Ardm-Dis: %d\n",quad->armdisarm);
		return 0;
		}

int printAngles(struct Quad *quad)
		{
		printf ("Angle X: %f  ", quad->angleX);
		printf ("Angle Y: %f  ", quad->angleY);
		printf ("Angle Z: %f\n", quad->angleZ);
		return 0;
		}


int printval2 (struct Quad *quad)

		{
		printf ("OffsetXY: %f\n",quad->offsetUD);
		printf ("OffsetZ: %f\n", quad->offsetLR);
		printf ("OffsetD: %f\n",quad->offsetDir);
		printf ("M - UL: %f\n",quad->motorUL);
     		printf ("M - UR: %f\n",quad->motorUR);
		printf ("M - DL: %f\n",quad->motorDL);
		printf ("M - DR: %f\n",quad->motorDR);
		return 0;
		}

int printKpidS (struct Quad *quad){


		printf ("kpp: %f ", quad->kpp);
		printf ("kpd: %f ", quad->kdp);
		printf ("kip: %f\n", quad->kip);
		printf ("kpr: %f ", quad->kpr);
		printf ("kdr: %f ", quad->kdr);
		printf ("kir: %f\n", quad->kir);
		printf ("kpy: %f ", quad->kpy);
		printf ("kdy: %f ", quad->kdy);
		printf ("kiy: %f\n", quad->kiy);
		printf ("enableMotorUL: %d ", quad->enableUL);
		printf ("enableMotorUR: %d ", quad->enableUR);
		printf ("enableMotorDL: %d ", quad->enableDL);
		printf ("enableMotorDR: %d\n ", quad->enableDR);

		return 0;
		}


int resetval(struct Quad *quad)
		{
		quad->power = 0;
		quad->updown = SCALEUPDOWN;
		quad->leftright = SCALELEFTRIGHT;
		quad->direction = 0;
		quad->armdisarm = false;	
		return 0;
		}


void printQNCURSES (struct Quad *quad , int row , int col , float dt)
		{
		 mvprintw (0,1, "------------AIR-----------------");
		 mvprintw (1, 1, "POWER: %3.1f", quad->power);
		 mvprintw (2,1, "UD: %d ",quad->updown);
		 mvprintw (3,1, "LR: %d ",quad->leftright);
		 mvprintw (4,1, "Dir: %d ",quad->direction);
		 mvprintw (5,1, "Ardm-Dis: %d -- OffsetUD :%3.2f -- OffsetLR :%3.2f -- HOLDpos : %d",quad->armdisarm,quad->offsetUD, quad->offsetLR, quad->posMaintain);
		 mvprintw (10,1, "---------------------MOTORs-----------------------");
		 mvprintw (11,1, "UR: %3.3f",quad->motorUR);
		mvprintw (11,12, "UL: %3.3f", quad->motorUL);
		mvprintw (11,23, "DR: %3.3f", quad->motorDR);
		mvprintw (11,34, "DL: %3.3f", quad->motorDL);
		mvprintw (11,44, "dT: %1.1f        \n", dt);

		}


int readKsFromFile (struct Quad *quad) 
		{
		printf ("Set PID limits: loading Ks parameters from file\n");
		FILE *fp;
		fp = fopen("pidks.txt", "r");
		if (fp != NULL) 
				{
		fscanf(fp,"%f %f %f", &quad->kpp , &quad->kip , &quad->kdp);
		fscanf(fp,"%f %f %f", &quad->kpr , &quad->kir , &quad->kdr);
		fscanf(fp,"%f %f %f", &quad->kpy , &quad->kiy , &quad->kdy);
				
		if (fclose(fp) != 0)
		{
		return 1;
		}
				
		return 0;
			} else {
			return 1;
			}
		}

int savePidtoFile (struct Quad *quad)
	{

	FILE *fp;

	fp = fopen("pidks.txt", "w+");

	if (fp != NULL) 
	{
	
	// saving quad values 
	// quad values are not updated 
	// instead pids values are updated
	fprintf (fp , "%.3f %.5f %.3f\n", quad->kpp , quad->kip , quad->kdp);
	fprintf (fp , "%.3f %.5f %.3f\n", quad->kpr , quad->kir , quad->kdr);
	fprintf (fp , "%.3f %.5f %.3f\n", quad->kpy , quad->kiy , quad->kdy);
	printf ("New Ks settings saved in files: pidks.txt\n");	
	
	fclose(fp);
 
	return 0;

	} else {return 1;}
	
	} 

int readOffsetFromFile (struct Quad *quad) {

		
		FILE *fpOffs;
		fpOffs = fopen("offsets.txt", "r");
		if (fpOffs != NULL) 
		{
		fscanf(fpOffs,"%f %f", &quad->offsetUD , &quad->offsetLR);
		printf ("\nStarting read offset: P:%f R:%f\n", quad->offsetUD
		,quad->offsetLR);
		fclose(fpOffs);

		return 0;
		} else {return 1;}


		}



int saveOffsetToFile (struct Quad *quad) {

		
		FILE *fp;

		fp = fopen("offsets.txt", "w+");

		if (fp != NULL) 
		{
	
		// saving quad values 
		// quad values are not updated 
		fprintf (fp , "%.3f %.5f\n", quad->offsetUD , quad->offsetLR);
		printf ("\nNew Offset settings saved in file: offsets.txt\n");	
	
		fclose(fp);
 
		return 0;

		} else {return 1;}

		}


// scalar function 
float map(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}


