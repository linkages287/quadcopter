
// implement a structure to save each element 
// elements need to be saved on a file 
// and tested with a graph
#include "element.h"



/*========================================================================*/
/* ------------------insert data Element with MALLOC----------------------*/
/* L'ULTIMO ELEMENTO INSERITO VIENE ACCODATO IN TESTA ALLA LISTA   	  */
/*========================================================================*/

dataSet* add_element (  dataSet *last, long int seq , float RaP , float RaR , float RaY , float SaP , float SaR , float SaY,  float
pitchP , float pitchI , float pitchD , float rollP , float rollI , float rollD ,float yawP , float yawI , float yawD,float totCP, float totCR, float totCY , float dT)

  {
	dataSet *new = NULL;
	  
	/*definisce lo spazio di memoria dell'elemento 
	creato assegnandolo al puntatore*/

	new = (dataSet *)malloc (sizeof(dataSet));
        	
	/* se malloc restituisce NULL la funzione non e' riuscita ad effettuare 
	l'allocazione per il nuovo elemento*/
        
        if (new == NULL) 
	{
		
	printf ("Error memory HEAP");
	return new;
		
	}
	// --- data inserted in the list
	new->seq = seq;
	new->ReadanglePitch = RaP;
	new->ReadangleRoll = RaR;
	new->ReadangleYaw = RaY;
	new->PP = pitchP;
	new->PD = pitchD;
	new->PI = pitchI;
	new->RP = rollP;
	new->RI = rollI;
	new->RD = rollD;
	new->YP = yawP;
	new->YI = yawI;
	new->YD = yawD;
	new->totCorrectionP = totCP;
	new->totCorrectionR = totCR;
	new->totCorrectionY = totCY;
	new->SetanglePitch = SaP;
	new->SetangleRoll = SaR;
	new->SetangleYaw = SaY;
	new->dT = dT;
	// --- hoock to the next element 
	last->next = new;
	
		
	//new->next = last; // the last element is the 1 appearing  
	
	
	return new; 
  }	



void print_set (dataSet *set)
	
	{ 	/*definizione variabili locali*/
	        dataSet *new=NULL;

		for(new=set; new!=NULL ;new=new->next)	
		
		{


		printf (" sequencing number: %ld  \n", new->seq);	
		printf (" RaP: %.2f : ", new->ReadanglePitch);
		printf (" RaR: %.2f : ", new->ReadangleRoll);
		printf (" RaY: %.2f \n ", new->ReadangleYaw);

		printf (" AngleSet-P %.2f : ",new->SetanglePitch);
		printf (" AngleSet-R %.2f : ",new->SetangleRoll);
		printf (" AngleSet-Y %.2f \n ",new->SetangleYaw);

		printf (" PPval : %.2f : ", new->PP);
		printf (" PDval : %.2f : ", new->PD);
		printf (" PIval : %.2f\n ", new->PI);

		printf (" RPval : %.2f : ", new->RP);
		printf (" RDval : %.2f : ", new->RD);
		printf (" RIval : %.2f\n ", new->RI);

		printf (" YPval : %.2f : ", new->YP);
		printf (" YDval : %.2f : ", new->YD);
		printf (" YIval : %.2f\n ", new->YI);
		
		printf (" TOT-P-Corr: %.2f : ", new->totCorrectionP);
		printf (" TOT-R-Corr: %.2f : ", new->totCorrectionR);
		printf (" TOT-Y-Corr: %.2f \n", new->totCorrectionY);
		
		
		
		}				
	}   



int writeFile (dataSet *set)	
	{

		//w  - open for writing (file need not exist)
		//a  - open for appending (file need not exist)
		//r+ - open for reading and writing, start at beginning
		//w+ - open for reading and writing (overwrite file)
		//a+ - open for reading and writing (append if file exists)


		FILE *fpData;
		fpData = fopen("dataAnalize.txt", "w+");
		if (fpData != NULL) 
				{


				 dataSet *new=NULL;

				for(new=set; new->seq != DIAGTIME ;new=new->next)	
		
				{
				// to do scan set 
				// write data in set 
				// fprintf(fp, "%s %s %s %d", "We", "are", "in", 2012);

				fprintf (fpData , "%ld %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f\n", new->seq,
				new->ReadanglePitch , new->ReadangleRoll ,
				new->ReadangleYaw , new->PP , new->PD , new->PI
				, new->RP , new->RI , new->RD ,new->YP , new->YI
				,new->YD , new->totCorrectionP ,
				new->totCorrectionR , new->totCorrectionY ,
				new->SetanglePitch , new->SetangleRoll ,
				new->SetangleYaw ,new->dT );
				
				}
				fclose(fpData);

				return 0;
				} else {return 1;}

					}


void *writeFilePT (void *set)	
	{

		//w  - open for writing (file need not exist)
		//a  - open for appending (file need not exist)
		//r+ - open for reading and writing, start at beginning
		//w+ - open for reading and writing (overwrite file)
		//a+ - open for reading and writing (append if file exists)


		FILE *fpData;
		fpData = fopen("dataAnalize.txt", "w");
		if (fpData != NULL) 
				{
				

				dataSet * new = (dataSet *)set;
				
				for(; new->seq != DIAGTIME ;new=new->next)	
		
				{
				
				// to do scan set 
				// write data in set 
				// fprintf(fp, "%s %s %s %d", "We", "are", "in", 2012);

				fprintf (fpData , "%ld %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f\n", new->seq,
				new->ReadanglePitch , new->ReadangleRoll ,
				new->ReadangleYaw , new->PP , new->PD , new->PI
				, new->RP , new->RD , new->RI ,new->YP , new->YD
				,new->YI , new->totCorrectionP ,
				new->totCorrectionR , new->totCorrectionY ,
				new->SetanglePitch , new->SetangleRoll ,
				new->SetangleYaw ,new->dT );
				
				}
				fclose(fpData);
				
				} 
				return 0;
					}

void pthreatWriteFile (dataSet *set)
	{
		pthread_t writeThread;
		
		pthread_create (&writeThread , NULL , writeFilePT , (void *)set);		
		
		printf ("DIAGNOSTIC DATA ACQUIRED\n");


	}





	




		

			

