#include "toserial.h"


///static termios bzero;

int connectSerial(char *device) {


	int fd; //file descriptor for USB trasmission

	struct termios options; // define serial option in termios structure

   	//bzero(&options, sizeof(options));

	//------------------------------------------------------------------
	//opening device 
	fd = open(device, O_RDWR | O_NOCTTY | O_NDELAY);

	if (fd == -1)
     	{
	printf ("\nUnable to open the selected port %s\n", device);

	return 0;      
	}	
		else 	{
			printf ("\nDevice %s opened\n",device);
		  	
			}

	// configuring options

	fcntl(fd, F_SETFL, FNDELAY);
	memset(&options,0,sizeof(options));
	//bzero(&options, sizeof(options));
	
	//bit masking 8N1//
	options.c_cflag &= ~CSTOPB;
	options.c_cflag &= ~CSIZE;
	/* no hardware flow control */
 	options.c_cflag &= ~CRTSCTS;
	options.c_cflag = B115200 | CRTSCTS | CS8 | CLOCAL | CREAD;
        options.c_iflag = IGNPAR;
	options.c_oflag = 0; // raw oputput
	        
	//set trasmission speed
	cfsetispeed(&options, B115200);
	cfsetospeed(&options, B115200);

	/*Raw input is unprocessed. Input characters are passed through exactly 
	as they are received, when they are received. 		
	Generally you'll deselect the ICANON, ECHO, ECHOE, and ISIG options 
	when using raw input: */

	options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG); 
	
	// block reading untill MIN_SIZE is received
	// THIS SI A BLOKING CALL
	options.c_cc[VMIN]  = MIN_SIZE; 
	options.c_cc[VTIME] = 0; 

	/* set the options as ACTIVE NOW */	
	tcsetattr(fd, TCSANOW, &options);// write parameters
	tcflush( fd, TCIFLUSH );//flush anything in the channel
	printf ("Return fd: %d\n",fd);

	return fd;


}

int bleAliveSignal (int fd){

	char StringBuf[DIMB];

     	StringBuf[0] = '\0';

	int i = 0;	
	
		snprintf(StringBuf, DIMB , "WZ\n") ;
		
		//WRITE to serial FD attempt 3 times
		for (i=0;i<3;i++){

			if ((write (fd, StringBuf , DIMB-1)) <0)
				{
				printf ("Serial Write BLE failed\n");
				return -1;			
				}
			usleep(100);
		}
		return 1; // trasmissione succesfull
}


// inizialize altimeter kalman filtering with string
// fd -> device
// a -> setMeasurementError
// b -> setEstimateError
// c -> setProcessNoise
// d -> kalaman flag on off
int initAltimeter (int fd, float a, float b, float c, int d, int freq)
	{
	
	bool success = false;	
	
	char StringBuf[BAROBUF];

	StringBuf[0] = '\0';

	int i = 0;

	char buff='*';

	int readresponse = 0;

	snprintf(StringBuf, BAROBUF , "a%.3fb%.3fc%.3fd%de%dz\n",a,b,c,d,freq);

	//WRITE to BARO serial FD attempt 3 times
		while (!success){

			if ((write (fd, StringBuf , BAROBUF-1)) <0)
				{
				printf ("Serial Write %d ALTIMETER failed\n",i);
				
				i++;
				if(i==3)
				{
				printf ("barometer unlinked\n");

				return -1;
				}			
			}
			usleep(100);

		// read response if positive then success 
		//---------------------------------------		
		readresponse= read(fd, &buff,1)	;

	        if ((readresponse>0) && ((buff=='.')||(buff=='a')))
			{
			success=true;
			printf ("%f %f %f %d\n",a,b,c,d);
			}

		}
		return 1; // trasmissione succesfull

	} //end function






