
#include "imuLib.h"
 
// get acc and gyro sensor values
void get_acc_gyro (float * acc, float * gyro , int i2c_fd, unsigned char * buff, calibration c ){

		int comres = 0;

		i2cSetAddress (MCU6050_ADDR , i2c_fd); // set device address
		
		// read values from reg_addr
	        comres = I2C_bus_read (ACCEL_XOUT_H , buff, 14 , i2c_fd); 

		//

        	acc[0]= ((signed short)(buff[0]*256+buff[1])- c.OffsetAccX)/ACC_SENS_2G;
        	acc[1]= ((signed short)(buff[2]*256+buff[3])- c.OffsetAccY)/ACC_SENS_2G;
        	acc[2]= ((signed short)(buff[4]*256+buff[5])- c.OffsetAccZ)/ACC_SENS_2G; 

		gyro[0]= ((signed short)(buff[8]*256+buff[9]))/GYRO_SENS500  - c.offsetGyroX;
       	  	gyro[1]= ((signed short)(buff[10]*256+buff[11]))/GYRO_SENS500 - c.offsetGyroY;
          	gyro[2]= ((signed short)(buff[12]*256+buff[13]))/GYRO_SENS500 - c.offsetGyroZ; 

		if (comres <0) printf ("ACC or GYRO comm error reading, error: %d\n", comres);

}


// get mag sensor values
void get_mag (float * mag, int i2c_fd , unsigned char * buff, calibration c ){

		int comres = 0;
		i2cSetAddress (HMC5883L_ADDR, i2c_fd); // set device address
		comres = I2C_bus_read (MAG_XOUT_H , buff, 6 , i2c_fd); // read values from reg


		mag[0] = ((signed short)(buff[0]*256+buff[1])- c.OffsetMagX )* 0.92;
		mag[2] = ((signed short)(buff[2]*256+buff[3])- c.OffsetMagY )* 0.92;
		mag[1] = ((signed short)(buff[4]*256+buff[5])- c.OffsetMagZ )* 0.92;
		
		
		if (comres <0) printf ("MAG comm error reading\n");
		
		//printf ("MAG0 %d MAG1 %d MAG2 %d\n",maga,magb,magc);
}


void imuInit(int i2c_fd)
	
	{	
	//check MPU6000 manual for details on configuration parameters
	//setup is divided is ADDRESS and PARAMETER to be set

	unsigned char setup1[2] = {0x19,0x07}; // sample rate divider (0x07 means 8kHz/(1 + 0x07) = 1kHz)
	//Disable FSync, and set DLPF on
	unsigned char setup1_1[2] = {0x1A,0b00000001}; // enable DLP filter for acc and gyro
    	unsigned char setup2[2] = {0x1B,0x08}; // gyro config range (0x08 means +/-500 degs/sec)
					      //00001000 = 2^3 = 8   1*2^3+0*2^2+0*2^1+0*2^0
	unsigned char setup3[2] = {0x6B,0x02}; // power management
    	unsigned char setup4[2] = {0x1C,0b00000000}; // accel config range (0x00 means +/- 2g)
	unsigned char setup5[2] = {0x37,0x02}; // internal pin config for MAG 
					       // bypass mode to read i2c need to be enabled

	unsigned char setup6[2] = {0x6A,0x00}; // user control FIFO
	unsigned char setup7[2] = {0x6B,0x00}; // compass stby mode setted OFF to read value
	unsigned char setup9[2] = {0,0b00011000}; // REG A CONFIGURATION 
	unsigned char setup8[2] = {1,0b00100000}; // REG B CONFIGURATION  
	unsigned char setup10[2] ={2,0b00000000}; // OPERATION MODE 

	 // write the config data for mcu6050
	i2cSetAddress (MCU6050_ADDR, i2c_fd); // set device address
    	write (i2c_fd,setup1, 2); 
	write (i2c_fd,setup1_1, 2);
    	write (i2c_fd,setup2, 2);
    	write (i2c_fd,setup3, 2);
    	write (i2c_fd,setup4, 2);
	write (i2c_fd,setup5, 2);
	write (i2c_fd,setup6, 2);
	write (i2c_fd,setup7, 2);

 	// write the config data for hmc5883
	i2cSetAddress (HMC5883L_ADDR, i2c_fd); // set device address
	write (i2c_fd,setup8, 2);
	write (i2c_fd,setup9, 2);
	write (i2c_fd,setup10, 2);

	}



//----------------------------------------------------------------------
// set the I2C slave address for all subsequent I2C device transfers
//----------------------------------------------------------------------
void i2cSetAddress(int address, int i2c_fd)
{
	
	// set address 
	if (ioctl(i2c_fd, I2C_SLAVE, address) < 0) {
		printf("i2cSetAddress\n");
		
	}

	// use checksum
	if( ioctl( i2c_fd , I2C_PEC,  address) < 0) { 
		printf ("i2cSetPEC\n"); }
	
}

//----------------------------------------------------------------------

//----------------------------------------------------------------------
// open device i2c I2C_BUS2 contains the i2c bus to be used
//----------------------------------------------------------------------
int i2c_open()
{
	char buff[32];
	int i2c_fd = 0;

		sprintf(buff, "/dev/i2c-%d", I2C_BUS2);

		i2c_fd = open(buff, O_RDWR);

		if (i2c_fd < 0) {
			printf ("ERROR OPENING I2C USERSPACE DRIVER\n");
			i2c_fd = 0;
			
		}
	
 return i2c_fd;

}
//---------------------------------------------------------------------------

int I2C_bus_write(int reg_addr,  unsigned char *buff, int length , int i2c_fd)
{


int result, i;

	if (length > MAX_WRITE_LEN) {
		printf("Max write length exceeded in linux_i2c_write()\n");
		return -1;
	}


	
	if (length == 0) {
		result = write(i2c_fd, &reg_addr, 1);

		if (result < 0) {
			perror("write:1");
			return result;
		}
		else if (result != 1) {
			printf("Write fail:1 Tried 1 Wrote 0\n");
			return -1;
		}
	}
	else {
		buff[0] = reg_addr;

		for (i = 0; i < length; i++)
			buff[i+1] = buff[i];

		result = write(i2c_fd, buff, length + 1);

		if (result < 0) {
			perror("write:2");
			return result;
		}
		else if (result < (int)length) {
			printf("Write fail:2 Tried %u Wrote %d\n", length, result); 
			return -1;
		}
	}

	return 0;
}

//---------------------------
// read i2c bus function 
//---------------------------
int I2C_bus_read(int reg_addr,  unsigned char *buff, int cnt , int i2c_fd)

{
	// need to tell bno055 what address i need to read
	if (write(i2c_fd, &reg_addr, 1) != 1) {
	printf ("I2c write failed()\n");
    	return -1;
						}
	// read that address
	if (read(i2c_fd, buff, cnt) != cnt) {
	printf ("I2C read failure()\n");
    	return -2;
						}
return 0;
}


void delay_msek(long int msek)
{
	struct timespec ts;

	ts.tv_sec = msek / 1000;
	ts.tv_nsec = (msek % 1000) * 1000000;

	nanosleep(&ts, NULL);
}



void calibGyro (unsigned char *buff , int i2c_fd)

{

	printf ("GYRO CALIBRATION: maintain the quad stable , press enter when ready\n");
	
	getchar();

	FILE *fdata;
	fdata = fopen("calibGyro.txt", "w+");

   	int rawGyro[3];
	int counter0 = CALIBLOOP;
	float gyrox =0, gyroy =0, gyroz=0;

	bool calibdone = false;
	
	i2cSetAddress (MCU6050_ADDR , i2c_fd); // set device address
	
	while (calibdone == false)
			{

			// read varaibles

			I2C_bus_read (GYRO_XOUT_H , buff, 6 ,i2c_fd);

			rawGyro[0]= ((signed short)(buff[0]*256+buff[1]));
        		rawGyro[1]= ((signed short)(buff[2]*256+buff[3]));
        		rawGyro[2]= ((signed short)(buff[4]*256+buff[5]));	

			gyrox = gyrox + rawGyro[0];
			gyroy = gyroy + rawGyro[1];
			gyroz = gyroz + rawGyro[2];

			counter0 --;

			if (counter0 == 1 ) { calibdone = true;}

			}


			
			gyrox = (gyrox/CALIBLOOP)/65.5;
			gyroy = (gyroy/CALIBLOOP)/65.5;
			gyroz = (gyroz/CALIBLOOP)/65.5;
			printf ("Xo: %.3f Yo: %.3f  Zo: %.3f\n", gyrox, gyroy ,gyroz);


		if (fdata < 0) {
		printf ("error file  : calibAcc.txt");
		return;
	}

		fprintf(fdata, "%.2f\n%.2f\n%.2f\n", gyrox, gyroy ,gyroz);

		fclose(fdata);

}

void calibAcc (unsigned char *buff , int i2c_fd)
	{


	printf ("GYRO CALIBRATION: turn quadcopter around all axes very slowly, press ESC when calibration is done\n");
	getchar();
	int rawAcc[3];

	int c = 0;
	struct termios initial_settings,new_settings;
	tcgetattr(0,&initial_settings);
	new_settings = initial_settings;
	new_settings.c_lflag &= ~ICANON;
  	new_settings.c_lflag &= ~ECHO;
  	new_settings.c_lflag &= ~ISIG;
  	new_settings.c_cc[VMIN] = 0;
  	new_settings.c_cc[VTIME] = 0;

	 tcsetattr(0, TCSANOW, &new_settings);

	bool calibdone = false;
	FILE *fdata;
	fdata = fopen("calibAcc.txt", "w+");
	int xMin =0 , xMax =0 , yMin =0 , yMax =0 , zMin =0 , zMax =0 ;

	i2cSetAddress (MCU6050_ADDR , i2c_fd); // set device address
	
	while (calibdone == false)
			{

			c = getchar();

			if ( c == 27)
			{
				calibdone = true;

			}

			I2C_bus_read (ACCEL_XOUT_H  , buff, 6 ,i2c_fd);
			
			rawAcc[0]= ((signed short)(buff[0]*256+buff[1]));
        		rawAcc[1]= ((signed short)(buff[2]*256+buff[3]));
        		rawAcc[2]= ((signed short)(buff[4]*256+buff[5]));	

			if (rawAcc[0]> xMax)  { xMax = rawAcc[0]; }

			if (rawAcc[0]< xMin)  { xMin = rawAcc[0]; }
	 
			if (rawAcc[1]> yMax)  { yMax = rawAcc[1]; }

			if (rawAcc[1]< yMin)  { yMin = rawAcc[1]; }

			if (rawAcc[2]> zMax)  { zMax = rawAcc[2]; }

			if (rawAcc[2]< zMin)  { zMin = rawAcc[2]; }

		printf ("xMax: %d xMin: %d yMax: %d yMin: %d zMax: %d  zMin: %d\n" ,xMax , xMin ,yMax , yMin ,zMax , zMin );


		usleep(100000);
		}


	if (fdata < 0) {
		printf ("error file  : calibAcc.txt");
		return;
	}

			fprintf(fdata, "%d\n%d\n%d\n%d\n%d\n%d\n",xMax , xMin ,yMax , yMin ,zMax , zMin);

		fclose(fdata);
		tcsetattr(0, TCSANOW, &initial_settings);


	}




void calibMag (unsigned char *buff , int i2c_fd)
	{

	int rawMag[3];



	printf ("MAG CALIBRATION: turn quadcopter around all axes, press ESC when calibration is done\n");
	getchar();

	int c = 0;
	struct termios initial_settings,new_settings;
	tcgetattr(0,&initial_settings);
	new_settings = initial_settings;
	new_settings.c_lflag &= ~ICANON;
  	new_settings.c_lflag &= ~ECHO;
  	new_settings.c_lflag &= ~ISIG;
  	new_settings.c_cc[VMIN] = 0;
  	new_settings.c_cc[VTIME] = 0;

	 tcsetattr(0, TCSANOW, &new_settings);

	bool calibdone = false;
	FILE *fdata;
	fdata = fopen("calibMag.txt", "w+");

	if (fdata < 0) {
			printf ("error opening file  : calibAcc.txt");
			return;
				}

	int xMin =0 , xMax =0 , yMin =0 , yMax =0 , zMin =0 , zMax =0 ;


	i2cSetAddress (HMC5883L_ADDR, i2c_fd); // set device address
	
	while (calibdone == false)
			{

			c = getchar();

			// read key ESC
			if ( c == 27)
			{
				calibdone = true;

			}


			I2C_bus_read (MAG_XOUT_H , buff, 6 , i2c_fd); // read values from reg

			rawMag[0]= ((signed short)(buff[0]*256+buff[1]));
        		rawMag[2]= ((signed short)(buff[2]*256+buff[3]));
        		rawMag[1]= ((signed short)(buff[4]*256+buff[5]));	

			if (rawMag[0]> xMax)  { xMax = rawMag[0]; }

			if (rawMag[0]< xMin)  { xMin = rawMag[0]; }
	 
			if (rawMag[1]> yMax)  { yMax = rawMag[1]; }

			if (rawMag[1]< yMin)  { yMin = rawMag[1]; }

			if (rawMag[2]> zMax)  { zMax = rawMag[2]; }

			if (rawMag[2]< zMin)  { zMin = rawMag[2]; }

		printf ("xMax: %d xMin: %d yMax: %d yMin: %d zMax: %d  zMin: %d\n" ,xMax , xMin ,yMax , yMin ,zMax , zMin );

		usleep(100000);
		} // exit while after ESC pressed
	
		

			fprintf(fdata, "%d\n%d\n%d\n%d\n%d\n%d\n",xMax , xMin ,yMax , yMin ,zMax , zMin);

			fclose(fdata);
			tcsetattr(0, TCSANOW, &initial_settings);

	}




void loadCalibration (calibration *calib )
		{

			FILE *fdata , *fdata1 , *fdata2;

			fdata = fopen("calibGyro.txt", "r");

			
				if (fdata != NULL) 
				{
				fscanf(fdata,"%f", &calib->offsetGyroX ); //calib->offsetGyroX, calib->offsetGyroY ,calib->offsetGyroZ
				fscanf(fdata,"%f", &calib->offsetGyroY );
				fscanf(fdata,"%f", &calib->offsetGyroZ);
				}
				else printf ("Error in gyrocal file\n");

			

				fdata1 = fopen("calibAcc.txt", "r");


			
				if (fdata1 != NULL) 
				{
				fscanf(fdata1,"%d", &calib->offsetAccXMax ); //calib->offsetGyroX, calib->offsetGyroY ,calib->offsetGyroZ
				fscanf(fdata1,"%d", &calib->offsetAccXMin );
				fscanf(fdata1,"%d", &calib->offsetAccYMax );
				fscanf(fdata1,"%d", &calib->offsetAccYMin ); //calib->offsetGyroX, calib->offsetGyroY ,calib->offsetGyroZ
				fscanf(fdata1,"%d", &calib->offsetAccZMax );
				fscanf(fdata1,"%d", &calib->offsetAccZMin );
				}
				else printf ("Error in gyrocal file\n");




				fdata2 = fopen("calibMag.txt", "r"); 
				if (fdata1 != NULL) 
				{
				fscanf(fdata2,"%d", &calib->offsetMagXMax ); //calib->offsetGyroX, calib->offsetGyroY ,calib->offsetGyroZ
				fscanf(fdata2,"%d", &calib->offsetMagXMin );
				fscanf(fdata2,"%d", &calib->offsetMagYMax );
				fscanf(fdata2,"%d", &calib->offsetMagYMin ); //calib->offsetGyroX, calib->offsetGyroY ,calib->offsetGyroZ
				fscanf(fdata2,"%d", &calib->offsetMagZMax );
				fscanf(fdata2,"%d", &calib->offsetMagZMin );
				}
				else printf ("Error in gyrocal file\n");

				calib->OffsetAccX = (calib->offsetAccXMin + calib->offsetAccXMax)/2;
				calib->OffsetAccY = (calib->offsetAccYMin + calib->offsetAccYMax)/2;
				calib->OffsetAccZ = (calib->offsetAccZMin + calib->offsetAccZMax)/2;

				calib->OffsetMagX = (calib->offsetMagXMin + calib->offsetMagXMax)/2;
				calib->OffsetMagY = (calib->offsetMagYMin + calib->offsetMagYMax)/2;
				calib->OffsetMagZ = (calib->offsetMagZMin + calib->offsetMagZMax)/2;

				printCalib (*calib);

				fclose(fdata);
				fclose(fdata1);
				fclose(fdata2);

}


void printCalib (calibration c)

	{
		printf ("Loaded Calibration Data:\n");
		printf (" Gyro X: %.3f	",  c.offsetGyroX);
		printf (" Gyro Y: %.3f	",  c.offsetGyroY);
		printf (" Gyro Z: %.3f\n",  c.offsetGyroZ);


		printf (" acc X min: %d	",  c.offsetAccXMin);
		printf (" acc X max: %d\n", c.offsetAccXMax);

		printf (" acc Y min: %d	",  c.offsetAccYMin);
		printf (" acc Y max: %d\n", c.offsetAccYMax);

		printf (" acc Z min: %d	", c.offsetAccZMin);
		printf (" acc Z max: %d\n",c.offsetAccZMax);

		printf ("acc AVGx %d AVGy %d AVG z %d\n", c.OffsetAccX, c.OffsetAccY, c.OffsetAccZ);

		printf (" Mag X min: %d	", c.offsetMagXMin);
		printf (" Mag X max: %d\n", c.offsetMagXMax);

		printf (" Mag Y min: %d	", c.offsetMagYMin);
		printf (" Mag Y max: %d\n", c.offsetMagYMax);
		
		printf (" Mag Z min: %d	", c.offsetMagZMin);
		printf (" Mag Z max: %d\n", c.offsetMagYMax);

		printf ("mag AVGx %d AVGy %d AVG z %d\n", c.OffsetMagX, c.OffsetMagY, c.OffsetMagZ);




	}









