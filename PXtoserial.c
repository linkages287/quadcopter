#include "PXtoserial.h"
#include <mavlink.h>

int serial_open(char *device)
{

	int fd = 0 ; // file descriptor

	fd = open(device, (O_RDWR | O_NOCTTY | O_NDELAY));

	if (fd == -1)
     	{
	printf ("\nUnable to open the selected port %s\n", device);

	return 0;      
	}	
		else 	{
			printf ("\nDevice %s opened\n",device);
			}

	fcntl(fd, F_SETFL, 0);

	// configuring options
	struct termios options;
	tcgetattr(fd, &options);

	options.c_iflag &= ~(IGNBRK | BRKINT | ICRNL | INLCR | PARMRK | INPCK |
			     ISTRIP | IXON);
	options.c_oflag &= ~(OCRNL | ONLCR | ONLRET | ONOCR | OFILL | OPOST);
	options.c_lflag &= ~(ECHO | ECHONL | ICANON | IEXTEN | ISIG);
	options.c_cflag &= ~(CSIZE | PARENB);
	options.c_cflag |= CS8;

	options.c_cc[VMIN] = 1; /* minimum bytes to return from read() */
	options.c_cc[VTIME] = 10; /* inter-character timer */
	//set trasmission speed
	cfsetispeed(&options, B115200);
	cfsetospeed(&options, B115200);

	tcsetattr(fd, TCSANOW, &options);


	return fd;
}

int serial_write(int fd, uint8_t *buffer, int len)
{
	if (fd<=0) return -1;

	return write(fd, buffer, len);
}

int serial_read(int fd, uint8_t *buffer, int len)
{
	if (fd<=0) return -1;

	return read(fd, buffer, len);
}


bool configure_px4flow(int fd)
{
	uint8_t buffer[64];
	mavlink_message_t msg;
	
	//param config USB_SEND_VIDEO 

	mavlink_msg_param_set_pack(SYS_ID_SELF, COMP_ID_SELF, &msg,
				   SYS_ID_PF4FLOW, COMP_ID_PF4FLOW,
				   "USB_SEND_VIDEO", 0, MAV_PARAM_TYPE_UINT8);

	int len = mavlink_msg_to_send_buffer(buffer, &msg);
	int nwr = serial_write(fd, buffer, len);
	if (nwr != len)
		return false;

	//--------------------------------------------------------------
	// low pass filtering on output 
	mavlink_msg_param_set_pack(SYS_ID_SELF, COMP_ID_SELF, &msg,
				   SYS_ID_PF4FLOW, COMP_ID_PF4FLOW,
				   "BFLOW_LP_FIL", 1, MAV_PARAM_TYPE_UINT8);

	len = mavlink_msg_to_send_buffer(buffer, &msg);
	nwr = serial_write(fd, buffer, len);
	if (nwr != len)
		return false;

	//--------------------------------------------------------------
	// low pass filtering on output 
	mavlink_msg_param_set_pack(SYS_ID_SELF, COMP_ID_SELF, &msg,
				   SYS_ID_PF4FLOW, COMP_ID_PF4FLOW,
				   "SONAR_FILTERED", 1, MAV_PARAM_TYPE_UINT8);

	len = mavlink_msg_to_send_buffer(buffer, &msg);
	nwr = serial_write(fd, buffer, len);
	if (nwr != len)
		return false;

	mavlink_msg_param_set_pack(SYS_ID_SELF, COMP_ID_SELF, &msg,
				   SYS_ID_PF4FLOW, COMP_ID_PF4FLOW,
				   "USB_SEND_GYRO", 0, MAV_PARAM_TYPE_UINT8);

	len = mavlink_msg_to_send_buffer(buffer, &msg);
	nwr = serial_write(fd, buffer, len);
	if (nwr != len)
		return false;

	return true;


}

int readDataPX4 (int fd, struct Quad *quad)
{
	int i = 0; // indexer
	uint8_t buffer[32]; // buffer to save data 
	int nread = 0; // number if byte read	
	mavlink_message_t msg;
	mavlink_optical_flow_t of;
	mavlink_status_t status;
	
	nread = serial_read(fd, buffer, sizeof(buffer));
	
		for (i = 0; i < nread; i++) 
		{
			if (mavlink_parse_char(MAVLINK_COMM_0, buffer[i], &msg,
					       &status)) 
			{
			// check msg id type is optical flow 
			if (msg.msgid == MAVLINK_MSG_ID_OPTICAL_FLOW)

			{
			quad->px4_dtimer = quad->px4_timer;
			mavlink_msg_optical_flow_decode(&msg, &of);
			quad->px4_comp_y = of.flow_comp_m_y;
			quad->px4_comp_x = of.flow_comp_m_x;
			quad->px4_alt = of.ground_distance;
			quad->px4_image_quality =  of.quality;
			quad->px4_timer = of.time_usec;
			quad->px4_dtimer = quad->px4_timer-quad->px4_dtimer;
			quad->counter++; // for filtering purpoise
			return 1;//return data acquired
			}
			}
		}

	return 0; // return non new data acquired

}

	
	// DEBUG SECTION--------------------------------------------------------
	//printf ("Px4 dt %f\n",elapsedTimePX4);
	//printComponents(&Yroll);
	//printf ("DT:%f PYout:%f InputY:%f\n",dt,
	//quad.px4OutY,Yroll.InputPoint);

	//printf (" DT:%f PXo:%f PYo:%f INX:%f INY:%f\n",dt , quad.px4OutX,
	//quad.px4OutY, Xpitch.InputPoint , Yroll.InputPoint );

	//printf ("dt: %f XIN: %f XOUT: %f XP: %f XI: %f\n",dt,
	//Xpitch.InputPoint,
	//quad.px4OutX, Xpitch.errorP , Xpitch.errorI);

	//printf ("dt: %f YIN: %f YOUT: %f YP: %f YI: %f YD: %f\n",dt,
	//Yroll.InputPoint,
	//quad.px4OutY, Yroll.errorP , Yroll.errorI , Yroll.errorD);

	//printf ("dt: %f altIN: %f setpoint: %f altout: %f \n",dt,
	//Alt.InputPoint,
	//Alt.SetPoint,quad.px4OutA);

	//printf ("holdingCode:%d, Xo:%f Yo:%f Ao:%f\n",quad.posMaintain,
	//quad.px4OutX, 	
	//quad.px4OutY, quad.px4OutA);
	//----------------------------------------------------------------------




