#include <stdlib.h>
#include <stdio.h>   /* Standard input/output definitions */
#include <string.h>  /* String function definitions */
#include <unistd.h>  /* UNIX standard function definitions */
#include <fcntl.h>   /* File control definitions */
#include <errno.h>   /* Error number definitions */
#include <termios.h> /* POSIX terminal control definitions */
#include <stdbool.h> /* std bool for boolean result */
#include <ctype.h>   /* is alpha or number */

#define _POSIX_SOURCE 1 /* POSIX compliant source */
#define MIN_SIZE 1
#define DIMB 5
#define BAROBUF 30
#define REFRESH_ATL 9.0f
#define ALTITUDE "/dev/ttyO2"

// connect to SERIAL DEV
int connectSerial(char *);

//send bre alize signal
int bleAliveSignal (int);

//  inizialize barometer 
int initAltimeter (int , float , float , float , int , int);
