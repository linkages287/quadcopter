#include <stdio.h>
#include <stdlib.h>  
#include <prussdrv.h>
#include <pruss_intc_mapping.h>

#define PRU_NUM 0
#define BASE_FREQ 50000
#define INIT_FREQ 10
#define PRUFILENAME "./pwmv2.bin"
#define DEVICE "/dev/ttyUSB0"

/*
@return initialized PRU with 0 pwm values
*/
void PWM_init();


/*
@return close PRU 
*/
void PWM_close();
