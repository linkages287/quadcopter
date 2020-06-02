#include "prulib.h"


void PWM_init()
	{

	char *pru_dataram0;
  	unsigned int percents[4]; // number of motors

	// Initialize structure used by prussdrv_pruintc_intc
	tpruss_intc_initdata pruss_intc_initdata = PRUSS_INTC_INITDATA;
	
	// Allocate and initialize memory
	prussdrv_init ();

	if(prussdrv_open(PRU_EVTOUT_0) != 0)
	{   
		    printf("Failed to open PRUSS driver!\n");
	}		


	// Setting for Map PRU interrupts
	prussdrv_pruintc_init(&pruss_intc_initdata);
	
	// Map PRU DATARAM; reinterpret the pointer type as a pointer to
	// our defined memory mapping struct. We could also use uint8_t *
  	// to access the RAM as a plain array of bytes, or uint32_t * to
  	// access it as words.
  	/* Allocate PRU memory. */
   	if (prussdrv_map_prumem(PRUSS0_PRU0_DATARAM,(void **) &pru_dataram0) == -1) {
      
		printf ("prussdrv_map_prumem PRUSS0_PRU0_DATARAM map failed\n");

   	}

	// Copy the PWM percentage and delay factor into PRU memory
	//unsigned int percents[4];
	
	// set initial freq to 0-----------------------------------------------
	percents[0] = INIT_FREQ; // (0‐100)
	percents[1] = INIT_FREQ; // (0‐100)
	percents[2] = INIT_FREQ; // (0‐100)
	percents[3] = INIT_FREQ; // (0‐100)
	//---------------------------------------------------------------------
	// write into shared RAM
	prussdrv_pru_write_memory(PRUSS0_PRU0_DATARAM, 0, percents, 16);

	// Load and execute binary on PRU -- bynary need to be compiled
	prussdrv_exec_program (PRU_NUM, "./pwmv2.bin");

	} // end PWM init

void PWM_close()
	{

	prussdrv_pru_disable(PRU_NUM);

	prussdrv_exit ();
		
	printf ("EXITing PWM signal\n");
	} // end second thread
