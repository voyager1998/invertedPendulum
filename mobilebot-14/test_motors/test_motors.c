/*******************************************************************************
* test_motors.c
*
* 
*******************************************************************************/
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include <unistd.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <rc/start_stop.h>
#include <rc/encoder_eqep.h>
#include <rc/adc.h>
#include <rc/time.h>
#include "../common/mb_motor.h"
#include "../common/mb_defs.h"


/*******************************************************************************
* int main() 
*
*******************************************************************************/
int main(){

	// make sure another instance isn't running
    // if return value is -3 then a background process is running with
    // higher privaledges and we couldn't kill it, in which case we should
    // not continue or there may be hardware conflicts. If it returned -4
    // then there was an invalid argument that needs to be fixed.
    if(rc_kill_existing_process(2.0)<-2) return -1;

	// start signal handler so we can exit cleanly
    if(rc_enable_signal_handler()==-1){
                fprintf(stderr,"ERROR: failed to start signal handler\n");
                return -1;
    }

    if(mb_motor_init()<0){
        fprintf(stderr,"ERROR: failed to initialze mb_motors\n");
        return -1;
    }

    // make PID file to indicate your project is running
	// due to the check made on the call to rc_kill_existing_process() above
	// we can be fairly confident there is no PID file already and we can
	// make our own safely.
	rc_make_pid_file();

	// done initializing so set state to RUNNING
	rc_set_state(RUNNING);
    
	if(rc_get_state()==RUNNING){
#ifdef MRC_VERSION_2v1
		mb_motor_brake(1);
#endif
		//run right forward for 1s
        printf("Right FWD\n");
		mb_motor_set(RIGHT_MOTOR, 0.8);
		mb_motor_set(LEFT_MOTOR, 0.0);
		rc_nanosleep(1E9);
		//run left forward for 1s
        printf("Left FWD\n");
		mb_motor_set(RIGHT_MOTOR, 0.0);
		mb_motor_set(LEFT_MOTOR, 0.8);
		rc_nanosleep(1E9);
		//run left backwards for 1s
        printf("Left BKWD\n");
		mb_motor_set(RIGHT_MOTOR, 0.0);
		mb_motor_set(LEFT_MOTOR, -0.8);
		rc_nanosleep(1E9);
		//run right backwards for 1s
        printf("Right BKWD\n");
		mb_motor_set(RIGHT_MOTOR, -0.8);
		mb_motor_set(LEFT_MOTOR, 0.0);
		rc_nanosleep(1E9);
		//set both forwards for 1s
#ifdef MRC_VERSION_2v1
		mb_motor_brake(0);
#endif
        printf("Both FWD\n");
		mb_motor_set_all(0.8);
		rc_nanosleep(1E9);
		//stop motors for 1s
		mb_motor_disable();
		rc_nanosleep(1E9);
	}
	
	// exit cleanly
	mb_motor_cleanup();
	rc_remove_pid_file();   // remove pid file LAST
	return 0;
}