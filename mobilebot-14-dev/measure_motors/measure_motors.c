/*******************************************************************************
* measure_motors.c
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
#include <rc/encoder.h>
#include <rc/adc.h>
#include <rc/time.h>
#include "../common/mb_motor.h"
#include "../common/mb_defs.h"


float enc2meters = (WHEEL_DIAMETER * M_PI) / (GEAR_RATIO * ENCODER_RES);

void test_speed(float duty, float dtime_s);

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

    if(rc_encoder_init()<0){
        fprintf(stderr,"ERROR: failed to initialze encoders\n");
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
        printf("|  PWM  | L Speed | R Speed |\n");

        for(float duty = 0.05; duty<=1.05; duty+=0.05){
            test_speed(duty, 5.0);
        }

		mb_motor_disable();
		rc_nanosleep(1E9);
	}
	
	// exit cleanly
	mb_motor_cleanup();
	rc_remove_pid_file();   // remove pid file LAST
	return 0;
}

void test_speed(float duty, float dtime_s){
    float left_encoder, right_encoder, left_speed, right_speed;
    rc_encoder_write(1, 0);
    rc_encoder_write(2, 0);
    mb_motor_set(RIGHT_MOTOR, duty);
    mb_motor_set(LEFT_MOTOR, -duty);
    rc_nanosleep((int)(dtime_s * 1E9));
    left_encoder = ENC_LEFT_POL * rc_encoder_read(LEFT_MOTOR);
    right_encoder = ENC_RIGHT_POL * rc_encoder_read(RIGHT_MOTOR);
    left_speed = enc2meters * left_encoder / dtime_s;
    right_speed = enc2meters * right_encoder / dtime_s;
    printf("[ %3.2f, %3.4f, %3.4f ]\n", duty, -left_speed, right_speed);
}