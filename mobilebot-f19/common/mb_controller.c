#include "../mobilebot/mobilebot.h"

/*******************************************************************************
* int mb_initialize()
*
* this initializes all the PID controllers from the configuration file
* you can use this as is or modify it if you want a different format
*
* return 0 on success
*
*******************************************************************************/

int mb_initialize_controller(){

    mb_load_controller_config();
    
    left_pid = rc_filter_empty();
    right_pid = rc_filter_empty();
    fwd_vel_pid = rc_filter_empty();
    turn_vel_pid = rc_filter_empty();

    rc_filter_pid(
        &left_pid,
        left_pid_params.kp,
        left_pid_params.ki,
        left_pid_params.kd,
        1.0/left_pid_params.dFilterHz,
        1.0/SAMPLE_RATE_HZ
        );

    rc_filter_pid(
        &right_pid,
        right_pid_params.kp,
        right_pid_params.ki,
        right_pid_params.kd,
        1.0/right_pid_params.dFilterHz,
        1.0/SAMPLE_RATE_HZ
        );
   
    rc_filter_pid(
        &fwd_vel_pid,
        fwd_vel_pid_params.kp,
        fwd_vel_pid_params.ki,
        fwd_vel_pid_params.kd,
        1.0/fwd_vel_pid_params.dFilterHz,
        1.0/SAMPLE_RATE_HZ
        );

    rc_filter_pid(
        &turn_vel_pid,
        turn_vel_pid_params.kp,
        turn_vel_pid_params.ki,
        turn_vel_pid_params.kd,
        1.0/turn_vel_pid_params.dFilterHz,
        1.0/SAMPLE_RATE_HZ
        );

    //saturation
    rc_filter_enable_saturation(&left_pid,-1.0, 1.0);
    rc_filter_enable_saturation(&right_pid,-1.0, 1.0);
    rc_filter_enable_saturation(&fwd_vel_pid,-MAX_FWD_VEL, MAX_FWD_VEL);
    rc_filter_enable_saturation(&turn_vel_pid,-MAX_TURN_VEL, MAX_TURN_VEL);

    return 0;
}

/*******************************************************************************
* int mb_load_controller_config()
*
* this provides a basic configuration load routine
* you can use this as is or modify it if you want a different format
*
* return 0 on success
*
*******************************************************************************/


int mb_load_controller_config(){
    FILE* file = fopen(CFG_PATH, "r");
    if (file == NULL){
        printf("Error opening pid.cfg\n");
    }

    fscanf(file, "%f %f %f %f", 
        &left_pid_params.kp,
        &left_pid_params.ki,
        &left_pid_params.kd,
        &left_pid_params.dFilterHz
        );

    fscanf(file, "%f %f %f %f",
        &right_pid_params.kp,
        &right_pid_params.ki,
        &right_pid_params.kd,
        &right_pid_params.dFilterHz
        );

    fscanf(file, "%f %f %f %f",
        &fwd_vel_pid_params.kp,
        &fwd_vel_pid_params.ki,
        &fwd_vel_pid_params.kd,
        &fwd_vel_pid_params.dFilterHz
        );

    fscanf(file, "%f %f %f %f",
        &turn_vel_pid_params.kp,
        &turn_vel_pid_params.ki,
        &turn_vel_pid_params.kd,
        &turn_vel_pid_params.dFilterHz
        );

    fclose(file);
    return 0;
}

/*******************************************************************************
* int mb_controller_update()
* 
* TODO: Write your cascaded PID controller here
* take inputs from the global mb_state
* write outputs to the global mb_state
*
* return 0 on success
*
*******************************************************************************/

int mb_controller_update(mb_state_t* mb_state, mb_setpoints_t* mb_setpoints){

    //set points for the fwd and turn velocities
    float fwd_sp, turn_sp, left_sp, right_sp;

    //set the set points to 0 if the command is 0 (prevents unwanted movement)
    if(mb_setpoints->fwd_velocity == 0){
        fwd_sp = 0;
    } 
    
    else {
        //set point = desired fwd velocity + PID error
        fwd_sp = mb_setpoints->fwd_velocity + rc_filter_march(&fwd_vel_pid, mb_setpoints->fwd_velocity - mb_state->fwd_velocity);
    }

    if(mb_setpoints->turn_velocity == 0){
        turn_sp = 0;
    } 
    else {
        //set point = desired turn velocity + PID error
        turn_sp = mb_setpoints->turn_velocity + rc_filter_march(&turn_vel_pid, mb_setpoints->turn_velocity - mb_state->turn_velocity);
    }

    //convert forward/turn velocities to L/R wheel velocities
    left_sp = (fwd_sp - turn_sp * WHEEL_BASE/2.0);
    right_sp = (fwd_sp + turn_sp * WHEEL_BASE/2.0);

    //inculdes motor model fit speed = 0.4*pwm + 0.02
    mb_state->left_cmd = (2.5*left_sp + 0.05) + rc_filter_march(&left_pid, left_sp - mb_state->left_velocity);
    mb_state->right_cmd =(2.5*right_sp + 0.05) + rc_filter_march(&right_pid, right_sp - mb_state->right_velocity);

    // set the set points to 0 if the command is 0
    if(left_sp == 0) mb_state->left_cmd = 0;
    if(right_sp == 0) mb_state->right_cmd = 0;
    
    return 0;
}


/*******************************************************************************
* int mb_destroy_controller()
* 
* TODO: Free all resources associated with your controller
*
* return 0 on success
*
*******************************************************************************/

int mb_destroy_controller(){
    rc_filter_free(&fwd_vel_pid);
    rc_filter_free(&turn_vel_pid);
    rc_filter_free(&right_pid);
    rc_filter_free(&left_pid);
    return 0;
}
