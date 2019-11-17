/*******************************************************************************
* mb_odometry.c
*
* TODO: Implement these functions to add odometry and dead rekoning 
*
*******************************************************************************/

#include "../mobilebot/mobilebot.h"
#include "mb_defs.h"
#include <math.h>

#define PI 3.14159265358979323846

#define DTHETA_THRESH 0.001f

/*******************************************************************************
* mb_initialize_odometry() 
*
* TODO: initialize odometry
* NOTE: you should initialize from Optitrack data if available
*
*******************************************************************************/
void mb_initialize_odometry(mb_odometry_t* mb_odometry, float x, float y, float theta){
    mb_odometry->x = x;
    mb_odometry->y = y;
    mb_odometry->theta = theta;
}


/*******************************************************************************
* mb_update_odometry() 
*
* TODO: calculate odometry from internal variables
*       publish new odometry to lcm ODOMETRY_CHANNEL
*
*******************************************************************************/
void mb_update_odometry(mb_odometry_t* mb_odometry, mb_state_t* mb_state){

    float enc2meters = (WHEEL_DIAMETER * PI) / (GEAR_RATIO * ENCODER_RES);
    float dx = enc2meters*(mb_state->left_encoder + mb_state->right_encoder) / 2;
    
    //perform gyrodometry
    float dtheta_odo = enc2meters*(mb_state->right_encoder - mb_state->left_encoder) / WHEEL_BASE;
    
    float dtheta_imu = mb_angle_diff_radians(mb_state->last_yaw,mb_state->tb_angles[2]);
    float dtheta_GO = dtheta_imu - dtheta_odo;
    float dtheta = 0.0;
    //printf("GO: %f\t", dtheta_GO);
    if(fabs(dtheta_GO) > DTHETA_THRESH){
        dtheta = dtheta_imu;
        //printf("imu: %f\n", dtheta_imu);
    }
    else {
        dtheta = dtheta_odo;
        //printf("odo: %f\n", dtheta_odo);
    }


    //calculate velocities
    mb_state -> turn_velocity = dtheta / DT;
    mb_state -> fwd_velocity = dx / DT;
    mb_state -> left_velocity = enc2meters * mb_state->left_encoder / DT;
    mb_state -> right_velocity = enc2meters * mb_state->right_encoder / DT;

    //update odometry
    mb_odometry->x += dx * cos(mb_odometry->theta + dtheta/2.0f);
    mb_odometry->y += dx * sin(mb_odometry->theta + dtheta/2.0f);
    mb_odometry->theta = mb_clamp_radians(mb_odometry->theta + dtheta);

}


/*******************************************************************************
* mb_clamp_radians() 
*******************************************************************************/
float mb_clamp_radians(float angle){

    if(angle < -PI)
    {
        for(; angle < -PI; angle += 2.0*PI);
    }
    else if(angle > PI)
    {
        for(; angle > PI; angle -= 2.0*PI);
    }

    return angle;
}

float mb_angle_diff_radians(float angle1, float angle2){
    float diff = angle2 - angle1;
    while(diff < -PI) diff+=2.0*PI;
    while(diff > PI) diff-=2.0*PI;
    return diff;
}