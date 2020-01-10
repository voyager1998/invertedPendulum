/*******************************************************************************
* mb_odometry.h
*
* 
*******************************************************************************/
#ifndef MB_ODOMETRY_H
#define MB_ODOMETRY_H
#include "mb_structs.h"

void mb_initialize_odometry(mb_odometry_t* mb_odometry, float x, float y, float theta);
void mb_update_odometry(mb_odometry_t* mb_odometry, mb_state_t* mb_state);
float mb_clamp_radians(float angle);
float mb_angle_diff_radians(float angle1, float angle2);

#endif