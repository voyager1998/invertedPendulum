#ifndef MB_CONTROLLER_H
#define MB_CONTROLLER_H


#include "../mobilebot/mobilebot.h"
#define CFG_PATH "pid.cfg"

int mb_initialize_controller();
int mb_load_controller_config();
int mb_controller_update(mb_state_t* mb_state, mb_setpoints_t* mb_setpoints);
int mb_destroy_controller();

rc_filter_t left_pid;
rc_filter_t right_pid;
rc_filter_t fwd_vel_pid;
rc_filter_t turn_vel_pid;

pid_parameters_t left_pid_params;
pid_parameters_t right_pid_params;
pid_parameters_t fwd_vel_pid_params;
pid_parameters_t turn_vel_pid_params;

#endif

