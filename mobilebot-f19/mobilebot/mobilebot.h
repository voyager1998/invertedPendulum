#ifndef MB_H
#define MB_H

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <sys/types.h>
#include <unistd.h>
#include <math.h>
#include <time.h>

#include <rc/adc.h>
#include <rc/button.h>
#include <rc/cpu.h>
#include <rc/dsm.h>
#include <rc/encoder.h>
#include <rc/gpio.h>
#include <rc/i2c.h>
#include <rc/led.h>
#include <rc/math.h>
#include <rc/model.h>
#include <rc/motor.h>
#include <rc/mpu.h>
#include <rc/pru.h>
#include <rc/pthread.h>
#include <rc/start_stop.h>
#include <rc/time.h>
#include <rc/version.h>

#include <lcm/lcm.h>
#include "../lcmtypes/pose_xyt_t.h"
#include "../lcmtypes/mbot_encoder_t.h"
#include "../lcmtypes/mbot_imu_t.h"
#include "../lcmtypes/mbot_motor_command_t.h"
#include "../lcmtypes/odometry_t.h"
#include "../lcmtypes/oled_message_t.h"
#include "../lcmtypes/timestamp_t.h"
#include "../lcmtypes/reset_odometry_t.h"

#include "../common/mb_defs.h"
#include "../common/mb_structs.h"
#include "../common/mb_controller.h"
#include "../common/mb_odometry.h"
#include "../common/mb_motor.h"

// global variables
lcm_t * lcm;
rc_mpu_data_t imu_data;
pthread_mutex_t state_mutex;
mb_state_t mb_state;
mb_setpoints_t mb_setpoints;
mb_odometry_t mb_odometry;
int64_t now;
int64_t time_offset;
int time_offset_initialized;

// functions
void mobilebot_controller();
void update_now();
void read_mb_sensors();
void publish_mb_msgs();
void cleanup_threads();
void led_heartbeat();


//LCM handler functions
void optitrack_message_handler(const lcm_recv_buf_t* rbuf,
                               const char* channel,
                               const pose_xyt_t* pose,
                               void* userdata);

void motor_command_handler(const lcm_recv_buf_t *rbuf, 
                                  const char *channel,
                                  const mbot_motor_command_t *msg, 
                                  void *user);

void timesync_handler(const lcm_recv_buf_t * rbuf, 
                             const char *channel,
                             const timestamp_t *timestamp, 
                             void *_user);

//thread functions
void* setpoint_control_loop(void* ptr);
void* printf_loop(void* ptr);
void* lcm_subscribe_loop(void* ptr);

#endif
