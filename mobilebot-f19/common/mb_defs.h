/*******************************************************************************
* mb_defs.h
*
*   defines for your bot
*   You will need to fill this in based on the data sheets, schematics, etc. 
*      and your specific configuration...
* 
*******************************************************************************/
#ifndef MB_DEFS_H
#define MB_DEFS_H

#define MRC_VERSION_1v3
//#define MRC_VERSION_2v1
#define DEFAULT_PWM_FREQ        25000 // period of motor drive pwm
#define LEFT_MOTOR              2     // id of left motor
#define RIGHT_MOTOR             1     // id of right motor
#define MDIR1_CHIP              1
#define MDIR1_PIN               28  //gpio1.28  P9.12
#define MDIR2_CHIP              1
#define MDIR2_PIN               16  //gpio1.16  P9.15
#ifdef MRC_VERSION_2v1
#define MOT_BRAKE_EN            0,20    // gpio0.20  P9.41
#endif
#ifdef MRC_VERSION_1v3
#define MOT_EN                  0,20    // gpio0.20  P9.41
#endif
#define MOT_1_POL            -1    // polarity of motor 1
#define MOT_2_POL          1    // polarity of motor 2
#define ENC_LEFT_POL           -1    // polarity of encoder 1
#define ENC_RIGHT_POL          1    // polarity of encoder 2
#define MOT_1_CS                0    // analog in of motor 1 current sense
#define MOT_2_CS                1    // analog in of motor 2 current sense
#define GEAR_RATIO              34.0  // gear ratio of motor
#define ENCODER_RES             48.0  // encoder counts per motor shaft revolution
#define WHEEL_DIAMETER          0.08 // diameter of wheel in meters
#define WHEEL_BASE              0.15  // wheel separation distance in meters
#define FWD_VEL_SENSITIVITY     0.75   // sensitivity of RC control for moving
#define TURN_VEL_SENSITIVITY    0.25   // sensitivity of RC control for turning
#define MAX_FWD_VEL             0.8   // maximum forwad speed (m/s)
#define MAX_TURN_VEL            2.5   // maximum turning speed (rad/s)

#define SAMPLE_RATE_HZ          20   // main filter and control loop speed
#define DT                      0.05  // 1/sample_rate
#define PRINTF_HZ               10    // rate of print loop
#define RC_CTL_HZ               25    // rate of RC data update
#define LCM_HZ                  100    // rate of LCM subscribe
#define LCM_PRIORITY            60    // priority of LCM thread (lower is less critical)
#define SETPOINT_PRIORITY       30    // priority of setpoint thread (lower is less critical)
#define CONTROLLER_PRIORITY     90    // priority of controller (lower is less critical)
#define LED_OFF                 1
#define LED_ON                  0
// LCM Channel Names
#define OPTITRACK_CHANNEL           "TRUE_POSE"
#define ODOMETRY_CHANNEL            "ODOMETRY"
#define RESET_ODOMETRY_CHANNEL      "RESET_ODOMETRY"
#define CONTROLLER_PATH_CHANNEL     "CONTROLLER_PATH"
#define MBOT_IMU_CHANNEL            "MBOT_IMU"
#define MBOT_ENCODER_CHANNEL        "MBOT_ENCODERS"
#define MBOT_MOTOR_COMMAND_CHANNEL  "MBOT_MOTOR_COMMAND"
#define MBOT_TIMESYNC_CHANNEL       "MBOT_TIMESYNC"
#define LCM_ADDRESS                 "udpm://239.255.76.67:7667?ttl=2"

#endif
