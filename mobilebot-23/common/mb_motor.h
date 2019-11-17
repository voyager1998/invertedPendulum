/*******************************************************************************
* mb_motor.h
*******************************************************************************/

#ifndef MB_MOTOR_H
#define MB_MOTOR_H

//fuctions
int mb_motor_init();
int mb_motor_init_freq(int pwm_freq_hz);
int mb_motor_disable();
int mb_motor_brake(int brake_en);
int mb_motor_set(int motor, double duty);
int mb_motor_set_all(double duty);
int mb_motor_cleanup();
double mb_motor_read_current(int motor);

#endif