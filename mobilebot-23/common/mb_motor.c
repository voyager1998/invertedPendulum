/*******************************************************************************
* mb_motors.c
*
* Control up to 2 DC motor drivers
*
*******************************************************************************/
#include <stdio.h>
#include <rc/motor.h>
#include <rc/model.h>
#include <rc/gpio.h>
#include <rc/pwm.h>
#include <rc/adc.h>
#include "mb_motor.h"
#include "mb_defs.h"

// preposessor macros
#define unlikely(x) __builtin_expect (!!(x), 0)

// global initialized flag
static int init_flag = 0;

/*******************************************************************************
* int mb_motor_init()
* 
* initialize mb_motor with default frequency
*******************************************************************************/
int mb_motor_init(){
    
    return mb_motor_init_freq(DEFAULT_PWM_FREQ);

}

/*******************************************************************************
* int mb_motor_init_freq()
* 
* set up pwm channels, gpio assignments and make sure motors are left off.
*******************************************************************************/
int mb_motor_init_freq(int pwm_freq_hz){
    
    if(unlikely(rc_pwm_init(1,pwm_freq_hz))){
        fprintf(stderr,"ERROR in rc_motor_init, failed to initialize pwm subsystem 1\n");
        return -1;
    }

    if(unlikely(rc_gpio_init(MDIR1_CHIP, MDIR1_PIN, GPIOHANDLE_REQUEST_OUTPUT))){
            fprintf(stderr,"ERROR in rc_motor_init, failed to set up gpio %d,%d\n", MDIR1_CHIP, MDIR1_PIN);
            return -1;

    }
    if(unlikely(rc_gpio_init(MDIR2_CHIP, MDIR2_PIN, GPIOHANDLE_REQUEST_OUTPUT))){
            fprintf(stderr,"ERROR in rc_motor_init, failed to set up gpio %d,%d\n", MDIR2_CHIP, MDIR2_PIN);
            return -1;

    }

#ifdef MRC_VERSION_1v3
    if(unlikely(rc_gpio_init(MOT_EN, GPIOHANDLE_REQUEST_OUTPUT))){
            fprintf(stderr,"ERROR in rc_motor_init, failed to set up gpio %d,%d\n", MOT_EN);
            return -1;

    }
#endif

#ifdef MRC_VERSION_2v1
    if(unlikely(rc_gpio_init(MOT_BRAKE, GPIOHANDLE_REQUEST_OUTPUT))){
            fprintf(stderr,"ERROR in rc_motor_init, failed to set up gpio %d,%d\n", MOT_BRAKE_EN);
            return -1;

    }
#endif

    if(rc_adc_init()){
        fprintf(stderr,"ERROR: failed to run rc_init_adc()\n");
        return -1;
    }

    init_flag = 1;
    if(unlikely(mb_motor_set_all(0))){
        fprintf(stderr,"ERROR in mb_motor_init\n");
        init_flag = 0;
        return -1;
    }

#ifdef MRC_VERSION_1v3
    rc_gpio_set_value(MOT_EN, 0);
#endif

#ifdef MRC_VERSION_2v1
    if(unlikely(mb_motor_brake(0))){
        fprintf(stderr,"ERROR in mb_motor_init\n");
        init_flag = 0;
        return -1;
    }
#endif

    init_flag = 1;
    return 0;
}

/*******************************************************************************
* mb_motor_cleanup()
* 
*******************************************************************************/
int mb_motor_cleanup(){
    if(!init_flag) return 0;
    mb_motor_disable();
    rc_pwm_cleanup(1);
#ifdef MRC_VERSION_1v3
    rc_gpio_cleanup(MOT_EN);
#endif
#ifdef MRC_VERSION_2v1
    rc_gpio_cleanup(MOT_BRAKE_EN);
#endif
    rc_gpio_cleanup(MDIR1_CHIP, MDIR1_PIN);
    rc_gpio_cleanup(MDIR2_CHIP, MDIR2_PIN);
    return 0;
}

#ifdef MRC_VERSION_2v1
/*******************************************************************************
* mb_motor_brake()
* 
* allows setting the brake function on the motor drivers
* returns 0 on success, -1 on failure
*******************************************************************************/
int mb_motor_brake(int brake_en){
    
    if(unlikely(!init_flag)){
        fprintf(stderr,"ERROR: trying to enable brake before motors have been initialized\n");
        return -1;
    }
   return rc_gpio_set_value(MOT_BRAKE_EN, brake_en);
}
#endif

/*******************************************************************************
* int mb_disable_motors()
* 
* disables PWM output signals
* returns 0 on success
*******************************************************************************/
int mb_motor_disable(){
    
    if(unlikely(!init_flag)){
        fprintf(stderr,"ERROR: trying to disable motors before motors have been initialized\n");
        return -1;
    }
#ifdef MRC_VERSION_1v3
    rc_gpio_set_value(MOT_EN, 1);
#endif

    return mb_motor_set_all(0.0);
}


/*******************************************************************************
* int mb_motor_set(int motor, double duty)
* 
* set a motor direction and power
* motor is from 1 to 2, duty is from -1.0 to +1.0
* uses the defines in mb_defs.h
* returns 0 on success
*******************************************************************************/
int mb_motor_set(int motor, double duty){
    
    if(unlikely(!init_flag)){
        fprintf(stderr,"ERROR: trying to rc_set_motor_all before they have been initialized\n");
        return -1;
    }

    char pwmch;
    int dir_chip, dir_pin, dir;
    int pwmss = 1;
    
    // check that the duty cycle is within +-1
    if  (duty > 1.0)        duty = 1.0;
    else if (duty <-1.0)    duty =-1.0;

    if(motor == 1) {
        pwmch = 'A'; 
        dir_chip = MDIR1_CHIP; 
        dir_pin = MDIR1_PIN;
        duty = MOT_1_POL * duty;
    }
    else if(motor == 2) {
        pwmch = 'B'; 
        dir_chip = MDIR2_CHIP; 
        dir_pin = MDIR2_PIN;
        duty = MOT_2_POL * duty;
    }
    else {
        fprintf(stderr,"ERROR in mb_motor_set, motor must be 1 or 2");
        return -1;
    }


    // determine the direction pins to H-bridge
    if(duty>=0.0)   dir=1;
    else{           dir=0; duty=-duty;}

    if(unlikely(rc_gpio_set_value(dir_chip,dir_pin, dir))){
        fprintf(stderr,"ERROR in rc_motor_set, failed to write to gpio pin %d,%d\n",dir_chip,dir_pin);
        return -1;
    }

    if(unlikely(rc_pwm_set_duty(pwmss, pwmch, duty))){
        fprintf(stderr,"ERROR in rc_motor_set, failed to write to pwm %d%c\n",pwmss, pwmch);
        return -1;
    }

    return 0;
}

/*******************************************************************************
* int mb_motor_set_all(double duty)
* 
* applies the same duty cycle argument to both motors
*******************************************************************************/
int mb_motor_set_all(double duty){
    
    int rtn;

    if(unlikely(!init_flag)){
        printf("ERROR: trying to rc_set_motor_all before they have been initialized\n");
        return -1;
    }

    rtn = mb_motor_set(1, duty);
    if(!rtn) mb_motor_set(2, duty);
    
    return rtn;
}


#ifdef MRC_VERSION_2v1
/*******************************************************************************
* int mb_motor_read_current(int motor)
* 
* returns the measured current in A
*******************************************************************************/
double mb_motor_read_current(int motor){
    //DRV8801 driver board CS pin puts out 500mV/A
    double current = 2.0 * rc_adc_read_volt(motor);
    return current;
}
#endif