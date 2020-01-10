#include "mobilebot.h"
#include "../lcmtypes/equilibrium_angle_t.h"
#define PI 3.14159265359

lcm_t * lcm;
float pendulum_angle;

void imu_message_handler(const lcm_recv_buf_t* rbuf,
			 const char* channel,
			 const mbot_imu_t *msg,
			 void *user)
{
     pendulum_angle = msg->tb_angles[0] * 180 / PI;
     printf("pendulum angle: %f\n", pendulum_angle);
}

int main()
{
    lcm = lcm_create(LCM_ADDRESS);
    //mbot_imu_t_subscribe(lcm,
    //		   	 PENDULUM_IMU_CHANNEL,
    //			 imu_message_handler,
    //			 NULL);
    //while(1)
    //{
    //	lcm_handle_timeout(lcm, 1);
    //	rc_nanosleep(1E9/LCM_HZ);
    //}
    
    float angle;
    scanf("%f", &angle);
    equilibrium_angle_t msg;
    msg.utime = 0;
    msg.equilibrium_angle = angle;
    msg.set_equilibrium = 1;
    equilibrium_angle_t_publish(lcm, "EQUILIBRIUM_ANGLE_CHANNEL", &msg);
    lcm_destroy(lcm);
    return 0;

}
