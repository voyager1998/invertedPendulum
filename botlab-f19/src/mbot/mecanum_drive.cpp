#include <mbot/mbot_channels.h>
#include <common/timestamp.h>
#include <lcmtypes/mbot_motor_command_t.hpp>
#include <lcmtypes/odometry_t.hpp>
#include <lcmtypes/pose_xyt_t.hpp>
#include <lcmtypes/robot_path_t.hpp>
#include <lcmtypes/timestamp_t.hpp>
#include <lcmtypes/message_received_t.hpp>
#include <lcmtypes/mbot_imu_t.hpp>
#include <common/angle_functions.hpp>
#include <common/pose_trace.hpp>
#include <common/lcm_config.h>
#include <slam/slam_channels.h>
#include <lcm/lcm-cpp.hpp>
#include <algorithm>
#include <iostream>
#include <cassert>
#include <signal.h>
#include <math.h>

#include <lcmtypes/mbot_command_t.hpp>
#include <lcmtypes/mbot_status_t.hpp>

using std::cout;
using std::endl;

#define USE_DEGREE

#define FUTURE 0
#define MAXSPEED 1.0
#define TESTSPEED

#define KP 0.2f
#define KI 0.15f
#define KD 0.005f
#define KPV 2.25f

#define EQUIBANGLE 86.0f

#define UPDATETIME 50
// #define DT 0.02f

float clamp_speed(float speed)
{
    if(speed < -1.0f)
    {
        return -1.0f;
    }
    else if(speed > 1.0f)
    {
        return 1.0f;
    }
    
    return speed;
}

class pid_controller {
public:
    pid_controller(float p, float i, float d) {
        set_pid(p, i, d);
        intergral = 0.0f;
        previous_error = 0.0f;
        // DT = 1.0f / (float)UPDATETIME;
        cur_error = 0.0f;
        cur_control = 0.0f;
        isPrevErrorInitialize = false;
        isPrevTimeInitialized = false;
    }

    float update(float current_error, int64_t now) {
        cout << "current error: " << current_error << endl;
        float derivative = 0.0f;
        if (isPrevTimeInitialized) {
            float dt = (float)(now - prev_time) / 1000000.0f;
            cout << "delta t = " << dt << endl;
            intergral += current_error * dt;
            cout << "intergral: " << intergral << endl;
            if (isPrevErrorInitialize) {
                if (dt > 0.01){
                    derivative = (current_error - previous_error) / dt;
                }
            } else {
                isPrevErrorInitialize = true;
            }
            cout << "derivative: " << derivative << endl;
        } else {
            isPrevTimeInitialized = true;
        }
        prev_time = now;
        previous_error = current_error;
        float control = -1.0 * ((current_error > 0 ? 1.0f : -1.0f) * kp * sqrt(fabs(current_error)) + ki * intergral + kd * derivative);
        cout << "control: " << control << endl;
        cur_error = current_error;
        cur_control = control;
        return control;
    }

    void set_pid(float p, float i, float d) {
        kp = p;
        ki = i;
        kd = d;
    }
    void reset() {
        intergral = 0.0f;
        previous_error = 0.0f;
    }

private:
    float intergral;
    float previous_error;
    bool isPrevErrorInitialize;

    float cur_error;
    float cur_control;

    float kp, ki, kd;

    int64_t prev_time;
    bool isPrevTimeInitialized;
};

class MecanumDrive {
public:
    MecanumDrive(lcm::LCM* instance) : lcmInstance(instance), pid(KP, KI, KD), equib_theta(90) {
        time_offset = 0;
        timesync_initialized_ = false;

        confirm.utime = 0;
        confirm.creation_time = 0;
        confirm.channel = "";

        direction = 0.0;
        acceleration = 0.0;
        last_dir = 0.0;
        last_velocity = 0.0;

        Vx = 0.0;
        Vy = 0.0;

        curr_timestamp = now();
        last_pwm = 0.0;
    }

    mbot_motor_command_t updateCommand14(void) {
        mbot_motor_command_t cmd;
        cmd.utime = now();
#ifndef TESTSPEED
        float clamp;
        float deltaV = (cmd.utime - curr_timestamp) / 1000000.0 * acceleration;
        curr_timestamp = cmd.utime;
        Vx = Vx + deltaV * std::cos(direction);
        clamp = 0.5 / (std::max(0.5f, fabs(Vx)));
        Vx *= clamp;
        Vy = Vy + deltaV * std::sin(direction);
        clamp = 0.5 / (std::max(0.5f, fabs(Vy)));
        Vy *= clamp;
        cout << "New velocity: " << Vx << ", " << Vy << endl;

        cmd.trans_v = Vx - Vy;
#else
        cmd.trans_v = acceleration;
#endif
        cmd.angular_v = 0.0;
        return cmd;
    }

    mbot_motor_command_t updateCommand23(void) {
        mbot_motor_command_t cmd;
        cmd.utime = now();
#ifdef TESTSPEED
        cmd.trans_v = acceleration;
#else
        cmd.trans_v = Vx + Vy;
#endif

        cmd.angular_v = 0.0;
        return cmd;
    }

    bool timesync_initialized() { return timesync_initialized_; }

    void handleTimesync(const lcm::ReceiveBuffer* buf, const std::string& channel, const timestamp_t* timesync){
        timesync_initialized_ = true;
        time_offset = timesync->utime - utime_now();
    }

    void handleIMU(const lcm::ReceiveBuffer* buf, const std::string& channel, const mbot_imu_t* imu_data) {
        // float accelerometerAngle = atan2f((float)imu_data->accel[1], (float)imu_data->accel[2]) * 180 / 3.1415;
        // float tbAngle = imu_data->tb_angles[0] * 180 / 3.1415;
        // if (fabs(tbAngle) < 3) {
        //     pendulum_theta = accelerometerAngle;
        // } else {
        //     pendulum_theta = tbAngle;
        // }
        pendulum_theta = imu_data->tb_angles[0] * 180 / M_PI;
        printf("pendulum angle: %f\n", pendulum_theta);
    }
/*
    void handleVelocity(int64_t time, float theta, float v) {  // receive current v and dir from encoder
        curr_timestamp = time;
        last_dir = theta;
        last_velocity = v;
    }
*/
    void handleDrive(/*float tar_dir, float tar_acc*/){
        direction = 0;
        float error = equib_theta - pendulum_theta;
        // if (fabs(error)<0.1){
        //     error = 0.0;
        //     // pid.reset();
        // }
        error -= last_pwm * KPV;
        acceleration = pid.update(error, now());
        last_pwm = acceleration;
        // if(acceleration > 0){
        //     acceleration = std::max(acceleration, 0.1f);
        // }else{
        //     acceleration = std::min(acceleration, -0.1f);
        // }

        // if (fabs(acceleration) > 10.0) {
        //     acceleration = 0;
        //     // pid.reset();
        // }

        // printf("new acceleration: %f\n", acceleration);
        // curr_timestamp = now();
    }

    void loadEquibTheta(){
        cout << "input equib angle: " << endl;
        std::cin >> equib_theta;
    }

private:
    float direction; // see proposal for theta definition
    float acceleration;
    float last_pwm;

    float Vx;
    float Vy;

    int64_t curr_timestamp;
    float last_dir;
    float last_velocity;

    float pendulum_theta;  // angle between pendulum and vertical line
    float pendulum_alpha;  // falling direction of pendulum
    float equib_theta;

    pid_controller pid;

    int64_t time_offset;

    bool timesync_initialized_;

    message_received_t confirm;
    lcm::LCM* lcmInstance;

    int64_t now() {
        return utime_now() + time_offset;
    }
};

int main(int argc, char** argv) {
    lcm::LCM lcmInstance(MULTICAST_URL);

    MecanumDrive controller(&lcmInstance);
    controller.loadEquibTheta();
    lcmInstance.subscribe(MBOT_TIMESYNC_CHANNEL, &MecanumDrive::handleTimesync, &controller);
    lcmInstance.subscribe(PENDULUM_IMU_CHANNEL, &MecanumDrive::handleIMU, &controller);

    signal(SIGINT, exit);

    // controller.handleDrive(M_PI/4.0, 0.01);
   
    while (true) {
        lcmInstance.handleTimeout(UPDATETIME);
        controller.handleDrive();
        if (controller.timesync_initialized()) {
            mbot_motor_command_t cmd14 = controller.updateCommand14();
            lcmInstance.publish(MBOT_MOTOR_COMMAND_CHANNEL_14, &cmd14);
            mbot_motor_command_t cmd23 = controller.updateCommand23();
            lcmInstance.publish(MBOT_MOTOR_COMMAND_CHANNEL_23, &cmd23);
        }
    }

    return 0;
}
