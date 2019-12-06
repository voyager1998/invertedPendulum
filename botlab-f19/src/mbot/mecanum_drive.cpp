#include <mbot/mbot_channels.h>
#include <common/timestamp.h>
#include <lcmtypes/mbot_motor_command_t.hpp>
#include <lcmtypes/odometry_t.hpp>
#include <lcmtypes/pose_xyt_t.hpp>
#include <lcmtypes/robot_path_t.hpp>
#include <lcmtypes/timestamp_t.hpp>
#include <lcmtypes/message_received_t.hpp>
#include <lcmtypes/mbot_imu_t.hpp>
#include <lcmtypes/camera_pose_xy_t.hpp>
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

#define KP_x 0.02f
#define KI_x 0.01f
#define KD_x 0.0f

#define KP_y 0.02f
#define KI_y 0.01f
#define KD_y 0.0f

#define KPV 30.0f

#define EQUIBANGLE 86.0f

#define UPDATETIME 50
// #define DT 0.02f

// #define USEIMU

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
            //cout << "delta t = " << dt << endl;
            intergral += current_error * dt;
            //cout << "intergral: " << intergral << endl;
            if (isPrevErrorInitialize) {
                if (dt > 0.01){
                    derivative = (current_error - previous_error) / dt;
                }
            } else {
                isPrevErrorInitialize = true;
            }
            //cout << "derivative: " << derivative << endl;
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
    MecanumDrive(lcm::LCM* instance) : lcmInstance(instance), pid_x(KP_x, KI_x, KD_x), pid_y(KP_y, KI_y, KD_y), equib_theta(90), frameID(0) {
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
        cmd.trans_v = Vx - Vy;
        cmd.angular_v = 0.0;
        return cmd;
    }

    mbot_motor_command_t updateCommand23(void) {
        mbot_motor_command_t cmd;
        cmd.utime = now();
        cmd.trans_v = Vx + Vy;
        cmd.angular_v = 0.0;
        return cmd;
    }

    bool timesync_initialized() { return timesync_initialized_; }

    void handleTimesync(const lcm::ReceiveBuffer* buf, const std::string& channel, const timestamp_t* timesync){
        timesync_initialized_ = true;
        time_offset = timesync->utime - utime_now();
    }

/*
    void handleVelocity(int64_t time, float theta, float v) {  // receive current v and dir from encoder
        curr_timestamp = time;
        last_dir = theta;
        last_velocity = v;
    }
*/

    void handleIMU(const lcm::ReceiveBuffer* buf, const std::string& channel, const mbot_imu_t* imu_data) {
        pendulum_theta = imu_data->tb_angles[0] * 180 / M_PI;
        printf("pendulum angle: %f\n", pendulum_theta);
    }

    void handleCameraPose(const lcm::ReceiveBuffer* buf, const std::string& channel, const camera_pose_xy_t* camera_pose){
        camera_x = camera_pose->x;
        camera_y = camera_pose->y;
        cout << "frame " << frameID <<endl;
        frameID++;
        cout << "camera_x: " << camera_x << "   camera_y: " << camera_y << std::endl;
    }

    void handleDrive(/*float tar_dir, float tar_acc*/){
        // the x direction of car = y direction of camera !!!
        float error_x = equib_y - camera_y;
        error_x -= Vx * KPV;
        Vx = pid_x.update(error_x, now());
        float error_y = equib_x - camera_x;
        error_y -= Vy * KPV;
        Vy = pid_y.update(error_y, now());
        if (fabs(Vx > 1.0f) || fabs(Vy > 1.0f)){
            Vx = 0.0f;
            Vy = 0.0f;
        }
    }

    void loadEquibTheta(){
        cout << "input equib angle: " << endl;
        std::cin >> equib_theta;
    }
   
    void loadEquibPose(){
        cout << "inpute equib pose_x: " << endl;
        std::cin >> equib_x;
        cout << "inpute equib pose_y: " << endl;
        std::cin >> equib_y;
    }

private:
    message_received_t confirm;
    lcm::LCM* lcmInstance;

    float direction; // see proposal for theta definition
    float acceleration;
    float last_pwm;

    float Vx;
    float Vy;

    int frameID;
    float camera_x;
    float camera_y;

    int64_t curr_timestamp;
    float last_dir;
    float last_velocity;

    pid_controller pid_x;
    pid_controller pid_y;

    float pendulum_theta;  // angle between pendulum and vertical line
    float pendulum_alpha;  // falling direction of pendulum

    float equib_theta;
    float equib_x;
    float equib_y;

    int64_t time_offset;

    bool timesync_initialized_;

   
    int64_t now() {
        return utime_now() + time_offset;
    }
};

int main(int argc, char** argv) {
    lcm::LCM lcmInstance(MULTICAST_URL);

    MecanumDrive controller(&lcmInstance);
    controller.loadEquibPose();
    lcmInstance.subscribe(MBOT_TIMESYNC_CHANNEL, &MecanumDrive::handleTimesync, &controller);
#ifdef USEIMU
    lcmInstance.subscribe(PENDULUM_IMU_CHANNEL, &MecanumDrive::handleIMU, &controller);
#else
    lcmInstance.subscribe(CAMERA_POSE_CHANNEL, &MecanumDrive::handleCameraPose, &controller);
#endif

    signal(SIGINT, exit);
   
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
