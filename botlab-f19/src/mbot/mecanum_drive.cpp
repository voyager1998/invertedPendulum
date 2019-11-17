#include <mbot/mbot_channels.h>
#include <common/timestamp.h>
#include <lcmtypes/mbot_motor_command_t.hpp>
#include <lcmtypes/odometry_t.hpp>
#include <lcmtypes/pose_xyt_t.hpp>
#include <lcmtypes/robot_path_t.hpp>
#include <lcmtypes/timestamp_t.hpp>
#include <lcmtypes/message_received_t.hpp>
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

#define FUTURE 0

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

class MecanumDrive {
public:
    MecanumDrive(lcm::LCM* instance) : lcmInstance(instance) {
        ////////// TODO: Initialize your controller state //////////////

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
    }

    mbot_motor_command_t updateCommand14(void) {
        mbot_motor_command_t cmd;
        cmd.utime = now();
        float deltaV = (cmd.utime - curr_timestamp) / 1000000.0 * acceleration;
        curr_timestamp = cmd.utime;
        // float newVx = last_velocity * std::cos(last_dir) + deltaV * std::cos(direction);
        // float newVy = last_velocity * std::sin(last_dir) + deltaV * std::sin(direction);
        // last_dir = atan2(newVy, newVx);
        // last_velocity = sqrt(newVx * newVx + newVy * newVy);
        Vx = Vx + deltaV * std::cos(direction);
        Vy = Vy + deltaV * std::sin(direction);
        cout << "New velocity: " << Vx << ", " << Vy << endl;

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

    void handleVelocity(int64_t time, double theta, double v) {  // receive current v and dir from encoder
        curr_timestamp = time;
        last_dir = theta;
        last_velocity = v;
    }

    void handleDrive(double tar_dir, double tar_acc){
        direction = tar_dir;
        acceleration = tar_acc;
        curr_timestamp = now();
    }

private:
    double direction; // see proposal for theta definition
    double acceleration;

    double Vx;
    double Vy;

    int64_t curr_timestamp;
    double last_dir;
    double last_velocity;

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
    lcmInstance.subscribe(MBOT_TIMESYNC_CHANNEL, &MecanumDrive::handleTimesync, &controller);

    signal(SIGINT, exit);

    controller.handleDrive(M_PI/2.0, 0.05);

    double theta = 0.0;
    int a = 0.0;
    while (true) {
        lcmInstance.handleTimeout(10);  // update at 100Hz minimum
        // theta += 0.00001;
        // a = (a + 1) % 2000000;
        // controller.handleDrive(theta, ((double)a - 1000000.0) / 2000000.0);

        if (controller.timesync_initialized()) {
            mbot_motor_command_t cmd14 = controller.updateCommand14();
            lcmInstance.publish(MBOT_MOTOR_COMMAND_CHANNEL_14, &cmd14);
            mbot_motor_command_t cmd23 = controller.updateCommand23();
            lcmInstance.publish(MBOT_MOTOR_COMMAND_CHANNEL_23, &cmd23);
        }
    }

    return 0;
}
