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

#include <lcmtypes/mbot_command_t.hpp>
#include <lcmtypes/mbot_status_t.hpp>

using std::cout;
using std::endl;

#define FUTURE 50000

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
    }

    mbot_motor_command_t updateCommand14(void) {
        mbot_motor_command_t cmd;
        cmd.trans_v = 0.1f;
        cmd.angular_v = 0.0f;

        cmd.utime = now() + ;

        return cmd;
    }
    mbot_motor_command_t updateCommand23(void) {
        mbot_motor_command_t cmd;

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
    }

private:
    double direction; // see proposal for theta definition
    double acceleration;

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

    while (true) {
        lcmInstance.handleTimeout(50);  // update at 20Hz minimum

        if (controller.timesync_initialized()) {
            mbot_motor_command_t cmd14 = controller.updateCommand14();
            lcmInstance.publish(MBOT_MOTOR_COMMAND_CHANNEL, &cmd14);
            mbot_motor_command_t cmd23 = controller.updateCommand23();
            lcmInstance.publish(MBOT_MOTOR_COMMAND_CHANNEL, &cmd23);
        }
    }

    return 0;
}
