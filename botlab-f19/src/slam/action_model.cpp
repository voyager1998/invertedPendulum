#include <time.h>
#include <cassert>
#include <cmath>
#include <common/angle_functions.hpp>
#include <iostream>
#include <lcmtypes/particle_t.hpp>
#include <slam/action_model.hpp>
using namespace std;

#define K1 1.0
#define K2 0.4

ActionModel::ActionModel(void) {
    // Handle any initialization for your ActionModel
    previous_odo.x = 0.0f;
    previous_odo.y = 0.0f;
    previous_odo.theta = 0.0f;

    de_odo.x = 0.0;
    de_odo.y = 0.0;
    de_odo.theta = 0.0;
}

bool ActionModel::updateAction(const pose_xyt_t& odometry) {
    // Implement code here to compute a new distribution of the motion of the robot
    if (previous_odo.x != odometry.x || previous_odo.y != odometry.y || previous_odo.theta != odometry.theta) {
        de_odo.x = odometry.x - previous_odo.x;
        de_odo.y = odometry.y - previous_odo.y;
        de_odo.theta = odometry.theta - previous_odo.theta;
        de_odo.utime = odometry.utime;

        previous_odo.utime = odometry.utime;
        previous_odo.x = odometry.x;
        previous_odo.y = odometry.y;
        previous_odo.theta = odometry.theta;
        return true;
    }
    return false;
}

particle_t ActionModel::applyAction(const particle_t& sample) {
    // Implement your code for sampling new poses from the distribution computed in updateAction
    // Make sure you create a new valid particle_t. Don't forget to set the new time and new parent_pose.
    particle_t new_sample;
    new_sample.parent_pose.utime = sample.pose.utime;
    new_sample.parent_pose.x = sample.pose.x;
    new_sample.parent_pose.y = sample.pose.y;
    new_sample.parent_pose.theta = sample.pose.theta;

    float delta_S = sqrt(de_odo.x * de_odo.x + de_odo.y * de_odo.y);
    float alpha = atan2(de_odo.y, de_odo.x) - sample.pose.theta;

    srand(time(NULL));
    random_device rd;
    normal_distribution<float> randomTheta1(0.0, K1 * alpha);
    normal_distribution<float> randomS(0.0, K2 * delta_S);
    normal_distribution<float> randomTheta2(0.0, K1 * (de_odo.theta - alpha));

    float e1 = randomTheta1(rd);
    float e2 = randomS(rd);
    float e3 = randomTheta2(rd);

    new_sample.pose.x = sample.pose.x + (delta_S + e2) * cos(sample.pose.theta + alpha + e1);
    new_sample.pose.y = sample.pose.y + (delta_S + e2) * sin(sample.pose.theta + alpha + e1);
    new_sample.pose.theta = wrap_to_pi(sample.pose.theta + de_odo.theta + e1 + e3);

    new_sample.weight = sample.weight;  //TODO: sample.weight? 1/numparticles?
    new_sample.pose.utime = de_odo.utime;
    return new_sample;
}
