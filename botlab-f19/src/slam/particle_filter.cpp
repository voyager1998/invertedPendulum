#include <time.h>
#include <cassert>
#include <lcmtypes/pose_xyt_t.hpp>
#include <slam/occupancy_grid.hpp>
#include <slam/particle_filter.hpp>
#include <common/angle_functions.hpp>

#define RESAMPLEPORTION 0.8
#define ITERATIONS 10
#define STARTITER 4
// #define USEGAUSSIAN
// #define USETRI
// #define USEITERATIVE
// #define RANDOMINIT
using namespace std;

int sampleFromWeightBar(float sample, std::vector<float> weight_bar) {
    int index = 0;
    while(sample > weight_bar[index]){
        index++;
    }
    return index;
}

ParticleFilter::ParticleFilter(int numParticles)
    : kNumParticles_(numParticles) {
    assert(kNumParticles_ > 1);
    posterior_.resize(kNumParticles_);
    posteriorPose_.x = posteriorPose_.y = posteriorPose_.theta = 0.0;
}

void ParticleFilter::initializeFilterAtPose(const pose_xyt_t& pose, const OccupancyGrid& map) {
    // TODO: Implement your method for initializing the particles in the particle filter
#ifdef RANDOMINIT
    float map_width = map.widthInMeters();
    float map_height = map.heightInMeters();
    srand(time(NULL));
    random_device rd;
    normal_distribution<float> randomXY(0.0, min(map_width, map_height) / 200.0);
    normal_distribution<float> randomTheta(0.0, M_PI / 18.0);
    for (int i = 0; i < kNumParticles_; i++) {
        posterior_[i].pose.x = pose.x + randomXY(rd);
        posterior_[i].pose.y = pose.y + randomXY(rd);
        posterior_[i].pose.theta = wrap_to_pi(pose.theta + randomTheta(rd));
        posterior_[i].weight = 1.0 / (float)kNumParticles_;
    }
#else
    for (int i = 0; i < kNumParticles_; i++) {
        posterior_[i].pose.x = pose.x;
        posterior_[i].pose.y = pose.y;
        posterior_[i].pose.theta =pose.theta;
        posterior_[i].weight = 1.0 / (float)kNumParticles_;
    }
#endif
}

pose_xyt_t ParticleFilter::updateFilter(const pose_xyt_t& odometry,
                                        const lidar_t& laser,
                                        const OccupancyGrid& map) {
    // Only update the particles if motion was detected. If the robot didn't move, then
    // obviously don't do anything.
    bool hasRobotMoved = actionModel_.updateAction(odometry);

    if (hasRobotMoved) {
        // cout << "--------------------------------------" << endl;
#ifndef USEITERATIVE
        auto prior = resamplePosteriorDistribution();
        auto proposal = computeProposalDistribution(prior);
        posterior_ = computeNormalizedPosterior(proposal, laser, map);
#else
        posterior_ = PosteriorGenerater(laser, map);
#endif
        posteriorPose_ = estimatePosteriorPose(posterior_);
    }

    posteriorPose_.utime = odometry.utime;

    return posteriorPose_;
}

pose_xyt_t ParticleFilter::poseEstimate(void) const {
    return posteriorPose_;
}

particles_t ParticleFilter::particles(void) const {
    particles_t particles;
    particles.num_particles = posterior_.size();
    particles.particles = posterior_;
    return particles;
}

std::vector<particle_t> ParticleFilter::resamplePosteriorDistribution(void) {
    // TODO: Implement your algorithm for resampling from the posterior distribution
    std::vector<particle_t> prior;
    prior.resize(kNumParticles_);

    std::vector<float> weight_bar;
    float sum = 0.0;
    for (int i = 0; i < kNumParticles_; i++) {
        sum += posterior_[i].weight;
        weight_bar.push_back(sum);
    }
    if (abs(sum - 1.0) > 0.01) std::cout << "sum not equal to 1" << std::endl;

    srand(time(NULL));
    random_device rd;

    // sample from weight bar
    uniform_real_distribution<float> random(0.0, sum);
    for (int i = 0; i < RESAMPLEPORTION * kNumParticles_; i++) {
        float sample = random(rd);
        int index = sampleFromWeightBar(sample, weight_bar);
        particle_t temp = posterior_[index];
        // if (temp.weight < 0.2) cout << "Get particle with small weight = " << temp.weight << endl;
        temp.weight = 1.0 / (float)kNumParticles_;  // gaussian???
        prior[i] = temp;
    }

    // add random particles around posteriorPose_
    normal_distribution<float> randomXY(0.0, 0.05);
    normal_distribution<float> randomTheta(0.0, M_PI / 36.0);

    for (int i = RESAMPLEPORTION * kNumParticles_; i < kNumParticles_; i++){
        particle_t temp;
        temp.parent_pose = posteriorPose_;
        temp.pose = posteriorPose_;
        temp.pose.x += randomXY(rd);
        temp.pose.y += randomXY(rd);
        temp.pose.theta = wrap_to_pi(randomTheta(rd) + temp.pose.theta);
        temp.weight = 1.0 / (float)kNumParticles_;  // gaussian???
        prior[i] = temp;
    }
    return prior;
}

std::vector<particle_t> ParticleFilter::computeProposalDistribution(const std::vector<particle_t>& prior) {
    // TODO: Implement your algorithm for creating the proposal distribution by sampling from the ActionModel
    std::vector<particle_t> proposal;
    for (auto i : prior) {
        proposal.push_back(actionModel_.applyAction(i));
    }
    return proposal;
}

std::vector<particle_t> ParticleFilter::computeNormalizedPosterior(const std::vector<particle_t>& proposal,
                                                                   const lidar_t& laser,
                                                                   const OccupancyGrid& map) {
    // TODO: Implement your algorithm for computing the normalized posterior distribution using the
    //       particles in the proposal distribution
    std::vector<particle_t> posterior;
    vector<double> logPs;
    for (auto i : proposal) {
#if defined(USEGAUSSIAN) || defined(USETRI)
#ifdef USEGAUSSIAN
        double logP = sensorModel_.Gaussianlikelihood(i, laser, map);
#endif
#ifdef USETRI
        double logP = sensorModel_.Trilikelihood(i, laser, map);
#endif
#else
        double logP = sensorModel_.likelihood(i, laser, map);
#endif
        logPs.push_back(logP);
    }
    double logPmax = *max_element(logPs.begin(), logPs.end());
    // cout << "log(P).max = " << logPmax << endl;
    // cout << "Corresponding Gaussian P = " << 
    // sensorModel_.Gaussianlikelihood(proposal[max_element(logPs.begin(), logPs.end()) - logPs.begin()], laser, map) << endl;
    // cout << "lidar_size = " << laser.num_ranges << endl;

    vector<double>
        shifted_P;
    double sum_shifted_P = 0.0;
    int highProbParticles = 0;
    for (size_t i = 0; i < logPs.size(); i++) {
        logPs[i] += -logPmax;
        double temp = exp(logPs[i]);
        if (temp > 0.1) highProbParticles++;
        shifted_P.push_back(temp);
        sum_shifted_P += temp;
    }
    // cout << "Num of Particles with high prob: " << highProbParticles << endl;
    for (size_t i = 0; i < shifted_P.size(); i++) {
        particle_t temp = proposal[i];
        temp.weight = shifted_P[i] / sum_shifted_P;
        // if (temp.weight > 0) cout << "weight = " << temp.weight << endl;
        posterior.push_back(temp);
    }
    return posterior;
}

pose_xyt_t ParticleFilter::estimatePosteriorPose(const std::vector<particle_t>& posterior) {  
    // mean pose or maximun weight pose
    // TODO: Implement your method for computing the final pose estimate based on the posterior distribution
    pose_xyt_t pose;
    pose.x = 0;
    pose.y = 0;
    pose.theta = 0;
    pose.utime = posterior[0].pose.utime;

    // max weight:
    // float max_weight = 0.0;
    // for (auto i : posterior) {
    //     if (max_weight < i.weight) {
    //         max_weight = i.weight;
    //         pose.x = i.pose.x;
    //         pose.y = i.pose.y;
    //         pose.theta = i.pose.theta;
    //     }
    //     if (max_weight > 0.5) {
    //         cout << "Max weight pose found = " << max_weight << endl;
    //         break;
    //     }
    // }

    // weighted mean:
    for (auto i : posterior) {
        pose.x += i.weight * i.pose.x;
        pose.y += i.weight * i.pose.y;
        pose.theta = wrap_to_pi(pose.theta + i.weight * i.pose.theta);
    }
    // std::cout << "Current Slam pose: " << pose.x << ", " << pose.y << ", " << pose.theta << std::endl;
    return pose;
}

std::vector<particle_t> ParticleFilter::PosteriorGenerater(const lidar_t& laser,
                                                           const OccupancyGrid& map) {
    //------------------Prior-------------------------
    std::vector<particle_t> prior;
    prior.resize(STARTITER * kNumParticles_ / ITERATIONS);

    std::vector<float> weight_bar;
    float sum = 0.0;
    for (int i = 0; i < kNumParticles_; i++) {
        sum += posterior_[i].weight;
        weight_bar.push_back(sum);
    }
    if (abs(sum - 1.0) > 0.01) std::cout << "sum is wrong: " << sum << std::endl;

    srand(time(NULL));
    random_device rd;

    // first sample (STARTITER / ITERATIONS) * kNumParticles_
    // sample from weight bar
    uniform_real_distribution<float> random(0.0, sum);
    for (int i = 0; i < RESAMPLEPORTION * STARTITER * kNumParticles_ / ITERATIONS; i++) { 
        float sample = random(rd);
        int index = sampleFromWeightBar(sample, weight_bar);
        particle_t temp = posterior_[index];
        // if (temp.weight < 0.2) cout << "Get particle with small weight = " << temp.weight << endl;
        temp.weight = 1.0 / kNumParticles_;  // gaussian???
        prior[i] = temp;
    }

    // add random particles around posteriorPose_
    normal_distribution<float> randomXY(0.0, 0.05);
    normal_distribution<float> randomTheta(0.0, M_PI / 36.0);

    for (int i = RESAMPLEPORTION * STARTITER * kNumParticles_ / ITERATIONS; i < STARTITER * kNumParticles_ / ITERATIONS; i++) {
        particle_t temp;
        temp.parent_pose = posteriorPose_;
        temp.pose = posteriorPose_;
        temp.pose.x += randomXY(rd);
        temp.pose.y += randomXY(rd);
        temp.pose.theta = wrap_to_pi(randomTheta(rd) + temp.pose.theta);
        temp.weight = 1.0 / kNumParticles_;  // gaussian???
        prior[i] = temp;
    }
    //------------------Prior-------------------------

    //-----------------First set of Proposal-----------
    std::vector<particle_t> proposal;
    for (auto i : prior) {
        proposal.push_back(actionModel_.applyAction(i));
    }
    //-----------------First set of Proposal-----------

    //----------calculate P and resample----------
    std::vector<particle_t> posterior;
    vector<double> logPs;
    for (auto i : proposal) {
#if defined(USEGAUSSIAN) || defined(USETRI)
#ifdef USEGAUSSIAN
        double logP = sensorModel_.Gaussianlikelihood(i, laser, map);
#endif
#ifdef USETRI
        double logP = sensorModel_.Trilikelihood(i, laser, map);
#endif
#else
        double logP = sensorModel_.likelihood(i, laser, map);
#endif
        logPs.push_back(logP);
    }
    auto resampleCenter = proposal[(max_element(logPs.begin(), logPs.end()) - logPs.begin())];
    //------------resample for (ITERATIONS - STARTITER) times-------
    for (int i = STARTITER; i < ITERATIONS; i++) {
        normal_distribution<float> randomXY(0.0, 0.1 / (i + 1 - STARTITER));
        normal_distribution<float> randomTheta(0.0, M_PI / (18.0 * (i + 1 - STARTITER)));
        for (int k = i * kNumParticles_ / ITERATIONS; k < (i + 1) * kNumParticles_ / ITERATIONS;k++){
            particle_t temp = resampleCenter;
            temp.pose.x += randomXY(rd);
            temp.pose.y += randomXY(rd);
            temp.pose.theta = wrap_to_pi(randomTheta(rd) + temp.pose.theta);
            proposal.push_back(temp);
#if defined(USEGAUSSIAN) || defined(USETRI)
#ifdef USEGAUSSIAN
            double logP = sensorModel_.Gaussianlikelihood(temp, laser, map);
#endif
#ifdef USETRI
            double logP = sensorModel_.Trilikelihood(temp, laser, map);
#endif
#else
            double logP = sensorModel_.likelihood(temp, laser, map);
#endif
            logPs.push_back(logP);
        }
        resampleCenter = proposal[(max_element(logPs.begin(), logPs.end()) - logPs.begin())];
    }
    //--------------------Proposal Generated-------------------------------

    double logPmax = *max_element(logPs.begin(), logPs.end());
    cout << "log(P).max = " << logPmax << endl;

    vector<double> shifted_P;
    double sum_shifted_P = 0.0;
    if (logPs.size() != kNumParticles_) cout << "wrong size: " << logPs.size() << endl;
    int highProbParticles = 0;
    for (int i = 0; i < kNumParticles_; i++) {
        double temp = exp(logPs[i] - logPmax);
        if (temp > 0.1) highProbParticles++;
        shifted_P.push_back(temp);
        sum_shifted_P += temp;
    }
    cout << "Num of Particles with high prob: " << highProbParticles << endl;

    for (int i = 0; i < kNumParticles_; i++) {
        particle_t temp = proposal[i];
        temp.weight = shifted_P[i] / sum_shifted_P;
        // if (temp.weight > 0) cout << "weight = " << temp.weight << endl;
        posterior.push_back(temp);
    }
    return posterior;
}
