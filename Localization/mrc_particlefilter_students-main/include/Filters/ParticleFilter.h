
#pragma once

#include "./Filters/ParticleFilterBase.h"
#include "./FilterComponents/Resampler.h"

// Implementation of an Conventional Particle Filter using two types of resampling and two types of ressampling scheme
// For a theoretical background the reader is reffered to the original paper on which this code is based:
//
// Elfring J, Torta E, van de Molengraft R. Particle Filters: A Hands-On Tutorial. Sensors. 2021; 21(2):438.

enum resamplingScheme
{
    Always,
    everyNtimesteps,
    effectiveParticleThreshold,
    Never
};

class ParticleFilter : public ParticleFilterBase
{
public:
    /// @brief Constructor to create an initialised instatiation of the particle filter with
    ///        samples uniformly distributed in the enviroment
    /// @param World World model containing the gridmap and size of the enviroment
    /// @param N Amount of particles at t = 0
    ParticleFilter(const World &World,
                   const int &N) : ParticleFilterBase(World, N) {}

    /// @brief Constructor to create an initialised instatiation of the particle filter with
    ///        samples distributed according to a gaussian with \mu = mean and \sigma = sigma
    /// @param World World model containing the gridmap and size of the enviroment
    /// @param mean The mean of the gaussian distribution
    /// @param sigma The spread of the gaussian distribution
    /// @param N Amount of particles at t = 0

    ParticleFilter(const World &world,
                   const double mean[3],
                   const double sigma[3],
                   const int &N) : ParticleFilterBase(world, mean, sigma, N) {}

    /// @brief Set the resampling scheme and policy
    /// @param algorithm Which type of resampling is used
    /// @param resamplingScheme The policy according to which is decided whether we resample
    /// @param resampleThreshold The threshold parameter of the policy
    void configureResampler(const nlohmann::json &programConfig) override;

    /// @brief Determine whether we need to resample according to the policy and threshold
    /// @return the boolean value signifying the above
    bool needsResampling() const;

    /// @brief Updates the current set of Particles of the Conventional Particle Filter
    ///        The Particles are first propagated according to the received odometry measurement
    ///        Afterwards the weights of the particles are recalculated based on the received laser reading and the known grid map
    ///        Resampling is performed if the condition for resampling is met.
    /// @param dPose The difference between the previous and current odometry measurement (the x,y distance and
    ///              angle driven since t-1)
    /// @param measurement The most up-to-date laser measurement
    /// @param world The world model containing a grid-map representaion of the world
    /// @param odommsg The last received odometry information
    void update(const Pose &dPose,
                const emc::LaserData &measurement,
                World &world,
                const emc::OdometryData &odommsg) override;

    /// @brief Update of the pose of the particles in the filter, except for the resampling.
    /// @param dPose The difference between the previous and current odometry measurement (the x,y distance and angle
    ///              driven since t-1)
    /// @param measurement  The most up-to-date laser measurement
    /// @param world The world model containing a grid-map representaion of the world
    /// @param odommsg The last received odometry information
    void conventionalUpdate(const Pose &dPose,
                            const measurementList &measurement,
                            World &world,
                            const emc::OdometryData &odommsg);

private:
    /// @brief Resampler containing the resampling algorithm used in update
    Resampler _resampler;
    /// @brief  String signifying the resampling policy, element from {Always, effectiveParticleThreshold, everyNtimesteps, Never}
    resamplingScheme _resamplingScheme;
    /// @brief Parameter used by the different resampling schemes, used in {effectiveParticleThreshold, everyNtimesteps}
    Likelihood _resampleThreshold;
};
