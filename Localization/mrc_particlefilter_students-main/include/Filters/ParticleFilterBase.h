#pragma once

#include <vector>
#include <string>
#include <iostream>
#include <algorithm>

#include "./Objects/Robot.h"
#include "./FilterComponents/Particle.h"

#include "./3rdparty/json.hpp"

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/PoseArray.h>
#include <tf2/LinearMath/Quaternion.h>
#include <nav_msgs/MapMetaData.h>

typedef std::vector<Particle> ParticleList;

struct MapConfig
{
    double mapResolution;
    double mapOffsetX;
    double mapOffsetY;
    double mapOrientation;
    bool mapInitialised;
};

class ParticleFilterBase
{
public:
    /// @brief Constructor to create an empty and uninitialised instatiation of the particle filter
    ParticleFilterBase() = default;

    /// @brief Constructor to create an initialised instatiation of the particle filter with
    ///        samples uniformly distributed in the enviroment, weights are
    ///        initialised to be equal for all particles
    /// @param World World model containing the gridmap and size of the enviroment
    /// @param N Amount of particles at t = 0
    ParticleFilterBase(const World &World,
                       const int &N);

    /// @brief Constructor to create an initialised instatiation of the particle filter with
    ///        samples distributed according to a gaussian with \mu = mean and \sigma = sigma
    ///         weights are initialised to be equal for all particles
    /// @param World World model containing the gridmap and size of the enviroment
    /// @param mean The mean of the gaussian distribution
    /// @param sigma The spread of the gaussian distribution
    /// @param N Amount of particles at t = 0
    ParticleFilterBase(const World &World,
                       const double mean[3],
                       const double sigma[3],
                       const int &N);

    /// @brief initialise the publishers used in visualizing the particle filter in RVIZ
    void init_publishers();

    /// @brief Set the measurement model parameters
    /// @param mdl struct containing the measurement model parameters
    void setMeasurementModel(const MeasModelParams &mdl);

    /// @brief Set the process noise level, used in the propagation of particles
    /// @param motion_forward_std stand. dev. of translational movement
    /// @param motion_turn_std stand. dev. of rotational movement
    void setNoiseLevel(double motion_forward_std,
                       double motion_turn_std);

    /// @brief Compute and get the weighted average pose of the filter
    /// @return The average pose
    Pose get_average_state();

    /// @brief Get a list containing all poses of all particles
    /// @return A vector containing the pose of each particle
    PoseList get_PositionList() const;

    /// @brief Propagate particles based on their kinematics, the odometry information and random noise
    /// @param dPose Distance and angle driven since last propagation
    /// @param offset_angle The current angle of the robot, according to the raw odometry information
    void propagateSamples(Pose dPose,
                          const double offset_angle);

    /// @brief Compute the weight of each particle according to the received measurement and the grid map
    /// @param measurement Received laser measurement
    /// @param world World model
    /// @return Vector containing the weight of each particle
    LikelihoodVector computeLikelihoods(const measurementList &measurement,
                                        World &world);

    /// @brief Set the weights of the particles according to the Likelihoodvector
    /// @param likelihoods The vector of likelihoods,
    ///                    indexed equally to the particle vector of the filter
    void set_weights(const LikelihoodVector &likelihoods);

    /// @brief Find the max weight of any particle
    /// @return The max weight of any particle
    double findMaxWeight();

    /// @brief Normalize the summed particle weights to one
    void normaliseWeights();

    /// @brief Get particle that corresponds to idx
    /// @param idx requested index
    /// @return the particle at idx
    Particle getParticle(const int &idx) const;

    /// @brief Updates the current set of Particles of the Conventional Particle Filter
    ///        Not implemented in the Base class of the Particle Filter
    /// @param dPose The difference between the previous and current odometry measurement (the x,y distance and
    ///              angle driven since t-1)
    /// @param measurement The most up-to-date laser measurement
    /// @param world The world model containing a grid-map representaion of the world
    /// @param odommsg The last received odometry information
    virtual void update(const Pose &dPose,
                        const emc::LaserData &measurement,
                        World &world,
                        const emc::OdometryData &odommsg) = 0;

    /// @brief Set the resampling scheme and policy for the conventional or adaptive particle
    ///        filter
    ///        Not implemented in the Base class of the Particle Filter
    /// @param programConfig the configuration of the program containing the required parameters
    virtual void configureResampler(const nlohmann::json &programConfig) = 0;

    /// @brief Get the number of particles in the filter
    /// @return The number of particles
    int getNumberParticles() const;

    /// @brief Recalculate the number of particles in the filter, after changing the amount of particles
    void resetNumberParticles();

    /// @brief Random number generator
    std::default_random_engine _generator;

    /// @brief Vector containing particles
    ParticleList _particles;

    /// @brief Processnoise values [stand. dev. on translation, stand.dev on rotation]
    double _processNoise[2];

    /// @brief amount of timesteps since last resample
    int _resampleCounter = 0;

    /// @brief measurement model paramters
    MeasModelParams _laserModel;

private:
    /// @brief Total number of samples at current timestep.
    int _N = 0;

    /// @brief Pointer to ROS nodehandle
    std::shared_ptr<ros::NodeHandle> _nh;

    /// @brief Publisher to rviz for laser predicition
    ros::Publisher _laser_pub;

    /// @brief Publisher to rviz for particle cloud
    ros::Publisher _particle_pub;

    /// @brief Subscriber to retrieve rviz map metadata
    ros::Subscriber _map_sub;

    /// @brief Publisher to rviz for average pose
    ros::Publisher _pose_pub;

    MapConfig _mapconfig;

    void mapCallback(const nav_msgs::MapMetaData::ConstPtr &msg);

public:
    /// @brief Print the state of all particles
    void printAllParticles() const;

    /// @brief Publish the predicted laser measurement at the mean of the filter to an RVIZ topic
    /// @param world The world model
    void pub_predicted_laser(World world);

    /// @brief Publish the pose of all particles of the filter to an RVIZ topic
    void pub_particles();

    /// @brief Publish the pose of the average pose of the filter to an RVIZ topic
    void pub_average_pose();

    /// @brief Time stamp corresponding to last received information
    double current_timestamp;
};
