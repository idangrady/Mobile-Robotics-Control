#pragma once

#include <vector>

#include "./3rdparty/rangelibc/includes/RangeLib.h"

#include <emc/io.h>

typedef std::vector<double> Pose;
typedef std::vector<Pose> PoseList;

typedef float measurement;
typedef std::vector<measurement> measurementList;
class World
{
public:
    /// @brief Construct the world model
    /// @param occupancyGrid The occupancy-grid
    /// @param max_range The max range of the raycaster/laser sensor
    /// @param resolution The resolution of the occupancy grid map
    World(const ranges::OMap &occupancyGrid, const double &max_range, const double &resolution);

    /// @brief Predict the measurement given the robot pose and given the grid map of the enviroment
    /// @param robotPose The robot pose for which we want to predict the measurement
    /// @param prediction The resulting prediction
    void predictMeasurement(const Pose &robotPose, measurementList &prediction);

    measurementList predictMeasurement(const Pose &robotPose);

    /// @brief Compute the distance driven since the last measurement
    /// @param odom The latest received odometry information
    /// @return The difference in pose since last measurement {dx, dy, dtheta}
    Pose computeDrivenDistance(emc::OdometryData &odom);

    /// @brief Set the parameters corresponding to the laser, number of rays, angle range, angle increment etc.
    /// @param lasermsg The first received laser message
    /// @param noiseparam The noise parameter which we assume the laser to have
    /// @param subsample The amount of rays skipped in prediction the measurement
    void setlaserparams(const emc::LaserData &lasermsg, const double &noiseparam, int subsample = 15);

    /// @brief Get the x-size of the enviroment
    /// @return _size_x
    double get_size_x() const { return _size_x; }

    /// @brief Get the y-size of the enviroment
    /// @return _size_y
    double get_size_y() const { return _size_y; }

    // Laser Sensor Parameters
    /// @brief Number of rays in a laser message
    int _N_rays;
    /// @brief Angle corresponding to the first item of the laser message
    double _angle_min;
    /// @brief Angle between items of the laser message
    double _angle_inc;
    /// @brief Angle corresponding to the last item of the laser message
    double _angle_max;
    /// @brief The amount of subsampling done for the prediction of the laser message
    int _subsample;
    /// @brief The assumed accuracy of the laser sensor N(0, _measurementNoise)
    double _measurementNoise;

private:
    /// @brief Generate a gaussian noise sample from a distribution N(mu, sigma)
    /// @param mu The mean of the distribution
    /// @param sigma The standard deviation of the distribution
    /// @return The noise sample
    double _get_noise_sample_gaussian(double mu, double sigma);

    /// @brief X size of the enviroment
    double _size_x;
    /// @brief Y size of the enviroment
    double _size_y;
    /// @brief Resolution of the grid map in ppm
    double _resolution;
    /// @brief max range of the raycaster
    double _maxRange;
    /// @brief Instation of class implementing the raycasting algorithm
    ranges::BresenhamsLine _caster;
    /// @brief Occupancy grid map of the world
    ranges::OMap _occupancyGrid;
    /// @brief Random number generator
    std::default_random_engine _generator;
    /// @brief Boolean to determine whether we have initialised the previous_offset
    bool _offset_set;
    /// @brief The last received odometery pose
    std::vector<double> previous_offset;
};
