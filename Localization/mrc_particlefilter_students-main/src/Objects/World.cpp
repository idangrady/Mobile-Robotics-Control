#include "./Objects/World.h"

#include "tools.h"

#include "./3rdparty/rangelibc/includes/RangeLib.h"

#include <string>
#include <cmath>

World::World(const ranges::OMap &occupancyGrid, const double &max_range, const double &resolution) : _maxRange(max_range),
                                                                                                     _resolution(resolution),
                                                                                                     _occupancyGrid(occupancyGrid),
                                                                                                     _caster(occupancyGrid, _maxRange / _resolution),
                                                                                                     _offset_set(false)
{
    _size_x = occupancyGrid.width * _resolution;
    _size_y = occupancyGrid.height * _resolution;
}

Pose World::computeDrivenDistance(emc::OdometryData &odom)
{
    double odom_dist_x, odom_dist_y, odom_angle;
    if (_offset_set)
    {
        odom_dist_x = odom.x - previous_offset[0];
        odom_dist_y = odom.y - previous_offset[1];
        odom_angle = wrapToPi(odom.a - previous_offset[2]);
    }
    else
    {
        odom_dist_x = 0;
        odom_dist_y = 0;
        odom_angle = 0;
        _offset_set = true;
    }

    Pose delPose = {odom_dist_x, odom_dist_y, odom_angle};
    previous_offset = {odom.x, odom.y, odom.a, odom.timestamp};
    return delPose;
}

void World::setlaserparams(const emc::LaserData &lasermsg, const double &noiseparam, int subsample)
{
    _N_rays = lasermsg.ranges.size();
    _angle_min = lasermsg.angle_min;
    _angle_max = lasermsg.angle_max;
    _angle_inc = lasermsg.angle_increment;
    _subsample = subsample;

    _measurementNoise = noiseparam;
}

double World::_get_noise_sample_gaussian(double mu, double sigma)
{
    std::normal_distribution<double> distribution(mu, sigma);
    return distribution(_generator);
}

measurementList World::predictMeasurement(const Pose &particlePose)
{
    measurementList prediction;
    predictMeasurement(particlePose, prediction);
    return prediction;
}

void World::predictMeasurement(const Pose &particlePose, measurementList &prediction)
{
    // The ray tracer using in this method is (pratically) unmodified from the one described in

    // CDDT: Fast Approximate 2D Ray Casting for Accelerated Localization
    // https://arxiv.org/pdf/1705.01167.pdf

    // The following map frame conventions are used in the raycaster:

    // - \theta = 0 is aligned with the x-axis
    // - Clockwise theta is positive
    // - The origin of the map is located in the top left corner
    // - The positive direction of the y-axis is downward
    // - The positive direction of the x-axis is to the right

    // The particle filter assumes the following conventions
    // - \theta = 0 is aligned with the x-axis
    // - Counter Clockwise theta is positive
    // The origin of the map is located in the bottom left
    // Positive y-axis is upward
    // Positive x-axis is to the right

    // Convert robotPose to gridmap frame and coordinates
    double xParticle = particlePose[0] / _resolution;
    double yParticle = (-particlePose[1] + _size_y) / _resolution;
    double thParticle = particlePose[2];
    // Prepare list of measurements
    prediction.clear();
    prediction.reserve(std::ceil(_N_rays / _subsample));
    // Loop over range of thetas
    for (int i = 0; i < _N_rays; i += _subsample)
    {
        double th_i = thParticle + _angle_min + i * _angle_inc;
        double d_i = _caster.calc_range(xParticle, yParticle, -th_i) +
                     _get_noise_sample_gaussian(0, _measurementNoise);

        if (d_i > _maxRange / _resolution)
        {
            d_i = 0;
        }

        measurement pred_i = d_i * _resolution;
        prediction.push_back(pred_i);
    }
    return;
}