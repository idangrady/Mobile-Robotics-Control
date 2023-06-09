#include "./Filters/ParticleFilterBase.h"

ParticleFilterBase::ParticleFilterBase(const World &world,
                                       const int &N)
{
    _N = N;
    Likelihood weight = 1.0 / double(N);
    _particles.reserve(_N);

        for(int i =0; i<N; i++){
        Particle p(world,weight, &_generator);
        _particles.push_back(p);
    }

}

ParticleFilterBase::ParticleFilterBase(const World &world,
                                       const double mean[3],
                                       const double sigma[3],
                                       const int &N)
{
    _N = N;
    Likelihood weight = 1.0 / double(_N);
    _particles.reserve(_N);
    for(int i =0; i<N; i++){
        Particle p(world,mean,sigma,weight, &_generator);
        _particles.push_back(p);
    }
}

Pose ParticleFilterBase::get_average_state()
{
    // Calculate Weighted Average over samples
    double x_ =0;
    double y_=0; 
    double theta_ = 0;

    double sumWeights =0; 
    for(int i =0; i<_particles.size();i++)
    {
        double curWeights = _particles[i].getWeight();
        x_+= curWeights*_particles[i].getPosition()[0];
        y_+= curWeights*_particles[i].getPosition()[1];
        theta_+= curWeights*_particles[i].getPosition()[2];
        sumWeights+=curWeights;
     }

    Pose weightedPosition = {x_/sumWeights, y_/sumWeights, theta_/sumWeights};
    return weightedPosition;
}

void ParticleFilterBase::propagateSamples(Pose dPose, const double offset_angle)
{
    for (Particle &p : _particles)
    {
        p.propagateSample(dPose, _processNoise, offset_angle);
    }
}

LikelihoodVector ParticleFilterBase::computeLikelihoods(const measurementList &measurement, World &world)
{
    LikelihoodVector result;
    result.reserve(_N);

    for (Particle &p : _particles)
    {
        Likelihood val_i = p.computeLikelihood(measurement, world, _laserModel);
        result.push_back(val_i);
    }

    return result;
}

//////////////////////////////////////////////////////////////////////////////////
/// LIKELY NO NEED TO CHANGE BELOW HERE
//////////////////////////////////////////////////////////////////////////////////

void ParticleFilterBase::setMeasurementModel(const MeasModelParams &mdl)
{
    _laserModel = mdl;
    return;
}

void ParticleFilterBase::setNoiseLevel(double motion_forward_std, double motion_turn_std)
{
    _processNoise[0] = motion_forward_std;
    _processNoise[1] = motion_turn_std;
}

void ParticleFilterBase::set_weights(const LikelihoodVector &likelihoods)
{
    for (int i = 0; i < ParticleFilterBase::getNumberParticles(); i++)
    {
        ParticleFilterBase::_particles[i].setWeight(likelihoods[i]);
    }
}

double ParticleFilterBase::findMaxWeight()
{
    // Lambda that compares the Weight value of Two particles
    auto compareAttribute = [](Particle &particle1, const Particle &particle2)
    { return particle1.getWeight() < particle2.getWeight(); };
    // Find the an iterator of the particle with the highest weight
    auto it = std::max_element(_particles.begin(), _particles.end(), compareAttribute);
    // Return the value of the weight of the found iterator
    return it->getWeight();
}

void ParticleFilterBase::normaliseWeights()
{
    // Lambda that returns the cumulative weight given a particle p
    auto accumulateWeight = [](Likelihood i, const Particle &p)
    { return i + p.getWeight(); };
    // Compute the cumulative weight of all the particles
    Likelihood totalSum = std::accumulate(begin(_particles),
                                          end(_particles),
                                          Likelihood(0.0),
                                          accumulateWeight);
    // lambda that divides the particle p by the totalSum variable
    auto divideWeight = [totalSum](Particle &p)
    { p.setWeight(p.getWeight() / totalSum); };
    // Divide all particles by the totalSum, which thus results in a cumSum of 1.
    std::for_each(_particles.begin(), _particles.end(), divideWeight);
    return;
}

int ParticleFilterBase::getNumberParticles() const
{
    return _N;
}

PoseList ParticleFilterBase::get_PositionList() const
{
    PoseList PositionList;
    PositionList.reserve(_N);
    for (const Particle &p : _particles)
    {
        PositionList.push_back(p.getPosition());
    }

    return PositionList;
}

void ParticleFilterBase::resetNumberParticles()
{
    _N = _particles.size();
}

Particle ParticleFilterBase::getParticle(const int &idx) const
{
    if (idx < _N)
        return _particles[idx];

    throw std::invalid_argument("Requested an invalid particle");
}

void ParticleFilterBase::printAllParticles() const
{
    for (int i = 0; i < _N; i++)
    {
        _particles[i].print(i);
    }
}

//////////////////////////////////////////////////////////////////////////////////
/// LIKELY NO NEED TO UNDERSTAND BELOW HERE
//////////////////////////////////////////////////////////////////////////////////

void ParticleFilterBase::init_publishers()
{
    _nh = std::make_shared<ros::NodeHandle>();
    _laser_pub = _nh->advertise<sensor_msgs::LaserScan>("/laser_match", 1);
    _particle_pub = _nh->advertise<geometry_msgs::PoseArray>("/particles", 1);
    _pose_pub = _nh->advertise<geometry_msgs::PoseArray>("/pose_estimate", 1);
    _map_sub = _nh->subscribe<nav_msgs::MapMetaData>("/map_metadata", 1,
                                                     &ParticleFilterBase::mapCallback, this);
}

void ParticleFilterBase::pub_predicted_laser(World world)
{
    Pose averagePose = get_average_state();

    sensor_msgs::LaserScan msg;
    msg.angle_min = world._angle_min;
    msg.angle_max = world._angle_max;
    msg.angle_increment = world._angle_inc * world._subsample;

    msg.range_min = 0.01;
    msg.range_max = 10;

    msg.header.frame_id = "internal/base_link";
    msg.header.stamp = ros::Time(current_timestamp);

    measurementList prediction;
    world.predictMeasurement(averagePose, prediction);

    msg.ranges = prediction;

    _laser_pub.publish(msg);
}

void ParticleFilterBase::pub_particles()
{
    geometry_msgs::PoseArray msg;

    msg.header.frame_id = "map";
    msg.header.stamp = ros::Time(current_timestamp);
    tf2::Quaternion a;
    msg.poses.reserve(_N);
    for (int i = 0; i < _N; i++)
    {
        geometry_msgs::Pose posemsg;
        auto pose_i = _particles[i].getPosition();

        posemsg.position.x = std::cos(_mapconfig.mapOrientation) * pose_i[0] - std::sin(_mapconfig.mapOrientation) * pose_i[1];
        posemsg.position.y = std::sin(_mapconfig.mapOrientation) * pose_i[0] + std::cos(_mapconfig.mapOrientation) * pose_i[1];
        posemsg.position.z = 0;

        a.setRPY(0, 0, pose_i[2] - _mapconfig.mapOrientation);
        posemsg.orientation.w = a.getW();
        posemsg.orientation.x = a.getX();
        posemsg.orientation.y = a.getY();
        posemsg.orientation.z = a.getZ();

        msg.poses.push_back(posemsg);
    }

    _particle_pub.publish(msg);
}

void ParticleFilterBase::pub_average_pose()
{
    geometry_msgs::PoseArray msg;

    msg.header.frame_id = "map";
    msg.header.stamp = ros::Time(current_timestamp);
    tf2::Quaternion a;
    msg.poses.reserve(1);
    geometry_msgs::Pose posemsg;
    Pose avg_pose = get_average_state();

    posemsg.position.x = std::cos(_mapconfig.mapOrientation) * avg_pose[0] - std::sin(_mapconfig.mapOrientation) * avg_pose[1];
    posemsg.position.y = std::sin(_mapconfig.mapOrientation) * avg_pose[0] + std::cos(_mapconfig.mapOrientation) * avg_pose[1];
    posemsg.position.z = 0;

    a.setRPY(0, 0, avg_pose[2] - _mapconfig.mapOrientation);
    posemsg.orientation.w = a.getW();
    posemsg.orientation.x = a.getX();
    posemsg.orientation.y = a.getY();
    posemsg.orientation.z = a.getZ();

    msg.poses.push_back(posemsg);

    _pose_pub.publish(msg);
}

void ParticleFilterBase::mapCallback(const nav_msgs::MapMetaData::ConstPtr &msg)
{
    _mapconfig.mapResolution = msg->resolution;
    _mapconfig.mapOffsetX = ((msg->width) * msg->resolution) / 2;
    _mapconfig.mapOffsetY = ((msg->height) * msg->resolution) / 2;

    auto q = msg->origin.orientation;
    double yaw = std::atan2(2 * (q.w * q.z + q.x * q.y), q.w * q.w + q.x * q.x - q.y * q.y - q.z * q.z);

    _mapconfig.mapOrientation = yaw;
    _mapconfig.mapInitialised = true;
}
