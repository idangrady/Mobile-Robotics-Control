#include "./FilterComponents/Particle.h"

#include "tools.h"

void Particle::initialise(const World &world,
                          const double &weight,
                          const Pose &randompose)
{
    // Set the weight of the particle
    _weight = weight;
    // Set the pose of the particle
    _particlePose = randompose;
}

Particle::Particle(const World &world,
                   const double &weight,
                   std::default_random_engine *generatorPtr)
{
    // Store a pointer to the generator of the particle filter
    _generatorPtr = generatorPtr;
    // Generate a pose
    std::uniform_real_distribution<double> xDistribution(0.0, world.get_size_x());
    std::uniform_real_distribution<double> yDistribution(0.0, world.get_size_y());
    std::uniform_real_distribution<double> thetaDistribution(0.0,  M_PI); // convert to radiance

    // Generate random numbers using the normal distributions
    double x = xDistribution(*_generatorPtr);
    double y = yDistribution(*_generatorPtr);
    double theta = thetaDistribution(*_generatorPtr);

    Pose randomPose = {x, y, theta};

    // run initialise
    initialise(world, weight, randomPose);
}

Particle::Particle(const World &world,
                   const double mean[3],
                   const double sigma[3],
                   const double &weight,
                   std::default_random_engine *generatorPtr)
{
    // Store a pointer to the generator of the particle filter
    _generatorPtr = generatorPtr;
    std::normal_distribution<double> xDistribution(mean[0], sigma[0]);
    std::normal_distribution<double> yDistribution(mean[1], sigma[1]);
    std::normal_distribution<double> thetaDistribution(mean[2], sigma[2]);

    // Generate random numbers using the normal distributions
    double x = xDistribution(*_generatorPtr);
    double y = yDistribution(*_generatorPtr);
    double theta = thetaDistribution(*_generatorPtr);

    Pose randomPose = {x, y, theta};
 
    // run initialise
    initialise(world, weight, randomPose);
}

    // /// @brief Propagate the Particle based on the received odom information
    // /// @param dPose The change in pose in odom frame
    // /// @param proc_noise The process noise
    // /// @param offset_angle The angle offset between odom frame and robot frame>>>
void Particle::propagateSample(const Pose &dPose,
                               const double proc_noise[2],
                               const double &offset_angle)
{
    bool print = false;

    double x, y, theta;
    x = _particlePose[0]; //robot at t in map frame
    y = _particlePose[1]; //robot at t in map frame
    theta = _particlePose[2]; //robot at t in map frame 

   if (print) {std::cout<<"x: "<<x<<", y "<<y<<std::endl;}
    // sampled based on the angular and linear velocity
    std::normal_distribution<double> xyDistribution(0, proc_noise[0]);
    std::normal_distribution<double> thetaDistribution(0, proc_noise[1]);

    // Incorporate the odom information to change the location of the particles
    // 1.) Sample a displacement in each direction
    // given the odometry information and magnitude of the noise
    // Generate random numbers using the normal distributions
    double xSampledNoise = xyDistribution(*_generatorPtr); //noise to robot in robot frame
    double ySampledNoise = xyDistribution(*_generatorPtr); //noise to robot in robot frame
     if (print) {std::cout<<"xSampledNoise: "<<xSampledNoise<<", ySampledNoise "<<ySampledNoise<<std::endl;}
    // angular
    double thteaSampledNoise = thetaDistribution(*_generatorPtr); //noise to robot in robot frame

    // // 2.) Update the value of theta given the sampled theta displacement
    // double xDisplacement_robot = (dPose[0] * std::cos(-offset_angle) - dPose[1] * std::sin(-offset_angle)) ; // dpos+noise in robot frame
    // double yDisplacement_robot = (dPose[0] * std::sin(-offset_angle) + dPose[1] * std::cos(-offset_angle)) ; // dpos+noise in robot frame

    // // // 2.) Update the value of theta given the sampled theta displacement
    // double thetaDisplacement = dPose[2] + thteaSampledNoise; //dpos in robot frame

    //inverse rotation
    double xDisplacement_mapframe = (xSampledNoise * std::cos(theta) + ySampledNoise * std::sin(theta)) ; // displacement to map frame
    double yDisplacement_mapframe = (-xSampledNoise * std::sin(theta) - ySampledNoise * std::cos(theta)) ;// displacement to map frame
    // if(print){
    //     std::cout<<"Start: " <<x<<"," <<y<<std::endl;
    //     std::cout<<"End" <<(x+ xDisplacement_mapframe)<<"," <<(y+ yDisplacement_mapframe)<<std::endl;
    // }




    // 3.) The new x and y location are obtained by using the displacement and new theta angle
    // Assuming that the robot first turns then drives
    // Make sure to transform the odometry information from odom frame into robot frame first
    // and afterwards into map frame

    Pose displacement_mapframe = {xDisplacement_mapframe, yDisplacement_mapframe, thetaDisplacement};

    double x_new = x+ displacement_mapframe[0];
    double y_new = y- displacement_mapframe[1]; // DO NOT CHANGE: Difference in map coordinate convetion
    // Write result to _particlePose
 //   double new_theta = theta + thetaDisplacement;
    _particlePose = {x_new, y_new, theta+thteaSampledNoise };
}

Likelihood Particle::computeLikelihood(const measurementList &data,
                                       World &world,
                                       const MeasModelParams &lm)
{

    Pose particlePose = getPosition();

    // 1) Generate a prediction
    measurementList prediction = world.predictMeasurement(particlePose);
    // We skip every subsamp beams
    int subsamp = world._subsample;

    // 2) Compute the likelihood of the current measurement given the prediction
    Likelihood likelihood = _weight;

    // 4) Return the result
    return likelihood;
}

double Particle::measurementmodel(const measurement &prediction,
                                  const measurement &data,
                                  const MeasModelParams &lm) const
{
    return 1;
}

//////////////////////////////////////////////////////////////////////////////////
/// LIKELY NO NEED TO CHANGE BELOW HERE
//////////////////////////////////////////////////////////////////////////////////

double Particle::_get_noise_sample_gaussian(const double &mu,
                                            const double &sigma) const
{
    std::normal_distribution<double> distribution(mu, sigma);
    return distribution(*_generatorPtr);
}

double Particle::_get_noise_sample_unfiorm(const double &minval,
                                           const double &maxval) const
{
    std::uniform_real_distribution<double> distribution(minval, maxval);
    return distribution(*_generatorPtr);
}

Pose Particle::getPosition() const
{
    return _particlePose;
}

void Particle::setWeight(const Likelihood &weight)
{
    _weight = weight;
}

Likelihood Particle::getWeight() const
{
    return _weight;
}

void Particle::print(const int &i) const
{
    std::cout << "-- Particle " << i << " -- " << std::endl;
    std::cout << " X: " << _particlePose[0] << " Y: " << _particlePose[1] << " Th: " << _particlePose[2] << " Weight: " << _weight << std::endl;
}