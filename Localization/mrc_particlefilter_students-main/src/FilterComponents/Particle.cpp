#include "./FilterComponents/Particle.h"

#include <numeric>
#include "tools.h"


#include <boost/math/distributions/normal.hpp>


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
    // I wrote it in two different ways
    int option_  =0;
    bool print = false;

    double x, y, theta;
    x = _particlePose[0]; //robot at t in map frame
    y = _particlePose[1]; //robot at t in map frame
    theta = _particlePose[2]; //robot at t in map frame 

    // updated
    double xNew, yNew, thetaNew;
    if(option_ ==0)
        {
        // // Incorporate the odom information to change the location of the particles
        // // 1.) Sample a displacement in each direction
        // // given the odometry information and magnitude of the noise
        // // Generate random numbers using the normal distributions

            double xSampledNoise = _get_noise_sample_gaussian(dPose[0], proc_noise[0]);
            double ySampledNoise= _get_noise_sample_gaussian(dPose[1], proc_noise[0]);
            double thteaSampledNoise= _get_noise_sample_gaussian(dPose[2], proc_noise[1]);

            // angular
            // bool print_theta =false;
            theta+= thteaSampledNoise;
            theta = wrapToPi(theta);
        
            double thetaRobotFrame=  theta  +offset_angle; // robot frame
            // // 2.) Update the value of theta given the sampled theta displacement
            //inverse rotation
            std::vector<double> dmap{xSampledNoise, ySampledNoise, 0};
            std::vector<double> InverseRotation = inverse_rotatePose(dmap, thetaRobotFrame);
            xNew = InverseRotation[0];
            yNew =  InverseRotation[1];
            thetaNew = theta;
        }
 
    if(option_==1){
    std::normal_distribution<double> xyDistribution(0, proc_noise[0]);
    std::normal_distribution<double> thetaDistribution(0, proc_noise[1]);
    double xSampledNoise = xyDistribution(*_generatorPtr); //noise to robot in robot frame
    double ySampledNoise = xyDistribution(*_generatorPtr); //noise to robot in robot frame
    // angular
    double thteaSampledNoise = thetaDistribution(*_generatorPtr); //noise to robot in robot frame
    theta += thteaSampledNoise;
    theta = wrapToPi(theta);
    // 2.) Update the value of theta given the sampled theta displacement
    double xDisplacement_robot = (dPose[0] * std::cos(-offset_angle) - dPose[1] * std::sin(-offset_angle)) ; // dpos+noise in robot frame
    double yDisplacement_robot = (dPose[0] * std::sin(-offset_angle) + dPose[1] * std::cos(-offset_angle)) ; // dpos+noise in robot frame

    // 2.) Update the value of theta given the sampled theta displacement
    double thetaDisplacement = dPose[2] ; //dpos in robot frame
    double xDisplacement_mapframe = (xDisplacement_robot * std::cos(-theta- thetaDisplacement) -yDisplacement_robot * std::sin(-theta- thetaDisplacement)) + xSampledNoise; // displacement to map frame
    double yDisplacement_mapframe = (xDisplacement_robot * std::sin(-theta- thetaDisplacement) + yDisplacement_robot * std::cos(-theta- thetaDisplacement )) + ySampledNoise;// displacement to map frame

    // update
    xNew= xDisplacement_mapframe;
    yNew= yDisplacement_mapframe; // DO NOT CHANGE: Difference in map coordinate convetion
     thetaNew = theta + thetaDisplacement;
    }

        // 3.) The new x and y location are obtained by using the displacement and new theta angle
    // Assuming that the robot first turns then drives
    // Make sure to transform the odometry information from odom frame into robot frame first
    // and afterwards into map frame

    _particlePose = {x+xNew, y-yNew, thetaNew};
}

Likelihood Particle::computeLikelihood(const measurementList &data,
                                       World &world,
                                       const MeasModelParams &lm)
{

      
    //start here
    Pose particlePose = getPosition();

    // 1) Generate a prediction
    measurementList prediction = world.predictMeasurement(particlePose);
    // We skip every subsamp beams
     int subsamp = world._subsample;

    int i = 0;
    int sumProbabilities = std::accumulate(data.begin(), data.end(), 0, [&i, subsamp](float a, float b) {
        if (i % subsamp == 0)
            return a + b;
        else
            return a;
    });
    std::cout<<"sumProbabilities "<< sumProbabilities << std::endl; 

    std::vector<double> liklihoodPrediction;
    // 2) Compute the likelihood of the current measurement given the prediction
    Likelihood likelihood = 1; // can also be modeles when start with _weight
    double sumLiklihoods =0;

    for (size_t i = 0; i < data.size(); ++i)
    {
        // Skip measurements based on subsampling factor
        if (i % subsamp != 0)
            continue;

        double curLikelihood = measurementmodel(prediction[i], data[i], lm);
        sumLiklihoods+=curLikelihood;
        liklihoodPrediction.push_back(curLikelihood);
 
    }

    for(size_t i = 0; i < liklihoodPrediction.size(); ++i)
    {
    likelihood *=(liklihoodPrediction[i]/sumLiklihoods); // try to normlize
    }
        //std::cout<<"Cur i "<< i << "- "<< curLikelihood << std::endl; 

    // 4) Return the result
    return likelihood;
}

double Particle::measurementmodel(const measurement &prediction,
                                  const measurement &data,
                                  const MeasModelParams &lm) const
{

  
    //  z_k^t represents an element of the data 
    //  z_k^t* represents an element of the prediction

     double likelihood = 0.0;

    // Gaussian component

    double gaussian_likelihood = (data< lm.max_range)? exp(-0.5 * pow((data - prediction) / lm.hit_sigma, 2)) / (lm.hit_sigma * sqrt(2 * M_PI)) :0; // if data< max range-> val else 0
    likelihood += lm.hit_prob * gaussian_likelihood;   

    // Exponential component -> unexpected obstacle
    double deltaExponent = 1/(1-std::exp(-lm.short_lambda*prediction));
    double exponential_likelihood =  (data<prediction) ?  deltaExponent* lm.short_lambda * exp(-lm.short_lambda * data): 0;  // if data <prediction, otherwise 0
    likelihood += lm.short_prob * exponential_likelihood;
 
    // Uniform component -> Random measurements
    double uniform_likelihood = (data<lm.rand_range) ?  1 / lm.rand_range :0 ; // if data between 0 and z_Max, otherwise 0
    likelihood += lm.rand_prob* uniform_likelihood;

    // PMax-> Failures
    double faliaturesLikelihood= (data==lm.max_range) ? 1: 0; // if data ==maxrange ->1 else 0
    likelihood += lm.max_prob* faliaturesLikelihood;
 
   // std::cout<<"likelihood "<< likelihood << std::endl; 

    return likelihood;

 
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