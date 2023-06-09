#pragma once

#include "./Objects/World.h"

#include <math.h>
#include <random>

typedef double Likelihood;
typedef std::vector<Likelihood> LikelihoodVector;
// Implementation of a particle, used in the implementation in the

struct MeasModelParams
{
    // - - - - - - - - - - - - - - -
    // Gaussian component          |
    // - - - - - - - - - - - - - - -
    /// @brief Probability of correct range
    double hit_prob;

    /// @brief Standard deviation on measurement when correct
    double hit_sigma;

    // - - - - - - - - - - - - - - -
    // Uniform component           |
    // - - - - - - - - - - - - - - -
    /// @brief Probability of a random reading
    double rand_prob;

    /// @brief Random reading is distributed from [0, rand_range]
    double rand_range; // max range of random reading

    // - - - - - - - - - - - - - - -
    // Exponential component       |
    // - - - - - - - - - - - - - - -
    /// @brief Probability of a short reading
    double short_prob;

    /// @brief Lambda paramter of exponential dist.
    double short_lambda; // Lambda parameter of exponent. dist.

    // - - - - - - - - - - - - - - -
    // Discrete Uniform component  |
    // - - - - - - - - - - - - - - -
    /// @brief Probability of max range
    double max_prob;

    /// @brief Width of max range distribution
    double max_range;
};

class Particle
{
public:
    /// @brief Construct a particle which is distributed uniform and randomly wihtin map bounds
    /// @param World    The world model
    /// @param weight   The weight of the particle
    /// @param _generatorPtr    A pointer to the random number generator of the particle filter
    Particle(const World &World,
             const double &weight,
             std::default_random_engine *_generatorPtr);

    /// @brief Construct a particle which is distributed randomly according to a gaussian centered at mean
    ///        and with stand dev. sigma
    /// @param World The world model
    /// @param mean The mean of the gaussian
    /// @param sigma The stand dev. of the gaussian
    /// @param weight The weight of the particle
    /// @param _generatorPtr A pointer to the random number generator of the particle filter
    Particle(const World &World,
             const double mean[3],
             const double sigma[3],
             const double &weight,
             std::default_random_engine *_generatorPtr);

    /// @brief
    /// @return Pose of the Particle
    Pose getPosition() const;

    /// @brief
    /// @return Weight of the Particle
    Likelihood getWeight() const;

    /// @brief
    /// @param weight the new weight of the Particle
    void setWeight(const Likelihood &weight);

    /// @brief Propagate the Particle based on the received odom information
    /// @param dPose The change in pose in odom frame
    /// @param proc_noise The process noise
    /// @param offset_angle The angle offset between odom frame and robot frame
    void propagateSample(const Pose &dPose,
                         const double proc_noise[2],
                         const double &offset_angle);

    /// @brief Compute the likelihood of this particle, given the measurement and the map
    /// @param measurement The last received LRF measurment
    /// @param world The world model
    /// @param lm The parameters of the measurement model
    /// @return Particle Likelihood
    Likelihood computeLikelihood(const measurementList &measurement,
                                 World &world,
                                 const MeasModelParams &lm);

    /// @brief Implements the measurement model
    /// @param prediction The predicted measurment
    /// @param data The actual measurment
    /// @param lm The parameters of the measurement model
    /// @return Likelihood of the measurement given the prediction (normalised between 0 and 1)
    double measurementmodel(const measurement &prediction,
                            const measurement &data,
                            const MeasModelParams &lm) const;

    /// @brief Get an uniform sample between [minval, maxval]
    /// @param minval
    /// @param maxval
    /// @return
    double _get_noise_sample_unfiorm(const double &minval,
                                     const double &maxval) const;

    /// @brief Get a random sample from N(mu, sigma)
    /// @param mu
    /// @param sigma
    /// @return
    double _get_noise_sample_gaussian(const double &mu,
                                      const double &sigma) const;

    /// @brief Print this particle to terminal
    /// @param i particle index
    void print(const int &i) const;

private:
    /// @brief The general part of the constructor
    /// @param world The world model
    /// @param weight The weight of the particle
    /// @param randompose The pose of the particle
    void initialise(const World &world,
                    const double &weight,
                    const Pose &randompose);

    /// @brief Particle Pose
    Pose _particlePose;

    /// @brief Particle Weight
    Likelihood _weight;

    /// @brief Pointer to the random number generator
    std::default_random_engine *_generatorPtr;
};
