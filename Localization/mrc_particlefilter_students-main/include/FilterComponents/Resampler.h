#pragma once
#include "./Filters/ParticleFilterBase.h"

enum resampleAlgorithm
{
    Multinomial,
    Stratified,
    Adaptive
};

class Resampler
{
public:
    /// @brief Initialise the resampler by setting the algorithm and storing
    /// a pointer to the random number generator
    /// @param generatorPtr
    /// @param algorithm
    void initaliseResampler(std::default_random_engine *generatorPtr, std::string algorithm);

    /// @brief Resample according to the value set in _algorithm
    /// @param Particles The cloud of particles
    /// @param N The amount of particles
    void resample(ParticleList &Particles, const int N);

    /// @brief Generate a sample for the resampling in the Adaptive Particle Filter
    /// @param Particles The cloud of particles
    /// @return The sample index
    int generateSampleIndex(const ParticleList &Particles);

private:
    /// @brief Construct Vector Q, containing the cummulative sum of the Particle Weights
    /// @param Particles
    /// @return
    LikelihoodVector _makeCumulativeSumVector(const ParticleList &Particles);

    /// @brief Implements multinomial resampling
    /// @param Particles The cloud of particles
    /// @param N The amount of particles
    void _multinomial(ParticleList &Particles, const int N);

    /// @brief Implements Stratified Resampling
    /// @param Particles The cloud of particles
    /// @param N The amount of particles
    void _stratified(ParticleList &Particles, const int N);

    /// @brief The resampling alogrithm to use
    resampleAlgorithm _algorithm;

    /// @brief Pointer to the random number generator
    std::default_random_engine *_generatorPtr;
};
