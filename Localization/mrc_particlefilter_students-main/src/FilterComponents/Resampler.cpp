#include "./FilterComponents/Resampler.h"
#include <cstdlib> 
 

void Resampler::initaliseResampler(std::default_random_engine *generatorPtr, std::string algorithm)
{
    _generatorPtr = generatorPtr;

    if (algorithm.compare("Multinomial") == 0)
    {
        _algorithm = Multinomial;
    }
    else if (algorithm.compare("Stratified") == 0)
    {
        _algorithm = Stratified;
    }
    else if (algorithm.compare("Adaptive") == 0)
    {
        _algorithm = Adaptive;
    }
    else
    {
        throw std::invalid_argument("The _resamplingAlgorithm is invalid");
    }
}

void Resampler::_multinomial(ParticleList &Particles, const int N)
{
    // Counters
    int n = 0;


    // Clear current Particles, save old particles
    ParticleList OldParticles = Particles;
    Particles.clear();
    Particles.reserve(N);

    // Compute Cumulative Sum Vector of weights
    LikelihoodVector Q= _makeCumulativeSumVector(OldParticles);
    float sum_weights = 0;
    while (n < N)
    {
        // TODO
        float v1 = (rand() % 100) / 100.0; ; // check float
        int m = 0;
        while(v1<Q[m])
        {
            m++;
        }
        // rechalculate the weights
        OldParticles[m].setWeight(1/N);
        Particles.push_back(OldParticles[m]);
        n++;
       }
       // check later
    //     // normlizing the weights
    //    for(int i=0; i<N;i++){
    //         Particles[i].setWeight(Particles[i].getWeight()/=sum_weights); // normlizing the weights.
    //     }
    return;
}

void Resampler::_stratified(ParticleList &Particles, const int N)
{
    // Counters
    int n = 0;
    int m = 0;

    // Clear current Particles, save old particles
    ParticleList OldParticles = Particles;
    Particles.clear();
    Particles.reserve(N);

    // Compute Cumulative Sum Vector of weights
    LikelihoodVector Q = _makeCumulativeSumVector(OldParticles);
    float sum_weights= 0;
 
    while (n < N)
    {
        float u = (float)((rand() % N) / 100.0);
        u +=n/N;
        while(u<Q[m]){m++;};
        n++;
        OldParticles[m].setWeight(1/N);
        Particles.push_back(OldParticles[m]);
    }
    return;
}

//////////////////////////////////////////////////////////////////////////////////
/// LIKELY NO NEED TO CHANGE BELOW HERE
//////////////////////////////////////////////////////////////////////////////////

void Resampler::resample(ParticleList &Particles, const int N)
{
    // Selects the desired resampling algorithm
    if (_algorithm == Multinomial)
    {
        _multinomial(Particles, N);
    }
    else if (_algorithm == Stratified)
    {
        _stratified(Particles, N);
    }
    else
    {
        throw std::invalid_argument("The _resamplingAlgorithm is invalid");
    }
}

LikelihoodVector Resampler::_makeCumulativeSumVector(const ParticleList &Particles)
{
    LikelihoodVector Q;          // Create Empty vector
    Q.reserve(Particles.size()); // Reserve space for N elements
    long double runningSum = 0;  // Running sum of particle 0 till i

    for (const Particle &p : Particles)
    {
        runningSum += p.getWeight(); // Add the weight of particle i to the running sum
        Q.push_back(runningSum);     // Append the running sum
    }
    return Q;
}

int Resampler::generateSampleIndex(const ParticleList &Particles)
{
    // This functions samples a particle index based on the
    // weights of the respective particles

    // Generate a vector containing the cumulative weight of the particles
    // (Would be more efficient to only calculate this once every time the resampler is called)
    LikelihoodVector Q = _makeCumulativeSumVector(Particles);

    // Sample a unif-dist. random variable, between 0 and 1
    std::uniform_real_distribution<double> distribution(1e-6, 1);
    double u = distribution(*_generatorPtr);

    // Lambda that determines wheter the element of the cum.sum is larger than the sample
    auto islarger = [u](Likelihood i)
    { return i >= u; };
    // Find the first particle that statisfies the above
    auto it = std::find_if(begin(Q), end(Q), islarger);
    // Obtain idx from iterator
    int m = it - Q.begin();
    // Return this index
    return m;
}