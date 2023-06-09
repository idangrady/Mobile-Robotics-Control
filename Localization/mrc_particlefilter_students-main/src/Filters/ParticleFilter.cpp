#include "./Filters/ParticleFilter.h"

void ParticleFilter::update(const Pose &dPose,
                            const emc::LaserData &measurement,
                            World &world,
                            const emc::OdometryData &odommsg)
{
    ParticleFilterBase::current_timestamp = measurement.timestamp;
    // Update
    measurementList data = measurement.ranges;
    conventionalUpdate(dPose, data, world, odommsg);
    // Resample
    if (needsResampling())
    {
        _resampler.resample(_particles, ParticleFilterBase::getNumberParticles());
        _resampleCounter = 0;
    }
    else
    {
        _resampleCounter++;
    }

    return;
}

void ParticleFilter::conventionalUpdate(const Pose &dPose,
                                        const measurementList &measurement,
                                        World &world,
                                        const emc::OdometryData &odommsg)
{
    // Propagate ParticleFilter Samples based on observed odometry information
    ParticleFilterBase::propagateSamples(dPose, odommsg.a);

    // Compute Likelihoods and weights of the Particles based on the observed measurements
    LikelihoodVector likelihoods = ParticleFilterBase::computeLikelihoods(measurement, world);

    // Set the weight of each particle
    ParticleFilterBase::set_weights(likelihoods);

    // Normalise sum of particle weights to 1
    ParticleFilterBase::normaliseWeights();
}

void ParticleFilter::configureResampler(const nlohmann::json &programConfig)
{
    _resampler.initaliseResampler(&(_generator),
                                  programConfig["ParticleFilter"]["Resampling"]["Algorithm"]);

    std::string rS = programConfig["ParticleFilter"]["Resampling"]["Scheme"];
    if (rS.compare("Always") == 0)
        _resamplingScheme = Always;
    else if (rS.compare("effectiveParticleThreshold") == 0)
        _resamplingScheme = effectiveParticleThreshold;
    else if (rS.compare("everyNtimesteps") == 0)
        _resamplingScheme = everyNtimesteps;
    else if (rS.compare("Never") == 0)
        _resamplingScheme = Never;
    else
        throw std::invalid_argument("The _resamplingScheme is invalid");

    _resampleThreshold = programConfig["ParticleFilter"]["Resampling"]["Threshold"];
}

bool ParticleFilter::needsResampling() const
{
    // Always resample
    if (_resamplingScheme == Always)
    {
        return true;
    }
    // Resample according to the effective particle criterium
    else if (_resamplingScheme == effectiveParticleThreshold)
    {
        // Lambda to acummulate the squared of all particle weights
        auto accumulateSquaredWeight = [](Likelihood i, const Particle &o)
        { return i + o.getWeight() * o.getWeight(); };
        // Comupte the sum of the squared particle weights
        Likelihood sumSquaredWeights = std::accumulate(begin(_particles),
                                                       end(_particles),
                                                       Likelihood(0.0),
                                                       accumulateSquaredWeight);
        // Return true when the threshold is met
        return (1 / sumSquaredWeights) < _resampleThreshold * ParticleFilterBase::getNumberParticles();
    }
    // Resample every N time steps
    else if (_resamplingScheme == everyNtimesteps)
    {
        return ParticleFilterBase::_resampleCounter > _resampleThreshold;
    }
    // Never Resample
    else if (_resamplingScheme == Never)
    {
        return false;
    }
    else
    {
        assert(false);
    }
}
