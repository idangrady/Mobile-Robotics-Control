#include "tools.h"

#include <math.h>

nlohmann::json load_config_file(int argc, char *argv[])
{
    std::string filename;
    if (argc == 1) // No additional arguments provided. Use the default
    {
        // std::cout << "Using default config-file: ";
        filename = "../params.json";
        // std::cout << filename << std::endl;
    }
    else if (argc > 1) // Use the provided config-file
    {
        std::cout << "Using User-provided config-file: ";
        filename = argv[1];
        std::cout << filename << std::endl;
    }
    // Parse the config-file which is in json format
    nlohmann::json programConfig;
    try
    {
        std::ifstream jsonFile(filename);
        programConfig = nlohmann::json::parse(jsonFile);
    }
    catch (nlohmann::detail::parse_error &e)
    {
        std::cout << "\n";
        if (e.byte > 1) // Error not at the start of the file
        {
            std::cout << "Error while parsing JSON-file. Are you providing a valid file?" << std::endl;
        }
        else
        {
            std::cout << "Error while parsing JSON-file. Does this file exsist, or is it empty?" << std::endl;
            std::cout << "Are your sure the JSON-file path is specified relative to this terminal window?" << std::endl;
            std::cout << "\n";
        }
        std::cout << "Refer to the error below to find your issue: " << std::endl;
        std::cout << e.what() << std::endl;
    }

    return programConfig;
}

World constructWorldModel(const nlohmann::json &programConfig)
{
    std::string mapfile;
    double mapresolution;

    if (programConfig.contains("World") && programConfig["World"].contains("OccupancyGridMap"))
    {
        mapfile = programConfig["World"]["OccupancyGridMap"].value("filename", "../maps/heightmap.png");
        mapresolution = programConfig["World"]["OccupancyGridMap"].value("resolution", 0.025);
    }
    else
    {
        ROS_ERROR_STREAM("No Map File provided in the config file");
    }

    double maxrange;

    if (programConfig.contains("Robot") && programConfig["Robot"].contains("maxRange"))
    {
        maxrange = programConfig["Robot"]["maxRange"];
    }
    else
    {
        ROS_ERROR_STREAM("Please provide ['Robot']['maxRange'] in the config file to configure raycaster");
    }

    ranges::OMap occupancyGrid(mapfile);
    World world(occupancyGrid, maxrange, mapresolution);
    return world;
}

MeasModelParams constructMeasurementModel(const nlohmann::json &programConfig)
{
    MeasModelParams mdl;
    nlohmann::json modelconfig({});
    if (programConfig.contains("ParticleFilter") && programConfig["ParticleFilter"].contains("MeasurementModel"))
    {
        modelconfig = programConfig["ParticleFilter"]["MeasurementModel"];
    }
    mdl.hit_prob = modelconfig.value("hit_prob", 0.80);
    mdl.hit_sigma = modelconfig.value("hit_sigma", 0.20);
    mdl.rand_prob = modelconfig.value("rand_prob", 0.05);
    mdl.rand_range = modelconfig.value("rand_range", 10.0);
    mdl.short_prob = modelconfig.value("short_prob", 0.10);
    mdl.short_lambda = modelconfig.value("short_lambda", 0.10);
    mdl.max_prob = modelconfig.value("max_prob", 0.05);
    mdl.max_range = modelconfig.value("max_range", 0.25);
    return mdl;
}

ParticleFilterBase *constructParticleFilter(const World &world, const nlohmann::json &programConfig)
{
    Pose initGuess = programConfig["Robot"]["InitialGuess"];
    // ------
    // Initialise the Particle Filter
    // ------
    // Request relevant part of the config file
    nlohmann::json pfconfig({});
    if (programConfig.contains("ParticleFilter"))
    {
        pfconfig = programConfig["ParticleFilter"];
    }
    else
    {
        ROS_WARN_STREAM("No Particle Filter configuration set, using defaults");
    }

    int NParticles = programConfig["ParticleFilter"].value("Particles", 1e3);
    // Extract the chosen particleFilter algorithm from the JSON-file
    std::string PFFlavor = programConfig["ParticleFilter"].value("Flavor", "Adaptive");
    bool ConventionalPF = PFFlavor.compare("Conventional") == 0;
    // Choose the particleFilter Algorithm to be used
    ParticleFilterBase *pFilt;
    // Provide an initial guess of our location
    double guess[3] = {initGuess[0], initGuess[1], initGuess[2]};
    double guessSigma[3] = {5 / 3, 5 / 3, 1.57 / 3};

    if (ConventionalPF) // Conventional PF
    {
        pFilt = new ParticleFilter(world, guess, guessSigma, NParticles);
    }
    else
    {
        throw std::invalid_argument("The Particle Filter Flavor is invalid");
    }

    MeasModelParams mdl = constructMeasurementModel(programConfig);
    pFilt->configureResampler(programConfig);
    pFilt->setMeasurementModel(mdl);

    nlohmann::json propconfig({});
    if (pfconfig.contains("PropagationParameters"))
    {
        propconfig = pfconfig["PropagationParameters"];
    }

    double motion_forward_std = propconfig.value("motion_forward_std", 0.1);
    double motion_turn_std = propconfig.value("motion_turn_std", 0.1);

    pFilt->setNoiseLevel(motion_forward_std, motion_turn_std);
    return pFilt;
}

void reportStatistics(const int &timestep, const time_point &startOfIteration, ParticleFilterBase *pFilt)
{
    time_point end = std::chrono::steady_clock::now();
    std::cout << "Timestep " << timestep << " Complete. Time Taken: " << std::chrono::duration_cast<std::chrono::microseconds>(end - startOfIteration).count() / 1000.0 << " milliseconds"
              << "\n";

    Pose FilterPose = pFilt->get_average_state();
    std::cout << "Filter Pose: [ " << FilterPose[0] << " ," << FilterPose[1] << " ," << FilterPose[2] << " ]" << std::endl;
    std::cout << std::endl;
}

double wrapToPi(double angle)
{
    if (angle > M_PI)
    {
        int k = std::ceil(angle / 2 * M_PI);
        angle -= 2 * k * M_PI;
        // Rerun to ensure we're indeed correct
        angle = wrapToPi(angle);
    }
    else if (angle < -M_PI)
    {
        int k = std::floor(angle / 2 * M_PI);
        angle += 2 * M_PI;
        // Rerun to ensure we're indeed correct
        angle = wrapToPi(angle);
    }
    return angle;
}

Pose rotatePose(const Pose &pose_in, const double &angle)
{
    Pose pose_out = {0, 0, 0};

    pose_out[0] = pose_in[0] * std::cos(angle) - pose_in[1] * std::sin(angle);
    pose_out[1] = pose_in[0] * std::sin(angle) + pose_in[1] * std::cos(angle);
    pose_out[2] = pose_in[2]; // Difference in angles are preserved under rotation

    return pose_out;
}

Pose inverse_rotatePose(const Pose &pose_in, const double &angle)
{
    Pose pose_out = {0, 0, 0};

    pose_out[0] = pose_in[0] * std::cos(angle) + pose_in[1] * std::sin(angle);
    pose_out[1] = -pose_in[0] * std::sin(angle) + pose_in[1] * std::cos(angle);
    pose_out[2] = pose_in[2]; // Difference in angles are preserved under rotation

    return pose_out;
}