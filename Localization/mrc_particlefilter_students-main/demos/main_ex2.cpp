// "Local Includes"
#include "./Objects/World.h"
#include "./Objects/Robot.h"

#include "./Filters/ParticleFilter.h"

#include "./FilterComponents/Resampler.h"

#include "./tools.h"

// EMC-enviroment
#include <emc/io.h>
#include <emc/rate.h>

// Timing and file input/output
#include <chrono>

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "particlefilterMRC_ex2");
    // Load the intended config-file
    auto programConfig = load_config_file(argc, argv);
    // Initialise the World Model
    // ------
    // Initialise the world and OccupancyGridMap
    World world = constructWorldModel(programConfig);
    ParticleFilterBase *pFilt = constructParticleFilter(world, programConfig);
    pFilt->init_publishers();
    //----------------------------------------------------------------------------------
    //  Configure the MRC-enviroment
    emc::Rate r(5);
    emc::IO io;
    Robot robot(&io);
    int timestep = 0;
    // ------
    // Loop the simulation for the length of the simulation
    while (io.ok())
    {
        // Start Clock of this iteration
        time_point startOfIteration = std::chrono::steady_clock::now();
        // Publish particle poses
        pFilt->pub_particles();
        pFilt->pub_average_pose();
        // Stop clock and report time taken
        time_point endOfIteration = std::chrono::steady_clock::now();
        reportStatistics(timestep, startOfIteration, pFilt);
        // Interface with underlying ros system
        ros::spinOnce();
        timestep++;
    }
    // Exit program
    delete pFilt; // Make sure to free our memory allocated by new
}
