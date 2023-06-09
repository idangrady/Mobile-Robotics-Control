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
    ros::init(argc, argv, "particlefilterMRC");
    // Load the intended config-file
    auto programConfig = load_config_file(argc, argv);
    // Initialise the World Model
    // ------
    // Initialise the world and OccupancyGridMap
    World world = constructWorldModel(programConfig);
    ParticleFilterBase *pFilt = constructParticleFilter(world, programConfig);
    pFilt->init_publishers();
    //----------------------------------------------------------------------------------
    // Configure the MRC-enviroment
    emc::Rate r(5);
    emc::IO io;
    Robot robot(&io);
    // Storage for newest message
    emc::LaserData laserdata;
    emc::OdometryData odomdata;
    // Bookkeeping Variables
    bool OdomUpdate = false;
    bool LaserUpdate = false;
    bool laserinit = false;
    int timestep = 0;
    // ------
    // Loop the simulation for the length of the simulation
    while (io.ok())
    {
        r.sleep();
        // Obtain odometry measurements from the robots/simulator
        if (robot.newOdomData())
        {
            odomdata = robot.getOdomData();
            OdomUpdate = true;
        }
        // Obtain laser measurements from the robots/simulator
        if (robot.newLaserData())
        {
            laserdata = robot.getLaserData();
            LaserUpdate = true;

            if (not laserinit)
            {
                double expected_dist_noise = programConfig["ParticleFilter"]
                                                          ["PropagationParameters"]
                                                          ["meas_dist_std"];

                world.setlaserparams(laserdata, expected_dist_noise);
                laserinit = true;
            }
        }
        // The ParticleFilter incorporates the Measurement
        if (OdomUpdate and LaserUpdate)
        // Update Odom every time, update laser only when certain distance was driven or initialised
        {
            // Start Clock of this iteration
            time_point startOfIteration = std::chrono::steady_clock::now();
            Pose dPose = world.computeDrivenDistance(odomdata);
            pFilt->update(dPose, laserdata, world, odomdata);
            pFilt->pub_predicted_laser(world);
            pFilt->pub_particles();
            pFilt->pub_average_pose();

            pFilt->update(dPose, laserdata, world, odomdata);
            // Mark the stored messages stale
            OdomUpdate = false;
            LaserUpdate = false;
            // Stop clock and report time taken
            time_point endOfIteration = std::chrono::steady_clock::now();
            reportStatistics(timestep, startOfIteration, pFilt);
            timestep++;
        }
    }
    // Exit program
    delete pFilt; // Make sure to free our memory allocated by new
}
