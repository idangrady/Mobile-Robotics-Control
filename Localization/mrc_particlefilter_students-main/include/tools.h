#pragma once

#include "./Objects/World.h"
#include "./Objects/Robot.h"

#include "./Filters/ParticleFilter.h"

#include "./3rdparty/json.hpp"

#include <iostream>
#include <fstream>

#include <chrono>

typedef std::chrono::steady_clock::time_point time_point;

/// @brief Load and parse the json file
/// @param argc
/// @param argv
/// @return
nlohmann::json load_config_file(int argc, char *argv[]);

/// @brief Construct and configure the world model
/// @param programConfig the parsed json-config file
/// @return
World constructWorldModel(const nlohmann::json &programConfig);

/// @brief Construct and configure the measurement model struct
/// @param programConfig
/// @return
MeasModelParams constructMeasurementModel(const nlohmann::json &programConfig);

/// @brief Construct and configure the particle filter
/// @param world the world model
/// @param programConfig the parsed json-config file
/// @return
ParticleFilterBase *constructParticleFilter(const World &world, const nlohmann::json &programConfig);

/// @brief Log key statistics of the particle filter to the terminal
/// @param timestep The timestep index
/// @param startOfIteration The time point at which the current iteration was started
/// @param pFilt The current state of the particle filter
void reportStatistics(const int &timestep, const time_point &startOfIteration, ParticleFilterBase *pFilt);

/// @brief Wraps an angle to the range [-Pi, Pi]
/// @param angle An angle in the range [-inf, inf]
/// @return An angle in the range [-pi, pi]
double wrapToPi(double angle);

/// @brief Rotate the input pose by angle amount
/// @param pose_in
/// @param angle
/// @return The rotated pose
Pose rotatePose(const Pose &pose_in, const double &angle);

/// @brief inverse Rotate the input pose by angle amount
/// @param pose_in
/// @param angle
/// @return The inversely rotated pose
Pose inverse_rotatePose(const Pose &pose_in, const double &angle);
