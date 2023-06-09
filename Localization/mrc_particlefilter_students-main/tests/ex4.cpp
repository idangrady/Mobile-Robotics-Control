#include <gtest/gtest.h>
#include <cmath>

#include "./Filters/ParticleFilter.h"
#include "./tools.h"

#include <emc/io.h>

measurementList generateFakeMeasurement(World world, const Pose &robotPose)
{
  measurementList measurement;
  world._subsample = 1;
  world.predictMeasurement(robotPose, measurement);
  return measurement;
}

void generalTest(Pose robotPose, Pose offset)
{
  // Configuration and intialisation
  int argc = 2;
  char arg0[] = "./main.cpp";
  char arg1[] = "../config/testconfig.json";
  char *argv[] = {&arg0[0], &arg1[0]};

  auto programConfig = load_config_file(argc, argv);
  std::default_random_engine _generator;
  // Construct World Model
  World world = constructWorldModel(programConfig);

  world._N_rays = 1e3;
  world._angle_min = -2;
  world._angle_max = 2;
  world._angle_inc = 4 / 999;
  world._subsample = 15;
  world._measurementNoise = 0;
  // Generate Fake Laser Measurement
  auto fakedata = generateFakeMeasurement(world, robotPose);

  // Generate a particle at robotpos
  double robotpose[3] = {robotPose[0], robotPose[1], robotPose[2]};
  double zerosigma[3] = {0, 0, 0};
  Particle p(world, robotpose, zerosigma, 1.0, &_generator);

  // Generate an offset particle
  double robotpose2[3] = {robotPose[0] + offset[0],
                          robotPose[1] + offset[1],
                          robotPose[2] + offset[2]};

  Particle p2(world, robotpose2, zerosigma, 1.0, &_generator);

  MeasModelParams lm = constructMeasurementModel(programConfig);

  EXPECT_GT(p.computeLikelihood(fakedata, world, lm), 0);
  EXPECT_LT(p.computeLikelihood(fakedata, world, lm), 1);

  EXPECT_GT(p2.computeLikelihood(fakedata, world, lm), 0);
  EXPECT_LT(p2.computeLikelihood(fakedata, world, lm), 1);

  EXPECT_GT(p.computeLikelihood(fakedata, world, lm), p2.computeLikelihood(fakedata, world, lm));
}

TEST(Assignment4, particlelikelihood0)
{
  Pose robot = {4, 4, 0};
  Pose offset = {-1, -1, -0.25};
  generalTest(robot, offset);
}

TEST(Assignment4, particlelikelihood1)
{
  Pose robot = {5, 4, 0};
  Pose offset = {-1, 1, 0.5};
  generalTest(robot, offset);
}

TEST(Assignment4, particlelikelihood2)
{
  Pose robot = {3, 3, 3.14};
  Pose offset = {-0.1, -0.1, -0.1};
  generalTest(robot, offset);
}
