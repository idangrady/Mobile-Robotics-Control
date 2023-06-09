#include <gtest/gtest.h>
#include <cmath>

#include "./Filters/ParticleFilter.h"
#include "./tools.h"

#include <emc/io.h>

TEST(Assignment2, accurateAverage)
{

  int argc = 1;
  char arg0[] = "./main.cpp";
  char *argv[] = {&arg0[0]};

  auto programConfig = load_config_file(argc, argv);
  World world = constructWorldModel(programConfig);

  double N = 1e4;
  std::vector<std::vector<double>> means = {{1, 0, 0}, {0.33, 0.5, 0}, {1, 1, 0}, {5, 3, 3.14}, {3, 5, -3.14}};
  std::vector<std::vector<double>> sigmas = {{0.1, 0.1, 0.1}, {10, 16, 2}, {1, 1, 2}, {3.25, 2.5, 1}, {0, 5, 1}};

  for (int i = 0; i < means.size(); i++)
  {
    double mean_i[3] = {means[i][0], means[i][1], means[i][2]};
    double sigma_i[3] = {sigmas[i][0], sigmas[i][1], sigmas[i][2]};

    ParticleFilter pfilt(world, mean_i, sigma_i, N);
    Pose average_computed = pfilt.get_average_state();

    EXPECT_NEAR(average_computed[0], mean_i[0], 0.1) << "Average computed for state x is invalid" << std::endl;
    EXPECT_NEAR(average_computed[1], mean_i[1], 0.1) << "Average computed for state y is invalid" << std::endl;
    EXPECT_LT(std::fabs(wrapToPi(average_computed[2] - mean_i[2])), 0.1) << "Average computed for state th is invalid" << std::endl;
  }
}

TEST(Assignment2, nonUniformWeights)
{

  int argc = 1;
  char arg0[] = "./main.cpp";
  char *argv[] = {&arg0[0]};

  auto programConfig = load_config_file(argc, argv);
  World world = constructWorldModel(programConfig);

  double N = 1e4;
  double mean_i[3] = {1, 0, 0};
  double sigma_i[3] = {4, 4, 0.1};

  ParticleFilter pfilt(world, mean_i, sigma_i, N);

  LikelihoodVector likelihoods;
  likelihoods.resize(N);

  for (int i = 0; i < N; i++)
  {
    try
    {
      Particle particle_i = pfilt.getParticle(i);
      likelihoods[i] = 1 / (std::pow(mean_i[0] - particle_i.getPosition()[0], 2) + std::pow(mean_i[1] - particle_i.getPosition()[1], 2) + std::pow(mean_i[2] - particle_i.getPosition()[2], 2));
    }
    catch (std::invalid_argument const &e)
    {
      std::cout << e.what() << std::endl;
      FAIL() << "Prevented a likely Segfault: Possibly Not enough Particles in the Filter";
      return;
    }
  }

  pfilt.set_weights(likelihoods);
  Pose average_computed = pfilt.get_average_state();

  EXPECT_NEAR(average_computed[0], mean_i[0], 0.1) << "Average computed for state x is invalid" << std::endl;
  EXPECT_NEAR(average_computed[1], mean_i[1], 0.1) << "Average computed for state y is invalid" << std::endl;
  EXPECT_LT(std::fabs(wrapToPi(average_computed[2] - mean_i[2])), 0.1) << "Average computed for state theta is invalid" << std::endl;
}