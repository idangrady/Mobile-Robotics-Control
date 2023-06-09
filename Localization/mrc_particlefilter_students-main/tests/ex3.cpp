#include <gtest/gtest.h>
#include <cmath>

#include "./Filters/ParticleFilter.h"
#include "./tools.h"

bool performComparison(const Pose &Result, const Pose &Expectation, double tol)
{
  bool comp = true;
  comp = comp and std::fabs(Result[0] - Expectation[0]) < tol;
  comp = comp and std::fabs(Result[1] - Expectation[1]) < tol;
  comp = comp and std::fabs(Result[2] - Expectation[2]) < tol;

  if (not comp)
  {
    std::cout << std::endl;
    std::cout << "Comparison Failed" << std::endl;
    std::cout << "Expectation: [" << Expectation[0] << "," << Expectation[1] << "," << Expectation[2] << "]" << std::endl;
    std::cout << "Result: [" << Result[0] << "," << Result[1] << "," << Result[2] << "]" << std::endl;
    std::cout << "Tolerance: " << tol << std::endl;
  }

  return comp;
}

bool performComparison(const Pose &Result, const Pose &Expectation, std::vector<double> tol)
{
  bool comp = true;
  comp = comp and std::fabs(Result[0] - Expectation[0]) < tol[0];
  comp = comp and std::fabs(Result[1] - Expectation[1]) < tol[1];
  comp = comp and std::fabs(Result[2] - Expectation[2]) < tol[2];

  if (not comp)
  {
    std::cout << std::endl;
    std::cout << "Comparison Failed" << std::endl;
    std::cout << "Expectation: [" << Expectation[0] << "," << Expectation[1] << "," << Expectation[2] << "]" << std::endl;
    std::cout << "Result: [" << Result[0] << "," << Result[1] << "," << Result[2] << "]" << std::endl;
  }

  return comp;
}

void generalTest(const std::vector<std::vector<double>> &means,
                 const std::vector<std::vector<double>> &dpose,
                 const std::vector<std::vector<double>> &results,
                 const std::vector<double> &odomoffsets)
{
  int argc = 1;
  char arg0[] = "./main.cpp";
  char *argv[] = {&arg0[0]};

  auto programConfig = load_config_file(argc, argv);
  World world = constructWorldModel(programConfig);

  std::default_random_engine _generator;

  for (int i = 0; i < means.size(); i++)
  {
    double mean_i[3] = {means[i][0], means[i][1], means[i][2]};

    // For testing, let's assume noiseless and certain initial condition
    double sigma_i[3] = {0.0, 0.0, 0.0};
    double noise[2] = {0.0, 0.0};

    // Initialise the particle
    Particle p(world, mean_i, sigma_i, 1.0, &_generator);
    // Retrieve the difference in pose
    Pose posediff = dpose[i];
    // Propagate Particle
    p.propagateSample(posediff, noise, odomoffsets[i]);

    Pose new_Pose = p.getPosition();
    if (!performComparison(new_Pose, results[i], 0.1))
    {
      FAIL() << "Propagation of Sample outside of tolerance" << std::endl;
      return;
    }
  }
}

TEST(Assignment3, accuratePropagation_odom0_theta0)
{

  std::vector<std::vector<double>> means = {{1, 0, 0},
                                            {1, 0, -M_PI},
                                            {1, 1, M_PI},
                                            {1, 1, M_PI / 2},
                                            {3, 5, -0.1}};

  std::vector<std::vector<double>> dpose = {{1, 0, 0},
                                            {0, 1, 0},
                                            {3, 4, 0},
                                            {5, 3, 0},
                                            {3, 5, 0}};

  std::vector<double> odomoffsets = {0, 0, 0, 0, 0};

  std::vector<std::vector<double>> results = {{2, 0, 0},
                                              {1, 1, -M_PI},
                                              {-2, 5, M_PI},
                                              {4, 6, M_PI / 2},
                                              {5.485, -0.274, -0.1}};

  generalTest(means, dpose, results, odomoffsets);
}

TEST(Assignment3, accuratePropagation_odom_not0_theta0)
{

  std::vector<std::vector<double>> means = {{1, 0, 0},
                                            {1, 0, 0.25},
                                            {1, 1, 0},
                                            {1, 1, -0.41},
                                            {3, 5, 0}};

  std::vector<std::vector<double>> dpose = {{1, 0, 0},
                                            {0, 1, 0},
                                            {3, 4, 0},
                                            {5, 3, 0},
                                            {3, 5, 0}};

  std::vector<double> odomoffsets = {0.1, -0.5, M_PI, M_PI / 2, -M_PI / 2};

  std::vector<std::vector<double>> results = {{1.995, 0.099, 0.000},
                                              {0.753, -0.969, 0.250},
                                              {-2.000, 5.000, 0.000},
                                              {5.744, 4.389, -0.410},
                                              {-2.000, 2.000, 0.000}};

  generalTest(means, dpose, results, odomoffsets);
}

TEST(Assignment3, accuratePropagation_odom0_theta_not0)
{

  std::vector<std::vector<double>> means = {{1, 0, M_PI},
                                            {1, 0, 0},
                                            {1, 1, 0.68},
                                            {1, 1, -M_PI},
                                            {3, 5, -1.5 * M_PI}};

  std::vector<std::vector<double>> dpose = {{1, 0, -1.5},
                                            {0, 1, 1},
                                            {3, 4, 0.21},
                                            {5, 3, 0.35},
                                            {3, 5, -2}};

  std::vector<double> odomoffsets = {0, 0, 0, 0, 0};

  std::vector<std::vector<double>> results = {{0.929, 0.997, 1.641},
                                              {1.841, -0.540, 1.000},
                                              {5.997, 0.813, 0.890},
                                              {-4.726, 2.104, -2.791},
                                              {3.647, -0.795, -0.429}};

  generalTest(means, dpose, results, odomoffsets);
}

TEST(Assignment3, accuratePropagation_odom_not0_theta_not0)
{

  std::vector<std::vector<double>> means = {{1, 0, M_PI},
                                            {1, 0, 0},
                                            {1, 1, 0.68},
                                            {1, 1, -M_PI},
                                            {3, 5, -1.5 * M_PI}};

  std::vector<std::vector<double>> dpose = {{1, 0, -1.5},
                                            {0, 1, 1},
                                            {3, 4, 0.21},
                                            {5, 3, 0.35},
                                            {3, 5, -2}};

  std::vector<double> odomoffsets = {0.1, -0.5, M_PI, M_PI / 2, -M_PI / 2};

  std::vector<std::vector<double>> results = {{0.830, 0.985, 1.642},
                                              {1.479, -0.877, 1.000},
                                              {-3.996, 1.186, 0.890},
                                              {-0.103, -4.725, -2.791},
                                              {-2.794, 4.352, -0.429}};

  generalTest(means, dpose, results, odomoffsets);
}
