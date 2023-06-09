#include <gtest/gtest.h>
#include <cmath>

#include "./Filters/ParticleFilter.h"
#include "./tools.h"

#include <emc/io.h>

bool performComparison(const Pose &Result, const Pose &Expectation, double tol);
bool performComparison(const Pose &Result, const Pose &Expectation, std::vector<double> tol);

void general_nonDet_Test(const std::vector<double> &mean,
                         const std::vector<double> &dpose,
                         const std::vector<double> &result,
                         const double &odomoffset)
{
  int argc = 1;
  char arg0[] = "./main.cpp";
  char *argv[] = {&arg0[0]};

  auto programConfig = load_config_file(argc, argv);
  World world = constructWorldModel(programConfig);

  double mean_i[3] = {mean[0], mean[1], mean[2]};
  double sigma_i[3] = {0.0, 0.0, 0.0};

  double N = 1e3;
  ParticleFilter pFilt(world, mean_i, sigma_i, N);

  double straight_uncertainty = 0.1;
  double angle_uncertainty = 0.05;

  pFilt.setNoiseLevel(straight_uncertainty, angle_uncertainty);

  if (!performComparison(pFilt.get_average_state(), mean, 0.1))
  {
    ADD_FAILURE() << "Initialisation of Filter outside of tolerance" << std::endl;
  }

  pFilt.propagateSamples(dpose, odomoffset);

  if (!performComparison(pFilt.get_average_state(), result, 0.25))
  {
    ADD_FAILURE() << "Propagation of Filter outside of tolerance" << std::endl;
  }

  // Calculate Spread in X, Y and theta direction
  Pose mu = pFilt.get_average_state();
  PoseList plist = pFilt.get_PositionList();
  double SDx, SDy, SDt;
  SDx = SDy = SDt = 0;

  for (Pose p : plist)
  {
    SDx += 1 / N * std::pow(p[0] - mu[0], 2);
    SDy += 1 / N * std::pow(p[1] - mu[1], 2);
    SDt += 1 / N * std::pow(p[2] - mu[2], 2);
  }

  SDx = std::sqrt(SDx);
  SDy = std::sqrt(SDy);
  SDt = std::sqrt(SDt);

  auto filterspread = {SDx, SDy, SDt};
  auto expectedspread = {straight_uncertainty, straight_uncertainty, angle_uncertainty};
  auto tolerance = {0.5 * straight_uncertainty, 0.5 * straight_uncertainty, 0.5 * angle_uncertainty};

  if (!performComparison(filterspread, expectedspread, tolerance))
  {
    ADD_FAILURE() << "Spread resulting from propagation of Filter outside of tolerance" << std::endl;
  }
}

TEST(Assignment3, accuratePropagation_nonDeterministic0)
{
  std::vector<double> mean = {1, 1, 0};
  std::vector<double> dpose = {1, 0, 0};
  std::vector<double> result = {2, 1, 0};
  double odomoffset = 0;

  general_nonDet_Test(mean, dpose, result, odomoffset);
}

TEST(Assignment3, accuratePropagation_nonDeterministic1)
{
  std::vector<double> mean = {1, 1, 0};
  std::vector<double> dpose = {0, 1, 0};
  std::vector<double> result = {1, 0, 0};
  double odomoffset = 0;

  general_nonDet_Test(mean, dpose, result, odomoffset);
}

TEST(Assignment3, accuratePropagation_nonDeterministic2)
{
  std::vector<double> mean = {1, 1, 0};
  std::vector<double> dpose = {0, 0, 1};
  std::vector<double> result = {1, 1, 1};
  double odomoffset = 0;

  general_nonDet_Test(mean, dpose, result, odomoffset);
}