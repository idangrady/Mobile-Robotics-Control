#include <gtest/gtest.h>
#include <cmath>

#include "./Filters/ParticleFilter.h"
#include "./tools.h"

void verifyInBounds(const ParticleFilter &pfilt, const World &world)
{
  int N = pfilt.getNumberParticles();
  double wzx = world.get_size_x();
  double wzy = world.get_size_y();

  // Verify that all particles are within bounds
  for (int i = 0; i < N; i++)
  {
    Particle particle_i = pfilt.getParticle(i);
    Pose pose_i = particle_i.getPosition();

    EXPECT_GT(pose_i[0], 0);
    EXPECT_LT(pose_i[0], wzx);
    EXPECT_GT(pose_i[1], 0);
    EXPECT_LT(pose_i[1], wzy);
    EXPECT_GT(pose_i[2], -M_PI);
    EXPECT_LT(pose_i[2], M_PI);
  }
}

void calculateMeanPosition(const ParticleFilter &pfilt, double &meanx, double &meany)
{
  int N = pfilt.getNumberParticles();
  for (int i = 0; i < N; i++)
  {
    Particle particle_i = pfilt.getParticle(i);
    Pose pose_i = particle_i.getPosition();

    meanx += 1 / double(N) * pose_i[0];
    meany += 1 / double(N) * pose_i[1];
  }
}

void calculateSD(const ParticleFilter &pfilt, double &meanx, double &meany, double &SDx, double &SDy)
{
  int N = pfilt.getNumberParticles();
  for (int i = 0; i < N; i++)
  {
    Particle particle_i = pfilt.getParticle(i);
    Pose pose_i = particle_i.getPosition();

    SDx += 1 / double(N) * ((pose_i[0] - meanx) * (pose_i[0] - meanx));
    SDy += 1 / double(N) * ((pose_i[1] - meany) * (pose_i[1] - meany));
  }
  SDx = std::sqrt(SDx);
  SDy = std::sqrt(SDy);
}

TEST(Assignment1, SizeOfFilter)
{

  int argc = 1;
  char arg0[] = "./main.cpp";
  char *argv[] = {&arg0[0]};

  auto programConfig = load_config_file(argc, argv);
  World world = constructWorldModel(programConfig);

  // Make sure particles have equal weight upon construction
  std::vector<double> Ns = {1e2, 321, 1, 0, 1e3, 1e4};

  for (double N : Ns)
  {
    ParticleFilter pfilt(world, N);
    EXPECT_EQ(pfilt.getNumberParticles(), N) << "Possibly Not enough, or to much Particles in the Filter" << std::endl;
    EXPECT_EQ(pfilt.getNumberParticles(), pfilt._particles.size()) << "Possibly Not enough, or to much Particles in the Filter" << std::endl;
  }
}

TEST(Assignment1, weightofparticles)
{

  int argc = 1;
  char arg0[] = "./main.cpp";
  char *argv[] = {&arg0[0]};

  auto programConfig = load_config_file(argc, argv);
  World world = constructWorldModel(programConfig);

  int N = 1e4;
  ParticleFilter pfilt(world, N);
  // Make sure particles have equal weight upon construction
  for (int i = 0; i < N; i++)
  {
    try
    {
      Particle particle_i = pfilt.getParticle(i);
      double correctWeight = 1 / double(N);
      EXPECT_EQ(particle_i.getWeight(), correctWeight);
    }
    catch (std::invalid_argument const &e)
    {
      FAIL() << "Prevented a likely Segfault: Possibly Not enough Particles in the Filter" << std::endl;
    }
  }
}

TEST(Assignment1, uniformParticleSpread)
{
  int argc = 1;
  char arg0[] = "./main.cpp";
  char *argv[] = {&arg0[0]};

  auto programConfig = load_config_file(argc, argv);
  World world = constructWorldModel(programConfig);

  int N = 1e4;
  ParticleFilter pfilt(world, N);

  verifyInBounds(pfilt, world);
  // Calculate the mean of the particle locations (not orientation)
  double meanx = 0;
  double meany = 0;
  calculateMeanPosition(pfilt, meanx, meany);

  double world_size_x = world.get_size_x();
  double world_size_y = world.get_size_y();

  EXPECT_GT(meanx, 0);
  EXPECT_GT(meany, 0);
  EXPECT_LT(meanx, world_size_x);
  EXPECT_LT(meany, world_size_y);

  EXPECT_NEAR(meanx, world_size_x / 2, world_size_x / 10);
  EXPECT_NEAR(meany, world_size_y / 2, world_size_y / 10);
  // Calculate the SD of the particle locations
  double SDx = 0;
  double SDy = 0;
  calculateSD(pfilt, meanx, meany, SDx, SDy);

  EXPECT_GT(SDx, 0);
  EXPECT_GT(SDy, 0);
  EXPECT_LT(SDx, world_size_x);
  EXPECT_LT(SDy, world_size_y);
}

TEST(Assignment1, gaussianParticleSpread)
{
  int argc = 1;
  char arg0[] = "./main.cpp";
  char *argv[] = {&arg0[0]};

  auto programConfig = load_config_file(argc, argv);
  World world = constructWorldModel(programConfig);

  std::vector<std::vector<double>> means = {{1, 0, 0}, {0.33, 0.5, 0}, {1, 1, 0}, {5, 3, 0}, {3, 5, 0}};
  std::vector<std::vector<double>> sigmas = {{0, 0, 0.1}, {1, 1, 0.5}, {1, 1, 0.1}, {3.25, 2.5, 0.25}, {0, 5, 0}};

  for (int i = 0; i < means.size(); i++)
  {
    double mean_i[3] = {means[i][0], means[i][1], means[i][2]};
    double sigma_i[3] = {sigmas[i][0], sigmas[i][1], sigmas[i][2]};

    int N = 1e4;
    ParticleFilter pfilt(world, mean_i, sigma_i, N);

    // Calculate the mean of the particle locations (not orientation)
    double meanx = 0;
    double meany = 0;
    calculateMeanPosition(pfilt, meanx, meany);

    EXPECT_NEAR(meanx, means[i][0], 5e-2);
    EXPECT_NEAR(meany, means[i][1], 5e-2);
    // Calculate the SD of the particle locations
    double SDx = 0;
    double SDy = 0;

    calculateSD(pfilt, meanx, meany, SDx, SDy);
    EXPECT_NEAR(SDx, sigmas[i][0], 5e-2);
    EXPECT_NEAR(SDy, sigmas[i][1], 5e-2);
  }
}
