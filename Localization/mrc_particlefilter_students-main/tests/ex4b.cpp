#include <gtest/gtest.h>
#include <cmath>

#include "./Filters/ParticleFilter.h"
#include "./tools.h"

#include <emc/io.h>

TEST(Assignment4, likelihood_sanityCheck)
{
    int argc = 2;
    char arg0[] = "./main.cpp";
    char arg1[] = "../config/testconfig.json";
    char *argv[] = {&arg0[0], &arg1[0]};

    std::default_random_engine _generator;

    auto programConfig = load_config_file(argc, argv);
    World world = constructWorldModel(programConfig);
    MeasModelParams lm = constructMeasurementModel(programConfig);

    Particle p(world, 1.0, &_generator);

    // Check for normalised (not strictly necessary but enforced within assignment)
    EXPECT_LT(p.measurementmodel(0, 0, lm), 1);
    EXPECT_LT(p.measurementmodel(2.5, 2.5, lm), 1);
    EXPECT_LT(p.measurementmodel(10, 10, lm), 1);

    // Check for likelihood exact match greater than no exact match (due to gaussian term)
    EXPECT_LT(p.measurementmodel(0, 10, lm), p.measurementmodel(10, 10, lm));
    EXPECT_LT(p.measurementmodel(3.5, 2.5, lm), p.measurementmodel(2.5, 2.5, lm));

    // Check for short hit is more. or equally, likely than long hit (due to exponential term)
    EXPECT_LE(p.measurementmodel(6, 5, lm), p.measurementmodel(4, 5, lm));
    EXPECT_LE(p.measurementmodel(2, 1, lm), p.measurementmodel(0, 1, lm));

    // Check for likelihood always greater than zero
    EXPECT_GT(p.measurementmodel(8, 0, lm), 0);
    EXPECT_GT(p.measurementmodel(9, 2, lm), 0);
    EXPECT_GT(p.measurementmodel(2, 9, lm), 0);
    EXPECT_GT(p.measurementmodel(0, 8, lm), 0);
}
