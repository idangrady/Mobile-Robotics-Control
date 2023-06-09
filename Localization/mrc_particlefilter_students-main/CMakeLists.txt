# CMakeLists.txt
cmake_minimum_required(VERSION 3.10)
project("mrc-particlefilter")

set(DCMAKE_BUILD_TYPE,"Release")

# Fix flags for rangelibc and gtest
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_FLAGS "-march=native -ffast-math -fno-math-errno")
add_definitions(-DUSE_CUDA=0)

# Include ROS libraries
find_package(catkin REQUIRED COMPONENTS roscpp)
include_directories(${catkin_INCLUDE_DIRS})

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

# Add main executable
set(SOURCE_FILES
    src/tools.cpp
    src/Objects/World.cpp
    src/Objects/Robot.cpp
    src/FilterComponents/Particle.cpp
    src/Filters/ParticleFilter.cpp
    src/Filters/ParticleFilterBase.cpp
    src/FilterComponents/Resampler.cpp
    include/3rdparty/rangelibc/vendor/lodepng/lodepng.cpp)

add_library(particleFilterMRC ${SOURCE_FILES})
target_link_libraries(particleFilterMRC emc-framework)
target_link_libraries(particleFilterMRC pthread)
target_link_libraries(particleFilterMRC ${catkin_LIBRARIES})

target_include_directories(particleFilterMRC PUBLIC include/)
target_include_directories(particleFilterMRC SYSTEM INTERFACE include/3rdparty)

# # Complete main
add_executable(main src/main.cpp)
target_link_libraries(main particleFilterMRC)

# # Main of ex.1
add_executable(demo1 demos/main_ex1.cpp)
target_link_libraries(demo1 particleFilterMRC)

# # Main of ex.2
add_executable(demo2 demos/main_ex2.cpp)
target_link_libraries(demo2 particleFilterMRC)

# # Main of ex.3
add_executable(demo3 demos/main_ex3.cpp)
target_link_libraries(demo3 particleFilterMRC)

# # Setup gtestingc
if((NOT ${REAL_ROBOT}) OR NOT DEFINED ${REAL_ROBOT})
    include(GoogleTest)
    find_package(GTest REQUIRED)
    include_directories(${GTEST_INCLUDE_DIRS})
    enable_testing()

    add_executable(assignment1 tests/ex1.cpp)
    target_link_libraries(assignment1 particleFilterMRC)
    target_link_libraries(assignment1 ${GTEST_MAIN_LIBRARIES} ${GTEST_LIBRARIES})

    # # Test number 2
    add_executable(assignment2 tests/ex2.cpp)
    target_link_libraries(assignment2 particleFilterMRC)
    target_link_libraries(assignment2 ${GTEST_MAIN_LIBRARIES} ${GTEST_LIBRARIES})

    # # Test number 3
    add_executable(assignment3 tests/ex3.cpp tests/ex3b.cpp)
    target_link_libraries(assignment3 particleFilterMRC)
    target_link_libraries(assignment3 ${GTEST_MAIN_LIBRARIES} ${GTEST_LIBRARIES})

    # # Test number 4
    add_executable(assignment4 tests/ex4.cpp tests/ex4b.cpp)
    target_link_libraries(assignment4 particleFilterMRC)
    target_link_libraries(assignment4 ${GTEST_MAIN_LIBRARIES} ${GTEST_LIBRARIES})

    # # Tool Test
    add_executable(tooltest tests/tooltest.cpp)
    target_link_libraries(tooltest particleFilterMRC)
    target_link_libraries(tooltest ${GTEST_MAIN_LIBRARIES} ${GTEST_LIBRARIES})

    gtest_discover_tests(assignment1)
    gtest_discover_tests(assignment2)
    gtest_discover_tests(assignment3)
    gtest_discover_tests(assignment4)
    gtest_discover_tests(tooltest)
endif()
