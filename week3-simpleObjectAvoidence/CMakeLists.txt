cmake_minimum_required(VERSION 2.8)
project(week1)

# Find Python and NumPy
find_package(Python3 COMPONENTS Interpreter Development NumPy REQUIRED)
include_directories(${Python3_INCLUDE_DIRS} include)

# Find OpenCV
find_package(OpenCV REQUIRED)

# Set the source files and executable
set(SOURCES src/hello_work.cpp)

add_executable(hello ${SOURCES})

# Set the output directory for the executable
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

# Link libraries
target_link_libraries(hello emc-framework ${Python3_LIBRARIES} ${OpenCV_LIBS})

