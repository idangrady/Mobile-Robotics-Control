# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.16

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:


#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:


# Remove some rules from gmake that .SUFFIXES does not remove.
SUFFIXES =

.SUFFIXES: .hpux_make_needs_suffix_list


# Suppress display of executed commands.
$(VERBOSE).SILENT:


# A target that is always out of date.
cmake_force:

.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /usr/bin/cmake

# The command to remove a file.
RM = /usr/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/idangrady/Documents/Github/MRC/Project2/mrc-navigation-assignment-1-main

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/idangrady/Documents/Github/MRC/Project2/mrc-navigation-assignment-1-main/build

# Include any dependencies generated for this target.
include CMakeFiles/tooltest.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/tooltest.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/tooltest.dir/flags.make

CMakeFiles/tooltest.dir/tests/tooltest.cpp.o: CMakeFiles/tooltest.dir/flags.make
CMakeFiles/tooltest.dir/tests/tooltest.cpp.o: ../tests/tooltest.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/idangrady/Documents/Github/MRC/Project2/mrc-navigation-assignment-1-main/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/tooltest.dir/tests/tooltest.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/tooltest.dir/tests/tooltest.cpp.o -c /home/idangrady/Documents/Github/MRC/Project2/mrc-navigation-assignment-1-main/tests/tooltest.cpp

CMakeFiles/tooltest.dir/tests/tooltest.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/tooltest.dir/tests/tooltest.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/idangrady/Documents/Github/MRC/Project2/mrc-navigation-assignment-1-main/tests/tooltest.cpp > CMakeFiles/tooltest.dir/tests/tooltest.cpp.i

CMakeFiles/tooltest.dir/tests/tooltest.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/tooltest.dir/tests/tooltest.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/idangrady/Documents/Github/MRC/Project2/mrc-navigation-assignment-1-main/tests/tooltest.cpp -o CMakeFiles/tooltest.dir/tests/tooltest.cpp.s

# Object files for target tooltest
tooltest_OBJECTS = \
"CMakeFiles/tooltest.dir/tests/tooltest.cpp.o"

# External object files for target tooltest
tooltest_EXTERNAL_OBJECTS =

../bin/tooltest: CMakeFiles/tooltest.dir/tests/tooltest.cpp.o
../bin/tooltest: CMakeFiles/tooltest.dir/build.make
../bin/tooltest: libpathPlanningMRC.so
../bin/tooltest: /usr/lib/x86_64-linux-gnu/libgtest_main.a
../bin/tooltest: /usr/lib/x86_64-linux-gnu/libgtest.a
../bin/tooltest: /opt/ros/noetic/lib/libroscpp.so
../bin/tooltest: /usr/lib/x86_64-linux-gnu/libpthread.so
../bin/tooltest: /usr/lib/x86_64-linux-gnu/libboost_chrono.so.1.71.0
../bin/tooltest: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so.1.71.0
../bin/tooltest: /opt/ros/noetic/lib/librosconsole.so
../bin/tooltest: /opt/ros/noetic/lib/librosconsole_log4cxx.so
../bin/tooltest: /opt/ros/noetic/lib/librosconsole_backend_interface.so
../bin/tooltest: /usr/lib/x86_64-linux-gnu/liblog4cxx.so
../bin/tooltest: /usr/lib/x86_64-linux-gnu/libboost_regex.so.1.71.0
../bin/tooltest: /opt/ros/noetic/lib/libroscpp_serialization.so
../bin/tooltest: /opt/ros/noetic/lib/libxmlrpcpp.so
../bin/tooltest: /opt/ros/noetic/lib/librostime.so
../bin/tooltest: /usr/lib/x86_64-linux-gnu/libboost_date_time.so.1.71.0
../bin/tooltest: /opt/ros/noetic/lib/libcpp_common.so
../bin/tooltest: /usr/lib/x86_64-linux-gnu/libboost_system.so.1.71.0
../bin/tooltest: /usr/lib/x86_64-linux-gnu/libboost_thread.so.1.71.0
../bin/tooltest: /usr/lib/x86_64-linux-gnu/libconsole_bridge.so.0.4
../bin/tooltest: CMakeFiles/tooltest.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/idangrady/Documents/Github/MRC/Project2/mrc-navigation-assignment-1-main/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable ../bin/tooltest"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/tooltest.dir/link.txt --verbose=$(VERBOSE)
	/usr/bin/cmake -D TEST_TARGET=tooltest -D TEST_EXECUTABLE=/home/idangrady/Documents/Github/MRC/Project2/mrc-navigation-assignment-1-main/bin/tooltest -D TEST_EXECUTOR= -D TEST_WORKING_DIR=/home/idangrady/Documents/Github/MRC/Project2/mrc-navigation-assignment-1-main/build -D TEST_EXTRA_ARGS= -D TEST_PROPERTIES= -D TEST_PREFIX= -D TEST_SUFFIX= -D NO_PRETTY_TYPES=FALSE -D NO_PRETTY_VALUES=FALSE -D TEST_LIST=tooltest_TESTS -D CTEST_FILE=/home/idangrady/Documents/Github/MRC/Project2/mrc-navigation-assignment-1-main/build/tooltest[1]_tests.cmake -D TEST_DISCOVERY_TIMEOUT=5 -P /usr/share/cmake-3.16/Modules/GoogleTestAddTests.cmake

# Rule to build all files generated by this target.
CMakeFiles/tooltest.dir/build: ../bin/tooltest

.PHONY : CMakeFiles/tooltest.dir/build

CMakeFiles/tooltest.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/tooltest.dir/cmake_clean.cmake
.PHONY : CMakeFiles/tooltest.dir/clean

CMakeFiles/tooltest.dir/depend:
	cd /home/idangrady/Documents/Github/MRC/Project2/mrc-navigation-assignment-1-main/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/idangrady/Documents/Github/MRC/Project2/mrc-navigation-assignment-1-main /home/idangrady/Documents/Github/MRC/Project2/mrc-navigation-assignment-1-main /home/idangrady/Documents/Github/MRC/Project2/mrc-navigation-assignment-1-main/build /home/idangrady/Documents/Github/MRC/Project2/mrc-navigation-assignment-1-main/build /home/idangrady/Documents/Github/MRC/Project2/mrc-navigation-assignment-1-main/build/CMakeFiles/tooltest.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/tooltest.dir/depend

