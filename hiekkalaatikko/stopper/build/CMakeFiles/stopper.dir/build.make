# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 2.8

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

# The program to use to edit the cache.
CMAKE_EDIT_COMMAND = /usr/bin/ccmake

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/group2/fuerte_workspace/fsr2013/hiekkalaatikko/stopper

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/group2/fuerte_workspace/fsr2013/hiekkalaatikko/stopper/build

# Include any dependencies generated for this target.
include CMakeFiles/stopper.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/stopper.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/stopper.dir/flags.make

CMakeFiles/stopper.dir/src/stopper.o: CMakeFiles/stopper.dir/flags.make
CMakeFiles/stopper.dir/src/stopper.o: ../src/stopper.cpp
CMakeFiles/stopper.dir/src/stopper.o: ../manifest.xml
CMakeFiles/stopper.dir/src/stopper.o: /opt/ros/fuerte/share/roslang/manifest.xml
CMakeFiles/stopper.dir/src/stopper.o: /opt/ros/fuerte/share/roscpp/manifest.xml
	$(CMAKE_COMMAND) -E cmake_progress_report /home/group2/fuerte_workspace/fsr2013/hiekkalaatikko/stopper/build/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/stopper.dir/src/stopper.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -W -Wall -Wno-unused-parameter -fno-strict-aliasing -pthread -o CMakeFiles/stopper.dir/src/stopper.o -c /home/group2/fuerte_workspace/fsr2013/hiekkalaatikko/stopper/src/stopper.cpp

CMakeFiles/stopper.dir/src/stopper.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/stopper.dir/src/stopper.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -W -Wall -Wno-unused-parameter -fno-strict-aliasing -pthread -E /home/group2/fuerte_workspace/fsr2013/hiekkalaatikko/stopper/src/stopper.cpp > CMakeFiles/stopper.dir/src/stopper.i

CMakeFiles/stopper.dir/src/stopper.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/stopper.dir/src/stopper.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -W -Wall -Wno-unused-parameter -fno-strict-aliasing -pthread -S /home/group2/fuerte_workspace/fsr2013/hiekkalaatikko/stopper/src/stopper.cpp -o CMakeFiles/stopper.dir/src/stopper.s

CMakeFiles/stopper.dir/src/stopper.o.requires:
.PHONY : CMakeFiles/stopper.dir/src/stopper.o.requires

CMakeFiles/stopper.dir/src/stopper.o.provides: CMakeFiles/stopper.dir/src/stopper.o.requires
	$(MAKE) -f CMakeFiles/stopper.dir/build.make CMakeFiles/stopper.dir/src/stopper.o.provides.build
.PHONY : CMakeFiles/stopper.dir/src/stopper.o.provides

CMakeFiles/stopper.dir/src/stopper.o.provides.build: CMakeFiles/stopper.dir/src/stopper.o

# Object files for target stopper
stopper_OBJECTS = \
"CMakeFiles/stopper.dir/src/stopper.o"

# External object files for target stopper
stopper_EXTERNAL_OBJECTS =

../bin/stopper: CMakeFiles/stopper.dir/src/stopper.o
../bin/stopper: CMakeFiles/stopper.dir/build.make
../bin/stopper: CMakeFiles/stopper.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX executable ../bin/stopper"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/stopper.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/stopper.dir/build: ../bin/stopper
.PHONY : CMakeFiles/stopper.dir/build

CMakeFiles/stopper.dir/requires: CMakeFiles/stopper.dir/src/stopper.o.requires
.PHONY : CMakeFiles/stopper.dir/requires

CMakeFiles/stopper.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/stopper.dir/cmake_clean.cmake
.PHONY : CMakeFiles/stopper.dir/clean

CMakeFiles/stopper.dir/depend:
	cd /home/group2/fuerte_workspace/fsr2013/hiekkalaatikko/stopper/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/group2/fuerte_workspace/fsr2013/hiekkalaatikko/stopper /home/group2/fuerte_workspace/fsr2013/hiekkalaatikko/stopper /home/group2/fuerte_workspace/fsr2013/hiekkalaatikko/stopper/build /home/group2/fuerte_workspace/fsr2013/hiekkalaatikko/stopper/build /home/group2/fuerte_workspace/fsr2013/hiekkalaatikko/stopper/build/CMakeFiles/stopper.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/stopper.dir/depend

