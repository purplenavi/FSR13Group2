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
CMAKE_SOURCE_DIR = /home/group2/fuerte_workspace/fsr2013/hiekkalaatikko/imge_recognition

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/group2/fuerte_workspace/fsr2013/hiekkalaatikko/imge_recognition/build

# Include any dependencies generated for this target.
include CMakeFiles/ppl_recognition.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/ppl_recognition.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/ppl_recognition.dir/flags.make

CMakeFiles/ppl_recognition.dir/src/ppl_recognition.o: CMakeFiles/ppl_recognition.dir/flags.make
CMakeFiles/ppl_recognition.dir/src/ppl_recognition.o: ../src/ppl_recognition.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/group2/fuerte_workspace/fsr2013/hiekkalaatikko/imge_recognition/build/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/ppl_recognition.dir/src/ppl_recognition.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/ppl_recognition.dir/src/ppl_recognition.o -c /home/group2/fuerte_workspace/fsr2013/hiekkalaatikko/imge_recognition/src/ppl_recognition.cpp

CMakeFiles/ppl_recognition.dir/src/ppl_recognition.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/ppl_recognition.dir/src/ppl_recognition.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/group2/fuerte_workspace/fsr2013/hiekkalaatikko/imge_recognition/src/ppl_recognition.cpp > CMakeFiles/ppl_recognition.dir/src/ppl_recognition.i

CMakeFiles/ppl_recognition.dir/src/ppl_recognition.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/ppl_recognition.dir/src/ppl_recognition.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/group2/fuerte_workspace/fsr2013/hiekkalaatikko/imge_recognition/src/ppl_recognition.cpp -o CMakeFiles/ppl_recognition.dir/src/ppl_recognition.s

CMakeFiles/ppl_recognition.dir/src/ppl_recognition.o.requires:
.PHONY : CMakeFiles/ppl_recognition.dir/src/ppl_recognition.o.requires

CMakeFiles/ppl_recognition.dir/src/ppl_recognition.o.provides: CMakeFiles/ppl_recognition.dir/src/ppl_recognition.o.requires
	$(MAKE) -f CMakeFiles/ppl_recognition.dir/build.make CMakeFiles/ppl_recognition.dir/src/ppl_recognition.o.provides.build
.PHONY : CMakeFiles/ppl_recognition.dir/src/ppl_recognition.o.provides

CMakeFiles/ppl_recognition.dir/src/ppl_recognition.o.provides.build: CMakeFiles/ppl_recognition.dir/src/ppl_recognition.o

# Object files for target ppl_recognition
ppl_recognition_OBJECTS = \
"CMakeFiles/ppl_recognition.dir/src/ppl_recognition.o"

# External object files for target ppl_recognition
ppl_recognition_EXTERNAL_OBJECTS =

../bin/ppl_recognition: CMakeFiles/ppl_recognition.dir/src/ppl_recognition.o
../bin/ppl_recognition: /usr/lib/libboost_system-mt.so
../bin/ppl_recognition: /usr/lib/libboost_filesystem-mt.so
../bin/ppl_recognition: /usr/lib/libboost_thread-mt.so
../bin/ppl_recognition: /usr/lib/libboost_date_time-mt.so
../bin/ppl_recognition: /usr/lib/libboost_iostreams-mt.so
../bin/ppl_recognition: /opt/ros/fuerte/lib/libpcl_common.so
../bin/ppl_recognition: /opt/ros/fuerte/lib/libpcl_octree.so
../bin/ppl_recognition: /usr/lib/libvtkCommon.so.5.8.0
../bin/ppl_recognition: /usr/lib/libvtkRendering.so.5.8.0
../bin/ppl_recognition: /usr/lib/libvtkHybrid.so.5.8.0
../bin/ppl_recognition: /opt/ros/fuerte/lib/libpcl_io.so
../bin/ppl_recognition: /usr/lib/libboost_system-mt.so
../bin/ppl_recognition: /usr/lib/libboost_filesystem-mt.so
../bin/ppl_recognition: /usr/lib/libboost_thread-mt.so
../bin/ppl_recognition: /usr/lib/libboost_date_time-mt.so
../bin/ppl_recognition: /usr/lib/libboost_iostreams-mt.so
../bin/ppl_recognition: /opt/ros/fuerte/lib/libpcl_common.so
../bin/ppl_recognition: /usr/lib/libvtkCommon.so.5.8.0
../bin/ppl_recognition: /usr/lib/libvtkRendering.so.5.8.0
../bin/ppl_recognition: /usr/lib/libvtkHybrid.so.5.8.0
../bin/ppl_recognition: /opt/ros/fuerte/lib/libpcl_io.so
../bin/ppl_recognition: /usr/lib/libboost_system-mt.so
../bin/ppl_recognition: /usr/lib/libboost_filesystem-mt.so
../bin/ppl_recognition: /usr/lib/libboost_thread-mt.so
../bin/ppl_recognition: /usr/lib/libboost_date_time-mt.so
../bin/ppl_recognition: /usr/lib/libboost_iostreams-mt.so
../bin/ppl_recognition: /opt/ros/fuerte/lib/libpcl_common.so
../bin/ppl_recognition: /opt/ros/fuerte/lib/libpcl_octree.so
../bin/ppl_recognition: /usr/lib/libboost_system-mt.so
../bin/ppl_recognition: /usr/lib/libboost_filesystem-mt.so
../bin/ppl_recognition: /usr/lib/libboost_thread-mt.so
../bin/ppl_recognition: /usr/lib/libboost_date_time-mt.so
../bin/ppl_recognition: /usr/lib/libboost_iostreams-mt.so
../bin/ppl_recognition: /opt/ros/fuerte/lib/libpcl_common.so
../bin/ppl_recognition: /usr/lib/libvtkParallel.so.5.8.0
../bin/ppl_recognition: /usr/lib/libvtkRendering.so.5.8.0
../bin/ppl_recognition: /usr/lib/libvtkGraphics.so.5.8.0
../bin/ppl_recognition: /usr/lib/libvtkImaging.so.5.8.0
../bin/ppl_recognition: /usr/lib/libvtkIO.so.5.8.0
../bin/ppl_recognition: /usr/lib/libvtkFiltering.so.5.8.0
../bin/ppl_recognition: /usr/lib/libvtkCommon.so.5.8.0
../bin/ppl_recognition: /usr/lib/libvtksys.so.5.8.0
../bin/ppl_recognition: CMakeFiles/ppl_recognition.dir/build.make
../bin/ppl_recognition: CMakeFiles/ppl_recognition.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX executable ../bin/ppl_recognition"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/ppl_recognition.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/ppl_recognition.dir/build: ../bin/ppl_recognition
.PHONY : CMakeFiles/ppl_recognition.dir/build

CMakeFiles/ppl_recognition.dir/requires: CMakeFiles/ppl_recognition.dir/src/ppl_recognition.o.requires
.PHONY : CMakeFiles/ppl_recognition.dir/requires

CMakeFiles/ppl_recognition.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/ppl_recognition.dir/cmake_clean.cmake
.PHONY : CMakeFiles/ppl_recognition.dir/clean

CMakeFiles/ppl_recognition.dir/depend:
	cd /home/group2/fuerte_workspace/fsr2013/hiekkalaatikko/imge_recognition/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/group2/fuerte_workspace/fsr2013/hiekkalaatikko/imge_recognition /home/group2/fuerte_workspace/fsr2013/hiekkalaatikko/imge_recognition /home/group2/fuerte_workspace/fsr2013/hiekkalaatikko/imge_recognition/build /home/group2/fuerte_workspace/fsr2013/hiekkalaatikko/imge_recognition/build /home/group2/fuerte_workspace/fsr2013/hiekkalaatikko/imge_recognition/build/CMakeFiles/ppl_recognition.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/ppl_recognition.dir/depend
