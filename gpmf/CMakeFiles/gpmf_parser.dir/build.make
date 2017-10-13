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

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/ubuntu/temp/gopro-lib-imu-analysis

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/ubuntu/temp/gopro-lib-imu-analysis

# Include any dependencies generated for this target.
include gpmf/CMakeFiles/gpmf_parser.dir/depend.make

# Include the progress variables for this target.
include gpmf/CMakeFiles/gpmf_parser.dir/progress.make

# Include the compile flags for this target's objects.
include gpmf/CMakeFiles/gpmf_parser.dir/flags.make

gpmf/CMakeFiles/gpmf_parser.dir/gpmf_parser.cpp.o: gpmf/CMakeFiles/gpmf_parser.dir/flags.make
gpmf/CMakeFiles/gpmf_parser.dir/gpmf_parser.cpp.o: gpmf/gpmf_parser.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/ubuntu/temp/gopro-lib-imu-analysis/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object gpmf/CMakeFiles/gpmf_parser.dir/gpmf_parser.cpp.o"
	cd /home/ubuntu/temp/gopro-lib-imu-analysis/gpmf && /usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/gpmf_parser.dir/gpmf_parser.cpp.o -c /home/ubuntu/temp/gopro-lib-imu-analysis/gpmf/gpmf_parser.cpp

gpmf/CMakeFiles/gpmf_parser.dir/gpmf_parser.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/gpmf_parser.dir/gpmf_parser.cpp.i"
	cd /home/ubuntu/temp/gopro-lib-imu-analysis/gpmf && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/ubuntu/temp/gopro-lib-imu-analysis/gpmf/gpmf_parser.cpp > CMakeFiles/gpmf_parser.dir/gpmf_parser.cpp.i

gpmf/CMakeFiles/gpmf_parser.dir/gpmf_parser.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/gpmf_parser.dir/gpmf_parser.cpp.s"
	cd /home/ubuntu/temp/gopro-lib-imu-analysis/gpmf && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/ubuntu/temp/gopro-lib-imu-analysis/gpmf/gpmf_parser.cpp -o CMakeFiles/gpmf_parser.dir/gpmf_parser.cpp.s

gpmf/CMakeFiles/gpmf_parser.dir/gpmf_parser.cpp.o.requires:
.PHONY : gpmf/CMakeFiles/gpmf_parser.dir/gpmf_parser.cpp.o.requires

gpmf/CMakeFiles/gpmf_parser.dir/gpmf_parser.cpp.o.provides: gpmf/CMakeFiles/gpmf_parser.dir/gpmf_parser.cpp.o.requires
	$(MAKE) -f gpmf/CMakeFiles/gpmf_parser.dir/build.make gpmf/CMakeFiles/gpmf_parser.dir/gpmf_parser.cpp.o.provides.build
.PHONY : gpmf/CMakeFiles/gpmf_parser.dir/gpmf_parser.cpp.o.provides

gpmf/CMakeFiles/gpmf_parser.dir/gpmf_parser.cpp.o.provides.build: gpmf/CMakeFiles/gpmf_parser.dir/gpmf_parser.cpp.o

# Object files for target gpmf_parser
gpmf_parser_OBJECTS = \
"CMakeFiles/gpmf_parser.dir/gpmf_parser.cpp.o"

# External object files for target gpmf_parser
gpmf_parser_EXTERNAL_OBJECTS =

gpmf/libgpmf_parser.so: gpmf/CMakeFiles/gpmf_parser.dir/gpmf_parser.cpp.o
gpmf/libgpmf_parser.so: gpmf/CMakeFiles/gpmf_parser.dir/build.make
gpmf/libgpmf_parser.so: gpmf/CMakeFiles/gpmf_parser.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX shared library libgpmf_parser.so"
	cd /home/ubuntu/temp/gopro-lib-imu-analysis/gpmf && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/gpmf_parser.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
gpmf/CMakeFiles/gpmf_parser.dir/build: gpmf/libgpmf_parser.so
.PHONY : gpmf/CMakeFiles/gpmf_parser.dir/build

gpmf/CMakeFiles/gpmf_parser.dir/requires: gpmf/CMakeFiles/gpmf_parser.dir/gpmf_parser.cpp.o.requires
.PHONY : gpmf/CMakeFiles/gpmf_parser.dir/requires

gpmf/CMakeFiles/gpmf_parser.dir/clean:
	cd /home/ubuntu/temp/gopro-lib-imu-analysis/gpmf && $(CMAKE_COMMAND) -P CMakeFiles/gpmf_parser.dir/cmake_clean.cmake
.PHONY : gpmf/CMakeFiles/gpmf_parser.dir/clean

gpmf/CMakeFiles/gpmf_parser.dir/depend:
	cd /home/ubuntu/temp/gopro-lib-imu-analysis && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/ubuntu/temp/gopro-lib-imu-analysis /home/ubuntu/temp/gopro-lib-imu-analysis/gpmf /home/ubuntu/temp/gopro-lib-imu-analysis /home/ubuntu/temp/gopro-lib-imu-analysis/gpmf /home/ubuntu/temp/gopro-lib-imu-analysis/gpmf/CMakeFiles/gpmf_parser.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : gpmf/CMakeFiles/gpmf_parser.dir/depend
