# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.21

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:

#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:

# Disable VCS-based implicit rules.
% : %,v

# Disable VCS-based implicit rules.
% : RCS/%

# Disable VCS-based implicit rules.
% : RCS/%,v

# Disable VCS-based implicit rules.
% : SCCS/s.%

# Disable VCS-based implicit rules.
% : s.%

.SUFFIXES: .hpux_make_needs_suffix_list

# Command-line flag to silence nested $(MAKE).
$(VERBOSE)MAKESILENT = -s

#Suppress display of executed commands.
$(VERBOSE).SILENT:

# A target that is always out of date.
cmake_force:
.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /opt/cmake-3.21.4/bin/cmake

# The command to remove a file.
RM = /opt/cmake-3.21.4/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/rrqq/towr_workspace/src/towr/towr

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/rrqq/towr_workspace/build/towr

# Include any dependencies generated for this target.
include CMakeFiles/towr-example.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include CMakeFiles/towr-example.dir/compiler_depend.make

# Include the progress variables for this target.
include CMakeFiles/towr-example.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/towr-example.dir/flags.make

CMakeFiles/towr-example.dir/test/hopper_example.cc.o: CMakeFiles/towr-example.dir/flags.make
CMakeFiles/towr-example.dir/test/hopper_example.cc.o: /home/rrqq/towr_workspace/src/towr/towr/test/hopper_example.cc
CMakeFiles/towr-example.dir/test/hopper_example.cc.o: CMakeFiles/towr-example.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/rrqq/towr_workspace/build/towr/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/towr-example.dir/test/hopper_example.cc.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/towr-example.dir/test/hopper_example.cc.o -MF CMakeFiles/towr-example.dir/test/hopper_example.cc.o.d -o CMakeFiles/towr-example.dir/test/hopper_example.cc.o -c /home/rrqq/towr_workspace/src/towr/towr/test/hopper_example.cc

CMakeFiles/towr-example.dir/test/hopper_example.cc.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/towr-example.dir/test/hopper_example.cc.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/rrqq/towr_workspace/src/towr/towr/test/hopper_example.cc > CMakeFiles/towr-example.dir/test/hopper_example.cc.i

CMakeFiles/towr-example.dir/test/hopper_example.cc.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/towr-example.dir/test/hopper_example.cc.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/rrqq/towr_workspace/src/towr/towr/test/hopper_example.cc -o CMakeFiles/towr-example.dir/test/hopper_example.cc.s

# Object files for target towr-example
towr__example_OBJECTS = \
"CMakeFiles/towr-example.dir/test/hopper_example.cc.o"

# External object files for target towr-example
towr__example_EXTERNAL_OBJECTS =

towr-example: CMakeFiles/towr-example.dir/test/hopper_example.cc.o
towr-example: CMakeFiles/towr-example.dir/build.make
towr-example: libtowr.so
towr-example: /home/rrqq/towr_workspace/devel/lib/libifopt_ipopt.so
towr-example: /home/rrqq/towr_workspace/devel/lib/libifopt_core.so
towr-example: CMakeFiles/towr-example.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/rrqq/towr_workspace/build/towr/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable towr-example"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/towr-example.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/towr-example.dir/build: towr-example
.PHONY : CMakeFiles/towr-example.dir/build

CMakeFiles/towr-example.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/towr-example.dir/cmake_clean.cmake
.PHONY : CMakeFiles/towr-example.dir/clean

CMakeFiles/towr-example.dir/depend:
	cd /home/rrqq/towr_workspace/build/towr && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/rrqq/towr_workspace/src/towr/towr /home/rrqq/towr_workspace/src/towr/towr /home/rrqq/towr_workspace/build/towr /home/rrqq/towr_workspace/build/towr /home/rrqq/towr_workspace/build/towr/CMakeFiles/towr-example.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/towr-example.dir/depend
