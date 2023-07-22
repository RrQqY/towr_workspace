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
CMAKE_SOURCE_DIR = /home/rrqq/towr_workspace/src/ifopt

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/rrqq/towr_workspace/build/ifopt

# Include any dependencies generated for this target.
include ifopt_core/CMakeFiles/ifopt_core.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include ifopt_core/CMakeFiles/ifopt_core.dir/compiler_depend.make

# Include the progress variables for this target.
include ifopt_core/CMakeFiles/ifopt_core.dir/progress.make

# Include the compile flags for this target's objects.
include ifopt_core/CMakeFiles/ifopt_core.dir/flags.make

ifopt_core/CMakeFiles/ifopt_core.dir/src/problem.cc.o: ifopt_core/CMakeFiles/ifopt_core.dir/flags.make
ifopt_core/CMakeFiles/ifopt_core.dir/src/problem.cc.o: /home/rrqq/towr_workspace/src/ifopt/ifopt_core/src/problem.cc
ifopt_core/CMakeFiles/ifopt_core.dir/src/problem.cc.o: ifopt_core/CMakeFiles/ifopt_core.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/rrqq/towr_workspace/build/ifopt/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object ifopt_core/CMakeFiles/ifopt_core.dir/src/problem.cc.o"
	cd /home/rrqq/towr_workspace/build/ifopt/ifopt_core && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT ifopt_core/CMakeFiles/ifopt_core.dir/src/problem.cc.o -MF CMakeFiles/ifopt_core.dir/src/problem.cc.o.d -o CMakeFiles/ifopt_core.dir/src/problem.cc.o -c /home/rrqq/towr_workspace/src/ifopt/ifopt_core/src/problem.cc

ifopt_core/CMakeFiles/ifopt_core.dir/src/problem.cc.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/ifopt_core.dir/src/problem.cc.i"
	cd /home/rrqq/towr_workspace/build/ifopt/ifopt_core && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/rrqq/towr_workspace/src/ifopt/ifopt_core/src/problem.cc > CMakeFiles/ifopt_core.dir/src/problem.cc.i

ifopt_core/CMakeFiles/ifopt_core.dir/src/problem.cc.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/ifopt_core.dir/src/problem.cc.s"
	cd /home/rrqq/towr_workspace/build/ifopt/ifopt_core && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/rrqq/towr_workspace/src/ifopt/ifopt_core/src/problem.cc -o CMakeFiles/ifopt_core.dir/src/problem.cc.s

ifopt_core/CMakeFiles/ifopt_core.dir/src/composite.cc.o: ifopt_core/CMakeFiles/ifopt_core.dir/flags.make
ifopt_core/CMakeFiles/ifopt_core.dir/src/composite.cc.o: /home/rrqq/towr_workspace/src/ifopt/ifopt_core/src/composite.cc
ifopt_core/CMakeFiles/ifopt_core.dir/src/composite.cc.o: ifopt_core/CMakeFiles/ifopt_core.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/rrqq/towr_workspace/build/ifopt/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object ifopt_core/CMakeFiles/ifopt_core.dir/src/composite.cc.o"
	cd /home/rrqq/towr_workspace/build/ifopt/ifopt_core && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT ifopt_core/CMakeFiles/ifopt_core.dir/src/composite.cc.o -MF CMakeFiles/ifopt_core.dir/src/composite.cc.o.d -o CMakeFiles/ifopt_core.dir/src/composite.cc.o -c /home/rrqq/towr_workspace/src/ifopt/ifopt_core/src/composite.cc

ifopt_core/CMakeFiles/ifopt_core.dir/src/composite.cc.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/ifopt_core.dir/src/composite.cc.i"
	cd /home/rrqq/towr_workspace/build/ifopt/ifopt_core && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/rrqq/towr_workspace/src/ifopt/ifopt_core/src/composite.cc > CMakeFiles/ifopt_core.dir/src/composite.cc.i

ifopt_core/CMakeFiles/ifopt_core.dir/src/composite.cc.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/ifopt_core.dir/src/composite.cc.s"
	cd /home/rrqq/towr_workspace/build/ifopt/ifopt_core && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/rrqq/towr_workspace/src/ifopt/ifopt_core/src/composite.cc -o CMakeFiles/ifopt_core.dir/src/composite.cc.s

ifopt_core/CMakeFiles/ifopt_core.dir/src/leaves.cc.o: ifopt_core/CMakeFiles/ifopt_core.dir/flags.make
ifopt_core/CMakeFiles/ifopt_core.dir/src/leaves.cc.o: /home/rrqq/towr_workspace/src/ifopt/ifopt_core/src/leaves.cc
ifopt_core/CMakeFiles/ifopt_core.dir/src/leaves.cc.o: ifopt_core/CMakeFiles/ifopt_core.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/rrqq/towr_workspace/build/ifopt/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object ifopt_core/CMakeFiles/ifopt_core.dir/src/leaves.cc.o"
	cd /home/rrqq/towr_workspace/build/ifopt/ifopt_core && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT ifopt_core/CMakeFiles/ifopt_core.dir/src/leaves.cc.o -MF CMakeFiles/ifopt_core.dir/src/leaves.cc.o.d -o CMakeFiles/ifopt_core.dir/src/leaves.cc.o -c /home/rrqq/towr_workspace/src/ifopt/ifopt_core/src/leaves.cc

ifopt_core/CMakeFiles/ifopt_core.dir/src/leaves.cc.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/ifopt_core.dir/src/leaves.cc.i"
	cd /home/rrqq/towr_workspace/build/ifopt/ifopt_core && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/rrqq/towr_workspace/src/ifopt/ifopt_core/src/leaves.cc > CMakeFiles/ifopt_core.dir/src/leaves.cc.i

ifopt_core/CMakeFiles/ifopt_core.dir/src/leaves.cc.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/ifopt_core.dir/src/leaves.cc.s"
	cd /home/rrqq/towr_workspace/build/ifopt/ifopt_core && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/rrqq/towr_workspace/src/ifopt/ifopt_core/src/leaves.cc -o CMakeFiles/ifopt_core.dir/src/leaves.cc.s

# Object files for target ifopt_core
ifopt_core_OBJECTS = \
"CMakeFiles/ifopt_core.dir/src/problem.cc.o" \
"CMakeFiles/ifopt_core.dir/src/composite.cc.o" \
"CMakeFiles/ifopt_core.dir/src/leaves.cc.o"

# External object files for target ifopt_core
ifopt_core_EXTERNAL_OBJECTS =

ifopt_core/libifopt_core.so: ifopt_core/CMakeFiles/ifopt_core.dir/src/problem.cc.o
ifopt_core/libifopt_core.so: ifopt_core/CMakeFiles/ifopt_core.dir/src/composite.cc.o
ifopt_core/libifopt_core.so: ifopt_core/CMakeFiles/ifopt_core.dir/src/leaves.cc.o
ifopt_core/libifopt_core.so: ifopt_core/CMakeFiles/ifopt_core.dir/build.make
ifopt_core/libifopt_core.so: ifopt_core/CMakeFiles/ifopt_core.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/rrqq/towr_workspace/build/ifopt/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Linking CXX shared library libifopt_core.so"
	cd /home/rrqq/towr_workspace/build/ifopt/ifopt_core && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/ifopt_core.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
ifopt_core/CMakeFiles/ifopt_core.dir/build: ifopt_core/libifopt_core.so
.PHONY : ifopt_core/CMakeFiles/ifopt_core.dir/build

ifopt_core/CMakeFiles/ifopt_core.dir/clean:
	cd /home/rrqq/towr_workspace/build/ifopt/ifopt_core && $(CMAKE_COMMAND) -P CMakeFiles/ifopt_core.dir/cmake_clean.cmake
.PHONY : ifopt_core/CMakeFiles/ifopt_core.dir/clean

ifopt_core/CMakeFiles/ifopt_core.dir/depend:
	cd /home/rrqq/towr_workspace/build/ifopt && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/rrqq/towr_workspace/src/ifopt /home/rrqq/towr_workspace/src/ifopt/ifopt_core /home/rrqq/towr_workspace/build/ifopt /home/rrqq/towr_workspace/build/ifopt/ifopt_core /home/rrqq/towr_workspace/build/ifopt/ifopt_core/CMakeFiles/ifopt_core.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : ifopt_core/CMakeFiles/ifopt_core.dir/depend

