# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.10

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
CMAKE_SOURCE_DIR = /home/kist/wheel

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/kist/wheel/build

# Include any dependencies generated for this target.
include src/CMakeFiles/test.dir/depend.make

# Include the progress variables for this target.
include src/CMakeFiles/test.dir/progress.make

# Include the compile flags for this target's objects.
include src/CMakeFiles/test.dir/flags.make

src/CMakeFiles/test.dir/Controller.cpp.o: src/CMakeFiles/test.dir/flags.make
src/CMakeFiles/test.dir/Controller.cpp.o: ../src/Controller.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/kist/wheel/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object src/CMakeFiles/test.dir/Controller.cpp.o"
	cd /home/kist/wheel/build/src && g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/test.dir/Controller.cpp.o -c /home/kist/wheel/src/Controller.cpp

src/CMakeFiles/test.dir/Controller.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/test.dir/Controller.cpp.i"
	cd /home/kist/wheel/build/src && g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/kist/wheel/src/Controller.cpp > CMakeFiles/test.dir/Controller.cpp.i

src/CMakeFiles/test.dir/Controller.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/test.dir/Controller.cpp.s"
	cd /home/kist/wheel/build/src && g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/kist/wheel/src/Controller.cpp -o CMakeFiles/test.dir/Controller.cpp.s

src/CMakeFiles/test.dir/Controller.cpp.o.requires:

.PHONY : src/CMakeFiles/test.dir/Controller.cpp.o.requires

src/CMakeFiles/test.dir/Controller.cpp.o.provides: src/CMakeFiles/test.dir/Controller.cpp.o.requires
	$(MAKE) -f src/CMakeFiles/test.dir/build.make src/CMakeFiles/test.dir/Controller.cpp.o.provides.build
.PHONY : src/CMakeFiles/test.dir/Controller.cpp.o.provides

src/CMakeFiles/test.dir/Controller.cpp.o.provides.build: src/CMakeFiles/test.dir/Controller.cpp.o


src/CMakeFiles/test.dir/Communication.cpp.o: src/CMakeFiles/test.dir/flags.make
src/CMakeFiles/test.dir/Communication.cpp.o: ../src/Communication.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/kist/wheel/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object src/CMakeFiles/test.dir/Communication.cpp.o"
	cd /home/kist/wheel/build/src && g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/test.dir/Communication.cpp.o -c /home/kist/wheel/src/Communication.cpp

src/CMakeFiles/test.dir/Communication.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/test.dir/Communication.cpp.i"
	cd /home/kist/wheel/build/src && g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/kist/wheel/src/Communication.cpp > CMakeFiles/test.dir/Communication.cpp.i

src/CMakeFiles/test.dir/Communication.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/test.dir/Communication.cpp.s"
	cd /home/kist/wheel/build/src && g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/kist/wheel/src/Communication.cpp -o CMakeFiles/test.dir/Communication.cpp.s

src/CMakeFiles/test.dir/Communication.cpp.o.requires:

.PHONY : src/CMakeFiles/test.dir/Communication.cpp.o.requires

src/CMakeFiles/test.dir/Communication.cpp.o.provides: src/CMakeFiles/test.dir/Communication.cpp.o.requires
	$(MAKE) -f src/CMakeFiles/test.dir/build.make src/CMakeFiles/test.dir/Communication.cpp.o.provides.build
.PHONY : src/CMakeFiles/test.dir/Communication.cpp.o.provides

src/CMakeFiles/test.dir/Communication.cpp.o.provides.build: src/CMakeFiles/test.dir/Communication.cpp.o


# Object files for target test
test_OBJECTS = \
"CMakeFiles/test.dir/Controller.cpp.o" \
"CMakeFiles/test.dir/Communication.cpp.o"

# External object files for target test
test_EXTERNAL_OBJECTS =

src/test: src/CMakeFiles/test.dir/Controller.cpp.o
src/test: src/CMakeFiles/test.dir/Communication.cpp.o
src/test: src/CMakeFiles/test.dir/build.make
src/test: src/CMakeFiles/test.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/kist/wheel/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Linking CXX executable test"
	cd /home/kist/wheel/build/src && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/test.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
src/CMakeFiles/test.dir/build: src/test

.PHONY : src/CMakeFiles/test.dir/build

src/CMakeFiles/test.dir/requires: src/CMakeFiles/test.dir/Controller.cpp.o.requires
src/CMakeFiles/test.dir/requires: src/CMakeFiles/test.dir/Communication.cpp.o.requires

.PHONY : src/CMakeFiles/test.dir/requires

src/CMakeFiles/test.dir/clean:
	cd /home/kist/wheel/build/src && $(CMAKE_COMMAND) -P CMakeFiles/test.dir/cmake_clean.cmake
.PHONY : src/CMakeFiles/test.dir/clean

src/CMakeFiles/test.dir/depend:
	cd /home/kist/wheel/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/kist/wheel /home/kist/wheel/src /home/kist/wheel/build /home/kist/wheel/build/src /home/kist/wheel/build/src/CMakeFiles/test.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : src/CMakeFiles/test.dir/depend

