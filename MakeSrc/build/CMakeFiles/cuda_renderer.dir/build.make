# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.13

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
CMAKE_COMMAND = /home/yupeng/downloads/cmake-3.13.2-Linux-x86_64/bin/cmake

# The command to remove a file.
RM = /home/yupeng/downloads/cmake-3.13.2-Linux-x86_64/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/yupeng/RGBD-pipeline/MakeSrc

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/yupeng/RGBD-pipeline/MakeSrc/build

# Include any dependencies generated for this target.
include CMakeFiles/cuda_renderer.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/cuda_renderer.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/cuda_renderer.dir/flags.make

CMakeFiles/cuda_renderer.dir/src/main.cpp.o: CMakeFiles/cuda_renderer.dir/flags.make
CMakeFiles/cuda_renderer.dir/src/main.cpp.o: ../src/main.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/yupeng/RGBD-pipeline/MakeSrc/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/cuda_renderer.dir/src/main.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/cuda_renderer.dir/src/main.cpp.o -c /home/yupeng/RGBD-pipeline/MakeSrc/src/main.cpp

CMakeFiles/cuda_renderer.dir/src/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/cuda_renderer.dir/src/main.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/yupeng/RGBD-pipeline/MakeSrc/src/main.cpp > CMakeFiles/cuda_renderer.dir/src/main.cpp.i

CMakeFiles/cuda_renderer.dir/src/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/cuda_renderer.dir/src/main.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/yupeng/RGBD-pipeline/MakeSrc/src/main.cpp -o CMakeFiles/cuda_renderer.dir/src/main.cpp.s

# Object files for target cuda_renderer
cuda_renderer_OBJECTS = \
"CMakeFiles/cuda_renderer.dir/src/main.cpp.o"

# External object files for target cuda_renderer
cuda_renderer_EXTERNAL_OBJECTS =

libcuda_renderer.a: CMakeFiles/cuda_renderer.dir/src/main.cpp.o
libcuda_renderer.a: CMakeFiles/cuda_renderer.dir/build.make
libcuda_renderer.a: CMakeFiles/cuda_renderer.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/yupeng/RGBD-pipeline/MakeSrc/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX static library libcuda_renderer.a"
	$(CMAKE_COMMAND) -P CMakeFiles/cuda_renderer.dir/cmake_clean_target.cmake
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/cuda_renderer.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/cuda_renderer.dir/build: libcuda_renderer.a

.PHONY : CMakeFiles/cuda_renderer.dir/build

CMakeFiles/cuda_renderer.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/cuda_renderer.dir/cmake_clean.cmake
.PHONY : CMakeFiles/cuda_renderer.dir/clean

CMakeFiles/cuda_renderer.dir/depend:
	cd /home/yupeng/RGBD-pipeline/MakeSrc/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/yupeng/RGBD-pipeline/MakeSrc /home/yupeng/RGBD-pipeline/MakeSrc /home/yupeng/RGBD-pipeline/MakeSrc/build /home/yupeng/RGBD-pipeline/MakeSrc/build /home/yupeng/RGBD-pipeline/MakeSrc/build/CMakeFiles/cuda_renderer.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/cuda_renderer.dir/depend
