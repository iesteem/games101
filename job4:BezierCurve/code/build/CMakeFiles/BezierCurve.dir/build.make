# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.22

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
CMAKE_COMMAND = /opt/homebrew/Cellar/cmake/3.22.2/bin/cmake

# The command to remove a file.
RM = /opt/homebrew/Cellar/cmake/3.22.2/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /Users/iesteem/Code/games101/job4:BezierCurve/code

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /Users/iesteem/Code/games101/job4:BezierCurve/code/build

# Include any dependencies generated for this target.
include CMakeFiles/BezierCurve.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include CMakeFiles/BezierCurve.dir/compiler_depend.make

# Include the progress variables for this target.
include CMakeFiles/BezierCurve.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/BezierCurve.dir/flags.make

CMakeFiles/BezierCurve.dir/main.cpp.o: CMakeFiles/BezierCurve.dir/flags.make
CMakeFiles/BezierCurve.dir/main.cpp.o: ../main.cpp
CMakeFiles/BezierCurve.dir/main.cpp.o: CMakeFiles/BezierCurve.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/Users/iesteem/Code/games101/job4:BezierCurve/code/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/BezierCurve.dir/main.cpp.o"
	/Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/BezierCurve.dir/main.cpp.o -MF CMakeFiles/BezierCurve.dir/main.cpp.o.d -o CMakeFiles/BezierCurve.dir/main.cpp.o -c /Users/iesteem/Code/games101/job4:BezierCurve/code/main.cpp

CMakeFiles/BezierCurve.dir/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/BezierCurve.dir/main.cpp.i"
	/Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /Users/iesteem/Code/games101/job4:BezierCurve/code/main.cpp > CMakeFiles/BezierCurve.dir/main.cpp.i

CMakeFiles/BezierCurve.dir/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/BezierCurve.dir/main.cpp.s"
	/Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /Users/iesteem/Code/games101/job4:BezierCurve/code/main.cpp -o CMakeFiles/BezierCurve.dir/main.cpp.s

# Object files for target BezierCurve
BezierCurve_OBJECTS = \
"CMakeFiles/BezierCurve.dir/main.cpp.o"

# External object files for target BezierCurve
BezierCurve_EXTERNAL_OBJECTS =

BezierCurve: CMakeFiles/BezierCurve.dir/main.cpp.o
BezierCurve: CMakeFiles/BezierCurve.dir/build.make
BezierCurve: /opt/homebrew/opt/opencv@2/lib/libopencv_videostab.2.4.13.dylib
BezierCurve: /opt/homebrew/opt/opencv@2/lib/libopencv_ts.a
BezierCurve: /opt/homebrew/opt/opencv@2/lib/libopencv_superres.2.4.13.dylib
BezierCurve: /opt/homebrew/opt/opencv@2/lib/libopencv_stitching.2.4.13.dylib
BezierCurve: /opt/homebrew/opt/opencv@2/lib/libopencv_contrib.2.4.13.dylib
BezierCurve: /opt/homebrew/opt/opencv@2/lib/libopencv_nonfree.2.4.13.dylib
BezierCurve: /opt/homebrew/opt/opencv@2/lib/libopencv_ocl.2.4.13.dylib
BezierCurve: /opt/homebrew/opt/opencv@2/lib/libopencv_gpu.2.4.13.dylib
BezierCurve: /opt/homebrew/opt/opencv@2/lib/libopencv_photo.2.4.13.dylib
BezierCurve: /opt/homebrew/opt/opencv@2/lib/libopencv_objdetect.2.4.13.dylib
BezierCurve: /opt/homebrew/opt/opencv@2/lib/libopencv_legacy.2.4.13.dylib
BezierCurve: /opt/homebrew/opt/opencv@2/lib/libopencv_video.2.4.13.dylib
BezierCurve: /opt/homebrew/opt/opencv@2/lib/libopencv_ml.2.4.13.dylib
BezierCurve: /opt/homebrew/opt/opencv@2/lib/libopencv_calib3d.2.4.13.dylib
BezierCurve: /opt/homebrew/opt/opencv@2/lib/libopencv_features2d.2.4.13.dylib
BezierCurve: /opt/homebrew/opt/opencv@2/lib/libopencv_highgui.2.4.13.dylib
BezierCurve: /opt/homebrew/opt/opencv@2/lib/libopencv_imgproc.2.4.13.dylib
BezierCurve: /opt/homebrew/opt/opencv@2/lib/libopencv_flann.2.4.13.dylib
BezierCurve: /opt/homebrew/opt/opencv@2/lib/libopencv_core.2.4.13.dylib
BezierCurve: CMakeFiles/BezierCurve.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/Users/iesteem/Code/games101/job4:BezierCurve/code/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable BezierCurve"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/BezierCurve.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/BezierCurve.dir/build: BezierCurve
.PHONY : CMakeFiles/BezierCurve.dir/build

CMakeFiles/BezierCurve.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/BezierCurve.dir/cmake_clean.cmake
.PHONY : CMakeFiles/BezierCurve.dir/clean

CMakeFiles/BezierCurve.dir/depend:
	cd /Users/iesteem/Code/games101/job4:BezierCurve/code/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /Users/iesteem/Code/games101/job4:BezierCurve/code /Users/iesteem/Code/games101/job4:BezierCurve/code /Users/iesteem/Code/games101/job4:BezierCurve/code/build /Users/iesteem/Code/games101/job4:BezierCurve/code/build /Users/iesteem/Code/games101/job4:BezierCurve/code/build/CMakeFiles/BezierCurve.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/BezierCurve.dir/depend

