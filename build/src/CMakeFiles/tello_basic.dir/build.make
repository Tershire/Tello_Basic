# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.28

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
CMAKE_COMMAND = /usr/local/bin/cmake

# The command to remove a file.
RM = /usr/local/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/tershire/Documents/research/Tello_Basic

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/tershire/Documents/research/Tello_Basic/build

# Include any dependencies generated for this target.
include src/CMakeFiles/tello_basic.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include src/CMakeFiles/tello_basic.dir/compiler_depend.make

# Include the progress variables for this target.
include src/CMakeFiles/tello_basic.dir/progress.make

# Include the compile flags for this target's objects.
include src/CMakeFiles/tello_basic.dir/flags.make

src/CMakeFiles/tello_basic.dir/camera/camera.cpp.o: src/CMakeFiles/tello_basic.dir/flags.make
src/CMakeFiles/tello_basic.dir/camera/camera.cpp.o: /home/tershire/Documents/research/Tello_Basic/src/camera/camera.cpp
src/CMakeFiles/tello_basic.dir/camera/camera.cpp.o: src/CMakeFiles/tello_basic.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --progress-dir=/home/tershire/Documents/research/Tello_Basic/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object src/CMakeFiles/tello_basic.dir/camera/camera.cpp.o"
	cd /home/tershire/Documents/research/Tello_Basic/build/src && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT src/CMakeFiles/tello_basic.dir/camera/camera.cpp.o -MF CMakeFiles/tello_basic.dir/camera/camera.cpp.o.d -o CMakeFiles/tello_basic.dir/camera/camera.cpp.o -c /home/tershire/Documents/research/Tello_Basic/src/camera/camera.cpp

src/CMakeFiles/tello_basic.dir/camera/camera.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Preprocessing CXX source to CMakeFiles/tello_basic.dir/camera/camera.cpp.i"
	cd /home/tershire/Documents/research/Tello_Basic/build/src && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/tershire/Documents/research/Tello_Basic/src/camera/camera.cpp > CMakeFiles/tello_basic.dir/camera/camera.cpp.i

src/CMakeFiles/tello_basic.dir/camera/camera.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Compiling CXX source to assembly CMakeFiles/tello_basic.dir/camera/camera.cpp.s"
	cd /home/tershire/Documents/research/Tello_Basic/build/src && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/tershire/Documents/research/Tello_Basic/src/camera/camera.cpp -o CMakeFiles/tello_basic.dir/camera/camera.cpp.s

src/CMakeFiles/tello_basic.dir/camera/brown_conrady.cpp.o: src/CMakeFiles/tello_basic.dir/flags.make
src/CMakeFiles/tello_basic.dir/camera/brown_conrady.cpp.o: /home/tershire/Documents/research/Tello_Basic/src/camera/brown_conrady.cpp
src/CMakeFiles/tello_basic.dir/camera/brown_conrady.cpp.o: src/CMakeFiles/tello_basic.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --progress-dir=/home/tershire/Documents/research/Tello_Basic/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object src/CMakeFiles/tello_basic.dir/camera/brown_conrady.cpp.o"
	cd /home/tershire/Documents/research/Tello_Basic/build/src && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT src/CMakeFiles/tello_basic.dir/camera/brown_conrady.cpp.o -MF CMakeFiles/tello_basic.dir/camera/brown_conrady.cpp.o.d -o CMakeFiles/tello_basic.dir/camera/brown_conrady.cpp.o -c /home/tershire/Documents/research/Tello_Basic/src/camera/brown_conrady.cpp

src/CMakeFiles/tello_basic.dir/camera/brown_conrady.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Preprocessing CXX source to CMakeFiles/tello_basic.dir/camera/brown_conrady.cpp.i"
	cd /home/tershire/Documents/research/Tello_Basic/build/src && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/tershire/Documents/research/Tello_Basic/src/camera/brown_conrady.cpp > CMakeFiles/tello_basic.dir/camera/brown_conrady.cpp.i

src/CMakeFiles/tello_basic.dir/camera/brown_conrady.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Compiling CXX source to assembly CMakeFiles/tello_basic.dir/camera/brown_conrady.cpp.s"
	cd /home/tershire/Documents/research/Tello_Basic/build/src && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/tershire/Documents/research/Tello_Basic/src/camera/brown_conrady.cpp -o CMakeFiles/tello_basic.dir/camera/brown_conrady.cpp.s

src/CMakeFiles/tello_basic.dir/camera/pinhole.cpp.o: src/CMakeFiles/tello_basic.dir/flags.make
src/CMakeFiles/tello_basic.dir/camera/pinhole.cpp.o: /home/tershire/Documents/research/Tello_Basic/src/camera/pinhole.cpp
src/CMakeFiles/tello_basic.dir/camera/pinhole.cpp.o: src/CMakeFiles/tello_basic.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --progress-dir=/home/tershire/Documents/research/Tello_Basic/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object src/CMakeFiles/tello_basic.dir/camera/pinhole.cpp.o"
	cd /home/tershire/Documents/research/Tello_Basic/build/src && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT src/CMakeFiles/tello_basic.dir/camera/pinhole.cpp.o -MF CMakeFiles/tello_basic.dir/camera/pinhole.cpp.o.d -o CMakeFiles/tello_basic.dir/camera/pinhole.cpp.o -c /home/tershire/Documents/research/Tello_Basic/src/camera/pinhole.cpp

src/CMakeFiles/tello_basic.dir/camera/pinhole.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Preprocessing CXX source to CMakeFiles/tello_basic.dir/camera/pinhole.cpp.i"
	cd /home/tershire/Documents/research/Tello_Basic/build/src && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/tershire/Documents/research/Tello_Basic/src/camera/pinhole.cpp > CMakeFiles/tello_basic.dir/camera/pinhole.cpp.i

src/CMakeFiles/tello_basic.dir/camera/pinhole.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Compiling CXX source to assembly CMakeFiles/tello_basic.dir/camera/pinhole.cpp.s"
	cd /home/tershire/Documents/research/Tello_Basic/build/src && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/tershire/Documents/research/Tello_Basic/src/camera/pinhole.cpp -o CMakeFiles/tello_basic.dir/camera/pinhole.cpp.s

src/CMakeFiles/tello_basic.dir/marker/aruco_detector.cpp.o: src/CMakeFiles/tello_basic.dir/flags.make
src/CMakeFiles/tello_basic.dir/marker/aruco_detector.cpp.o: /home/tershire/Documents/research/Tello_Basic/src/marker/aruco_detector.cpp
src/CMakeFiles/tello_basic.dir/marker/aruco_detector.cpp.o: src/CMakeFiles/tello_basic.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --progress-dir=/home/tershire/Documents/research/Tello_Basic/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Building CXX object src/CMakeFiles/tello_basic.dir/marker/aruco_detector.cpp.o"
	cd /home/tershire/Documents/research/Tello_Basic/build/src && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT src/CMakeFiles/tello_basic.dir/marker/aruco_detector.cpp.o -MF CMakeFiles/tello_basic.dir/marker/aruco_detector.cpp.o.d -o CMakeFiles/tello_basic.dir/marker/aruco_detector.cpp.o -c /home/tershire/Documents/research/Tello_Basic/src/marker/aruco_detector.cpp

src/CMakeFiles/tello_basic.dir/marker/aruco_detector.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Preprocessing CXX source to CMakeFiles/tello_basic.dir/marker/aruco_detector.cpp.i"
	cd /home/tershire/Documents/research/Tello_Basic/build/src && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/tershire/Documents/research/Tello_Basic/src/marker/aruco_detector.cpp > CMakeFiles/tello_basic.dir/marker/aruco_detector.cpp.i

src/CMakeFiles/tello_basic.dir/marker/aruco_detector.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Compiling CXX source to assembly CMakeFiles/tello_basic.dir/marker/aruco_detector.cpp.s"
	cd /home/tershire/Documents/research/Tello_Basic/build/src && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/tershire/Documents/research/Tello_Basic/src/marker/aruco_detector.cpp -o CMakeFiles/tello_basic.dir/marker/aruco_detector.cpp.s

src/CMakeFiles/tello_basic.dir/port/config.cpp.o: src/CMakeFiles/tello_basic.dir/flags.make
src/CMakeFiles/tello_basic.dir/port/config.cpp.o: /home/tershire/Documents/research/Tello_Basic/src/port/config.cpp
src/CMakeFiles/tello_basic.dir/port/config.cpp.o: src/CMakeFiles/tello_basic.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --progress-dir=/home/tershire/Documents/research/Tello_Basic/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "Building CXX object src/CMakeFiles/tello_basic.dir/port/config.cpp.o"
	cd /home/tershire/Documents/research/Tello_Basic/build/src && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT src/CMakeFiles/tello_basic.dir/port/config.cpp.o -MF CMakeFiles/tello_basic.dir/port/config.cpp.o.d -o CMakeFiles/tello_basic.dir/port/config.cpp.o -c /home/tershire/Documents/research/Tello_Basic/src/port/config.cpp

src/CMakeFiles/tello_basic.dir/port/config.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Preprocessing CXX source to CMakeFiles/tello_basic.dir/port/config.cpp.i"
	cd /home/tershire/Documents/research/Tello_Basic/build/src && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/tershire/Documents/research/Tello_Basic/src/port/config.cpp > CMakeFiles/tello_basic.dir/port/config.cpp.i

src/CMakeFiles/tello_basic.dir/port/config.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Compiling CXX source to assembly CMakeFiles/tello_basic.dir/port/config.cpp.s"
	cd /home/tershire/Documents/research/Tello_Basic/build/src && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/tershire/Documents/research/Tello_Basic/src/port/config.cpp -o CMakeFiles/tello_basic.dir/port/config.cpp.s

src/CMakeFiles/tello_basic.dir/port/setting.cpp.o: src/CMakeFiles/tello_basic.dir/flags.make
src/CMakeFiles/tello_basic.dir/port/setting.cpp.o: /home/tershire/Documents/research/Tello_Basic/src/port/setting.cpp
src/CMakeFiles/tello_basic.dir/port/setting.cpp.o: src/CMakeFiles/tello_basic.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --progress-dir=/home/tershire/Documents/research/Tello_Basic/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_6) "Building CXX object src/CMakeFiles/tello_basic.dir/port/setting.cpp.o"
	cd /home/tershire/Documents/research/Tello_Basic/build/src && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT src/CMakeFiles/tello_basic.dir/port/setting.cpp.o -MF CMakeFiles/tello_basic.dir/port/setting.cpp.o.d -o CMakeFiles/tello_basic.dir/port/setting.cpp.o -c /home/tershire/Documents/research/Tello_Basic/src/port/setting.cpp

src/CMakeFiles/tello_basic.dir/port/setting.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Preprocessing CXX source to CMakeFiles/tello_basic.dir/port/setting.cpp.i"
	cd /home/tershire/Documents/research/Tello_Basic/build/src && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/tershire/Documents/research/Tello_Basic/src/port/setting.cpp > CMakeFiles/tello_basic.dir/port/setting.cpp.i

src/CMakeFiles/tello_basic.dir/port/setting.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Compiling CXX source to assembly CMakeFiles/tello_basic.dir/port/setting.cpp.s"
	cd /home/tershire/Documents/research/Tello_Basic/build/src && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/tershire/Documents/research/Tello_Basic/src/port/setting.cpp -o CMakeFiles/tello_basic.dir/port/setting.cpp.s

src/CMakeFiles/tello_basic.dir/system.cpp.o: src/CMakeFiles/tello_basic.dir/flags.make
src/CMakeFiles/tello_basic.dir/system.cpp.o: /home/tershire/Documents/research/Tello_Basic/src/system.cpp
src/CMakeFiles/tello_basic.dir/system.cpp.o: src/CMakeFiles/tello_basic.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --progress-dir=/home/tershire/Documents/research/Tello_Basic/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_7) "Building CXX object src/CMakeFiles/tello_basic.dir/system.cpp.o"
	cd /home/tershire/Documents/research/Tello_Basic/build/src && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT src/CMakeFiles/tello_basic.dir/system.cpp.o -MF CMakeFiles/tello_basic.dir/system.cpp.o.d -o CMakeFiles/tello_basic.dir/system.cpp.o -c /home/tershire/Documents/research/Tello_Basic/src/system.cpp

src/CMakeFiles/tello_basic.dir/system.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Preprocessing CXX source to CMakeFiles/tello_basic.dir/system.cpp.i"
	cd /home/tershire/Documents/research/Tello_Basic/build/src && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/tershire/Documents/research/Tello_Basic/src/system.cpp > CMakeFiles/tello_basic.dir/system.cpp.i

src/CMakeFiles/tello_basic.dir/system.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Compiling CXX source to assembly CMakeFiles/tello_basic.dir/system.cpp.s"
	cd /home/tershire/Documents/research/Tello_Basic/build/src && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/tershire/Documents/research/Tello_Basic/src/system.cpp -o CMakeFiles/tello_basic.dir/system.cpp.s

# Object files for target tello_basic
tello_basic_OBJECTS = \
"CMakeFiles/tello_basic.dir/camera/camera.cpp.o" \
"CMakeFiles/tello_basic.dir/camera/brown_conrady.cpp.o" \
"CMakeFiles/tello_basic.dir/camera/pinhole.cpp.o" \
"CMakeFiles/tello_basic.dir/marker/aruco_detector.cpp.o" \
"CMakeFiles/tello_basic.dir/port/config.cpp.o" \
"CMakeFiles/tello_basic.dir/port/setting.cpp.o" \
"CMakeFiles/tello_basic.dir/system.cpp.o"

# External object files for target tello_basic
tello_basic_EXTERNAL_OBJECTS =

/home/tershire/Documents/research/Tello_Basic/lib/libtello_basic.so: src/CMakeFiles/tello_basic.dir/camera/camera.cpp.o
/home/tershire/Documents/research/Tello_Basic/lib/libtello_basic.so: src/CMakeFiles/tello_basic.dir/camera/brown_conrady.cpp.o
/home/tershire/Documents/research/Tello_Basic/lib/libtello_basic.so: src/CMakeFiles/tello_basic.dir/camera/pinhole.cpp.o
/home/tershire/Documents/research/Tello_Basic/lib/libtello_basic.so: src/CMakeFiles/tello_basic.dir/marker/aruco_detector.cpp.o
/home/tershire/Documents/research/Tello_Basic/lib/libtello_basic.so: src/CMakeFiles/tello_basic.dir/port/config.cpp.o
/home/tershire/Documents/research/Tello_Basic/lib/libtello_basic.so: src/CMakeFiles/tello_basic.dir/port/setting.cpp.o
/home/tershire/Documents/research/Tello_Basic/lib/libtello_basic.so: src/CMakeFiles/tello_basic.dir/system.cpp.o
/home/tershire/Documents/research/Tello_Basic/lib/libtello_basic.so: src/CMakeFiles/tello_basic.dir/build.make
/home/tershire/Documents/research/Tello_Basic/lib/libtello_basic.so: /usr/local/lib/libopencv_gapi.so.4.9.0
/home/tershire/Documents/research/Tello_Basic/lib/libtello_basic.so: /usr/local/lib/libopencv_stitching.so.4.9.0
/home/tershire/Documents/research/Tello_Basic/lib/libtello_basic.so: /usr/local/lib/libopencv_alphamat.so.4.9.0
/home/tershire/Documents/research/Tello_Basic/lib/libtello_basic.so: /usr/local/lib/libopencv_aruco.so.4.9.0
/home/tershire/Documents/research/Tello_Basic/lib/libtello_basic.so: /usr/local/lib/libopencv_bgsegm.so.4.9.0
/home/tershire/Documents/research/Tello_Basic/lib/libtello_basic.so: /usr/local/lib/libopencv_bioinspired.so.4.9.0
/home/tershire/Documents/research/Tello_Basic/lib/libtello_basic.so: /usr/local/lib/libopencv_ccalib.so.4.9.0
/home/tershire/Documents/research/Tello_Basic/lib/libtello_basic.so: /usr/local/lib/libopencv_cudabgsegm.so.4.9.0
/home/tershire/Documents/research/Tello_Basic/lib/libtello_basic.so: /usr/local/lib/libopencv_cudafeatures2d.so.4.9.0
/home/tershire/Documents/research/Tello_Basic/lib/libtello_basic.so: /usr/local/lib/libopencv_cudaobjdetect.so.4.9.0
/home/tershire/Documents/research/Tello_Basic/lib/libtello_basic.so: /usr/local/lib/libopencv_cudastereo.so.4.9.0
/home/tershire/Documents/research/Tello_Basic/lib/libtello_basic.so: /usr/local/lib/libopencv_dnn_objdetect.so.4.9.0
/home/tershire/Documents/research/Tello_Basic/lib/libtello_basic.so: /usr/local/lib/libopencv_dnn_superres.so.4.9.0
/home/tershire/Documents/research/Tello_Basic/lib/libtello_basic.so: /usr/local/lib/libopencv_dpm.so.4.9.0
/home/tershire/Documents/research/Tello_Basic/lib/libtello_basic.so: /usr/local/lib/libopencv_face.so.4.9.0
/home/tershire/Documents/research/Tello_Basic/lib/libtello_basic.so: /usr/local/lib/libopencv_freetype.so.4.9.0
/home/tershire/Documents/research/Tello_Basic/lib/libtello_basic.so: /usr/local/lib/libopencv_fuzzy.so.4.9.0
/home/tershire/Documents/research/Tello_Basic/lib/libtello_basic.so: /usr/local/lib/libopencv_hdf.so.4.9.0
/home/tershire/Documents/research/Tello_Basic/lib/libtello_basic.so: /usr/local/lib/libopencv_hfs.so.4.9.0
/home/tershire/Documents/research/Tello_Basic/lib/libtello_basic.so: /usr/local/lib/libopencv_img_hash.so.4.9.0
/home/tershire/Documents/research/Tello_Basic/lib/libtello_basic.so: /usr/local/lib/libopencv_intensity_transform.so.4.9.0
/home/tershire/Documents/research/Tello_Basic/lib/libtello_basic.so: /usr/local/lib/libopencv_line_descriptor.so.4.9.0
/home/tershire/Documents/research/Tello_Basic/lib/libtello_basic.so: /usr/local/lib/libopencv_mcc.so.4.9.0
/home/tershire/Documents/research/Tello_Basic/lib/libtello_basic.so: /usr/local/lib/libopencv_quality.so.4.9.0
/home/tershire/Documents/research/Tello_Basic/lib/libtello_basic.so: /usr/local/lib/libopencv_rapid.so.4.9.0
/home/tershire/Documents/research/Tello_Basic/lib/libtello_basic.so: /usr/local/lib/libopencv_reg.so.4.9.0
/home/tershire/Documents/research/Tello_Basic/lib/libtello_basic.so: /usr/local/lib/libopencv_rgbd.so.4.9.0
/home/tershire/Documents/research/Tello_Basic/lib/libtello_basic.so: /usr/local/lib/libopencv_saliency.so.4.9.0
/home/tershire/Documents/research/Tello_Basic/lib/libtello_basic.so: /usr/local/lib/libopencv_sfm.so.4.9.0
/home/tershire/Documents/research/Tello_Basic/lib/libtello_basic.so: /usr/local/lib/libopencv_stereo.so.4.9.0
/home/tershire/Documents/research/Tello_Basic/lib/libtello_basic.so: /usr/local/lib/libopencv_structured_light.so.4.9.0
/home/tershire/Documents/research/Tello_Basic/lib/libtello_basic.so: /usr/local/lib/libopencv_superres.so.4.9.0
/home/tershire/Documents/research/Tello_Basic/lib/libtello_basic.so: /usr/local/lib/libopencv_surface_matching.so.4.9.0
/home/tershire/Documents/research/Tello_Basic/lib/libtello_basic.so: /usr/local/lib/libopencv_tracking.so.4.9.0
/home/tershire/Documents/research/Tello_Basic/lib/libtello_basic.so: /usr/local/lib/libopencv_videostab.so.4.9.0
/home/tershire/Documents/research/Tello_Basic/lib/libtello_basic.so: /usr/local/lib/libopencv_wechat_qrcode.so.4.9.0
/home/tershire/Documents/research/Tello_Basic/lib/libtello_basic.so: /usr/local/lib/libopencv_xfeatures2d.so.4.9.0
/home/tershire/Documents/research/Tello_Basic/lib/libtello_basic.so: /usr/local/lib/libopencv_xobjdetect.so.4.9.0
/home/tershire/Documents/research/Tello_Basic/lib/libtello_basic.so: /usr/local/lib/libopencv_xphoto.so.4.9.0
/home/tershire/Documents/research/Tello_Basic/lib/libtello_basic.so: /usr/local/lib/libopencv_shape.so.4.9.0
/home/tershire/Documents/research/Tello_Basic/lib/libtello_basic.so: /usr/local/lib/libopencv_highgui.so.4.9.0
/home/tershire/Documents/research/Tello_Basic/lib/libtello_basic.so: /usr/local/lib/libopencv_datasets.so.4.9.0
/home/tershire/Documents/research/Tello_Basic/lib/libtello_basic.so: /usr/local/lib/libopencv_plot.so.4.9.0
/home/tershire/Documents/research/Tello_Basic/lib/libtello_basic.so: /usr/local/lib/libopencv_text.so.4.9.0
/home/tershire/Documents/research/Tello_Basic/lib/libtello_basic.so: /usr/local/lib/libopencv_ml.so.4.9.0
/home/tershire/Documents/research/Tello_Basic/lib/libtello_basic.so: /usr/local/lib/libopencv_phase_unwrapping.so.4.9.0
/home/tershire/Documents/research/Tello_Basic/lib/libtello_basic.so: /usr/local/lib/libopencv_cudacodec.so.4.9.0
/home/tershire/Documents/research/Tello_Basic/lib/libtello_basic.so: /usr/local/lib/libopencv_videoio.so.4.9.0
/home/tershire/Documents/research/Tello_Basic/lib/libtello_basic.so: /usr/local/lib/libopencv_cudaoptflow.so.4.9.0
/home/tershire/Documents/research/Tello_Basic/lib/libtello_basic.so: /usr/local/lib/libopencv_cudalegacy.so.4.9.0
/home/tershire/Documents/research/Tello_Basic/lib/libtello_basic.so: /usr/local/lib/libopencv_cudawarping.so.4.9.0
/home/tershire/Documents/research/Tello_Basic/lib/libtello_basic.so: /usr/local/lib/libopencv_optflow.so.4.9.0
/home/tershire/Documents/research/Tello_Basic/lib/libtello_basic.so: /usr/local/lib/libopencv_ximgproc.so.4.9.0
/home/tershire/Documents/research/Tello_Basic/lib/libtello_basic.so: /usr/local/lib/libopencv_video.so.4.9.0
/home/tershire/Documents/research/Tello_Basic/lib/libtello_basic.so: /usr/local/lib/libopencv_imgcodecs.so.4.9.0
/home/tershire/Documents/research/Tello_Basic/lib/libtello_basic.so: /usr/local/lib/libopencv_objdetect.so.4.9.0
/home/tershire/Documents/research/Tello_Basic/lib/libtello_basic.so: /usr/local/lib/libopencv_calib3d.so.4.9.0
/home/tershire/Documents/research/Tello_Basic/lib/libtello_basic.so: /usr/local/lib/libopencv_dnn.so.4.9.0
/home/tershire/Documents/research/Tello_Basic/lib/libtello_basic.so: /usr/local/lib/libopencv_features2d.so.4.9.0
/home/tershire/Documents/research/Tello_Basic/lib/libtello_basic.so: /usr/local/lib/libopencv_flann.so.4.9.0
/home/tershire/Documents/research/Tello_Basic/lib/libtello_basic.so: /usr/local/lib/libopencv_photo.so.4.9.0
/home/tershire/Documents/research/Tello_Basic/lib/libtello_basic.so: /usr/local/lib/libopencv_cudaimgproc.so.4.9.0
/home/tershire/Documents/research/Tello_Basic/lib/libtello_basic.so: /usr/local/lib/libopencv_cudafilters.so.4.9.0
/home/tershire/Documents/research/Tello_Basic/lib/libtello_basic.so: /usr/local/lib/libopencv_imgproc.so.4.9.0
/home/tershire/Documents/research/Tello_Basic/lib/libtello_basic.so: /usr/local/lib/libopencv_cudaarithm.so.4.9.0
/home/tershire/Documents/research/Tello_Basic/lib/libtello_basic.so: /usr/local/lib/libopencv_core.so.4.9.0
/home/tershire/Documents/research/Tello_Basic/lib/libtello_basic.so: /usr/local/lib/libopencv_cudev.so.4.9.0
/home/tershire/Documents/research/Tello_Basic/lib/libtello_basic.so: src/CMakeFiles/tello_basic.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --bold --progress-dir=/home/tershire/Documents/research/Tello_Basic/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_8) "Linking CXX shared library /home/tershire/Documents/research/Tello_Basic/lib/libtello_basic.so"
	cd /home/tershire/Documents/research/Tello_Basic/build/src && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/tello_basic.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
src/CMakeFiles/tello_basic.dir/build: /home/tershire/Documents/research/Tello_Basic/lib/libtello_basic.so
.PHONY : src/CMakeFiles/tello_basic.dir/build

src/CMakeFiles/tello_basic.dir/clean:
	cd /home/tershire/Documents/research/Tello_Basic/build/src && $(CMAKE_COMMAND) -P CMakeFiles/tello_basic.dir/cmake_clean.cmake
.PHONY : src/CMakeFiles/tello_basic.dir/clean

src/CMakeFiles/tello_basic.dir/depend:
	cd /home/tershire/Documents/research/Tello_Basic/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/tershire/Documents/research/Tello_Basic /home/tershire/Documents/research/Tello_Basic/src /home/tershire/Documents/research/Tello_Basic/build /home/tershire/Documents/research/Tello_Basic/build/src /home/tershire/Documents/research/Tello_Basic/build/src/CMakeFiles/tello_basic.dir/DependInfo.cmake "--color=$(COLOR)"
.PHONY : src/CMakeFiles/tello_basic.dir/depend

