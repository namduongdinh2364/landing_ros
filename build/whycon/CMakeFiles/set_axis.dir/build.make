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
CMAKE_SOURCE_DIR = /home/ducanh/ducanh_ws/landing_uav/landing_ros/src/whycon

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/ducanh/ducanh_ws/landing_uav/landing_ros/build/whycon

# Include any dependencies generated for this target.
include CMakeFiles/set_axis.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/set_axis.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/set_axis.dir/flags.make

CMakeFiles/set_axis.dir/src/ros/set_axis_node.cpp.o: CMakeFiles/set_axis.dir/flags.make
CMakeFiles/set_axis.dir/src/ros/set_axis_node.cpp.o: /home/ducanh/ducanh_ws/landing_uav/landing_ros/src/whycon/src/ros/set_axis_node.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/ducanh/ducanh_ws/landing_uav/landing_ros/build/whycon/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/set_axis.dir/src/ros/set_axis_node.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/set_axis.dir/src/ros/set_axis_node.cpp.o -c /home/ducanh/ducanh_ws/landing_uav/landing_ros/src/whycon/src/ros/set_axis_node.cpp

CMakeFiles/set_axis.dir/src/ros/set_axis_node.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/set_axis.dir/src/ros/set_axis_node.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/ducanh/ducanh_ws/landing_uav/landing_ros/src/whycon/src/ros/set_axis_node.cpp > CMakeFiles/set_axis.dir/src/ros/set_axis_node.cpp.i

CMakeFiles/set_axis.dir/src/ros/set_axis_node.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/set_axis.dir/src/ros/set_axis_node.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/ducanh/ducanh_ws/landing_uav/landing_ros/src/whycon/src/ros/set_axis_node.cpp -o CMakeFiles/set_axis.dir/src/ros/set_axis_node.cpp.s

CMakeFiles/set_axis.dir/src/ros/set_axis.cpp.o: CMakeFiles/set_axis.dir/flags.make
CMakeFiles/set_axis.dir/src/ros/set_axis.cpp.o: /home/ducanh/ducanh_ws/landing_uav/landing_ros/src/whycon/src/ros/set_axis.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/ducanh/ducanh_ws/landing_uav/landing_ros/build/whycon/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/set_axis.dir/src/ros/set_axis.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/set_axis.dir/src/ros/set_axis.cpp.o -c /home/ducanh/ducanh_ws/landing_uav/landing_ros/src/whycon/src/ros/set_axis.cpp

CMakeFiles/set_axis.dir/src/ros/set_axis.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/set_axis.dir/src/ros/set_axis.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/ducanh/ducanh_ws/landing_uav/landing_ros/src/whycon/src/ros/set_axis.cpp > CMakeFiles/set_axis.dir/src/ros/set_axis.cpp.i

CMakeFiles/set_axis.dir/src/ros/set_axis.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/set_axis.dir/src/ros/set_axis.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/ducanh/ducanh_ws/landing_uav/landing_ros/src/whycon/src/ros/set_axis.cpp -o CMakeFiles/set_axis.dir/src/ros/set_axis.cpp.s

# Object files for target set_axis
set_axis_OBJECTS = \
"CMakeFiles/set_axis.dir/src/ros/set_axis_node.cpp.o" \
"CMakeFiles/set_axis.dir/src/ros/set_axis.cpp.o"

# External object files for target set_axis
set_axis_EXTERNAL_OBJECTS =

/home/ducanh/ducanh_ws/landing_uav/landing_ros/devel/lib/whycon/set_axis: CMakeFiles/set_axis.dir/src/ros/set_axis_node.cpp.o
/home/ducanh/ducanh_ws/landing_uav/landing_ros/devel/lib/whycon/set_axis: CMakeFiles/set_axis.dir/src/ros/set_axis.cpp.o
/home/ducanh/ducanh_ws/landing_uav/landing_ros/devel/lib/whycon/set_axis: CMakeFiles/set_axis.dir/build.make
/home/ducanh/ducanh_ws/landing_uav/landing_ros/devel/lib/whycon/set_axis: /opt/ros/noetic/lib/libcv_bridge.so
/home/ducanh/ducanh_ws/landing_uav/landing_ros/devel/lib/whycon/set_axis: /opt/ros/noetic/lib/libcamera_info_manager.so
/home/ducanh/ducanh_ws/landing_uav/landing_ros/devel/lib/whycon/set_axis: /opt/ros/noetic/lib/libcamera_calibration_parsers.so
/home/ducanh/ducanh_ws/landing_uav/landing_ros/devel/lib/whycon/set_axis: /opt/ros/noetic/lib/libimage_geometry.so
/home/ducanh/ducanh_ws/landing_uav/landing_ros/devel/lib/whycon/set_axis: /usr/lib/x86_64-linux-gnu/libopencv_calib3d.so.4.2.0
/home/ducanh/ducanh_ws/landing_uav/landing_ros/devel/lib/whycon/set_axis: /usr/lib/x86_64-linux-gnu/libopencv_core.so.4.2.0
/home/ducanh/ducanh_ws/landing_uav/landing_ros/devel/lib/whycon/set_axis: /usr/lib/x86_64-linux-gnu/libopencv_dnn.so.4.2.0
/home/ducanh/ducanh_ws/landing_uav/landing_ros/devel/lib/whycon/set_axis: /usr/lib/x86_64-linux-gnu/libopencv_features2d.so.4.2.0
/home/ducanh/ducanh_ws/landing_uav/landing_ros/devel/lib/whycon/set_axis: /usr/lib/x86_64-linux-gnu/libopencv_flann.so.4.2.0
/home/ducanh/ducanh_ws/landing_uav/landing_ros/devel/lib/whycon/set_axis: /usr/lib/x86_64-linux-gnu/libopencv_highgui.so.4.2.0
/home/ducanh/ducanh_ws/landing_uav/landing_ros/devel/lib/whycon/set_axis: /usr/lib/x86_64-linux-gnu/libopencv_imgcodecs.so.4.2.0
/home/ducanh/ducanh_ws/landing_uav/landing_ros/devel/lib/whycon/set_axis: /usr/lib/x86_64-linux-gnu/libopencv_imgproc.so.4.2.0
/home/ducanh/ducanh_ws/landing_uav/landing_ros/devel/lib/whycon/set_axis: /usr/lib/x86_64-linux-gnu/libopencv_ml.so.4.2.0
/home/ducanh/ducanh_ws/landing_uav/landing_ros/devel/lib/whycon/set_axis: /usr/lib/x86_64-linux-gnu/libopencv_objdetect.so.4.2.0
/home/ducanh/ducanh_ws/landing_uav/landing_ros/devel/lib/whycon/set_axis: /usr/lib/x86_64-linux-gnu/libopencv_photo.so.4.2.0
/home/ducanh/ducanh_ws/landing_uav/landing_ros/devel/lib/whycon/set_axis: /usr/lib/x86_64-linux-gnu/libopencv_stitching.so.4.2.0
/home/ducanh/ducanh_ws/landing_uav/landing_ros/devel/lib/whycon/set_axis: /usr/lib/x86_64-linux-gnu/libopencv_video.so.4.2.0
/home/ducanh/ducanh_ws/landing_uav/landing_ros/devel/lib/whycon/set_axis: /usr/lib/x86_64-linux-gnu/libopencv_videoio.so.4.2.0
/home/ducanh/ducanh_ws/landing_uav/landing_ros/devel/lib/whycon/set_axis: /usr/lib/x86_64-linux-gnu/libopencv_aruco.so.4.2.0
/home/ducanh/ducanh_ws/landing_uav/landing_ros/devel/lib/whycon/set_axis: /usr/lib/x86_64-linux-gnu/libopencv_bgsegm.so.4.2.0
/home/ducanh/ducanh_ws/landing_uav/landing_ros/devel/lib/whycon/set_axis: /usr/lib/x86_64-linux-gnu/libopencv_bioinspired.so.4.2.0
/home/ducanh/ducanh_ws/landing_uav/landing_ros/devel/lib/whycon/set_axis: /usr/lib/x86_64-linux-gnu/libopencv_ccalib.so.4.2.0
/home/ducanh/ducanh_ws/landing_uav/landing_ros/devel/lib/whycon/set_axis: /usr/lib/x86_64-linux-gnu/libopencv_datasets.so.4.2.0
/home/ducanh/ducanh_ws/landing_uav/landing_ros/devel/lib/whycon/set_axis: /usr/lib/x86_64-linux-gnu/libopencv_dnn_objdetect.so.4.2.0
/home/ducanh/ducanh_ws/landing_uav/landing_ros/devel/lib/whycon/set_axis: /usr/lib/x86_64-linux-gnu/libopencv_dnn_superres.so.4.2.0
/home/ducanh/ducanh_ws/landing_uav/landing_ros/devel/lib/whycon/set_axis: /usr/lib/x86_64-linux-gnu/libopencv_dpm.so.4.2.0
/home/ducanh/ducanh_ws/landing_uav/landing_ros/devel/lib/whycon/set_axis: /usr/lib/x86_64-linux-gnu/libopencv_face.so.4.2.0
/home/ducanh/ducanh_ws/landing_uav/landing_ros/devel/lib/whycon/set_axis: /usr/lib/x86_64-linux-gnu/libopencv_freetype.so.4.2.0
/home/ducanh/ducanh_ws/landing_uav/landing_ros/devel/lib/whycon/set_axis: /usr/lib/x86_64-linux-gnu/libopencv_fuzzy.so.4.2.0
/home/ducanh/ducanh_ws/landing_uav/landing_ros/devel/lib/whycon/set_axis: /usr/lib/x86_64-linux-gnu/libopencv_hdf.so.4.2.0
/home/ducanh/ducanh_ws/landing_uav/landing_ros/devel/lib/whycon/set_axis: /usr/lib/x86_64-linux-gnu/libopencv_hfs.so.4.2.0
/home/ducanh/ducanh_ws/landing_uav/landing_ros/devel/lib/whycon/set_axis: /usr/lib/x86_64-linux-gnu/libopencv_img_hash.so.4.2.0
/home/ducanh/ducanh_ws/landing_uav/landing_ros/devel/lib/whycon/set_axis: /usr/lib/x86_64-linux-gnu/libopencv_line_descriptor.so.4.2.0
/home/ducanh/ducanh_ws/landing_uav/landing_ros/devel/lib/whycon/set_axis: /usr/lib/x86_64-linux-gnu/libopencv_optflow.so.4.2.0
/home/ducanh/ducanh_ws/landing_uav/landing_ros/devel/lib/whycon/set_axis: /usr/lib/x86_64-linux-gnu/libopencv_phase_unwrapping.so.4.2.0
/home/ducanh/ducanh_ws/landing_uav/landing_ros/devel/lib/whycon/set_axis: /usr/lib/x86_64-linux-gnu/libopencv_plot.so.4.2.0
/home/ducanh/ducanh_ws/landing_uav/landing_ros/devel/lib/whycon/set_axis: /usr/lib/x86_64-linux-gnu/libopencv_quality.so.4.2.0
/home/ducanh/ducanh_ws/landing_uav/landing_ros/devel/lib/whycon/set_axis: /usr/lib/x86_64-linux-gnu/libopencv_reg.so.4.2.0
/home/ducanh/ducanh_ws/landing_uav/landing_ros/devel/lib/whycon/set_axis: /usr/lib/x86_64-linux-gnu/libopencv_rgbd.so.4.2.0
/home/ducanh/ducanh_ws/landing_uav/landing_ros/devel/lib/whycon/set_axis: /usr/lib/x86_64-linux-gnu/libopencv_saliency.so.4.2.0
/home/ducanh/ducanh_ws/landing_uav/landing_ros/devel/lib/whycon/set_axis: /usr/lib/x86_64-linux-gnu/libopencv_shape.so.4.2.0
/home/ducanh/ducanh_ws/landing_uav/landing_ros/devel/lib/whycon/set_axis: /usr/lib/x86_64-linux-gnu/libopencv_stereo.so.4.2.0
/home/ducanh/ducanh_ws/landing_uav/landing_ros/devel/lib/whycon/set_axis: /usr/lib/x86_64-linux-gnu/libopencv_structured_light.so.4.2.0
/home/ducanh/ducanh_ws/landing_uav/landing_ros/devel/lib/whycon/set_axis: /usr/lib/x86_64-linux-gnu/libopencv_superres.so.4.2.0
/home/ducanh/ducanh_ws/landing_uav/landing_ros/devel/lib/whycon/set_axis: /usr/lib/x86_64-linux-gnu/libopencv_surface_matching.so.4.2.0
/home/ducanh/ducanh_ws/landing_uav/landing_ros/devel/lib/whycon/set_axis: /usr/lib/x86_64-linux-gnu/libopencv_text.so.4.2.0
/home/ducanh/ducanh_ws/landing_uav/landing_ros/devel/lib/whycon/set_axis: /usr/lib/x86_64-linux-gnu/libopencv_tracking.so.4.2.0
/home/ducanh/ducanh_ws/landing_uav/landing_ros/devel/lib/whycon/set_axis: /usr/lib/x86_64-linux-gnu/libopencv_videostab.so.4.2.0
/home/ducanh/ducanh_ws/landing_uav/landing_ros/devel/lib/whycon/set_axis: /usr/lib/x86_64-linux-gnu/libopencv_viz.so.4.2.0
/home/ducanh/ducanh_ws/landing_uav/landing_ros/devel/lib/whycon/set_axis: /usr/lib/x86_64-linux-gnu/libopencv_ximgproc.so.4.2.0
/home/ducanh/ducanh_ws/landing_uav/landing_ros/devel/lib/whycon/set_axis: /usr/lib/x86_64-linux-gnu/libopencv_xobjdetect.so.4.2.0
/home/ducanh/ducanh_ws/landing_uav/landing_ros/devel/lib/whycon/set_axis: /usr/lib/x86_64-linux-gnu/libopencv_xphoto.so.4.2.0
/home/ducanh/ducanh_ws/landing_uav/landing_ros/devel/lib/whycon/set_axis: /opt/ros/noetic/lib/libimage_transport.so
/home/ducanh/ducanh_ws/landing_uav/landing_ros/devel/lib/whycon/set_axis: /opt/ros/noetic/lib/libtf.so
/home/ducanh/ducanh_ws/landing_uav/landing_ros/devel/lib/whycon/set_axis: /opt/ros/noetic/lib/libtf2_ros.so
/home/ducanh/ducanh_ws/landing_uav/landing_ros/devel/lib/whycon/set_axis: /opt/ros/noetic/lib/libactionlib.so
/home/ducanh/ducanh_ws/landing_uav/landing_ros/devel/lib/whycon/set_axis: /opt/ros/noetic/lib/libmessage_filters.so
/home/ducanh/ducanh_ws/landing_uav/landing_ros/devel/lib/whycon/set_axis: /opt/ros/noetic/lib/libtf2.so
/home/ducanh/ducanh_ws/landing_uav/landing_ros/devel/lib/whycon/set_axis: /opt/ros/noetic/lib/libnodeletlib.so
/home/ducanh/ducanh_ws/landing_uav/landing_ros/devel/lib/whycon/set_axis: /opt/ros/noetic/lib/libbondcpp.so
/home/ducanh/ducanh_ws/landing_uav/landing_ros/devel/lib/whycon/set_axis: /usr/lib/x86_64-linux-gnu/libuuid.so
/home/ducanh/ducanh_ws/landing_uav/landing_ros/devel/lib/whycon/set_axis: /opt/ros/noetic/lib/libclass_loader.so
/home/ducanh/ducanh_ws/landing_uav/landing_ros/devel/lib/whycon/set_axis: /usr/lib/x86_64-linux-gnu/libPocoFoundation.so
/home/ducanh/ducanh_ws/landing_uav/landing_ros/devel/lib/whycon/set_axis: /usr/lib/x86_64-linux-gnu/libdl.so
/home/ducanh/ducanh_ws/landing_uav/landing_ros/devel/lib/whycon/set_axis: /opt/ros/noetic/lib/libroslib.so
/home/ducanh/ducanh_ws/landing_uav/landing_ros/devel/lib/whycon/set_axis: /opt/ros/noetic/lib/librospack.so
/home/ducanh/ducanh_ws/landing_uav/landing_ros/devel/lib/whycon/set_axis: /usr/lib/x86_64-linux-gnu/libpython3.8.so
/home/ducanh/ducanh_ws/landing_uav/landing_ros/devel/lib/whycon/set_axis: /usr/lib/x86_64-linux-gnu/libboost_program_options.so.1.71.0
/home/ducanh/ducanh_ws/landing_uav/landing_ros/devel/lib/whycon/set_axis: /usr/lib/x86_64-linux-gnu/libtinyxml2.so
/home/ducanh/ducanh_ws/landing_uav/landing_ros/devel/lib/whycon/set_axis: /opt/ros/noetic/lib/libroscpp.so
/home/ducanh/ducanh_ws/landing_uav/landing_ros/devel/lib/whycon/set_axis: /usr/lib/x86_64-linux-gnu/libpthread.so
/home/ducanh/ducanh_ws/landing_uav/landing_ros/devel/lib/whycon/set_axis: /usr/lib/x86_64-linux-gnu/libboost_chrono.so.1.71.0
/home/ducanh/ducanh_ws/landing_uav/landing_ros/devel/lib/whycon/set_axis: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so.1.71.0
/home/ducanh/ducanh_ws/landing_uav/landing_ros/devel/lib/whycon/set_axis: /opt/ros/noetic/lib/librosconsole.so
/home/ducanh/ducanh_ws/landing_uav/landing_ros/devel/lib/whycon/set_axis: /opt/ros/noetic/lib/librosconsole_log4cxx.so
/home/ducanh/ducanh_ws/landing_uav/landing_ros/devel/lib/whycon/set_axis: /opt/ros/noetic/lib/librosconsole_backend_interface.so
/home/ducanh/ducanh_ws/landing_uav/landing_ros/devel/lib/whycon/set_axis: /usr/lib/x86_64-linux-gnu/liblog4cxx.so
/home/ducanh/ducanh_ws/landing_uav/landing_ros/devel/lib/whycon/set_axis: /usr/lib/x86_64-linux-gnu/libboost_regex.so.1.71.0
/home/ducanh/ducanh_ws/landing_uav/landing_ros/devel/lib/whycon/set_axis: /opt/ros/noetic/lib/libxmlrpcpp.so
/home/ducanh/ducanh_ws/landing_uav/landing_ros/devel/lib/whycon/set_axis: /opt/ros/noetic/lib/libroscpp_serialization.so
/home/ducanh/ducanh_ws/landing_uav/landing_ros/devel/lib/whycon/set_axis: /opt/ros/noetic/lib/librostime.so
/home/ducanh/ducanh_ws/landing_uav/landing_ros/devel/lib/whycon/set_axis: /usr/lib/x86_64-linux-gnu/libboost_date_time.so.1.71.0
/home/ducanh/ducanh_ws/landing_uav/landing_ros/devel/lib/whycon/set_axis: /opt/ros/noetic/lib/libcpp_common.so
/home/ducanh/ducanh_ws/landing_uav/landing_ros/devel/lib/whycon/set_axis: /usr/lib/x86_64-linux-gnu/libboost_system.so.1.71.0
/home/ducanh/ducanh_ws/landing_uav/landing_ros/devel/lib/whycon/set_axis: /usr/lib/x86_64-linux-gnu/libboost_thread.so.1.71.0
/home/ducanh/ducanh_ws/landing_uav/landing_ros/devel/lib/whycon/set_axis: /usr/lib/x86_64-linux-gnu/libconsole_bridge.so.0.4
/home/ducanh/ducanh_ws/landing_uav/landing_ros/devel/lib/whycon/set_axis: /home/ducanh/ducanh_ws/landing_uav/landing_ros/devel/lib/libwhycon.so
/home/ducanh/ducanh_ws/landing_uav/landing_ros/devel/lib/whycon/set_axis: /usr/lib/x86_64-linux-gnu/libopencv_stitching.so.4.2.0
/home/ducanh/ducanh_ws/landing_uav/landing_ros/devel/lib/whycon/set_axis: /usr/lib/x86_64-linux-gnu/libopencv_aruco.so.4.2.0
/home/ducanh/ducanh_ws/landing_uav/landing_ros/devel/lib/whycon/set_axis: /usr/lib/x86_64-linux-gnu/libopencv_bgsegm.so.4.2.0
/home/ducanh/ducanh_ws/landing_uav/landing_ros/devel/lib/whycon/set_axis: /usr/lib/x86_64-linux-gnu/libopencv_bioinspired.so.4.2.0
/home/ducanh/ducanh_ws/landing_uav/landing_ros/devel/lib/whycon/set_axis: /usr/lib/x86_64-linux-gnu/libopencv_ccalib.so.4.2.0
/home/ducanh/ducanh_ws/landing_uav/landing_ros/devel/lib/whycon/set_axis: /usr/lib/x86_64-linux-gnu/libopencv_dnn_objdetect.so.4.2.0
/home/ducanh/ducanh_ws/landing_uav/landing_ros/devel/lib/whycon/set_axis: /usr/lib/x86_64-linux-gnu/libopencv_dnn_superres.so.4.2.0
/home/ducanh/ducanh_ws/landing_uav/landing_ros/devel/lib/whycon/set_axis: /usr/lib/x86_64-linux-gnu/libopencv_dpm.so.4.2.0
/home/ducanh/ducanh_ws/landing_uav/landing_ros/devel/lib/whycon/set_axis: /usr/lib/x86_64-linux-gnu/libopencv_highgui.so.4.2.0
/home/ducanh/ducanh_ws/landing_uav/landing_ros/devel/lib/whycon/set_axis: /usr/lib/x86_64-linux-gnu/libopencv_face.so.4.2.0
/home/ducanh/ducanh_ws/landing_uav/landing_ros/devel/lib/whycon/set_axis: /usr/lib/x86_64-linux-gnu/libopencv_freetype.so.4.2.0
/home/ducanh/ducanh_ws/landing_uav/landing_ros/devel/lib/whycon/set_axis: /usr/lib/x86_64-linux-gnu/libopencv_fuzzy.so.4.2.0
/home/ducanh/ducanh_ws/landing_uav/landing_ros/devel/lib/whycon/set_axis: /usr/lib/x86_64-linux-gnu/libopencv_hdf.so.4.2.0
/home/ducanh/ducanh_ws/landing_uav/landing_ros/devel/lib/whycon/set_axis: /usr/lib/x86_64-linux-gnu/libopencv_hfs.so.4.2.0
/home/ducanh/ducanh_ws/landing_uav/landing_ros/devel/lib/whycon/set_axis: /usr/lib/x86_64-linux-gnu/libopencv_img_hash.so.4.2.0
/home/ducanh/ducanh_ws/landing_uav/landing_ros/devel/lib/whycon/set_axis: /usr/lib/x86_64-linux-gnu/libopencv_line_descriptor.so.4.2.0
/home/ducanh/ducanh_ws/landing_uav/landing_ros/devel/lib/whycon/set_axis: /usr/lib/x86_64-linux-gnu/libopencv_quality.so.4.2.0
/home/ducanh/ducanh_ws/landing_uav/landing_ros/devel/lib/whycon/set_axis: /usr/lib/x86_64-linux-gnu/libopencv_reg.so.4.2.0
/home/ducanh/ducanh_ws/landing_uav/landing_ros/devel/lib/whycon/set_axis: /usr/lib/x86_64-linux-gnu/libopencv_rgbd.so.4.2.0
/home/ducanh/ducanh_ws/landing_uav/landing_ros/devel/lib/whycon/set_axis: /usr/lib/x86_64-linux-gnu/libopencv_saliency.so.4.2.0
/home/ducanh/ducanh_ws/landing_uav/landing_ros/devel/lib/whycon/set_axis: /usr/lib/x86_64-linux-gnu/libopencv_shape.so.4.2.0
/home/ducanh/ducanh_ws/landing_uav/landing_ros/devel/lib/whycon/set_axis: /usr/lib/x86_64-linux-gnu/libopencv_stereo.so.4.2.0
/home/ducanh/ducanh_ws/landing_uav/landing_ros/devel/lib/whycon/set_axis: /usr/lib/x86_64-linux-gnu/libopencv_structured_light.so.4.2.0
/home/ducanh/ducanh_ws/landing_uav/landing_ros/devel/lib/whycon/set_axis: /usr/lib/x86_64-linux-gnu/libopencv_phase_unwrapping.so.4.2.0
/home/ducanh/ducanh_ws/landing_uav/landing_ros/devel/lib/whycon/set_axis: /usr/lib/x86_64-linux-gnu/libopencv_superres.so.4.2.0
/home/ducanh/ducanh_ws/landing_uav/landing_ros/devel/lib/whycon/set_axis: /usr/lib/x86_64-linux-gnu/libopencv_optflow.so.4.2.0
/home/ducanh/ducanh_ws/landing_uav/landing_ros/devel/lib/whycon/set_axis: /usr/lib/x86_64-linux-gnu/libopencv_surface_matching.so.4.2.0
/home/ducanh/ducanh_ws/landing_uav/landing_ros/devel/lib/whycon/set_axis: /usr/lib/x86_64-linux-gnu/libopencv_tracking.so.4.2.0
/home/ducanh/ducanh_ws/landing_uav/landing_ros/devel/lib/whycon/set_axis: /usr/lib/x86_64-linux-gnu/libopencv_datasets.so.4.2.0
/home/ducanh/ducanh_ws/landing_uav/landing_ros/devel/lib/whycon/set_axis: /usr/lib/x86_64-linux-gnu/libopencv_plot.so.4.2.0
/home/ducanh/ducanh_ws/landing_uav/landing_ros/devel/lib/whycon/set_axis: /usr/lib/x86_64-linux-gnu/libopencv_text.so.4.2.0
/home/ducanh/ducanh_ws/landing_uav/landing_ros/devel/lib/whycon/set_axis: /usr/lib/x86_64-linux-gnu/libopencv_dnn.so.4.2.0
/home/ducanh/ducanh_ws/landing_uav/landing_ros/devel/lib/whycon/set_axis: /usr/lib/x86_64-linux-gnu/libopencv_ml.so.4.2.0
/home/ducanh/ducanh_ws/landing_uav/landing_ros/devel/lib/whycon/set_axis: /usr/lib/x86_64-linux-gnu/libopencv_videostab.so.4.2.0
/home/ducanh/ducanh_ws/landing_uav/landing_ros/devel/lib/whycon/set_axis: /usr/lib/x86_64-linux-gnu/libopencv_videoio.so.4.2.0
/home/ducanh/ducanh_ws/landing_uav/landing_ros/devel/lib/whycon/set_axis: /usr/lib/x86_64-linux-gnu/libopencv_viz.so.4.2.0
/home/ducanh/ducanh_ws/landing_uav/landing_ros/devel/lib/whycon/set_axis: /usr/lib/x86_64-linux-gnu/libopencv_ximgproc.so.4.2.0
/home/ducanh/ducanh_ws/landing_uav/landing_ros/devel/lib/whycon/set_axis: /usr/lib/x86_64-linux-gnu/libopencv_video.so.4.2.0
/home/ducanh/ducanh_ws/landing_uav/landing_ros/devel/lib/whycon/set_axis: /usr/lib/x86_64-linux-gnu/libopencv_xobjdetect.so.4.2.0
/home/ducanh/ducanh_ws/landing_uav/landing_ros/devel/lib/whycon/set_axis: /usr/lib/x86_64-linux-gnu/libopencv_imgcodecs.so.4.2.0
/home/ducanh/ducanh_ws/landing_uav/landing_ros/devel/lib/whycon/set_axis: /usr/lib/x86_64-linux-gnu/libopencv_objdetect.so.4.2.0
/home/ducanh/ducanh_ws/landing_uav/landing_ros/devel/lib/whycon/set_axis: /usr/lib/x86_64-linux-gnu/libopencv_calib3d.so.4.2.0
/home/ducanh/ducanh_ws/landing_uav/landing_ros/devel/lib/whycon/set_axis: /usr/lib/x86_64-linux-gnu/libopencv_features2d.so.4.2.0
/home/ducanh/ducanh_ws/landing_uav/landing_ros/devel/lib/whycon/set_axis: /usr/lib/x86_64-linux-gnu/libopencv_flann.so.4.2.0
/home/ducanh/ducanh_ws/landing_uav/landing_ros/devel/lib/whycon/set_axis: /usr/lib/x86_64-linux-gnu/libopencv_xphoto.so.4.2.0
/home/ducanh/ducanh_ws/landing_uav/landing_ros/devel/lib/whycon/set_axis: /usr/lib/x86_64-linux-gnu/libopencv_photo.so.4.2.0
/home/ducanh/ducanh_ws/landing_uav/landing_ros/devel/lib/whycon/set_axis: /usr/lib/x86_64-linux-gnu/libopencv_imgproc.so.4.2.0
/home/ducanh/ducanh_ws/landing_uav/landing_ros/devel/lib/whycon/set_axis: /usr/lib/x86_64-linux-gnu/libopencv_core.so.4.2.0
/home/ducanh/ducanh_ws/landing_uav/landing_ros/devel/lib/whycon/set_axis: /usr/lib/x86_64-linux-gnu/libboost_program_options.so.1.71.0
/home/ducanh/ducanh_ws/landing_uav/landing_ros/devel/lib/whycon/set_axis: /usr/lib/x86_64-linux-gnu/libboost_thread.so.1.71.0
/home/ducanh/ducanh_ws/landing_uav/landing_ros/devel/lib/whycon/set_axis: /usr/lib/x86_64-linux-gnu/libboost_atomic.so.1.71.0
/home/ducanh/ducanh_ws/landing_uav/landing_ros/devel/lib/whycon/set_axis: /usr/lib/x86_64-linux-gnu/libboost_system.so.1.71.0
/home/ducanh/ducanh_ws/landing_uav/landing_ros/devel/lib/whycon/set_axis: CMakeFiles/set_axis.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/ducanh/ducanh_ws/landing_uav/landing_ros/build/whycon/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Linking CXX executable /home/ducanh/ducanh_ws/landing_uav/landing_ros/devel/lib/whycon/set_axis"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/set_axis.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/set_axis.dir/build: /home/ducanh/ducanh_ws/landing_uav/landing_ros/devel/lib/whycon/set_axis

.PHONY : CMakeFiles/set_axis.dir/build

CMakeFiles/set_axis.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/set_axis.dir/cmake_clean.cmake
.PHONY : CMakeFiles/set_axis.dir/clean

CMakeFiles/set_axis.dir/depend:
	cd /home/ducanh/ducanh_ws/landing_uav/landing_ros/build/whycon && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/ducanh/ducanh_ws/landing_uav/landing_ros/src/whycon /home/ducanh/ducanh_ws/landing_uav/landing_ros/src/whycon /home/ducanh/ducanh_ws/landing_uav/landing_ros/build/whycon /home/ducanh/ducanh_ws/landing_uav/landing_ros/build/whycon /home/ducanh/ducanh_ws/landing_uav/landing_ros/build/whycon/CMakeFiles/set_axis.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/set_axis.dir/depend

