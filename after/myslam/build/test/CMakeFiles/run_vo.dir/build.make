# CMAKE generated file: DO NOT EDIT!
<<<<<<< HEAD
# Generated by "Unix Makefiles" Generator, CMake Version 3.16
=======
# Generated by "Unix Makefiles" Generator, CMake Version 3.23
>>>>>>> fbc9515c775cd61c2cb5a913f937852e2c828cb9

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:

<<<<<<< HEAD

=======
>>>>>>> fbc9515c775cd61c2cb5a913f937852e2c828cb9
#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:

<<<<<<< HEAD

# Remove some rules from gmake that .SUFFIXES does not remove.
SUFFIXES =

.SUFFIXES: .hpux_make_needs_suffix_list


# Suppress display of executed commands.
$(VERBOSE).SILENT:


# A target that is always out of date.
cmake_force:

=======
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
>>>>>>> fbc9515c775cd61c2cb5a913f937852e2c828cb9
.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
<<<<<<< HEAD
CMAKE_COMMAND = /usr/bin/cmake

# The command to remove a file.
RM = /usr/bin/cmake -E remove -f
=======
CMAKE_COMMAND = /usr/local/bin/cmake

# The command to remove a file.
RM = /usr/local/bin/cmake -E rm -f
>>>>>>> fbc9515c775cd61c2cb5a913f937852e2c828cb9

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
<<<<<<< HEAD
CMAKE_SOURCE_DIR = /home/xzy/CLionProjects/myslam

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/xzy/CLionProjects/myslam/build

# Include any dependencies generated for this target.
include test/CMakeFiles/run_vo.dir/depend.make
=======
CMAKE_SOURCE_DIR = /home/guojiawei/cv/cv2/cv/after/myslam

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/guojiawei/cv/cv2/cv/after/myslam/build

# Include any dependencies generated for this target.
include test/CMakeFiles/run_vo.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include test/CMakeFiles/run_vo.dir/compiler_depend.make
>>>>>>> fbc9515c775cd61c2cb5a913f937852e2c828cb9

# Include the progress variables for this target.
include test/CMakeFiles/run_vo.dir/progress.make

# Include the compile flags for this target's objects.
include test/CMakeFiles/run_vo.dir/flags.make

<<<<<<< HEAD
test/CMakeFiles/run_vo.dir/run_vo.cpp.o: test/CMakeFiles/run_vo.dir/flags.make
test/CMakeFiles/run_vo.dir/run_vo.cpp.o: ../test/run_vo.cpp
<<<<<<< HEAD
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/xzy/CLionProjects/myslam/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object test/CMakeFiles/run_vo.dir/run_vo.cpp.o"
	cd /home/xzy/CLionProjects/myslam/build/test && /usr/bin/g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/run_vo.dir/run_vo.cpp.o -c /home/xzy/CLionProjects/myslam/test/run_vo.cpp

test/CMakeFiles/run_vo.dir/run_vo.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/run_vo.dir/run_vo.cpp.i"
	cd /home/xzy/CLionProjects/myslam/build/test && /usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/xzy/CLionProjects/myslam/test/run_vo.cpp > CMakeFiles/run_vo.dir/run_vo.cpp.i

test/CMakeFiles/run_vo.dir/run_vo.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/run_vo.dir/run_vo.cpp.s"
	cd /home/xzy/CLionProjects/myslam/build/test && /usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/xzy/CLionProjects/myslam/test/run_vo.cpp -o CMakeFiles/run_vo.dir/run_vo.cpp.s

# Object files for target run_vo
run_vo_OBJECTS = \
"CMakeFiles/run_vo.dir/run_vo.cpp.o"
=======
test/CMakeFiles/run_vo.dir/run_vo.cpp.o: test/CMakeFiles/run_vo.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/guojiawei/cv/cv2/cv/after/myslam/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object test/CMakeFiles/run_vo.dir/run_vo.cpp.o"
	cd /home/guojiawei/cv/cv2/cv/after/myslam/build/test && /usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT test/CMakeFiles/run_vo.dir/run_vo.cpp.o -MF CMakeFiles/run_vo.dir/run_vo.cpp.o.d -o CMakeFiles/run_vo.dir/run_vo.cpp.o -c /home/guojiawei/cv/cv2/cv/after/myslam/test/run_vo.cpp

test/CMakeFiles/run_vo.dir/run_vo.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/run_vo.dir/run_vo.cpp.i"
	cd /home/guojiawei/cv/cv2/cv/after/myslam/build/test && /usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/guojiawei/cv/cv2/cv/after/myslam/test/run_vo.cpp > CMakeFiles/run_vo.dir/run_vo.cpp.i

test/CMakeFiles/run_vo.dir/run_vo.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/run_vo.dir/run_vo.cpp.s"
	cd /home/guojiawei/cv/cv2/cv/after/myslam/build/test && /usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/guojiawei/cv/cv2/cv/after/myslam/test/run_vo.cpp -o CMakeFiles/run_vo.dir/run_vo.cpp.s

=======
>>>>>>> 57a421dfa97dba6cacb6f122a609cdcfae31afd2
test/CMakeFiles/run_vo.dir/run_vo2.cpp.o: test/CMakeFiles/run_vo.dir/flags.make
test/CMakeFiles/run_vo.dir/run_vo2.cpp.o: ../test/run_vo2.cpp
test/CMakeFiles/run_vo.dir/run_vo2.cpp.o: test/CMakeFiles/run_vo.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/guojiawei/cv/cv2/cv/after/myslam/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object test/CMakeFiles/run_vo.dir/run_vo2.cpp.o"
	cd /home/guojiawei/cv/cv2/cv/after/myslam/build/test && /usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT test/CMakeFiles/run_vo.dir/run_vo2.cpp.o -MF CMakeFiles/run_vo.dir/run_vo2.cpp.o.d -o CMakeFiles/run_vo.dir/run_vo2.cpp.o -c /home/guojiawei/cv/cv2/cv/after/myslam/test/run_vo2.cpp

test/CMakeFiles/run_vo.dir/run_vo2.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/run_vo.dir/run_vo2.cpp.i"
	cd /home/guojiawei/cv/cv2/cv/after/myslam/build/test && /usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/guojiawei/cv/cv2/cv/after/myslam/test/run_vo2.cpp > CMakeFiles/run_vo.dir/run_vo2.cpp.i

test/CMakeFiles/run_vo.dir/run_vo2.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/run_vo.dir/run_vo2.cpp.s"
	cd /home/guojiawei/cv/cv2/cv/after/myslam/build/test && /usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/guojiawei/cv/cv2/cv/after/myslam/test/run_vo2.cpp -o CMakeFiles/run_vo.dir/run_vo2.cpp.s

test/CMakeFiles/run_vo.dir/server.cpp.o: test/CMakeFiles/run_vo.dir/flags.make
test/CMakeFiles/run_vo.dir/server.cpp.o: ../test/server.cpp
test/CMakeFiles/run_vo.dir/server.cpp.o: test/CMakeFiles/run_vo.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/guojiawei/cv/cv2/cv/after/myslam/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object test/CMakeFiles/run_vo.dir/server.cpp.o"
	cd /home/guojiawei/cv/cv2/cv/after/myslam/build/test && /usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT test/CMakeFiles/run_vo.dir/server.cpp.o -MF CMakeFiles/run_vo.dir/server.cpp.o.d -o CMakeFiles/run_vo.dir/server.cpp.o -c /home/guojiawei/cv/cv2/cv/after/myslam/test/server.cpp

test/CMakeFiles/run_vo.dir/server.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/run_vo.dir/server.cpp.i"
	cd /home/guojiawei/cv/cv2/cv/after/myslam/build/test && /usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/guojiawei/cv/cv2/cv/after/myslam/test/server.cpp > CMakeFiles/run_vo.dir/server.cpp.i

test/CMakeFiles/run_vo.dir/server.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/run_vo.dir/server.cpp.s"
	cd /home/guojiawei/cv/cv2/cv/after/myslam/build/test && /usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/guojiawei/cv/cv2/cv/after/myslam/test/server.cpp -o CMakeFiles/run_vo.dir/server.cpp.s

# Object files for target run_vo
run_vo_OBJECTS = \
<<<<<<< HEAD
"CMakeFiles/run_vo.dir/run_vo.cpp.o" \
"CMakeFiles/run_vo.dir/run_vo2.cpp.o"
>>>>>>> fbc9515c775cd61c2cb5a913f937852e2c828cb9
=======
"CMakeFiles/run_vo.dir/run_vo2.cpp.o" \
"CMakeFiles/run_vo.dir/server.cpp.o"
>>>>>>> 57a421dfa97dba6cacb6f122a609cdcfae31afd2

# External object files for target run_vo
run_vo_EXTERNAL_OBJECTS =

<<<<<<< HEAD
../bin/run_vo: test/CMakeFiles/run_vo.dir/run_vo.cpp.o
<<<<<<< HEAD
../bin/run_vo: test/CMakeFiles/run_vo.dir/build.make
../bin/run_vo: ../lib/libmyslam.so
../bin/run_vo: /usr/lib/x86_64-linux-gnu/libopencv_stitching.so.4.2.0
../bin/run_vo: /usr/lib/x86_64-linux-gnu/libopencv_aruco.so.4.2.0
../bin/run_vo: /usr/lib/x86_64-linux-gnu/libopencv_bgsegm.so.4.2.0
../bin/run_vo: /usr/lib/x86_64-linux-gnu/libopencv_bioinspired.so.4.2.0
../bin/run_vo: /usr/lib/x86_64-linux-gnu/libopencv_ccalib.so.4.2.0
../bin/run_vo: /usr/lib/x86_64-linux-gnu/libopencv_dnn_objdetect.so.4.2.0
../bin/run_vo: /usr/lib/x86_64-linux-gnu/libopencv_dnn_superres.so.4.2.0
../bin/run_vo: /usr/lib/x86_64-linux-gnu/libopencv_dpm.so.4.2.0
../bin/run_vo: /usr/lib/x86_64-linux-gnu/libopencv_highgui.so.4.2.0
../bin/run_vo: /usr/lib/x86_64-linux-gnu/libopencv_face.so.4.2.0
../bin/run_vo: /usr/lib/x86_64-linux-gnu/libopencv_freetype.so.4.2.0
../bin/run_vo: /usr/lib/x86_64-linux-gnu/libopencv_fuzzy.so.4.2.0
../bin/run_vo: /usr/lib/x86_64-linux-gnu/libopencv_hdf.so.4.2.0
../bin/run_vo: /usr/lib/x86_64-linux-gnu/libopencv_hfs.so.4.2.0
../bin/run_vo: /usr/lib/x86_64-linux-gnu/libopencv_img_hash.so.4.2.0
../bin/run_vo: /usr/lib/x86_64-linux-gnu/libopencv_line_descriptor.so.4.2.0
../bin/run_vo: /usr/lib/x86_64-linux-gnu/libopencv_quality.so.4.2.0
../bin/run_vo: /usr/lib/x86_64-linux-gnu/libopencv_reg.so.4.2.0
../bin/run_vo: /usr/lib/x86_64-linux-gnu/libopencv_rgbd.so.4.2.0
../bin/run_vo: /usr/lib/x86_64-linux-gnu/libopencv_saliency.so.4.2.0
../bin/run_vo: /usr/lib/x86_64-linux-gnu/libopencv_shape.so.4.2.0
../bin/run_vo: /usr/lib/x86_64-linux-gnu/libopencv_stereo.so.4.2.0
../bin/run_vo: /usr/lib/x86_64-linux-gnu/libopencv_structured_light.so.4.2.0
../bin/run_vo: /usr/lib/x86_64-linux-gnu/libopencv_phase_unwrapping.so.4.2.0
../bin/run_vo: /usr/lib/x86_64-linux-gnu/libopencv_superres.so.4.2.0
../bin/run_vo: /usr/lib/x86_64-linux-gnu/libopencv_optflow.so.4.2.0
../bin/run_vo: /usr/lib/x86_64-linux-gnu/libopencv_surface_matching.so.4.2.0
../bin/run_vo: /usr/lib/x86_64-linux-gnu/libopencv_tracking.so.4.2.0
../bin/run_vo: /usr/lib/x86_64-linux-gnu/libopencv_datasets.so.4.2.0
../bin/run_vo: /usr/lib/x86_64-linux-gnu/libopencv_plot.so.4.2.0
../bin/run_vo: /usr/lib/x86_64-linux-gnu/libopencv_text.so.4.2.0
../bin/run_vo: /usr/lib/x86_64-linux-gnu/libopencv_dnn.so.4.2.0
../bin/run_vo: /usr/lib/x86_64-linux-gnu/libopencv_ml.so.4.2.0
../bin/run_vo: /usr/lib/x86_64-linux-gnu/libopencv_videostab.so.4.2.0
../bin/run_vo: /usr/lib/x86_64-linux-gnu/libopencv_videoio.so.4.2.0
../bin/run_vo: /usr/lib/x86_64-linux-gnu/libopencv_viz.so.4.2.0
../bin/run_vo: /usr/lib/x86_64-linux-gnu/libopencv_ximgproc.so.4.2.0
../bin/run_vo: /usr/lib/x86_64-linux-gnu/libopencv_video.so.4.2.0
../bin/run_vo: /usr/lib/x86_64-linux-gnu/libopencv_xobjdetect.so.4.2.0
../bin/run_vo: /usr/lib/x86_64-linux-gnu/libopencv_imgcodecs.so.4.2.0
../bin/run_vo: /usr/lib/x86_64-linux-gnu/libopencv_objdetect.so.4.2.0
../bin/run_vo: /usr/lib/x86_64-linux-gnu/libopencv_calib3d.so.4.2.0
../bin/run_vo: /usr/lib/x86_64-linux-gnu/libopencv_features2d.so.4.2.0
../bin/run_vo: /usr/lib/x86_64-linux-gnu/libopencv_flann.so.4.2.0
../bin/run_vo: /usr/lib/x86_64-linux-gnu/libopencv_xphoto.so.4.2.0
../bin/run_vo: /usr/lib/x86_64-linux-gnu/libopencv_photo.so.4.2.0
../bin/run_vo: /usr/lib/x86_64-linux-gnu/libopencv_imgproc.so.4.2.0
../bin/run_vo: /usr/lib/x86_64-linux-gnu/libopencv_core.so.4.2.0
../bin/run_vo: /home/xzy/Downloads/Sophus/build/libSophus.so
../bin/run_vo: /usr/lib/x86_64-linux-gnu/libpcl_apps.so
../bin/run_vo: /usr/lib/x86_64-linux-gnu/libpcl_surface.so
../bin/run_vo: /usr/lib/x86_64-linux-gnu/libpcl_keypoints.so
../bin/run_vo: /usr/lib/x86_64-linux-gnu/libpcl_tracking.so
../bin/run_vo: /usr/lib/x86_64-linux-gnu/libpcl_recognition.so
../bin/run_vo: /usr/lib/x86_64-linux-gnu/libpcl_registration.so
../bin/run_vo: /usr/lib/x86_64-linux-gnu/libpcl_stereo.so
../bin/run_vo: /usr/lib/x86_64-linux-gnu/libpcl_outofcore.so
../bin/run_vo: /usr/lib/x86_64-linux-gnu/libpcl_people.so
../bin/run_vo: /usr/lib/x86_64-linux-gnu/libpcl_segmentation.so
../bin/run_vo: /usr/lib/x86_64-linux-gnu/libpcl_features.so
../bin/run_vo: /usr/lib/x86_64-linux-gnu/libpcl_filters.so
../bin/run_vo: /usr/lib/x86_64-linux-gnu/libpcl_sample_consensus.so
../bin/run_vo: /usr/lib/x86_64-linux-gnu/libpcl_ml.so
../bin/run_vo: /usr/lib/x86_64-linux-gnu/libpcl_visualization.so
../bin/run_vo: /usr/lib/x86_64-linux-gnu/libpcl_search.so
../bin/run_vo: /usr/lib/x86_64-linux-gnu/libpcl_kdtree.so
../bin/run_vo: /usr/lib/x86_64-linux-gnu/libpcl_io.so
../bin/run_vo: /usr/lib/x86_64-linux-gnu/libpcl_octree.so
../bin/run_vo: /usr/lib/x86_64-linux-gnu/libpcl_common.so
../bin/run_vo: /usr/lib/x86_64-linux-gnu/libboost_system.so
../bin/run_vo: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
../bin/run_vo: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
../bin/run_vo: /usr/lib/x86_64-linux-gnu/libboost_iostreams.so
../bin/run_vo: /usr/lib/x86_64-linux-gnu/libboost_regex.so
../bin/run_vo: /usr/lib/x86_64-linux-gnu/libqhull.so
../bin/run_vo: /usr/lib/libOpenNI.so
../bin/run_vo: /usr/lib/libOpenNI2.so
../bin/run_vo: /usr/lib/x86_64-linux-gnu/libvtkChartsCore-7.1.so.7.1p.1
../bin/run_vo: /usr/lib/x86_64-linux-gnu/libvtkInfovisCore-7.1.so.7.1p.1
../bin/run_vo: /usr/lib/x86_64-linux-gnu/libjpeg.so
../bin/run_vo: /usr/lib/x86_64-linux-gnu/libpng.so
../bin/run_vo: /usr/lib/x86_64-linux-gnu/libtiff.so
../bin/run_vo: /usr/lib/x86_64-linux-gnu/libexpat.so
../bin/run_vo: /usr/lib/x86_64-linux-gnu/libvtkIOGeometry-7.1.so.7.1p.1
../bin/run_vo: /usr/lib/x86_64-linux-gnu/libvtkIOLegacy-7.1.so.7.1p.1
../bin/run_vo: /usr/lib/x86_64-linux-gnu/libvtkIOPLY-7.1.so.7.1p.1
../bin/run_vo: /usr/lib/x86_64-linux-gnu/libvtkRenderingLOD-7.1.so.7.1p.1
../bin/run_vo: /usr/lib/x86_64-linux-gnu/libvtkViewsContext2D-7.1.so.7.1p.1
../bin/run_vo: /usr/lib/x86_64-linux-gnu/libvtkViewsCore-7.1.so.7.1p.1
../bin/run_vo: /usr/lib/x86_64-linux-gnu/libvtkInteractionWidgets-7.1.so.7.1p.1
../bin/run_vo: /usr/lib/x86_64-linux-gnu/libvtkFiltersModeling-7.1.so.7.1p.1
../bin/run_vo: /usr/lib/x86_64-linux-gnu/libvtkInteractionStyle-7.1.so.7.1p.1
../bin/run_vo: /usr/lib/x86_64-linux-gnu/libvtkFiltersExtraction-7.1.so.7.1p.1
../bin/run_vo: /usr/lib/x86_64-linux-gnu/libvtkFiltersStatistics-7.1.so.7.1p.1
../bin/run_vo: /usr/lib/x86_64-linux-gnu/libvtkImagingFourier-7.1.so.7.1p.1
../bin/run_vo: /usr/lib/x86_64-linux-gnu/libvtkalglib-7.1.so.7.1p.1
../bin/run_vo: /usr/lib/x86_64-linux-gnu/libvtkFiltersHybrid-7.1.so.7.1p.1
../bin/run_vo: /usr/lib/x86_64-linux-gnu/libvtkImagingGeneral-7.1.so.7.1p.1
../bin/run_vo: /usr/lib/x86_64-linux-gnu/libvtkImagingSources-7.1.so.7.1p.1
../bin/run_vo: /usr/lib/x86_64-linux-gnu/libvtkImagingHybrid-7.1.so.7.1p.1
../bin/run_vo: /usr/lib/x86_64-linux-gnu/libvtkRenderingAnnotation-7.1.so.7.1p.1
../bin/run_vo: /usr/lib/x86_64-linux-gnu/libvtkImagingColor-7.1.so.7.1p.1
../bin/run_vo: /usr/lib/x86_64-linux-gnu/libvtkRenderingVolume-7.1.so.7.1p.1
../bin/run_vo: /usr/lib/x86_64-linux-gnu/libvtkIOXML-7.1.so.7.1p.1
../bin/run_vo: /usr/lib/x86_64-linux-gnu/libvtkIOXMLParser-7.1.so.7.1p.1
../bin/run_vo: /usr/lib/x86_64-linux-gnu/libvtkIOCore-7.1.so.7.1p.1
../bin/run_vo: /usr/lib/x86_64-linux-gnu/libvtkRenderingContextOpenGL2-7.1.so.7.1p.1
../bin/run_vo: /usr/lib/x86_64-linux-gnu/libvtkRenderingContext2D-7.1.so.7.1p.1
../bin/run_vo: /usr/lib/x86_64-linux-gnu/libvtkRenderingFreeType-7.1.so.7.1p.1
../bin/run_vo: /usr/lib/x86_64-linux-gnu/libfreetype.so
../bin/run_vo: /usr/lib/x86_64-linux-gnu/libvtkRenderingOpenGL2-7.1.so.7.1p.1
../bin/run_vo: /usr/lib/x86_64-linux-gnu/libvtkImagingCore-7.1.so.7.1p.1
../bin/run_vo: /usr/lib/x86_64-linux-gnu/libvtkRenderingCore-7.1.so.7.1p.1
../bin/run_vo: /usr/lib/x86_64-linux-gnu/libvtkCommonColor-7.1.so.7.1p.1
../bin/run_vo: /usr/lib/x86_64-linux-gnu/libvtkFiltersGeometry-7.1.so.7.1p.1
../bin/run_vo: /usr/lib/x86_64-linux-gnu/libvtkFiltersSources-7.1.so.7.1p.1
../bin/run_vo: /usr/lib/x86_64-linux-gnu/libvtkFiltersGeneral-7.1.so.7.1p.1
../bin/run_vo: /usr/lib/x86_64-linux-gnu/libvtkCommonComputationalGeometry-7.1.so.7.1p.1
../bin/run_vo: /usr/lib/x86_64-linux-gnu/libvtkFiltersCore-7.1.so.7.1p.1
../bin/run_vo: /usr/lib/x86_64-linux-gnu/libvtkIOImage-7.1.so.7.1p.1
../bin/run_vo: /usr/lib/x86_64-linux-gnu/libvtkCommonExecutionModel-7.1.so.7.1p.1
../bin/run_vo: /usr/lib/x86_64-linux-gnu/libvtkCommonDataModel-7.1.so.7.1p.1
../bin/run_vo: /usr/lib/x86_64-linux-gnu/libvtkCommonTransforms-7.1.so.7.1p.1
../bin/run_vo: /usr/lib/x86_64-linux-gnu/libvtkCommonMisc-7.1.so.7.1p.1
../bin/run_vo: /usr/lib/x86_64-linux-gnu/libvtkCommonMath-7.1.so.7.1p.1
../bin/run_vo: /usr/lib/x86_64-linux-gnu/libvtkCommonSystem-7.1.so.7.1p.1
../bin/run_vo: /usr/lib/x86_64-linux-gnu/libvtkCommonCore-7.1.so.7.1p.1
../bin/run_vo: /usr/lib/x86_64-linux-gnu/libvtksys-7.1.so.7.1p.1
../bin/run_vo: /usr/lib/x86_64-linux-gnu/libvtkDICOMParser-7.1.so.7.1p.1
../bin/run_vo: /usr/lib/x86_64-linux-gnu/libvtkmetaio-7.1.so.7.1p.1
../bin/run_vo: /usr/lib/x86_64-linux-gnu/libz.so
../bin/run_vo: /usr/lib/x86_64-linux-gnu/libGLEW.so
../bin/run_vo: /usr/lib/x86_64-linux-gnu/libSM.so
../bin/run_vo: /usr/lib/x86_64-linux-gnu/libICE.so
../bin/run_vo: /usr/lib/x86_64-linux-gnu/libX11.so
../bin/run_vo: /usr/lib/x86_64-linux-gnu/libXext.so
../bin/run_vo: /usr/lib/x86_64-linux-gnu/libXt.so
../bin/run_vo: /usr/lib/x86_64-linux-gnu/libflann_cpp.so
../bin/run_vo: /usr/local/lib/libglog.a
../bin/run_vo: /usr/local/lib/libgflags.a
../bin/run_vo: test/CMakeFiles/run_vo.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/xzy/CLionProjects/myslam/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable ../../bin/run_vo"
	cd /home/xzy/CLionProjects/myslam/build/test && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/run_vo.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
test/CMakeFiles/run_vo.dir/build: ../bin/run_vo

.PHONY : test/CMakeFiles/run_vo.dir/build

test/CMakeFiles/run_vo.dir/clean:
	cd /home/xzy/CLionProjects/myslam/build/test && $(CMAKE_COMMAND) -P CMakeFiles/run_vo.dir/cmake_clean.cmake
.PHONY : test/CMakeFiles/run_vo.dir/clean

test/CMakeFiles/run_vo.dir/depend:
	cd /home/xzy/CLionProjects/myslam/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/xzy/CLionProjects/myslam /home/xzy/CLionProjects/myslam/test /home/xzy/CLionProjects/myslam/build /home/xzy/CLionProjects/myslam/build/test /home/xzy/CLionProjects/myslam/build/test/CMakeFiles/run_vo.dir/DependInfo.cmake --color=$(COLOR)
=======
=======
>>>>>>> 57a421dfa97dba6cacb6f122a609cdcfae31afd2
../bin/run_vo: test/CMakeFiles/run_vo.dir/run_vo2.cpp.o
../bin/run_vo: test/CMakeFiles/run_vo.dir/server.cpp.o
../bin/run_vo: test/CMakeFiles/run_vo.dir/build.make
../bin/run_vo: ../lib/libmyslam.so
../bin/run_vo: /usr/local/lib/libopencv_gapi.so.4.5.5
../bin/run_vo: /usr/local/lib/libopencv_highgui.so.4.5.5
../bin/run_vo: /usr/local/lib/libopencv_ml.so.4.5.5
../bin/run_vo: /usr/local/lib/libopencv_objdetect.so.4.5.5
../bin/run_vo: /usr/local/lib/libopencv_photo.so.4.5.5
../bin/run_vo: /usr/local/lib/libopencv_stitching.so.4.5.5
../bin/run_vo: /usr/local/lib/libopencv_video.so.4.5.5
../bin/run_vo: /usr/local/lib/libopencv_calib3d.so.4.5.5
../bin/run_vo: /usr/local/lib/libopencv_dnn.so.4.5.5
../bin/run_vo: /usr/local/lib/libopencv_features2d.so.4.5.5
../bin/run_vo: /usr/local/lib/libopencv_flann.so.4.5.5
../bin/run_vo: /usr/local/lib/libopencv_videoio.so.4.5.5
../bin/run_vo: /usr/local/lib/libopencv_imgcodecs.so.4.5.5
../bin/run_vo: /usr/local/lib/libopencv_imgproc.so.4.5.5
../bin/run_vo: /usr/local/lib/libopencv_viz.so.4.5.5
../bin/run_vo: /usr/local/lib/libopencv_core.so.4.5.5
../bin/run_vo: /usr/local/lib/libSophus.so
../bin/run_vo: /usr/lib/x86_64-linux-gnu/libvtkDomainsChemistry-6.3.so.6.3.0
../bin/run_vo: /usr/lib/x86_64-linux-gnu/libvtkFiltersGeneric-6.3.so.6.3.0
../bin/run_vo: /usr/lib/x86_64-linux-gnu/libvtkFiltersHyperTree-6.3.so.6.3.0
../bin/run_vo: /usr/lib/x86_64-linux-gnu/libvtkFiltersParallelFlowPaths-6.3.so.6.3.0
../bin/run_vo: /usr/lib/x86_64-linux-gnu/libvtkFiltersFlowPaths-6.3.so.6.3.0
../bin/run_vo: /usr/lib/x86_64-linux-gnu/libvtkFiltersParallelGeometry-6.3.so.6.3.0
../bin/run_vo: /usr/lib/x86_64-linux-gnu/libvtkFiltersParallelImaging-6.3.so.6.3.0
../bin/run_vo: /usr/lib/x86_64-linux-gnu/libvtkFiltersParallelMPI-6.3.so.6.3.0
../bin/run_vo: /usr/lib/x86_64-linux-gnu/libvtkFiltersParallelStatistics-6.3.so.6.3.0
../bin/run_vo: /usr/lib/x86_64-linux-gnu/libvtkFiltersProgrammable-6.3.so.6.3.0
../bin/run_vo: /usr/lib/x86_64-linux-gnu/libvtkFiltersPython-6.3.so.6.3.0
../bin/run_vo: /usr/lib/libvtkWrappingTools-6.3.a
../bin/run_vo: /usr/lib/x86_64-linux-gnu/libvtkFiltersReebGraph-6.3.so.6.3.0
../bin/run_vo: /usr/lib/x86_64-linux-gnu/libvtkFiltersSMP-6.3.so.6.3.0
../bin/run_vo: /usr/lib/x86_64-linux-gnu/libvtkFiltersSelection-6.3.so.6.3.0
../bin/run_vo: /usr/lib/x86_64-linux-gnu/libvtkFiltersVerdict-6.3.so.6.3.0
../bin/run_vo: /usr/lib/x86_64-linux-gnu/libvtkverdict-6.3.so.6.3.0
../bin/run_vo: /usr/lib/x86_64-linux-gnu/libvtkGUISupportQtOpenGL-6.3.so.6.3.0
../bin/run_vo: /usr/lib/x86_64-linux-gnu/libvtkGUISupportQtSQL-6.3.so.6.3.0
../bin/run_vo: /usr/lib/x86_64-linux-gnu/libvtkGUISupportQtWebkit-6.3.so.6.3.0
../bin/run_vo: /usr/lib/x86_64-linux-gnu/libvtkViewsQt-6.3.so.6.3.0
../bin/run_vo: /usr/lib/x86_64-linux-gnu/libvtkIOAMR-6.3.so.6.3.0
../bin/run_vo: /usr/lib/x86_64-linux-gnu/libvtkIOEnSight-6.3.so.6.3.0
../bin/run_vo: /usr/lib/x86_64-linux-gnu/libvtkIOExport-6.3.so.6.3.0
../bin/run_vo: /usr/lib/x86_64-linux-gnu/libvtkRenderingGL2PS-6.3.so.6.3.0
../bin/run_vo: /usr/lib/x86_64-linux-gnu/libvtkRenderingContextOpenGL-6.3.so.6.3.0
../bin/run_vo: /usr/lib/x86_64-linux-gnu/libvtkIOFFMPEG-6.3.so.6.3.0
../bin/run_vo: /usr/lib/x86_64-linux-gnu/libvtkIOMovie-6.3.so.6.3.0
../bin/run_vo: /usr/lib/x86_64-linux-gnu/libvtkIOGDAL-6.3.so.6.3.0
../bin/run_vo: /usr/lib/x86_64-linux-gnu/libvtkIOGeoJSON-6.3.so.6.3.0
../bin/run_vo: /usr/lib/x86_64-linux-gnu/libvtkIOImport-6.3.so.6.3.0
../bin/run_vo: /usr/lib/x86_64-linux-gnu/libvtkIOInfovis-6.3.so.6.3.0
../bin/run_vo: /usr/lib/x86_64-linux-gnu/libvtkIOMINC-6.3.so.6.3.0
../bin/run_vo: /usr/lib/x86_64-linux-gnu/libvtkIOMPIImage-6.3.so.6.3.0
../bin/run_vo: /usr/lib/x86_64-linux-gnu/libvtkIOMPIParallel-6.3.so.6.3.0
../bin/run_vo: /usr/lib/x86_64-linux-gnu/libvtkIOParallel-6.3.so.6.3.0
../bin/run_vo: /usr/lib/x86_64-linux-gnu/libvtkIONetCDF-6.3.so.6.3.0
../bin/run_vo: /usr/lib/x86_64-linux-gnu/libvtkIOMySQL-6.3.so.6.3.0
../bin/run_vo: /usr/lib/x86_64-linux-gnu/libvtkIOODBC-6.3.so.6.3.0
../bin/run_vo: /usr/lib/x86_64-linux-gnu/libvtkIOPLY-6.3.so.6.3.0
../bin/run_vo: /usr/lib/x86_64-linux-gnu/libvtkIOParallelExodus-6.3.so.6.3.0
../bin/run_vo: /usr/lib/x86_64-linux-gnu/libvtkIOExodus-6.3.so.6.3.0
../bin/run_vo: /usr/lib/x86_64-linux-gnu/libvtkexoIIc-6.3.so.6.3.0
../bin/run_vo: /usr/lib/x86_64-linux-gnu/libvtkIOParallelLSDyna-6.3.so.6.3.0
../bin/run_vo: /usr/lib/x86_64-linux-gnu/libvtkIOLSDyna-6.3.so.6.3.0
../bin/run_vo: /usr/lib/x86_64-linux-gnu/libvtkIOParallelNetCDF-6.3.so.6.3.0
../bin/run_vo: /usr/lib/x86_64-linux-gnu/libvtkIOParallelXML-6.3.so.6.3.0
../bin/run_vo: /usr/lib/x86_64-linux-gnu/libvtkIOPostgreSQL-6.3.so.6.3.0
../bin/run_vo: /usr/lib/x86_64-linux-gnu/libvtkIOVPIC-6.3.so.6.3.0
../bin/run_vo: /usr/lib/x86_64-linux-gnu/libvtkVPIC-6.3.so.6.3.0
../bin/run_vo: /usr/lib/x86_64-linux-gnu/libvtkIOVideo-6.3.so.6.3.0
../bin/run_vo: /usr/lib/x86_64-linux-gnu/libvtkIOXdmf2-6.3.so.6.3.0
../bin/run_vo: /usr/lib/x86_64-linux-gnu/libvtkxdmf2-6.3.so.6.3.0
../bin/run_vo: /usr/lib/x86_64-linux-gnu/libvtkImagingMath-6.3.so.6.3.0
../bin/run_vo: /usr/lib/x86_64-linux-gnu/libvtkImagingMorphological-6.3.so.6.3.0
../bin/run_vo: /usr/lib/x86_64-linux-gnu/libvtkImagingStatistics-6.3.so.6.3.0
../bin/run_vo: /usr/lib/x86_64-linux-gnu/libvtkImagingStencil-6.3.so.6.3.0
../bin/run_vo: /usr/lib/x86_64-linux-gnu/libvtkInteractionImage-6.3.so.6.3.0
../bin/run_vo: /usr/lib/x86_64-linux-gnu/libvtkLocalExample-6.3.so.6.3.0
../bin/run_vo: /usr/lib/x86_64-linux-gnu/libvtkParallelMPI4Py-6.3.so.6.3.0
../bin/run_vo: /usr/lib/x86_64-linux-gnu/libvtkRenderingExternal-6.3.so.6.3.0
../bin/run_vo: /usr/lib/x86_64-linux-gnu/libvtkRenderingFreeTypeFontConfig-6.3.so.6.3.0
../bin/run_vo: /usr/lib/x86_64-linux-gnu/libvtkRenderingImage-6.3.so.6.3.0
../bin/run_vo: /usr/lib/x86_64-linux-gnu/libvtkRenderingLOD-6.3.so.6.3.0
../bin/run_vo: /usr/lib/x86_64-linux-gnu/libvtkRenderingMatplotlib-6.3.so.6.3.0
../bin/run_vo: /usr/lib/x86_64-linux-gnu/libvtkWrappingPython27Core-6.3.so.6.3.0
../bin/run_vo: /usr/lib/x86_64-linux-gnu/libvtkPythonInterpreter-6.3.so.6.3.0
../bin/run_vo: /usr/lib/x86_64-linux-gnu/libvtkRenderingParallel-6.3.so.6.3.0
../bin/run_vo: /usr/lib/x86_64-linux-gnu/libvtkFiltersParallel-6.3.so.6.3.0
../bin/run_vo: /usr/lib/x86_64-linux-gnu/libvtkRenderingParallelLIC-6.3.so.6.3.0
../bin/run_vo: /usr/lib/x86_64-linux-gnu/libvtkParallelMPI-6.3.so.6.3.0
../bin/run_vo: /usr/lib/x86_64-linux-gnu/libvtkRenderingLIC-6.3.so.6.3.0
../bin/run_vo: /usr/lib/x86_64-linux-gnu/libvtkRenderingQt-6.3.so.6.3.0
../bin/run_vo: /usr/lib/x86_64-linux-gnu/libvtkFiltersTexture-6.3.so.6.3.0
../bin/run_vo: /usr/lib/x86_64-linux-gnu/libvtkGUISupportQt-6.3.so.6.3.0
../bin/run_vo: /usr/lib/x86_64-linux-gnu/libQt5Widgets.so.5.9.5
../bin/run_vo: /usr/lib/x86_64-linux-gnu/libQt5Gui.so.5.9.5
../bin/run_vo: /usr/lib/x86_64-linux-gnu/libQt5Core.so.5.9.5
../bin/run_vo: /usr/lib/x86_64-linux-gnu/libvtkRenderingVolumeAMR-6.3.so.6.3.0
../bin/run_vo: /usr/lib/x86_64-linux-gnu/libvtkFiltersAMR-6.3.so.6.3.0
../bin/run_vo: /usr/lib/x86_64-linux-gnu/libvtkParallelCore-6.3.so.6.3.0
../bin/run_vo: /usr/lib/x86_64-linux-gnu/libvtkIOLegacy-6.3.so.6.3.0
../bin/run_vo: /usr/lib/x86_64-linux-gnu/libvtkRenderingVolumeOpenGL-6.3.so.6.3.0
../bin/run_vo: /usr/lib/x86_64-linux-gnu/libvtkRenderingOpenGL-6.3.so.6.3.0
../bin/run_vo: /usr/lib/x86_64-linux-gnu/libGLU.so
../bin/run_vo: /usr/lib/x86_64-linux-gnu/libSM.so
../bin/run_vo: /usr/lib/x86_64-linux-gnu/libICE.so
../bin/run_vo: /usr/lib/x86_64-linux-gnu/libX11.so
../bin/run_vo: /usr/lib/x86_64-linux-gnu/libXext.so
../bin/run_vo: /usr/lib/x86_64-linux-gnu/libXt.so
../bin/run_vo: /usr/lib/x86_64-linux-gnu/libvtkTestingGenericBridge-6.3.so.6.3.0
../bin/run_vo: /usr/lib/x86_64-linux-gnu/libvtkTestingIOSQL-6.3.so.6.3.0
../bin/run_vo: /usr/lib/x86_64-linux-gnu/libvtkIOSQL-6.3.so.6.3.0
../bin/run_vo: /usr/lib/x86_64-linux-gnu/libvtkTestingRendering-6.3.so.6.3.0
../bin/run_vo: /usr/lib/x86_64-linux-gnu/libvtkViewsContext2D-6.3.so.6.3.0
../bin/run_vo: /usr/lib/x86_64-linux-gnu/libvtkViewsGeovis-6.3.so.6.3.0
../bin/run_vo: /usr/lib/x86_64-linux-gnu/libvtkViewsInfovis-6.3.so.6.3.0
../bin/run_vo: /usr/lib/x86_64-linux-gnu/libvtkChartsCore-6.3.so.6.3.0
../bin/run_vo: /usr/lib/x86_64-linux-gnu/libvtkRenderingContext2D-6.3.so.6.3.0
../bin/run_vo: /usr/lib/x86_64-linux-gnu/libvtkFiltersImaging-6.3.so.6.3.0
../bin/run_vo: /usr/lib/x86_64-linux-gnu/libvtkRenderingLabel-6.3.so.6.3.0
../bin/run_vo: /usr/lib/x86_64-linux-gnu/libvtkGeovisCore-6.3.so.6.3.0
../bin/run_vo: /usr/lib/x86_64-linux-gnu/libvtkIOXML-6.3.so.6.3.0
../bin/run_vo: /usr/lib/x86_64-linux-gnu/libvtkIOGeometry-6.3.so.6.3.0
../bin/run_vo: /usr/lib/x86_64-linux-gnu/libvtkIOXMLParser-6.3.so.6.3.0
../bin/run_vo: /usr/lib/x86_64-linux-gnu/libvtkInfovisLayout-6.3.so.6.3.0
../bin/run_vo: /usr/lib/x86_64-linux-gnu/libvtkInfovisBoostGraphAlgorithms-6.3.so.6.3.0
../bin/run_vo: /usr/lib/x86_64-linux-gnu/libvtkInfovisCore-6.3.so.6.3.0
../bin/run_vo: /usr/lib/x86_64-linux-gnu/libvtkViewsCore-6.3.so.6.3.0
../bin/run_vo: /usr/lib/x86_64-linux-gnu/libvtkInteractionWidgets-6.3.so.6.3.0
../bin/run_vo: /usr/lib/x86_64-linux-gnu/libvtkFiltersHybrid-6.3.so.6.3.0
../bin/run_vo: /usr/lib/x86_64-linux-gnu/libvtkImagingGeneral-6.3.so.6.3.0
../bin/run_vo: /usr/lib/x86_64-linux-gnu/libvtkImagingSources-6.3.so.6.3.0
../bin/run_vo: /usr/lib/x86_64-linux-gnu/libvtkFiltersModeling-6.3.so.6.3.0
../bin/run_vo: /usr/lib/x86_64-linux-gnu/libvtkInteractionStyle-6.3.so.6.3.0
../bin/run_vo: /usr/lib/x86_64-linux-gnu/libvtkImagingHybrid-6.3.so.6.3.0
../bin/run_vo: /usr/lib/x86_64-linux-gnu/libvtkIOImage-6.3.so.6.3.0
../bin/run_vo: /usr/lib/x86_64-linux-gnu/libvtkDICOMParser-6.3.so.6.3.0
../bin/run_vo: /usr/lib/x86_64-linux-gnu/libvtkIOCore-6.3.so.6.3.0
../bin/run_vo: /usr/lib/x86_64-linux-gnu/libvtkmetaio-6.3.so.6.3.0
../bin/run_vo: /usr/lib/x86_64-linux-gnu/libvtkRenderingAnnotation-6.3.so.6.3.0
../bin/run_vo: /usr/lib/x86_64-linux-gnu/libvtkRenderingFreeType-6.3.so.6.3.0
../bin/run_vo: /usr/lib/x86_64-linux-gnu/libvtkftgl-6.3.so.6.3.0
../bin/run_vo: /usr/lib/x86_64-linux-gnu/libGL.so
../bin/run_vo: /usr/lib/x86_64-linux-gnu/libvtkImagingColor-6.3.so.6.3.0
../bin/run_vo: /usr/lib/x86_64-linux-gnu/libvtkRenderingVolume-6.3.so.6.3.0
../bin/run_vo: /usr/lib/x86_64-linux-gnu/libvtkRenderingCore-6.3.so.6.3.0
../bin/run_vo: /usr/lib/x86_64-linux-gnu/libvtkCommonColor-6.3.so.6.3.0
../bin/run_vo: /usr/lib/x86_64-linux-gnu/libvtkFiltersExtraction-6.3.so.6.3.0
../bin/run_vo: /usr/lib/x86_64-linux-gnu/libvtkFiltersStatistics-6.3.so.6.3.0
../bin/run_vo: /usr/lib/x86_64-linux-gnu/libvtkImagingFourier-6.3.so.6.3.0
../bin/run_vo: /usr/lib/x86_64-linux-gnu/libvtkImagingCore-6.3.so.6.3.0
../bin/run_vo: /usr/lib/x86_64-linux-gnu/libvtkalglib-6.3.so.6.3.0
../bin/run_vo: /usr/lib/x86_64-linux-gnu/libvtkFiltersGeometry-6.3.so.6.3.0
../bin/run_vo: /usr/lib/x86_64-linux-gnu/libvtkFiltersSources-6.3.so.6.3.0
../bin/run_vo: /usr/lib/x86_64-linux-gnu/libvtkFiltersGeneral-6.3.so.6.3.0
../bin/run_vo: /usr/lib/x86_64-linux-gnu/libvtkFiltersCore-6.3.so.6.3.0
../bin/run_vo: /usr/lib/x86_64-linux-gnu/libvtkCommonExecutionModel-6.3.so.6.3.0
../bin/run_vo: /usr/lib/x86_64-linux-gnu/libvtkCommonComputationalGeometry-6.3.so.6.3.0
../bin/run_vo: /usr/lib/x86_64-linux-gnu/libvtkCommonDataModel-6.3.so.6.3.0
../bin/run_vo: /usr/lib/x86_64-linux-gnu/libvtkCommonMisc-6.3.so.6.3.0
../bin/run_vo: /usr/lib/x86_64-linux-gnu/libvtkCommonTransforms-6.3.so.6.3.0
../bin/run_vo: /usr/lib/x86_64-linux-gnu/libvtkCommonMath-6.3.so.6.3.0
../bin/run_vo: /usr/lib/x86_64-linux-gnu/libvtkCommonSystem-6.3.so.6.3.0
../bin/run_vo: /usr/lib/x86_64-linux-gnu/libvtksys-6.3.so.6.3.0
../bin/run_vo: /usr/lib/x86_64-linux-gnu/libvtkWrappingJava-6.3.so.6.3.0
../bin/run_vo: /usr/lib/x86_64-linux-gnu/libvtkCommonCore-6.3.so.6.3.0
../bin/run_vo: /usr/lib/x86_64-linux-gnu/libboost_system.so
../bin/run_vo: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
../bin/run_vo: /usr/lib/x86_64-linux-gnu/libboost_thread.so
../bin/run_vo: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
../bin/run_vo: /usr/lib/x86_64-linux-gnu/libboost_iostreams.so
../bin/run_vo: /usr/lib/x86_64-linux-gnu/libboost_serialization.so
../bin/run_vo: /usr/lib/x86_64-linux-gnu/libboost_chrono.so
../bin/run_vo: /usr/lib/x86_64-linux-gnu/libboost_atomic.so
../bin/run_vo: /usr/lib/x86_64-linux-gnu/libboost_regex.so
../bin/run_vo: /usr/lib/x86_64-linux-gnu/libpcl_common.so
../bin/run_vo: /usr/lib/x86_64-linux-gnu/libpcl_octree.so
../bin/run_vo: /usr/lib/libOpenNI.so
../bin/run_vo: /usr/lib/libOpenNI2.so
../bin/run_vo: /usr/lib/x86_64-linux-gnu/libexpat.so
../bin/run_vo: /usr/lib/x86_64-linux-gnu/libjpeg.so
../bin/run_vo: /usr/lib/x86_64-linux-gnu/libpng.so
../bin/run_vo: /usr/lib/x86_64-linux-gnu/libtiff.so
../bin/run_vo: /usr/lib/x86_64-linux-gnu/libgl2ps.so
../bin/run_vo: /usr/lib/x86_64-linux-gnu/libjsoncpp.so
../bin/run_vo: /usr/lib/x86_64-linux-gnu/libpcl_io.so
../bin/run_vo: /usr/lib/x86_64-linux-gnu/libflann_cpp_s.a
../bin/run_vo: /usr/lib/x86_64-linux-gnu/libpcl_kdtree.so
../bin/run_vo: /usr/lib/x86_64-linux-gnu/libpcl_search.so
../bin/run_vo: /usr/lib/x86_64-linux-gnu/libpcl_sample_consensus.so
../bin/run_vo: /usr/lib/x86_64-linux-gnu/libpcl_filters.so
../bin/run_vo: /usr/lib/x86_64-linux-gnu/libpcl_features.so
../bin/run_vo: /usr/lib/x86_64-linux-gnu/libpcl_ml.so
../bin/run_vo: /usr/lib/x86_64-linux-gnu/libpcl_segmentation.so
../bin/run_vo: /usr/lib/x86_64-linux-gnu/libpcl_visualization.so
../bin/run_vo: /usr/lib/x86_64-linux-gnu/libqhull.so
../bin/run_vo: /usr/lib/x86_64-linux-gnu/libpcl_surface.so
../bin/run_vo: /usr/lib/x86_64-linux-gnu/libpcl_registration.so
../bin/run_vo: /usr/lib/x86_64-linux-gnu/libpcl_keypoints.so
../bin/run_vo: /usr/lib/x86_64-linux-gnu/libpcl_tracking.so
../bin/run_vo: /usr/lib/x86_64-linux-gnu/libpcl_recognition.so
../bin/run_vo: /usr/lib/x86_64-linux-gnu/libpcl_stereo.so
../bin/run_vo: /usr/lib/x86_64-linux-gnu/libpcl_apps.so
../bin/run_vo: /usr/lib/x86_64-linux-gnu/libpcl_outofcore.so
../bin/run_vo: /usr/lib/x86_64-linux-gnu/libpcl_people.so
../bin/run_vo: /usr/lib/x86_64-linux-gnu/libboost_system.so
../bin/run_vo: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
../bin/run_vo: /usr/lib/x86_64-linux-gnu/libboost_thread.so
../bin/run_vo: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
../bin/run_vo: /usr/lib/x86_64-linux-gnu/libboost_iostreams.so
../bin/run_vo: /usr/lib/x86_64-linux-gnu/libboost_serialization.so
../bin/run_vo: /usr/lib/x86_64-linux-gnu/libboost_chrono.so
../bin/run_vo: /usr/lib/x86_64-linux-gnu/libboost_atomic.so
../bin/run_vo: /usr/lib/x86_64-linux-gnu/libboost_regex.so
../bin/run_vo: /usr/lib/x86_64-linux-gnu/libpcl_common.so
../bin/run_vo: /usr/lib/x86_64-linux-gnu/libpcl_octree.so
../bin/run_vo: /usr/lib/libOpenNI.so
../bin/run_vo: /usr/lib/libOpenNI2.so
../bin/run_vo: /usr/lib/x86_64-linux-gnu/libexpat.so
../bin/run_vo: /usr/lib/x86_64-linux-gnu/libjpeg.so
../bin/run_vo: /usr/lib/x86_64-linux-gnu/libpng.so
../bin/run_vo: /usr/lib/x86_64-linux-gnu/libtiff.so
../bin/run_vo: /usr/lib/x86_64-linux-gnu/libgl2ps.so
../bin/run_vo: /usr/lib/x86_64-linux-gnu/libjsoncpp.so
../bin/run_vo: /usr/lib/x86_64-linux-gnu/libpcl_io.so
../bin/run_vo: /usr/lib/x86_64-linux-gnu/libflann_cpp_s.a
../bin/run_vo: /usr/lib/x86_64-linux-gnu/libpcl_kdtree.so
../bin/run_vo: /usr/lib/x86_64-linux-gnu/libpcl_search.so
../bin/run_vo: /usr/lib/x86_64-linux-gnu/libpcl_sample_consensus.so
../bin/run_vo: /usr/lib/x86_64-linux-gnu/libpcl_filters.so
../bin/run_vo: /usr/lib/x86_64-linux-gnu/libpcl_features.so
../bin/run_vo: /usr/lib/x86_64-linux-gnu/libpcl_ml.so
../bin/run_vo: /usr/lib/x86_64-linux-gnu/libpcl_segmentation.so
../bin/run_vo: /usr/lib/x86_64-linux-gnu/libpcl_visualization.so
../bin/run_vo: /usr/lib/x86_64-linux-gnu/libqhull.so
../bin/run_vo: /usr/lib/x86_64-linux-gnu/libpcl_surface.so
../bin/run_vo: /usr/lib/x86_64-linux-gnu/libpcl_registration.so
../bin/run_vo: /usr/lib/x86_64-linux-gnu/libpcl_keypoints.so
../bin/run_vo: /usr/lib/x86_64-linux-gnu/libpcl_tracking.so
../bin/run_vo: /usr/lib/x86_64-linux-gnu/libpcl_recognition.so
../bin/run_vo: /usr/lib/x86_64-linux-gnu/libpcl_stereo.so
../bin/run_vo: /usr/lib/x86_64-linux-gnu/libpcl_apps.so
../bin/run_vo: /usr/lib/x86_64-linux-gnu/libpcl_outofcore.so
../bin/run_vo: /usr/lib/x86_64-linux-gnu/libpcl_people.so
../bin/run_vo: /usr/lib/x86_64-linux-gnu/libfreetype.so
../bin/run_vo: /usr/lib/x86_64-linux-gnu/libpython2.7.so
../bin/run_vo: /usr/lib/x86_64-linux-gnu/libproj.so
../bin/run_vo: /usr/lib/x86_64-linux-gnu/libnetcdf_c++.so
../bin/run_vo: /usr/lib/x86_64-linux-gnu/libnetcdf.so
../bin/run_vo: /usr/lib/x86_64-linux-gnu/libtheoraenc.so
../bin/run_vo: /usr/lib/x86_64-linux-gnu/libtheoradec.so
../bin/run_vo: /usr/lib/x86_64-linux-gnu/libogg.so
../bin/run_vo: /usr/lib/x86_64-linux-gnu/libxml2.so
../bin/run_vo: /usr/lib/x86_64-linux-gnu/hdf5/openmpi/libhdf5.so
../bin/run_vo: /usr/lib/x86_64-linux-gnu/libsz.so
../bin/run_vo: /usr/lib/x86_64-linux-gnu/libz.so
../bin/run_vo: /usr/lib/x86_64-linux-gnu/libdl.so
../bin/run_vo: /usr/lib/x86_64-linux-gnu/libm.so
../bin/run_vo: /usr/lib/x86_64-linux-gnu/openmpi/lib/libmpi.so
../bin/run_vo: /usr/local/lib/libglog.so.0.6.0
../bin/run_vo: test/CMakeFiles/run_vo.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/guojiawei/cv/cv2/cv/after/myslam/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Linking CXX executable ../../bin/run_vo"
	cd /home/guojiawei/cv/cv2/cv/after/myslam/build/test && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/run_vo.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
test/CMakeFiles/run_vo.dir/build: ../bin/run_vo
.PHONY : test/CMakeFiles/run_vo.dir/build

test/CMakeFiles/run_vo.dir/clean:
	cd /home/guojiawei/cv/cv2/cv/after/myslam/build/test && $(CMAKE_COMMAND) -P CMakeFiles/run_vo.dir/cmake_clean.cmake
.PHONY : test/CMakeFiles/run_vo.dir/clean

test/CMakeFiles/run_vo.dir/depend:
	cd /home/guojiawei/cv/cv2/cv/after/myslam/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/guojiawei/cv/cv2/cv/after/myslam /home/guojiawei/cv/cv2/cv/after/myslam/test /home/guojiawei/cv/cv2/cv/after/myslam/build /home/guojiawei/cv/cv2/cv/after/myslam/build/test /home/guojiawei/cv/cv2/cv/after/myslam/build/test/CMakeFiles/run_vo.dir/DependInfo.cmake --color=$(COLOR)
>>>>>>> fbc9515c775cd61c2cb5a913f937852e2c828cb9
.PHONY : test/CMakeFiles/run_vo.dir/depend

