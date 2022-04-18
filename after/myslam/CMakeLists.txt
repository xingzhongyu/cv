cmake_minimum_required(VERSION 3.16)
project(myslam)

<<<<<<< HEAD
set(CMAKE_CXX_STANDARD 14)
=======
set( CMAKE_BUILD_TYPE Release )
set( CMAKE_CXX_FLAGS "-std=c++14"  )








>>>>>>> fbc9515c775cd61c2cb5a913f937852e2c828cb9

list( APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules )
set( EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin )
set( LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib )

############### dependencies ######################
# Eigen
include_directories( "/usr/include/eigen3" )
# OpenCV
find_package( OpenCV REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )
# Sophus
find_package( Sophus REQUIRED )
include_directories( ${Sophus_INCLUDE_DIRS} )
# G2O

find_package( g2o REQUIRED )
include_directories( ${G2O_INCLUDE_DIRS} )

find_package (glog REQUIRED)

#PCL
<<<<<<< HEAD
find_package(PCL 1.10 REQUIRED)
=======
find_package(PCL  REQUIRED)


>>>>>>> fbc9515c775cd61c2cb5a913f937852e2c828cb9
include_directories( ${PCL_INCLUDE_DIRS} )
link_directories(${PCL_LIBRARY_DIRS})

add_definitions(${PCL_DEFINITIONS})

<<<<<<< HEAD
=======
set(Sophus_LIBRARIES  "/usr/local/lib/libSophus.so")





>>>>>>> fbc9515c775cd61c2cb5a913f937852e2c828cb9

set( THIRD_PARTY_LIBS
        ${OpenCV_LIBS}
        ${Sophus_LIBRARIES}
        ${PCL_LIBRARIES}
        glog::glog
        g2o_core g2o_stuff g2o_types_sba
        )
############### source and test ######################
include_directories( ${PROJECT_SOURCE_DIR}/include )

add_subdirectory( src )


add_subdirectory( test )
