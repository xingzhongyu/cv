cmake_minimum_required( VERSION 3.10 )

set( CMAKE_BUILD_TYPE Release )
set( CMAKE_CXX_FLAGS "-std=c++14" )

# opencv
find_package( OpenCV REQUIRED )

include_directories( ${OpenCV_INCLUDE_DIRS} )

# eigen
include_directories( "/usr/include/eigen3/" )

# pcl
find_package(PCL REQUIRED)

include_directories( ${PCL_INCLUDE_DIRS} )
link_directories(${PCL_LIBRARY_DIRS})
add_definitions( ${PCL_DEFINITIONS} )
list( APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules )

# octomap
find_package( octomap REQUIRED )
include_directories( ${OCTOMAP_INCLUDE_DIRS} )

LIST( APPEND CMAKE_MODULE_PATH  ~/下载/g2o/cmake_modules )
SET( G2O_ROOT  ~/下载/g2o )

find_package( G2O  REQUIRED)
include_directories(${G2O_INCLUDE_DIRS})
find_package( CSparse REQUIRED )

add_executable(test test.cpp )
target_link_libraries(test ${OpenCV_LIBS})


add_executable( pointcloud_mapping pointcloud_mapping.cpp )
target_link_libraries( pointcloud_mapping ${OpenCV_LIBS} ${PCL_LIBRARIES}  ${OCTOMAP_LIBRARIES}  g2o_core g2o_stuff g2o_types_sba g2o_csparse_extension 
${CSPARSE_LIBRARY}

)




# add_executable( octomap_mapping octomap_mapping.cpp)
# target_link_libraries( octomap_mapping ${OpenCV_LIBS} ${PCL_LIBRARIES} ${OCTOMAP_LIBRARIES} )

# add_executable(show show.cpp)
# target_link_libraries(show ${PCL_LIBRARIES})
