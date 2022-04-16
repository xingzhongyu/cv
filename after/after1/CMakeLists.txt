SET(TARGETFILENAME  "object_node2")
PROJECT(  ${TARGETFILENAME} )

cmake_minimum_required(VERSION 3.16)
set(CMAKE_CXX_STANDARD 14)

if(NOT DEFINED CMAKE_SUPPRESS_DEVELOPER_WARNINGS)
    set(CMAKE_SUPPRESS_DEVELOPER_WARNINGS 1 CACHE INTERNAL "No dev warnings")
endif()
find_package( OpenCV REQUIRED )
find_package(g2o REQUIRED)

# SET(NODE_LIBRARY_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/thirdPart/node)
SET(src src/args_callback.cc src/map.cc src/map.h src/map2.cc)


include_directories(${CMAKE_JS_INC})
include_directories("/home/xzy/object_node2/thirdPart/node"  "/home/xzy/object_node2/thirdPart/node-addon-api "${OpenCV_INCLUDE_DIRS})
# LINK_DIRECTORIES(${NODE_LIBRARY_DIRS})
ADD_LIBRARY ( ${TARGETFILENAME} SHARED 
    
    ${src}
)
find_package( Sophus REQUIRED )
include_directories( ${Sophus_INCLUDE_DIRS} )



find_package(PCL 1.10 REQUIRED)
include_directories( ${PCL_INCLUDE_DIRS} )
link_directories(${PCL_LIBRARY_DIRS})

add_definitions(${PCL_DEFINITIONS})



SET_TARGET_PROPERTIES(${TARGETFILENAME} PROPERTIES SUFFIX ".node")
# add_executable(${src} ${src})
target_link_libraries(${TARGETFILENAME} ${OpenCV_LIBS} ${Sophus_LIBRARIES}
${PCL_LIBRARIES} ${CMAKE_JS_LIB})


