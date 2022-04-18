<<<<<<< HEAD
# Install script for directory: /home/xzy/CLionProjects/myslam
=======
# Install script for directory: /home/guojiawei/cv/cv2/cv/after/myslam
>>>>>>> fbc9515c775cd61c2cb5a913f937852e2c828cb9

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/usr/local")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "Release")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

<<<<<<< HEAD
if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/home/xzy/CLionProjects/myslam/build/src/cmake_install.cmake")
=======
# Set default install directory permissions.
if(NOT DEFINED CMAKE_OBJDUMP)
  set(CMAKE_OBJDUMP "/usr/bin/objdump")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/home/guojiawei/cv/cv2/cv/after/myslam/build/src/cmake_install.cmake")
>>>>>>> fbc9515c775cd61c2cb5a913f937852e2c828cb9
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
<<<<<<< HEAD
  include("/home/xzy/CLionProjects/myslam/build/test/cmake_install.cmake")
=======
  include("/home/guojiawei/cv/cv2/cv/after/myslam/build/test/cmake_install.cmake")
>>>>>>> fbc9515c775cd61c2cb5a913f937852e2c828cb9
endif()

if(CMAKE_INSTALL_COMPONENT)
  set(CMAKE_INSTALL_MANIFEST "install_manifest_${CMAKE_INSTALL_COMPONENT}.txt")
else()
  set(CMAKE_INSTALL_MANIFEST "install_manifest.txt")
endif()

string(REPLACE ";" "\n" CMAKE_INSTALL_MANIFEST_CONTENT
       "${CMAKE_INSTALL_MANIFEST_FILES}")
<<<<<<< HEAD
file(WRITE "/home/xzy/CLionProjects/myslam/build/${CMAKE_INSTALL_MANIFEST}"
=======
file(WRITE "/home/guojiawei/cv/cv2/cv/after/myslam/build/${CMAKE_INSTALL_MANIFEST}"
>>>>>>> fbc9515c775cd61c2cb5a913f937852e2c828cb9
     "${CMAKE_INSTALL_MANIFEST_CONTENT}")
