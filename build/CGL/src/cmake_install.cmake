# Install script for directory: /Users/nicholaskriss/Desktop/cs184/cs184-final-project/CGL/src

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
    set(CMAKE_INSTALL_CONFIG_NAME "")
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

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE STATIC_LIBRARY FILES "/Users/nicholaskriss/Desktop/cs184/cs184-final-project/build/CGL/src/libCGL.a")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libCGL.a" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libCGL.a")
    execute_process(COMMAND "/Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/ranlib" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libCGL.a")
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/CGL" TYPE FILE FILES
    "/Users/nicholaskriss/Desktop/cs184/cs184-final-project/CGL/src/CGL.h"
    "/Users/nicholaskriss/Desktop/cs184/cs184-final-project/CGL/src/vector2D.h"
    "/Users/nicholaskriss/Desktop/cs184/cs184-final-project/CGL/src/vector3D.h"
    "/Users/nicholaskriss/Desktop/cs184/cs184-final-project/CGL/src/vector4D.h"
    "/Users/nicholaskriss/Desktop/cs184/cs184-final-project/CGL/src/matrix3x3.h"
    "/Users/nicholaskriss/Desktop/cs184/cs184-final-project/CGL/src/matrix4x4.h"
    "/Users/nicholaskriss/Desktop/cs184/cs184-final-project/CGL/src/quaternion.h"
    "/Users/nicholaskriss/Desktop/cs184/cs184-final-project/CGL/src/complex.h"
    "/Users/nicholaskriss/Desktop/cs184/cs184-final-project/CGL/src/color.h"
    "/Users/nicholaskriss/Desktop/cs184/cs184-final-project/CGL/src/osdtext.h"
    "/Users/nicholaskriss/Desktop/cs184/cs184-final-project/CGL/src/viewer.h"
    "/Users/nicholaskriss/Desktop/cs184/cs184-final-project/CGL/src/base64.h"
    "/Users/nicholaskriss/Desktop/cs184/cs184-final-project/CGL/src/tinyxml2.h"
    "/Users/nicholaskriss/Desktop/cs184/cs184-final-project/CGL/src/renderer.h"
    )
endif()

