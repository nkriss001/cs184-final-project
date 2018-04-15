# Install script for directory: /home/cs184/cs184/projects/cs184-final-project/CGL/src

# Set the install prefix
IF(NOT DEFINED CMAKE_INSTALL_PREFIX)
  SET(CMAKE_INSTALL_PREFIX "/usr/local")
ENDIF(NOT DEFINED CMAKE_INSTALL_PREFIX)
STRING(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
IF(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  IF(BUILD_TYPE)
    STRING(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  ELSE(BUILD_TYPE)
    SET(CMAKE_INSTALL_CONFIG_NAME "")
  ENDIF(BUILD_TYPE)
  MESSAGE(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
ENDIF(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)

# Set the component getting installed.
IF(NOT CMAKE_INSTALL_COMPONENT)
  IF(COMPONENT)
    MESSAGE(STATUS "Install component: \"${COMPONENT}\"")
    SET(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  ELSE(COMPONENT)
    SET(CMAKE_INSTALL_COMPONENT)
  ENDIF(COMPONENT)
ENDIF(NOT CMAKE_INSTALL_COMPONENT)

# Install shared libraries without execute permission?
IF(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  SET(CMAKE_INSTALL_SO_NO_EXE "1")
ENDIF(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE STATIC_LIBRARY FILES "/home/cs184/cs184/projects/cs184-final-project/build/CGL/src/libCGL.a")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/CGL" TYPE FILE FILES
    "/home/cs184/cs184/projects/cs184-final-project/CGL/src/CGL.h"
    "/home/cs184/cs184/projects/cs184-final-project/CGL/src/vector2D.h"
    "/home/cs184/cs184/projects/cs184-final-project/CGL/src/vector3D.h"
    "/home/cs184/cs184/projects/cs184-final-project/CGL/src/vector4D.h"
    "/home/cs184/cs184/projects/cs184-final-project/CGL/src/matrix3x3.h"
    "/home/cs184/cs184/projects/cs184-final-project/CGL/src/matrix4x4.h"
    "/home/cs184/cs184/projects/cs184-final-project/CGL/src/quaternion.h"
    "/home/cs184/cs184/projects/cs184-final-project/CGL/src/complex.h"
    "/home/cs184/cs184/projects/cs184-final-project/CGL/src/color.h"
    "/home/cs184/cs184/projects/cs184-final-project/CGL/src/osdtext.h"
    "/home/cs184/cs184/projects/cs184-final-project/CGL/src/viewer.h"
    "/home/cs184/cs184/projects/cs184-final-project/CGL/src/base64.h"
    "/home/cs184/cs184/projects/cs184-final-project/CGL/src/tinyxml2.h"
    "/home/cs184/cs184/projects/cs184-final-project/CGL/src/renderer.h"
    )
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

