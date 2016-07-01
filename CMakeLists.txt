#-----------------------------------------------------------------------------
#
#  CMake Config
#
#  Velodyne Tracking
#
#-----------------------------------------------------------------------------

cmake_minimum_required(VERSION 2.8.3 FATAL_ERROR)
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

#-----------------------------------------------------------------------------
#
#  Project version
#
#-----------------------------------------------------------------------------
project(velodyne_tracking)

set(VT_VERSION_MAJOR 1)
set(VT_VERSION_MINOR 2)
set(VT_VERSION_PATCH 1)

set(VT_VERSION
    ${VT_VERSION_MAJOR}.${XX_VERSION_MINOR}.${XX_VERSION_PATCH})

#-----------------------------------------------------------------------------
#
#  Find catkin macros and libraries
#
#-----------------------------------------------------------------------------
find_package(catkin REQUIRED COMPONENTS
  pcl_ros
  roscpp
  std_msgs
  visualization_msgs
  geometry_msgs
  tf2_kdl
  image_transport
  cv_bridge
)

catkin_package(
  DEPENDS
  CATKIN_DEPENDS roscpp
  INCLUDE_DIRS
  LIBRARIES
)
#---------------------------------------------

include_directories(
  ${catkin_INCLUDE_DIRS}
  include
  datasets
)

#-----------------------------------------------------------------------------
#
#  Find external dependencies
#
#-----------------------------------------------------------------------------

#---------------------------------------------
# Point Cloud Library (PCL)
#---------------------------------------------
find_package(PCL 1.7 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

#---------------------------------------------
# Osmium library
#---------------------------------------------
find_package(Osmium REQUIRED COMPONENTS io geos)
include_directories(SYSTEM ${OSMIUM_INCLUDE_DIRS})
#add_definitions(${OSMIUM_WARNING_OPTIONS})

#---------------------------------------------
# PLplot
#---------------------------------------------
find_package(PLplot REQUIRED COMPONENTS)
include_directories(SYSTEM ${PLplot_INCLUDE_DIRS})

#---------------------------------------------
# dlib library
#---------------------------------------------
# Add the dlib library for svm classification

# For static build :
# 1. Download dlib library and put it into include folder 
# 2. Uncomment : 
# include(include/dlib-18.18/dlib/cmake)

find_package(dlib 18.18 REQUIRED)
include_directories(SYSTEM ${dlib_INCLUDE_DIRS})

#---------------------------------------------
# OpenCV
#---------------------------------------------
find_package( OpenCV REQUIRED )

#---------------------------------------------
# KFilter library location
#---------------------------------------------
link_directories(/usr/local/lib)

#-----------------------------------------------------------------------------
#
#  Decide which C++ version to use (Minimum/default: C++11).
#
#-----------------------------------------------------------------------------
if(NOT MSVC)
    if(NOT USE_CPP_VERSION)
        set(USE_CPP_VERSION c++11)
    endif()
    message(STATUS "Use C++ version: ${USE_CPP_VERSION}")
    # following only available from cmake 2.8.12:
    #   add_compile_options(-std=${USE_CPP_VERSION})
    # so using this instead:
    add_definitions(-std=${USE_CPP_VERSION})
endif()

#-----------------------------------------------------------------------------
#
#  Compiler and Linker flags
#
#-----------------------------------------------------------------------------
set(USUAL_COMPILE_OPTIONS "-O3 -g")

set(CMAKE_CXX_FLAGS_DEV "${USUAL_COMPILE_OPTIONS}"
    CACHE STRING "Flags used by the compiler during developer builds."
    FORCE)

set(CMAKE_EXE_LINKER_FLAGS_DEV ""
    CACHE STRING "Flags used by the linker during developer builds."
    FORCE)
mark_as_advanced(
    CMAKE_CXX_FLAGS_DEV
    CMAKE_EXE_LINKER_FLAGS_DEV
)

set(CMAKE_CXX_FLAGS_RELWITHDEBINFO "${USUAL_COMPILE_OPTIONS}"
    CACHE STRING "Flags used by the compiler during RELWITHDEBINFO builds."
    FORCE)

#-----------------------------------------------------------------------------
#
#  Build Type
#
#-----------------------------------------------------------------------------
set(CMAKE_CONFIGURATION_TYPES "Debug Release RelWithDebInfo MinSizeRel Dev")

# In 'Dev' mode: compile with very strict warnings and turn them into errors.
if(CMAKE_BUILD_TYPE STREQUAL "Dev")
    if(NOT MSVC)
        add_definitions(-Werror -fno-omit-frame-pointer)
    endif()
    add_definitions(${OSMIUM_WARNING_OPTIONS})
endif()

# Force RelWithDebInfo build type if none was given
if(CMAKE_BUILD_TYPE)
    set(build_type ${CMAKE_BUILD_TYPE})
else()
    set(build_type "RelWithDebInfo")
endif()

set(CMAKE_BUILD_TYPE ${build_type}
    CACHE STRING
    "Choose the type of build, options are: ${CMAKE_CONFIGURATION_TYPES}."
    FORCE)

#-----------------------------------------------------------------------------
#
#  Internal dependencies
#
#-----------------------------------------------------------------------------

#---------------------------------------------
# objectEKF
#---------------------------------------------
add_library(objectEKF include/objectEKF/objectEKF.cpp)
target_link_libraries(objectEKF kalman)

#---------------------------------------------
# object
#---------------------------------------------
add_library(RGBconverter include/RGBConverter/RGBConverter.cpp)
add_library(object include/objectTracking/object.cpp)
target_link_libraries(object objectEKF RGBconverter ${catkin_LIBRARIES} ${PCL_LIBRARIES})

#---------------------------------------------
# label
#---------------------------------------------
add_library(label include/label/label.cpp)
target_link_libraries(label ${dlib_LIBRARIES})

#---------------------------------------------
# osm
#---------------------------------------------
add_library(osm include/osm/osm.cpp)
target_link_libraries(osm ${OSMIUM_LIBRARIES})

#---------------------------------------------
# objectTracking
#---------------------------------------------
add_library(oxts include/oxts/oxts.cpp)
add_library(mp include/mercatorProjection/mercatorProjection.cpp)
add_library(objectTracking include/objectTracking/objectTracking.cpp)
target_link_libraries(objectTracking object oxts mp osm label)

#---------------------------------------------
# velocityProfile
#---------------------------------------------
add_library(lowess include/Lowess/lowess.cpp)
add_library(velocityProfile include/velocityProfile/velocityProfile.cpp)
target_link_libraries(velocityProfile lowess)

#-----------------------------------------------------------------------------
#
#  Executable
#
#-----------------------------------------------------------------------------

#---------------------------------------------
# kitti2don.launch
#---------------------------------------------
add_executable(kitti2pcl2don src/kitti2pcl2don.cpp)
target_link_libraries(kitti2pcl2don ${Boost_LIBRARIES} objectTracking)

#---------------------------------------------
# velodyne_tracking.launch
#---------------------------------------------
add_executable(track src/track.cpp)
target_link_libraries(track ${Boost_LIBRARIES} objectTracking ${OpenCV_LIBS})

#---------------------------------------------
# kitti2dlib.launch
#---------------------------------------------
add_executable(kitti2dlib src/kitti2dlib.cpp)
target_link_libraries(kitti2dlib ${Boost_LIBRARIES} objectTracking)

#---------------------------------------------
# Driver Behavior
#---------------------------------------------
add_executable(driverBehavior src/driverBehavior.cpp)
target_link_libraries(driverBehavior ${PLplot_LIBRARIES} osm mp ${Boost_LIBRARIES} velocityProfile object)

#-----------------------------------------------------------------------------