CMAKE_MINIMUM_REQUIRED(VERSION 2.6)

PROJECT(markertracker C CXX)

set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin)

# RelWithDebInfo, Release or Debug
SET(CMAKE_BUILD_TYPE Debug FORCE)
SET(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -g -std=c++11  -DMAGICKCORE_QUANTUM_DEPTH=16 -DMAGICKCORE_HDRI_ENABLE=0" )

SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR}/cmake)

#==============================================================================
# Dependencies
#==============================================================================
# ros
find_package(catkin REQUIRED COMPONENTS roscpp controller_manager std_msgs)
catkin_package(CATKIN_DEPENDS message_runtime)

# Eigen
find_package( PkgConfig )
pkg_check_modules( EIGEN3 REQUIRED eigen3 )
include_directories( ${EIGEN3_INCLUDE_DIRS} )

# SFML
pkg_check_modules( SFML REQUIRED sfml-graphics )

# image magick
find_package( ImageMagick COMPONENTS Magick++ MagickWand MagickCore REQUIRED)
include_directories(${ImageMagick_Magick++_INCLUDE_DIR})

# assimp
pkg_check_modules(assimp REQUIRED assimp)
include_directories( ${assimp_INCLUDE_DIRS} )

# glew
pkg_check_modules(glew REQUIRED glew)
include_directories( ${glew_INCLUDE_DIRS} )

# opengl
find_package(OpenGL REQUIRED)

# OpenCV
find_package(OpenCV 3.1.0 COMPONENTS opencv_core opencv_highgui opencv_imgproc opencv_calib3d REQUIRED)

# sdformat
find_package(SDFormat 4 REQUIRED)
include_directories(${SDFormat_INCLUDE_DIRS})

# pcl
find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
add_definitions(${PCL_DEFINITIONS})

# boost
find_package(Boost COMPONENTS filesystem REQUIRED)

# ros
pkg_check_modules( roscpp REQUIRED roscpp)
pkg_check_modules( visualization_msgs REQUIRED visualization_msgs)

include_directories(include omgl/include)
#==============================================================================
# Application
#==============================================================================

ADD_EXECUTABLE(markertracker src/main.cpp src/markertracker.cpp omgl/src/mesh.cpp omgl/src/model.cpp omgl/src/renderer.cpp)
TARGET_LINK_LIBRARIES(markertracker ${SFML_LIBRARIES} ${glew_LIBRARIES} ${assimp_LIBRARIES} ${OPENGL_LIBRARIES}
        ${ImageMagick_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS} ${SDFormat_LIBRARIES} ${BOOST_LIBRARIES} -lpthread
        ${catkin_LIBRARIES} )

