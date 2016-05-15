CMAKE_MINIMUM_REQUIRED(VERSION 2.6)

PROJECT(markertracker C CXX)

set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin)

# RelWithDebInfo, Release or Debug
SET(CMAKE_BUILD_TYPE Debug FORCE)
SET(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -g -std=c++11" )

SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR}/cmake)

#==============================================================================
# Dependencies
#==============================================================================

# Eigen
find_package( PkgConfig )
pkg_check_modules( EIGEN3 REQUIRED eigen3 )
include_directories( ${EIGEN3_INCLUDE_DIRS} )

# OpenCV
SET(CUDA_TOOLKIT_ROOT_DIR /usr/local/cuda-7.5)
FIND_PACKAGE(CUDA REQUIRED)
FIND_PACKAGE(OpenCV 3.1.0 COMPONENTS opencv_core opencv_highgui opencv_imgproc opencv_imgcodecs opencv_videoio opencv_calib3d REQUIRED)

# GLEW
find_package(GLEW REQUIRED)

# opengl
find_package(OpenGL REQUIRED)

# SFML
pkg_check_modules( SFML REQUIRED sfml-graphics )

# assimp
find_package(assimp REQUIRED)

# image magick
find_package( ImageMagick COMPONENTS Magick++ MagickWand MagickCore REQUIRED)
include_directories(${ImageMagick_Magick++_INCLUDE_DIR})

include_directories(include openglconvenience/include)
#==============================================================================
# Application
#==============================================================================

ADD_EXECUTABLE(markertracker src/main.cpp src/markertracker.cpp src/mesh.cpp src/renderer.cpp)
TARGET_LINK_LIBRARIES(markertracker ${SFML_LIBRARIES} ${GLEW_LIBRARIES} ${ASSIMP_LIBRARIES} ${OPENGL_LIBRARIES} ${OpenCV_LIBS} ${ImageMagick_LIBRARIES} )

