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
# Eigen
find_package( PkgConfig )
pkg_check_modules( EIGEN3 REQUIRED eigen3 )
include_directories( ${EIGEN3_INCLUDE_DIRS} )

# SFML
pkg_check_modules( SFML REQUIRED sfml-graphics )

# glew
find_package(GLEW REQUIRED)

# image magick
find_package( ImageMagick COMPONENTS Magick++ MagickWand MagickCore REQUIRED)
include_directories(${ImageMagick_Magick++_INCLUDE_DIR})

# assimp
find_package(assimp REQUIRED)

# opengl
find_package(OpenGL REQUIRED)

# OpenCV
find_package(OpenCV 3.1.0 COMPONENTS opencv_core opencv_highgui opencv_imgproc opencv_imgcodecs opencv_calib3d opencv_videoio REQUIRED)

# sdformat
find_package(SDFormat REQUIRED)
include_directories(${SDFormat_INCLUDE_DIRS})

# pcl
find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
add_definitions(${PCL_DEFINITIONS})

# boost
find_package(Boost COMPONENTS filesystem REQUIRED)

include_directories(include omgl/include)
#==============================================================================
# Application
#==============================================================================

ADD_EXECUTABLE(markertracker src/main.cpp src/markertracker.cpp omgl/src/mesh.cpp omgl/src/model.cpp omgl/src/renderer.cpp)
TARGET_LINK_LIBRARIES(markertracker ${SFML_LIBRARIES} ${GLEW_LIBRARIES} ${ASSIMP_LIBRARIES} ${OPENGL_LIBRARIES}
        ${ImageMagick_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS} ${SDFormat_LIBRARIES} ${BOOST_LIBRARIES})

