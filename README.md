## markertracker
tracks a marker model consisting of 4 UV-leds 

## dependencies 
Eigen, glew, SFML, image magick, assimp, opengl, OpenCV, sdformat, pcl, boost, ros
### on Ubuntu 14.04
```
#!bash
sudo apt-get install libeigen3-dev libglew-dev libsfml-dev libmagick++-dev libassimp-dev libglm-dev libopencv-dev libpcl-1.7-all-dev ros-jade-desktop-full
```
## checkout 
```
#!bash
git clone https://github.com/Roboy/markertracker
cd path/to/markertracker
git submodule init
git submoulde update
```
## build
```
#!bash
cd path/to/markertracker
mkdir build
cd build
cmake ..
make 
```
## run
```
#!bash
cd path/to/markertracker/build/devel/lib/markertracker
./markertracker
```
