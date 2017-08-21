#!/usr/bin/env bash
set -e
set -x

# Install some packages that were missed in v1.1. Not necessary anymore in v1.2
sudo apt-get install ros-kinetic-{tf-conversions,cv-bridge,image-transport,camera-info-manager,theora-image-transport,joy,image-proc} -y
sudo apt-get install ros-kinetic-compressed-image-transport -y
sudo apt-get install libyaml-cpp-dev -y

# # packages for the IMU
sudo apt-get install ros-kinetic-phidgets-drivers
sudo apt-get install ros-kinetic-imu-complementary-filter ros-kinetic-imu-filter-madgwick
sudo apt-get install ros-kinetic-visualization-msgs 
sudo apt-get install ros-kinetic-nav-msgs
sudo apt-get install ros-kinetic-cmake-modules

# # scipy for lane-filter
# sudo apt-get install libblas-dev liblapack-dev libatlas-base-dev gfortran
# sudo pip install scipy --upgrade
