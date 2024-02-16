#!/bin/bash
mkdir build
cd build
cmake ../
make
export GAZEBO_PLUGIN_PATH=`pwd`:$GAZEBO_PLUGIN_PATH
echo $GAZEBO_PLUGIN_PATH
