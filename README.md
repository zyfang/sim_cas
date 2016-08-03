# Interactive Kitchen Games in Gazebo
Mirror of https://bitbucket.org/zyfang/sim_cas

## Prerequisites
Install Gazebo 5.3.0:
 * http://gazebosim.org/tutorials?tut=install_ubuntu&cat=install
  
Install libconfig library for reading config files  
 * libconfig

Razer Hydra device for interaction

## Build
~~~
mkdir build
cd build
cmake ..
make
~~~

## Set up plugin path
~~~
echo "export GAZEBO_PLUGIN_PATH=/<path>/kitchen_games/build:${GAZEBO_PLUGIN_PATH}" >> ~/.bashrc
source ~/.bashrc
~~~

## Run example
~~~
gazebo worlds/kitchen.world -u --verbose
~~~

## Razer Hydra device (right) buttons :
 * `middle button` -> starts/stops hand tracking
 * `bumper button` -> creates pancake
 * `button 2` -> starts/stops logging the data