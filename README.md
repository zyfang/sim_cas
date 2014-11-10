# hand_sim client

# prerequisites
 * gazebo
 * libconfig

# build
~~~
mkdir build
cd build
cmake ..
make
~~~

# set up plugin path
~~~
echo "export GAZEBO_PLUGIN_PATH=/<path>/hand_sim_client/build:${GAZEBO_PLUGIN_PATH}" >> ~/.bashrc
source ~/.bashrc
~~~

# run example
~~~
gazebo worlds/kitchen.world -u --verbose
~~~

# hydra buttons:
 * `middle button` -> starts/stops hand tracking
 * `bumper button` -> creates pancake
 * `button 2` -> starts/stops logging the data