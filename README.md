# Simple Gazebo Mass PID Control

This repository include an example of using ROS and Gazebo to position control a simple unit mass, using C++.
A simple PID controller (not the internal ROS controller) is used as the controller.

## Initial Setup 
```sh
git clone https://github.com/fdcl-gwu/gazebo_mass_pid_control.git
cd simple_mass_pid_control
catkin_make
cd devel && source setup.bash && cd ../
```

## Running the code
```sh
roslaunch simple_mass_gazebo simple_world.launch 
rosrun simple_mass_control mass_control
```

## Explanation
This code is organized as follows:
* `src/simple_mass_control`: C++ code for controlling the mass
* `src/simple_mass_gazebo`: Gazebo simulation related files

If you just want to run the simulation, running
```sh
roslaunch simple_mass_gazebo simple_world.launch 
```
should do the trick.
This uses `simple_mass_control/src/mass_plugin.cpp` file which uses the Gazebo model plugin as described [here](http://gazebosim.org/tutorials?tut=ros_gzplugins).
Determining the current state and control input is done inside this file.

The other C++ files are there just to show how to communicate with the publisher and user ROS service calls to get model states.
You can run them with
```sh
rosrun simple_mass_control mass_control
```
which reads the data published by the plugging file, and prints them in the terminal.
You can get similar results with
```
rostopic echo /states
```
after launching Gazebo simulation.
