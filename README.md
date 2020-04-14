# Simple Mass PID Control

## Initial Setup 
```sh
git clone ...
catkin_make
cd devel && source setup.bash && cd ../
```

## Running the code
```sh
roslaunch simple_mass_gazebo simple_world.launch 
rosrun simple_mass_control mass_control
```
