# Racecar Simulator

This is a lightweight 2D simulator of the MIT Racecar.
It can be built with ROS, or it can be used as a standalone C++ library.

## ROS

### Installation

Building this package with ROS requires the following packages:

- tf2
- tf2_ros
- tf2_geometry_msgs
- ackermann_msgs
- nav_msgs
- sensor_msgs
- geometry_msgs
- joy
- map_server

These are installed by default with the desktop version of ROS.
To install the simulator package, clone it into your catkin workspace:

    cd ~/catkin_ws/src
    git clone racecar_simulator

Then run ```catkin_make``` to build it:

    cd ~/catkin_ws
    catkin_make
    source devel/setup.bash

## Usage

To run the simulator on its own, run:

    roslaunch racecar_simulator simulate.launch

This will launch everything you need for a full simulation; roscore, the simulator, the joystick server, a model of the racecar and a preselected map.

### RVIZ

### Parameters

## C++ API

## Implementation Details

Distance transform.

## TODO

- Finish documentation
- Add max steering angle and max speed
- Add colision detection?
