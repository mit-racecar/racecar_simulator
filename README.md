# Racecar Simulator

This is a lightweight 2D simulator of the MIT Racecar.
It can be built with ROS, or it can be used as a standalone C++ library.

## ROS

### Dependencies

If you have ```ros-melodic-desktop``` installed, the additional dependencies you must install are:

- tf2_geometry_msgs
- ackermann_msgs
- joy
- map_server

You can install them by running:

    sudo apt-get install ros-melodic-tf2-geometry-msgs ros-melodic-ackermann-msgs ros-melodic-joy ros-melodic-map-server

The full list of dependencies can be found in the package.xml file.

### Installation

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
