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
    git clone https://github.com/mit-racecar/racecar_simulator.git
    
Then run ```catkin_make``` to build it:

    cd ~/catkin_ws
    catkin_make
    source devel/setup.bash

## Quick Start

To run the simulator on its own, run:

    roslaunch racecar_simulator simulate.launch

This will launch everything you need for a full simulation; roscore, the simulator, a preselected map, a model of the racecar and the joystick server.

### RVIZ Visualization

With the simulator running, open rviz.
In the left panel at the bottom click the "Add" button, then in the "By topic" tab add the ```/map``` topic and the ```/scan``` topic.
Then in the "By display type" tab add the RobotModel type.
In the left panel under the newly added LaserScan section, change the size to 0.1 meters for a clearer visualization of the lidar (shown in rainbow).

You should see the car sitting in the middle of a 2D map of MIT's building 31 as shown below:

![The racecar in the starting position](https://raw.githubusercontent.com/mit-racecar/racecar_simulator/master/media/racecar_simulator_rviz_1.png)

You can use a USB joystick to drive the car around, or you can place the car manually by clicking the "2D Pose Estimate button" on the top of the screen and dragging your mouse on the desired pose.

![The racecar in a cubicle](https://raw.githubusercontent.com/mit-racecar/racecar_simulator/master/media/racecar_simulator_rviz_2.png)

### ROS API

To make the car drive autonomously, publish [AckermannDrive](http://docs.ros.org/melodic/api/ackermann_msgs/html/msg/AckermannDrive.html) messages to the ```/drive``` topic.

To instantly move the car to a new state publish [Pose](http://docs.ros.org/lunar/api/geometry_msgs/html/msg/Pose.html) messages to the ```/pose``` topic. This can be useful for scripting the car through a series of automated tests.

The simulated lidar is published to the ```/scan``` topic as [LaserScan](http://docs.ros.org/melodic/api/sensor_msgs/html/msg/LaserScan.html) messages.

The pose of the car is broadcast as a transformation between the ```map``` frame and the ```base_link``` frame. ```base_link``` is the center of the rear axis. The ```laser``` frame defines the frame from which the lidar scan is taken and another transform is broadcast between it and ```base_link```.

### Parameters

## C++ API

## Implementation Details

Distance transform.
Ackermann kinematics

## TODO

- Finish documentation
- Add max steering angle and max speed
- Add colision detection?
