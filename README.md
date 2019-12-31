# MSCV2_RoboticsProject


# Table of contents
[ Introduction ](#introduction)

&ensp;&ensp;&ensp;[Description of the Project ](#description)

&ensp;&ensp;&ensp;[Dependencies ](#dependencies)

&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;[ Turtlebot_vibot ](#turtlebot_vibot)

&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;[ Point Cloud Library ](#pcl)

&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;[ ActionLib ](#actionlib)

&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;[ RgbdSlam ](#rgbdslam)

&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;[ Rtabmap ](#rtabmap_ros)

[1. 2D Mapping and Navigation ](#2DMappingAndNavigation)

&ensp;&ensp;&ensp;[1.1. 2D Mapping ](#2DMapping)

&ensp;&ensp;&ensp;[1.2. 2D Navigation with path planning](#2DNavigationWithPathPlanning)

[2. 2D + 3D Mapping and Navigation ](#2D+3DMappingAndNavigation)

&ensp;&ensp;&ensp;[2.1. 2D+3D Mapping ](#2D+3DMapping)

&ensp;&ensp;&ensp;[2.2. 2D Navigation With 3D pointCloud Registration](#2DNavigationwith3DPointCloudRegistration)

[ Conclusion ](#conclusion)


<a name="introduction"></a>
# Introduction

<a name="description"></a>
## Description of the Project

<a name="dependencies"></a>
## Dependencies

This repository contains 2 packages : the *my_package* package has to be installed on the workstation and the *my_package_turtlebot* has to be installed on the Turtlebot's laptop. These packages won't work properly if the following packages are not installed.

<a name="turtlebot_vibot"></a>
### Turtlebot_vibot

Find [here](https://github.com/roboticslab-fr/turtlebot_vibot "turtlebot_vibot package") the turtlebot_vibot package.

<a name="pcl"></a>
### Point Cloud Library

Find [here](https://github.com/ros-perception/perception_pcl "perception_pcl package") the perception_pcl package.

<a name="actionlib"></a>
### ActionLib

The ActionLib package provides a standardized interface for performing tasks. Like for example, moving the base to a target location or performing a laser scan. The action client and server communicate with each other using a predefined action protocol. This action protocol relies on ROS topics in a specified ROS namespace in order to transport messages.

Find [here](https://github.com/ros/actionlib "actionlib package") the actionlib package.

<a name="rgbdslam"></a>
### RgbdSlam

rgbdslam (v2) is a SLAM solution for RGB-D cameras. It provides the current pose of the camera and allows to create a registered point cloud or an octomap. It features a GUI interface for easy usage, but can also be controlled by ROS service calls, e.g., when running on a robot.

Find [here](http://wiki.ros.org/rgbdslam "rgbdslam package") the rgbdslam package.

<a name="rtabmap"></a>
### Rtabmap

This package is a ROS wrapper of RTAB-Map (Real-Time Appearance-Based Mapping), a RGB-D SLAM approach based on a global loop closure detector with real-time constraints. This package can be used to generate a 3D point clouds of the environment and/or to create a 2D occupancy grid map for navigation. The tutorials and demos show some examples of mapping with RTAB-Map.

Find [here](http://wiki.ros.org/rtabmap_ros "rtabmap_ros package") the rtabmap_ros package.


<a name="2DMappingAndNavigation"></a>
# 1.&ensp; 2D Mapping and Navigation




<a name="2DMapping"></a>
## 1.1.&ensp; 2D Mapping

1. On the Turtlebot's laptop :
``` console
roslaunch my_package_turtlebot mapping.launch
```
This [mapping.launch](https://github.com/Tostaky71/MSCV2_RoboticsProject/blob/master/my_package_turtlebot/launch/mapping.launch "mapping.launch Turtlebot laptop") file brings up the kobuki base and the LiDar of the Turtlebot, and activates the mapping process.

2. On the Workstation :
``` console
roslaunch my_package mapping.launch
```
This [mapping.launch](https://github.com/Tostaky71/MSCV2_RoboticsProject/blob/master/my_package/launch/mapping.launch "mapping.launch Workstation") file allows us to control the Turtlebot manually with the logitech joystick, and visualize the robot and the mapping on Rviz. The joystick has to be plugged in the Workstation.


<a name="2DNavigationWithPathPlanning"></a>
## 1.2.&ensp; 2D Navigation with path planning

1. On the Turtlebot's laptop :
``` console
roslaunch my_package_turtlebot navigation.launch
```
This [navigation.launch](https://github.com/Tostaky71/MSCV2_RoboticsProject/blob/master/my_package_turtlebot/launch/navigation.launch "navigation.launch Turtlebot laptop") file brings up the kobuki base and the LiDar of the Turtlebot, and activates the navigation process with amcl and the LiDar.

2. On the Workstation :
``` console
roslaunch my_package navigation.launch
```
This [navigation.launch](https://github.com/Tostaky71/MSCV2_RoboticsProject/blob/master/my_package/launch/navigation.launch "navigation.launch Turtlebot laptop") file allows us to visualize the robot and its navigation on Rviz, and activates the pre-defined navigation.


<a name="2D+3DMappingAndNavigation"></a>
# 2.&ensp; 2D + 3D Mapping and 3D Point Cloud Registration by 2D Navigation



<a name="2D+3DMapping"></a>
## 2.1.&ensp; 2D + 3D Mapping



<a name="2DNavigationwith3DPointCloudRegistration"></a>
## 2.2.&ensp; 2D Navigation with 3D Point Cloud Registration


<a name="conclusion"></a>
# Conclusion

Find [here](https://www.youtube.com/watch?v=6kZGIS9Ye74&t=1s "project video") the video that illustrates our project.






