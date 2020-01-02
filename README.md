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

[2. 2D+3D Mapping and Navigation with 3D Point Cloud Registration](#2D+3DMappingandNavigationwith3DPointCloudRegistration)

&ensp;&ensp;&ensp;[2.1. 2D+3D Mapping ](#2D+3DMapping)

&ensp;&ensp;&ensp;[2.2. 2D Navigation With 3D PointCloud Registration](#2DNavigationwith3DPointCloudRegistration)

[ Conclusion ](#conclusion)

[ References ](#references)


<a name="introduction"></a>
# Introduction

This project is done in Linux operationg system version 16.04.6 LTS (Xenial Xerus), and implemented on Turtlebot2 robot, ROS which is a middleware operating system in Kinetic version. Find below a hardware description of our Turtlebot2, equipped with a LiDar and a Kinect.

![alt text](https://github.com/Tostaky71/MSCV2_RoboticsProject/blob/master/images/hardware_description.PNG)

<a name="description"></a>
## Description of the Project

This project is composed of two parts : the first part is 2D Mapping and Navigation, and the second part is about 2D+3D Mapping and Navigation with 3D PointCloud Registration.

All this part has to control a robot automaticaly in its environment. This environment is composed of a 2D map which allows the robot to localize itself (create initial coordinate points) and a 3D map which will be created by the robot.
For the 3D map, we use RGBD-SLAM approach. RGBDSLAM acquire colored 3D models of objects and indoor scenes with a Kinect-style camera quickly. Actually the 3D map is a colored point cloud mapping designed by the RGB-D depth camera Kinect by Microsoft. When the robot has its 2D map, it obtains and matches the initial coordinates with the map and then moves with the goal coordinates pre-defined to finally create the 3D map of the area. On the 3D map obtained we can reconstruct a very realistic colored view and extract the details we want, for example if we want to reconstruct in 3D a big monument, we can extract the images of damages due to time from a drone and try to repair it by simulations before.

<a name="dependencies"></a>
## Dependencies

This repository contains 2 packages : the *my_package* package has to be installed on the workstation and the *my_package_turtlebot* has to be installed on the Turtlebot's laptop. These packages won't work properly if the following packages are not installed.

<a name="turtlebot_vibot"></a>
### Turtlebot_vibot

The turtlebot_vibot dependency provides the minimum to work with the Kobuki robot. It is composed of 2 main packages : The turtlebot_vibot _bringup to active and manipulate the robot (it enables the Lidar and the Kinect sensors only) and turtlebot_vibot_nav to use the 2D mapping with the LIDAR and make the robot autonomous with a knowned 2D map.

Find [here](https://github.com/roboticslab-fr/turtlebot_vibot "turtlebot_vibot package") the turtlebot_vibot package.

<a name="pcl"></a>
### Point Cloud Library

The PCL pointcloud library provides everything we want to create a real time 3D mapping for turtlebot robots. It contains filtering, feature estimation, surface reconstruction, registration, model fitting and segmentation. This library could be used with a laser scan (here the LIDAR) or a stereoscopic RGB-D Camera (here the Kinect) and can represent single scenes, create a map (for navigation) or reconstruct Object Models (for object recognition).
For our part we only use this library for 3D mapping in case of autonomous navigation.

Here is an example of pointcloud using for a single scene reconstruction.

![alt text](https://github.com/Tostaky71/MSCV2_RoboticsProject/blob/master/images/PCL_example.png)

Find [here](https://github.com/ros-perception/perception_pcl "perception_pcl package") the perception_pcl package.

<a name="actionlib"></a>
### ActionLib

The ActionLib package provides a standardized interface for performing tasks. Like for example, moving the base to a target location or performing a laser scan. The action client and server communicate with each other using a predefined action protocol. This action protocol relies on ROS topics in a specified ROS namespace in order to transport messages.

Find [here](https://github.com/ros/actionlib "actionlib package") the actionlib package.

<a name="rgbdslam"></a>
### RgbdSlam

rgbdslam (v2) is a SLAM solution for RGB-D cameras. It provides the current pose of the camera and allows to create a registered point cloud or an octomap. It features a GUI interface for easy usage, but can also be controlled by ROS service calls, e.g., when running on a robot.

Find [here](http://wiki.ros.org/rgbdslam "rgbdslam package") the description of this rgbdslam package. As the installation of this package was tricky, so in order to make things more convenient and easy to understand we have written a script ([setup.sh](https://github.com/Tostaky71/MSCV2_RoboticsProject/blob/master/setup.sh "Script to install rgbdslam")) which will do the job of installing rgbdlam package from scratch. 

<a name="rtabmap"></a>
### Rtabmap

This package is a ROS wrapper of RTAB-Map (Real-Time Appearance-Based Mapping), a RGB-D SLAM approach based on a global loop closure detector with real-time constraints. This package can be used to generate a 3D point clouds of the environment and/or to create a 2D occupancy grid map for navigation. The tutorials and demos show some examples of mapping with RTAB-Map.

Find [here](http://wiki.ros.org/rtabmap_ros "rtabmap_ros package") the rtabmap_ros package.


<a name="2DMappingAndNavigation"></a>
# 1.&ensp; 2D Mapping and Navigation

This part presents our method to, first, build a 2D map of a room, and then let the Turtlebot navigate autonomously thanks to a pre-defined path on the known 2D map, and with obstacles avoidance.


<a name="2DMapping"></a>
## 1.1.&ensp; 2D Mapping

For the 2D mapping, we have chosen to use the LiDar instead of the Kinect, because the Kinect can only see a small angle before its eyes whereas the LiDar sees everywhere around itself, a 360° angle.

![alt text](https://github.com/Tostaky71/MSCV2_RoboticsProject/blob/master/images/KinectLidar.PNG)

1. On the Turtlebot's laptop :
```
$ roslaunch my_package_turtlebot mapping.launch
```
This [mapping.launch](https://github.com/Tostaky71/MSCV2_RoboticsProject/blob/master/my_package_turtlebot/launch/mapping.launch "mapping.launch Turtlebot laptop") file brings up the kobuki base and the LiDar of the Turtlebot, and activates the mapping process.

2. On the Workstation :
```
$ roslaunch my_package mapping.launch
```
This [mapping.launch](https://github.com/Tostaky71/MSCV2_RoboticsProject/blob/master/my_package/launch/mapping.launch "mapping.launch Workstation") file allows us to control the Turtlebot manually with the logitech joystick, and visualize the robot and the mapping on Rviz. The joystick has to be plugged in the Workstation.

When the map has been built, we need to save it in order to do the next part : navigation. We use the command below to save the map, where */turtlebot_vibot_nav/maps/my_map* is the path and the name of the map file :
```
$ rosrun map_server map_saver -f /turtlebot_vibot_nav/maps/my_map
```
We should obtain two files describing the map : my_map.pgm and my_map.yaml


<a name="2DNavigationWithPathPlanning"></a>
## 1.2.&ensp; 2D Navigation with path planning

### Modifications

In this part, we need to modify few lines in the *turtlebot_vibot* package. For that, go into its sub-package *turtlebot_vibot_nav*, the *launch* directory and open the [amcl_demo_rplidar.launch](https://github.com/roboticslab-fr/turtlebot_vibot/blob/master/turtlebot_vibot_nav/launch/amcl_demo_rplidar.launch "amcl_demo_rplidar.launch turtlebot_vibot") file to modify it with the few lines below, starting from line 20 of the file :
```
<!-- Map server -->
<!--arg name="map_file" default="$(env TURTLEBOT_MAP_FILE)"/-->
<arg name="map_file" default="$(find turtlebot_vibot_nav)/maps/my_map.yaml"/>
<node name="map_server" pkg="map_server" type="map_server" args="$(arg map_file)" />
```
This modification of giving the path map, allows us to open the known map directly by launching this file. Plus, to start an autonomous navigation, the robot has to know its initial location on the known map, so we decide an initial position where we will always place the turtlebot before its navigation starts. For that, we need to make a manual navigation of the turtlebot in order to get its position coordinates on the known map. This is executable with the following commands :
1. On the Turtlebot's laptop :
```
$ roslaunch my_package_turtlebot navigation.launch
```
2. On the Workstation :
```
$ roslaunch turtlebot_teleop logitech.launch
$ roslaunch turtlebot_rviz_launchers view_navigation.launch --screen
$ rosrun tf tf_echo /map /base_link
```
This last line shows the current position coordinates of the turtlebot on the known map. Once we have placed (with the joystick) the turtlebot in the position we want to be the initial position of the autonomous navigation, we get its coordinates and we place them into the [amcl_demo_rplidar.launch](https://github.com/roboticslab-fr/turtlebot_vibot/blob/master/turtlebot_vibot_nav/launch/amcl_demo_rplidar.launch "amcl_demo_rplidar.launch turtlebot_vibot") file at lines 25, 26 and 27, instead of the 0.0 values :
```
<arg name="initial_pose_x" default="0.0"/> <!-- Use 17.0 for willow's map in simulation -->
<arg name="initial_pose_y" default="0.0"/> <!-- Use 17.0 for willow's map in simulation -->
<arg name="initial_pose_a" default="0.0"/>
```
Now we need to define some way points the turtlebot will go through during its autonomous navigation, starting from the initial position we have defined.

### Path planning

For this task, a python file has been created : [my_map_navigation.py](https://github.com/Tostaky71/MSCV2_RoboticsProject/blob/master/my_package/scripts/my_map_navigation.py "my_map_navigation.py") in which we use the *SimpleActionClient* of the actionlib library. This allows us to send an action goal to the turtlebot server by specifying which message we want to send. In our python file, we use the messages *actionlib_msgs*, *move_base_msgs* (*MoveBaseAction, MoveBaseGoal*), *the geometry_msgs* (*Point*) and the *kobuki_msgs* (*Sound*). We use this last one in order to make a particular sound each time the turtlebot reaches a way point. Lines 26 to 34 of our [my_map_navigation.py](https://github.com/Tostaky71/MSCV2_RoboticsProject/blob/master/my_package/scripts/my_map_navigation.py "my_map_navigation.py") file, we declare the coordinates of each way points into some variables :
```
# declare the coordinates of interest 
self.x1 = 1.431
self.y1 = 2.828
.
.
.
self.x0 = -1.462
self.y0 = -0.748
```
Then, for each way point, we create an *if* condition in which we say if the previous way point has been given and tried to be reached, then do the same for the current way point. It means that sometimes, the turtlebot is not able to reach the way point because of an obstacle located at the coordinates of that way point. So, in this case, the turtlebot does not find any path to reach that position and then gives up and goes to the next way point.

Below are way points we have defined for our own navigation, in this order : initial position, (x1,y1), (x2,y2), (x3,y3), (x0,y0).
![alt text](https://github.com/Tostaky71/MSCV2_RoboticsProject/blob/master/images/my_map_navigation.PNG)

### Execute the autonomous navigation

1. On the Turtlebot's laptop :
```
$ roslaunch my_package_turtlebot navigation.launch
```
This [navigation.launch](https://github.com/Tostaky71/MSCV2_RoboticsProject/blob/master/my_package_turtlebot/launch/navigation.launch "navigation.launch Turtlebot laptop") file brings up the kobuki base and the LiDar of the Turtlebot, and activates the navigation process with amcl and the LiDar.

2. On the Workstation :
```
$ roslaunch my_package navigation.launch
```
This [navigation.launch](https://github.com/Tostaky71/MSCV2_RoboticsProject/blob/master/my_package/launch/navigation.launch "navigation.launch Turtlebot laptop") file allows us to visualize the robot and its navigation on Rviz, and activates the pre-defined navigation.


<a name="2D+3DMappingandNavigationwith3DPointCloudRegistration"></a>
# 2.&ensp; 2D+3D Mapping and Navigation with 3D Point Cloud Registration
This part of our project will use a RGBDSLAM approach to first build a RTAB-map which is in the form of a databse file (by default rtabmap.db) and then using this database to navigate in this map usimg AMCL algorithm with pre-defined waypoints (path planning and with obstacle avoidance) and registering the 3-D Point Cloud Data of the environment in real time.


<a name="2D+3DMapping"></a>
## 2.1.&ensp; 2D+3D Mapping
For the 2D+3D Mapping, we used rtabmap_ros package where rtabmap is its main node which is a wrapper of the RTAB-Map Core library. This is where the graph of the map is incrementally built and optimized when a loop closure is detected. The online output of the node is the local graph with the latest added data to the map. The default location of the RTAB-Map database is "home/.ros/rtabmap.db" and the workspace is also set to "home/.ros". 

As we need to store the 3D information, we have to make sure we set subscribe_scan to "true" and explicitly set Grid/FromDepth to "true" to assemble 3D Kinect clouds for /rtabmap/cloud_map.

```
 <!-- Mapping -->
<node pkg="rtabmap_ros" type="rtabmap" name="rtabmap">
   <param name="Grid/FromDepth" type="string" value="true"/>   
   <param name="subscribe_scan"      type="bool"   value="true"/>
</node>
```

By default, rtabmap is in mapping mode i.e. it runs with an agrument "--delete_db_on_start" or "-d": Delete the database before starting. While in mapping mode, this argument should always be given otherwise the previous mapping session is loaded. All these have been taken care of in [3d-mapping-rtabmap.launch](https://github.com/Tostaky71/MSCV2_RoboticsProject/blob/master/my_package_turtlebot/launch/3d-mapping-rtabmap.launch "3d-mapping-rtabmap.launch Turtlebot laptop").

1. On the Turtlebot's laptop :
```
$ roslaunch my_package_turtlebot 3d-mapping-rtabmap.launch
```
This [3d-mapping-rtabmap.launch](https://github.com/Tostaky71/MSCV2_RoboticsProject/blob/master/my_package_turtlebot/launch/3d-mapping-rtabmap.launch "3d-mapping-rtabmap.launch Turtlebot laptop") file brings up the kobuki base, the logitech joystick to move the turtlebot and rgbd parameters of the camera (the Kinect) for starting a mapping process using both 2D and 3D information from the scene. The mapping process is similar to what we use in 
[gmapping_demo.launch](https://github.com/turtlebot/turtlebot_apps/blob/indigo/turtlebot_navigation/launch/gmapping_demo.launch "gmapping_demo.launch Turtlebot laptop") of the official [SLAM Map Building with TurtleBot tutorial](http://wiki.ros.org/turtlebot_navigation/Tutorials/Build%20a%20map%20with%20SLAM
"SLAM Map Building with TurtleBot"). 


2. On the Workstation :
```
$ roslaunch my_package view-rviz-rtabmap-mapping.launch
```

This [view-rviz-rtabmap-mapping.launch](https://github.com/Tostaky71/MSCV2_RoboticsProject/blob/master/my_package/launch/view-rviz-rtabmap-mapping.launch "view-rviz-rtabmap-mapping.launch Workstation") file will load rviz with a pre-saved configuration file to visualize the 2D+3D map with turtlebot model. After the mapping session is done, the RTAB-map or we can call it as rtabmap.db will be saved in the default database directory i.e. "home/.ros/". Make sure to either create a copy of this MAP otherwise, if another mapping session is started immediately after the last one, then it will erase the previous database file. The results after a successfull mapping can be seen as shown below:

![alt text](https://github.com/Tostaky71/MSCV2_RoboticsProject/blob/master/images/2d%2B3d_mapping.png)


<a name="2DNavigationwith3DPointCloudRegistration"></a>
## 2.2.&ensp; 2D Navigation with 3D Point Cloud Registration

### 2D Navigation

For the navigation, a python file has been created : [my_map_navigation2.py](https://github.com/Tostaky71/MSCV2_RoboticsProject/blob/master/my_package/scripts/my_map_navigation2.py "my_map_navigation2.py") which is very similar to the [my_map_navigation.py](https://github.com/Tostaky71/MSCV2_RoboticsProject/blob/master/my_package/scripts/my_map_navigation.py "my_map_navigation.py") file explained in part 1.2. Thanks to the 3D information we get from the map, the robot is able to localize itself autonomously, by turning round. So, we start the navigation by making the turtlebot turning round, so that he could localize itself from the begining of the navigation. Moreover, at each way point, we have decided to make the turtlebot also turning round in order to get a better 3D point cloud registration of its environment. We made it by importing the message *Twist* of *geometry_msgs*.

Below are way points we have defined for our own navigation, in this order : (x1,y1), (x2,y2), (x3,y3), (x0,y0), (xCenter, yCenter).
![alt text](https://github.com/Tostaky71/MSCV2_RoboticsProject/blob/master/images/my_map_navigation2.PNG)

The good thing is, until the robot localizes itself, it will not start updating the database file (rtabmap.db). This can be controlled by adding these lines to the the launch files:

```
<node name="rtabmap" pkg="rtabmap_ros" type="rtabmap" output="screen" args="$(arg args)">

<!-- localization mode -->
	  <param     if="$(arg localization)" name="Mem/IncrementalMemory" type="string" value="true"/>
	  <param unless="$(arg localization)" name="Mem/IncrementalMemory" type="string" value="true"/>
	  <param name="Mem/InitWMWithAllNodes" type="string" value="$(arg localization)"/> 
  </node>
```

### 3D Point Cloud Registration

Point cloud registration is the process of aligning two or more 3-D point clouds of the same scene. For example, the process can include reconstructing a 3-D scene from a Kinect device, building a map of a roadway for automobiles, and deformable motion tracking.

One important thing while navigation is, we should always have a RTAB-map (rtabmap.db) which was built during mapping session in Section 2.1 above. In order to achieve this goal, we have removed the argument which was given in mapping mode i.e. "--delete_db_on_start" in 
 [3d-navigation-rtabmap.launch](https://github.com/Tostaky71/MSCV2_RoboticsProject/blob/master/my_package_turtlebot/launch/3d-navigation-rtabmap.launch "3d-navigation-rtabmap.launch Turtlebot laptop") to avoid removal of the map.
 
 1. On the Turtlebot's laptop :
```
$ roslaunch my_package_turtlebot 3d-navigation-rtabmap.launch
```
This [3d-navigation-rtabmap.launch](https://github.com/Tostaky71/MSCV2_RoboticsProject/blob/master/my_package_turtlebot/launch/3d-navigation-rtabmap.launch "3d-navigation-rtabmap.launch Turtlebot laptop") file brings up the kobuki base, rgbd parameters of the camera (the Kinect) with localization parameters activated and executes a python file my_map_navigation2.py for starting a navigation with pre-defined waypoints (as explained above in this Section 2.2)

2. On the Workstation :
```
$ roslaunch my_package view-rviz-rtabmap-navigation.launch
```

This [view-rviz-rtabmap-navigation.launch](https://github.com/Tostaky71/MSCV2_RoboticsProject/blob/master/my_package/launch/view-rviz-rtabmap-navigation.launch "view-rviz-rtabmap-navigation.launch Workstation") file will load rviz with a pre-saved configuration file to visualize only 3D Point Cloud data which is being published presently on /rtabmap/cloud_map. Once the navigation starts, with a full rotation of the turtlebot and finishes after being through each of the pre-defined waypoints, we can obtain the 3D PointCloud data of the enivronment as shown in figure below:

![alt text](https://github.com/Tostaky71/MSCV2_RoboticsProject/blob/master/images/3D%20point%20cloud%20reconstruction.PNG)

Further if we want to save the obtained PointCloud information into a pcd file by using pcl_ros as shown below:
```
rosrun pcl_ros pointcloud_to_pcd input:=/rtabmap/cloud_map
```

And for using a pcd file to display the 3d map in rviz later, we use following command:
```
rosrun pcl_ros pcd_to_pointcloud point_cloud_file.pcd
```
<a name="conclusion"></a>
# Conclusion
In this project, we were able to develop a turtlebot2 robot to do the mapping and navigation in 2D and for the computer vision part of the robot, with RGBD approach, we were able to build visual odometry, 3D mapping and 3D reconstruction. 

![alt text](https://github.com/Tostaky71/MSCV2_RoboticsProject/blob/master/images/Conclution-Image.jpg)

Find [here](https://www.youtube.com/watch?v=6kZGIS9Ye74&t=1s "project video") the video that illustrates our project.


<a name="references"></a>
# References

[Navigation sending simple goals ROS tutorial](http://wiki.ros.org/navigation/Tutorials/SendingSimpleGoals "Sending simple goals ROS")

[ActionLib SimpleActionClient ROS tutorial](http://wiki.ros.org/actionlib_tutorials/Tutorials/SimpleActionClient "SimpleActionClient ROS")

[Navigate in a map Gaitech tutorial](http://edu.gaitech.hk/turtlebot/map-navigation.html "Gaitech navigation")

[Publishing messages examples](https://python.hotexamples.com/examples/kobuki_msgs.msg/Sound/value/python-sound-value-method-examples.html "Publishing messages")

