# Daedalus-ROS-tools
This repository is a fork of some simple tools used in the DAEDALUS-project, where a spherical mapping system was implemented in coorporation with the European Space Agency (ESA). 
See the link for more details about that:
https://www.informatik.uni-wuerzburg.de/space/mitarbeiter/nuechter/projects/daedalus/

## About the toolset
The tools provided are ROS programs, thus you can extract the contents of the src-folder into your existing catkin-workspace's src-folder.

### encoder_2_height
This is a node that reads the joint states of a rotation-encoder and publishes the amount of cable that has been rolled over the cable reel, using the Helix-Arc-length formula. Radius of the large and small gears, as well as helix radius and cable diameter can be adjusted in the .cpp file directly.

### file_writer
This is a node that reads a Pointcloud as well as pose measurements from ROS topics, and records them on disk using the .3d and .pose format (3DTK-like, lefthanded-system OpenGL-style). The format is as follows:
 - .3d : x y z reflectance
 - .pose x y z rotX rotY rotZ
 
 Note that x, y, and z are given in meters, reflectance as [0-255] and rotations in degrees.
 The topics can be modified directly in the .cpp file.

### file_writer_on_rest
Does the same as the above, but only writes on disk if the last <N_PAST_POSES> recorded poses had a small difference of at most <EPS_POSE_DIFF>, i.e., the vehicle was resting for a significant amount of time.
Both parameters can be tunes in the .cpp file and are defaulted to:
 - N_PAST_POSES = 50   # used for 200Hz input
 - EPS_POSE_DIFF = 0.001   # small number, must be of larger order than system noise!!! 

In addition, the pose this time has the type of nav_msgs::Odometry::ConstPtr, yet the callback functions are easily replaceable.

### livox_update_time
Simple node to give each sensor_msgs::PointCloud2 the current time in its header.
This might come in handy when messing around with bagfiles containing LiDAR measurements, and has been used with a LIVOX device.

### pose_merger
Node that reads off the ROS topics "position" and "orientation" and publishes "pose". 
Position must be of type geometry_msgs::Point::ConstPtr, orientation must be of type sensor_msgs::Imu::ConstPtr, and pose will be of type geometry_msgs::PoseStamped.

### testing 
Contains simple test cases where certain ROS topics get advertised and dummy data gets published.
Usually usefull when checking if your installation works or for debugging.

## Dependency list
You need to have ROS installed and a catkin workspace available.
Tested with both noetic, kinetic, and melodic and works with Ubuntu 18.04 as well as 20.04. 
Usually, an installation of ros-<distro>-desktop-full will do the trick.
However, if you are having problems with your dependencies, or want to keep a minimal installation, here is a list of packages you have to install:
 - ros-<distro>-geometry-msgs
 - ros-<distro>-std-msgs
 - ros-<distro>-sensor-msgs
 - ros-<distro>-tf
 - ros-<distro>-tf2
 - ros-<distro>-pcl-conversions
 - ros-<distro>-pcl-msgs
 - ros-<distro>-pcl-ros
 - ros-<distro>-nav-msgs
 - ros-<distro>-cpp-common

