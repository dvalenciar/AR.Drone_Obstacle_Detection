# AR.Drone Obstacle Detection

This repository contains the simulation source-code for obstacle detection using the AR Drone. This code detects the obstacles using through a Salient Histogram Back-projection algorithm.  

## Pre-requisites

* Operation System
  * Ubuntu 16.04
* [ROS](http://wiki.ros.org/kinetic/Installation/Ubuntu) (Kinetic)
* [GAZEBO 7.0](http://gazebosim.org/)
  

## Getting started - 
Make sure ROS and GAZEBO are correctly installed. 

Also, please install [ardrone_autonmy](https://github.com/AutonomyLab/ardrone_autonomy) and [tum_simuator packages](https://github.com/eborghi10/AR.Drone-ROS). You can find the instructions here:
[AR Drone Gazebo Installation](https://github.com/dvalenciar/AR_Drone_ROS_GUI#getting-started)


## Installation 

Download and install the package in your personal workspace (e.g. ~/drone_simulation_ws)

  ```
  cd ~/drone_simulation_ws/src
  git clone https://github.com/dvalenciar/AR.Drone_Obstacle_Detection.git
  cd ..
  catkin_make
  ```
  
  ## How to Run ~ Simple ~ single obstacle simulation ##


1. **Source your workspace environment**

  ```
  cd ~/drone_simulation_ws/
  source devel/setup.bash
  ```
2. **Run a simulation by executing a launch file:**

  ```
  roslaunch obstacle_detection simple.launch
  ```
  it will automatically start the obstacle detection node
  ![](https://github.com/dvalenciar/AR.Drone_Obstacle_Detection/blob/master/pic1.png)
  
  
3. **Take off the AR.Drone**

  ```
  rostopic pub -1 /ardrone/takeoff std_msgs/Empty
  ```


4. **Run the trajectory controller node**

 ```
 rosrun obstacle_detection PIDwithObstacleDete.py
 ```

The Drone will begin to move automatically in a straight line (until reaching the position X = 10, Y = 0). The algorithm  will detect the obstacle and avoid it.
