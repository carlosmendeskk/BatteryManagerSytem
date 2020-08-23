# Robots

## Overview:

ROS Package that simulates a battery manager system for a fleet of robots. 

## Architecture
---
![Alt text](doc/diagram.png?raw=true "Manager Robot interfaces")

The goal for this project was to create a manager system that deals with several robots. To fullfil this, two type of nodes were created: 
- **Manager Node**
 
- **Robot Node**   

These two type of nodes communicate via two types of services:
- **Register Service**:
  - Service provided by the **Manager node** used to register a specific robot in the Manager list. 

- **Control Service**:
  - Service provided by the **Robot node** that enables the user to control the robot. 

To simulate the battery a class Battery was created. Based on the charge and discharge rates it calculates the % of battery 
based on the elapsed time.

![Alt text](doc/RobotBattery.png?raw=true "Robot Battery")

### **Notes**:
    Currently the maximum number of allowed robots is 10.

## Installation

### Software requirements:

>**Distributor ID:**	*Ubuntu*

>**Release:**	*18.04*

>**Codename:**	*bionic*

>**ROS Distribution:** *Melodic*

### Install:

```sh
$ mkdir -p ~/robot_ws/src && cd ~/robot_ws/src/
$ git clone git@github.com:carlosmendeskk/BatteryManagerSytem.git robots
$ cd ~/robot_ws/
$ catkin_make
```

### Run:

There are multiple ways to launch the simulation. 

**Launch Files**:
-  simulation.launch - Spawns the manager node and two robots.
-  manager.launch - Spawn the manager node. 
-  robot.launch - Spawns a robot node. 

**Notes**:
- When launching the robot, ensure that the manager node is running. If the manager Register service is not running, the robot node will only wait 5 seconds until it stops.
- If manager node is dies, the robot nodes will all stop operation.
- 
```
$ source devel/setup.bash
$ roslaunch robots simulation.launch
```


### Example Output:
The mentioned launch file starts the manager node and 2 robot nodes.


### Future Work:
-   Add retry mechanism into robot to reconnect with manager if it gets disconnected.
-   Extend the Control service to support several type of method ids.
-   Add state machine to robot class.