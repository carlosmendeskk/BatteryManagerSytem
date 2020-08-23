# Robots

## Overview:

ROS Package that simulates a fleet manager system. The manager systems takes only battery level into account.

## Architeture
---
TODO

## Installation
---
TODO

### Requirements:

>**Distributor ID:**	*Ubuntu*

>**Release:**	*18.04*

>**Codename:**	*bionic*

>**ROS Distribution:** *Melodic*

### Install and Run:

```sh
$ mkdir -p ~/robot_ws/src && cd ~/robot_ws/src/
$ git clone git@github.com:carlosmendeskk/BatteryManagerSytem.git robots
$ cd ~/robot_ws/
$ catkin_make
$ source devel/setup.bash
$ roslaunch robots simulation.launch
```

### Example Output:
The mentioned launch file starts the manager node and 2 robot nodes.
