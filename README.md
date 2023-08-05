# UR3_ERC_Maintenance_Stack

This repository contains the Maintenance Stack for the ERC Remote competition. 

## Table of Contents
1. [Requirements](#Requirements)
    1. [Download UR3 simulation](#Download-UR3-simulation)
    1. [Run simulation](#Run-simulation)
1. [Building](#building)
1. [Running the Simulation](#Running-the-Simulation)
    1. [Launching the Objective](#Launching-the-Objective)

## Requirements

The simulation requires ROS and simulations of the Universal Robots UR3 robot created for the ERC competition
 - [ROS Noetic Ninjemys](http://wiki.ros.org/noetic/Installation/) on [Ubuntu 20.04 Focal Fossa](https://releases.ubuntu.com/20.04/)
 - [ERC-Remote-Maintenance-Sim](https://github.com/EuropeanRoverChallenge/ERC-Remote-Maintenance-Sim)

### Download UR3 simulation

For the simulation to work on your system, you must install dependencies and download repositories by running the following commands:

```sh
source /opt/ros/noetic/setup.bash
sudo apt-get update && apt-get upgrade -y && apt-get install -y lsb-core g++
sudo apt-get install git
rosdep init && rosdep update
sudo apt install ros-noetic-moveit -y
sudo apt install ros-noetic-ros-controllers* -y
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
git clone -b kinetic-devel https://github.com/ros-industrial/universal_robot.git
sudo rm -r universal_robot/ur_msgs
git clone https://github.com/roboticsgroup/roboticsgroup_gazebo_plugins
git clone https://github.com/Michal-Bidzinski/UR3_sim.git
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```
### Run simulation
To run UR3 simulation in Gazebo with MoveIt!, and RVzi GUI, including an example cell:
```sh
roslaunch ur3_sim simulation.launch
```
To run UR3 simulation in Gazebo with MoveIt!, and RVzi GUI, containing only a robot with a grapple and a camera (as per real setup):
```sh
roslaunch ur3_sim real_station.launch
```

## Building
This repository contains all the competition submissions.

To clone the repo:
```sh
git clone https://github.com/rukna40/ERC_Maintenance_Stack.git
```

navigate to your root directory:
```sh
cd
```
clone this repository
Navigate to the repository:
```sh
cd ERC_Maintenance_Stack
```
Now, use the `catkin_make` tool to build the workspace:
```sh
catkin_make
```
If you don’t want to have to source the setup file every time you open a new shell, then you can add the commands to your shell startup script:
```sh
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
echo "source ~/ERC_Maintenance_Stack/devel/setup.bash" >> ~/.bashrc
```
## Running the Simulation
Before running the launch file, you will need to make all the Python files executable.

To do that navigate to the `scripts` folder of that package, and run the following command
```sh
chmod +x *.py 
```

 ### Pull docker
 ```sh
 sudo docker pull ghcr.io/rukna40/erc_maintenance_stack:latest
 ```
 ### Run Codes

 Run the docker.
 ```sh
 sudo docker run -it --net=host --name mrm_img ghcr.io/rukna40/erc_maintenance_stack:latest
 ```
 Navigate to the scripts folder inside the docker.
 ```sh
 cd src/marsrovermanipal_td2/scripts
 ```
 Make the scripts executable.
 ```sh
 chmod +x *.py 
 ```
 Navigate back to the root directory.
 ```sh
 cd ../../../
 ```
 Open another terminal and navigate to UR3 simulation directory.
 ```sh
 cd catkin_ws
 ```
 Source setup.bash.
 ```sh
 source devel/setup.bash
 ```
 Launch the UR3 simulation.
 ```sh
 roslaunch ur3_sim simulation.launch
 ```
 Go back to docker terminal and source it.
 ```sh
 source devel/setup.bash
 ```
 Launch the Objectives.
 ```sh
 roslaunch marsrovermanipal_td2 obj1.launch.
 ```
 ### Launching the Objective
To run the simulation, run `roslaunch <package_name> obj<objective_no>.launch` and the parameters if any.

Example:

- objective 1
```sh
roslaunch marsrovermanipal_td2 obj1.launch
```
- objective 2
```sh
roslaunch marsrovermanipal_td2 obj2.launch tags:="1, 2, 3, 4"
```
- objective 3
```sh
roslaunch marsrovermanipal_td2 obj3.launch
```
- objective 4
```sh
roslaunch marsrovermanipal_td2 obj4.launch angle:=45
```
- objective 5
```sh
roslaunch marsrovermanipal_td2 obj5.launch
```
- objective 6
```sh
roslaunch marsrovermanipal_td2 obj6.launch
```
- objective 7
```sh
roslaunch marsrovermanipal_td2 obj7.launch
```
- objective 8
```sh
roslaunch marsrovermanipal_td2 obj8.launch
```
- objective 9
```sh
roslaunch marsrovermanipal_td2 obj9.launch tag:="1"
```
- objective 10
```sh
roslaunch marsrovermanipal_td2 obj10.launch
```
