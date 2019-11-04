# Beginner Tutorials For Ros
This ROS package illustrates how to subscribe and publish messages on ROS

### Dependencies
The package assumes that **ROS Kinetic** is installed.  
Cmake minimum version - 2.8.3   

### Build the project

```
mkdir ~/catkin_ws/src
cd ~/catkin_ws/src
git clone 'https://github.com/mesneym/beginner_tutorials.git'
cd  ~/catkin_ws 
catkin_make
```
#### Add package to ROS workspace
```
. ~/catkin_ws/devel/setup.bash
```

### Run the Package
Follow the instructions below to run the package.


#### Start the Master core
On a new terminal Add package to ROS workspace and start roscore
```
roscore
```

#### Start the publisher
On a new terminal Add package to ROS workspace and start the publisher
```
rosrun beginner_tutorials talker
```

#### Start the subscriber
On a new terminal Add package to ROS workspace and start the subscriber
```
rosrun beginner_tutorials listener
```
Press ctrl-c to terminate both listner and subscriber
