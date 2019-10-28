# Beginner Tutorials For Ros
This ROS package illustrates how to subscribe and publish messages on ROS

### Dependencies
The package assumes that **ROS Kinetic** is installed.  
Cmake minimum version - 2.8.3   

### Build the project

```
cd ~/
git clone https://github.com/mesneym/beginner_tutorials.git'
cd beginner_tutorials/
catkin_make
```
#### Add package to ROS workspace
```
. ~/beginner_tutorials/devel/setup.bash
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
rosrun publish_subscriber talker
```

#### Start the subscriber
On a new terminal Add package to ROS workspace and start the subscriber
```
rosrun publish_subscriber listener
```
Press ctrl-c to terminate both listner and subscriber
