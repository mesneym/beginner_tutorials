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

#### Starting both publish and subscriber using Roslaunch(optional) 
On a new terminal, Add package to ROS workspace, enter specified   
freq(display frequency) and launch  application. For eg

```
cd ~/catkin_ws/
roslaunch beginner_tutorials beginner_tutorials.launch freq:="Integer Value"
```
If the frequency argument is not specified, the default frequency
value(10) is utilized

#### Calling Service
On a new terminal, Add package to ROS workspacke and start service with  
the instructions below. For Eg, to perfom integer over interger division  
with 4 and 2 as operands, enter the following.
```
rosservice call /divide_two_nums 4 2
```

### Rqt Console
To launch Rqt Console, open a new terminal and enter the instruction given  
below

```
rosrun rqt_console rqt_console
```
open a new terminal and enter the following

```
rosrun rqt_logger_level rqt_logger_level
```


### Record and Play Topics
To record published messages and play them at a later time, create a new terminal, add   
package to workspace and follow the instructions below.

```
  cd ~/catkin_ws
  roslaunch beginner_tutorials beginner_tutorials.launch record:=true

```
Wait for 15sec and terminate program  

Enter the command below to view information on the published messages recorded
```
  rosbag info record.bag
```

To play recorded messages, open two terminals, add package to workspace and run
instructions in the different terminals.


```
rosrun beginner_tutorials listner
```

```
rosbag play record.bag

```
Verify messages received by listner node


