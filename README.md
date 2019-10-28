# Beginner Tutorials For Ros

### Build the project

```
cd ~/
git clone https://github.com/mesneym/beginner_tutorials.git'
cd beginner_tutorials/
catkin_make
```
####To add package to workspace
```
. ~/beginner_tutorials/Publish_Subscriber_pkg/devel/setup.bash
```

### Run the Package

#### Start the Master core
```
roscore
```

#### Start the publisher
```
rosrun publish_subscriber talker
```

#### Start the subscriber
```
rosrun publish_subscriber listener
```
Press ctrl-c to terminate both listner and subscriber
