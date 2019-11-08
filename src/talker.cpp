/******************************************************************************
 * MIT License
 *
 * Copyright (c) 2019 Akwasi A Obeng 
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 ******************************************************************************/

#include <sstream>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "beginner_tutorials/DivideTwoNum.h"
#include <tf/transform_broadcaster.h>

/**
 * @brief divide -perfoms division operation
 * @param req - contains operands for division
 * @param res - stores result of division operation
 * @return boolean - returns true for successful operation
 */
bool divide(beginner_tutorials::DivideTwoNum::Request  &req,
         beginner_tutorials::DivideTwoNum::Response &res) {
     if (req.b == 0)
       ROS_FATAL_STREAM("dividing by zero");
     res.result = req.a / req.b;  // perfoming int over int division
     ROS_WARN_STREAM("precision may be lost from int over int division");

     // print both operands and result
     ROS_INFO("request: x=%ld, y=%ld", (long int)req.a, (long int)req.b);
     ROS_INFO("sending back response: [%ld]", (long int)res.result);
     return true;
}

 /**
  * This tutorial demonstrates simple sending of messages over the ROS system.
  */
int main(int argc, char **argv) {
   /**
	* The ros::init() function needs to see argc and argv so that it can perform
	* any ROS arguments and name remapping that were provided at the command line.
	* For programmatic remappings you can use a different version of init() which takes
	* remappings directly, but for most command-line programs, passing argc and argv is
	* the easiest way to do it.  The third argument to init() is the name of the node.
	*
	* You must call one of the versions of ros::init() before using any other
	* part of the ROS system.
	*/
ros::init(argc, argv, "talker");
   /**
	* NodeHandle is the main access point to communications with the ROS system.
	* The first NodeHandle constructed will fully initialize this node, and the last
	* NodeHandle destructed will close down the node.
	*/
ros::NodeHandle n;
   /**
	* The advertise() function is how you tell ROS that you want to
	* publish on a given topic name. This invokes a call to the ROS
	* master node, which keeps a registry of who is publishing and who
	* is subscribing. After this advertise() call is made, the master
	* node will notify anyone who is trying to subscribe to this topic name,
	* and they will in turn negotiate a peer-to-peer connection with this
	* node.  advertise() returns a Publisher object which allows you to
	* publish messages on that topic through a call to publish().  Once
	* all copies of the returned Publisher object are destroyed, the topic
	* will be automatically unadvertised.
	*
	* The second parameter to advertise() is the size of the message queue
	* used for publishing messages.  If messages are published more quickly
	* than we can send them, the number here specifies how many messages to
	* buffer up before throwing some away.
	*/
ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);
ros::ServiceServer service = n.advertiseService("divide_two_nums", divide);
ROS_INFO_STREAM("Ready to perform Division operation");

int frequency =  atoi(argv[1]);
if(frequency < 0) {  // frequencies can't be negative
    ROS_ERROR_STREAM("Negative display frequency");
    frequency = 10;  // resetting frequency to default
}


ros::Rate loop_rate(frequency);  // display rate

   /**
	* A count of how many messages we have sent. This is used to create
	* a unique string for each message.
	*/
int count = 0;
while (ros::ok()) {
	 /**
	  * This is a message object. You stuff it with data, and then publish it.
	  */
    std_msgs::String msg;
    std::stringstream ss;
    ss << "ENPM 808X " << count;
    msg.data = ss.str();
    ROS_INFO_STREAM("" << msg.data.c_str());
    ROS_DEBUG_STREAM("display frequency " << frequency);

    /**
     * Setup the matrix representing the position and orientation
     * of talker frame with respect to world frame 
     */
    tf::Transform transform;
    transform.setOrigin( tf::Vector3(2,2,2) ); 
    tf::Quaternion q;  
    q.setRPY(10, 10, 10);  
    transform.setRotation(q); 

    /*
     * Broadcast Frame of talker(child frame) with respect to some 
     * world frame(parent frame) for some time stamp
     */
    tf::TransformBroadcaster br;
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "talker"));

	 /**
	  * The publish() function is how you send messages. The parameter
	  * is the message object. The type of this object must agree with the type
	  * given as a template parameter to the advertise<>() call, as was done
	  * in the constructor above.
	 */
    chatter_pub.publish(msg);

    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }
  return 0;
}
