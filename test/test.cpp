
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

/**
 *@file test.cpp
 *@author: Akwasi A Obeng
 *Created on:Nov 10th, 2019
 *@brief Test for talker node
 */

#include <sstream>
#include "tf/transform_broadcaster.h"
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "gtest/gtest.h"
#include "beginner_tutorials/DivideTwoNum.h"

/**
 * @brief Testcase to check divide_two_nums sevice
 * @param None
 * @return None
 */
TEST(TestSuite, serviceTest) {
  ros::NodeHandle n;
  ros::ServiceClient client =  n.serviceClient<beginner_tutorials::DivideTwoNum>
                               ("divide_two_nums");
  beginner_tutorials::DivideTwoNum srv;  // Performs integer division
  srv.request.a = 1;                     // 1/2 should return 0
  srv.request.b = 2;

  EXPECT_EQ(0, (int)srv.response.result);
}

/**
  * This tutorial demonstrates a simple test for ROS
  */
int main(int argc, char **argv) {
  ros::init(argc, argv, "tester");
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
