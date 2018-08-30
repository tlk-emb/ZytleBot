/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2014, Robert Bosch LLC.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Robert Bosch nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*
*********************************************************************/

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sstream>
#include <std_srvs/Empty.h>

#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/UInt8MultiArray.h"
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <geometry_msgs/Twist.h>

#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>

#include <fcntl.h>
#include <errno.h>
#include <sys/mman.h>
#include <sys/ioctl.h>
#include <iostream>
#include <stdio.h>
#include <string.h>

#include <linux/videodev2.h>


bool left_turn = true;
ros::Time left_time = ros::Time::now();
ros::Time right_time = ros::Time::now();

void imageViewer(const std_msgs::UInt8MultiArray& msg)
{
	cv::Mat rawimg(480, 640, CV_8UC2);
	cv::Mat dstimg(480, 640, CV_8UC2);
	memcpy(rawimg.data, &msg.data[0], 640 * 480 * 2);
	cv::cvtColor(rawimg, dstimg, cv::COLOR_YUV2RGB_YUYV);
	geometry_msgs::Twist twist;
	twist.linear.x = 0.0;
	twist.linear.y = 0.0;
	twist.linear.z = 0.0;
	twist.angular.x = 0.0;
	twist.angular.y = 0.0;

	cv::Mat color_mask, black_image, hsv_image;
    cv::cvtColor(image, hsv_image, CV_BGR2HSV);
    cv::inRange(hsv_image, cv::Scalar(0,0,0, 0) , cv::Scalar(179,128,100, 0), color_mask);
    cv::bitwise_and(image, image, result_image, color_mask);

    nzCount = cv::CountNonZero(result_image);

    std::cout << nzCount << std::endl;
	
	cv::imshow("rawimg", dstimg);
	cv::imshow("black_range", result_image);
	cv::waitKey(3);
	
	if (left_turn) {
	    twist.angular.z = 0.2;
	    twist_pub.publish(twist);
		
		left_time = ros::Time::now();
		if (left_time - right_time > ros::Duration(2.0))left_turn = false;
	} else {
		twist.angular.z = -0.2;
		twist_pub.publish(twist);

		right_time = ros::Time::now();
		if (left_time - right_time > ros::Duration(2.0))left_turn = true;
	}
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "pcamSub");
	ros::NodeHandle n;
	ros::Publisher twist_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);
	ros::Subscriber sub = n.subscribe("array", 1, imageViewer);
	ros::spin();

	return 0;
}
