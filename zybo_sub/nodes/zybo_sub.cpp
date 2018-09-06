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

#define WIDTH 640
#define HEIGHT 480

cv::Mat camera_mtx;
cv::Mat camera_dist;
void imageViewer(const std_msgs::UInt8MultiArray& msg)
{
	cv::Mat rawimg(HEIGHT, WIDTH , CV_8UC2);
	cv::Mat dstimg(HEIGHT, WIDTH, CV_8UC2);
	memcpy(rawimg.data, &msg.data[0], WIDTH * HEIGHT * 2);
	cv::cvtColor(rawimg, dstimg, cv::COLOR_YUV2RGB_YUYV);

	cv::Mat after_img, dstimg_half;
//cv::resize(dstimg, dstimg_half, cv::Size(), 0.5, 0.5);

	cv::undistort(dstimg, after_img, camera_mtx, camera_dist);
	
	
	cv::imshow("before_img", dstimg);
	cv::imshow("after_img", after_img);
	cv::waitKey(3);
}

int main(int argc, char **argv)
{
	cv::FileStorage fs("/home/cf/catkin_ws/calibration.yml", cv::FileStorage::READ);
	
	fs["mtx"] >> camera_mtx;
	fs["dist"] >> camera_dist;
	fs.release();
	ros::init(argc, argv, "pcamSub");
	ros::NodeHandle n;
	ros::Subscriber sub = n.subscribe("image_array", 1, imageViewer);
	ros::spin();

	return 0;
}
