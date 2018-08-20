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


static int xioctl(int fd, int request, void *arg)
{
    int r;
	do {
		r = ioctl (fd, request, arg);
		if (request == VIDIOC_DQBUF) {
			std::cout << "r : " << r << std::endl;
		}
	} while (-1 == r && EINTR == errno);
	return r;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "ubuntuPcam");
	ros::NodeHandle n;
	ros::Publisher pub = n.advertise<std_msgs::UInt8MultiArray>("array",  640 * 480 * 2);

	unsigned char *buffer;

	// 1. Open Video Device.
	int fd;
	fd = open("/dev/video0", O_RDWR, 0);
	if (fd == -1)
	{
	    std::cout << "Failed to open video device." << std::endl;
	    return 1;
	}

	// 2. Querying video capabilities.
	struct v4l2_capability caps;
	memset(&caps, 0, sizeof(caps));
	if (-1 == xioctl(fd, VIDIOC_QUERYCAP, &caps))
	{
		std::cout << "Failed to query capabilities." << std::endl;
	    return 1;
	}
	std::cout << "bus_info	: " << caps.bus_info << std::endl;
	std::cout << "card		: " << caps.card << std::endl;
	std::cout << "driver	: " << caps.driver << std::endl;
	std::cout << "version	: " << caps.version << std::endl;

	// 3. Format Specification.
	{
		struct v4l2_format fmt;
		memset(&(fmt), 0, sizeof(fmt));

		fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		fmt.fmt.pix.width = 640;
		fmt.fmt.pix.height = 480;
		fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_YUYV;
		//fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_GREY;
		fmt.fmt.pix.field = V4L2_FIELD_NONE;

		if (-1 == xioctl(fd, VIDIOC_S_FMT, &fmt))
		{
			std::cout << "Failed to set pixel format." << std::endl;
			return 1;
		}
	}

	// 4. Request Buffer
	{
		struct v4l2_requestbuffers req;
		memset(&(req), 0, sizeof(req));
		req.count = 1;
		req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		req.memory = V4L2_MEMORY_MMAP;

		if (-1 == xioctl(fd, VIDIOC_REQBUFS, &req))
		{
			std::cout << "Failed to request buffer." << std::endl;
			return 1;
		}
	}

	// 5. Query Buffer
	{
		struct 	v4l2_buffer buf;
		memset(&(buf), 0, sizeof(buf));
		buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		buf.memory = V4L2_MEMORY_MMAP;
		//buf.index = bufferindex;
		buf.index = 0;
		if(-1 == xioctl(fd, VIDIOC_QUERYBUF, &buf))
		{
			std::cout << "Failed to query buffer." << std::endl;
			return 1;
		}

		std::cout << "buf.length : " << buf.length << std::endl;
		std::cout << "buf.m.offset : " << buf.m.offset << std::endl;

		buffer = (unsigned char*)mmap (NULL, buf.length, PROT_READ | PROT_WRITE, MAP_SHARED, fd, buf.m.offset);
	}

	// 6. Start Streaming
	{
		struct 	v4l2_buffer buf;
		memset(&(buf), 0, sizeof(buf));
		buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		if(-1 == xioctl(fd, VIDIOC_STREAMON, &buf.type))
		{
			std::cout << "Start Capture" << std::endl;
			return 1;
		}
	}

	//cv::namedWindow("edges",1);
	//cv::Mat bayerRaw(480, 640, CV_8UC1);
	//cv::Mat color(480, 1, CV_8UC3);
	std_msgs::UInt8MultiArray camdata;
	camdata.data = std::vector<uint8_t>(640*480*2);

	struct 	v4l2_buffer buf;
	buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	buf.memory = V4L2_MEMORY_MMAP;
	buf.index = 0;
	while (true){
		// 7. Capture Image
		{
			// Connect buffer to queue for next capture.
			if (-1 == xioctl(fd, VIDIOC_QBUF, &buf)) {
				std::cout << "VIDIOC_QBUF" << std::endl;
			}

			fd_set fds;
			FD_ZERO(&fds);
			FD_SET(fd, &fds);
			struct timeval tv = {0};
			tv.tv_sec = 2;
			int r = select(fd+1, &fds, NULL, NULL, &tv);

			if(-1 == r)
			{
				std::cout << "Waiting for Frame" << std::endl;
				return 1;
			}

			memset(&(buf), 0, sizeof(buf));
			buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
			buf.memory = V4L2_MEMORY_MMAP;

			if(-1 == xioctl(fd, VIDIOC_DQBUF, &buf))
			{
				std::cout << "Retrieving Frame" << std::endl;
				return 1;
			}

		}

		// 8. Store Image in OpenCV Data Type
		{
			memcpy(&camdata.data[0], buffer, 640 * 480 * 2);
			pub.publish(camdata);
			ROS_INFO("I published something!");
			//cv::cvtColor(bayerRaw, color, CV_BayerGB2BGR);
		}

		// 9. Display Image
		/*
		{
			FILE* fp = fopen("./camdatalinux.dat", "wb");
			fwrite(camdata, sizeof(camdata), 1, fp);
		 	fclose(fp);
		 	break;
		}
		*/

	}

	// 10. Turn off streaming.
	if (-1 == xioctl(fd, VIDIOC_STREAMOFF, &buf.type)) {
		std::cout << "VIDIOC_STREAMOFF" << std::endl;
	}

	return 0;
}
