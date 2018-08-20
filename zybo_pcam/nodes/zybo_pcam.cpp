/*
before run this code, you should run the following commands.
sudo apt-get install v4l-utils
sudo media-ctl -d /dev/media0 -V '"ov5640 2-003c":0 [fmt:UYVY/'1920x1080'@1/'15' field:none]'
sudo media-ctl -d /dev/media0 -V '"43c60000.mipi_csi2_rx_subsystem":0 [fmt:UYVY/'1920x1080' field:none]'
*/
#include <string.h>
#include <fcntl.h>
#include <errno.h>
#include <sys/mman.h>
#include <sys/ioctl.h>
#include <iostream>
#include <fstream>
#include <linux/videodev2.h>

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


#define FMT_NUM_PLANES 3
#define WIDTH 1920
#define HEIGHT 1080

struct buffer_addr_struct{
	void *start[FMT_NUM_PLANES];
	size_t length[FMT_NUM_PLANES];
} *buffers;

static int xioctl(int fd, int request, void *arg){
	int r;
	do {
		r = ioctl (fd, request, arg);
		if (request == VIDIOC_DQBUF) {
			std::cout << "r : " << r << std::endl;
		}
	} while (-1 == r && EINTR == errno);
	return r;
}

int main(int argc, char **argv) {

	ros::init(argc, argv, "zyboPcam");
	ros::NodeHandle n;
	ros::Publisher pub = n.advertise<std_msgs::UInt8MultiArray>("image_array",  640 * 480 * 2);

	unsigned char *buffer;

	// 1. Open Video Device.
	int fd;
	fd = open("/dev/video0", O_RDWR, 0);
	if (fd == -1){
		std::cout << "Failed to open video device." << std::endl;
		return 1;
	}

	// 2. Querying video capabilities.
	struct v4l2_capability caps;
	memset(&caps, 0, sizeof(caps));
	if (-1 == xioctl(fd, VIDIOC_QUERYCAP, &caps)){
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

		fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
		fmt.fmt.pix_mp.width = WIDTH;
		fmt.fmt.pix_mp.height = HEIGHT;
		fmt.fmt.pix_mp.pixelformat = V4L2_PIX_FMT_YUYV;
		fmt.fmt.pix_mp.field = V4L2_FIELD_NONE;
		
		if (-1 == xioctl(fd, VIDIOC_S_FMT, &fmt)){
			std::cout << "Failed to set pixel format." << std::endl;
			return 1;
		}
	}

	int num_planes;
	struct v4l2_requestbuffers reqbuf;
	const int MAX_BUF_COUNT = 3;/*we want at least 3 buffers*/
	
	// 4. Request Buffer
	{
		memset(&(reqbuf), 0, sizeof(reqbuf));
		reqbuf.count = FMT_NUM_PLANES;
		reqbuf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
		reqbuf.memory = V4L2_MEMORY_MMAP;

		if (-1 == xioctl(fd, VIDIOC_REQBUFS, &reqbuf)){
			std::cout << "Failed to request buffer." << std::endl;
			return 1;
		}
		if (reqbuf.count < MAX_BUF_COUNT){
			std::cout << "Not enought buffer memory." << std::endl;
			return 1;
		}
		std::cout << "reqbuf.count : " << reqbuf.count << std::endl;

		buffers = (buffer_addr_struct*) calloc(reqbuf.count, sizeof(*buffers));
		assert(buffers != NULL);

	}

	// 5. Query Buffer
	{
		for(int i = 0; i < reqbuf.count; i++){ 

			struct  v4l2_plane planes[FMT_NUM_PLANES];
			struct  v4l2_buffer buf;
			memset(&(buf), 0, sizeof(buf));
			memset(planes, 0, sizeof(planes));
			buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
			buf.memory = V4L2_MEMORY_MMAP;
			buf.m.planes = planes;
			buf.length = FMT_NUM_PLANES;
			buf.index = i;
			if(-1 == xioctl(fd, VIDIOC_QUERYBUF, &buf)){
				std::cout << "Failed to query buffer." << std::endl;
				return 1;
			}
			num_planes = buf.length;
			std::cout << "buf.length : " << buf.length << std::endl;
			std::cout << "buf.m.offset : " << buf.m.offset << std::endl;

			for(int j = 0; j < num_planes; j++){
				buffers[i].length[j] = buf.m.planes[j].length;
				std::cout << "buf.m.planes[j].length : " << buf.m.planes[j].length << std::endl;
				buffers[i].start[j] = mmap(NULL, buf.m.planes[j].length, PROT_READ | PROT_WRITE, MAP_SHARED, fd, buf.m.planes[j].m.mem_offset);
				if(MAP_FAILED == buffers[i].start[j]){
					std::cout << "mmap error" << std::endl;
				}
				std::cout << "buffers[i].start[j] : " << buffers[i].start[j] << std::endl; 

			}
		}
	}

	// 6. Start Streaming
	{
		struct 	v4l2_buffer buf;
		memset(&(buf), 0, sizeof(buf));
		buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
		if(-1 == xioctl(fd, VIDIOC_STREAMON, &buf.type))
		{
			std::cout << "Fail to start Capture" << std::endl;
			return 1;
		}
	}

	std_msgs::UInt8MultiArray camdata;
	camdata.data = std::vector<uint8_t>(WIDTH*HEIGHT*2);

	struct 	v4l2_buffer buf;
	buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
	buf.memory = V4L2_MEMORY_MMAP;
	buf.index = 0;
	struct v4l2_plane planes[FMT_NUM_PLANES];
	buf.m.planes = planes;
	buf.length = FMT_NUM_PLANES;

	while (true) {

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

			if(-1 == r){
				std::cout << "Waiting for Frame" << std::endl;
				return 1;
			}

			if(-1 == xioctl(fd, VIDIOC_DQBUF, &buf)){
				std::cout << "Retrieving Frame" << std::endl;
				return 1;
			}

		}

		
		// 8. Store Image in OpenCV Data Type
		{
			for(int j = 0; j < num_planes; j++){
				memcpy(&camdata.data[0], buffers[0].start[j], WIDTH*HEIGHT*2);
				pub.publish(camdata);
				ROS_INFO("I published something!");
			}
		}

		// 9. Display Image
		/*
		{
			FILE* fp = fopen("./camdata.dat", "wb");
			fwrite(camdata, sizeof(camdata), 1, fp);
			fclose(fp);
		}
		*/

	} //while loop
	
	// 10. Turn off streaming.
	if (-1 == xioctl(fd, VIDIOC_STREAMOFF, &buf.type)) {
		std::cout << "VIDIOC_STREAMOFF" << std::endl;
	}

	// 11. Unmap memory.
	for(int i = 0; i < reqbuf.count; i++){
		for(int j = 0; j < num_planes; j++){
			munmap(buffers[i].start[j], buffers[i].length[j]);
		}
	}
	
	return 0;
}
