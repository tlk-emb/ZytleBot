#include <fcntl.h>
#include <errno.h>
#include <sys/mman.h>
#include <sys/ioctl.h>
#include <iostream>
#include <stdio.h>
#include <string.h>
#include <iostream>
#include <chrono>

#include <linux/videodev2.h>
// #include <opencv2/opencv.hpp>
// #include <opencv2/imgproc/imgproc.hpp>
// #include <opencv2/highgui/highgui.hpp>

using namespace std;
#define REQUEST_BUFFER_NUM 10

static int xioctl(int fd, int request, void *arg)
{
    int r;
	do {
		r = ioctl (fd, request, arg);
		if (request == VIDIOC_DQBUF) {
			std::cout << "r : " << r << std::endl;
		}
		if(errno == EAGAIN){
			cout << "EAGAIN" << endl;
		}else if(errno == EINVAL){
			cout << "EINVAL" << endl;
		}else if(errno == EIO){
			cout << "EIO" << endl;
		}else if(errno == EPIPE){
			cout << "EPIPE" << endl;
		}else if(errno == EACCES){
			cout << "EACCESS" << endl;
		}else if(errno == EBUSY){
			cout << "EBUSY" << endl;
		}else{
			cout << "other error" << endl;	
		}
	} while (-1 == r && EINTR == errno);
	if(r == -1){
		cout << "loop breaked and error occurred" << endl;
		if(errno == EAGAIN){
			cout << "EAGAIN" << endl;
		}else if(errno == EINVAL){
			cout << "EINVAL" << endl;
		}else if(errno == EIO){
			cout << "EIO" << endl;
		}else if(errno == EPIPE){
			cout << "EPIPE" << endl;
		}else if(errno == EACCES){
			cout << "EACCESS" << endl;
		}else if(errno == EBUSY){
			cout << "EBUSY" << endl;
		}else{
			cout << "other error" << endl;	
		}
	}
	return r;
}
unsigned char *buffers[4];

int main() {

	// 1. Open Video Device.
	int fd;
	fd = open("/dev/video1", O_RDWR, 0);
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
		fmt.fmt.pix.field = V4L2_FIELD_NONE;

		if (-1 == xioctl(fd, VIDIOC_S_FMT, &fmt))
		{
			std::cout << "Failed to set pixel format." << std::endl;
			return 1;
		}
	}

	int got_buffer_num;
	// 4. Request Buffer
	{
		struct v4l2_requestbuffers req;
		memset(&(req), 0, sizeof(req));
		req.count = REQUEST_BUFFER_NUM;
		req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		req.memory = V4L2_MEMORY_MMAP;

		if (-1 == xioctl(fd, VIDIOC_REQBUFS, &req))
		{
			std::cout << "Failed to request buffer." << std::endl;
			return 1;
		}
		cout << "we could get " << req.count  << " buffers. " << endl;
		got_buffer_num = req.count;
	}

	// 5. Query Buffer
	{
		for(int bufferindex = 0; bufferindex < got_buffer_num; bufferindex++){
			struct 	v4l2_buffer buf;
			memset(&(buf), 0, sizeof(buf));
			buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
			buf.memory = V4L2_MEMORY_MMAP;
			buf.index = bufferindex;
			if(-1 == xioctl(fd, VIDIOC_QUERYBUF, &buf))
			{
				std::cout << "Failed to query buffer." << std::endl;
				return 1;
			}

			std::cout << "buf.length : " << buf.length << std::endl;
			std::cout << "buf.m.offset : " << buf.m.offset << std::endl;

			buffers[bufferindex] = (unsigned char*)mmap (NULL, buf.length, PROT_READ | PROT_WRITE, MAP_SHARED, fd, buf.m.offset);
			if (MAP_FAILED == buffers[bufferindex])
				cerr << "mmap" << endl;
		}
	}

	
	// 5.5 QBUF Request
	{
		for (int i = 0; i < got_buffer_num; ++i) {
			struct v4l2_buffer buf;
			buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
			buf.memory = V4L2_MEMORY_MMAP;
			buf.index = i;
			if (-1 == xioctl(fd, VIDIOC_QBUF, &buf))
				cerr << "VIDIOC_QBUF" << endl;
		}
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

	char camdata[640*480*2];
	std::chrono::system_clock::time_point  t1, t2, t3, t4, t5, t6, t7;

	for(int i = 0; i < 100; i++){
		t1 = std::chrono::system_clock::now();
		// 7. Capture Image
		// 8. Store Image in OpenCV Data Type
		{
			{
				struct v4l2_buffer buf;
				buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
				buf.memory = V4L2_MEMORY_MMAP;
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
		
				memset(&(buf), 0, sizeof(buf));
				buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
				buf.memory = V4L2_MEMORY_MMAP;

				if(-1 == xioctl(fd, VIDIOC_DQBUF, &buf))
				{
					std::cout << "Retrieving Frame" << std::endl;
					return 1;
				}

				cout << "buf.index : " << buf.index << endl;
			
				// Connect buffer to queue for next capture.
				if (-1 == xioctl(fd, VIDIOC_QBUF, &buf)) {
					std::cout << "VIDIOC_QBUF" << std::endl;
				}
				memcpy(camdata, buffers[buf.index], 640 * 480 * 2);
			}

		}
		
		t2 = std::chrono::system_clock::now();
		// 9. Convert YUYV to BGR and Display Image
		{
			// cv::Mat rawimg(480, 640, CV_8UC2);
			// cv::Mat dstimg(480, 640, CV_8UC2);
			// memcpy(rawimg.data, camdata, 640 * 480 * 2);
			// cv::cvtColor(rawimg, dstimg, cv::COLOR_YUV2BGR_YUYV);
			
			// cv::imshow("image_test", dstimg);
			// cv::waitKey(1);

		}
		//show fps
        double elapsed = (double)std::chrono::duration_cast<std::chrono::milliseconds>(t2-t1).count();
        cout << "elapsed:" << elapsed << "[milisec]" << endl;
        cout << "fps:" << 1000.0/elapsed << "[fps]" << endl;

	}

	// 10. Turn off streaming.
	struct v4l2_buffer buf;
	buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	buf.memory = V4L2_MEMORY_MMAP;
	if (-1 == xioctl(fd, VIDIOC_STREAMOFF, &buf.type)) {
		std::cout << "VIDIOC_STREAMOFF" << std::endl;
	}

	return 0;
}