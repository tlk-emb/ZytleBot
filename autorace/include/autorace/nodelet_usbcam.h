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
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/UInt8MultiArray.h"
#include <opencv2/opencv.hpp>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>


#include <nodelet/nodelet.h>
#include <thread>

#include <fcntl.h>
#include <errno.h>
#include <sys/mman.h>
#include <sys/ioctl.h>
#include <iostream>
#include <stdio.h>
#include <string.h>


using namespace std;
#define REQUEST_BUFFER_NUM 10
#define WIDTH 640
#define HEIGHT 480



namespace autorace {
    class NodeletUsbcam : public nodelet::Nodelet {

    private:
        unsigned char *buffers[REQUEST_BUFFER_NUM];

        ros::NodeHandle n;
        int fd;
        struct v4l2_capability caps;
        int got_buffer_num;

        XmlRpc::XmlRpcValue params;
        int usbcam_frame;

        std::chrono::system_clock::time_point  t1, t2, t3, t4, t5, t6, t7;

        std_msgs::UInt8MultiArrayPtr camdata;
        struct 	v4l2_buffer buf;

        ros::Timer image_pub_;

        ros::Publisher pub;

        bool CbFlag;

    public:
        // コンストラクタ
        NodeletUsbcam() {
        }

        ~NodeletUsbcam() {
            buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
            buf.memory = V4L2_MEMORY_MMAP;
            if (-1 == xioctl(fd, VIDIOC_STREAMOFF, &buf.type)) {
                std::cout << "VIDIOC_STREAMOFF" << std::endl;
            }

            for(int i = 0; i < got_buffer_num; i++){
                if(munmap(buffers[i], 480 * 640 * 2) != 0){
                    cerr << "munmap failed" << endl;
                }else{
                    cout << "munmap success" << endl;
                }
            }
            close(fd);
        }

        void onInit() {
            n = getNodeHandle();
            pub = n.advertise<std_msgs::UInt8MultiArray>("/usbcam/image_array",  640 * 480 * 2);


            n.getParam("/nodelet_autorace/autorace", params);
            usbcam_frame = (int)params["usbcam_frame"];

            setInit();

            image_pub_ = n.createTimer(ros::Duration(1.0/usbcam_frame), boost::bind(&NodeletUsbcam::imageCb, this, _1));
        }

        void imageCb(const ros::TimerEvent& event) {
            t1 = std::chrono::system_clock::now();
            // 7. Capture Image
            // 8. Store Image in OpenCV Data Type
            if (CbFlag) {
                cout << "usb Cb fail" << endl;
            } else {
                CbFlag = true;
                cout << "usb Cb start" << endl;
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
                        return;
                    }

                    memset(&(buf), 0, sizeof(buf));
                    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
                    buf.memory = V4L2_MEMORY_MMAP;

                    if(-1 == xioctl(fd, VIDIOC_DQBUF, &buf))
                    {
                        std::cout << "USB Retrieving Frame" << std::endl;
                        reset();
                        return;
                    }

                    memcpy(&(camdata->data[0]), buffers[buf.index], 640 * 480 * 2);
                    pub.publish(camdata);

                    // Connect buffer to queue for next capture.
                    if (-1 == xioctl(fd, VIDIOC_QBUF, &buf)) {
                        std::cout << "VIDIOC_QBUF" << std::endl;
                    }

                }
                CbFlag = false;
                cout << "CbFlag end" << endl;
            }

            t2 = std::chrono::system_clock::now();
        }

        void reset() {
            buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
            buf.memory = V4L2_MEMORY_MMAP;
            if (-1 == xioctl(fd, VIDIOC_STREAMOFF, &buf.type)) {
                std::cout << "VIDIOC_STREAMOFF" << std::endl;
            }

            for(int i = 0; i < got_buffer_num; i++){
                if(munmap(buffers[i], 480 * 640 * 2) != 0){
                    cerr << "munmap failed" << endl;
                }else{
                    cout << "munmap success" << endl;
                }
            }
            close(fd);

            cout << "restart  USB!!! " << endl;
            setInit();
        }

        void setInit() {
            CbFlag = false;
            fd = open("/dev/video1", O_RDWR, 0);
            if (fd == -1)
            {
                std::cout << "Failed to open video device." << std::endl;
                return;
            }

            // 2. Querying video capabilities.
            memset(&caps, 0, sizeof(caps));
            if (-1 == xioctl(fd, VIDIOC_QUERYCAP, &caps))
            {
                std::cout << "Failed to query capabilities." << std::endl;
                return;
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
                    return;
                }
            }

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
                    return;
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
                        return;
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

            cout << "5.5 end" << endl;
            // 6. Start Streaming
            {
                struct 	v4l2_buffer buf;
                memset(&(buf), 0, sizeof(buf));
                buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
                if(-1 == xioctl(fd, VIDIOC_STREAMON, &buf.type))
                {
                    std::cout << "Start Capture" << std::endl;
                    return;
                }
            }



            std::vector<uint8_t> vec = std::vector<uint8_t>(WIDTH*HEIGHT*2);

            std_msgs::UInt8MultiArrayPtr camdatatemp(new std_msgs::UInt8MultiArray);
            camdatatemp->data = vec;

            camdata = camdatatemp;
        }
    };
}
