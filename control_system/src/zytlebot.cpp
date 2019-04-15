#include <fstream>
#include <iostream>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/videoio/videoio.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/video/background_segm.hpp>
#include "unistd.h"
#include <math.h>
#include <stdio.h>
#include <opencv2/opencv.hpp>
#include <geometry_msgs/Twist.h>
#include <string>


#define DEBUG true

using namespace std;
using namespace cv;

class ControlSystem {
private:
    ros::NodeHandle nh_;
    ros::Subscriber image_sub_;
    ros::Subscriber red_pub_;

    ros::Publisher signal_search_;

public:
    geometry_msgs::Twist twist;

    // constructor
    ControlSystem(){
        // Subscribe image and callback function
        image_sub_ = nh_.subscribe("/image_raw", 1,
                                   &ControlSystem::imageCb, this);

        //  publish cmd_vel
        twist_pub = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);

        //geometry_msgs::Twist twist;
        twist.linear.x = 0.0;
        twist.linear.y = 0.0;
        twist.linear.z = 0.0;
        twist.angular.x = 0.0;
        twist.angular.y = 0.0;
        twist.angular.z = 0.0;
        limitedTwistPub();

        setSearchType();
    }

    // destructor
    ~ControlSystem() {
        twist.linear.x = 0.0;
        twist.angular.z = 0.0;
        twist_pub.publish(twist);
        cv::destroyAllWindows();
    }

    // callback function
    void imageCb(const std_msgs::ImageConstPtr &msg) {

        cv_bridge::CvImagePtr cv_ptr;
        try {
            // ROS msg format to OpenCV
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        }
        catch (cv_bridge::Exception &e) {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }
        cv::Mat base_image = cv_ptr->image;

        // ---------------Image processing----------

        // ---------------controller----------------
        if(twist.liner.x <= 0.20) {
            twist.liner.x += 0.01;
        }
        // ----------------Debug-------------------------
        if(DEBUG) {
            cout << "velocity : " << twist.linear.x << "angular : " << twist.angular.z << endl;
            imshow("input image", cv_ptr->image);
            waitKey(3);
        }
    }
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "control_system");
    ControlSystem cs;
    ros::spin();
    return 0;
}

