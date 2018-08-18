#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>


class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;
  
public:
  // コンストラクタ
  ImageConverter()
    : it_(nh_)
  {
    // カラー画像をサブスクライブ
    image_sub_ = it_.subscribe("/camera/rgb/image_raw", 1, 
      &ImageConverter::imageCb, this);
    //  処理した画像をパブリッシュ  
    image_pub_ = it_.advertise("/image_topic", 1);
 }

  // デストラクタ
  ~ImageConverter()
  {
    // 全てのウインドウは破壊
    cv::destroyAllWindows();
  }

  // コールバック関数
  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    cv_bridge::CvImagePtr cv_ptr, cv_ptr2, cv_ptr3;
    try
    {
      // ROSからOpenCVの形式にtoCvCopy()で変換。cv_ptr->imageがcv::Matフォーマット。
      cv_ptr    = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
      cv_ptr3   = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    cv::Mat hsv_image, color_mask, gray_image, cv_image2, cv_image3;
    // RGB表色系をHSV表色系へ変換して、hsv_imageに格納
    cv::cvtColor(cv_ptr->image, hsv_image, CV_BGR2HSV);

    // // 色相(Hue), 彩度(Saturation), 明暗(Value, brightness)
    // 指定した範囲の色でマスク画像color_mask(CV_8U:符号なし8ビット整数)を生成 
    // マスク画像は指定した範囲の色に該当する要素は255(8ビットすべて1)、それ以外は0 
    // 白を検出する       
    cv::inRange(hsv_image, cv::Scalar(0,0,180, 0) , cv::Scalar(180,45,255, 0), color_mask);
    cv::bitwise_and(cv_ptr->image, cv_ptr->image, cv_image2, color_mask);
    // エッジを検出するためにCannyアルゴリズムを適用
    cv::Canny(cv_ptr3->image, cv_ptr3->image, 15.0, 30.0, 3);

    // ウインドウに円を描画。中心(100, 100), 半径20[pixel]、色緑
    cv::circle(cv_ptr->image, cv::Point(100, 100), 20, CV_RGB(0,255,0));

    // ハフ変換

    cv::Mat dst, color_dst;
    cv::Canny( cv_image2, dst, 50, 200, 3);
    cv::cvtColor( dst, color_dst, CV_GRAY2BGR );
    
    std::vector<cv::Vec4i> lines;
    cv::HoughLinesP( dst, lines, 1, CV_PI/180, 100, 500, 200);

    for( size_t i = 0; i < lines.size(); i++ )
    {
        cv::line( cv_image2, cv::Point(lines[i][0], lines[i][1]),
            cv::Point(lines[i][2], lines[i][3]), cv::Scalar(0,0,255), 3, 8 );
    }



    // 画像サイズを縦横半分に変更
    cv::Mat cv_half_image, cv_half_image2, cv_half_image3;
    cv::resize(cv_ptr->image, cv_half_image,cv::Size(),0.25,0.25);
    cv::resize(cv_image2, cv_half_image2,cv::Size(),0.25,0.25);
    cv::resize(cv_ptr3->image, cv_half_image3,cv::Size(),0.25,0.25);

    // ウインドウ表示
    cv::imshow("Original Image", cv_half_image);
    cv::imshow("Result Image", cv_half_image3);
    cv::imshow("Line Image", cv_half_image2);
    cv::waitKey(3);
    
    //エッジ画像をパブリッシュ。OpenCVからROS形式にtoImageMsg()で変換。
    image_pub_.publish(cv_ptr3->toImageMsg());
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_converter");
  ImageConverter ic;
  ros::spin();
  return 0;
}