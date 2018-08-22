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
  int Hue_l, Hue_h, Saturation_l, Saturation_h, Lightness_l, Lightness_h;
  
public:
  // コンストラクタ
  ImageConverter()
    : it_(nh_)
  {
    Hue_l = 0;
    Hue_h = 180;
    Saturation_l = 0;
    Saturation_h = 45;
    Lightness_l = 180;
    Lightness_h = 255;
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
    // cv::cvtColor(cv_ptr->image, hsv_image, CV_BGR2HSV);

    // // 色相(Hue), 彩度(Saturation), 明暗(Value, brightness)
    // 指定した範囲の色でマスク画像color_mask(CV_8U:符号なし8ビット整数)を生成 
    // マスク画像は指定した範囲の色に該当する要素は255(8ビットすべて1)、それ以外は0 
    // 白を検出する       
    
    // cv::inRange(hsv_image, cv::Scalar(Hue_l,Saturation_l,Lightness_l, 0) , cv::Scalar(Hue_h,Saturation_h,Lightness_h, 0), color_mask);
    // cv::bitwise_and(cv_ptr->image, cv_ptr->image, cv_image2, color_mask);
    cv_image2 = whiteDetect(cv_ptr->image);
    // エッジを検出するためにCannyアルゴリズムを適用
    cv::Canny(cv_ptr3->image, cv_ptr3->image, 15.0, 30.0, 3);

    // ウインドウに円を描画。中心(100, 100), 半径20[pixel]、色緑
    cv::circle(cv_ptr->image, cv::Point(100, 100), 20, CV_RGB(0,255,0));

    // ハフ変換

/*
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
*/


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

  cv::Mat whiteDetect(cv::Mat image)
  {
    cv::Mat color_mask, result_image, hsv_image, gray_image;
    // 左半分をくりぬく
    cv::Rect roi(cv::Point(0, 0), cv::Size(image.size().width / 2, image.size().height));
    cv::Mat right_image = image(roi);

    cv::cvtColor(right_image, hsv_image, CV_BGR2HSV);

    cv::inRange(hsv_image, cv::Scalar(Hue_l,Saturation_l,Lightness_l, 0) , cv::Scalar(Hue_h,Saturation_h,Lightness_h, 0), color_mask);
    cv::bitwise_and(right_image, right_image, result_image, color_mask);  
    

/*
    // nonzeroをカウント
    int fraction_num = cv::countNonZero(color_mask);
    
    //　光度の影響を調整
    if (fraction_num > 35000) {
      if (Lightness_l < 255) Lightness_l += 5;
    } else if (fraction_num < 5000) {
      if (Lightness_l > 50) Lightness_l -= 5;
    }
*/
    // 白線の重心を決定
    cv::cvtColor(result_image, gray_image,CV_RGB2GRAY);
    cv::Moments mu = cv::moments( gray_image, true );
    cv::Point2f mc = cv::Point2f( mu.m10/mu.m00 , mu.m01/mu.m00 );
    cv::circle( result_image, mc, 30, cv::Scalar(0, 0, 255), -1, 4);

    return result_image;

  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_converter");
  ImageConverter ic;
  ros::spin();
  return 0;
}
