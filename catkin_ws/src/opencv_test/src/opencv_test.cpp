#include <iostream>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "unistd.h"

#include <geometry_msgs/Twist.h>

#define PI 3.141592653589793
#define BIRDSEYE_LENGTH 100
#define BURGER_MAX_LIN_VEL 0.22
#define BURGER_MAX_ANG_VEL 2.84


// 直線を中点、傾き、長さで表す
typedef struct straight {
  cv::Point2f middle;
  float degree;
  float length;
} STRAIGHT;

class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;
  int Hue_l, Hue_h, Saturation_l, Saturation_h, Lightness_l, Lightness_h;
  geometry_msgs::Twist twist;

  ros::Publisher twist_pub;
  ros::Timer timer;
  // cv::Mat curve_image;
  int line_lost_cnt;

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
    line_lost_cnt = 0;

    //curve_image = cv::imread("src/cv_bridge_tutorial/src/curve2.png");

    // カラー画像をサブスクライブ
    image_sub_ = it_.subscribe("/camera/rgb/image_raw", 1, 
      &ImageConverter::imageCb, this);
    //  処理した挙動をパブリッシュ  
    //twist_pub = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);
    twist_pub = nh_.advertise<geometry_msgs::Twist>("/cmd_vel2", 1000);
    // 0.1秒ごとに制御を呼び出す
    //timer = nh.createTimer(ros::Duration(0.1), &ImageConverter::timerCallback, this);

    //image_pub_ = it_.advertise("/image_topic", 1);

    // twist初期化
    //geometry_msgs::Twist twist;
    twist.linear.x = 0.0;
    twist.linear.y = 0.0;
    twist.linear.z = 0.0;
    twist.angular.x = 0.0;
    twist.angular.y = 0.0;
    twist.angular.z = 0.0;
    twist_pub.publish(twist);
 }

  // デストラクタ
  ~ImageConverter()
  {
    // 全てのウインドウは破壊
    cv::destroyAllWindows();
  }

  // ライン検出の結果によって左右に操作
  //　ラインがあればtrue
  // intは+1で左, 0で直進, -1で右
  // ラインが見つからなければ左に回転
  void controler(int vel, int dir){

    twist.linear.x += 0.01 * vel;
    twist.angular.z = 0.1 * dir;

    if (twist.linear.x > BURGER_MAX_LIN_VEL) twist.linear.x = BURGER_MAX_LIN_VEL;
    if (twist.linear.x < 0) twist.linear.x = 0;
    if (twist.angular.z > BURGER_MAX_ANG_VEL) twist.angular.z = BURGER_MAX_ANG_VEL;
    if ((twist.angular.z < 0 - BURGER_MAX_ANG_VEL)) twist.angular.z = 0 - BURGER_MAX_ANG_VEL;


    
    std::cout << twist.linear.x << std::endl;
    std::cout << twist.angular.z << std::endl;
    
    twist_pub.publish(twist);

  }

  void curveStart() {
    twist.linear.x = 0.2;
    twist.angular.z = -0.2;

    twist_pub.publish(twist);

    sleep(8);
    std::cout << "end curve!!!" << std::endl;

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


    // 俯瞰画像

    cv_image2 = birdsEye(cv_ptr->image);
    cv_image3 = whiteBinary(cv_image2);

    // 俯瞰画像の左半分だけのROI
    cv::Mat left_roi(cv_image3, cv::Rect(0, 0, BIRDSEYE_LENGTH / 2 , BIRDSEYE_LENGTH));


    // ハフ変換
    cv::Mat dst, color_dst;
    cv::Canny( left_roi, dst, 50, 200, 3);
    cv::cvtColor( dst, color_dst, CV_GRAY2BGR );



    std::vector<cv::Vec4i> lines;
    cv::HoughLinesP( dst, lines, 1, CV_PI/180, 20, 40, 25);


    // 角度平均をとる
    int average_cnt = 0;
    float degree_average_sum = 0;
    float most_left_middle_x = BIRDSEYE_LENGTH * 0.5;

    // 垂直に近い点のみ線を引く
    for( size_t i = 0; i < lines.size(); i++ )
    {
      STRAIGHT line = toStraightStruct(lines[i]);
      if (line.degree < 20 && line.degree > -20) {
        //赤線を引く 
        cv::line( left_roi, cv::Point(lines[i][0], lines[i][1]),
          cv::Point(lines[i][2], lines[i][3]), cv::Scalar(0,0,255), 3, 8 );
        
        degree_average_sum += line.degree;
        if (most_left_middle_x > line.middle.x) most_left_middle_x = line.middle.x;
        average_cnt++;
      }
        // デバッグ用
    }
    if( average_cnt != 0) {
      float degree_average = degree_average_sum / average_cnt;
      std::cout << most_left_middle_x << std::endl;
      // 角度平均が-5以上なら左に曲がる、5以上なら右に曲がる
      if (degree_average < -10) {
        controler(0, degree_average * -0.2);
      } else if(degree_average > 10) {
        controler (0, degree_average * -0.2);

      // 中点が右過ぎたら左に、左過ぎたら右に曲がる
      } else if (most_left_middle_x > BIRDSEYE_LENGTH * 0.25) {
        controler (1, -1);
      } else if (most_left_middle_x < BIRDSEYE_LENGTH * 0.05) {
        controler (1, 1);
      } else {
        controler (1, 0);
      }

      line_lost_cnt = 0;
  }

    // 画像サイズを縦横半分に変更
    cv::Mat cv_half_image, cv_half_image2, cv_half_image3, cv_half_image4;
    cv::resize(cv_ptr->image, cv_half_image,cv::Size(),0.25,0.25);
    cv::resize(cv_image2, cv_half_image2,cv::Size(),4,4);
    cv::resize(cv_image3, cv_half_image3,cv::Size(),4,4);
    cv::resize(left_roi, cv_half_image4,cv::Size(),4,4);

    // ウインドウ表示
    cv::imshow("Original Image", cv_half_image);
    cv::imshow("Result Image", cv_half_image3);
    cv::imshow("ROI", cv_half_image2);
    cv::imshow("LEFT ROI", cv_half_image4);
    cv::waitKey(3);

    if( average_cnt == 0) {
    // ラインが見つからなかったらカーブを疑う
    controler(-1, 0);
    line_lost_cnt++;
    std::cout << "line lost" << line_lost_cnt << std::endl;

    if (line_lost_cnt > 10) {
      curveStart();
      line_lost_cnt = 0;
    }
  }
    
    //エッジ画像をパブリッシュ。OpenCVからROS形式にtoImageMsg()で変換。
    image_pub_.publish(cv_ptr3->toImageMsg());
    }

  // curve.pngと類似度を得る
    /*
  void curveMatch(cv::Mat image) {

    int imageCount = 1; // 入力画像の枚数
    int channelsToUse[] = { 0 }; // 0番目のチャネルを使う
    int dimention = 1; // ヒストグラムの次元数
    int binCount = 256; // ヒストグラムのビンの数
    int binCounts[] = { binCount };
    float range[] = { 0, 256 }; // データの範囲は0～255
    const float* histRange[] = { range };
    cv::Mat histogram1;
    cv::calcHist(&image, imageCount, channelsToUse, cv::Mat(), histogram1, dimention, binCounts, histRange);
    cv::Mat histogram2;
    cv::calcHist(&curve_image, imageCount, channelsToUse, cv::Mat(), histogram2, dimention, binCounts, histRange);

    // 類似度を調べる（同じ画像を読み込んだため1が出力される）
    double correlation = compareHist(histogram1, histogram2, CV_COMP_CORREL);
    std::cout << correlation << std::endl;

    if (correlation > 0.9999) {
      std::cout << "curve detect!" << std::endl;
      curveStart();
    }
    */

/*
    cv::Mat result;
    double minVal, maxVal;
    cv::matchTemplate(image, curve_image, result, 5);
    cv::minMaxLoc(result, &minVal, &maxVal);
    std::cout << minVal << "   " << maxVal << std::endl;
  }
*/
  // imageを渡して俯瞰画像を得る
  cv::Mat birdsEye(cv::Mat image)
  {
    int width = image.size().width;
    int height = image.size().height;
    // 奥行の広さ（小さいほど狭い）
    float width_ratio = 0.17;
    // 上部
    float height_h = 0.592;
    // 下部
    float height_l = 0.79;
    // 画素値
    int result_size = BIRDSEYE_LENGTH;
    cv::Mat map_matrix, dst_image;
    cv::Point2f src_pnt[4], dst_pnt[4];

    src_pnt[0] = cv::Point (width * (0.5 - width_ratio) , height * height_h);
    src_pnt[1] = cv::Point (0, height * height_l);
    src_pnt[2] = cv::Point (width * (0.5 + width_ratio), height * height_h);
    src_pnt[3] = cv::Point (width, height * height_l);

    dst_pnt[0] = cv::Point (0, 0);
    dst_pnt[1] = cv::Point (0, result_size);
    dst_pnt[2] = cv::Point (result_size, 0);
    dst_pnt[3] = cv::Point (result_size, result_size);

    map_matrix = cv::getPerspectiveTransform (src_pnt, dst_pnt);
    cv::warpPerspective (image, dst_image, map_matrix, cv::Size(result_size, result_size));
    return dst_image;
  }


// 二点をSTRAIGHT構造体で返す
STRAIGHT toStraightStruct(cv::Vec4i line)
{
  STRAIGHT result;

  //中点
  result.middle = cv::Point ((line[0] + line[2]) / 2, (line[1] + line[3]) / 2);
  // 距離
  result.length = (line[0] - line[2]) *  (line[0] - line[2]) + (line[1] - line[3]) *  (line[1] - line[3]);
  // 角度
  float radian = atan2(line[1] - line[3], line[0] - line[2]);
  // radianからdegree

  if ( radian * 180.0 / PI >= 90 ) {
    result.degree = radian * 180.0 / PI - 90 ;
  } else {
    result.degree = radian * 180.0 / PI + 90;
  }
  return result;
}


// 二点間の傾きを求め、長さをかけて重さとする
// x1 y1, x2 y2
float lineWeight(cv::Vec4i line)
{
  // 距離
  float distance = (line[0] - line[2]) *  (line[0] - line[2]) + (line[1] - line[3]) *  (line[1] - line[3]);

  // 角度
  float radian = atan2(line[1] - line[3], line[0] - line[2]);

  // radianからdegree
  float degree = radian * 180.0 / PI;

  return degree;
}


// 白色検出（返り値はRGB）
  cv::Mat whiteBinary(cv::Mat image)
  {
    cv::Mat color_mask, result_image, hsv_image;
    cv::cvtColor(image, hsv_image, CV_BGR2HSV);
    cv::inRange(hsv_image, cv::Scalar(Hue_l,Saturation_l,Lightness_l, 0) , cv::Scalar(Hue_h,Saturation_h,Lightness_h, 0), color_mask);
    cv::bitwise_and(image, image, result_image, color_mask);

    return result_image;
  }

  cv::Mat whiteLineDetect(cv::Mat image)
  {
    cv::Mat color_mask, result_image, hsv_image;
    cv::cvtColor(image, hsv_image, CV_BGR2HSV);
    cv::inRange(hsv_image, cv::Scalar(Hue_l,Saturation_l,Lightness_l, 0) , cv::Scalar(Hue_h,Saturation_h,Lightness_h, 0), color_mask);
    cv::bitwise_and(image, image, result_image, color_mask);

    return result_image;
  }

  cv::Mat whiteDetect(cv::Mat image)
  {
    /*
    cv::resize(image, image,cv::Size(),0.25,0.25);
    cv::Mat color_mask, result_image, hsv_image;
    // 左半分をくりぬく
    cv::Rect roi(cv::Point(0, 0), cv::Size(image.size().width / 2, image.size().height));
    cv::Mat right_image = image(roi);

    //白色検出
    cv::cvtColor(right_image, hsv_image, CV_BGR2HSV);
    cv::inRange(hsv_image, cv::Scalar(Hue_l,Saturation_l,Lightness_l, 0) , cv::Scalar(Hue_h,Saturation_h,Lightness_h, 0), color_mask);
    cv::bitwise_and(right_image, right_image, result_image, color_mask);  

    // nonzeroをカウント
    int fraction_num = cv::countNonZero(color_mask);
    
    //　光度の影響を調整
    if (fraction_num > 35000) {
      if (Lightness_l < 255) Lightness_l += 5;
    } else if (fraction_num < 5000) {
      if (Lightness_l > 50) Lightness_l -= 5;
    }

    // 白線の重心を決定
    cv::Moments mu = cv::moments( result_image, true );
    cv::Point2f mc = cv::Point2f( mu.m10/mu.m00 , mu.m01/mu.m00 );
    cv::circle( result_image, mc, 4, cv::Scalar(100), 2, 4);

    return result_image;
    */
    return image;

  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_converter");
  ImageConverter ic;
  ros::spin();
  return 0;
}
