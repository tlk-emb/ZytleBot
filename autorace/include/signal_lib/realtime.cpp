#include <iostream>
#include <chrono>
#include <vector>

#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>

#include "feature.h"
#include "forest.h"
#include "window_candidate.h"
#include "hw.h"
using namespace std;
using namespace cv;

#define FEATURE_SIZE 64*3*2+12*3*2+72*4
#define hwmode true

float test_one_image(Mat img){
  std::chrono::system_clock::time_point  t1, t2, t3, t4, t5, t6, t7;

  Mat resized_img, gray;
  cv::resize(img, resized_img, cv::Size(64 ,32), INTER_LINEAR);
  cv::Mat resized_hls;
  cv::cvtColor(resized_img, resized_hls, CV_BGR2HSV);

  cv::Size spatial_size(8, 8);
  Mat spatial_rgb, spatial_hls;
  cv::resize(resized_img, spatial_rgb, spatial_size, INTER_LINEAR);
  cv::resize(resized_hls, spatial_hls, spatial_size, INTER_LINEAR);
  
  unsigned short feature[744] = {0};
  memset(feature, 0, sizeof(unsigned short) * FEATURE_SIZE);

  ravel(spatial_hls, feature);
  ravel(spatial_rgb, feature + 192);

  if(hwmode){
    t1 = std::chrono::system_clock::now();
    //create HSV histogram
    for(int i = 0; i < 32; i++)  memcpy(hist_imageBuffer + 64 * i * 3, resized_hls.data + resized_hls.step * i, resized_hls.cols * 3 * sizeof(unsigned char));
    color_hist_hw(feature + 192 * 2);
    //create RGB histogram
    for(int i = 0; i < 32; i++)  memcpy(hist_imageBuffer + 64 * i * 3, resized_img.data + resized_img.step * i, resized_img.cols * 3 * sizeof(unsigned char));
    color_hist_hw(feature + 192 * 2 + 36);
    //calculate HOG feature
    t2 = std::chrono::system_clock::now();
    cv::cvtColor(resized_img, gray, CV_BGR2GRAY);
    for(int i = 0; i < 32; i++)  memcpy(hog_imageBuffer + 64 * i, gray.data + gray.step * i, gray.cols * sizeof(unsigned char));
    calc_hog_hw(feature + 192 * 2 + 36 * 2);
    t3 = std::chrono::system_clock::now();
  }else{
    hist(resized_hls, feature + 192 * 2);
    hist(resized_img, feature + 192 * 2 + 36);
    cv::cvtColor(resized_img, gray, CV_BGR2GRAY);
    lite_hog(gray, feature + 192 * 2 + 36 * 2);
  }

  float proba = randomforest_classifier(feature);
  if(proba >= 0.65) cout << proba << endl;
  return proba;
}

int main(int argc, const char* argv[])
{
  cv::VideoCapture cap(1);
  if(!cap.isOpened()){
    cout << "failed" << endl;
    return -1;
  }
  if(hwmode) hw_setup();
  cout << "hw setup completed" << endl;
  std::chrono::system_clock::time_point  t1, t2, t3, t4, t5, t6, t7;
  while(1){
      t1 = std::chrono::system_clock::now();
      cv::Mat frame;
      cap >> frame; // get a new frame from camera
      cv::Mat frame_copy = frame.clone();

      //prepare for removing arienai
      cv::Mat hls;
      cv::cvtColor(frame, hls, CV_RGB2HLS);
      int d[480][640];
      int sum = 0;
      for(int y = 0; y < frame.rows; y++){
        int tmpsum = 0;
        for(int x = 0; x < frame.cols; x++){
          cv::Vec<unsigned char, 3> pix = hls.ptr<cv::Vec3b>(y)[x];
          if((pix[0] < 20 || pix[0] > 200) && pix[1] > 128){
            tmpsum++;
          }
          if(y == 0) d[y][x] = tmpsum;
          else d[y][x] = d[y-1][x] + tmpsum;
        }
      }
      for(int i = 0; i < window_num; i++){
        cv::Mat out;
        int sy = w[i][0][0];
        int sx = w[i][1][0];
        int ey = w[i][0][1];
        int ex = w[i][1][1];
        //remove arienai
        int satisfy_num = d[ey-1][ex-1] - d[ey-1][sx-1] - d[sy-1][ex-1] + d[sy-1][sx-1];
        // if(satisfy_num < (ey - sy) * (ex - sx) * 2 / 100) continue;
        // cout << "satisfy num" << satisfy_num << endl;

        cv::Mat cropped(frame, cv::Rect(sx, sy, ex - sx, ey - sy));
        // cv::imwrite("crop.png", cropped);
        float proba = test_one_image(cropped);

        if(proba >= 0.65) rectangle(frame_copy, Point(sx, sy), Point(ex, ey), Scalar(0,0,200), 3); //x,y //Scaler = B,G,R
      }
      //cv::imshow("window", frame_copy);

      int key = cv::waitKey(1);
      if(key == 113)//qボタンが押されたとき
      {
          break;//whileループから抜ける．
      }
      t2 = std::chrono::system_clock::now();
      //show fps
      double elapsed = (double)std::chrono::duration_cast<std::chrono::milliseconds>(t2-t1).count();
      cout << "elapsed:" << elapsed << "[milisec]" << endl;
      cout << "fps:" << 1000.0/elapsed << "[fps]" << endl;
  }
  //cv::destroyAllWindows();
  if(hwmode) hw_release();
  return 0;
}
