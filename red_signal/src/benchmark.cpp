#include <iostream>
#include <chrono>
#include <map>
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

map<int, vector<pair<int, int> > > mp;
set<int> widthkind;

int sx_min = 999;
int sy_min = 999;
int ex_max = -1;
int ey_max = -1;
#define WINDOW_WIDTH 64
#define WINDOW_HEIGHT 32
bool imgout = false;
float proba_thresh = 0.65;
#define hwmode true
#define checkmode false
#define FEATURE_SIZE 64*3*2+12*3*2+72*4

void check_window(){
    for(int i = 0; i < window_num; i++){
        int sy = w[i][0][0];
        int sx = w[i][1][0];
        int ey = w[i][0][1];
        int ex = w[i][1][1];
        // cv::Mat cropped(frame, cv::Rect(sx, sy, ex - sx, ey - sy));
        widthkind.insert(ex-sx);
        sx_min = min(sx_min, sx);
        sy_min = min(sy_min, sy);
        ex_max = max(ex_max, ex);
        ey_max = max(ey_max, ey);
    }

    for(int i = 0; i < window_num; i++){
    //(old_x - sx_min) * ratio
        int sy = w[i][0][0];
        int sx = w[i][1][0];
        int ey = w[i][0][1];
        int ex = w[i][1][1];

        int original_width = ex - sx;
        sy = (int)((float) (sy - sy_min) * (float)WINDOW_WIDTH/original_width);
        sx = (int)((float) (sx - sx_min) * (float)WINDOW_WIDTH/original_width);
        mp[original_width].push_back(make_pair(sx, sy));
    }
}

bool inputdatacheck(unsigned char* ptr, cv::Mat img){
    bool rst = true;
    for(int y = 0; y < img.rows; y++){
        for(int x = 0; x < img.cols; x++){
            cv::Vec<unsigned char, 3> pix = img.ptr<cv::Vec3b>(y)[x];
            if(ptr[(y*img.cols+x)*3] != pix[0]){
                rst = false;
                cout << y << " " << x << " "  << ptr[(y*img.cols+x)*3] << " " << pix[0];
            }
            if(ptr[(y*img.cols+x)*3+1] != pix[1]){
                rst = false;
                cout << y << " " << x << " "  << ptr[(y*img.cols+x)*3+1] << " " << pix[1];
            }
            if(ptr[(y*img.cols+x)*3+2] != pix[2]){
                rst = false;
                cout << y << " " << x << " "  << ptr[(y*img.cols+x)*3+2] << " " << pix[2];
            }
        }
    }
    return rst;
}

bool inputdatacheck2(unsigned char* ptr, cv::Mat img){
  bool rst = true;
  for(int y = 0; y < img.rows; y++){
	for(int x = 0; x < img.cols; x++){
	  int pix = int(img.ptr<uchar>(y)[x]);
	  if(ptr[y*img.cols+x] != pix){
		rst = false;
		cout << y << " " << x << ptr[y*img.cols+x] << " " << pix << endl;
		//cout << "unko" << endl;
	  }
	}
  }
  if(rst) cout << "input data check2 OK" << endl;
  return rst;
}

bool hwresultcheck(unsigned short* sw_feature, unsigned short* hw_feature, int start, int end){
    bool showmode = true;
    bool flg = true;
    for(int i = start; i < end; i++){
        if(((int) sw_feature[i]) != ((int) hw_feature[i])){
            flg = false;
			cout << i << " is wrong , sw: " << int(sw_feature[i]) << " hw: " << int(hw_feature[i]) << endl;
        }
    }
    if(!flg) cout << "hwresult check NG" << endl;
    return flg;
}

float test_one_window(Mat rgb, Mat hls, Mat gray, double* time1, double* time2){
    std::chrono::system_clock::time_point  t1, t2, t3, t4, t5, t6, t7;
    cv::Size spatial_size(8, 8);
    Mat resized_rgb, resized_hls;
    cv::resize(rgb, resized_rgb, spatial_size);
    cv::cvtColor(resized_rgb, resized_hls, CV_RGB2HLS);

    //64*3*2+12*3*2+72*4=744
    unsigned short sw_feature[FEATURE_SIZE] = {0};
    unsigned short hw_feature[FEATURE_SIZE] = {0};
    
	/*for(int i = 0; i < 32; i++){
	  for(int j = 0; j < 64; j++) cout << int(gray.ptr<uchar>(i)[j]) << ", ";
	  cout << endl;
	  }*/
    t1 = std::chrono::system_clock::now();
    if(hwmode){
        memset(hw_feature, 0, sizeof(unsigned short) * FEATURE_SIZE);
        ravel(resized_hls, hw_feature);
        ravel(resized_rgb, hw_feature + 192);
		t1 = std::chrono::system_clock::now();
        //create RGB histogram
        for(int i = 0; i < 32; i++)  memcpy(hist_imageBuffer + 64 * i * 3, rgb.data + rgb.step * i, rgb.cols * 3 * sizeof(unsigned char));
        color_hist_hw(hw_feature + 192 * 2);
        //create HSV histogram
        for(int i = 0; i < 32; i++)  memcpy(hist_imageBuffer + 64 * i * 3, hls.data + hls.step * i, hls.cols * 3 * sizeof(unsigned char));
        color_hist_hw(hw_feature + 192 * 2 + 36);
        //calculate HOG feature
		t2 = std::chrono::system_clock::now();
		for(int i = 0; i < 32; i++)  memcpy(hog_imageBuffer + 64 * i, gray.data + gray.step * i, gray.cols * sizeof(unsigned char));
		//inputdatacheck2(hog_imageBuffer, gray);
        calc_hog_hw(hw_feature + 192 * 2 + 36 * 2);
		t3 = std::chrono::system_clock::now();
    }
	//t2 = std::chrono::system_clock::now();
	*time1 += (long double)std::chrono::duration_cast<std::chrono::microseconds>(t2-t1).count()/1000;
	*time2 += (long double)std::chrono::duration_cast<std::chrono::microseconds>(t3-t2).count()/1000;
    if(!hwmode || checkmode){
        memset(sw_feature, 0, sizeof(double) *584);
        ravel(resized_hls, sw_feature);
        ravel(resized_rgb, sw_feature + 192);
        hist(rgb, sw_feature + 192 * 2);
        hist(hls, sw_feature + 192 * 2 + 36);
        lite_hog(gray, sw_feature + 192 * 2 + 36 * 2);
    }
	if(checkmode) hwresultcheck(sw_feature, hw_feature, 0, FEATURE_SIZE);
    //Classify by Random Forest
    // clf_res res(0, 0);
    double proba;
    if(hwmode) proba = randomforest_classifier(hw_feature);
    else       proba = randomforest_classifier(sw_feature);
    // float red_proba = (float)res.red / (res.not_red + res.red);
    // cout << red_proba << endl;
    return proba;
}
void test_one_frame(Mat frame){
    std::chrono::system_clock::time_point  t1, t2, t3, t4, t5, t6, t7;

    t1 = std::chrono::system_clock::now();
    //1. crop use frame 
    Mat rgb(frame, cv::Rect(sx_min, sy_min, ex_max - sx_min, ey_max - sy_min));
    //2. convert to HLS image
    Mat hls;
    cv::cvtColor(rgb, hls, CV_RGB2HLS);
    Mat gray;
    cv::cvtColor(rgb, gray, CV_RGB2GRAY);
    Mat rgb_each_window[4];
    Mat hls_each_window[4];
    Mat gray_each_window[4];
    //3. resize for each window kind
    int cnt = 0;
    for(auto itr = widthkind.begin(); itr != widthkind.end(); ++itr) {
        int width = *itr;
        cout << (float)WINDOW_WIDTH/(float)width << endl;
    //TODO:crop use frame
        cv::resize(rgb, rgb_each_window[cnt++], cv::Size(), (float)WINDOW_WIDTH/(float)width , (float)WINDOW_WIDTH/(float)width);
    }
    cnt = 0;
    for(auto itr = widthkind.begin(); itr != widthkind.end(); ++itr) {
        int width = *itr;
        //TODO:crop use frame
        cv::resize(hls, hls_each_window[cnt++], cv::Size(), (float)WINDOW_WIDTH/(float)width , (float)WINDOW_WIDTH/(float)width);
    }
    cnt = 0;
    for(auto itr = widthkind.begin(); itr != widthkind.end(); ++itr) {
        int width = *itr;
        //TODO:crop use frame
        cv::resize(gray, gray_each_window[cnt++], cv::Size(), (float)WINDOW_WIDTH/(float)width , (float)WINDOW_WIDTH/(float)width);
    }
    t2 = std::chrono::system_clock::now();
    //4.crop for each window and classify for each window
    int test_count = 0;
    cnt = 0;
    double time1 = 0;
    double time2 = 0;
    for(auto itr = widthkind.begin(); itr != widthkind.end(); ++itr) {
        int width = *itr;
        auto windows = mp[width];
        for(int i = 0; i < windows.size(); i++){
            Mat rgb_test(rgb_each_window[cnt], cv::Rect(windows[i].first, windows[i].second, WINDOW_WIDTH, WINDOW_HEIGHT));
            Mat hls_test(hls_each_window[cnt], cv::Rect(windows[i].first, windows[i].second, WINDOW_WIDTH, WINDOW_HEIGHT));
            Mat gray_test(gray_each_window[cnt], cv::Rect(windows[i].first, windows[i].second, WINDOW_WIDTH, WINDOW_HEIGHT));
            float result = test_one_window(rgb_test, hls_test, gray_test, &time1, &time2);
            // imwrite("wind/img" + to_string(test_count) + ".png", rgb_test);
            if(result >= proba_thresh && imgout){
                cout << "found: " << proba_thresh << endl;
            }
            test_count++;
        }
        cnt++;
    }
    t3 = std::chrono::system_clock::now();
    // cout << "test_count" << test_count << endl;

    double tmp1 = (long double)std::chrono::duration_cast<std::chrono::microseconds>(t2-t1).count()/1000;
    double tmp2 = (long double)std::chrono::duration_cast<std::chrono::microseconds>(t3-t2).count()/1000;
    double all = (long double)std::chrono::duration_cast<std::chrono::microseconds>(t3-t1).count()/1000;
    cout << "preprocessing time : " << tmp1 << "[milisec]" << endl;
    cout << "classify time : " << tmp2 << "[milisec]" << endl;
    cout << "all time : " << all << "[milisec]" << endl;
    cout << "hw hist time" << time1 << "[milisec]" << endl;
    cout << "hw hog time" << time2 << "[milisec]" << endl;
}

int main(int argc, const char* argv[]){

    check_window();

    cout << "hw_setup" << endl;
    if(hwmode) hw_setup();

    for(auto itr = widthkind.begin(); itr != widthkind.end(); ++itr) {
        int width = *itr;
        cout << width << ":" << mp[width].size() << endl;
    }
    Mat img = imread("frame.png");
    cout << img.cols << endl;
    test_one_frame(img);

    if(hwmode) hw_release();
    return 0;
}
