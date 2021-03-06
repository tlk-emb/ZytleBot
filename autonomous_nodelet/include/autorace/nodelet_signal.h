#include <iostream>
#include <chrono>
#include <vector>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "unistd.h"
#include <math.h>
#include <stdio.h>
#include <geometry_msgs/Twist.h>

#include "signal_lib/feature.h"
#include "signal_lib/forest.h"
#include "signal_lib/window_candidate.h"
#include "signal_lib/hw.h"
#include "std_msgs/String.h"

// JSON読み込み
#include <iostream>
#include <fstream>
#include <sstream>
#include <json_lib/json11.hpp>

using namespace std;
using namespace cv;

map<int, vector<pair<int, int> > > mp[2];
map<int, vector<pair<int, int> > > original_mp[2];
set<int> widthkind[2];

int sx_min[2] = {999, 999};
int sy_min[2] = {999, 999};
int ex_max[2] = {-1, -1};
int ey_max[2] = {-1, -1};
#define WINDOW_WIDTH 64
#define WINDOW_HEIGHT 32
bool imgout = false;
float proba_thresh = 0.65;
#define hwmode true
#define checkmode false
#define showdetailtime false
#define FEATURE_SIZE 672

unsigned short sw_feature[FEATURE_SIZE*4] = {0};
unsigned short hw_feature[FEATURE_SIZE*4] = {0};


void check_window(){
    for(int i = 0; i < window_num; i++){
        int sy = w[i][0][0];
        int sx = w[i][1][0];
        int ey = w[i][0][1];
        int ex = w[i][1][1];
        // cv::Mat cropped(frame, cv::Rect(sx, sy, ex - sx, ey - sy));
        widthkind[0].insert(ex-sx);
        sx_min[0] = min(sx_min[0], sx);
        sy_min[0] = min(sy_min[0], sy);
        ex_max[0] = max(ex_max[0], ex);
        ey_max[0] = max(ey_max[0], ey);
    }

    for(int i = 0; i < window_num; i++){
        //(old_x - sx_min) * ratio
        int sy = w[i][0][0];
        int sx = w[i][1][0];
        int ey = w[i][0][1];
        int ex = w[i][1][1];

        int original_width = ex - sx;
        int ssy = (int)((float) (sy - sy_min[0]) * (float)WINDOW_WIDTH/original_width);
        int ssx = (int)((float) (sx - sx_min[0]) * (float)WINDOW_WIDTH/original_width);
        mp[0][original_width].push_back(make_pair(ssx, ssy));
        //store original coordinate
        original_mp[0][original_width].push_back(make_pair(sx, sy));
    }
}

void check_window2(std::string project_folder){

    ifstream fin(project_folder + "/signal.json" );
    if( !fin ){
        cout << "signal json load failed" << endl;
        return;
    }

    cout << "signal json loaded" << endl;

    stringstream strstream;
    strstream << fin.rdbuf();
    fin.close();
    string jsonstr(strstream.str());

    string err;
    auto json = json11::Json::parse(jsonstr, err);
    auto params = json["signal"];


    //TODO: load window range for crossing singal from json file
    int cross_signal_range_sx = params["cross_signal_range_sx"].int_value();
    int cross_signal_range_sy = params["cross_signal_range_sy"].int_value();
    int cross_signal_range_ex = params["cross_signal_range_ex"].int_value();
    int cross_signal_range_ey = params["cross_signal_range_ey"].int_value();
    int cross_signal_x_step = params["cross_signal_x_step"].int_value();
    int cross_signal_y_step = params["cross_signal_y_step"].int_value();
    int cross_signal_height_upper = params["cross_signal_height_upper"].int_value();
    int cross_signal_height_step = params["cross_signal_height_step"].int_value();

    sx_min[1] = cross_signal_range_sx;
    sy_min[1] = cross_signal_range_sy;
    for(int height = 20; height < cross_signal_height_upper; height+=cross_signal_height_step){
        int width = height * 2;
        widthkind[1].insert(width);
        for(int y = cross_signal_range_sy; y <= cross_signal_range_ey; y+=cross_signal_y_step){
            for(int x = cross_signal_range_sx; x <= cross_signal_range_ex; x+=cross_signal_x_step){
                int original_width = width;
                ex_max[1] = max(ex_max[1], x + width);
                ey_max[1] = max(ey_max[1], y + height);
                int ssy = (int)((float) (y - cross_signal_range_sy) * (float)WINDOW_WIDTH/original_width);
                int ssx = (int)((float) (x - cross_signal_range_sx) * (float)WINDOW_WIDTH/original_width);
                mp[1][original_width].push_back(make_pair(ssx, ssy));
                //store original coordinate
                original_mp[1][original_width].push_back(make_pair(x, y));
            }
        }
    }
}

namespace autorace{
    class NodeletSignal : public nodelet::Nodelet {

        ros::NodeHandle nh_;
        ros::Subscriber image_sub_;
        ros::Subscriber signal_search_;

        ros::Publisher red_flag_;


        int window_mode;
        int how_search; // -1で探さない 0で外周 1で交差点
        bool find_flag;
        int find_count;

    public:
        // コンストラクタ
        NodeletSignal(){
        }

        // デストラクタ
        ~NodeletSignal() {
            if (hwmode) hw_release();
            // 全てのウインドウは破壊
            cv::destroyAllWindows();
        }

        void onInit() {
            window_mode = 0;
            how_search = 0;
            cout << "nodelet_signal start" << endl;

            XmlRpc::XmlRpcValue ros_params;

            nh_ = getNodeHandle();
            nh_.getParam("/nodelet_autorace/autorace", ros_params);

            check_window();
            check_window2((std::string)ros_params["project_folder"]);


            // TODO subscribe先はtopicに応じて変更
            image_sub_ = nh_.subscribe("/usbcam/image_array", 1,
                                       &NodeletSignal::imageCb, this);

            signal_search_ = nh_.subscribe("/signal_search_type", 1, &NodeletSignal::signalSearchCb, this);

            red_flag_ = nh_.advertise<std_msgs::String>("/red_flag", 1);

            if(hwmode) hw_setup();
            cout << "nodelet hw setup completed" << endl;

            find_flag = false;
            find_count = 0;
        }

        void signalSearchCb(const std_msgs::String &msg) {
            if (msg.data == "-1") {
                how_search = -1;
            } else if (msg.data == "0") {
                how_search = 0;
            } else {
                how_search = 1;
            }
        }

        // コールバック関数
        void imageCb(const std_msgs::UInt8MultiArrayPtr &msg) {

            cout << "search signal Type = " << how_search << "!!!!!!" << endl;

            std_msgs::String send_msg;
            send_msg.data = "false";
            if (how_search != -1) {
                cout << "searching signal!!!!" << endl;
                cv::Mat baseImage(480, 640, CV_8UC2);
                cv::Mat dstimg(480, 640, CV_8UC2);
                memcpy(baseImage.data, &(msg->data[0]), 640 * 480 * 2);
                cv::cvtColor(baseImage, dstimg, cv::COLOR_YUV2BGR_YUYV);

                // TODO receive window_mode
                if (how_search == 0) window_mode = 0;
                if (how_search == 1) window_mode = 1;
                // window_mode = 0 or 1
                // TODO show receive window mode

                // baseImageをなんか処理する

                // TODO 640*480以外の場合
                // resize(baseImage, baseImage, cv::Size(), 640.0/baseImage.cols ,480.0/baseImage.rows);

                std::chrono::system_clock::time_point t1, t2, t3, t4, t5, t6, t7;
                t1 = std::chrono::system_clock::now();
                cv::Mat frame_copy = dstimg.clone();

                //process the current frame
                auto rst = test_one_frame(dstimg, window_mode);

                bool now_find = rst.size() > 0;

                if (now_find) { // 見つかった場合
                    if (find_count >= 0) { // find_countが0以上の場合
                        find_count++;
                    } else {
                        find_count = 1; // find_countが負の値の場合
                    }
                } else { //　見つからなかった場合
                    if (find_count <= 0) {
                        find_count--; // find_countが負の場合、-1
                    } else {
                        find_count = -1; // find_countが正の値の場合
                    }
                }

                // 3回連続で見つけたり見失ったらフラグを変更
                if (find_count >= 2) {
                    find_flag = true;
                } else if (find_count <= -3) {
                    find_flag = false;
                }

                //draw rectangle on detecting area
                for (int j = 0; j < rst.size(); j++) {
                    vector<int> coord = rst[j].first;
                    float proba = rst[j].second;
                    int sx = coord[0];
                    int sy = coord[1];
                    int ex = coord[2];
                    int ey = coord[3];
                    cout << sx << " " << sy << " " << ex << " " << ey << endl;
                    cout << proba << endl;
                    //rectangle(frame_copy, Point(sx, sy), Point(ex, ey), Scalar(0, 0, 200), 2); //x,y //Scaler = B,G,R
                    //cv::putText(frame_copy, to_string(proba), cv::Point(sx + 5, sy + 5), cv::FONT_HERSHEY_SIMPLEX, 0.5,
                    //            cv::Scalar(255, 0, 0), 1, CV_AA);
                }
                //cv::imshow("result", frame_copy);*/
                t2 = std::chrono::system_clock::now();
                //show fps
                double elapsed = (double) std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count();
                cout << "elapsed:" << elapsed << "[milisec]" << endl;
                cout << "fps:" << 1000.0 / elapsed << "[fps]" << endl;


                std::cout << "publish something" << std::endl;

                // flagの送信処理
                if (find_flag) {
                    send_msg.data = "true";
                }
            } else {
                cout << "don't search signal" << endl;
                find_count = 0;
            }

            red_flag_.publish(send_msg);
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

        void test_four_window(float* result, int num, Mat rgb[4], Mat hls[4], Mat gray[4], double* time0, double* time1, double* time2, double *time3){
            std::chrono::system_clock::time_point  t0, t1, t2, t3, t4, t5, t6, t7;
            cv::Size spatial_size(8, 8);
            Mat spatial_rgb[4], spatial_hls[4];
            t0 = std::chrono::system_clock::now();
            for(int i = 0; i < num; i++){
                cv::resize(rgb[i], spatial_rgb[i], spatial_size);
                cv::resize(hls[i], spatial_hls[i], spatial_size);
            }

            if(hwmode){
                memset(hw_feature, 0, sizeof(unsigned short) * FEATURE_SIZE * 4);
                for(int i = 0; i < num; i++){
                    ravel(spatial_hls[i], i * FEATURE_SIZE + hw_feature);
                    ravel(spatial_rgb[i], i * FEATURE_SIZE + hw_feature + 192);
                }
                t1 = std::chrono::system_clock::now();
                //calculate HOG feature
                t2 = std::chrono::system_clock::now();
                for(int i = 0; i < 32; i++)  memcpy(hog_imageBuffer0 + 64 * i, gray[0].data + gray[0].step * i, gray[0].cols * sizeof(unsigned char));
                if(num > 1) for(int i = 0; i < 32; i++)  memcpy(hog_imageBuffer1 + 64 * i, gray[1].data + gray[1].step * i, gray[1].cols * sizeof(unsigned char));
                if(num > 2) for(int i = 0; i < 32; i++)  memcpy(hog_imageBuffer2 + 64 * i, gray[2].data + gray[2].step * i, gray[2].cols * sizeof(unsigned char));
                if(num > 3) for(int i = 0; i < 32; i++)  memcpy(hog_imageBuffer3 + 64 * i, gray[3].data + gray[3].step * i, gray[3].cols * sizeof(unsigned char));
                calc_hog_hw(num, hw_feature + 192 * 2, hw_feature + 192 * 2 + FEATURE_SIZE, hw_feature + 192 * 2 + FEATURE_SIZE * 2, hw_feature + 192 * 2 + FEATURE_SIZE * 3);
                t3 = std::chrono::system_clock::now();
            }
            //t2 = std::chrono::system_clock::now();
            *time0 += (long double)std::chrono::duration_cast<std::chrono::microseconds>(t1-t0).count()/1000;
            *time1 += (long double)std::chrono::duration_cast<std::chrono::microseconds>(t2-t1).count()/1000;
            *time2 += (long double)std::chrono::duration_cast<std::chrono::microseconds>(t3-t2).count()/1000;
            if(!hwmode || checkmode){
                memset(sw_feature, 0, sizeof(unsigned short) * FEATURE_SIZE * 4);
                for(int i = 0; i < num; i++){
                    ravel(spatial_hls[i], i * FEATURE_SIZE + sw_feature);
                    ravel(spatial_rgb[i], i * FEATURE_SIZE + sw_feature + 192);
                    lite_hog(gray[i], i * FEATURE_SIZE + sw_feature + 192 * 2);
                }
            }
            if(checkmode) hwresultcheck(sw_feature, hw_feature, 0, FEATURE_SIZE * 4);
            //Classify by Random Forest
            t4 = std::chrono::system_clock::now();
            for(int i = 0; i < 4; i++){
                if(hwmode) result[i] = randomforest_classifier(i * FEATURE_SIZE + hw_feature);
                else       result[i] = randomforest_classifier(i * FEATURE_SIZE + sw_feature);
            }
            t5 = std::chrono::system_clock::now();
            *time3 += (long double)std::chrono::duration_cast<std::chrono::microseconds>(t5-t4).count()/1000;
        }

        vector<pair<vector<int>, float>> test_one_frame(Mat frame, int mode){
            vector<pair<vector<int>, float>> rst;
            std::chrono::system_clock::time_point  t1, t2, t3, t4, t5, t6, t7;

            t1 = std::chrono::system_clock::now();
            //1. crop use frame
            Mat rgb(frame, cv::Rect(sx_min[mode], sy_min[mode], ex_max[mode] - sx_min[mode], ey_max[mode] - sy_min[mode]));
            //2. convert to HLS image
            Mat hls;
            cv::cvtColor(rgb, hls, CV_BGR2HSV);
            Mat gray;
            cv::cvtColor(rgb, gray, CV_BGR2GRAY);
            t2 = std::chrono::system_clock::now();
            Mat rgb_each_window[4];
            Mat hls_each_window[4];
            Mat gray_each_window[4];
            //3. resize for each window kind
            int cnt = 0;
            for(auto itr = widthkind[mode].begin(); itr != widthkind[mode].end(); ++itr) {
                int width = *itr;
                //TODO:crop use frame
                cv::resize(rgb, rgb_each_window[cnt++], cv::Size(), (float)WINDOW_WIDTH/(float)width , (float)WINDOW_WIDTH/(float)width, INTER_NEAREST);
            }
            cnt = 0;
            for(auto itr = widthkind[mode].begin(); itr != widthkind[mode].end(); ++itr) {
                int width = *itr;
                //TODO:crop use frame
                cv::resize(hls, hls_each_window[cnt++], cv::Size(), (float)WINDOW_WIDTH/(float)width , (float)WINDOW_WIDTH/(float)width, INTER_NEAREST);
            }
            cnt = 0;
            for(auto itr = widthkind[mode].begin(); itr != widthkind[mode].end(); ++itr) {
                int width = *itr;
                //TODO:crop use frame
                cv::resize(gray, gray_each_window[cnt++], cv::Size(), (float)WINDOW_WIDTH/(float)width , (float)WINDOW_WIDTH/(float)width, INTER_NEAREST);
            }
            t3 = std::chrono::system_clock::now();
            //4.crop for each window and classify for each window
            int test_count = 0;
            int widthkind_cnt = 0;
            double time0 = 0;
            double time1 = 0;
            double time2 = 0;
            double time3 = 0;
            for(auto itr = widthkind[mode].begin(); itr != widthkind[mode].end(); ++itr) {
                int width = *itr;
                auto windows = mp[mode][width];
                //process 4 window images for each iteration
                for(int i = 0; i*4 < windows.size(); i++){
                    int process_window_num = min(i*4+3, (int)windows.size()-1) - (i*4) + 1; //maximum 4
                    Mat rgb_test[4], hls_test[4], gray_test[4];
                    for(int j = 0; j < process_window_num; j++){
                        rgb_test[j]  = rgb_each_window[widthkind_cnt](cv::Rect(windows[i*4+j].first, windows[i*4+j].second, WINDOW_WIDTH, WINDOW_HEIGHT));
                        hls_test[j]  = hls_each_window[widthkind_cnt](cv::Rect(windows[i*4+j].first, windows[i*4+j].second, WINDOW_WIDTH, WINDOW_HEIGHT));
                        gray_test[j] = gray_each_window[widthkind_cnt](cv::Rect(windows[i*4+j].first, windows[i*4+j].second, WINDOW_WIDTH, WINDOW_HEIGHT));
                    }
                    float result[4];
                    test_four_window(result, process_window_num, rgb_test, hls_test, gray_test, &time0, &time1, &time2, &time3);
                    // imwrite("wind/img" + to_string(test_count) + ".png", rgb_test);
                    for(int j = 0; j < process_window_num; j++){
                        if(result[j] >= proba_thresh){
                            int original_window_sx = original_mp[mode][width][i*4+j].first;
                            int original_window_sy = original_mp[mode][width][i*4+j].second;
                            int original_window_ex = original_window_sx + width;
                            int original_window_ey = original_window_sy + width/2;
                            vector<int> coord = {original_window_sx, original_window_sy, original_window_ex, original_window_ey};
                            rst.push_back(make_pair(coord, result[j]));
                        }
                    }
                    test_count+=process_window_num;
                }
                widthkind_cnt++;
            }
            t4 = std::chrono::system_clock::now();
            cout << "test_count" << test_count << endl;

            double tmp1 = (long double)std::chrono::duration_cast<std::chrono::microseconds>(t2-t1).count()/1000;
            double tmp2 = (long double)std::chrono::duration_cast<std::chrono::microseconds>(t3-t2).count()/1000;
            double tmp3 = (long double)std::chrono::duration_cast<std::chrono::microseconds>(t4-t3).count()/1000;
            double all = (long double)std::chrono::duration_cast<std::chrono::microseconds>(t4-t1).count()/1000;
            if(showdetailtime){
                cout << "preprocessing color convert time : " << tmp1 << "[milisec]" << endl;
                cout << "preprocessing resize time : " << tmp2 << "[milisec]" << endl;
                cout << "classify time : " << tmp3 << "[milisec]" << endl;
                cout << "all time : " << all << "[milisec]" << endl;
                cout << "sw ravel time" << time0 << "[milisec]" << endl;
                cout << "hw hist time" << time1 << "[milisec]" << endl;
                cout << "hw hog time" << time2 << "[milisec]" << endl;
                cout << "random forest time" << time3 << "[milisec]" << endl;
            }
            return rst;
        }
    };
}