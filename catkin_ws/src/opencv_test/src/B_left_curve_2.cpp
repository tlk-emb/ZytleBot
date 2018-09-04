#include <iostream>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "unistd.h"
#include <math.h>
#include <stdio.h>
#include <opencv2/opencv.hpp>
#include <geometry_msgs/Twist.h>

// pcam使用時
#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/UInt8MultiArray.h"


#define PI 3.141592653589793
#define BIRDSEYE_LENGTH 100
#define BURGER_MAX_LIN_VEL 0.22
#define BURGER_MAX_ANG_VEL 2.84

// 直進の調整
#define STRAIGHT_TOO_RIGHT_LANE 0.25
#define STRAIGHT_TOO_LEFT_LANE 0.05


// カーブの調整
#define RIGHT_CURVE_END_MARGIN_TIME 1
#define RIGHT_CURVE_END_TIME 6
#define RIGHT_CURVE_ROT -0.4

#define LEFT_CURVE_END_TIME 2.45
#define LEFT_CURVE_END_MARGIN_TIME 0
#define LEFT_CURVE_VEL 0.1
#define LEFT_CURVE_ROT 0.7
#define LEFT_CURVE_AFTER_ROT 0.1

#define AVOID_OBSTACLE_VEL 0.2
#define AVOID_OBSTACLE_ROT -0.3
#define AVOID_ROT_TIME 1
#define AVOID_ROT_STRAIGHT 3
#define AVOID_STRAIGHT_TIME 6

// 一度認識したオブジェクトの位置を推定するための比率
// 大きくすればするほど、後に発見したオブジェクトの優先度が高まる
#define INTERSECTION_PREDICTION_TIME_RATIO 1.5
// カメラの下部から実際に曲がるまでのマージン(s)を決定 TIME_RATIOの影響を受ける
#define INTERSECTION_PREDICTION_UNDER_MARGIN 0.8


#define OBJECT_DISCARDING_TIME 3


// 走行ラインの左からの割合
// カーブがギリギリの場合大きく
#define RUN_LINE 0.18

// 最初の交差点もしくはカーブ位置 dirは以下のように0が南で右回り
// map_dataは下で書き換える必要がある
// 2
//1 3
// 0
#define NEXT_X 0
#define NEXT_Y 0
#define START_DIR 1

#define HUE_L 0
#define HUE_H 180
#define SATURATION_L 0
#define SATURATION_H 45
#define LIGHTNESS_L 220
#define LIGHTNESS_H 255

// zybo_camera用設定
#define CAMERA_WIDTH 640
#define CAMERA_HEIGHT 480


#define WIDTH_RATIO 0.17
#define HEIGHT_H 0.592
#define HEIGHT_L 0.79

/*
#define WIDTH_RATIO 0.19
#define HEIGHT_H 0.59
#define HEIGHT_L 0.8
*/


typedef struct object {
public:
    // オブジェクトの種類
    // obstacle, intersection, people
    std::string objType;
    int beforeX;
    int beforeY;
    int findCnt;
    ros::Time timeStamp;
} OBJECT;


// 直線を中点、傾き、長さで表す
typedef struct straight {
    cv::Point2f middle;
    float degree;
    float length;
} STRAIGHT;

// map_data[y][x][0]がタイルの種類
// map_data[y][x][1]が向きを表している
// 向きは1がデータ画像のとおりで、0~3で右回りに表現されている
int map_data[7][5][2] = {{{3, 0}, {4, 0}, {7, 2}, {4, 0}, {3, 1}},
                         {{6, 1}, {0, 0}, {1, 1}, {0, 0}, {6, 1}},
                         {{4, 1}, {0, 0}, {5, 1}, {0, 0}, {4, 1}},
                         {{7, 1}, {2, 0}, {8, 0}, {2, 2}, {7, 3}},
                         {{4, 1}, {0, 0}, {5, 3}, {0, 0}, {4, 1}},
                         {{6, 1}, {0, 0}, {1, 3}, {0, 0}, {6, 1}},
                         {{3, 3}, {4, 0}, {7, 0}, {4, 0}, {3, 2}}};

/*
now_phaseについて
0なら直線検出モード
1なら右カーブ検出決め打ち移動状態
2なら左カーブ検出
3ならカーブ終了処理中直線検出以降状態（緩やかにカーブをやめて直線モードに移行）
4なら障害物検知状態
5なら信号検知状態
*/


class ImageConverter {
    ros::NodeHandle nh_;
    /* if zybo
    ros::Subscriber image_sub_;
    */
    // if_pc
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;

    int Hue_l, Hue_h, Saturation_l, Saturation_h, Lightness_l, Lightness_h;
    geometry_msgs::Twist twist;

    ros::Publisher twist_pub;
    // cv::Mat curve_image;
    int line_lost_cnt;
    int curve_detect_cnt;
    int find_T;
    int find_X;

    bool find_left_line;

    std::string now_phase;

    // 発見したオブジェクト（交差点、障害物）のリスト
    std::list <OBJECT> objects;


    // 次のtileを保存
    int next_tile_x;
    int next_tile_y;
    int now_dir;

    // 検出された直線のx座標
    int detected_line_x;

    ros::Time phase_start_t;
    ros::Time line_lost_time;

    // change phaseで初期化
    // bottomにオブジェクトが到達したかどうか
    bool reachBottomRightLaneRightT;
    bool reachBottomRightLaneLeftT;
    bool reachBottomLeftLaneLeftT;

    bool findLeftLaneLeftT;

    int mostUnderLeftLaneLeftT;


public:
    // コンストラクタ
    ImageConverter()
    // if pc
            : it_(nh_) {
        Hue_l = HUE_L;
        Hue_h = HUE_H;
        Saturation_l = SATURATION_L;
        Saturation_h = SATURATION_H;
        Lightness_l = LIGHTNESS_L;
        Lightness_h = LIGHTNESS_H;
        line_lost_cnt = 0;
        next_tile_x = NEXT_X;
        next_tile_y = NEXT_Y;
        now_dir = START_DIR;

        find_left_line = false;


        detected_line_x = 0;

        // start時間を初期化
        phase_start_t = ros::Time::now();
        line_lost_time = ros::Time::now();

        now_phase = "straight";

        reachBottomRightLaneRightT = false;
        reachBottomRightLaneLeftT = false;
        reachBottomLeftLaneLeftT = false;
        findLeftLaneLeftT = false;
        mostUnderLeftLaneLeftT = 0;


        // カラー画像をサブスクライブ
        /* if_zybo
          image_sub_ = nh_.subscribe("/image_array", 1,
          &ImageConverter::imageCb, this);
        */
        image_sub_ = it_.subscribe("/camera/rgb/image_raw", 1,
                                   &ImageConverter::imageCb, this);

        //  処理した挙動をパブリッシュ
        //twist_pub = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);
        twist_pub = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);
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
    ~ImageConverter() {
        // 全てのウインドウは破壊
        cv::destroyAllWindows();
    }


    // コールバック関数
    // if zybo
    // void imageCb(const std_msgs::UInt8MultiArray& msg)
    void imageCb(const sensor_msgs::ImageConstPtr &msg) {
        /* if_zybo
        cv::Mat base_image(CAMERA_HEIGHT, CAMERA_WIDTH, CV_8UC2);
        cv::Mat dstimg(CAMERA_HEIGHT, CAMERA_WIDTH, CV_8UC2);
        memcpy(base_image.data, &msg.data[0], CAMERA_WIDTH * CAMERA_HEIGHT * 2);
        cv::cvtColor(base_image, dstimg, cv::COLOR_YUV2RGB_YUYV);
        */


        // if_pc
        cv_bridge::CvImagePtr cv_ptr;
        try {
            // ROSからOpenCVの形式にtoCvCopy()で変換。cv_ptr->imageがcv::Matフォーマット。
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        }
        catch (cv_bridge::Exception &e) {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }
        cv::Mat base_image = cv_ptr->image;
        ////////


        cv::Mat hsv_image, color_mask, gray_image, birds_eye, road_white_binary;

        // 俯瞰画像

        birds_eye = birdsEye(base_image);
        road_white_binary = whiteBinary(birds_eye);

        cv::Mat aroundImg, aroundWhiteBinary;
        aroundImg = birdsEyeAround(base_image);
        aroundWhiteBinary = whiteBinary(aroundImg);

        std::vector <cv::Vec4i> around_lines = getHoughLinesP(aroundWhiteBinary, 0, 10, 5);


        cv::Mat polarSrc, polarResult;
        polarSrc = road_white_binary.clone();
        polarResult = polarCoordinateConversion(polarSrc);


        cv::Mat road_hough;

        // デバッグ用
        cv::Canny(road_white_binary, road_hough, 50, 200, 3);

        // 俯瞰画像のROIを縦中央で分割
        cv::Mat left_roi(road_white_binary, cv::Rect(0, 0, BIRDSEYE_LENGTH / 2, BIRDSEYE_LENGTH));
        cv::Mat right_roi(road_white_binary, cv::Rect(BIRDSEYE_LENGTH / 2, 0, BIRDSEYE_LENGTH / 2, BIRDSEYE_LENGTH));

        // cv::Mat image, int threshold, double minLineLength, double maxLineGap
        std::vector <cv::Vec4i> polar_lines = getHoughLinesP(polarResult, 20, 40, 5);
        std::vector <cv::Vec4i> left_lines = getHoughLinesP(left_roi, 20, 40, 5);
        std::vector <cv::Vec4i> right_lines = getHoughLinesP(right_roi, 0, 10, 5);

        // curveがあったかどうか
        bool find_curve = false;
        // 極座標変換した画像に対して線を引く
        for (size_t i = 0; i < polar_lines.size(); i++) {
            STRAIGHT polar_line = toStraightStruct(polar_lines[i]);
            if (polar_line.middle.x < 30 && polar_line.degree < 10 && polar_line.degree > -10) {
                find_curve = true;
                cv::line(polarResult, cv::Point(polar_lines[i][0], polar_lines[i][1]),
                         cv::Point(polar_lines[i][2], polar_lines[i][3]), cv::Scalar(255, 0, 0), 3, 8);
            }
        }
        if (find_curve) curve_detect_cnt++;

        // 右画像に対して,Tや十が存在するかサーチする
        /*
        for( int i = 0; i < right_lines.size(); i++ )
        {
          STRAIGHT right_line = toStraightStruct(right_lines[i]);
          // 水平に近い
          if (right_line.degree > 80 || right_line.degree < -80) {
            for( int j = 0; j < right_lines.size(); j++ ) {
              if (j == i) continue;
              STRAIGHT check_line = toStraightStruct(right_lines[j]);

              // 比較する対象を垂直に絞る
              if (check_line.degree < 20 && check_line.degree > -20){

                // pointがpoint二点の間にあるかどうか調べる関数
                if (crossCheck(right_lines[i], right_lines[j])){
                  cv::line( right_roi, cv::Point(right_lines[i][0], right_lines[i][1]),
                    cv::Point(right_lines[i][2], right_lines[i][3]), cv::Scalar(0,255,0), 3, 8 );
                  cv::line( right_roi, cv::Point(right_lines[j][0], right_lines[j][1]),
                    cv::Point(right_lines[j][2], right_lines[j][3]), cv::Scalar(0,255,0), 3, 8 );
                }
              }
            }
          }
        }
         */
        intersectionDetection(around_lines, aroundWhiteBinary);


        // 左車線について
        // 角度平均をとり、全体の角度が垂直になるようにする
        // 最も左車線を検出し、いい感じになるよう調整する
        // 車線が見つからない場合、find_left_lineがfalseになる
        int average_cnt = 0;
        float degree_average_sum = 0;
        float most_left_middle_x = BIRDSEYE_LENGTH * 0.5;
        float robot_vel = 0;
        float robot_rot = 0;
        find_left_line = false;

        // 垂直に近い点のみ線を引く
        for (size_t i = 0; i < left_lines.size(); i++) {
            STRAIGHT left_line = toStraightStruct(left_lines[i]);
            if (left_line.degree < 20 && left_line.degree > -20) {
                //赤線を引く
                cv::line(left_roi, cv::Point(left_lines[i][0], left_lines[i][1]),
                         cv::Point(left_lines[i][2], left_lines[i][3]), cv::Scalar(0, 0, 255), 3, 8);

                degree_average_sum += left_line.degree;
                if (most_left_middle_x > left_line.middle.x) {
                    most_left_middle_x = left_line.middle.x;
                    detected_line_x = left_line.middle.x;
                }
                find_left_line = true;
                average_cnt++;
            }
        }


        // 左車線を検出できた場合
        if (find_left_line) {
            line_lost_time = ros::Time::now();
            float degree_average = degree_average_sum / average_cnt;

            // 中点が右過ぎたら左に、左過ぎたら右に曲がる
            if (detected_line_x > BIRDSEYE_LENGTH * STRAIGHT_TOO_RIGHT_LANE) {
                robot_vel = 1;
                robot_rot = -1;
            } else if (detected_line_x < BIRDSEYE_LENGTH * STRAIGHT_TOO_LEFT_LANE) {
                robot_vel = 1;
                robot_rot = 1;
            } else if (degree_average < -10 || degree_average > 10) {
                // 角度平均が-5以上なら左に曲がる、5以上なら右に曲がる
                robot_rot = degree_average * -0.2;
            } else {
                robot_vel = 1;
            }
            line_lost_cnt = 0;
        } else {
            // 車線が見つからなかった場合、LeftRoadLeftTで最下のものを基準に
            line_lost_cnt += 1;
            updateLeftLine(road_white_binary);
            robot_vel = 0;
            robot_rot = (BIRDSEYE_LENGTH * RUN_LINE - detected_line_x) / 40;
        }

        // 次のタイルの左車線が不安定な場合、LeftLaneのLeftTを優先してdetected_line_xにする
        if (line_lost_cnt > 5) {
            if (map_data[next_tile_y][next_tile_x][0] == 3 &&
                (map_data[next_tile_y][next_tile_x][1] - now_dir + 4) % 4 == 3) {
                if (findLeftLaneLeftT) {
                    detected_line_x = mostUnderLeftLaneLeftT;
                    line_lost_time = ros::Time::now();
                    // 中点が右過ぎたら左に、左過ぎたら右に曲がる
                    if (detected_line_x > BIRDSEYE_LENGTH * STRAIGHT_TOO_RIGHT_LANE) {
                        robot_vel = 1;
                        robot_rot = -1;
                    } else if (detected_line_x < BIRDSEYE_LENGTH * STRAIGHT_TOO_LEFT_LANE) {
                        robot_vel = 1;
                        robot_rot = 1;
                    } else {
                        robot_vel = 1;
                    }
                }
            }
        }

        // ---------------controller----------------
        updateObject();

        if (now_phase == "straight") {
            ros::Time now = ros::Time::now();
            if (now - line_lost_time > ros::Duration(5.0)) {
                changePhase("search_line");
            } else {
                searchTile();
                lineTrace(robot_vel, robot_rot);
            }
        } else if (now_phase == "turn_right") {
            rightCurveTrace(road_white_binary);
        } else if (now_phase == "search_line") {
            searchLine();
        } else if (now_phase == "turn_left") {
            leftTurn();
        } else if (now_phase == "find_obs") {
            // obstacleAvoidance();
        }
        /*
        else if ("detect_right_line")
        {

        } else if ("start_calibration")
        {
        }
         */

        // ---------------controller end----------------

        // 以下デバッグ用
        // 画像サイズを縦横半分に変更


        ////////////
        // 白を見つけたらbaseを更新して、10ピクセル以内にまた白があればそれを仮の白線とする。
        int temp_detected_line = detected_line_x;
        int point_i = detected_line_x;
        int point_j = detected_line_x;


        // 複数あった場合、直前のライントレースの結果との差を利用する
        int temp_dif = BIRDSEYE_LENGTH / 2;
        // uchar *road_hough_bottom = road_hough.ptr<uchar>(BIRDSEYE_LENGTH - 1);
        for (int i = 0; i < detected_line_x + BIRDSEYE_LENGTH / 10; i++) {
            int p = road_hough.at<uchar>(BIRDSEYE_LENGTH - 1, i);
            if (p) {
                for (int j = i + 1; j < i + BIRDSEYE_LENGTH / 10; j++) {
                    int q = road_hough.at<uchar>(BIRDSEYE_LENGTH - 1, j);
                    if (q) {
                        int this_dif = (i + j) / 2 - detected_line_x;
                        if (this_dif < temp_dif) {
                            point_i = i;
                            point_j = j;
                            temp_dif = this_dif;
                            temp_detected_line = (i + j) / 2;
                        }
                    }
                }
            }
        }

        for (OBJECT object : objects) {
            if (object.objType == "right_T") {
                cv::ellipse(road_white_binary, cv::Point(object.beforeX + BIRDSEYE_LENGTH / 2, object.beforeY),
                            cv::Size(10, 10), 0, 0, 360, cv::Scalar(0, 200, 0), -1, 8);
            } else if (object.objType == "left_lane_left_T" || object.objType == "right_lane_left_T") {
                cv::ellipse(road_white_binary, cv::Point(object.beforeX + BIRDSEYE_LENGTH / 2, object.beforeY),
                            cv::Size(10, 10), 0, 0, 360, cv::Scalar(0, 0, 200), -1, 8);
            }
        }


        cv::line(road_white_binary, cv::Point(point_i, BIRDSEYE_LENGTH - 1),
                 cv::Point(point_j, BIRDSEYE_LENGTH - 1), cv::Scalar(255, 0, 0), 3, 8);


        cv::Mat cv_half_image, birds_eye_x4, white_binary_x4, left_roi_x4, right_roi_x4, polarResult_x4, aroundImg_x4, aroundWhiteBinary_x4;
        cv::resize(base_image, cv_half_image, cv::Size(), 0.25, 0.25);
        cv::resize(birds_eye, birds_eye_x4, cv::Size(), 4, 4);
        cv::resize(road_white_binary, white_binary_x4, cv::Size(), 4, 4);
        cv::resize(left_roi, left_roi_x4, cv::Size(), 4, 4);
        cv::resize(right_roi, right_roi_x4, cv::Size(), 4, 4);
        cv::resize(polarResult, polarResult_x4, cv::Size(), 4, 4);
        cv::resize(aroundImg, aroundImg_x4, cv::Size(), 2, 2);
        cv::resize(aroundWhiteBinary, aroundWhiteBinary_x4, cv::Size(), 2, 2);

        // ウインドウ表示
        cv::imshow("Original Image", cv_half_image);
        cv::imshow("WHITE BINARY", white_binary_x4);
        // cv::imshow("ROI", birds_eye_x4);
        //cv::imshow("LEFT ROI", left_roi_x4);
        //cv::imshow("RIGHT ROI",  right_roi_x4);
        cv::imshow("road hough", road_hough);
        cv::imshow("center line", aroundImg_x4);
        cv::imshow("aroundWhite", aroundWhiteBinary_x4);
        // cv::imshow("polarCoordinateConversion", polarResult_x4);

        cv::waitKey(3);

        //エッジ画像をパブリッシュ。OpenCVからROS形式にtoImageMsg()で変換。
        //image_pub_.publish(cv_ptr3->toImageMsg());
    }

////////////////関数//////////////////
    // phaseの変更ともろもろの値の初期化
    void changePhase(std::string next_phase) {
        std::cout << "change phase!" << next_phase << std::endl;

        objects.clear();
        // 前のphaseの結果によって変更される値を処理する
        now_phase = next_phase;
        curve_detect_cnt = 0;
        phase_start_t = ros::Time::now();
        reachBottomLeftLaneLeftT = false;
        findLeftLaneLeftT = false;
        reachBottomRightLaneLeftT = false;
        reachBottomRightLaneRightT = false;
        line_lost_time = ros::Time::now();
    }

    // タイルは直進中(now_phase = "straight")のときみ検索する
    // タイルを見つけた時の処理は、dir（進行方角）の変更、次タイルの決定、now_phaseの変更、reachBottomObject類の初期化
    void searchTile() {
        // nextTileを検索
        // カーブを右に曲がるならfind_curveを探索
        if (map_data[next_tile_y][next_tile_x][0] == 3 &&
            (map_data[next_tile_y][next_tile_x][1] - now_dir + 4) % 4 == 2) {
            // if (line_lost_cnt > 10 && curve_detect_cnt > 1) {
            ros::Time now = ros::Time::now();
            if (now - line_lost_time > ros::Duration(RIGHT_CURVE_END_MARGIN_TIME)) {
                now_dir = (now_dir + 1) % 4;
                setNextTile();
                changePhase("turn_right");
            }
        }
        if (map_data[next_tile_y][next_tile_x][0] == 3 &&
            (map_data[next_tile_y][next_tile_x][1] - now_dir + 4) % 4 == 3) {
            // 左カーブ
            // if (line_lost_cnt > 10 && curve_detect_cnt > 1) {
            if (reachBottomLeftLaneLeftT) {
                now_dir = (now_dir + 3) % 4;
                setNextTile();
                changePhase("turn_left");
            }
        }
    }


    // タイルが見つかったときに呼び出される
    // 今next_tileとなっているものを現在位置とし、今の方向と現在位置から次のタイル目標を決定する
    // road4は特徴のない直線のため無視する
    void setNextTile() {
        int next_x = next_tile_x;
        int next_y = next_tile_y;

        bool find_tile = false;

        // road4をスキップするために繰り返す
        while (!find_tile) {

            // 0が南で右回り, 原点は左上
            switch (now_dir) {
                case 0:
                    next_x = next_x;
                    next_y = next_y + 1;
                    break;
                case 1:
                    next_x = next_x - 1;
                    next_y = next_y;
                    break;
                case 2:
                    next_x = next_x;
                    next_y = next_y - 1;
                    break;
                default:
                    next_x = next_x + 1;
                    next_y = next_y;
                    break;
            }

            // road4(ただの直線)でないかチェック
            // 今だけカーブ検索中！！！！
            if (map_data[next_y][next_x][0] == 3) {
                find_tile = true;
            }
        }

        // next_tileの更新
        next_tile_x = next_x;
        next_tile_y = next_y;
    }
    /////////実際に動かす関数//////////////////

    // 決め打ちで左カーブ
    // 入射時の速度でカーブ時間を変更
    void leftTurn() {
        twist.linear.x = LEFT_CURVE_VEL;
        twist.angular.z = LEFT_CURVE_ROT;
        ros::Time now = ros::Time::now();
        if (now - phase_start_t > ros::Duration(LEFT_CURVE_END_TIME + LEFT_CURVE_END_MARGIN_TIME)) {
            changePhase("search_line");
        } else if (now - phase_start_t > ros::Duration(LEFT_CURVE_END_TIME)) {
            twist.angular.z = LEFT_CURVE_AFTER_ROT;
        }
        twist_pub.publish(twist);
    }
    /*

    // 障害物検知
    // 決め打ちで右にカーブし、決め打ちで左に戻る
    void  obstacleAvoidance() {
      ros::Time now = ros::Time::now();
      //　右車線に向けて回転
      if (now - phase_start_t <  ros::Duration(AVOLD_ROT_TIME)) {
        twist.linear.x = AVOID_OBSTACLE_VEL;
        twist.angular.z = AVOID_OBSTACLE_ROT;
      } else if(now - phase_start_t <  ros::Duration(AVOLD_ROT_TIME + AVOID_ROT_STRAIGHT))
      { // 右車線に向けて直進
        twist.linear.x = AVOID_OBSTACLE_VEL;
        twist.angular.z = 0;
      } else if(now - phase_start_t <  ros::Duration(AVOLD_ROT_TIME * 2 + AVOID_ROT_STRAIGHT))
      { // 右車線に対して水平になるように回転
        twist.linear.x = AVOID_OBSTACLE_VEL;
        twist.angular.z = -1 * AVOID_OBSTACLE_ROT;
      } else if(now - phase_start_t <  ros::Duration(AVOLD_ROT_TIME * 2 + AVOID_ROT_STRAIGHT + AVOID_STRAIGHT_TIME))
      { // 右車線を直進
        twist.linear.x = AVOID_OBSTACLE_VEL;
        twist.angular.z = 0;
      } else if(now - phase_start_t <  ros::Duration(AVOLD_ROT_TIME * 3 + AVOID_ROT_STRAIGHT + AVOID_STRAIGHT_TIME))
      { // 左車線に向けて回転
        twist.linear.x = AVOID_OBSTACLE_VEL;
        twist.angular.z = -1 * AVOID_OBSTACLE_ROT;
      } else if(now - phase_start_t <  ros::Duration(AVOLD_ROT_TIME * 3 + AVOID_ROT_STRAIGHT * 3 + AVOID_STRAIGHT_TIME))
      { // 左車線に向けて直進
        twist.linear.x = AVOID_OBSTACLE_VEL;
        twist.angular.z = 0;
      } else if(now - phase_start_t <  ros::Duration(AVOLD_ROT_TIME * 4 + AVOID_ROT_STRAIGHT * 2 + AVOID_STRAIGHT_TIME))
      { //左車線と水平になるように回転
        twist.linear.x = AVOID_OBSTACLE_VEL;
        twist.angular.z = AVOID_OBSTACLE_ROT;
      } else {
        changePhase("straight");
      }
      twist_pub.publish(twist);
    }
  */
    // カーブを曲がるときにラインを追跡して挙動決定
    // 交差点で曲がる時はまた別
    void rightCurveTrace(cv::Mat road_binary) {

        int before_line_x = detected_line_x;

        // detected_line_xをラインの左下から更新する
        updateLeftLine(road_binary);

        // ロボットの速度決定
        twist.linear.x = 0.2;
        twist.angular.z = (BIRDSEYE_LENGTH * RUN_LINE - detected_line_x) / 40;
        twist_pub.publish(twist);

        // 終了処理
        ros::Time now = ros::Time::now();
        if (now - phase_start_t > ros::Duration(RIGHT_CURVE_END_TIME) && find_left_line) {
            changePhase("straight");
        } else if (now - phase_start_t > ros::Duration(RIGHT_CURVE_END_TIME + RIGHT_CURVE_END_MARGIN_TIME)) {
            changePhase("straight");
        }
    }


    // もし次のタイルがTもしくは左カーブの場合、mostUnderLeftLaneLeftTから更新する
    // 白に二値化された画像から一番左下のラインを読み取ってdetected_line_xを更新する
    void updateLeftLine(cv::Mat road_binary) {
        cv::Mat road_hough;
        cv::Canny(road_binary, road_hough, 50, 200, 3);

        // 白を見つけたらbaseを更新して、10ピクセル以内にまた白があればそれを仮の白線とする。
        int temp_detected_line = detected_line_x;


        // 複数あった場合、直前のライントレースの結果との差を利用する
        int temp_dif = BIRDSEYE_LENGTH / 2;
        // uchar *road_hough_bottom = road_hough.ptr<uchar>(BIRDSEYE_LENGTH - 1);
        for (int i = 0; i < detected_line_x + BIRDSEYE_LENGTH / 10; i++) {
            int p = road_hough.at<uchar>(BIRDSEYE_LENGTH - 1, i);
            if (p) {
                for (int j = i + 1; j < i + BIRDSEYE_LENGTH / 10; j++) {
                    int q = road_hough.at<uchar>(BIRDSEYE_LENGTH - 1, j);
                    if (q) {
                        int this_dif = (i + j) / 2 - detected_line_x;
                        if (this_dif < temp_dif) {
                            temp_dif = this_dif;
                            temp_detected_line = (i + j) / 2;
                        }
                    }
                }
            }
        }
        detected_line_x = temp_detected_line;
    }


    // 確率ハフ変換によってラインを得る
    std::vector <cv::Vec4i> getHoughLinesP(cv::Mat image, int threshold, double minLineLength, double maxLineGap) {
        // 左側をハフ変換
        cv::Mat temp_dst, temp_color_dst;
        cv::Canny(image, temp_dst, 50, 200, 3);
        // cv::cvtColor( temp_dst, temp_color_dst, CV_GRAY2BGR );
        std::vector <cv::Vec4i> lines;
        cv::HoughLinesP(temp_dst, lines, 1, CV_PI / 180, threshold, minLineLength, maxLineGap);
        return lines;
    }

    // ライン検出の結果によって左右に操作
    // ラインがあればtrue
    // intは+1で左, 0で直進, -1で右
    // ラインが見つからなければ左に回転
    void lineTrace(int vel, int dir) {

        twist.linear.x += 0.01 * vel;
        twist.angular.z = 0.1 * dir;

        if (twist.linear.x > BURGER_MAX_LIN_VEL) twist.linear.x = BURGER_MAX_LIN_VEL;
        if (twist.linear.x < 0) twist.linear.x = 0;
        if (twist.angular.z > BURGER_MAX_ANG_VEL) twist.angular.z = BURGER_MAX_ANG_VEL;
        if ((twist.angular.z < 0 - BURGER_MAX_ANG_VEL)) twist.angular.z = 0 - BURGER_MAX_ANG_VEL;
        twist_pub.publish(twist);
    }

    void searchLine() {

        ros::Time now = ros::Time::now();
        if (find_left_line) {
            changePhase("straight");
            twist.angular.z = -1 * twist.angular.z;
            twist_pub.publish(twist);
            return;
        }
        std::cout << "line search" << std::endl;


        // 三秒ごとに首を振る向きを変える
        if (now - phase_start_t < ros::Duration(3.0)) {
            twist.linear.x = 0;
            twist.angular.z = -0.1;
        } else if (now - phase_start_t < ros::Duration(6.0)) {
            twist.angular.z = 0.1;
        } else if (now - phase_start_t < ros::Duration(7.0)) {
            twist.angular.z = 0.3;
        } else if (now - phase_start_t < ros::Duration(9.0)) {
            twist.angular.z = 0.3;
        }else {
            phase_start_t = ros::Time::now();
            std::cout << "one more search" << std::endl;
        }

        twist_pub.publish(twist);
    }

    // 極座標変換
    cv::Mat polarCoordinateConversion(cv::Mat image) {
        cv::Mat polar_dst;
        polar_dst = image.clone();
        int h = BIRDSEYE_LENGTH;
        int w = BIRDSEYE_LENGTH;
        float r, theta;
        for (int y = 0; y < h; y++) {
            for (int x = 0; x < w; x++) {
                theta = (float) M_PI * (1.0f - (float) y / h) / 2.0f;
                r = (float) w - x;
                if (theta < 1e-15) continue;
                int src_x = (float) w - r * cos(theta);
                int src_y = (float) h - r * sin(theta);
                //int src_x = (float)w - r;
                //int src_y = (float)h - (float)h*2.0f*theta/M_PI;
                if (0 <= src_x && src_x <= w - 1 && 0 <= src_y && src_y <= h - 1) {
                    cv::Vec3b *src_point = image.ptr<cv::Vec3b>(src_y);
                    cv::Vec3b *dst_point = polar_dst.ptr<cv::Vec3b>(y);

                    dst_point[x + 0] = src_point[src_x + 0];
                    dst_point[x + 1] = src_point[src_x + 1];
                    dst_point[x + 2] = src_point[src_x + 2];
                } else {
                    cv::Vec3b *dst_point = polar_dst.ptr<cv::Vec3b>(y);

                    dst_point[x + 0] = 0;
                    dst_point[x + 1] = 0;
                    dst_point[x + 2] = 0;
                }
            }
        }
        return polar_dst;
    }


    // imageを渡して俯瞰画像を得る
    cv::Mat birdsEye(cv::Mat image) {
        int width = image.size().width;
        int height = image.size().height;
        // 奥行の広さ（小さいほど狭い）
        float width_ratio = WIDTH_RATIO;
        // 上部
        float height_h = HEIGHT_H;
        // 下部
        float height_l = HEIGHT_L;
        // 画素値
        int result_size = BIRDSEYE_LENGTH;
        cv::Mat map_matrix, dst_image;
        cv::Point2f src_pnt[4], dst_pnt[4];

        src_pnt[0] = cv::Point(width * (0.5 - width_ratio), height * height_h);
        src_pnt[1] = cv::Point(0, height * height_l);
        src_pnt[2] = cv::Point(width * (0.5 + width_ratio), height * height_h);
        src_pnt[3] = cv::Point(width, height * height_l);

        dst_pnt[0] = cv::Point(0, 0);
        dst_pnt[1] = cv::Point(0, result_size);
        dst_pnt[2] = cv::Point(result_size, 0);
        dst_pnt[3] = cv::Point(result_size, result_size);

        map_matrix = cv::getPerspectiveTransform(src_pnt, dst_pnt);
        cv::warpPerspective(image, dst_image, map_matrix, cv::Size(result_size, result_size));
        return dst_image;
    }

    // imageを渡して中央線の画像を得る
    //　精度が甘くても、一番下から表示できるように設定
    cv::Mat birdsEyeAround(cv::Mat image) {
        int width = image.size().width;
        int height = image.size().height;
        // 奥行の広さ（小さいほど狭い）
        float width_ratio = WIDTH_RATIO;
        // 上部
        float height_h = HEIGHT_H;
        // 下部
        float height_l = HEIGHT_L;
        // 画素値
        int result_size = BIRDSEYE_LENGTH;
        cv::Mat map_matrix, dstImageCenter;
        cv::Point2f src_pnt[4], dst_pnt[4];

        src_pnt[0] = cv::Point(width * (0.5 - width_ratio), height * height_h);
        src_pnt[1] = cv::Point(width * (0 - (1 - height_l) * (0.5 - width_ratio) / (height_l - height_h)), height);
        src_pnt[2] = cv::Point(width * (0.5 + width_ratio), height * height_h);
        src_pnt[3] = cv::Point(width * (1 + (1 - height_l) * (0.5 - width_ratio) / (height_l - height_h)), height);

        dst_pnt[0] = cv::Point(result_size, 0);
        dst_pnt[1] = cv::Point(result_size, result_size);
        dst_pnt[2] = cv::Point(result_size * 2, 0);
        dst_pnt[3] = cv::Point(result_size * 2, result_size);

        map_matrix = cv::getPerspectiveTransform(src_pnt, dst_pnt);
        cv::warpPerspective(image, dstImageCenter, map_matrix, cv::Size(result_size * 3, result_size), cv::INTER_LINEAR,
                            cv::BORDER_CONSTANT, cv::Scalar::all(100));
        return dstImageCenter;
    }

    // 画像の中から一番下の障害物を検知
    // wideViewから検索
    // まず、赤色っぽいものの二値化を行う
    // 
    void detectObstacle(){
        
    }


// 二点をSTRAIGHT構造体で返す
    STRAIGHT toStraightStruct(cv::Vec4i line) {
        STRAIGHT result;

        //中点
        result.middle = cv::Point((line[0] + line[2]) / 2, (line[1] + line[3]) / 2);
        // 距離
        result.length = (line[0] - line[2]) * (line[0] - line[2]) + (line[1] - line[3]) * (line[1] - line[3]);
        // 角度
        float radian = atan2(line[1] - line[3], line[0] - line[2]);
        // radianからdegree

        if (radian * 180.0 / PI >= 90) {
            result.degree = radian * 180.0 / PI - 90;
        } else {
            result.degree = radian * 180.0 / PI + 90;
        }
        return result;
    }

// 二点間の傾きを求め、長さをかけて重さとする
// x1 y1, x2 y2
    float lineWeight(cv::Vec4i line) {
        // 距離
        float distance = (line[0] - line[2]) * (line[0] - line[2]) + (line[1] - line[3]) * (line[1] - line[3]);

        // 角度
        float radian = atan2(line[1] - line[3], line[0] - line[2]);

        // radianからdegree
        float degree = radian * 180.0 / PI;

        return degree;
    }


// 白色検出（返り値はRGB）
    cv::Mat whiteBinary(cv::Mat image) {
        cv::Mat color_mask, result_image, hsv_image;
        cv::cvtColor(image, hsv_image, CV_BGR2HSV);
        cv::inRange(hsv_image, cv::Scalar(Hue_l, Saturation_l, Lightness_l, 0),
                    cv::Scalar(Hue_h, Saturation_h, Lightness_h, 0), color_mask);
        cv::bitwise_and(image, image, result_image, color_mask);

        return result_image;
    }


    // 水平のラインと縦のラインが近しいか計測
    // 片方の点がもう一つの線のx,yで形作られる◇の中に存在するかどうかで判別
    // 1なら右、-1なら左を示す
    int crossCheck(cv::Vec4i horiLine, cv::Vec4i verLine) {
        int dir = 0;
        if ((horiLine[0] > verLine[0] - 5) && (horiLine[0] < verLine[2] + 5)) {
            if ((horiLine[1] > verLine[1] - 5) && (horiLine[1] < verLine[3] + 5)) {
                dir = 1;
            } else if ((horiLine[1] > verLine[3] - 5) && (horiLine[1] < verLine[1] + 5)) {
                dir = 1;
            }
        } else if ((horiLine[2] > verLine[0] - 5) && (horiLine[2] < verLine[2] + 5)) {
            if ((horiLine[3] > verLine[1] - 5) && (horiLine[3] < verLine[3] + 5)) {
                dir = -1;
            } else if ((horiLine[3] > verLine[3] - 5) && (horiLine[3] < verLine[1] + 5)) {
                dir = -1;
            }
        }
        return dir;
    }


    // 画像から交差点を検知し、オブジェクトリストに追加、更新する。
    void intersectionDetection(std::vector <cv::Vec4i> lines, cv::Mat whiteImg) {
        // 右画像に対して,Tや十が存在するかサーチする
        for (int i = 0; i < lines.size(); i++) {
            STRAIGHT right_line = toStraightStruct(lines[i]);
            // 水平に近い
            if (right_line.degree > 80 || right_line.degree < -80) {
                for (int j = 0; j < lines.size(); j++) {
                    if (j == i) continue;
                    STRAIGHT check_line = toStraightStruct(lines[j]);

                    // 比較する対象を垂直に絞る
                    if (check_line.degree < 20 && check_line.degree > -20) {
                        int dir = (crossCheck(lines[i], lines[j]));
                        // pointがpoint二点の間にあるかどうか調べる関数
                        if (dir == 1 & lines[i][1] > 30) { // 右に伸びていて、かつある程度下にある場合
                            addObject("right_T", lines[i][0], lines[i][1]);
                            cv::line(whiteImg, cv::Point(lines[i][0], lines[i][1]),
                                     cv::Point(lines[i][2], lines[i][3]), cv::Scalar(0, 255, 0), 3, 8);
                            cv::line(whiteImg, cv::Point(lines[j][0], lines[j][1]),
                                     cv::Point(lines[j][2], lines[j][3]), cv::Scalar(0, 255, 0), 3, 8);
                        } else if (dir == -1 && lines[i][3] > 30) {
                            if (lines[i][2] > BIRDSEYE_LENGTH * 0.5 && lines[i][2] < BIRDSEYE_LENGTH * 1.5) {
                                addObject("left_lane_left_T", lines[i][2], lines[i][3]);
                            } else if (lines[i][2] > BIRDSEYE_LENGTH * 1.5 && lines[i][2] < BIRDSEYE_LENGTH * 2.5) {
                                addObject("right_lane_left_T", lines[i][2], lines[i][3]);
                            }
                            cv::line(whiteImg, cv::Point(lines[i][0], lines[i][1]),
                                     cv::Point(lines[i][2], lines[i][3]), cv::Scalar(0, 0, 255), 3, 8);
                            cv::line(whiteImg, cv::Point(lines[j][0], lines[j][1]),
                                     cv::Point(lines[j][2], lines[j][3]), cv::Scalar(0, 0, 255), 3, 8);
                        }
                    }
                    /*
                    cv::line( right_roi, cv::Point(lines[i][0], lines[i][1]),
                              cv::Point(lines[i][2], lines[i][3]), cv::Scalar(0,255,0), 3, 8 );
                    cv::line( right_roi, cv::Point(right_lines[j][0], right_lines[j][1]),
                              cv::Point(right_lines[j][2], right_lines[j][3]), cv::Scalar(0,255,0), 3, 8 );
                    */
                }
            }
        }
    }

    /*
    // 道路から赤色の物体を検知し、オブジェクトリストに追加、更新する。
    void redObjectDetectionOnRoad(cv::Mat image)
    {
      // 画像から赤色抽出
      // y軸ごとに
        // 右画像に対して,Tや十が存在するかサーチする
        for( int i = 0; i < right_lines.size(); i++ )
        {
            STRAIGHT right_line = toStraightStruct(right_lines[i]);
            // 水平に近い
            if (right_line.degree > 80 || right_line.degree < -80) {
                for( int j = 0; j < right_lines.size(); j++ ) {
                    if (j == i) continue;
                    STRAIGHT check_line = toStraightStruct(right_lines[j]);

                    // 比較する対象を垂直に絞る
                    if (check_line.degree < 20 && check_line.degree > -20) {
                        int dir = (crossCheck(right_lines[i], right_lines[j]));
                        // pointがpoint二点の間にあるかどうか調べる関数
                        if (dir == 1) { // 右に伸びている場合
                            addObject("right_T", right_lines[i][0], right_lines[i][1]);
                        } else if (dir == -1) {
                            addObject("left_T", right_lines[i][2], right_lines[i][3]);
                        }
                    }
                }
            }
        }
    }
    */


    // オブジェクトを発見した時、それが以前発見されたものと一致するかどうかを調べ、一致しなかったら追加
    // 一致する場合タイムスタンプと位置を更新し、カウントを1増やす
    void addObject(std::string objType, int objectX, int objectY) {
        bool findObj = false;
        /*
        for(OBJECT compare : objects) {
          if( compare.objType == objType && (objectX > compare.beforeX - 15) && (std::abs(objectY - compare.beforeY) < 30)) {
            compare.beforeX = objectX;
            compare.beforeY = objectY;
            compare.findCnt += 1;
            compare.timeStamp = ros::Time::now();
            findObj = true;
            break;
          }
        }
         */
        std::list<OBJECT>::iterator itr;
        for (itr = objects.begin(); itr != objects.end();) {
            OBJECT compare = *itr;
            if (compare.objType == objType && (objectX > compare.beforeX - 5) &&
                (std::abs(objectY - compare.beforeY) < 10)) {
                compare.beforeX = objectX;
                compare.beforeY = objectY;
                compare.findCnt += 1;
                compare.timeStamp = ros::Time::now();
                *itr = compare;
                findObj = true;
                break;
            }
            itr++;
        }
        if (!findObj) {
            OBJECT obj = {objType, objectX, objectY, 1, ros::Time::now()};
            objects.push_back(obj);
        }
    }


    // オブジェクトが一定時間発見されていなければ破棄
    void updateObject() {
        ros::Time now = ros::Time::now();
        std::list<OBJECT>::iterator itr;
        int objCnt;
        int mostUnderLeftLaneLeftT_y = 0;
        bool nowFindLeftLaneLeftT = false;
        mostUnderLeftLaneLeftT = detected_line_x;


        for (itr = objects.begin(); itr != objects.end();) {
            OBJECT obj = *itr;

            // オブジェクトが下に到達する時刻を推定し、下に到達したと推定された場合アクションのためのフラグを立てる
            // タイルを進める、左に曲がる等をsearchTile()で行う
            if (obj.objType == "left_lane_left_T") { // 今だけテスト
                double  tempTime= ((1 - ((double)obj.beforeY) / BIRDSEYE_LENGTH) * 4  + INTERSECTION_PREDICTION_UNDER_MARGIN) * INTERSECTION_PREDICTION_TIME_RATIO * (0.2 / (twist.linear.x + 0.001));
                std::cout << obj.beforeY << "   " << now - obj.timeStamp -  ros::Duration(tempTime) << std::endl;
                if (now - obj.timeStamp > ros::Duration(tempTime)) {
                    std::cout << "find object! beforeY = " << obj.beforeY << std::endl;
                    if (obj.findCnt > 1) {
                        if (obj.objType == "left_lane_left_T") {
                            reachBottomLeftLaneLeftT = true;
                        } else if (obj.objType == "right_lane_left_T") {
                            reachBottomRightLaneLeftT = true;
                        } else if (obj.objType == "right_T") {
                            reachBottomRightLaneRightT = true;
                        }
                    }
                    itr = objects.erase(itr);
                    continue;
                }
            }
            if (obj.objType == "left_lane_left_T") {
                nowFindLeftLaneLeftT = true;
                if (mostUnderLeftLaneLeftT_y > obj.beforeY) {
                    mostUnderLeftLaneLeftT_y = obj.beforeY;
                    mostUnderLeftLaneLeftT = obj.beforeX - BIRDSEYE_LENGTH; // BIRDSEYE_LENGTH分だけ右にずれているため
                    std::cout << "mostUnderLeftLaneLeftT   " <<  mostUnderLeftLaneLeftT << std::endl;
                }
            }
            itr++;
            objCnt++;
        }

        // メンバ変数を更新
        if (nowFindLeftLaneLeftT) {
            findLeftLaneLeftT = true;
        } else {
            findLeftLaneLeftT = false;
        }
    }
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "image_converter");
    ImageConverter ic;
    ros::spin();
    return 0;
}

