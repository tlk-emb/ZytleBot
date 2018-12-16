#include <fstream>
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
#include <string>
#include <cstdlib>
#include <typeinfo>

// pcam使用時
#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/UInt8MultiArray.h"


#define PI 3.141592653589793
#define DEBUG true

// 最初の交差点もしくはカーブ位置 dirは以下のように0が南で右回り
// map_dataは下で書き換える必要がある
// 2
//1 3
// 0
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
    double beforeX;
    double beforeY;
    int findCnt;
    ros::Time timeStamp;
} OBJECT;


// 直線を中点、傾き、長さで表す
typedef struct straight {
    cv::Point2f middle;
    double degree;
    double length;
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

// ちょっと多めに書かないとセグフォ起こす気がする
int intersectionDir[100] = {0};

/*
 * TODO
 * 右カーブ直後の横断歩道の認識が苦手なため、afterCurveSkipフラグによってスキップさせている
 */

class ImageConverter {
    ros::NodeHandle nh_;
    /* if zybo
    ros::Subscriber image_sub_;
    */
    // if_pc
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;

    // 定数宣言
    int BIRDSEYE_LENGTH, CAMERA_WIDTH, CAMERA_HEIGHT;

    double BURGER_MAX_LIN_VEL, BURGER_MAX_ANG_VEL, RIGHT_CURVE_START_LOST_LINE_TIME, RIGHT_CURVE_END_MARGIN_TIME, RIGHT_CURVE_END_TIME,
            RIGHT_CURVE_VEL , RIGHT_CURVE_ROT , LEFT_CURVE_END_TIME , LEFT_CURVE_END_MARGIN_TIME , LEFT_CURVE_VEL , LEFT_CURVE_ROT , LEFT_CURVE_AFTER_ROT ,
            AVOID_OBSTACLE_VEL , AVOID_OBSTACLE_ROT , AVOID_ROT_TIME , AVOID_ROT_STRAIGHT , AVOID_STRAIGHT_TIME , AVOID_BEFORE_STRAIGHT_MARGIN_TIME , INTERSECTION_PREDICTION_TIME_RATIO ,
            CROSSWALK_UNDER_MARGIN , INTERSECTION_PREDICTION_UNDER_MARGIN , INTERSECTION_CURVE_START_FLAG_RATIO , RUN_LINE , RUN_LINE_MARGIN , WIDTH_RATIO , HEIGHT_H , HEIGHT_L, INTERSECTION_STRAIGHT_TIME;


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
    std::string nextSearchObject;

    // 発見したオブジェクト（交差点、障害物）のリスト
    std::list <OBJECT> objects;


    // 次のtileを保存
    int next_tile_x;
    int next_tile_y;
    int now_dir;

    // 検出された直線のx座標
    int detected_line_x;

    ros::Time phaseStartTime;
    ros::Time tileUpdatedTime;
    ros::Time line_lost_time;
    ros::Time cycleTime;

    // change phaseで初期化
    // bottomにオブジェクトが到達したかどうか
    bool reachBottomRightLaneRightT;
    bool reachBottomRightLaneLeftT;
    bool reachBottomLeftLaneLeftT;
    bool reachBottomLeftLaneStraightEnd;

    // 交差点の挙動決定のための配列の位置
    int nowIntersectionCount;

    // BIRDS_EYE_LENGTHの3/4に右のT字路が到達したかどうか
    bool intersectionDetectionFlag;

    // 横断歩道の（停止線）の位置に来た時trueになる
    bool crosswalkFlag;


    // カーブの次が横断歩道の場合、カーブ終了後横断歩道を認識するまで少しストップ
    bool curveAfterCrosswalk;

    double mileage;
    double phaseRunMileage;
    double detected_angle;


    // 加速するかしないか
    bool acceleration;

    // テンプレートマッチングで探す形
    std::string searchType;

    int mostUnderLeftLaneLeftT;

    XmlRpc::XmlRpcValue params;

    cv::Mat template_right_T;
    cv::Mat template_left_T;
    cv::Mat template_under_T;
    cv::Mat template_crosswalk;
    cv::Mat template_curve;
    cv::Mat template_intersection;

    cv::Mat aroundDebug;

    // デバッグ
    cv::Mat display;


public:
    // コンストラクタ
    ImageConverter()
    // if pc
            : it_(nh_) {


        nh_.getParam("/zybo_autorace/autorace", params);

        // 定数をセット

        Hue_l = (int)params["hue_l"];
        Hue_h = (int)params["hue_h"];
        Saturation_l = (int)params["saturation_l"];
        Saturation_h = (int)params["saturation_h"];
        Lightness_l = (int)params["lightness_l"];
        Lightness_h = (int)params["lightness_h"];
        line_lost_cnt = 0;
        next_tile_x = (int)params["next_x"];
        next_tile_y = (int)params["next_y"];
        now_dir = (int)params["start_dir"];




        BURGER_MAX_LIN_VEL = (double)params["burger_max_lin_vel"];
        BURGER_MAX_ANG_VEL = (double)params["burger_max_ang_vel"];
        INTERSECTION_STRAIGHT_TIME =(double)params["intersection_straight_time"];

        RIGHT_CURVE_START_LOST_LINE_TIME = (double)params["right_curve_start_lost_line_time"];
        RIGHT_CURVE_END_MARGIN_TIME = (double)params["right_curve_end_margin_time"];
        RIGHT_CURVE_END_TIME = (double)params["right_curve_end_time"];

        RIGHT_CURVE_VEL = (double)params["right_curve_vel"];
        RIGHT_CURVE_ROT = (double)params["right_curve_rot"];

        LEFT_CURVE_END_TIME = (double)params["left_curve_end_time"];
        LEFT_CURVE_END_MARGIN_TIME = (double)params["left_curve_end_margin_time"];

        LEFT_CURVE_VEL = (double)params["left_curve_vel"];
        LEFT_CURVE_ROT = (double)params["left_curve_rot"];
        LEFT_CURVE_AFTER_ROT = (double)params["left_curve_after_rot"];
        AVOID_OBSTACLE_VEL = (double)params["avoid_obstacle_vel"];
        AVOID_OBSTACLE_ROT = (double)params["avoid_obstacle_rot"];
        AVOID_ROT_TIME = (double)params["avoid_rot_time"];


        AVOID_ROT_STRAIGHT = (double)params["avoid_rot_straight"];
        AVOID_STRAIGHT_TIME = (double)params["avoid_straight_time"];
        AVOID_BEFORE_STRAIGHT_MARGIN_TIME = (double)params["avoid_before_straight_margin_time"];
        INTERSECTION_PREDICTION_TIME_RATIO = (double)params["intersection_prediction_time_ratio"];
        INTERSECTION_CURVE_START_FLAG_RATIO = (double)params["intersection_curve_start_flag_ratio"];
        CROSSWALK_UNDER_MARGIN = (double)params["crosswalk_under_margin"];
        INTERSECTION_PREDICTION_UNDER_MARGIN = (double)params["intersection_prediction_under_margin"];
        RUN_LINE = (double)params["run_line"];
        RUN_LINE_MARGIN = (double)params["run_line_margin"];
        WIDTH_RATIO = (double)params["width_ratio"];
        HEIGHT_H = (double)params["height_h"];
        HEIGHT_L  = (double)params["height_l"];

        BIRDSEYE_LENGTH = (int)params["birdseye_length"];
        CAMERA_WIDTH = (int)params["camera_width"];
        CAMERA_HEIGHT = (int)params["camera_height"];

        std::string project_folder = (std::string)params["project_folder"] + "/image/sozai1.png";

        template_right_T = cv::imread((std::string)params["project_folder"] + "/image/right_T.png", 1);
        template_left_T = cv::imread((std::string)params["project_folder"] + "/image/left_T.png", 1);
        template_under_T = cv::imread((std::string)params["project_folder"] + "/image/under_T.png", 1);
        template_crosswalk = cv::imread((std::string)params["project_folder"] + "/image/crosswalk.png", 1);
        template_curve = cv::imread((std::string)params["project_folder"] + "/image/curve.png", 1);
        template_intersection = cv::imread((std::string)params["project_folder"] + "/image/intersection.png", 1);

        find_left_line = false;


        detected_line_x = 0;

        // start時間を初期化
        phaseStartTime = ros::Time::now();
        line_lost_time = ros::Time::now();
        tileUpdatedTime = ros::Time::now();
        cycleTime = ros::Time::now();

        now_phase = "straight";


        reachBottomRightLaneRightT = false;
        reachBottomRightLaneLeftT = false;
        reachBottomLeftLaneLeftT = false;
        reachBottomLeftLaneStraightEnd = false;
        mostUnderLeftLaneLeftT = 0;
        nowIntersectionCount = 0;
        phaseRunMileage = 0;
        detected_angle = 0;
        intersectionDetectionFlag = false;
        curveAfterCrosswalk = false;
        crosswalkFlag = false;

        searchType == "";

        acceleration = false;


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
        limitedTwistPub();

        setSearchType();
    }

    // デストラクタ
    ~ImageConverter() {
        twist.linear.x = 0.0;
        twist.angular.z = 0.0;
        limitedTwistPub();
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

        detected_angle = 0;

        cv::Mat hsv_image, color_mask, gray_image, birds_eye;

        // 俯瞰画像
        birds_eye = birdsEye(base_image);

        cv::Mat aroundImg, aroundWhiteBinary;
        aroundImg = birdsEyeAround(base_image);
        aroundWhiteBinary = whiteBinary(aroundImg);

        aroundDebug = aroundWhiteBinary.clone();

        std::vector <cv::Vec4i> around_lines = getHoughLinesP(aroundWhiteBinary, 0, 10, 5);

        display = aroundWhiteBinary.clone();

        cv::Mat road_white_binary(aroundWhiteBinary, cv::Rect(BIRDSEYE_LENGTH, 0, BIRDSEYE_LENGTH, BIRDSEYE_LENGTH));
        cv::Mat left_roi(aroundWhiteBinary, cv::Rect(BIRDSEYE_LENGTH / 2, 0, BIRDSEYE_LENGTH, BIRDSEYE_LENGTH));
        cv::Mat right_roi(aroundWhiteBinary, cv::Rect(BIRDSEYE_LENGTH * 1.5, 0, BIRDSEYE_LENGTH / 2, BIRDSEYE_LENGTH));


        // 走行距離を求める
        mileage = twist.linear.x * (double)(ros::Time::now().toSec() - cycleTime.toSec()) * INTERSECTION_PREDICTION_TIME_RATIO;
        phaseRunMileage += mileage;

        // processingStartTimeの更新
        ros::Time processingStartTime = ros::Time::now();

        cv::Mat road_clone = road_white_binary.clone();

        // 左レーンの発見フラグをリセット
        find_left_line = false;

        // デバッグ
        system("clear");
        std::cout << "現在のフェーズ : " << now_phase << std::endl;
        std::string direction;
        switch (now_dir) {
            case 0: direction = "南";
                break;
            case 1: direction = "西";
                break;
            case 2: direction = "北";
                break;
            case 3: direction = "東";
                break;
            default: direction = "error";
                break;
        }

        std::cout << "次の目的地 : x = " << next_tile_x << " y =  " << next_tile_y << " type=" << map_data[next_tile_x][next_tile_y][0] << std::endl;
        std::cout << "現在の進行方向  " << direction << std::endl;

        // ---------------controller----------------

        updateObject();

        if (now_phase == "straight") {
            ros::Time now = ros::Time::now();
            // if (now - line_lost_time > ros::Duration(4.0)) {
            if (false) {
                changePhase("search_line");
            } else {
                double degree_average = detectLane(left_roi);
                detected_angle = degree_average;
                // レーン検出してdetected_lineを更新、平均角度を求める
                findRedObs(birds_eye);
                intersectionDetectionByTemplateMatching(aroundWhiteBinary, degree_average);
                searchObject();
                lineTrace(degree_average, road_white_binary);
                limitedTwistPub();
            }
        } else if (now_phase == "trace_right_curve") {
            rightCurveTrace(road_white_binary);
        } else if (now_phase == "search_line") {
            double degree_average = detectLane(left_roi);
            searchLine();
        } else if (now_phase == "search_right_lane_right_T") {
            // bool nowFindRightLaneRightT = intersectionDetection(around_lines, aroundWhiteBinary);
            // searchRightLaneRightT(nowFindRightLaneRightT);
        } else if (now_phase == "turn_left") {
            leftTurn();
        } else if (now_phase == "turn_right") {
            //intersectionDetectionByTemplateMatching(aroundWhiteBinary, 0);
            //searchObject();
            curveDetectLane(aroundWhiteBinary);
        } else if (now_phase == "find_obs") {
            obstacleAvoidance(road_white_binary, aroundWhiteBinary);
        } else if (now_phase == "intersection_straight") {
            intersectionStraight(road_clone);
            limitedTwistPub();
        } else if (now_phase == "crosswalk") {
            crosswalkRedStop();
        }

        // ---------------controller end----------------

        // 以下デバッグ用
        testTemplateMatching(aroundWhiteBinary,  template_right_T, cv::Scalar(0, 0, 100));
        testTemplateMatching(aroundWhiteBinary,  template_left_T, cv::Scalar(0, 100, 100));
        testTemplateMatching(aroundWhiteBinary,  template_under_T, cv::Scalar(100, 0, 0));
        testTemplateMatching(aroundWhiteBinary,  template_crosswalk, cv::Scalar(0, 255, 100));
        testTemplateMatching(aroundWhiteBinary,  template_intersection, cv::Scalar(255, 0, 0));

        std::cout << "走行距離 : " << mileage << " 合計 "  << phaseRunMileage << std::endl;
        std::cout << "実行時間 : " << ros::Time::now().toSec() - processingStartTime.toSec() << "s" << std::endl;
        std::cout << "周期時間 : " << ros::Time::now().toSec() - cycleTime.toSec() << "s" << std::endl;
        // cycleTimeの更新
        cycleTime = ros::Time::now();


        cv::resize(base_image, base_image, cv::Size(), 0.5, 0.5);

        std::cout << "速度     : " <<twist.linear.x << " 角度 : " << twist.angular.z << std::endl;
        testOutputObject();
        cv::imshow("road", aroundDebug);
        cv::moveWindow("road", 20, 20);
        cv::imshow("origin", base_image);
        cv::moveWindow("origin", 400, 20);
        cv::waitKey(3);

    }

////////////////関数//////////////////
    // phaseの変更ともろもろの値の初期化
    void changePhase(std::string next_phase) {
        std::cout << "change phase!" << next_phase << std::endl;
        // 前のphaseの結果によって変更される値を処理する
        now_phase = next_phase;
        phaseStartTime = ros::Time::now();
        phaseRunMileage = 0;
        resetFlag();
    }

    void resetFlag() {
        objects.clear();
        curve_detect_cnt = 0;
        reachBottomLeftLaneLeftT = false;
        reachBottomRightLaneLeftT = false;
        reachBottomRightLaneRightT = false;
        intersectionDetectionFlag = false;
        reachBottomLeftLaneStraightEnd = false;
        crosswalkFlag = false;
        line_lost_time = ros::Time::now();
    }


    // 左車線について
    // 角度平均をとり、全体の角度が垂直になるようにする
    // 最も左車線を検出し、いい感じになるよう調整する
    // 車線が見つからない場合、find_left_lineがfalseになる
    double detectLane(cv::Mat left_roi){

        std::vector <cv::Vec4i> left_lines = getHoughLinesP(left_roi, 20, 40, 5);

        int average_cnt = 0;
        double degree_average_sum = 0;
        double most_left_middle_x = BIRDSEYE_LENGTH * 0.5;
        double degree_average = 0;
        int mostDistantY = BIRDSEYE_LENGTH;
        int mostDistantX = 0;

        // 垂直に近い点のみ線を引く
        for (size_t i = 0; i < left_lines.size(); i++) {
            STRAIGHT left_line = toStraightStruct(left_lines[i]);
            if (left_line.degree < 30 && left_line.degree > -30) {

                if (left_lines[i][1] < mostDistantY) {
                    mostDistantX = left_lines[i][0];
                    mostDistantY = left_lines[i][1];
                }
                if (left_lines[i][3] < mostDistantY) {
                    mostDistantX = left_lines[i][2];
                    mostDistantY = left_lines[i][3];
                }

                //cv::line(aroundDebug, cv::Point(left_lines[i][0] + BIRDSEYE_LENGTH * 0.5, left_lines[i][1]),
                //         cv::Point(left_lines[i][2] + BIRDSEYE_LENGTH * 0.5, left_lines[i][3]), cv::Scalar(0, 0, 255), 3, 8);

                degree_average_sum += left_line.degree;
                if (most_left_middle_x > std::abs(left_line.middle.x - BIRDSEYE_LENGTH * 0.5)) {
                    most_left_middle_x = std::abs(left_line.middle.x - BIRDSEYE_LENGTH * 0.5);
                    detected_line_x = left_line.middle.x - BIRDSEYE_LENGTH * 0.5;
                }
                find_left_line = true;
                average_cnt++;
            }
        }

        if (find_left_line) {
            // addMostDistantObject("left_lane_end", mostDistantX, mostDistantY); // 左車線の最も遠い点を直線の終点として保持しておく
            // reachBottomLeftLaneStraightEnd = false;
            line_lost_time = ros::Time::now();
            degree_average = degree_average_sum / average_cnt;
            cv::line(aroundDebug, cv::Point(detected_line_x + BIRDSEYE_LENGTH, 0),
                     cv::Point(detected_line_x + BIRDSEYE_LENGTH, BIRDSEYE_LENGTH), cv::Scalar(0, 255, 255), 3, 8);
        }

        std::cout << "左車線の検知 : " << find_left_line << " | 検知数 = " << average_cnt << std::endl;
        std::cout << "推定された左車線の位置 : " << detected_line_x << std::endl;
        std::cout << "全体の傾き : " << degree_average << std::endl;

        return degree_average;
    }

    /*
     * タイルは直進中(now_phase = "straight")のときみ検索する
     * タイルを見つけた時の処理は、dir（進行方角）の変更、次タイルの決定、now_phaseの変更、reachBottomObject類の初期化, 交差点ならnowIntersectionCountを進める
     * T字路などが画面下部に到達したことを利用するならばフラグをリセット
     *
     * tileの種類
     * road1 直進（線付き）
     * road2 横断歩道（交差点）
     * road3 カーブ
     * road4 直線
     * road5 横断歩道（交差点）
     * road6 横断歩道 (直進)
     * road7 T字路(1が右)
     * road8 十字路
     */
    void searchObject() {
        ros::Time now = ros::Time::now();

        // タイルの種類 1~8がそれぞれFPTのroad meshに対応
        int tileType = map_data[next_tile_y][next_tile_x][0];

        // タイルの回転 1が画像通りで0~3で表している
        int tileRot = map_data[next_tile_y][next_tile_x][1];

        // タイルと入射角の差　どの方角からタイルに侵入するかを判別
        int differenceDirection = (tileRot - now_dir + 4) % 4;
        // 交差点で次にどの方角へ向かうかが決められているので、それと現在の方角の差をとるために使う
        int nextDirection = (intersectionDir[nowIntersectionCount] - now_dir + 4) % 4;

        if (tileType == 3 && differenceDirection== 2) {
            // nextTileを検索
            // カーブを右に曲がるならfind_curveを探索
            if (now - line_lost_time > ros::Duration(RIGHT_CURVE_START_LOST_LINE_TIME)) {
                curveAfterCrosswalk = true;
                now_dir = (now_dir + 1) % 4;
                changePhase("trace_right_curve");
                setNextTile();
            }
        } else if (tileType == 3 && differenceDirection == 3) {
            // 左カーブ
            if (now - line_lost_time > ros::Duration(RIGHT_CURVE_START_LOST_LINE_TIME)) {
                now_dir = (now_dir + 3) % 4;
                changePhase("turn_left");
                setNextTile();
            }
        } else if (tileType == 2 || tileType == 5 || tileType == 6) {
            // 横断歩道
            if (crosswalkFlag) {
                std::cout << "横断歩道発見" << std::endl;
                changePhase("crosswalk");
                setNextTile();
            }
        } else if (intersectionDetectionFlag) {
            if (tileType == 7) { // T字路
                if (differenceDirection == 3) {
                    // T字路に左から入る
                    if (nextDirection == 0) { // 直進
                        nowIntersectionCount++;
                        changePhase("straight");
                        setNextTile();

                    } else { // 右に曲がる
                        nowIntersectionCount++;
                        now_dir = (now_dir + 1) % 4;
                        changePhase("turn_right");
                        setNextTile();

                    }
                } else if (differenceDirection == 0) {
                    // T字路の下から突き当りに向かって入った場合
                    if (nextDirection == 1) { // 右に曲がる
                        nowIntersectionCount++;
                        now_dir = (now_dir + 1) % 4;
                        changePhase("turn_right");
                        setNextTile();

                    } else { // 左に曲がる
                        nowIntersectionCount++;
                        now_dir = (now_dir + 3) % 4;
                        changePhase("turn_left");
                        setNextTile();

                    }
                } else { // T字路に右から入った場合
                    if (nextDirection == 0) { // 直進 左車線が消えるため、特殊な動作をさせる
                        nowIntersectionCount++;
                        changePhase("intersection_straight");
                        setNextTile();

                    }
                    if (nextDirection == 3) { // 左に曲がる
                        nowIntersectionCount++;
                        now_dir = (now_dir + 3) % 4;
                        changePhase("turn_left");
                        setNextTile();

                    }

                }

            } else if (tileType == 8) {
                if (nextDirection == 1) {
                    nowIntersectionCount++;
                    std::cout << "十字路を右に曲がる" << std::endl;
                    now_dir = (now_dir + 1) % 4;
                    changePhase("turn_right");
                    setNextTile();
                } else if (nextDirection == 3) {
                    nowIntersectionCount++;
                    now_dir = (now_dir + 3) % 4;
                    changePhase("turn_left");
                    setNextTile();
                }
            } else {
                nowIntersectionCount++;
                changePhase("intersection_straight");
                setNextTile();
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

            // 今の進行方向によって次のタイルを検索
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
            int nextTile = map_data[next_y][next_x][0];

            // カーブの後はcurveAfterCrosswalkがtrueになっているので、直後のnextTileが横断歩道の時のみtrueのまま
            // 別の場合はcurveAfterCrosswalkをfalseにする
            // if (nextTile == 2 || nextTile == 5 || nextTile == 6 || nextTile == 7 || nextTile == 8) {
            if (nextTile == 3 || nextTile == 7 || nextTile == 8) {
                find_tile = true;
            }
        }

        // next_tileの更新
        next_tile_x = next_x;
        next_tile_y = next_y;

        std::cout << "next tile " << next_x << " " << next_y << "   type=" << map_data[next_y][next_x][0] << std::endl;
        std::cout << "now dir  " << now_dir << std::endl;

        setSearchType();
    }

    // 次に探すべき模様を決定する
    void setSearchType() {
        // searchTypeの更新
        // タイルの種類 1~8がそれぞれFPTのroad meshに対応
        int tileType = map_data[next_tile_y][next_tile_x][0];

        // タイルの回転 1が画像通りで0~3で表している
        int tileRot = map_data[next_tile_y][next_tile_x][1];

        // タイルと入射角の差　どの方角からタイルに侵入するかを判別
        int differenceDirection = (tileRot - now_dir + 4) % 4;

        if (tileType == 2 || tileType == 5 || tileType == 6) {
            // 横断歩道
            searchType = "crosswalk";
        } else if (tileType == 7) { // T字路
            if(differenceDirection == 3) {
                // T字路に左から入る
                searchType = "right_T";
            } else if(differenceDirection == 0) {
                // T字路の下から突き当りに向かって入った場合
                searchType = "under_T";
            } else {
                // T字路に右から入った場合
                searchType = "left_T";
            }
        } else if (tileType == 8) {
            searchType = "intersection";
        } else {
            searchType = "";
        }
    }
    /////////実際に動かす関数//////////////////

    // 5秒ストップ
    void crosswalkRedStop() {
        ros::Time now = ros::Time::now();
        twist.linear.x = 0;
        twist.angular.z = 0;

        limitedTwistPub();
        if (now - phaseStartTime >  ros::Duration(5.0)) {
            changePhase("straight");
        }
    }

    // 直線
    // 傾きからまっすぐ走らせる
    void intersectionStraight(cv::Mat roadRoi) {
        ros::Time now = ros::Time::now();
        //　右車線に向けて回転
        if (now - phaseStartTime >  ros::Duration(INTERSECTION_STRAIGHT_TIME)) {
            changePhase("straight");
        }
        std::vector <cv::Vec4i> lines = getHoughLinesP(roadRoi, 0, 10, 5);

        double averageDegreeSum = 0;
        double averageCnt = 0.0001;

        for (int i = 0; i < lines.size(); i++) {
            STRAIGHT line = toStraightStruct(lines[i]);
            // 垂直に近い
            if (line.degree > -20 && line.degree < 20) {
                averageDegreeSum += line.degree;
                averageCnt += 1;
                // デバッグ
                cv::line(roadRoi, cv::Point(lines[i][0], lines[i][1]),
                         cv::Point(lines[i][2], lines[i][3]), cv::Scalar(0, 255, 0), 3, 8);
            }
        }

        double averageDegree = averageDegreeSum / averageCnt;

        if (averageDegree < -10 || averageDegree > 10) {
            // 角度平均が-5以上なら左に曲がる、5以上なら右に曲がる
            twist.angular.z = averageDegree * -0.01;
        }
        twist.linear.x = 0.2;
    }

    // 決め打ちで左カーブ
    // 入射時の速度でカーブ時間を変更
    void leftTurn() {
        twist.linear.x = LEFT_CURVE_VEL;
        twist.angular.z = LEFT_CURVE_ROT;
        ros::Time now = ros::Time::now();
        if (now - phaseStartTime > ros::Duration(LEFT_CURVE_END_TIME + LEFT_CURVE_END_MARGIN_TIME)) {
            changePhase("search_line");
        } else if (now - phaseStartTime > ros::Duration(LEFT_CURVE_END_TIME)) {
            twist.angular.z = LEFT_CURVE_AFTER_ROT;
        }
        limitedTwistPub();
    }

    /*
     * 交差点の右カーブの補正
     * カーブ中に目的のレーンの左車線を検索し、検知した左車線の延長がRUN_LINEに来るようにする
     */
    void curveDetectLane(cv::Mat image){
        ros::Time now = ros::Time::now();
        if (now - phaseStartTime > ros::Duration(RIGHT_CURVE_END_TIME + RIGHT_CURVE_END_MARGIN_TIME)) {
            changePhase("search_line");
        } else if (now - phaseStartTime > ros::Duration(RIGHT_CURVE_END_TIME)) {
            if (find_left_line) {
                changePhase("search_line");
            } else {
                // ハフ変換
                cv::Mat temp_dst;
                cv::Canny(image, temp_dst, 50, 200, 3);
                std::vector <cv::Vec4i> left_lines;
                cv::HoughLinesP(temp_dst, left_lines, 1, CV_PI / 180, 20, 40, 5);

                double temp_detect_line = 0.0;
                int runLine = BIRDSEYE_LENGTH * (1 + RUN_LINE);

                // 角度が0~60の直線を検出して表示
                for (size_t i = 0; i < left_lines.size(); i++) {
                    STRAIGHT left_line = toStraightStruct(left_lines[i]);
                    if (left_line.degree < 90 && left_line.degree > 0) {
                        if (left_line.middle.x < BIRDSEYE_LENGTH * 1.5 && left_line.middle.x > BIRDSEYE_LENGTH * 0.5) {
                            if (DEBUG) {
                                cv::line(aroundDebug, cv::Point(left_lines[i][0], left_lines[i][1]),
                                         cv::Point(left_lines[i][2], left_lines[i][3]), cv::Scalar(0, 0, 255), 3, 8);
                            }
                            // left_linesからBIRDSEYE_LENGTH * 0.7に到達する地点でのx座標を推定し、BIRDSEYE_LENGTH + RUN_LINEとのずれによって
                            // 現在のLEFT_CURVE_VEL, LEFT_CURVE_ROTを補正する。
                            if (temp_detect_line == 0) {
                                temp_detect_line =(BIRDSEYE_LENGTH - left_lines[i][3]) / (left_lines[i][3] - left_lines[i][1]) * (left_lines[i][2] - left_lines[i][0]) + left_lines[i][2];
                                std::cout << "temp_detect_line   " <<  temp_detect_line << std::endl;
                            } else {
                                double temp = (BIRDSEYE_LENGTH - left_lines[i][3]) / (left_lines[i][3] - left_lines[i][1]) * (left_lines[i][2] - left_lines[i][0]) + left_lines[i][2];
                                if (abs(runLine - temp) < abs(runLine - temp_detect_line)) {
                                    temp_detect_line = temp;
                                }
                            }
                        }
                    }
                }
                if (DEBUG) {
                    cv::line(aroundDebug, cv::Point(temp_detect_line, 0),
                             cv::Point(temp_detect_line, BIRDSEYE_LENGTH), cv::Scalar(0, 255, 255), 3, 8);
                }
                twist.linear.x = RIGHT_CURVE_VEL;
                if (temp_detect_line == 0) {
                    twist.angular.z = RIGHT_CURVE_ROT;
                } else {
                    twist.angular.z = RIGHT_CURVE_ROT + (BIRDSEYE_LENGTH * (1 + RUN_LINE) - temp_detect_line) / 100;
                }
            }
        } else {
            twist.linear.x = RIGHT_CURVE_VEL;
            twist.angular.z = RIGHT_CURVE_ROT;
        }
        limitedTwistPub();
    }

    // 決め打ちで右カーブ
    void determinationRightTurn() {
        twist.linear.x = RIGHT_CURVE_VEL;
        twist.angular.z = RIGHT_CURVE_ROT;
        ros::Time now = ros::Time::now();
        if (now - phaseStartTime > ros::Duration(RIGHT_CURVE_END_TIME) && find_left_line) {
            changePhase("search_line");
        } else if (now - phaseStartTime > ros::Duration(RIGHT_CURVE_END_TIME + RIGHT_CURVE_END_MARGIN_TIME)) {
            changePhase("search_line");
        }
        limitedTwistPub();
    }

    // 障害物検知
    // 決め打ちで右にカーブし、決め打ちで左に戻る
    void  obstacleAvoidance(cv::Mat road_white_binary, cv::Mat aroundWhiteBinary) {
        ros::Time now = ros::Time::now();
        //　右車線に向けて回転
        if (now - phaseStartTime <  ros::Duration(AVOID_ROT_TIME)) {
            twist.linear.x = AVOID_OBSTACLE_VEL;
            twist.angular.z = AVOID_OBSTACLE_ROT;
        } else if(now - phaseStartTime <  ros::Duration(AVOID_ROT_TIME + AVOID_ROT_STRAIGHT))
        { // 右車線に向けて直進
            twist.linear.x = AVOID_OBSTACLE_VEL;
            twist.angular.z = 0;
        } else if(now - phaseStartTime <  ros::Duration(AVOID_ROT_TIME * 2 + AVOID_ROT_STRAIGHT))
        { // 右車線に対して水平になるように回転
            twist.linear.x = AVOID_OBSTACLE_VEL;
            twist.angular.z = -1 * AVOID_OBSTACLE_ROT;
        } else if(now - phaseStartTime <  ros::Duration(AVOID_ROT_TIME * 2 + AVOID_ROT_STRAIGHT + AVOID_BEFORE_STRAIGHT_MARGIN_TIME))
        { // 直進向く寸前に反動を消す
            twist.linear.x = AVOID_OBSTACLE_VEL;
            twist.angular.z = AVOID_OBSTACLE_ROT / 5;
        }else if(now - phaseStartTime <  ros::Duration(AVOID_ROT_TIME * 2 + AVOID_ROT_STRAIGHT + AVOID_STRAIGHT_TIME)) { // 右車線を反転させてライントレースすることで、左車線と同様のアルゴリズムで走らせる(注// アングルも逆)
            cv::Mat flipImg, flipAroundImg;
            cv::flip(flipAroundImg, aroundWhiteBinary, 1);
            cv::flip(road_white_binary, flipImg, 1);
            cv::Mat flip_left_roi(flipImg, cv::Rect(BIRDSEYE_LENGTH / 2, 0, BIRDSEYE_LENGTH, BIRDSEYE_LENGTH));

            double degree_average = detectLane(flip_left_roi);
            lineTrace(degree_average, flipImg);
            twist.linear.x = 0.2;
            twist.angular.z = -1 * twist.angular.z;
            limitedTwistPub();
        } else if(now - phaseStartTime <  ros::Duration(AVOID_ROT_TIME * 3 + AVOID_ROT_STRAIGHT + AVOID_STRAIGHT_TIME))
        { // 左車線に向けて回転
            twist.linear.x = AVOID_OBSTACLE_VEL;
            twist.angular.z = -1 * AVOID_OBSTACLE_ROT;
        } else if(now - phaseStartTime <  ros::Duration(AVOID_ROT_TIME * 3 + AVOID_ROT_STRAIGHT * 2 + AVOID_STRAIGHT_TIME))
        { // 左車線に向けて直進
            twist.linear.x = AVOID_OBSTACLE_VEL;
            twist.angular.z = 0;
        } else if(now - phaseStartTime <  ros::Duration(AVOID_ROT_TIME * 4 + AVOID_ROT_STRAIGHT * 2 + AVOID_STRAIGHT_TIME))
        { //左車線と水平になるように回転
            twist.linear.x = AVOID_OBSTACLE_VEL;
            twist.angular.z = AVOID_OBSTACLE_ROT;
        } else {
            changePhase("search_line");
        }
        limitedTwistPub();
    }



    // カーブを曲がるときにラインを追跡して挙動決定
    // 交差点で曲がる時はまた別
    void rightCurveTrace(cv::Mat road_binary) {

        int before_line_x = detected_line_x;

        // detected_line_xをラインの左下から更新する
        updateLeftLine(road_binary);

        // ロボットの速度決定
        twist.linear.x = 0.2;
        twist.angular.z = (BIRDSEYE_LENGTH * RUN_LINE - detected_line_x) / 40;
        limitedTwistPub();

        // 終了処理
        ros::Time now = ros::Time::now();
        if (now - phaseStartTime > ros::Duration(RIGHT_CURVE_END_TIME) && find_left_line) {
            if (curveAfterCrosswalk) {
                curveAfterCrosswalk = false;
                changePhase("search_right_lane_right_T");
            } else {
                changePhase("search_line");
            }
        } else if (now - phaseStartTime > ros::Duration(RIGHT_CURVE_END_TIME + RIGHT_CURVE_END_MARGIN_TIME)) {
            if (curveAfterCrosswalk) {
                curveAfterCrosswalk = false;
                changePhase("search_right_lane_right_T");
            } else {
                changePhase("search_line");
            }
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
            int p = road_hough.at<uchar>(BIRDSEYE_LENGTH * HEIGHT_L - 1, i);
            if (p) {
                for (int j = i + 1; j < i + BIRDSEYE_LENGTH / 10; j++) {

                    int q = road_hough.at<uchar>(BIRDSEYE_LENGTH * HEIGHT_L - 1, j);
                    if (q) {
                        int this_dif = std::abs((i + j) / 2 - detected_line_x);
                        if (this_dif < temp_dif) {
                            temp_dif = this_dif;
                            temp_detected_line = (i + j) / 2;
                        }
                    }
                }
            }
        }
        std::cout << "ラインX = " << detected_line_x << std::endl;
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
    // void lineTrace(int vel, int dir) {
    void lineTrace(float degree_average, cv::Mat road_white_binary) {

        if (find_left_line) {
            // 中点が右過ぎたら左に、左過ぎたら右に曲がる
            if (detected_line_x > BIRDSEYE_LENGTH * (RUN_LINE + RUN_LINE_MARGIN)) {
                twist.linear.x += 0.01;
                twist.angular.z = -0.1;
            } else if (detected_line_x < BIRDSEYE_LENGTH * (RUN_LINE - RUN_LINE_MARGIN)) {
                twist.linear.x += 0.01;
                twist.angular.z = 0.1;
            } else if (degree_average < -10 || degree_average > 10) {
                // 角度平均が-5以上なら左に曲がる、5以上なら右に曲がる
                twist.angular.z = degree_average * -0.01;
            } else {
                twist.linear.x += 0.03;
            }
            line_lost_cnt = 0;
        } else {
            // 車線が見つからなかった場合、LeftRoadLeftTで最下のものを基準に
            line_lost_cnt += 1;
            updateLeftLine(road_white_binary);
            twist.angular.z = (BIRDSEYE_LENGTH * RUN_LINE - detected_line_x) / 400;
        }
    }

    void limitedTwistPub() {
        if (twist.linear.x > BURGER_MAX_LIN_VEL) twist.linear.x = BURGER_MAX_LIN_VEL;
        if (twist.linear.x < 0) twist.linear.x = 0;
        if (twist.angular.z > BURGER_MAX_ANG_VEL) twist.angular.z = BURGER_MAX_ANG_VEL;
        if ((twist.angular.z < 0 - BURGER_MAX_ANG_VEL)) twist.angular.z = 0 - BURGER_MAX_ANG_VEL;
        twist_pub.publish(twist);
    }

    // ラインが見つからないときに首を振ることで直線を探す
    void searchLine() {
        ros::Time now = ros::Time::now();
        if (find_left_line) {
            changePhase("straight");
            twist.angular.z = 0;
            limitedTwistPub();
            return;
        }
        std::cout << "line search" << std::endl;


        // 三秒ごとに首を振る向きを変える
        if (now - phaseStartTime < ros::Duration(3.0)) {
            twist.linear.x = 0;
            twist.angular.z = -0.1;
        } else if (now - phaseStartTime < ros::Duration(6.0)) {
            twist.angular.z = 0.1;
        } else if (now - phaseStartTime < ros::Duration(9.0)) {
            twist.angular.z = 0.3;
        } else if (now - phaseStartTime < ros::Duration(12.0)) {
            twist.angular.z = -0.3;
        } else {
            phaseStartTime = ros::Time::now();
            std::cout << "one more search" << std::endl;
        }

        limitedTwistPub();
    }

    void searchRightLaneRightT(bool nowFindRightLaneRightT) {
        ros::Time now = ros::Time::now();
        if (nowFindRightLaneRightT) {
            changePhase("straight");
            twist.angular.z = -1 * twist.angular.z;
            limitedTwistPub();
            return;
        }
        std::cout << "now search right_lane_right_T " << std::endl;

        // 三秒ごとに首を振る向きを変える
        if (now - phaseStartTime < ros::Duration(3.0)) {
            twist.linear.x = 0;
            twist.angular.z = -0.1;
        } else if (now - phaseStartTime < ros::Duration(6.0)) {
            twist.angular.z = 0.1;
        } else if (now - phaseStartTime < ros::Duration(7.0)) {
            twist.angular.z = 0.3;
        } else if (now - phaseStartTime < ros::Duration(9.0)) {
            twist.angular.z = 0.3;
        } else {
            phaseStartTime = ros::Time::now();
            std::cout << "Right Lane one more search" << std::endl;
        }
        limitedTwistPub();
    }



    // imageを渡して俯瞰画像を得る
    cv::Mat birdsEye(cv::Mat image) {
        int width = image.size().width;
        int height = image.size().height;
        // 奥行の広さ（小さいほど狭い）
        double width_ratio = WIDTH_RATIO;
        // 上部
        double height_h = HEIGHT_H;
        // 下部
        double height_l = HEIGHT_L;
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
        double width_ratio = WIDTH_RATIO;
        // 上部
        double height_h = HEIGHT_H;
        // 下部
        double height_l = HEIGHT_L;
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
    // まず、車体正面のBIRDSLENGTH四方を取り出し、赤色っぽいものの二値化を行う
    // y軸方向で切り出し、一番yが大きいものをy座標としてobjectsに追加or更新
    // 全体の赤色値を信頼度とし、一定値以上あれば障害物として認知
    // objectのy座標が一定以下になれば回避行動フェイズにチェンジ
    /*
    void detectObstacle(){

    }
     */


// 二点をSTRAIGHT構造体で返す
    STRAIGHT toStraightStruct(cv::Vec4i line) {
        STRAIGHT result;

        //中点
        result.middle = cv::Point((line[0] + line[2]) / 2, (line[1] + line[3]) / 2);
        // 長さ
        result.length = (line[0] - line[2]) * (line[0] - line[2]) + (line[1] - line[3]) * (line[1] - line[3]);
        // 角度
        double radian = atan2(line[1] - line[3], line[0] - line[2]);
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
    double lineWeight(cv::Vec4i line) {
        // 距離
        double distance = (line[0] - line[2]) * (line[0] - line[2]) + (line[1] - line[3]) * (line[1] - line[3]);

        // 角度
        double radian = atan2(line[1] - line[3], line[0] - line[2]);

        // radianからdegree
        double degree = radian * 180.0 / PI;

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

    // objTypeに一致するオブジェクトをすべて消去
    void deleteObject(std::string objType) {
        std::list<OBJECT>::iterator itr;
        for (itr = objects.begin(); itr != objects.end();) {
            OBJECT compare = *itr;
            if (compare.objType == objType) {
                itr = objects.erase(itr);
                continue;
            }
            itr++;
        }
    }

    // オブジェクトを最も遠いもの(Yが小さいもの)に更新
    // ひとつ見つけた時点で終了するため、この関数で追加するobjTypeはaddObjectを用いない
    void addMostDistantObject(std::string objType, int objectX, int objectY) {
        bool findObj = false;
        std::list<OBJECT>::iterator itr;
        for (itr = objects.begin(); itr != objects.end();) {
            OBJECT compare = *itr;
            if (compare.objType == objType && objectY < compare.beforeY) {
                compare.beforeX = objectX;
                compare.beforeY = objectY;
                compare.findCnt += 1;
                compare.timeStamp = ros::Time::now();
                *itr = compare;
                findObj = true;
                std::cout << objType << " update object cnt = " << compare.findCnt << std::endl;
                break;
            }
            itr++;
        }
        if (!findObj) {
            OBJECT obj = {objType, objectX, objectY, 1, ros::Time::now()};
            objects.push_front(obj);
        }
    }


    // オブジェクトを発見した時、それが以前発見されたものと一致するかどうかを調べ、一致しなかったら追加
    // 一致する場合タイムスタンプと位置を更新し、カウントを1増やす
    void addObject(std::string objType, int objectX, int objectY) {
        bool findObj = false;
        std::list<OBJECT>::iterator itr;
        for (itr = objects.begin(); itr != objects.end();) {
            OBJECT compare = *itr;
            if (compare.objType == objType && (std::abs(objectX - compare.beforeX) < 30) &&
                (std::abs(objectY - compare.beforeY) < 30)) {
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

    void findRedObs(cv::Mat birds_eye){
        cv::Mat red_mask1, red_mask2, red_image, red_hsv_image;
        cv::Mat redRoi(birds_eye, cv::Rect(BIRDSEYE_LENGTH * 0.2, BIRDSEYE_LENGTH / 2, BIRDSEYE_LENGTH / 2, BIRDSEYE_LENGTH / 2));
        cv::cvtColor(redRoi, red_hsv_image, CV_BGR2HSV);
        cv::inRange(red_hsv_image, cv::Scalar(0, 127, 0, 0),
                    cv::Scalar(15, 255, 255, 0), red_mask1);
        cv::inRange(red_hsv_image, cv::Scalar(150, 127, 0, 0),
                    cv::Scalar(179, 255, 255, 0), red_mask2);
        // cv::bitwise_and(redRoi, redRoi, red_image, red_mask1 + red_mask2);

        int fractionNum = cv::countNonZero(red_mask1 + red_mask2);
        if (fractionNum > 500) {
            changePhase("find_obs");
        }
    }

    /*
     * 交差点をテンプレートマッチングで検索する
     * T字路の場合、右T(right_T)、左T(left_T)、下T(under_T)の３種類
     * さらに、横断歩道(crosswalk)、交差点(intersection)の計5種類が存在する
     * マップデータから次の判別すべきタイルは判断できるので、判断されたタイルに適した画像を検出すればよい
     * 判別すべき画像はnextSearchObjectで保持しておく
     */
    void intersectionDetectionByTemplateMatching(cv::Mat aroundWhiteBinary, double template_angle)
    {
        cv::Mat template_img;

        // Xの領域を区切る
        int searchLeftX = (int)(BIRDSEYE_LENGTH * 1);

        bool doSearch = true;

        if (searchType == "right_T") {
            template_img = template_right_T;
        } else if (searchType == "left_T") {
            template_img = template_left_T;
        } else if (searchType == "under_T") {
            template_img = template_under_T;
        } else if (searchType == "crosswalk") {
            template_img = template_crosswalk;
        } else if (searchType == "intersection") {
            template_img = template_intersection;
        } else {
            doSearch = false;
        }

        std::cout << "現在" << searchType << "検索中" << std::endl;

        double maxVal;
        cv::Mat result;

        if (doSearch) {
            // 傾きを元に元画像を回転
            cv::Mat affine = cv::getRotationMatrix2D(cv::Point2f(template_img.cols / 2 , template_img.rows / 2), template_angle * -0.7, 1.0);
            cv::Mat template_rot;
            cv::warpAffine(template_img, template_rot, affine, template_img.size(), cv::INTER_CUBIC);
            cv::imshow("template_rot", template_rot);
            cv::moveWindow("template_rot", 20, 200);

            cv::Mat searchRoi(aroundWhiteBinary, cv::Rect(searchLeftX, 0, BIRDSEYE_LENGTH * 1.5, BIRDSEYE_LENGTH));

            cv::matchTemplate(searchRoi, template_rot, result, cv::TM_CCORR_NORMED);
            cv::Point maxPt;
            cv::minMaxLoc(result, 0, &maxVal, 0, &maxPt);
            std::cout << "一致度　= " << maxVal << " | 位置　x = " << maxPt.x + template_img.cols / 2 << "  y = " << maxPt.y + template_img.rows / 2 << std::endl;
            // cv::rectangle(aroundDebug, cv::Point(searchLeftX + maxPt.x, maxPt.y), cv::Point(searchLeftX + maxPt.x + template_right_T.cols, maxPt.y + template_right_T.rows), cv::Scalar(255 * maxVal, 255 * maxVal, 255 * maxVal), 2, 8, 0);
            if (maxVal > 0.75) {
                addObject(searchType, searchLeftX + maxPt.x  + template_right_T.cols / 2, maxPt.y + template_right_T.rows / 2);
                std::cout << searchType << " find! y =  " << maxPt.y + template_img.rows / 2 << std::endl;
            }
        }
    }

    void testTemplateMatching(cv::Mat aroundWhiteBinary, cv::Mat template_img, cv::Scalar color) {
        double maxVal;
        cv::Mat result;

        // 傾きを元に元画像を回転
        cv::Mat affine = cv::getRotationMatrix2D(cv::Point2f(template_img.cols / 2 , template_img.rows / 2), detected_angle * -0.7, 1.0);
        cv::Mat template_rot;
        cv::warpAffine(template_img, template_rot, affine, template_img.size(), cv::INTER_CUBIC);

        cv::matchTemplate(aroundWhiteBinary, template_rot, result, cv::TM_CCORR_NORMED);
        cv::Point maxPt;
        cv::minMaxLoc(result, 0, &maxVal, 0, &maxPt);
        std::cout << "一致度　= " << maxVal << " | 位置　x = " << maxPt.x + template_img.cols / 2 << "  y = " << maxPt.y + template_img.rows / 2 << std::endl;
        if (maxVal > 0.75) {
            cv::rectangle(aroundDebug, cv::Point(maxPt.x, maxPt.y),
                          cv::Point(maxPt.x + template_right_T.cols, maxPt.y + template_right_T.rows),
                          color, 2, 8, 0);
        }
    }


    // 現在のオブジェクト状況を出力
    void testOutputObject() {
        int objCnt = 1;
        std::list<OBJECT>::iterator itr;
        for (itr = objects.begin(); itr != objects.end();) {
            OBJECT obj = *itr;

            std::cout << objCnt << " Type" << obj.objType << std::endl;
            std::cout << "検知回数 : " << obj.findCnt << " |  y =  " << obj.beforeY << std::endl;

            itr++;
            objCnt++;
        }
    }
    // オブジェクトが一定時間発見されていなければ破棄
    void updateObject() {
        ros::Time now = ros::Time::now();
        std::list<OBJECT>::iterator itr;
        int objCnt;
        int mostUnderLeftLaneLeftT_y = 0;
        mostUnderLeftLaneLeftT = detected_line_x;


        for (itr = objects.begin(); itr != objects.end();) {
            OBJECT obj = *itr;

            // 走行距離分Y座標を修正
            obj.beforeY = obj.beforeY + mileage*3.5;

            // オブジェクトが一定位置に見えたら曲がる(止まる)フラグを立てる場合
            if (obj.objType == "crosswalk") {
                if (obj.beforeY >100 -  CROSSWALK_UNDER_MARGIN)  {
                    crosswalkFlag = true;
                }
            } else if (obj.beforeY >100 -  INTERSECTION_PREDICTION_UNDER_MARGIN)  {
                if (obj.findCnt > 1) {
                    if (obj.objType == "right_T" || obj.objType == "left_T" || obj.objType == "under_T" || obj.objType == "intersection") {
                        intersectionDetectionFlag = true;
                    }

                    if (obj.objType == "crosswalk") {
                        crosswalkFlag = true;
                    }
                }
            }
            *itr = obj;


            // オブジェクトが下に到達する時刻を推定し、下に到達したと推定された場合アクションのためのフラグを立てる
            // タイルを進める、左に曲がる等をsearchTile()で行う
            double  reachBottomTime = ((1 - ((double)obj.beforeY) / BIRDSEYE_LENGTH) * 4  + INTERSECTION_PREDICTION_UNDER_MARGIN) * INTERSECTION_PREDICTION_TIME_RATIO * (0.2 / (twist.linear.x + 0.001));

            if (obj.objType == "left_lane_left_T") {
                // std::cout << "左レーンTの残り時間= " << now - obj.timeStamp - ros::Duration(reachBottomTime) << std::endl;
                if (mostUnderLeftLaneLeftT_y > obj.beforeY) {
                    mostUnderLeftLaneLeftT_y = obj.beforeY;
                    mostUnderLeftLaneLeftT = obj.beforeX - BIRDSEYE_LENGTH; // BIRDSEYE_LENGTH分だけ右にずれているため
                }
            }

            if (now - obj.timeStamp > ros::Duration(reachBottomTime)) {
                if (obj.findCnt > 1) {
                    if (obj.objType == "left_lane_left_T") {
                        std::cout << "left lane left T = true " << std::endl;
                        reachBottomLeftLaneLeftT = true;
                    } else if (obj.objType == "right_lane_left_T") {
                        reachBottomRightLaneLeftT = true;
                    } else if (obj.objType == "right_lane_right_T") {
                        reachBottomRightLaneRightT = true;
                    } else if (obj.objType == "left_lane_end") {
                        // std::cout << "left lane end = true " << std::endl;
                        reachBottomLeftLaneStraightEnd = true;
                    }
                }
                itr = objects.erase(itr);
                continue;
            }
            itr++;
            objCnt++;
        }
    }
};

int main(int argc, char **argv) {

    // 進行方向読み込み
    std::ifstream ifs("/home/sou/catkin_ws/src/opencv_test/bending_direction.txt");
    std::string str;
    if (ifs.fail()){
        std::cerr << "text file load fail" << std::endl;
        return -1;
    }
    int cnt=0;
    while (getline(ifs, str)){
        // std::cout << "[" << str << "]" << std::endl;
        int num = std::atoi(str.c_str());
        // std::cout << num << std::endl;
        intersectionDir[cnt++] = num;
    }
    for(int i = 0; i < cnt; i++){
        std::cout << intersectionDir[i] << std::endl;
    }

    ros::init(argc, argv, "image_converter");
    ImageConverter ic;
    ros::spin();
    return 0;
}
