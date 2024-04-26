/**
* @file     megarover_controller.h
* @brief    Megaroverのコレオノイド用コントローラープログラムのヘッダファイル
* @author   S.Kumada
* @date     2022/08/10
* @details  関数定義ヘッダファイル
*/

#ifndef MEGAROVER_CONTROLLER_H_
#define MEGAROVER_CONTROLLER_H_

#define DEBUG 0 // デバックフラグ

// choreonoidライブラリヘッダ
#include <cnoid/SimpleController>
#include <cnoid/RangeCamera>
#include <cnoid/RangeSensor>
#include <cnoid/AccelerationSensor>
#include <cnoid/RateGyroSensor>
#include <fmt/format.h>

// ROSライブラリヘッダ
#include <ros/node_handle.h>
#include <sensor_msgs/Imu.h>
#include <mutex>
#include <iostream>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Twist.h>
// For angle confirmation(Debug).
#include <std_msgs/Float32MultiArray.h>
// simulation time
#include <std_msgs/String.h>
#include <cnoid/EigenUtil>
#include <math.h>
// Output csv file.
#include <fstream>

// 定数
#define TREAD_WIDTH             0.284       // 車輪間距離定数(メガローバー)[m]
#define TIRE_DIAMETER           0.076       // タイヤの直径の長さ[mm]
#define MAX_VEL                 0.8         // 最大並進速度[m/s]
#define MIN_VEL                 -MAX_VEL    // 最低並進速度[m/s]
#define MAX_ANG_VEL             2.51        // 最大旋回速度[rad/s]
#define MIN_ANG_VEL             -MAX_ANG_VEL// 最低旋回速度[m/s]
#define MAX_ACC                 1.8         // 最大加速度[m/s^2]
#define MIN_ACC                 -MAX_ACC    // 最低加速度[m/s^2]
#define INITIAL_POSE_X          0.0
#define INITIAL_POSE_Y          0.0
#define INITIAL_POSE_THETA      0.0
#define ROBOT_PREFIX            "megarover_01_sim"
#define ODOM_FRAME_ID           "odom"
#define CHILD_FRAME_ID          "base_footprint"
#define IMU_FRAME_ID            "base_footprint"
#define VELOCITY_CMD_TOPIC      "rover_twist"
#define IMU_TOPIC               "imu"
#define ODOMETRY_TOPIC          "odom"
#define LEFT_WHEEL_JOINT_NAME   "left_wheel_joint"
#define RIGHT_WHEEL_JOINT_NAME  "right_wheel_joint"
#define ACCELEROMETER_NAME      "AccelSensor"
#define GYROSCOPE_NAME          "GyroSensor"

#define RATE_1HZ        1.0
#define RATE_10HZ       10.0
#define RATE_15HZ       15.0
#define RATE_30HZ       30.0
#define RATE_45HZ       45.0
#define RATE_60HZ       60.0
#define RATE_200HZ      200.0

#define ROS_QUEUE_SIZE_1    1
#define ROS_QUEUE_SIZE_10   10
#define ROS_QUEUE_SIZE_100  100

// namespace
using namespace std;
using namespace cnoid;
using fmt::format;

/// 関節
enum TOPIC_IDX
{
    ODOMETRY    ,   // O odomトピック
    IMU         ,   // 1 IMUトピック
    VELOCITY    ,   // 2 cmd_velトピック(rover_twist)

    TOPIC_NUM      // 3 TOPIC_IDX上限値
};

enum WHEEL_IDX
{
    WHEEL_LEFT  ,   // 0 左ホイール
    WHEEL_RIGHT ,   // 1 右ホイール

    WHEEL_NUM       // 2 WHEEL_IDX上限値
};

enum SENSOR_IDX
{
    ACCEL_SENSOR    ,   // O 加速度センサ
    GYRO_SENSOR     ,   // 1 ジャイロセンサ

    SENSOR_NUM      // 2 SENSOR_IDX上限値
};

enum COORDINATE_IDX
{
    X   ,   // O X座標
    Y   ,   // 1 Y座標
    Z   ,   // 2 Z座標

    COORDINATE_NUM      // 3 SENSOR_IDX上限値
};

enum DIRECTION_IDX
{
    LINEAR_X   ,   // O X方向速度
    LINEAR_Y   ,   // 1 Y方向速度
    LINEAR_Z   ,   // 2 Z方向速度

    DIRECTION_NUM      // 3 DIRECTION_IDX上限値
};

enum ROTATION_IDX
{
    ANGULAR_X   ,   // O X方向回転速度
    ANGULAR_Y   ,   // 1 Y方向回転速度
    ANGULAR_Z   ,   // 2 Z方向回転速度

    ROTATION_NUM      // 3 ROTATION_IDX上限値
};

enum UPDATE_FUNCTION_IDX
{
    UPDATE_ODOMETRY     ,   // O オドメトリーの更新関数
    UPDATE_IMU          ,   // 1 IMUの更新関数 

    UPDATE_FUNCTION_NUM     // 2 UPDATE_FUNCTION_IDX上限値
};

class MegaRoverRosController : public SimpleController
{
    public:
    


        /**
        * @brief   MMegaRoverRosControllerクラスのコンストラクタ
        * @details 初期化を行う
        */
        MegaRoverRosController();

        /**
        * @brief   MegaRoverRosControllerクラスのデストラクタ
        * @details オブジェクトの破棄を行う
        */
        ~MegaRoverRosController();

        /**
         * @brief           コントローラの初期化関数
         * @param[in,out]   config SimpleControllerConfigオブジェクトへのポインタ
         * @return          bool true:初期化完了
         * @details         プロジェクトに読み込まれた時点で実行される初期化関数
         */
        virtual bool configure(SimpleControllerConfig* config);
        
        /**
         * @brief           コントローラの初期化関数
         * @param[in,out]   *io SimpleControllerIOオブジェクトへのポインタ
         * @return          bool true:初期化完了
         * @details         シミュレーション開始時に実行される初期化関数
         */
        virtual bool initialize(SimpleControllerIO* io);

        /**
         * @brief       コントローラの制御関数
         * @param       void
         * @return      bool true:処理成功
         * @details     コントローラの入力・制御・出力処理を実行する
         */
        virtual bool control();


        /**
         * @brief       シミュレーション停止処理関数
         * @param       void
         * @return      void
         * @details     シミュレーション停止時に実行される
         */
        virtual void stop();

    	ros::NodeHandle nh_;
        // Publisher
        ros::Publisher pub_odom_;
        ros::Publisher pub_imu_;
        // Subscriber
        ros::Subscriber sub_velocity_;

    	tf::TransformBroadcaster tf_broadcaster_;

        double current_time_[TOPIC_NUM], last_time_[TOPIC_NUM], update_period_[TOPIC_NUM];
	    double update_rate_[TOPIC_NUM] = { RATE_30HZ, RATE_200HZ, RATE_10HZ };

	    SimpleControllerIO* io_;
	    BodyPtr ptr_body_;

        // Actuation mode
        int actuationMode_;

    	// Wheel names
        string wheelNames_[WHEEL_NUM];
        // Wheel links
        Link* wheel_links_[WHEEL_NUM];
        // 関節角度
        double prev_angle_[WHEEL_NUM]; // 前の関節角度値
        double ref_angle_[WHEEL_NUM];  // 参照値
        // 時間変化量
        double dt_;
        // トレッド幅の半分
        double half_tread_dist_;
        // ホイールの半径
        double wheel_radius_;
        // ホイールの回転角度
	    Vector2 prev_wheel_angle_;

        // 最大並進速度[m/s]
        double max_vel_;
        // 最大旋回速度[m/s]
        double max_ang_vel_;
        // 最低並進速度[m/s]
        double min_vel_;
        // 最低旋回速度[m/s]
        double min_ang_vel_;

        // 現在の座標位置
        geometry_msgs::Pose current_pose_;
        // 現在の姿勢(th:回転角度[rad])
        double th_;
        // v:現在の速度[m/s] vth:現在の角速度[rad/s]
        Vector3 v_, vth_;
        // １つ前の角速度[rad/s]
       	Vector3 prev_vth_;
        // 最新の角速度[rad/s]
        double last_vth_;
        // 速度制御コマンド
        Vector3 cmd_vel_, cmd_vel_th_;

        // IMUの使用可否
        bool is_use_imu_;
        // IMUセンサ角度
        double imu_th_;
        // 加速度センサ
	    AccelerationSensorPtr ptr_accel_sensor_;
        // 加速度センサの有効
        bool is_active_accel_;
        // ジャイロセンサ
    	RateGyroSensorPtr ptr_gyro_sensor_;
        // ジャイロセンサの有効
        bool is_active_gyro_;
        // シグナル設定状態監視
	    ScopedConnection gyroSensorConnection;
        // センサデバイス名
	    string deviceName_[SENSOR_NUM] = { "AccelSensor", "GyroSensor" };
        // センシングステップ数
	    int sensing_steps_[SENSOR_NUM];
        // センサ値の合計(dv_sum: w_sum:クォータニオン)
    	Vector3 dv_sum_[SENSOR_NUM], w_sum_[SENSOR_NUM];

        // Odometry and imu frame id
        string odom_frame_id_;
        string child_frame_id_;
        string imu_frame_id_;
        string odom_topic_name_;
        string velocity_cmd_topic_name_;
        string imu_topic_name_;

        // CSV出力処理用
        double count = 0;
        double ptheta;
        double last_time;
	    ofstream ofs;

    private:

        /**
        * @brief        移動コマンドの受信コールバック関数
        * @param[in]    msg 移動コマンドメッセージデータ
        * @return       void
        * @details      rover_twistトピックをサブスクライブした際に実行される
        */
        void recvVelocityCmdCB(const geometry_msgs::Twist& msg);

        /**
         * @brief       odomの更新処理関数
         * @param       void
         * @return      void
         * @details     オドメトリ・TFの更新を実行する
         */
        void updateOdom();

        /**
         * @brief       IMUの更新処理関数
         * @param       void
         * @return      void
         * @details     /imuトピックの更新を実行する
         */
        void updateImu();

        /**
         * @brief       座標の更新処理関数
         * @param       void
         * @return      void
         * @details     ロボットの座標値を更新する
         */                
        void updateCoord();


};


#endif