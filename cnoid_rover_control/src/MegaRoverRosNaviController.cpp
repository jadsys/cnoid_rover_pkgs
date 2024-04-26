/*
 * MegaRoverRosController.cpp
 *
 *  Created on: 2021/04/01
 *      Author: Tsuyoshi Anazawa
 *
 *  Outline: This program that controls the TurtleBot3 model with ROS navigation.
 */

#include <cnoid/SimpleController>
#include <cnoid/RangeCamera>
#include <cnoid/RangeSensor>
#include <cnoid/AccelerationSensor>
#include <cnoid/RateGyroSensor>
#include <fmt/format.h>
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

using namespace std;
using namespace cnoid;
using fmt::format;

class MegaRoverRosController : public SimpleController
{
	ros::NodeHandle nh_;
	// Publisher for Odometry
	ros::Publisher odom_pub_;
	tf::TransformBroadcaster tfBroadcaster_;
	// 0: odom, 1: imu, 2: cmd_vel
	double currentTime[3], lastTime[3], update_period[3];
	double update_rate[3] = { 30.0, 200.0, 10.0 };

	// Publisher for imu
	ros::Publisher imu_pub_;
	AccelerationSensorPtr accelSensor;
	RateGyroSensorPtr gyroSensor;
	string deviceName[2] = { "AccelSensor", "GyroSensor" };
	ScopedConnection accelSensorConnection;
	ScopedConnection gyroSensorConnection;

	// Subscriber for cmd_vel
	ros::Subscriber cmd_vel_sub_;
	// Publisher for angle[deg](Debug)
	ros::Publisher theta_pub_;
	// Number of wheels
	static const int WHEEL_NUM = 2;
	// Wheel names
	const string wheelNames[WHEEL_NUM] = { "left_wheel_joint", "right_wheel_joint" };
	// Actuation mode
	int actuationMode;
	// Wheel links
	Link* wheels[2];
	double qprev[2];
	double dt;
	double qref[2];
	SimpleControllerIO* io;
	// current coordinate position
	double x, y, th;
	double x_local;
	double y_local;
	double th_local;
	double omega = 0.0;
	double debug_th;
	// Current velocities[m/s] and angular velocities[rad/s]
	Vector3 v, vth;
	// Last velocities[m/s] and angular velocities[rad/s]
	Vector3 prevVth, debug_vth;
	double imuTh;
	Vector3 cmd_vel, cmd_vel_th;
	double timeSpan = 0.05;
	// Odometry and imu frame id
	const string ODOM_FRAME_ID = "odom";
	const string CHILD_FRAME_ID = "base_footprint";
	const string IMU_FRAME_ID = "base_footprint";
	string odom_frame_id_;
	string child_frame_id_;
	string imu_frame_id_;
	// Initial position of the robot
	const double INITIAL_POSE_X = 0.0;
	const double INITIAL_POSE_Y = 0.0;
	const double INITIAL_POSE_THETA = 0.0;
	// Limit velocities and accelerations
	const double MAX_VEL = 0.8;
	const double MIN_VEL = -0.8;
	const double MAX_ACCEL = 2.51;
	const double MIN_ACCEL = -2.51;
	// half of the tread
	double d = 0.284 / 2;
	// Wheel radius
	double r = 0.076;
	Vector2 prevWheelTh;
	Vector3 dv_sum[2], w_sum[2];
	int sensingSteps[2];

	// rootLink
	BodyPtr ioBody;
	double count = 0;
	double ptheta;
	// Simulation time.
	ros::Publisher simtime_pub_;
	// Output csv file.
	ofstream ofs;
	double current_time, last_time, last_time_2;

public:
	double clamp(const double x, const double min, const double max)
	{
	  return std::min(std::max(min, x), max);
	}

	virtual bool configure(SimpleControllerConfig* config) override
	{
		return true;
	}

	virtual bool initialize(SimpleControllerIO* io) override
	{

		
		this->io = io;
		ostream& os = io->os();
		// Body variable.
		Body* body = io->body();
		// Add to actual value.
		ioBody = body;

		// prefix名の取得
		std::string robot_prefix_ = ioBody->name();
		odom_frame_id_	= robot_prefix_ + "/" + ODOM_FRAME_ID;
		child_frame_id_	= robot_prefix_ + "/" + CHILD_FRAME_ID;
		imu_frame_id_	= robot_prefix_ + "/" + IMU_FRAME_ID;

		// Set joint's actuation mode.
		actuationMode = Link::JointTorque;
		// Get controller option.
		string option = io->optionString();

		if(!option.empty()){
			// When controller option is empty.
			if(option == "velocity" || option == "position"){
				// When controller option is velocity or position.
				actuationMode = Link::JointVelocity;
				os << format("アクチュエーションモード ： {0} ", option) << endl;
			} else if(option == "torque"){
				// When controller option is torque.
				actuationMode = Link::JointTorque;
				os << format("アクチュエーションモード ： {0} ", option) << endl;
			} else {
				// In other cases.
				os << format("Warning: Unknown option {}.", option) << endl;
			}
		}

		// Add to actual value.
		io->enableInput(ioBody->rootLink(), LINK_POSITION);

		for(int i = 0; i < WHEEL_NUM; ++i)
		{
			// Store wheel links in array.
			wheels[i] = body->link(wheelNames[i]);
			if(!wheels[i]){
				// When there is no wheel links.
				os << format("{0} of {1} is not found.", wheelNames[i], body->name()) << endl;
				return false;
			}

			// Set the wheel array's actuation mode.
			wheels[i]->setActuationMode(actuationMode);
			// Enable output to wheel links.
			io->enableIO(wheels[i]);
			// Store current joint angles in array
			qref[i] = qprev[i] = wheels[i]->q();
		}

		accelSensor = body->findDevice<AccelerationSensor>(deviceName[0]);
		if(!accelSensor){
			os << format("{0} of {1} is not found.", deviceName[0], body->name()) << endl;
			//return false;
		} else {
			io->enableInput(accelSensor);
//			accelSensorConnection.disconnect();
//			accelSensorConnection = accelSensor->sigStateChanged().connect(
//					[&](){ updateImu(); });
		}
		
		gyroSensor = body->findDevice<RateGyroSensor>(deviceName[1]);
		if(!gyroSensor){
			os << format("{0} of {1} is not found.", deviceName[1], body->name()) << endl;
			//return false;
		} else {
			io->enableInput(gyroSensor);
//			gyroSensorConnection.disconnect();
//			gyroSensorConnection = gyroSensor->sigStateChanged().connect(
//					[&](){ updateImu(); });
		}


		// Create subscriber for cmd_vel
		cmd_vel_sub_ = nh_.subscribe("/" + robot_prefix_ + "/rover_twist", 1, &MegaRoverRosController::cmdvelCallback, this);
		// Create publisher for odometry(topic: odom, buffer_size: 10)
		odom_pub_ = nh_.advertise<nav_msgs::Odometry>("/" + robot_prefix_ + "/odom", 10);
		// Create publisherfor imu(topic: imu, buffer_size: 10)
		imu_pub_ = nh_.advertise<sensor_msgs::Imu>("/" + robot_prefix_ + "/imu", 10);
		// Create publisher for angle(Debug)
		theta_pub_ = nh_.advertise<std_msgs::Float32MultiArray>("/" + robot_prefix_ + "/theta", 10);
		// Create publisher for simulation time.
		simtime_pub_ = nh_.advertise<std_msgs::String>("/" + robot_prefix_ + "/simtime", 10);
		// Origin of odom coordinate system assuming the initial position of the robot
		x = INITIAL_POSE_X;
		y = INITIAL_POSE_Y;
		th = imuTh = INITIAL_POSE_THETA;

		x_local = INITIAL_POSE_X;
		y_local = INITIAL_POSE_Y;
		th_local = INITIAL_POSE_THETA;

		// Add to actual value.
		ptheta = INITIAL_POSE_THETA;

		prevVth[1] = 0.0;
		prevVth[2] = 0.0;
		debug_vth[2] = 0.0;

		for(int i = 0; i < sizeof(wheels) / sizeof(wheels[0]); ++i)
		{
			prevWheelTh[i] = wheels[i]->q();
		}

		currentTime[3] = lastTime[3] = io->currentTime();
		for(int i = 0; i < sizeof(currentTime) / sizeof(currentTime[0]); ++i)
		{
			update_period[i] = 1.0 / update_rate[i];
		}

		string file_path = "/home/ros/.ros/cnoid_coord_navi.csv";
		ifstream ifs(file_path);
		string hdr;
		bool hdr_flg = false;
		if(ifs.fail()){
			hdr = "sim_time,x,y,z";
			hdr_flg = true;
		}
		ofs.open(file_path, ios::app);
		if(hdr_flg){
			ofs << hdr << endl;
		}

		current_time = last_time = last_time_2 = io->currentTime();
		dt = io->timeStep();
		sensingSteps[0] = sensingSteps[1] = 0;


		return true;
	}

	void cmdvelCallback(const geometry_msgs::Twist& msg)
	{
		cmd_vel[0] = msg.linear.x;
		cmd_vel[1] = msg.linear.y;
		cmd_vel[2] = msg.linear.z;
		cmd_vel_th[0] = msg.angular.x;
		cmd_vel_th[1] = msg.angular.y;
		cmd_vel_th[2] = msg.angular.z;
	}

	void updateCoord()
	{
		Vector2 wheel_th, wheel_omg, wheel_v;

		for(int i = 0; i < sizeof(wheels) / sizeof(wheels[0]); ++i)
		{	// wheel数分（MRの場合左右2個）繰り返す

			// 最新のホイール角度を取り出す
			double latest_wheel_angle = wheels[i]->q(); // 最新のホイールの回転角度
			// 回転角度の変化量を求める
			wheel_th[i] = (latest_wheel_angle - prevWheelTh[i]); // prevWheelTh[i]は1つ前のホイールの回転角度
			// 回転速度を求める
//			wheel_omg[i] = wheel_th[i] / dt; // ⊿wheel_th / ⊿t(単位時間あたりの変化量)
			wheel_omg[i] = wheel_th[i] / update_period[0];
			// 周速度を求める
			wheel_v[i] = r * wheel_omg[i];
			
			

#if DEBUG
			// 右左のどちらか判別
			string w_nm;
			
			if(i == 0)
			{
				w_nm = "left";
			}
			else 
			{
				w_nm = "right";
			}
#endif
			// ホイールの最終回転角度更新
			prevWheelTh[i] = latest_wheel_angle;

		}

		// 車体速度
		v[0] = clamp((wheel_v[0] + wheel_v[1]) / 2.0, MIN_VEL, MAX_VEL);	// x方向の速度=全車輪の周速度/車輪数
		v[1] = 0.0;	// y方向の速度

		// ROS_INFO("updateCoord : wheel_v[0]: %lf, wheel_v[1] %lf, v[0]: %lf, v[1]: %lf ]", wheel_v[0], wheel_v[1], v[0], v[1]);
			

//		omega = clamp((wheel_v[1] - wheel_v[0]) / (2.0 * d), MIN_ACCEL, MAX_ACCEL);
//		omega = gyroSensor->w().z();
//		double deltaTh = ((omega + prevVth[2]) / 2.0) * dt;
//		double deltaX = (v[0] * cos(th) - v[1] * sin(th)) * dt;
//		double deltaY = (v[0] * sin(th) + v[1] * cos(th)) * dt;
		double deltaX = (v[0] * cos(th) - v[1] * sin(th)) * update_period[0];
		double deltaY = (v[0] * sin(th) + v[1] * cos(th)) * update_period[0];
		double deltaTh = ((omega + prevVth[2]) / 2.0) * update_period[0];
//		double deltaTh = ((omega + prevVth[2]) / 2.0) * dt;

		x += deltaX;
		y += deltaY;
		th += deltaTh;

		// Debug
		// ROS_INFO("updateCoord : delta theta: %lf, [ x: %lf, y: %lf, yaw: %lf ], omega %lf  , prevVth[2] %lf ", deltaTh, x, y, th, omega , prevVth[2]);
		
		prevVth[1] = prevVth[2];
		prevVth[2] = omega; // omegaはupdateOdomでImuから取得した最新の角速度（パブする）
	}

	void updateOdom()
	{
		currentTime[0] = io->currentTime();
		double updateSince = currentTime[0] - lastTime[0];
		w_sum[0] += gyroSensor->w();//vector w_sum[0]は最終更新からのx,y,z方向の角速度の合計値
		sensingSteps[0]++;

		if(updateSince >= update_period[0]){
			/* オドメトリ、TFの更新処理 */
			// 平均の角速度を求める
			auto w = w_sum[0] / sensingSteps[0]; // wは最終更新からのx,y,z方向の角速度の平値
			// 旋回角度をクォータニオンへ変換
			geometry_msgs::Quaternion odomQuat = tf::createQuaternionMsgFromYaw(th);
			// Create a TransformedStamped message to send to tf
			// /odom ⇔ /base_footprintフレーム間の座標関係の配信
			geometry_msgs::TransformStamped odomTrans;
			odomTrans.header.stamp.fromSec(currentTime[0]); // 現在時間 simtime
			odomTrans.header.frame_id = odom_frame_id_;
			odomTrans.child_frame_id = child_frame_id_;

			// Substitute coordinate transformation data from odometry data and send using Transform Broadcaster
			odomTrans.transform.translation.x = x;
			odomTrans.transform.translation.y = y;
			odomTrans.transform.translation.z = 0.0;
			odomTrans.transform.rotation = odomQuat;
			tfBroadcaster_.sendTransform(odomTrans);

			// The navigation stack publishes nav_msgs / Odometry messages for velocities information
			// 車輪オドメトリ情報のパブリッシュ
			nav_msgs::Odometry odom;
			odom.header.stamp.fromSec(currentTime[0]);
			odom.header.frame_id = odom_frame_id_;

			odom.pose.pose.position.x = x;	// x座標[m]
			odom.pose.pose.position.y = y;	// y座標[m]
			odom.pose.pose.position.z = 0.0;// z座標[m]
			odom.pose.pose.orientation = odomQuat;	// 姿勢のクォータニオン

			odom.child_frame_id = child_frame_id_;
			odom.twist.twist.linear.x = v[0];	// x方向速度[m/s]
			odom.twist.twist.linear.y = v[1];	// y方向速度[m/s]
//			odom.twist.twist.angular.z = omega;
//			odom.twist.twist.angular.z = gyroSensor->w().z();
			odom.twist.twist.angular.z = omega = w.z();	// z方向角速度の平均値[rad/s]

			// データパブリッシュ
			odom_pub_.publish(odom);
			ros::spinOnce();

			/* データ更新処理 */
			// ロボットの現在座標位置の更新
			updateCoord();

			lastTime[0] += update_period[0];
//			prevVth[2] = omega;
			// Debug
//			debug_vth[2] = debug_omega;
//			prevVth[2] = gyroSensor->w().z();

			// リセット処理（バッファ・カウンタ）
			w_sum[0].setZero();
			sensingSteps[0] = 0;

			// Debug
//			ROS_INFO("[ x y yaw ] = [ %lf %lf %lf ] ", x, y, th);

			// ROS_INFO("-------------------- Start output of odometry --------------------");
			// ROS_INFO("position: ( %lf, %lf, %lf )", x, y, 0.0);
			// ROS_INFO("orientation: ( %lf, %lf, %lf )", 0.0, 0.0, odomQuat.z);
			// ROS_INFO("linear: ( %lf, %lf, %lf )", v[0], v[1], 0.0);
			// ROS_INFO("angular: ( %lf, %lf, %lf )", 0.0, 0.0, vth[2]);
			// ROS_INFO("-------------------- End output of odometry --------------------");

		} // if(updateSince >= update_period[0]){

		/* シミュレータ時間のパブ */
		std_msgs::String msg;
		msg.data = to_string(io->currentTime());
		simtime_pub_.publish(msg);

	}

	void updateImu()
	{
		currentTime[1] = io->currentTime();
		double updateSince = currentTime[1] - lastTime[1];

		dv_sum[1] += accelSensor->dv();
		w_sum[1] += gyroSensor->w();
		sensingSteps[1]++;

		if(updateSince >= update_period[1]){
			currentTime[1] = io->currentTime();
//			auto dv = accelSensor->dv();
//			auto w = gyroSensor->w();
			auto dv = dv_sum[1] / sensingSteps[1];
			auto w = w_sum[1] / sensingSteps[1];
//			omega = w.z();
			sensor_msgs::Imu imu;

//			vth[2] = w.z();
			imuTh += w.z() * update_period[1];

			geometry_msgs::Quaternion imuQuat = tf::createQuaternionMsgFromYaw(imuTh);
			imu.header.stamp.fromSec(currentTime[1]);
			imu.header.frame_id = imu_frame_id_;
			imu.linear_acceleration.x = dv.x();
			imu.linear_acceleration.y = dv.y();
			imu.linear_acceleration.z = dv.z();
			imu.angular_velocity.x = w.x();
			imu.angular_velocity.y = w.y();
			imu.angular_velocity.z = w.z();
			imu.orientation = imuQuat;

			dv_sum[1].setZero();
			w_sum[1].setZero();
			sensingSteps[1] = 0;
			ros::spinOnce();
			imu_pub_.publish(imu);

			lastTime[1] += update_period[1];

//			ROS_INFO("-------------------- Start output of imu --------------------");
//			ROS_INFO("Accel Sensor  dvx: %lf, dvy: %lf, dvz: %lf", dv.x(), dv.y(), dv.z());
//			ROS_INFO("Gyro Sensor  wx: %lf, wy: %lf, wz: %lf", w.x(), w.y(), w.z());
//			ROS_INFO("-------------------- End output of imu --------------------");
		}
	}

	virtual bool control() override
	{

		current_time = io->currentTime();

		double updateSince = current_time - last_time_2;

		if(actuationMode == Link::JointVelocity){
			// When actuation mode is velocity.
			if(cmd_vel[0] > 0.0)
			{
				int test = 1;
			}
			wheels[0]->dq_target() = 1 * clamp(cmd_vel[0] - cmd_vel_th[2] * d, MIN_VEL, MAX_VEL) / r;
			wheels[1]->dq_target() = 1 * clamp(cmd_vel[0] + cmd_vel_th[2] * d, MIN_VEL, MAX_VEL) / r;

			if(updateSince >= update_period[0] )
			{

				// 周速度を求める
				Vector2 wheel_v;
				wheel_v[0] = r *(1 * clamp(cmd_vel[0] - cmd_vel_th[2] * d, MIN_VEL, MAX_VEL) / r);
				wheel_v[1] = r *(1 * clamp(cmd_vel[0] + cmd_vel_th[2] * d, MIN_VEL, MAX_VEL) / r);
				
				// 車体速度
				Vector3 v_local;
				v_local[0] = (wheel_v[0]+ wheel_v[1]) / 2.0;
				v_local[1] = 0.0;
				
				// ROS_INFO("control : wheel_v[0]: %lf, wheel_v[1] %lf, v_local[0]: %lf, v_local[1]: %lf ]", wheel_v[0], wheel_v[1], v_local[0], v_local[1]);
				
				// 座標と回転角度の変化量
				double deltaX = (v_local[0] * cos(th_local) - v_local[1] * sin(th_local)) * update_period[0];
				double deltaY = (v_local[0] * sin(th_local) + v_local[1] * cos(th_local)) * update_period[0];
				double deltaTh = ((omega + prevVth[1]) / 2.0) * update_period[0];

				x_local  += deltaX;
				y_local  += deltaY;
				th_local += deltaTh;

				// ROS_INFO("control : delta theta: %lf, [ x: %lf, y: %lf, yaw: %lf ], omega %lf, prevVth[1] %lf ", deltaTh, x_local, y_local, th_local, omega , prevVth[1]);

				last_time_2 = current_time;
			}

		} else if(actuationMode == Link::JointTorque)
		{
			constexpr double wheelRadius = 0.076;
			constexpr double halfAxleWidth = 0.142;
			constexpr double kdd = 5.0;
			double dq_target[2];
			
			{
				double dq_x = cmd_vel[0] / wheelRadius;
				double dq_yaw = cmd_vel[2] * halfAxleWidth / wheelRadius;
				dq_target[0] = dq_x - dq_yaw;
				dq_target[1] = dq_x + dq_yaw;
			}
			
			for(int i=0; i < 2; ++i)
			{
				auto wheel = wheels[i];
				wheel->u() = kdd * (dq_target[i] - wheel->dq());
			}
			// // When actuation mode is torque.
			// static const double K = 35.0;	// rotorInertia = 1.0e-06
			// double dq_target[2];		// Target angular velocity.
			// dq_target[0] = clamp(cmd_vel[0] - cmd_vel_th[2] * d, MIN_VEL, MAX_VEL) / r;
			// dq_target[1] = clamp(cmd_vel[0] + cmd_vel_th[2] * d, MIN_VEL, MAX_VEL) / r;

			// for(int i = 0; i < 2; ++i){
			// 	double q = wheels[i]->q();

			// 	double dq = (q - qprev[i]) / dt;	// Current angular velocity.

			// 	wheels[i]->u() = K * (dq_target[i] - dq);	// Torque value given to tire.
			// 	qprev[i] = wheels[i]->q();	// Update previous joint angular.
			// }
		}

		updateImu();
		updateOdom();

		auto T = ioBody->rootLink()->position();
		Vector3 rpy = rpyFromRot(T.rotation());

		if(current_time - last_time >= 1.0){
			ofs << current_time << "," << T.translation().x() << ","  << T.translation().y() << "," << 0.0 << endl;
			last_time = current_time;
		}
		if((rpy[2] > 0.0 && ptheta > 0.0) || (rpy[2] < 0.0 && ptheta < 0.0)){
			count += fabs(rpy[2] - ptheta);
		} else {
			count += fabs(rpy[2] + ptheta);
		}
//		ROS_INFO("rpy[2]: %lf, ptheta: %lf", rpy[2], ptheta);
		ptheta = rpy[2];

		
//		ROS_INFO("actual theta: %lf", rpy[2]);

		return true;
	}

	virtual void stop() override
	{
		auto T = ioBody->rootLink()->position();
		Vector3 rpy = rpyFromRot(T.rotation());

		ROS_INFO("odom theta: %lf, actual theta: %lf", th, count);
		x = y = th = 0.0;
		for(int i = 0; i < sizeof(v)/sizeof(v[0]); ++i){
			v[i] = vth[i] = 0.0;
		}

		cmd_vel_sub_.shutdown();
		odom_pub_.shutdown();
		imu_pub_.shutdown();
		ofs.close();

	}
};

CNOID_IMPLEMENT_SIMPLE_CONTROLLER_FACTORY(MegaRoverRosController)
