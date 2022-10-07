#include <ros/ros.h>
#include <eigen3/Eigen/Core>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/CompanionProcessStatus.h>
#include <tf/tf.h>
#include <eigen3/Eigen/Geometry>
#include <sensor_msgs/Imu.h>
#include <iostream>
#include <sstream>
#include <fstream>
#include <unistd.h>
#include <ctime>
#include <string>
#include <tf2/LinearMath/Quaternion.h>
#include <cstdlib>
#include "std_msgs/String.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Int16.h"

#include <stdio.h>
#include <stdlib.h>
#include <mavros_msgs/PositionTarget.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <std_srvs/SetBool.h>
#include <std_msgs/Bool.h>
#include <nav_msgs/Odometry.h>

#include <tf/transform_listener.h>
// #include <eigen_conversions/eigen_msg.h>

#include <dynamic_reconfigure/server.h>
#include <pid_controller/PIDControllerConfig.h>

#include "pid_controller/States.h"

#include "pid.h"

#define FACTORZ			0.2 // Descend Factor

#define MAX_ERROR		10
#define MAX_ALTITUDE	20

/* type mode control */
#define	POSITION_MODE	0
#define	VELOCITY_MODE	4

#define	DISTANCE_ON_MARKER(x)	x*(-1)

using namespace std;
using namespace sensor_msgs;
using namespace Eigen;

enum class MAV_STATE {
  MAV_STATE_UNINIT,
  MAV_STATE_BOOT,
  MAV_STATE_CALIBRATIN,
  MAV_STATE_STANDBY,
  MAV_STATE_ACTIVE,
  MAV_STATE_CRITICAL,
  MAV_STATE_EMERGENCY,
  MAV_STATE_POWEROFF,
  MAV_STATE_FLIGHT_TERMINATION,
};


class Controller
{
private:
	ros::NodeHandle nh_;
	ros::NodeHandle nh_private_;
	ros::Subscriber state_sub, imu_sub, local_position_sub;
	ros::Subscriber marker_pose_sub, decrese_height_sub;
	ros::Subscriber mavtwistSub_, kalman_sub;
	ros::Subscriber odom_sub, setTarget_sub;

	ros::Publisher cam_info_pub;
	ros::Publisher velocity_pub, local_pos_pub, setRaw_pub;
	ros::Publisher systemstatusPub_;
	ros::Publisher kalman_pub;

	ros::Publisher debugPIDx_pub, debugPIDy_pub;

	ros::Time last_request_;
	ros::Time lastTime;
	ros::Timer Control_Timer_, checkStatus_timer_;

	Matrix3f R;
	Eigen::Matrix3d cam2drone_matrix_, mavros_imu_data_;
	Eigen::Vector3d targetPos_, markerPos_, markerPosNEU_;
	Eigen::Vector3d mavPos_;
	Eigen::Vector3d mavVel_, mavRate_;

	Eigen::Vector4d mavAtt_;

	ros::ServiceServer return_home_service_, start_land_service_;

	ros::ServiceClient arming_client_;
	ros::ServiceClient set_mode_client_;

	mavros_msgs::State current_state_;
	mavros_msgs::SetMode offb_set_mode_, land_set_mode_;
	mavros_msgs::CommandBool arm_cmd_;

	bool sim_enable_;
	bool startLanding, decrease_height_;

	MAV_STATE companion_state_ = MAV_STATE::MAV_STATE_ACTIVE;

	enum FlightState {
		WAITING_FOR_HOME_POSE,
		MISSION_EXECUTION,
		LANDING,
		LANDED,
		HOME
	} node_state;

	geometry_msgs::Pose home_pose_;
	bool received_home_pose;
	double initTargetPos_x_, initTargetPos_y_, initTargetPos_z_, initYAW_;
	int control_mode_;

	pid_controller::States states;

	PID* pidx;
	PID* pidy;
	PID* pidYAW;


	PID* pidvx;
	PID* pidvy;

	double pre_errorVx, pre_errorVy, Vdkp, Vdkd;

public:
	Controller(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private)
	: nh_(nh),
	nh_private_(nh_private),
	node_state(MISSION_EXECUTION),
	startLanding(false),
	decrease_height_(false)
	{
		nh_private_.param<bool>("enable_sim", sim_enable_, true);
		nh_private_.param<double>("init_pos_x", initTargetPos_x_, 0.0);
		nh_private_.param<double>("init_pos_y", initTargetPos_y_, 0.0);
		nh_private_.param<double>("init_pos_z", initTargetPos_z_, 8.0);
		nh_private_.param<double>("init_yaw", initYAW_, 0.0);
		nh_private_.param<int>("control_mode", control_mode_, 0);

		pidx = new PID(0.5, -0.5, 1, 0, 0.01); // max, min, kp, kd, ki
		pidy = new PID(0.5, -0.5, 1, 0, 0.01);

		// pidvx = new PID(1.0, -1.0, 1, 0, 0.01); // max, min, kp, kd, ki
		// pidvy = new PID(1.0, -1.0, 1, 0, 0.01);

		pidYAW = new PID(0.75, -0.75, 0.2, 0.01, 0.00085);

		/** setup subscribe and public*/ 
		local_position_sub = nh_.subscribe
			("mavros/local_position/pose", 1, &Controller::mavrosPose_Callback, this, ros::TransportHints().tcpNoDelay());
		
		mavtwistSub_ = nh_.subscribe
			("mavros/local_position/velocity_local", 1, &Controller::mavtwistCallback, this, ros::TransportHints().tcpNoDelay());
		
		state_sub = nh_.subscribe
			("mavros/state", 10, &Controller::mavrosState_Callback, this, ros::TransportHints().tcpNoDelay());

		imu_sub = nh_.subscribe
			("/mavros/imu/data", 10, &Controller::imuCallback, this, ros::TransportHints().tcpNoDelay());

		odom_sub = nh_.subscribe
			("/mavros/local_position/odom", 1, &Controller::uavOdomCallback, this);

		setTarget_sub = nh_.subscribe
			("cmd/local_position/target", 1, &Controller::SetTargetCallback, this);



		local_pos_pub = nh_.advertise <geometry_msgs::PoseStamped>
			("mavros/setpoint_position/local", 10);

		debugPIDx_pub = nh_.advertise <geometry_msgs::PoseStamped>
			("debug_pidx", 10);

		debugPIDy_pub = nh_.advertise <geometry_msgs::PoseStamped>
			("debug_pidy", 10);

		arming_client_ = nh_.serviceClient <mavros_msgs::CommandBool>
			("mavros/cmd/arming");

		set_mode_client_ = nh_.serviceClient <mavros_msgs::SetMode>
			("mavros/set_mode");

		velocity_pub   = nh_.advertise <geometry_msgs::TwistStamped>
			("/mavros/setpoint_velocity/cmd_vel", 10 );

		setRaw_pub = nh_.advertise<mavros_msgs::PositionTarget>("/mavros/setpoint_raw/local",10);

		systemstatusPub_ = nh_.advertise<mavros_msgs::CompanionProcessStatus>
			("mavros/companion_process/status", 1);

		kalman_pub = nh_.advertise<pid_controller::States>
			("/kalman_states", 10);

		kalman_sub = nh_.subscribe
			("/predicted_states", 10, &Controller::kalmanCallback, this, ros::TransportHints().tcpNoDelay());

		Control_Timer_ = nh_.createTimer(ros::Duration(0.01), &Controller::ControlStateMachine_Callback, this);

		checkStatus_timer_ = nh_.createTimer(ros::Duration(1), &Controller::CheckStatus_Callback, this);

		marker_pose_sub = nh_.subscribe
			("/tf_marker", 1, &Controller::ReceivedMarkerPose_Callback, this, ros::TransportHints().tcpNoDelay());

		return_home_service_ = nh_.advertiseService("return_home", &Controller::EnableReturnHome_Service, this);

		start_land_service_ = nh_.advertiseService("start_land", &Controller::EnableLand_Service, this);

		decrese_height_sub = nh_.subscribe
			("/decrease_height", 1, &Controller::CheckAllowDecreaseHeight_Callback, this, ros::TransportHints().tcpNoDelay());


		/* Rotation matrix from camera to UAV. It's customizable with different setup */
		cam2drone_matrix_ << 0.0 , -1.0 , 0.0 , -1.0 , 0.0 , 0.0 , 0.0 , 0.0 , -1.0;
		targetPos_ << initTargetPos_x_, initTargetPos_y_, initTargetPos_z_;
		markerPos_ << 0.0, 0.0, initTargetPos_z_;
		mavVel_ << 0.0, 0.0, 0.0;
		pre_errorVx = 0;
		pre_errorVy = 0;
		Vdkp = 0.15;
		Vdkd = 0.01;

	}
	
	void SetTargetCallback(const geometry_msgs::PoseStamped &msg)
	{
		targetPos_ << msg.pose.position.x, msg.pose.position.y, msg.pose.position.z;
		ROS_INFO_STREAM("TARGET: " << targetPos_);
	}

	void kalmanCallback(const pid_controller::States& msg)
	{

	}

	void uavOdomCallback(const nav_msgs::Odometry::ConstPtr& odom)
	{
		// store current position in our planner
		// tf::poseMsgToEigen(odom->pose.pose, mavPos_);
		mavPos_ = toEigen(odom->pose.pose.position);

		// store current vleocity
		// tf::vectorMsgToEigen(odom->twist.twist.linear, mavVel_);
	}

	void imuCallback(const sensor_msgs::Imu& msg)
	{
		double x, y, z, w;
		Quaterniond quat;

		x = msg.orientation.x;
		y = msg.orientation.y;
		z = msg.orientation.z;
		w = msg.orientation.w;
		/* making a quaternion of position */
		quat = Eigen::Quaterniond(w, x, y, z);
		/* making rotation matrix from quaternion */
		mavros_imu_data_ = quat.toRotationMatrix();
	}

	void CheckAllowDecreaseHeight_Callback(const std_msgs::Bool &msg){

		decrease_height_ = msg.data;
	}

	bool EnableLand_Service(std_srvs::SetBool::Request &request, std_srvs::SetBool::Response &response) {

		ROS_INFO_STREAM("Start landing on marker.");
		startLanding = true;

		return true;
	}

	bool EnableReturnHome_Service(std_srvs::SetBool::Request &request, std_srvs::SetBool::Response &response) {
		
		ROS_INFO_STREAM("Service called the return home.");
		node_state = HOME;

		return true;
	}

	// void imuCallback(const sensor_msgs::Imu::ConstPtr &msg)
	// {
	// 	double x,y,z,w;

	// 	x = msg->orientation.x;
	// 	y = msg->orientation.y;
	// 	z = msg->orientation.z;
	// 	w = msg->orientation.w;
	// 	/* making a quaternion of position */
	// 	Quaternionf quat;
	// 	quat = Eigen::Quaternionf(w,x,y,z);

	// 	/*making rotation matrix from quaternion*/
	// 	R = quat.toRotationMatrix();
	// 	// cout << "R=" << endl << R << endl;
	// }

	Eigen::Vector3d toEigen(const geometry_msgs::Point &p)
	{
		Eigen::Vector3d ev3(p.x, p.y, p.z);
		return ev3;
	}

	inline Eigen::Vector3d toEigen(const geometry_msgs::Vector3 &v3)
	{
		Eigen::Vector3d ev3(v3.x, v3.y, v3.z);
		return ev3;
	}

	void mavrosPose_Callback(const geometry_msgs::PoseStamped &msg)
	{
		if (!received_home_pose) {

			received_home_pose = true;
			home_pose_ = msg.pose;
			ROS_INFO_STREAM("Home pose initialized to: " << home_pose_);
		}

		mavPos_ = toEigen(msg.pose.position);
		mavAtt_(0) = msg.pose.orientation.w;
		mavAtt_(1) = msg.pose.orientation.x;
		mavAtt_(2) = msg.pose.orientation.y;
		mavAtt_(3) = msg.pose.orientation.z;
	}

	/* getting the state */
	void mavrosState_Callback(const mavros_msgs::State::ConstPtr& msg)
	{
		current_state_ = *msg;
	}

	void pubSystemStatus() {
		mavros_msgs::CompanionProcessStatus msg;

		msg.header.stamp = ros::Time::now();
		msg.component = 196;  // MAV_COMPONENT_ID_AVOIDANCE
		msg.state = (int)companion_state_;

		systemstatusPub_.publish(msg);
	}

	void CheckStatus_Callback(const ros::TimerEvent &event)
	{
		if (sim_enable_) {
			// Enable OFFBoard mode and arm automatically
			// This is only run if the vehicle is simulated
			arm_cmd_.request.value = true;

			offb_set_mode_.request.custom_mode = "OFFBOARD";

			if (current_state_.mode != "OFFBOARD" && (ros::Time::now() - last_request_ > ros::Duration(5.0))) {
				
				if (set_mode_client_.call(offb_set_mode_) && offb_set_mode_.response.mode_sent) {
					ROS_INFO("Offboard enabled");
				}

				last_request_ = ros::Time::now();
			}
			else {

				if (!current_state_.armed && (ros::Time::now() - last_request_ > ros::Duration(5.0))) {
					
					if (arming_client_.call(arm_cmd_) && arm_cmd_.response.success) {
						ROS_INFO("Vehicle armed");
					}

					last_request_ = ros::Time::now();
				}
			}
		}

		/* Check lost control */
		if (mavPos_(2) > MAX_ALTITUDE) {

			ROS_INFO_STREAM("Current height : " << mavPos_(2));
			ROS_INFO_STREAM("Return home pose : " << home_pose_);

			node_state = HOME;
		}

		pubSystemStatus();
	}

	void ControlStateMachine_Callback(const ros::TimerEvent &event)
	{
		switch (node_state) {

			case WAITING_FOR_HOME_POSE: {

				ros::Rate pause(2.0);

				ROS_INFO_STREAM("Waiting for home pose...");

				while (ros::ok() && !received_home_pose) {
					ros::spinOnce();
					pause.sleep();
				}

				ROS_INFO("Got pose! Drone Ready to be armed.");
				/* Change state */
				node_state = MISSION_EXECUTION;
			}
			break;

			case MISSION_EXECUTION: {

				// states.Xm = markerPos_(0);
				// states.Ym = markerPos_(1);
				// states.Vx = mavVel_(0);
				// states.Vy = mavVel_(1);

				// kalman_pub.publish(states);

				switch (control_mode_)
				{
					case POSITION_MODE: {

						Eigen::Vector3d desiredPos;

						if (startLanding) {
							/* Marker ----> NEU Frame*/
							markerPosNEU_ = mavros_imu_data_ * markerPos_;
							desiredPos(0) = markerPosNEU_(0) + mavPos_(0);
							desiredPos(1) = markerPosNEU_(1) + mavPos_(1);

							if (decrease_height_) {
								desiredPos(2) =  mavPos_(2) - FACTORZ;
							}
							else {
								desiredPos(2) =  mavPos_(2);
							}

							if (DISTANCE_ON_MARKER(markerPos_(2)) >= 2.0) {
								control_mode_ = VELOCITY_MODE;
							}
						}
						else {

							desiredPos = targetPos_;
						}

						pubPositionToFCU(desiredPos);
					}
						break;

					case VELOCITY_MODE: {
						
						Eigen::Vector3d desiredPos;

						if (startLanding) {

							desiredPos(0) = markerPos_(0);
							desiredPos(1) = markerPos_(1);

							if (decrease_height_) {
								desiredPos(2) =  mavPos_(2) - FACTORZ;
							}
							else {
								desiredPos(2) =  mavPos_(2);
							}

							// if (mavPos_(2) < 2.0) {
							if (DISTANCE_ON_MARKER(markerPos_(2)) < 2.0) {
								/* Change control mode */
								control_mode_ = POSITION_MODE;
							}

						}
						else {

							desiredPos(0) = targetPos_(0) - mavPos_(0);
							desiredPos(1) = targetPos_(1) - mavPos_(1);
							desiredPos(2) = targetPos_(2);
						}
						
						// ROS_INFO_STREAM("Desired pose: \n" << desiredPos);
						pubVelocityToFCU(desiredPos);
					}
						break;

					default:
						break;
				}

				/* Check by z axis in camera frame */
				if (DISTANCE_ON_MARKER(markerPos_(2)) < 0.7 && startLanding) {

					node_state = LANDED;
				}

			}
			break;

			case LANDING:
				break;

			case LANDED: {

				ROS_INFO("Landed. Please set to position control and disarm.");
				ROS_INFO_STREAM("Current Hight GPS: " << mavPos_(2));
				ROS_INFO_STREAM("Current Hight Marker: " << -markerPos_(2));

				if(current_state_.mode != "AUTO.LAND") {

					land_set_mode_.request.custom_mode = "AUTO.LAND";

					if(set_mode_client_.call(land_set_mode_) && land_set_mode_.response.mode_sent) {

						ROS_INFO("AUTO LANDING MODE is required");
					}
				}

				Control_Timer_.stop();
				checkStatus_timer_.stop();
			}
			break;

			case HOME: {
				geometry_msgs::PoseStamped desPose;

				desPose.header.stamp = ros::Time::now();
				desPose.pose.position.x = 0;
				desPose.pose.position.y = 0;
				desPose.pose.position.z = 0.5;

				local_pos_pub.publish(desPose);
			}
			break;
		}
	}

	void ReceivedMarkerPose_Callback(const geometry_msgs::PoseStamped &msg){

		Eigen::Vector3d markerInCamFrame;
		/* Marker ----> Drone Frame*/
		markerInCamFrame << msg.pose.position.x, msg.pose.position.y, msg.pose.position.z;
		markerPos_ = cam2drone_matrix_ * markerInCamFrame;

		// Round cm
		markerPos_(0) = round(markerPos_(0)*100) / 100;
		markerPos_(1) = round(markerPos_(1)*100) / 100;
		markerPos_(2) = round(markerPos_(2)*100) / 100;

		// ROS_INFO_STREAM("Distance to Marker: " << markerPos_);
		// received_marker_pose =true;
		// std::cout << "point des" << point_des << std::endl;
	};

	void mavtwistCallback(const geometry_msgs::TwistStamped &msg) {

		mavVel_ = toEigen(msg.twist.linear);
		mavRate_ = toEigen(msg.twist.angular);
	}

	void pubVelocityToFCU(Eigen::Vector3d &target_position)
	{
		/* Setpoint, Process Variable, Sample time for Vx */
		float Vx = 0.0;
		float Vy = 0.0;
		float Vyaw = 0.0;

		double Poutx, Pouty;
		double Ioutx, Iouty;
		double Doutx, Douty;
		double timeBetweenMarkers;

		/* Position target object to publish */
		mavros_msgs::PositionTarget pos;
		//FRAME_LOCAL_NED to move WRT to body_ned frame
		pos.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;

		if (!lastTime.isZero()) {

			timeBetweenMarkers = (ros::Time::now() - lastTime).toSec();

			lastTime = ros::Time::now();

			if (0 == timeBetweenMarkers)
			{
				// ROS_ERROR("delta_t is 0"at time: %f", ros::Time::now().toSec());
				return;
			}

		} else {

			ROS_INFO("lastTime is 0, doing nothing");
			lastTime = ros::Time::now();

			return;
		}

		// double timeBetweenMarkers = (ros::Time::now() - lastTime).toSec();
		// double timeBetweenMarkers = 0.01;
		// lastTime = ros::Time::now();
		// ROS_INFO_STREAM("TIME: " << timeBetweenMarkers);

		if (abs(target_position(0)) > MAX_ERROR || abs(target_position(1)) > MAX_ERROR) {

			pos.header.stamp = ros::Time::now();
			pos.header.frame_id = "base_link";
			pos.type_mask = 2019; // Mask for Vx, Vy, Z pos and Yaw rate

			pos.position.z = target_position(2);

			pos.velocity.x = 0;
			pos.velocity.y = 0;
			// pos.velocity.z = 1.0;

			pos.yaw_rate = 0.0;

			setRaw_pub.publish(pos);

			return;
		}


		Vx = (float) pidx->calculate(target_position(0), 0.0, timeBetweenMarkers, &Poutx, &Ioutx, &Doutx);
		Vy = (float) pidy->calculate(target_position(1), 0.0, timeBetweenMarkers, &Pouty, &Iouty, &Douty);
		// Vyaw = (float) pidYAW->calculate(0, initYAW_, timeBetweenMarkers);

		// ROS_INFO_STREAM("Desired velocity x: " << Vx);
		// ROS_INFO_STREAM("Desired velocity y: " << Vy);


		double errorVx = (Vx - mavVel_(0));
		double errorVy = (Vy - mavVel_(1));

		double Ex = errorVx * Vdkp; // + (errorVx - pre_errorVx) * Vdkd / timeBetweenMarkers;
		double Ey = errorVy * Vdkp; // + (errorVy - pre_errorVy) * Vdkd / timeBetweenMarkers;

		// control acceleration

		pos.header.stamp = ros::Time::now();
		pos.header.frame_id = "base_link";
		pos.type_mask = 2019; // Mask for Vx, Vy, Z pos and Yaw rate

		pos.position.z = target_position(2);

		pos.velocity.x = mavVel_(0) + Ex;
		pos.velocity.y = mavVel_(1) + Ey;
		// pos.velocity.z = 1.0;

		pos.yaw_rate = 0.0;

		setRaw_pub.publish(pos);

#if 0
		geometry_msgs::PoseStamped debugPIDx, debugPIDy;
		debugPIDx.header.stamp = ros::Time::now();
		debugPIDx.pose.position.x = Poutx;
		debugPIDx.pose.position.y = Ioutx;
		debugPIDx.pose.position.z = Doutx;


		debugPIDy.header.stamp = ros::Time::now();
		debugPIDy.pose.position.x = Pouty;
		debugPIDy.pose.position.y = Iouty;
		debugPIDy.pose.position.z = Douty;


		debugPIDx_pub.publish(debugPIDx);
		debugPIDy_pub.publish(debugPIDy);

#endif // debug

	}

	void pubPositionToFCU(const Eigen::Vector3d &target_position)
	{
		geometry_msgs::PoseStamped desPose;

		desPose.header.stamp = ros::Time::now();
		desPose.pose.position.x = target_position(0);
		desPose.pose.position.y = target_position(1);
		desPose.pose.position.z = target_position(2);

		local_pos_pub.publish(desPose);
	}

	void callback(dynamic_pid::PIDControllerConfig &config, uint32_t level) {

		ROS_INFO("Reconfigure Request: %f %f %f", config.dKP, config.dKI, config.dKD);

		// if (max_fb_acc_ != config.dKP) {
			pidx->setP(config.dKP);
			pidy->setP(config.dKP);
			ROS_INFO("Reconfigure request : dKP = %.2f ", config.dKP);
		// }
		// else if (Kpos_x_ != config.dKI) {
			pidx->setI(config.dKI);
			pidy->setI(config.dKI);
			ROS_INFO("Reconfigure request : dKI  = %.2f  ", config.dKI);
		// }
		// else if (Kpos_y_ != config.dKD) {
			pidx->setD(config.dKD);
			pidy->setD(config.dKD);
			ROS_INFO("Reconfigure request : dKD  = %.2f  ", config.dKI);
		// }

			Vdkp = config.VdKP;
			Vdkd = config.VdKD;
			ROS_INFO("Reconfigure request : vdKD  = %.2f  ", config.VdKP);

	}
};



int main(int argc, char **argv)
{
	ros::init(argc, argv, "offboard_control");
	ros::NodeHandle nh("");
	ros::NodeHandle nh_private("~");

	Controller Controller(nh, nh_private);
	
	dynamic_reconfigure::Server<dynamic_pid::PIDControllerConfig> server;
	dynamic_reconfigure::Server<dynamic_pid::PIDControllerConfig>::CallbackType f;

	f = boost::bind(&Controller::callback, Controller, _1, _2);
	server.setCallback(f);

	ros::spin();
	return 0;
}
