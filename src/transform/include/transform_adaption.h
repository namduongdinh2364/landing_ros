#ifndef TRANSFORM_ADAPTION_H
#define TRANSFORM_ADAPTION_H

#include <ros/ros.h>
#include <sstream>
#include <unistd.h>
#include <math.h>
#include <ctime>
#include "std_msgs/Float32.h"
#include "std_msgs/String.h"
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <eigen3/Eigen/Dense>
#include <sensor_msgs/Imu.h>
#include <tf2_msgs/TFMessage.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <std_srvs/SetBool.h>
#include <mav_planning_msgs/Curve.h>
#include "landing.h"


using namespace std;
using namespace Eigen;

enum class Axis {
  ROLL,
  PTICH,
  YAW,
};

enum class Coordinates {
  UAV_NED,
  UAV_MARKER,
};

class transformAdaption
{
private:
	ros::NodeHandle nh_;
	ros::NodeHandle nh_private_;
	ros::Publisher pub_desPose_;
	ros::Publisher pub_desYaw_;
	ros::Publisher pub_curvePoints_;
	ros::Subscriber sub_mavros_imu_data_;
	ros::Subscriber sub_mavros_local_position_;
	ros::Subscriber sub_marker_pose_;
	ros::ServiceClient land_client_;
	ros::Timer markerPose_loop_;
	ros::Timer check_loop_;

	geometry_msgs::PoseStamped desPose_;
	ros::Time last_time_trans_, stable_time_trans_, timeout_repeat_trans_;
	Eigen::Vector3f cur_pose_, markerPose_, cam_Pose_, drone_Pose_, m2neu_Pose_;
	Eigen::Matrix3f mavros_imu_data_;
	Eigen::Matrix3f cam2drone_matrix_;
	mav_planning_msgs::Curve curve_points_;

	double cur_yaw_;
	double m_yaw_;
	int detect_failed_repeat_, stable_loop_;
	bool accept_landing;
	bool marker_detected_;
	bool locked_trans_, locked_landing_, locked_inc_altitude_;

public:
	transformAdaption(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private);
	double get_Euler_from_quat(double xq, double yq, double zq, double wq, Axis type);
	double get_err_yaw(double d_yaw, double m_yaw);
	void mavrosPose_Callback(const geometry_msgs::PoseStamped& msg);
	void get_marker_pose_Callback(const geometry_msgs::PoseStamped& msg);
	void imu_Callback(const sensor_msgs::Imu& msg);
	void loopCallback(const ros::TimerEvent& event);
	void checkloopCallback(const ros::TimerEvent& event);
	void check_landing_constraints_loop();
	void check_detect_timeout_loop();
	void get_marker_pose_with_distance();
};

#endif