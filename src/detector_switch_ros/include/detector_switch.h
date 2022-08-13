#ifndef DETECTOR_SWITCH_H
#define DETECTOR_SWITCH_H

#include <ros/ros.h>
#include <iostream>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf2_msgs/TFMessage.h>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <eigen3/Eigen/Dense>
#include "landing.h"
#include <geometry_msgs/PoseArray.h>
#include <std_srvs/SetBool.h>
#include <std_msgs/Header.h>
#include <std_msgs/Bool.h>
#include <math.h>

#define ANGLE_1 2.0
#define ANGLE_2 10.0
#define ANGLE_3 10.0
#define PI 3.14159265

using namespace std;

class DetectorSwitch
{
private:
	ros::NodeHandle nh_;
	ros::NodeHandle nh_private_;
	ros::Subscriber sub_tf_whycon_;
	ros::Subscriber sub_tf_apriltag_;
	ros::Subscriber sub_tf_aruco_;
	ros::Publisher pub_tf_;

	ros::Publisher pub_aruco_;
	ros::Publisher pub_whycon_;
	ros::Publisher pub_apirltag_;




	ros::Publisher pub_decrease_height;

	ros::Timer pose_loop_;
	ros::Timer stable_loop_;
	ros::Subscriber sub_mavros_local_position_;
	ros::Time last_time_whycon_, last_time_apriltag_, last_time_aruco_;
	ros::Publisher pub_desPose_;
	ros::ServiceServer land_service_;
	geometry_msgs::PoseStamped apriltag_Pose_, whycon_Pose_, aruco_Pose_, cur_pose_;
	std_msgs::Bool decrease;
	// std_msgs::Bool detect_marker;
	double max_height, range_err, range_err_x, range_err_y;
	int type_;
	int numStableWhy, numStableAru, numStableApr;
	bool detected_apriltag, detected_whycon, detected_aruco;
	bool stable_apriltag, stable_whycon, stable_aruco,land;
	bool lock_height;
	double range1, range2, range3;
	std_msgs::Header detect_marker;

public:

	DetectorSwitch(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private);
	void getPoseArucoCallback(const geometry_msgs::PoseStamped& msg);
	void getPoseApriltagCallback(const tf2_msgs::TFMessage& msg);
	void getPoseWhyconCallback(const geometry_msgs::PoseStamped& msg);
	void pubPoseCallback(const ros::TimerEvent& event);
	void setstatusCallback(const ros::TimerEvent& event);
	void stableCheckCallback(const ros::TimerEvent& event);
	void mavrosPose_Callback(const geometry_msgs::PoseStamped& msg);
	bool landCallback(std_srvs::SetBool::Request &request, std_srvs::SetBool::Response &response);

};

#endif /* DETECTOR_SWITCH_H */