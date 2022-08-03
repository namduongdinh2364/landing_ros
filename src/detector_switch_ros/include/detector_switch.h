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
	ros::Timer pose_loop_;
	ros::Timer stable_loop_;
	ros::Subscriber sub_mavros_local_position_;
	ros::Time last_time_whycon_, last_time_apriltag_, last_time_aruco_;
	ros::Publisher pub_desPose_;

	geometry_msgs::PoseStamped apriltag_Pose_, whycon_Pose_, aruco_Pose_, cur_pose_;
	double marker_size_;
	int type_;
	int numStableWhy, numStableAru, numStableApr;
	bool detected_apriltag, detected_whycon, detected_aruco;
	bool stable_apriltag, stable_whycon, stable_aruco;
public:

	DetectorSwitch(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private);
	void getPoseArucoCallback(const geometry_msgs::PoseStamped& msg);
	void getPoseApriltagCallback(const tf2_msgs::TFMessage& msg);
	void getPoseWhyconCallback(const geometry_msgs::PoseArray& msg);
	void pubPoseCallback(const ros::TimerEvent& event);
	void setstatusCallback(const ros::TimerEvent& event);
	void stableCheckCallback(const ros::TimerEvent& event);
	void mavrosPose_Callback(const geometry_msgs::PoseStamped& msg);
};

#endif /* DETECTOR_SWITCH_H */