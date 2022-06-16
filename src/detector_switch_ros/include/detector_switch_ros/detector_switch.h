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

using namespace std;

class DetectorSwitch
{
private:
	ros::NodeHandle nh_;
	ros::NodeHandle nh_private_;
	ros::Subscriber sub_tf_cctag_;
	ros::Subscriber sub_tf_apriltag_;
	ros::Subscriber sub_tf_aruco_;
	ros::Publisher pub_tf_;
	ros::Timer pose_loop_;
	ros::Timer status_loop_;
	ros::Subscriber sub_mavros_local_position_;
	ros::Time last_time_cctag_, last_time_apriltag_, last_time_aruco_;

	geometry_msgs::PoseStamped apriltag_Pose_, cctag_Pose_, aruco_Pose_, cur_pose_;
	double marker_size_;
	int type_;
	bool detected_apriltag, detected_cctag, detected_aruco;
public:

	DetectorSwitch(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private);
	void getPoseArucoCallback(const geometry_msgs::PoseStamped& msg);
	void getPoseApriltagCallback(const tf2_msgs::TFMessage& msg);
	void getPoseCCtagCallback(const geometry_msgs::PoseWithCovarianceStamped& msg);
	void pubPoseCallback(const ros::TimerEvent& event);
	void setstatusCallback(const ros::TimerEvent& event);
	void mavrosPose_Callback(const geometry_msgs::PoseStamped& msg);
};

#endif /* CCTAG_DETECTOR_H */