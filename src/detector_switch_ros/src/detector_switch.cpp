#include "detector_switch.h"

DetectorSwitch::DetectorSwitch(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private)
	: nh_(nh), nh_private_(nh_private)
{
	detected_apriltag = false;
	detected_aruco = false;
	detected_whycon = false;
	pub_tf_ = nh_.advertise<geometry_msgs::PoseStamped>
		("/tf_marker", 1);

	sub_mavros_local_position_ = nh_private_.subscribe
		("/mavros/local_position/pose", 1, &DetectorSwitch::mavrosPose_Callback, this, ros::TransportHints().tcpNoDelay());
	sub_tf_aruco_ = nh_.subscribe
		("/aruco_bundle/pose", 1, &DetectorSwitch::getPoseArucoCallback, this, ros::TransportHints().tcpNoDelay());
	sub_tf_apriltag_ = nh_.subscribe
		("/tf", 1, &DetectorSwitch::getPoseApriltagCallback, this, ros::TransportHints().tcpNoDelay());
	sub_tf_whycon_ = nh_.subscribe
		("/whycon/pose", 1, &DetectorSwitch::getPoseWhyconCallback, this, ros::TransportHints().tcpNoDelay());
	pose_loop_ = nh_.createTimer(ros::Duration(0.1), &DetectorSwitch::pubPoseCallback, this);
	type_ = 4;
}

void DetectorSwitch::pubPoseCallback(const ros::TimerEvent& event)
{
	if((TIME_NOW - last_time_whycon_) > TIME_DURATION(0.5)) {
		detected_whycon = false;
	}

	if((TIME_NOW - last_time_apriltag_) > TIME_DURATION(0.5)) {
		detected_apriltag = false;
	}

	if((TIME_NOW - last_time_aruco_) > TIME_DURATION(0.5)) {
		detected_aruco = false;
	}
	switch (type_) {
		case 1:
			if(detected_apriltag)
				pub_tf_.publish(apriltag_Pose_);
			break;
		case 2:
			if(detected_aruco)
				pub_tf_.publish(aruco_Pose_);
			break;
		case 3:
			if(detected_whycon)
				pub_tf_.publish(whycon_Pose_);
			break;
		default:
			/*
			 * If whycon is detected, it will be used,
			 * but if aruco or apriltag is detected they will replaced whycon.
			 * Apriltag has highest priority, Aruco is the next and then Whycon.
			 * 
			 * TODO: Need to improve data structure (e.g. used a vector of varialbe pair)
			 */
			if (detected_whycon) {
				if(sqrt(pow(whycon_Pose_.pose.position.x,2) + pow(whycon_Pose_.pose.position.y,2)) > 1.5 
					|| (!detected_apriltag && !detected_aruco))
				{
					pub_tf_.publish(whycon_Pose_);
					std::cout << "whycon is used" << std::endl;
				}
				else if(detected_apriltag && cur_pose_.pose.position.z < 4 || !detected_aruco && detected_apriltag) {
					pub_tf_.publish(apriltag_Pose_);
					std::cout << "apriltag is used" << std::endl;
				}
				else if(detected_aruco) {
					pub_tf_.publish(aruco_Pose_);
					std::cout << "aruco is used" << std::endl;
				}
			}
			else {
				if (detected_apriltag && cur_pose_.pose.position.z < 4 || !detected_aruco && detected_apriltag) {
					pub_tf_.publish(apriltag_Pose_);
					std::cout << "whycon isn't detected and apriltag is used" << std::endl;
				}
				else if(detected_aruco) {
					pub_tf_.publish(aruco_Pose_);
					std::cout << "whycon isn't detected and aruco is used" << std::endl;
				}
			}
			break;
	}
}

void DetectorSwitch::mavrosPose_Callback(const geometry_msgs::PoseStamped& msg)
{
	cur_pose_ = msg;
}

void DetectorSwitch::getPoseArucoCallback(const geometry_msgs::PoseStamped& msg)
{
	last_time_aruco_ = TIME_NOW;
	aruco_Pose_ = msg;
	detected_aruco = true;
}
void DetectorSwitch::getPoseApriltagCallback(const tf2_msgs::TFMessage& msg)
{
	if(msg.transforms[0].child_frame_id == "apriltag_bundle") {
		last_time_apriltag_ = TIME_NOW;
		apriltag_Pose_.pose.position.x = msg.transforms[0].transform.translation.x;
		apriltag_Pose_.pose.position.y = msg.transforms[0].transform.translation.y;
		apriltag_Pose_.pose.position.z = msg.transforms[0].transform.translation.z;
		apriltag_Pose_.pose.orientation.x = msg.transforms[0].transform.rotation.x;
		apriltag_Pose_.pose.orientation.y = msg.transforms[0].transform.rotation.y;
		apriltag_Pose_.pose.orientation.z = msg.transforms[0].transform.rotation.z;
		apriltag_Pose_.pose.orientation.w = msg.transforms[0].transform.rotation.w;
		detected_apriltag = true;
	}
}
void DetectorSwitch::getPoseWhyconCallback(const geometry_msgs::PoseArray& msg)
{
	last_time_whycon_ = TIME_NOW;
	whycon_Pose_.pose.position.x = msg.poses[0].position.x;
	whycon_Pose_.pose.position.y = msg.poses[0].position.y;
	whycon_Pose_.pose.position.z = msg.poses[0].position.z;
	whycon_Pose_.pose.orientation.x = msg.poses[0].orientation.x;
	whycon_Pose_.pose.orientation.y = msg.poses[0].orientation.y;
	whycon_Pose_.pose.orientation.z = msg.poses[0].orientation.z;
	whycon_Pose_.pose.orientation.w = msg.poses[0].orientation.w;
	detected_whycon = true;
}
