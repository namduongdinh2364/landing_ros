#include "detector_switch_ros/detector_switch.h"

DetectorSwitch::DetectorSwitch(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private)
	: nh_(nh), nh_private_(nh_private)
{
	pub_tf_ = nh_.advertise<geometry_msgs::PoseStamped>
		("/tf_marker", 1);

	sub_mavros_local_position_ = nh_private_.subscribe
		("/mavros/local_position/pose", 1, &DetectorSwitch::mavrosPose_Callback, this, ros::TransportHints().tcpNoDelay());
	sub_tf_apriltag_ = nh_.subscribe
		("/tf", 1, &DetectorSwitch::getPoseApriltagCallback, this, ros::TransportHints().tcpNoDelay());
	sub_tf_cctag_ = nh_.subscribe
		("/tf_cctag", 1, &DetectorSwitch::getPoseCCtagCallback, this, ros::TransportHints().tcpNoDelay());
	pose_loop_ = nh_.createTimer(ros::Duration(0.1), &DetectorSwitch::pubPoseCallback, this);
	status_loop_ = nh_.createTimer(ros::Duration(0.5), &DetectorSwitch::setstatusCallback, this);
}

void DetectorSwitch::setstatusCallback(const ros::TimerEvent& event)
{
	// detected_apriltag = false;
	// detected_aruco = false;
	// detected_cctag = false;
}

void DetectorSwitch::pubPoseCallback(const ros::TimerEvent& event)
{
	if((TIME_NOW - last_time_cctag_) > TIME_DURATION(0.5)) {
		detected_cctag = false;
	}

	if((TIME_NOW - last_time_apriltag_) > TIME_DURATION(0.5)) {
		detected_apriltag = false;
	}

	if((TIME_NOW - last_time_aruco_) > TIME_DURATION(0.5)) {
		detected_aruco = false;
	}
	switch (type_)
	{
	case 1:
		if (detected_apriltag)
			pub_tf_.publish(apriltag_Pose_);
		break;
	case 2:
		if (detected_aruco)
			pub_tf_.publish(aruco_Pose_);
		break;
	case 3:
		if (detected_cctag)
			pub_tf_.publish(cctag_Pose_);
		break;
	default:
		if (detected_cctag) {
			if (sqrt(pow(cctag_Pose_.pose.position.x,2) + pow(cctag_Pose_.pose.position.y,2)) > 1.5 
				|| (!detected_apriltag && !detected_aruco))
			{
				pub_tf_.publish(cctag_Pose_);
				std::cout << "cctag is used" << std::endl;
			}
			else if (detected_apriltag) {
				pub_tf_.publish(apriltag_Pose_);
				std::cout << "apriltag is used" << std::endl;
			}
			else if (detected_aruco) {
				pub_tf_.publish(aruco_Pose_);
				std::cout << "aruco is used" << std::endl;
			}
		}
		else {
			if (detected_apriltag) {
				pub_tf_.publish(apriltag_Pose_);
				std::cout << "apriltag is used" << std::endl;
			}
			else if (detected_aruco) {
				pub_tf_.publish(aruco_Pose_);
				std::cout << "aruco is used" << std::endl;
			}
		}
		break;
	}
}

void DetectorSwitch::mavrosPose_Callback(const geometry_msgs::PoseStamped& msg)
{
	cur_pose_ = msg;
}

void DetectorSwitch::getPoseArucoCallback(const geometry_msgs::PoseWithCovarianceStamped& msg)
{
	last_time_aruco_ = TIME_NOW;
	aruco_Pose_.pose.position.x = msg.pose.pose.position.x;
	aruco_Pose_.pose.position.y = msg.pose.pose.position.y;
	aruco_Pose_.pose.position.z = msg.pose.pose.position.z;
	aruco_Pose_.pose.orientation.x = msg.pose.pose.orientation.x;
	aruco_Pose_.pose.orientation.y = msg.pose.pose.orientation.y;
	aruco_Pose_.pose.orientation.z = msg.pose.pose.orientation.z;
	aruco_Pose_.pose.orientation.w = msg.pose.pose.orientation.w;
	detected_aruco = true;
}
void DetectorSwitch::getPoseApriltagCallback(const tf2_msgs::TFMessage& msg)
{
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
void DetectorSwitch::getPoseCCtagCallback(const geometry_msgs::PoseWithCovarianceStamped& msg)
{
	last_time_cctag_ = TIME_NOW;
	cctag_Pose_.pose.position.x = msg.pose.pose.position.x;
	cctag_Pose_.pose.position.y = msg.pose.pose.position.y;
	cctag_Pose_.pose.position.z = msg.pose.pose.position.z;
	cctag_Pose_.pose.orientation.x = msg.pose.pose.orientation.x;
	cctag_Pose_.pose.orientation.y = msg.pose.pose.orientation.y;
	cctag_Pose_.pose.orientation.z = msg.pose.pose.orientation.z;
	cctag_Pose_.pose.orientation.w = msg.pose.pose.orientation.w;
	detected_cctag = true;
}
