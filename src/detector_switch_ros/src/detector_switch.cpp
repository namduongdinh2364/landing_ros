#include "detector_switch.h"

DetectorSwitch::DetectorSwitch(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private)
	: nh_(nh), nh_private_(nh_private)
{
	detected_apriltag = false;
	detected_aruco = false;
	detected_whycon = false;
	pub_decrease_height = nh_.advertise<std_msgs::Bool> ("/decrease_height", 1);
	pub_tf_ = nh_.advertise<geometry_msgs::PoseStamped>
		("/tf_marker", 1);
	pub_desPose_ = nh_.advertise<geometry_msgs::PoseStamped>
		("cmd/set_desposition/local", 1);

	sub_mavros_local_position_ = nh_private_.subscribe
		("/mavros/local_position/pose", 1, &DetectorSwitch::mavrosPose_Callback, this, ros::TransportHints().tcpNoDelay());
	sub_tf_aruco_ = nh_.subscribe
		("/aruco_detector/pose", 1, &DetectorSwitch::getPoseArucoCallback, this, ros::TransportHints().tcpNoDelay());
	sub_tf_apriltag_ = nh_.subscribe
		("/tf", 1, &DetectorSwitch::getPoseApriltagCallback, this, ros::TransportHints().tcpNoDelay());
	sub_tf_whycon_ = nh_.subscribe
		("/whycon/camera/pose", 1, &DetectorSwitch::getPoseWhyconCallback, this, ros::TransportHints().tcpNoDelay());
	pose_loop_ = nh_.createTimer(ros::Duration(0.1), &DetectorSwitch::pubPoseCallback, this);
	stable_loop_ = nh_.createTimer(ros::Duration(0.2), &DetectorSwitch::stableCheckCallback, this);
	land_service_ = nh_.advertiseService("land3", &DetectorSwitch::landCallback, this);
	lock_height = true;
	type_ = 4;
	numStableWhy = 0;
	numStableAru = 0;
	numStableApr = 0;
}


bool DetectorSwitch::landCallback(std_srvs::SetBool::Request &request, std_srvs::SetBool::Response &response) {
  land = true;
  ROS_WARN_STREAM("Start detect");
  return true;
}

void DetectorSwitch::stableCheckCallback(const ros::TimerEvent& event)
{
	/*
	 * Every 0.2s check again to see if marker is detected.
	 * If marker is continuously detected in 0.6s.
	 * => marker is detected stably.
	 */
	if (stable_whycon) {
		numStableWhy ++;
		stable_whycon = false;
	}
	// else {
	//     numStableWhy = 0;
	//     detected_whycon = false;
	// }

	if (stable_aruco) {
		numStableAru ++;
		stable_aruco = false;
	}
	// else {
	//     numStableAru = 0;
	//     detected_aruco = false;
	// }

	if (stable_apriltag) {
		numStableApr ++;
		stable_apriltag = false;
	}
	// else {
	//     numStableApr = 0;
	//     detected_apriltag = false;
	// }

	if (numStableWhy > STABLENUMBER) {
		numStableWhy = STABLENUMBER;
		detected_whycon = true;
	}
	if (numStableAru > STABLENUMBER) {
		numStableAru = STABLENUMBER;
		detected_aruco = true;
	}
	if (numStableApr > STABLENUMBER) {
		numStableApr = STABLENUMBER;
		detected_apriltag = true;
	}

}

void DetectorSwitch::pubPoseCallback(const ros::TimerEvent& event)
{
	/*
	 * If then 0.3s the marker isn't detected.
	 * => marker is unstable and not detect.
	 */
	// if((TIME_NOW - last_time_whycon_) > TIME_DURATION(0.3)) {
	// 	detected_whycon = false;
	// 	// ROS_WARN_STREAM("Whycon is unstable");
	// }

	// if((TIME_NOW - last_time_aruco_) > TIME_DURATION(0.3)) {
	// 	detected_aruco = false;
	// 	// ROS_WARN_STREAM("Aruco is unstable");
	// }

	// if((TIME_NOW - last_time_apriltag_) > TIME_DURATION(0.3)) {
	// 	detected_apriltag = false;
	// 	// ROS_WARN_STREAM("Apriltag is unstable");
	// }

	if (!land) {
		max_height = cur_pose_.pose.position.z;
	}

	if (lock_height && land){
		lock_height = false;
		max_height = cur_pose_.pose.position.z;
		// std::cout << max_height << std::endl;
		// std::cout << "false" << std::endl;
	}

	if (cur_pose_.pose.position.z > (2* max_height)/3)
	{
		if (detected_whycon){
			// std::cout << "Whycon is used "  << std::endl;
			range_err = sqrt(pow(whycon_Pose_.pose.position.x,2) + pow(whycon_Pose_.pose.position.y,2));
			if (range_err < 0.6)
			{
				pub_tf_.publish(whycon_Pose_);
				decrease.data = true;
				pub_decrease_height.publish(decrease);
			}
			else
			{
				pub_tf_.publish(whycon_Pose_);
				decrease.data = false;
				pub_decrease_height.publish(decrease);
			}
		} 
		else if (detected_aruco)
		{	
			// std::cout << "Aruco is used "  << std::endl;
			range_err = sqrt(pow(aruco_Pose_.pose.position.x,2) + pow(aruco_Pose_.pose.position.y,2));
			if (range_err < 0.6)
			{
				pub_tf_.publish(aruco_Pose_);
				decrease.data = true;
				pub_decrease_height.publish(decrease);
			}
			else
			{
				pub_tf_.publish(aruco_Pose_);
				decrease.data = false;
				pub_decrease_height.publish(decrease);
			}
		} 
		else if (detected_apriltag)
		{	
			// std::cout << "Apirltag is used "  << std::endl;
			range_err = sqrt(pow(apriltag_Pose_.pose.position.x,2) + pow(apriltag_Pose_.pose.position.y,2));
			if (range_err < 0.6)
			{
				pub_tf_.publish(apriltag_Pose_);
				decrease.data = true;
				pub_decrease_height.publish(decrease);
			}
			else
			{
				pub_tf_.publish(apriltag_Pose_);
				decrease.data = false;
				pub_decrease_height.publish(decrease);
			}
		}
		else {
			// std::cout << "not have marker" << std::endl;
		}
	}
	else if (cur_pose_.pose.position.z > (max_height/3))
	{
		if (detected_aruco){
			// std::cout << "Aruco is used "  << std::endl;
			range_err = sqrt(pow(aruco_Pose_.pose.position.x,2) + pow(aruco_Pose_.pose.position.y,2));
			if (range_err < 0.35)
			{
				pub_tf_.publish(aruco_Pose_);
				decrease.data = true;
				pub_decrease_height.publish(decrease);
			}
			else
			{
				pub_tf_.publish(aruco_Pose_);
				decrease.data = false;
				pub_decrease_height.publish(decrease);
			}
			// pub_desPose_.publish(aruco_Pose_);
		} 
		else if (detected_whycon)
		{
			std::cout << "Whycon is used "  << std::endl;
			range_err = sqrt(pow(whycon_Pose_.pose.position.x,2) + pow(whycon_Pose_.pose.position.y,2));
			if (range_err < 0.35)
			{
				pub_tf_.publish(whycon_Pose_);
				decrease.data = true;
				pub_decrease_height.publish(decrease);
			}
			else
			{
				pub_tf_.publish(whycon_Pose_);
				decrease.data = false;
				pub_decrease_height.publish(decrease);
			}
		} 
		else if (detected_apriltag)
		{	
			std::cout << "Apirltag is used "  << std::endl;
			range_err = sqrt(pow(apriltag_Pose_.pose.position.x,2) + pow(apriltag_Pose_.pose.position.y,2));
			if (range_err < 0.35)
			{
				pub_tf_.publish(apriltag_Pose_);
				decrease.data = true;
				pub_decrease_height.publish(decrease);
			}
			else
			{
				pub_tf_.publish(apriltag_Pose_);
				decrease.data = false;
				pub_decrease_height.publish(decrease);
			}
		}
		else {
			// std::cout << "not have marker" << std::endl;
		}
	}
	else if (cur_pose_.pose.position.z < (max_height/3))
	{
		if (detected_apriltag){
			std::cout << "Apirltag is used "  << std::endl;
			range_err = sqrt(pow(apriltag_Pose_.pose.position.x,2) + pow(apriltag_Pose_.pose.position.y,2));
			if (range_err < 0.1)
			{	

				pub_tf_.publish(apriltag_Pose_);
				decrease.data = true;
				pub_decrease_height.publish(decrease);
			}
			else
			{
				pub_tf_.publish(apriltag_Pose_);
				decrease.data = false;
				pub_decrease_height.publish(decrease);
			}
		} 
		else if (detected_aruco)
		{	
			std::cout << "Aruco is used "  << std::endl;
			range_err = sqrt(pow(aruco_Pose_.pose.position.x,2) + pow(aruco_Pose_.pose.position.y,2));
			if (range_err < 0.1)
			{
				pub_tf_.publish(aruco_Pose_);
				decrease.data = true;
				pub_decrease_height.publish(decrease);
			}
			else
			{
				pub_tf_.publish(aruco_Pose_);
				decrease.data = false;
				pub_decrease_height.publish(decrease);
			}
		} 
		else if (detected_whycon)
		{	
			std::cout << "Whycon is used "  << std::endl;
			range_err = sqrt(pow(whycon_Pose_.pose.position.x,2) + pow(whycon_Pose_.pose.position.y,2));
			if (range_err < 0.1)
			{
				pub_tf_.publish(whycon_Pose_);
				decrease.data = true;
				pub_decrease_height.publish(decrease);
			}
			else
			{	
				
				pub_tf_.publish(whycon_Pose_);
				decrease.data = false;
				pub_decrease_height.publish(decrease);
			}
		}
		else {
			// std::cout << "not have marker at start" << std::endl;
		}
	}
	// switch (type_) {
	// 	case 1:
	// 		if(detected_apriltag)
	// 			pub_tf_.publish(apriltag_Pose_);
	// 		break;
	// 	case 2:
	// 		if(detected_aruco)
	// 			pub_tf_.publish(aruco_Pose_);
	// 		break;
	// 	case 3:
	// 		if(detected_whycon)
	// 			pub_desPose_.publish(whycon_Pose_);
	// 		break;
	// 	default:
	// 		/*
	// 		 * If whycon is detected, it will be used,
	// 		 * but if aruco or apriltag is detected they will replaced whycon.
	// 		 * Apriltag has highest priority, Aruco is the next and then Whycon.
	// 		 * 
	// 		 * TODO: Need to improve data structure (e.g. used a vector of varialbe pair)
	// 		 */
	// 		if (detected_whycon) {
	// 			if(sqrt(pow(whycon_Pose_.pose.position.x,2) + pow(whycon_Pose_.pose.position.y,2)) > 1.5 
	// 				|| (!detected_apriltag && !detected_aruco))
	// 			{
	// 				pub_desPose_.publish(whycon_Pose_);
	// 				std::cout << "Whycon is used | Z = " << cur_pose_.pose.position.z << std::endl;
	// 			}
	// 			else if(detected_apriltag && cur_pose_.pose.position.z < 4 || !detected_aruco && detected_apriltag) {
	// 				pub_tf_.publish(apriltag_Pose_);
	// 				std::cout << "Apriltag is used | Z = " << cur_pose_.pose.position.z << std::endl;
	// 			}
	// 			else if(detected_aruco) {
	// 				pub_tf_.publish(aruco_Pose_);
	// 				std::cout << "Aruco is used | Z = " << cur_pose_.pose.position.z << std::endl;
	// 			}
	// 		}
	// 		else {
	// 			if (detected_apriltag && cur_pose_.pose.position.z < 4 || !detected_aruco && detected_apriltag) {
	// 				pub_tf_.publish(apriltag_Pose_);
	// 				std::cout << "Apriltag is used | Z = " << cur_pose_.pose.position.z << std::endl;
	// 			}
	// 			else if(detected_aruco) {
	// 				pub_tf_.publish(aruco_Pose_);
	// 				std::cout << "Aruco is used | Z = " << cur_pose_.pose.position.z << std::endl;
	// 			}
	// 		}
	// 		break;
	// }
}

void DetectorSwitch::mavrosPose_Callback(const geometry_msgs::PoseStamped& msg)
{
	cur_pose_ = msg;
}

/* Get Position of marker Aruco in camera frame */
void DetectorSwitch::getPoseArucoCallback(const geometry_msgs::PoseStamped& msg)
{
	last_time_aruco_ = TIME_NOW;
	aruco_Pose_ = msg;
	stable_aruco = true;
	detected_aruco = true;

}

/* Get Position of marker AprilTag in camera frame */
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
		stable_apriltag = true;
		detected_apriltag = true;

		
	}
}

void DetectorSwitch::getPoseWhyconCallback(const geometry_msgs::PoseStamped& msg)
{
	last_time_whycon_ = TIME_NOW;
	whycon_Pose_.pose.position.x = msg.pose.position.x;
	whycon_Pose_.pose.position.y = msg.pose.position.y;
	whycon_Pose_.pose.position.z = msg.pose.position.z;
	whycon_Pose_.pose.orientation.x = msg.pose.orientation.x;
	whycon_Pose_.pose.orientation.y = msg.pose.orientation.y;
	whycon_Pose_.pose.orientation.z = msg.pose.orientation.z;
	whycon_Pose_.pose.orientation.w = msg.pose.orientation.w;
	stable_whycon = true;
	detected_whycon = true;
}
