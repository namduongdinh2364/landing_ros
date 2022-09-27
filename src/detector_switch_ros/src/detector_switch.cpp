#include "detector_switch.h"
#include <string>

DetectorSwitch::DetectorSwitch(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private)
	: nh_(nh), nh_private_(nh_private),
	detected_apriltag(false),
	detected_aruco(false),
	detected_whycon(false),
	startLand(false),
	lock_height(true)
{
	pub_decrease_height = nh_.advertise<std_msgs::Bool> ("/decrease_height", 1);

	pub_tf_marker_ = nh_.advertise<geometry_msgs::PoseStamped>
		("/tf_marker", 10);

	sub_mavros_local_position_ = nh_private_.subscribe
		("/mavros/local_position/pose", 1, &DetectorSwitch::mavrosPose_Callback, this, ros::TransportHints().tcpNoDelay());

	sub_tf_aruco_ = nh_.subscribe
		("/aruco_detector/pose", 1, &DetectorSwitch::getPoseArucoCallback, this, ros::TransportHints().tcpNoDelay());

	sub_tf_apriltag_ = nh_.subscribe
		("/tf", 1, &DetectorSwitch::getPoseApriltagCallback, this, ros::TransportHints().tcpNoDelay());

	sub_tf_whycon_ = nh_.subscribe
		("/whycon/camera/pose", 1, &DetectorSwitch::getPoseWhyconCallback, this, ros::TransportHints().tcpNoDelay());

	pose_loop_ = nh_.createTimer(ros::Duration(0.05), &DetectorSwitch::pubPoseCallback, this);

	stable_loop_ = nh_.createTimer(ros::Duration(0.1), &DetectorSwitch::stableCheckCallback, this);

	start_land_service_ = nh_.advertiseService("start_land_1", &DetectorSwitch::landCallback, this);

	numStableWhy = 0;
	numStableAru = 0;
	numStableApr = 0;

}

void DetectorSwitch::mavrosPose_Callback(const geometry_msgs::PoseStamped& msg)
{
	cur_pose_ = msg;
}

bool DetectorSwitch::landCallback(std_srvs::SetBool::Request &request, std_srvs::SetBool::Response &response) {
  startLand = true;
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

	if (stable_aruco) {
		numStableAru ++;
		stable_aruco = false;
	}

	if (stable_apriltag) {
		numStableApr ++;
		stable_apriltag = false;
	}

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
	if((TIME_NOW - last_time_whycon_) > TIME_DURATION(1.0)) {
		detected_whycon = false;
		numStableWhy = 0;
		// ROS_WARN_STREAM("Whycon is unstable");
	}

	if((TIME_NOW - last_time_aruco_) > TIME_DURATION(1.0)) {
		detected_aruco = false;
		numStableAru = 0;
		// ROS_WARN_STREAM("Aruco is unstable");
	}

	if((TIME_NOW - last_time_apriltag_) > TIME_DURATION(1.0)) {
		detected_apriltag = false;
		numStableApr = 0;
		// ROS_WARN_STREAM("Apriltag is unstable");
	}

	if (!detected_whycon && !detected_aruco && !detected_apriltag) {
		// ROS_WARN("Marker is NOT detected");
		return;
	}


	if (!startLand) {
		max_height = cur_pose_.pose.position.z;
	}

	if (lock_height && startLand){
		lock_height = false;
		max_height = cur_pose_.pose.position.z;
		
		range1 = max_height* tan(ANGLE_1*PI/180);
		range2 = max_height* tan(ANGLE_1*PI/180) *2 / 3;
		range3 = max_height* tan(ANGLE_3*PI/180) *1 / 3;
		// std::cout << max_height << std::endl;
		// std::cout << "false" << std::endl;
	}

	/* Update queue */
	priority_1 = {detected_whycon, detected_aruco, detected_apriltag};
	priority_2 = {detected_aruco, detected_whycon, detected_apriltag};
	priority_3 = {detected_apriltag, detected_aruco, detected_whycon};

	arrayPose_1 = {whyconPose, arucoPose, aprilPose};
	arrayPose_2 = {arucoPose, whyconPose, aprilPose};
	arrayPose_3 = {aprilPose, arucoPose, whyconPose};

	// cout << "size queue1: " << priority_1.size() << endl;
	// cout << "size queue2: " << priority_2.size() << endl;
	// cout << "size queue3: " << priority_3.size() << endl;

	if (cur_pose_.pose.position.z > (2* max_height)/3)
	{
		int i;

		for (i = 0; i < 3; i++) {
			if (priority_1[i]) {
				break;
			}
		}

		ROS_INFO_STREAM("Priority 1: " << i);
		range_err = sqrt(pow(arrayPose_1[i](0), 2) + pow(arrayPose_1[i](1), 2));

		if (range1 < range_err) {
			decrease.data = false;
		}
		else {
			decrease.data = true;
		}

		markerPose.header.frame_id = std::to_string(i);
		markerPose.header.stamp = ros::Time::now();
		markerPose.pose.position.x = arrayPose_1[i](0);
		markerPose.pose.position.y = arrayPose_1[i](1);
		markerPose.pose.position.z = arrayPose_1[i](2);
	}

	else if (cur_pose_.pose.position.z > (max_height/3))
	{
		int i;

		for (i = 0; i < 3; i++) {
			if (priority_2[i]) {
				break;
			}
		}

		ROS_INFO_STREAM("Priority 2: " << i);
		range_err = sqrt(pow(arrayPose_2[i](0), 2) + pow(arrayPose_2[i](1), 2));

		if (range2 < range_err) {
			decrease.data = false;
		}
		else {
			decrease.data = true;
		}

		markerPose.header.frame_id = std::to_string(i);
		markerPose.header.stamp = ros::Time::now();
		markerPose.pose.position.x = arrayPose_2[i](0);
		markerPose.pose.position.y = arrayPose_2[i](1);
		markerPose.pose.position.z = arrayPose_2[i](2);
	}

	else
	{
		int i;

		for (i = 0; i < 3; i++) {
			if (priority_3[i]) {
				break;
			}
		}

		ROS_INFO_STREAM("Priority 3: " << i);
		range_err = sqrt(pow(arrayPose_3[i](0), 2) + pow(arrayPose_3[i](1), 2));

		if (range3 < range_err) {
			decrease.data = false;
		}
		else {
			decrease.data = true;
		}

		markerPose.header.frame_id = std::to_string(i);
		markerPose.header.stamp = ros::Time::now();
		markerPose.pose.position.x = arrayPose_3[i](0);
		markerPose.pose.position.y = arrayPose_3[i](1);
		markerPose.pose.position.z = arrayPose_3[i](2);
	}

	pub_tf_marker_.publish(markerPose);
	pub_decrease_height.publish(decrease);

}

/* Get Position of marker Aruco in camera frame */
void DetectorSwitch::getPoseArucoCallback(const geometry_msgs::PoseStamped& msg)
{
	last_time_aruco_ = TIME_NOW;
	// aruco_Pose_.header.frame_id = "aruco";
	// aruco_Pose_ = msg;
	arucoPose << msg.pose.position.x, msg.pose.position.y, msg.pose.position.z;
	stable_aruco = true;
	detected_aruco = true;

}

/* Get Position of marker AprilTag in camera frame */
void DetectorSwitch::getPoseApriltagCallback(const tf2_msgs::TFMessage& msg)
{
	if(msg.transforms[0].child_frame_id == "apriltag_bundle") {
		last_time_apriltag_ = TIME_NOW;
		// apriltag_Pose_.header.frame_id = "apirltag";
		// apriltag_Pose_.pose.position.x = msg.transforms[0].transform.translation.x;
		// apriltag_Pose_.pose.position.y = msg.transforms[0].transform.translation.y;
		// apriltag_Pose_.pose.position.z = msg.transforms[0].transform.translation.z;
		// apriltag_Pose_.pose.orientation.x = msg.transforms[0].transform.rotation.x;
		// apriltag_Pose_.pose.orientation.y = msg.transforms[0].transform.rotation.y;
		// apriltag_Pose_.pose.orientation.z = msg.transforms[0].transform.rotation.z;
		// apriltag_Pose_.pose.orientation.w = msg.transforms[0].transform.rotation.w;
		aprilPose << msg.transforms[0].transform.translation.x, msg.transforms[0].transform.translation.y, msg.transforms[0].transform.translation.z;
		stable_apriltag = true;
		detected_apriltag = true;

	}
}

void DetectorSwitch::getPoseWhyconCallback(const geometry_msgs::PoseStamped& msg)
{
	last_time_whycon_ = TIME_NOW;
	// whycon_Pose_.header.frame_id = "whycon";
	// whycon_Pose_.pose.position.x = msg.pose.position.x;
	// whycon_Pose_.pose.position.y = msg.pose.position.y;
	// whycon_Pose_.pose.position.z = msg.pose.position.z;
	// whycon_Pose_.pose.orientation.x = msg.pose.orientation.x;
	// whycon_Pose_.pose.orientation.y = msg.pose.orientation.y;
	// whycon_Pose_.pose.orientation.z = msg.pose.orientation.z;
	// whycon_Pose_.pose.orientation.w = msg.pose.orientation.w;
	whyconPose << msg.pose.position.x, msg.pose.position.y, msg.pose.position.z;
	stable_whycon = true;
	detected_whycon = true;
}
