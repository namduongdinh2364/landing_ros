#include "transform_adaption.h"

transformAdaption::transformAdaption(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private)
	: nh_(nh), nh_private_(nh_private), cur_yaw_(0), locked_trans_(false), accept_landing(false),
	locked_landing_(false)
{
	/*Public*/
	pub_desPose_ = nh_.advertise<geometry_msgs::PoseStamped>
		("cmd/set_desposition/local", 1);
	pub_desYaw_ = nh_.advertise<std_msgs::Float32>
		("cmd/reference/yaw", 1);

	/*Subscribe*/
	sub_mavros_imu_data_ = nh_.subscribe
		("/mavros/imu/data", 1, &transformAdaption::imu_Callback, this, ros::TransportHints().tcpNoDelay());
	sub_mavros_local_position_ = nh_.subscribe
		("/mavros/local_position/pose", 1, &transformAdaption::mavrosPose_Callback, this, ros::TransportHints().tcpNoDelay());
	sub_marker_pose_ = nh_.subscribe
		("/tf_marker", 1, &transformAdaption::get_marker_pose_Callback, this, ros::TransportHints().tcpNoDelay());

	markerPoseUpdate_loop_ = nh_.createTimer(ros::Duration(0.01), &transformAdaption::loopCallback, this);
	check_loop_ = nh_.createTimer(ros::Duration(0.1), &transformAdaption::checkloopCallback, this);

	land_client_ = nh_.serviceClient<std_srvs::SetBool>("/land");
	/* Rotation matrix from camera to UAV. It's customizable with different setup */
	cam2drone_matrix_ << 0.0 , -1.0 , 0.0 , -1.0 , 0.0 , 0.0 , 0.0 , 0.0 , -1.0;

	timeout_repeat_trans_ = TIME_NOW;
}

double transformAdaption::get_Euler_from_quat(double xq, double yq, double zq, double wq, Axis type)
{
	tf2::Quaternion q;
	double roll, pitch, yaw, res;

	q.setValue(xq, yq, zq, wq);
	tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
	switch (type)
	{
		case Axis::ROLL:
			res = roll;
			break;
		case Axis::PTICH:
			res = pitch;
			break;
		default:
			res = yaw;
			break;
	}

	return res;
}

void transformAdaption::loopCallback(const ros::TimerEvent& event) {
	/*
	 * If one marker is detected, marker pose will be published.
	 */
	if (locked_trans_)
	{
		locked_trans_ = false;
		desPose_.pose.position.x = markerPose_(0);
		desPose_.pose.position.y = markerPose_(1);
		desPose_.pose.position.z = 0.0;
		if (cur_pose_(2) < 0.5) {
			accept_landing = true;
		}
		pub_desPose_.publish(desPose_);

		// if (abs(YAW_ANGLE(m_yaw_)) >= ERROR_ACCEPTANCE_YAW_DEGREES) {
		// 	std_msgs::Float32 msg_yaw;
		// 	double err_yaw = get_err_yaw(cur_yaw_, m_yaw_);
		// 	msg_yaw.data = err_yaw;
		// 	pub_desYaw_.publish(msg_yaw);
		// }
	}
}
void transformAdaption::checkloopCallback(const ros::TimerEvent& event) {
	// if(MAX_ALTITUDE < cur_pose_(2)) {
	// 	accept_landing = true;
	// 	desPose_.pose.position.z = cur_pose_(0);
	// 	desPose_.pose.position.z = cur_pose_(1);
	// 	desPose_.pose.position.z = 1.0;
	// 	pub_desPose_.publish(desPose_);
	// 	ROS_WARN_STREAM("Enable Landing: UAV is approaded the maximum altitude");
	// }
	check_landing_constraints_loop();
	// check_detect_timeout_loop();
}

void transformAdaption::mavrosPose_Callback(const geometry_msgs::PoseStamped& msg)
{
	cur_pose_(0) = msg.pose.position.x;
	cur_pose_(1) = msg.pose.position.y;
	cur_pose_(2) = msg.pose.position.z;
}
void transformAdaption::get_marker_pose_Callback(const geometry_msgs::PoseStamped& msg)
{
	/** Reset last transform time */
	last_time_trans_ = TIME_NOW;
	marker_detected_ = true;
	/**
	 * The detection stabilizes after 3 sec.
	 * detect_failed_repeat and locked_inc_altitude will be reset.
	 */
	if(TIME_NOW - stable_time_trans_ > TIME_DURATION(3.0)) {
		detect_failed_repeat_ = 0;
		locked_inc_altitude_ = false;
	}

	if(!locked_landing_) {
		locked_trans_ = true;	/* Allow to publish marker pose */
		m_yaw_ = get_Euler_from_quat(msg.pose.orientation.x,
									 msg.pose.orientation.y,
									 msg.pose.orientation.z,
									 msg.pose.orientation.w, Axis::YAW);
		/* Marker ----> Drone */
		cam_Pose_[0] = msg.pose.position.x;
		cam_Pose_[1] = msg.pose.position.y;
		cam_Pose_[2] = msg.pose.position.z;
		drone_Pose_ = cam2drone_matrix_ * cam_Pose_;
		/* Marker ----> NEU */
		m2neu_Pose_ = mavros_imu_data_ * drone_Pose_;

		markerPose_(0) = m2neu_Pose_[0] + cur_pose_(0);
		markerPose_(1) = m2neu_Pose_[1] + cur_pose_(1);
		markerPose_(2) = m2neu_Pose_[2] + cur_pose_(2);
	}
}

void transformAdaption::imu_Callback(const sensor_msgs::Imu& msg)
{
	double x, y, z, w;
	Quaternionf quat;

	x = msg.orientation.x;
	y = msg.orientation.y;
	z = msg.orientation.z;
	w = msg.orientation.w;
	cur_yaw_ = get_Euler_from_quat(x, y, z, w, Axis::YAW);
	/* making a quaternion of position */
	quat = Eigen::Quaternionf(w, x, y, z);
	/* making rotation matrix from quaternion */
	mavros_imu_data_ = quat.toRotationMatrix();
}

void transformAdaption::check_landing_constraints_loop()
{
	if (accept_landing && !locked_landing_) {
		locked_landing_ = true;
		std_srvs::SetBool land_cmd;
		land_cmd.request.data = true;
		land_client_.call(land_cmd);
		ROS_WARN_STREAM("LANDING is required");
		markerPoseUpdate_loop_.stop();
		check_loop_.stop();
	}
}

void transformAdaption::check_detect_timeout_loop()
{
	/**
	 * If can't detect marker during 2 second.
	 * Get detect again failed more than maxium,
	 * Landing will be called.
	 * TODO: Need to improve
	 */
	if((TIME_NOW - last_time_trans_) > TIME_DURATION(2.0) && marker_detected_) {
		static double new_altitude = 0;
		if(TIME_NOW - timeout_repeat_trans_ > TIME_DURATION(2.0)) {
			detect_failed_repeat_ ++;
			timeout_repeat_trans_ = TIME_NOW;
			ROS_INFO("Can't detect Marker %d", detect_failed_repeat_);
		}
		if(!locked_inc_altitude_) {
			new_altitude = cur_pose_(2) + INCREASE_ALTITUDE_NOT_DETECT;
		}
		locked_inc_altitude_ = true;
		stable_time_trans_ = TIME_NOW;
		desPose_.pose.position.x = cur_pose_(0);
		desPose_.pose.position.y = cur_pose_(1);
		desPose_.pose.position.z = new_altitude;
		if((TIME_NOW - last_time_trans_ > TIME_DURATION(2 * MAX_REPEAT_DETECT)) || detect_failed_repeat_ == MAX_REPEAT_DETECT) {
			accept_landing = true;
			ROS_WARN_STREAM("Detection failed: Timeout");
		}
		pub_desPose_.publish(desPose_);
	}
}

double transformAdaption::get_err_yaw(double d_yaw, double m_yaw)
{
	double err_degrees;
	err_degrees = YAW_ANGLE(d_yaw) - YAW_ANGLE(m_yaw);

	return err_degrees * PI/180;
}
