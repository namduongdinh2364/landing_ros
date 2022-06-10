#include "transform_adaption.h"
#include "generate_curve.h"

transformAdaption::transformAdaption(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private)
	: nh_(nh), nh_private_(nh_private), cur_yaw_(0), locked_trans_(false), accept_landing(false),
	locked_landing_(false)
{
	/*Public*/
	pub_desPose_ = nh_.advertise<geometry_msgs::PoseStamped>
		("cmd/set_desposition/local", 1);
	pub_desYaw_ = nh_.advertise<std_msgs::Float32>
		("cmd/reference/yaw", 1);
	pub_curvePoints_ = nh_.advertise<mav_planning_msgs::Curve>
		("cmd/reference/curvepoints", 1);
	/*Subscribe*/
	sub_mavros_imu_data_ = nh_.subscribe
		("/mavros/imu/data", 1, &transformAdaption::imu_Callback, this, ros::TransportHints().tcpNoDelay());
	sub_mavros_local_position_ = nh_.subscribe
		("/mavros/local_position/pose", 1, &transformAdaption::mavrosPose_Callback, this, ros::TransportHints().tcpNoDelay());
	sub_marker_pose_ = nh_.subscribe
		("/tf_marker", 1, &transformAdaption::get_marker_pose_Callback, this, ros::TransportHints().tcpNoDelay());
	markerPose_loop_ = nh_.createTimer(ros::Duration(0.5), &transformAdaption::loopCallback, this);
	check_loop_ = nh_.createTimer(ros::Duration(0.1), &transformAdaption::checkloopCallback, this);

	land_client_ = nh_.serviceClient<std_srvs::SetBool>("/land");

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
	if (locked_trans_)
	{	
		locked_trans_ = false;
		if (cur_pose_(2) > ALTITUDE_CHANGE_METHOD + 1.0) {
			pub_curvePoints_.publish(curve_points_);
		}
		else {
			pub_desPose_.publish(desPose_);
		}
		
		if (abs(YAW_ANGLE(m_yaw_)) >= ERROR_ACCEPTANCE_YAW_DEGREES) {
			std_msgs::Float32 msg_yaw;
			double err_yaw = get_err_yaw(cur_yaw_, m_yaw_);
			msg_yaw.data = err_yaw;
			// pub_desYaw_.publish(msg_yaw);
		}
		
#ifdef LOG_INFO
	cout<< "Marker2NEU : " << desPose_.pose.position.x <<'\t'<< desPose_.pose.position.y << '\t' << desPose_.pose.position.z << endl;
	cout<< "Current Yaw : " << cur_yaw_ << endl;
	cout<< "Marker Yaw : " << m_yaw_ << endl;	
	cout << "===================================================" << endl;
#endif /* LOG_INFO */
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
	check_detect_timeout_loop();
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
		locked_trans_ = true;
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

		markerPose_(0) = PRECISION(markerPose_(0));
		markerPose_(1) = PRECISION(markerPose_(1));
		markerPose_(2) = PRECISION(markerPose_(2));

		if (cur_pose_(2) > ALTITUDE_CHANGE_METHOD + 1.0) {
			vector<Eigen::Vector3d> points;
			generate_curve::getIntersectionPoints(&points, markerPose_, cur_pose_);
			curve_points_.points.clear();
			curve_points_.num_vertix = points.size();
			for (size_t i = 0; i < points.size(); i++) {
				mav_planning_msgs::Point3D tmp;
				tmp.x = points.at(i)[0];
				tmp.y = points.at(i)[1];
				tmp.z = points.at(i)[2];
				curve_points_.points.push_back(tmp);
			}
			// if (cur_pose_(2) > 8.5) {
			// 	desPose_.pose.position.x = markerPose_(0);
			// 	desPose_.pose.position.y = markerPose_(1);
			// 	desPose_.pose.position.z = 8.0;
			// }
			// else if (cur_pose_(2) <= 8.5) {
			// 	desPose_.pose.position.x = markerPose_(0);
			// 	desPose_.pose.position.y = markerPose_(1);
			// 	desPose_.pose.position.z = ALTITUDE_CHANGE_METHOD;
			// }
			
		}
		else {
			get_marker_pose_with_distance();
		}
		
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
		markerPose_loop_.stop();
		check_loop_.stop();
	}
}

void transformAdaption::check_detect_timeout_loop()
{
	/**
	 * If can't detect marker during 2 second.
	 * Get detect again failed more than maxium,
	 * Landing will be called.
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

void transformAdaption::get_marker_pose_with_distance()
{
	/**
	 *      A *---------------* B
	 *        |   \           |
	 *        |       \       |
	 *        |           \   |
	 *      D *---------------* C (UAV)
	 */
	/* Check error angle */
	float var_AC;
	var_AC = (float)sqrt(pow(drone_Pose_[0],2) + pow(drone_Pose_[1],2));

	if(var_AC <= DISTANCE) {
		desPose_.pose.position.x = markerPose_(0);
		desPose_.pose.position.y = markerPose_(1);

		if (cur_pose_(2) >= 0.8) {
			desPose_.pose.position.z = 0.0;
		}
		else {
			accept_landing = true;
		}
	} else {
		desPose_.pose.position.x = markerPose_(0);
		desPose_.pose.position.y = markerPose_(1);
		desPose_.pose.position.z = cur_pose_(2);
		// ROS_INFO("Aligning under 5m........!");
	}
}