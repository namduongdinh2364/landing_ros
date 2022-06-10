#include <mav_trajectory_generation_example/example_planner.h>

ExamplePlanner::ExamplePlanner(ros::NodeHandle& nh) :
    nh_(nh),
    max_v_(2.0),
    max_a_(2.0),
    marker_detected_(false),
    check_curve_(false),
	enable_landing_(false),
    current_velocity_(Eigen::Vector4d::Zero()){
      
  // Load params
	if(!nh_.getParam(ros::this_node::getName() + "/max_v", max_v_)) {
		ROS_WARN("[example_planner] param max_v not found");
	}
	if(!nh_.getParam(ros::this_node::getName() + "/max_a", max_a_)) {
		ROS_WARN("[example_planner] param max_a not found");
	}
	goal_velocity_ << 0.01, 0.01, 0.0, 0.0;
	// create publisher for RVIZ markers
	pub_markers_ =
		nh.advertise<visualization_msgs::MarkerArray>("trajectory_markers", 0);
	pub_trajectory_ =
		nh.advertise<mav_planning_msgs::PolynomialTrajectory4D>("trajectory", 0);

	sub_goal_yaw_ =
		nh.subscribe("cmd/reference/yaw", 1, &ExamplePlanner::yawtargetCallback, this);
	sub_uavpose =
		nh.subscribe("mavros/local_position/pose", 1, &ExamplePlanner::uavposeCallback, this, ros::TransportHints().tcpNoDelay());
	sub_uavtwist =
		nh.subscribe("mavros/local_position/velocity_local", 1, &ExamplePlanner::uavtwistCallback, this);
	sub_markerpose =
		nh.subscribe("cmd/set_desposition/local", 1, &ExamplePlanner::markerposeCallback, this);
	sub_imu_data_ = 
		nh.subscribe("/mavros/imu/data", 1, &ExamplePlanner::imuCallback, this, ros::TransportHints().tcpNoDelay());
	sub_enableLand_ =
		nh.subscribe("cmd/set_enable/land", 1, &ExamplePlanner::setlandCallback, this);
	sub_curvePoints_ = 
		nh.subscribe("cmd/reference/curvepoints", 1, &ExamplePlanner::curvePointsCallback, this);
	loop_genTrajectory_ =
		nh.createTimer(ros::Duration(5.0), &ExamplePlanner::generateTrajectoryCallback, this);
}

// Callback to get current Pose of UAV
void ExamplePlanner::uavposeCallback(const geometry_msgs::PoseStamped &msg) {
	// store current position in our planner
	current_pose_ << msg.pose.position.x, msg.pose.position.y, msg.pose.position.z, cur_yaw_;
}

void ExamplePlanner::uavtwistCallback(const geometry_msgs::TwistStamped &msg) {
	current_velocity_ << msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z, 0.0;
}

// Method to set maximum speed.
void ExamplePlanner::setMaxSpeed(const double max_v) {
	max_v_ = max_v;
}

// Plans a trajectory from the current position to the a goal position and velocity
// we neglect attitude here for simplicity
bool ExamplePlanner::planTrajectory(const Eigen::VectorXd& goal_pos,
                                    const Eigen::VectorXd& goal_vel,
                                    mav_trajectory_generation::Trajectory* trajectory) {


	// 3 Dimensional trajectory => through carteisan space, no orientation
	const int dimension = 4;

	// Array for all waypoints and their constrains
	mav_trajectory_generation::Vertex::Vector vertices;

	// Optimze up to 4th order derivative (SNAP)
	const int derivative_to_optimize =
		mav_trajectory_generation::derivative_order::SNAP;

	// we have 2 vertices:
	// Start = current position
	// end = desired position and velocity
	mav_trajectory_generation::Vertex start(dimension), end(dimension);

	/******* Configure start point *******/
	// set start point constraints to current position and set all derivatives to zero
	start.makeStartOrEnd(current_pose_,
						derivative_to_optimize);

	// set start point's velocity to be constrained to current velocity
	start.addConstraint(mav_trajectory_generation::derivative_order::VELOCITY,
						current_velocity_);

	// add waypoint to list
	vertices.push_back(start);
	for (size_t i = 0; i < curver_position_.size() && check_curve_; i++)
	{
		mav_trajectory_generation::Vertex middle(dimension);
		middle.addConstraint(mav_trajectory_generation::derivative_order::POSITION, curver_position_.at(i));
		vertices.push_back(middle);
	}
	check_curve_ = false;
	curver_position_.clear();

	/******* Configure end point *******/
	// set end point constraints to desired position and set all derivatives to zero
	end.makeStartOrEnd(goal_pos,
						derivative_to_optimize);

	// set start point's velocity to be constrained to current velocity
	end.addConstraint(mav_trajectory_generation::derivative_order::VELOCITY,
					goal_vel);

	// add waypoint to list
	vertices.push_back(end);

	// setimate initial segment times
	std::vector<double> segment_times;
	segment_times = estimateSegmentTimes(vertices, max_v_, max_a_);

	// Set up polynomial solver with default params
	mav_trajectory_generation::NonlinearOptimizationParameters parameters;

	// set up optimization problem
	const int N = 10;
	mav_trajectory_generation::PolynomialOptimizationNonLinear<N> opt(dimension, parameters);
	opt.setupFromVertices(vertices, segment_times, derivative_to_optimize);

	// constrain velocity and acceleration
	opt.addMaximumMagnitudeConstraint(mav_trajectory_generation::derivative_order::VELOCITY, max_v_);
	opt.addMaximumMagnitudeConstraint(mav_trajectory_generation::derivative_order::ACCELERATION, max_a_);

	// solve trajectory
	opt.optimize();

	// get trajectory as polynomial parameters
	opt.getTrajectory(&(*trajectory));

	return true;
}

bool ExamplePlanner::publishTrajectory(const mav_trajectory_generation::Trajectory& trajectory) {
	// send trajectory as markers to display them in RVIZ
	visualization_msgs::MarkerArray markers;
	double distance = 0.2; // Distance by which to seperate additional markers. Set 0.0 to disable.
	std::string frame_id = "map";

	mav_trajectory_generation::drawMavTrajectory(trajectory,
													distance,
													frame_id,
													&markers);
	pub_markers_.publish(markers);

	// send trajectory to be executed on UAV
	mav_planning_msgs::PolynomialTrajectory4D msg;
	mav_trajectory_generation::trajectoryToPolynomialTrajectoryMsg(trajectory, &msg);
	msg.header.frame_id = "map";
	if (enable_landing_) {
		pub_trajectory_.publish(msg);
	}

	return true;
}

void ExamplePlanner::generateTrajectoryCallback(const ros::TimerEvent& event) {
	if(marker_detected_) {
		mav_trajectory_generation::Trajectory trajectory;
		planTrajectory(goal_position_, goal_velocity_, &trajectory);
		publishTrajectory(trajectory);
		marker_detected_ = false;
	}
}

void ExamplePlanner::markerposeCallback(const geometry_msgs::PoseStamped &msg) {
	goal_position_ << msg.pose.position.x, msg.pose.position.y, msg.pose.position.z, goal_yaw_;
	marker_detected_ = true;
}

void ExamplePlanner::imuCallback(const sensor_msgs::Imu& msg)
{
	tf2::Quaternion q;
	double x, y, z, w;
	double roll, pitch, yaw;

	x = msg.orientation.x;
	y = msg.orientation.y;
	z = msg.orientation.z;
	w = msg.orientation.w;

	q.setValue(x, y, z, w);
	tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
	cur_yaw_ = yaw;
}

void ExamplePlanner::yawtargetCallback(const std_msgs::Float32& msg)
{
	goal_yaw_ = double(msg.data);
}

void ExamplePlanner::curvePointsCallback(const mav_planning_msgs::Curve& msg)
{
	// std::cout << "number vertex: "<< msg.num_vertix << std::endl;
	if (msg.num_vertix > 1) {
		curver_position_.resize(msg.num_vertix-1);
		for (size_t i = 0; i < msg.num_vertix -1; i++) {
			curver_position_[i] << msg.points[i].x, msg.points[i].y, msg.points[i].z, goal_yaw_;
		}
		check_curve_ = true;
	}
	
	// std::cout << curver_position_.at(0) << std::endl;
	// std::cout << curver_position_.at(1) << std::endl;
	// std::cout << curver_position_.at(2) << std::endl;
	// std::cout << curver_position_.at(3) << std::endl;
	goal_position_ << msg.points[msg.num_vertix -1].x, msg.points[msg.num_vertix -1].y, msg.points[msg.num_vertix -1].z, goal_yaw_;
	marker_detected_ = true;
}

void ExamplePlanner::setlandCallback(const std_msgs::Float32& msg)
{
	if (msg.data != 0.0) {
		enable_landing_ = true;
	}
	
}