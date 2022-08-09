#include <mav_trajectory_generation_example/example_planner.h>

ExamplePlanner::ExamplePlanner(ros::NodeHandle& nh) :
    nh_(nh),
    max_v_(0.5),
    max_a_(0.35),
    decrease_height_(false),
    start_landing_(false),
    node_state(WAITING_FOR_MARKER_POSE),
    current_velocity_(Eigen::Vector3d::Zero()),
    current_pose_(Eigen::Vector3d::Zero()) {
      
  // Load params
  if (!nh_.getParam(ros::this_node::getName() + "/max_v", max_v_)){
    ROS_WARN("[example_planner] param max_v not found");
  }
  if (!nh_.getParam(ros::this_node::getName() + "/max_a", max_a_)){
    ROS_WARN("[example_planner] param max_a not found");
  }
  velocity << 0.0, 0.0, 0.0;

  // create publisher for RVIZ markers
  pub_markers_ =
      nh.advertise<visualization_msgs::MarkerArray>("trajectory_markers", 0);

  pub_trajectory_ =
      nh.advertise<mav_planning_msgs::PolynomialTrajectory4D>("trajectory",
                                                              0);

  // subscriber for Odometry
  // sub_odom_ =
  //     nh.subscribe("uav_pose", 1, &ExamplePlanner::uavOdomCallback, this);
  sub_uavpose =
		nh.subscribe("mavros/local_position/pose", 1, &ExamplePlanner::uavposeCallback, this, ros::TransportHints().tcpNoDelay());
	sub_uavtwist =
		nh.subscribe("mavros/local_position/velocity_local", 1, &ExamplePlanner::uavtwistCallback, this);
  markerposeSub_ = nh_.subscribe("/cmd/set_desposition/local", 1, &ExamplePlanner::markerposeCallback, this, ros::TransportHints().tcpNoDelay());
  decrese_height_ = nh_.subscribe("/decrease_height", 1,  &ExamplePlanner::decreaseheightCallback, this,ros::TransportHints().tcpNoDelay());

  cmdloop_timer_ = nh_.createTimer(ros::Duration(0.2), &ExamplePlanner::cmdloopCallback, this);
  land_service_ = nh_.advertiseService("land4", &ExamplePlanner::enableLandCallback, this);
}

void ExamplePlanner::markerposeCallback(const geometry_msgs::PoseStamped &msg){
  point_des << msg.pose.position.x, msg.pose.position.y, msg.pose.position.z;
  received_marker_pose =true;
  // std::cout << "point des" << point_des << std::endl;
};

void ExamplePlanner::decreaseheightCallback(const std_msgs::Bool &msg){
  decrease_height_ = msg.data;
  if (decrease_height_)
  {
    mission = DECREASE;
    // std::cout << "DECREASE" << std::endl;
  }
  else{
    mission = MOVE;
    // std::cout << "MOVE" << std::endl;
    
  }
  
  // std::cout << "update status height: " << decrease_height_ << std::endl;
}

void ExamplePlanner::cmdloopCallback(const ros::TimerEvent &event){
  // if(check){
  //   if (!decrease_height_) {
  //       accept_update = true;
  //       pointUpdate(2) = current_pose_(2);
  //   // std::cout << "range fail" << std::endl;
  //   }
  //   else{
  //     pointUpdate(2) = 0.0;
  //     planTrajectory(pointUpdate, velocity, &trajectory);
  //     publishTrajectory(trajectory);
  //   }
  //   if (accept_update) 
  //   { 
  //     pointUpdate(0) = point_des(0);
  //     pointUpdate(1) = point_des(1);
  //     accept_update = false;
  //     // std::cout << "Update point" << std::endl;
  //     // std::cout << pointUpdate << std::endl;
  //     planTrajectory(pointUpdate, velocity, &trajectory);
  //     publishTrajectory(trajectory);
  //   }
  // }
  switch (node_state)
  {
    case WAITING_FOR_MARKER_POSE:{
      waitForPredicate(&received_marker_pose, "Waiting for marker pose...");
      ROS_INFO("Got pose! Drone Ready to be landed.");
      node_state = MISSION_EXECUTION;
      accept_update = true;
      break;
    }
    case MISSION_EXECUTION:{
      int save_state;
      switch (mission)
      {
      case DECREASE:
        { 
          pointUpdate(0) = point_des(0);
          pointUpdate(1) = point_des(1);
          pointUpdate(2) = 0.0;
          if (save_state!=DECREASE){
              accept_update = true;
              
          }
          if (accept_update)
          {
            planTrajectory(pointUpdate, velocity, &trajectory);
            publishTrajectory(trajectory);
            accept_update = false;
          }
          save_state = mission;
          if(current_pose_(2) < 0.1) {
            node_state = LANDED;
            std::cout << "check0" << std::endl;
          }
          break;
        }
      case MOVE:
        { 
          pointUpdate(0) = point_des(0);
          pointUpdate(1) = point_des(1);
          pointUpdate(2) = current_pose_(2);
          if (accept_update)
          { 
            save_point(0) = pointUpdate(0);
            save_point(1) = pointUpdate(1);
            save_point(2) = pointUpdate(2);
            distance_ = save_point.norm() - current_pose_.norm();
            planTrajectory(pointUpdate, velocity, &trajectory);
            publishTrajectory(trajectory);
            accept_update = false;
          }
          else{
            if ((!decrease_height_ && (save_point.norm() - current_pose_.norm() <= (distance_/4)))|| (save_state!=MOVE)){
              accept_update = true;
              // std::cout << "current" << current_pose_.norm() << std::endl;
              // std::cout << "save" << save_point.norm()<< std::endl;
              // std::cout << "update" << save_point.norm()<< std::endl;

            }
          }
          save_state = mission;
          if(current_pose_(2) < 0.1) {
            node_state = LANDED;
            std::cout << "check1" << std::endl;
          }
          // std::cout << save_state << std::endl;
          break;
        }
      }
    }
    case LANDED:{
      // std_srvs::SetBool land_cmd;
	    // land_cmd.request.data = true;
	    // set_mode_client_.call(land_cmd);
      // std::cout << "land" << std::endl;
      // offb_set_mode_.request.custom_mode = "AUTO.LAND";
      // if( set_mode_client_.call(offb_set_mode_) && offb_set_mode_.response.mode_sent)
      // {
      //     ROS_INFO("[ INFO] --------------- LAND ---------------\n");
      // }
      // node_state = LANDED;
      // ros::spinOnce();
    }
  }
};

void ExamplePlanner::uavposeCallback(const geometry_msgs::PoseStamped &msg) {
	// store current position in our planner
	current_pose_ << msg.pose.position.x, msg.pose.position.y, msg.pose.position.z;
}

void ExamplePlanner::uavtwistCallback(const geometry_msgs::TwistStamped &msg) {
	current_velocity_ << msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z;
}

// Callback to get current Pose of UAV
// void ExamplePlanner::uavOdomCallback(const nav_msgs::Odometry::ConstPtr& odom) {

//   // store current position in our planner
//   tf::poseMsgToEigen(odom->pose.pose, current_pose_);

//   // store current vleocity
//   tf::vectorMsgToEigen(odom->twist.twist.linear, current_velocity_);
// }

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
  const int dimension = 3;

  // Array for all waypoints and their constrains
  mav_trajectory_generation::Vertex::Vector vertices;

  // Optimze up to 4th order derivative (SNAP)
  const int derivative_to_optimize =
      mav_trajectory_generation::derivative_order::VELOCITY;

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

bool ExamplePlanner::publishTrajectory(const mav_trajectory_generation::Trajectory& trajectory){
  // send trajectory as markers to display them in RVIZ
  visualization_msgs::MarkerArray markers;
  double distance =
      0.2; // Distance by which to seperate additional markers. Set 0.0 to disable.
  std::string frame_id = "map";

  mav_trajectory_generation::drawMavTrajectory(trajectory,
                                               distance,
                                               frame_id,
                                               &markers);
  pub_markers_.publish(markers);

  // send trajectory to be executed on UAV
  mav_planning_msgs::PolynomialTrajectory msg;
  mav_trajectory_generation::trajectoryToPolynomialTrajectoryMsg(trajectory,
                                                                 &msg);
  msg.header.frame_id = "map";

  if (start_landing_) {
    pub_trajectory_.publish(msg);
  }

  return true;
}

bool ExamplePlanner::enableLandCallback(std_srvs::SetBool::Request &request, std_srvs::SetBool::Response &response) {
  start_landing_ = true;
  return true;
}
