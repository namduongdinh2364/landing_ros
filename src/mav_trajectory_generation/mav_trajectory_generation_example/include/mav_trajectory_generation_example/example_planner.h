#ifndef MAV_TRAJECTORY_GENERATION_EXAMPLE_PLANNER_H
#define MAV_TRAJECTORY_GENERATION_EXAMPLE_PLANNER_H

#include <iostream>
#include <ros/ros.h>
#include <Eigen/Dense>
#include <nav_msgs/Odometry.h>
#include <eigen_conversions/eigen_msg.h>
#include <mav_trajectory_generation/polynomial_optimization_nonlinear.h>
#include <mav_trajectory_generation_ros/ros_visualization.h>
#include <mav_trajectory_generation_ros/ros_conversions.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Twist.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <std_srvs/SetBool.h>


class ExamplePlanner {
 public:
  ExamplePlanner(ros::NodeHandle& nh);

  void uavOdomCallback(const nav_msgs::Odometry::ConstPtr& pose);

  void setMaxSpeed(double max_v);

  // Plans a trajectory to take off from the current position and
  // fly to the given altitude (while maintaining x,y, and yaw).
  bool planTrajectory(const Eigen::VectorXd& goal_pos,
                      const Eigen::VectorXd& goal_vel,
                      mav_trajectory_generation::Trajectory* trajectory);
                      
  bool planTrajectory(const Eigen::VectorXd& goal_pos,
                      const Eigen::VectorXd& goal_vel,
                      const Eigen::VectorXd& start_pos,
                      const Eigen::VectorXd& start_vel,
                      double v_max, double a_max,
                      mav_trajectory_generation::Trajectory* trajectory);
                      
  bool publishTrajectory(const mav_trajectory_generation::Trajectory& trajectory);
  void markerposeCallback(const geometry_msgs::PoseStamped &msg);
  void decreaseheightCallback(const std_msgs::Bool &msg);
  void cmdloopCallback(const ros::TimerEvent &event);
  void uavposeCallback(const geometry_msgs::PoseStamped &msg);
  void uavtwistCallback(const geometry_msgs::TwistStamped &msg);

 private:
  ros::Publisher pub_markers_;
  ros::Publisher pub_trajectory_;
  ros::Subscriber sub_odom_;
  ros::Subscriber sub_uavpose;
  ros::Subscriber sub_uavtwist;
  ros::Subscriber markerposeSub_;
  ros::Subscriber decrese_height_;
  ros::Timer cmdloop_timer_;
  ros::ServiceClient set_mode_client_;

  ros::NodeHandle& nh_;
  Eigen::Vector3d current_pose_;
  Eigen::Vector3d current_velocity_;
  Eigen::Vector3d current_angular_velocity_;
  double max_v_; // m/s
  double max_a_; // m/s^2
  double max_ang_v_;
  double max_ang_a_;
  double distance_;
  Eigen::Vector3d point_des;
  bool decrease_height_, accept_update, received_marker_pose;
  bool check = false;
  Eigen::Vector3d pointUpdate;
  Eigen::Vector3d save_point;
  mav_trajectory_generation::Trajectory trajectory;
  Eigen::Vector3d position, velocity;
  enum FlightState { WAITING_FOR_MARKER_POSE, MISSION_EXECUTION, LANDED} node_state;
  enum MissionState { DECREASE, MOVE} mission;
  template <class T>
  void waitForPredicate(const T *pred, const std::string &msg, double hz = 2.0) {
    ros::Rate pause(hz);
    ROS_INFO_STREAM(msg);
    while (ros::ok() && !(*pred)) {
      ros::spinOnce();
      pause.sleep();
    }
  };
  mavros_msgs::State current_state_;
  mavros_msgs::SetMode offb_set_mode_, land_set_mode_;

};

#endif // MAV_TRAJECTORY_GENERATION_EXAMPLE_PLANNER_H
