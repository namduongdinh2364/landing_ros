#ifndef MAV_TRAJECTORY_GENERATION_EXAMPLE_PLANNER_H
#define MAV_TRAJECTORY_GENERATION_EXAMPLE_PLANNER_H

#include <iostream>
#include <ros/ros.h>
#include "std_msgs/Float32.h"
#include <Eigen/Dense>
#include <nav_msgs/Odometry.h>
#include <eigen_conversions/eigen_msg.h>
#include <mav_trajectory_generation/polynomial_optimization_nonlinear.h>
#include <mav_trajectory_generation_ros/ros_visualization.h>
#include <mav_trajectory_generation_ros/ros_conversions.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <sensor_msgs/Imu.h>
#include <tf2_msgs/TFMessage.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <mav_planning_msgs/Curve.h>

class ExamplePlanner {
 public:
  ExamplePlanner(ros::NodeHandle& nh);

  void uavposeCallback(const geometry_msgs::PoseStamped &msg);
  void uavtwistCallback(const geometry_msgs::TwistStamped &msg);
  void markerposeCallback(const geometry_msgs::PoseStamped& msg);
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
  void generateTrajectoryCallback(const ros::TimerEvent& event);
  void imuCallback(const sensor_msgs::Imu& msg);
  void yawtargetCallback(const std_msgs::Float32& msg);
  void curvePointsCallback(const mav_planning_msgs::Curve& msg);
  void setlandCallback(const std_msgs::Float32& msg);
 private:
  ros::Publisher pub_markers_;
  ros::Publisher pub_trajectory_;
  ros::Subscriber sub_uavpose;
  ros::Subscriber sub_uavtwist;
  ros::Subscriber sub_markerpose;
  ros::Timer loop_genTrajectory_;
  ros::Subscriber sub_imu_data_;
  ros::Subscriber sub_goal_yaw_;
  ros::Subscriber sub_curvePoints_;
  ros::Subscriber sub_enableLand_;

  ros::NodeHandle& nh_;
  Eigen::Vector4d goal_position_, goal_velocity_;
  Eigen::Vector4d current_pose_;
  Eigen::Vector4d current_velocity_;
  Eigen::Vector3d current_angular_velocity_;
  std::vector<Eigen::Vector4d> curver_position_;
  double goal_yaw_;
  double cur_yaw_;
  double max_v_; // m/s
  double max_a_; // m/s^2
  double max_ang_v_;
  double max_ang_a_;
  bool marker_detected_;
  bool check_curve_;
  bool enable_landing_;
};

#endif // MAV_TRAJECTORY_GENERATION_EXAMPLE_PLANNER_H
