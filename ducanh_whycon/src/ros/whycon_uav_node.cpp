#include <ros/ros.h>
#include "whycon_uav_ros.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "uav_pose_publisher");
  ros::NodeHandle n("~");

  whycon::RobotPosePublisher robot_pose_publisher(n);
  ros::spin();
}

