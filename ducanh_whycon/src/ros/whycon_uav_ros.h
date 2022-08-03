#ifndef NEW_WHYCON_H
#define NEW_WHYCON_H

#include <ros/ros.h>
#include <geometry_msgs/PoseArray.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/PoseStamped.h>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <eigen3/Eigen/Dense>
#include <sensor_msgs/Imu.h>

using namespace Eigen;

namespace whycon {
  class RobotPosePublisher {
    public:
        RobotPosePublisher(ros::NodeHandle& n);
        ros::Publisher pub_target;
        ros::Subscriber sub_mavros_imu_data_;
        ros::Subscriber pose_sub, mavposeSub_;
        ros::Publisher pub_pose_marker_camera_frame;
        boost::shared_ptr<tf::TransformBroadcaster> broadcaster;

      double axis_length_tolerance;
      std::string world_frame, target_frame, axis_file;
      void on_poses(const geometry_msgs::PoseArrayConstPtr& pose_array);
      void mavposeCallback(const geometry_msgs::PoseStamped &msg);
        void imu_Callback(const sensor_msgs::Imu& msg);
    private:
        int num_pose;
        bool received_home_pose = false;
        geometry_msgs::Pose home_pose_;
        Eigen::Vector3f mavPos_, uav_Marker_pose, uav_Marker_pose1, world_Marker_pose, world_Marker_pose1, world_Marker_pose2, world_Marker_pose3, uav_Marker_pose2,uav_Marker_pose3;
        Eigen::Vector4d mavAtt_;
        // Eigen::Vector3f uav_Marker_pose;
        Eigen::Matrix3f uav2cam_matrix_, neu2uav_matrix_;
        Eigen::Quaternionf quat;

        Eigen::Matrix3f mavros_imu_data_;
  };
}

#endif // NEW_WHYCON_H