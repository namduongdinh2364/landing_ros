#include "whycon_uav_ros.h"
#include <geometry_msgs/Pose.h>
#include <angles/angles.h>

whycon::RobotPosePublisher::RobotPosePublisher(ros::NodeHandle& n)
{
    n.param("axis_length_tolerance", axis_length_tolerance, 0.05);
    n.param("world_frame", world_frame, std::string("world"));
    n.param("target_frame", target_frame, std::string("target"));

    broadcaster = boost::make_shared<tf::TransformBroadcaster>();
    pub_target = n.advertise<geometry_msgs::PoseStamped>("/whycon_target_position", 10);
    mavposeSub_ = n.subscribe("mavros/local_position/pose", 1, &whycon::RobotPosePublisher::mavposeCallback, this,
                        ros::TransportHints().tcpNoDelay());

    pub_pose_marker_camera_frame = n.advertise<geometry_msgs::PoseStamped>("/whycon/camera/pose", 10);

    pose_sub = n.subscribe<geometry_msgs::PoseArray>("/whycon/poses", 1, &whycon::RobotPosePublisher::on_poses, this);
    sub_mavros_imu_data_ = n.subscribe
        ("/mavros/imu/data", 1, &whycon::RobotPosePublisher::imu_Callback, this, ros::TransportHints().tcpNoDelay());
    uav2cam_matrix_ << 0.0 , -1.0 , 0.0 , -1.0 , 0.0 , 0.0 , 0.0 , 0.0 , -1.0;
}


void whycon::RobotPosePublisher::mavposeCallback(const geometry_msgs::PoseStamped &msg) {
//   if (!received_home_pose) {
//     received_home_pose = true;
//     home_pose_ = msg.pose;
//     ROS_INFO_STREAM("Home pose initialized to: " << home_pose_);
//   }
//   mavPos_(0) = msg.pose.position.x;
//   mavPos_(1) = msg.pose.position.y;
//   mavPos_(2) = msg.pose.position.z;
//   mavAtt_(0) = msg.pose.orientation.w;
//   mavAtt_(1) = msg.pose.orientation.x;
//   mavAtt_(2) = msg.pose.orientation.y;
//   mavAtt_(3) = msg.pose.orientation.z;
//   quat = Eigen::Quaterniond(mavAtt_(0),mavAtt_(1),mavAtt_(2),mavAtt_(3));
//   neu2uav_matrix_ = quat.toRotationMatrix();
}

void whycon::RobotPosePublisher::imu_Callback(const sensor_msgs::Imu& msg)
{
    // double x, y, z, w;
    // Quaternionf quat;

    // x = msg.orientation.x;
    // y = msg.orientation.y;
    // z = msg.orientation.z;
    // w = msg.orientation.w;
    // cur_yaw_ = get_Euler_from_quat(x, y, z, w, Axis::YAW);
    // /* making a quaternion of position */
    // quat = Eigen::Quaternionf(w, x, y, z);
    // /* making rotation matrix from quaternion */
    // mavros_imu_data_ = quat.toRotationMatrix();
}


/* this assumes an L-shaped pattern, defining the two axis of the robot on the plane (forward and left are positive) */
void whycon::RobotPosePublisher::on_poses(const geometry_msgs::PoseArrayConstPtr& pose_array)
{
//   ROS_INFO_STREAM("receiving poses");
  tf::Transform T;
//   float dists[3];
  const std::vector<geometry_msgs::Pose>& ps = pose_array->poses;
  num_pose = pose_array->poses.size();
  geometry_msgs::PoseStamped target_pose;
  switch (num_pose)
  {
  case 1:
    {
        Eigen::Vector3f pos;
        pos << ps[0].position.x, ps[0].position.y, ps[0].position.z;
        /* UAV ----> Marker */
        // uav_Marker_pose = uav2cam_matrix_*pos;
        // std::cout << "x--: " << uav_Marker_pose(0) << std::endl;
        // std::cout << "y--: " << uav_Marker_pose(1) << std::endl;
        // std::cout << "z--: " << uav_Marker_pose(2)<< std::endl;
        /* World ----> Marker */
        // world_Marker_pose = neu2uav_matrix_*uav_Marker_pose;
        // world_Marker_pose = mavros_imu_data_*uav_Marker_pose;
        // std::cout << "x--world: " << world_Marker_pose(0) << std::endl;
        // std::cout << "y--world: " << world_Marker_pose(1)  << std::endl;
        // std::cout << "z--world: " << world_Marker_pose(2)  << std::endl;
        // target_pose.header.stamp = ros::Time::now();
        // target_pose.pose.position.x = world_Marker_pose(0) + mavPos_(0);
        // target_pose.pose.position.y = world_Marker_pose(1) + mavPos_(1);
        // target_pose.pose.position.z = world_Marker_pose(2) + mavPos_(2);
        // std::cout << "x--tag: " << target_pose.pose.position.x << std::endl;
        // std::cout << "y--tag: " << target_pose.pose.position.y << std::endl;
        // std::cout << "z--tag: " << target_pose.pose.position.z << std::endl;
        // pub_target.publish(target_pose);

        target_pose.header.stamp = ros::Time::now();
        target_pose.pose.position.x = pos(0);
        target_pose.pose.position.y = pos(1);
        target_pose.pose.position.z = pos(2);
        pub_pose_marker_camera_frame.publish(target_pose);
    }
    break;
  case 2:
    {
        // point 1
        Eigen::Vector3f pos, pos1;
        pos << ps[0].position.x, ps[0].position.y, ps[0].position.z;
        /* UAV ----> Marker */
        // uav_Marker_pose = uav2cam_matrix_*pos;
        // /* World ----> Marker */
        // world_Marker_pose = mavros_imu_data_*uav_Marker_pose;

        //point 2
        pos1 << ps[1].position.x, ps[1].position.y, ps[1].position.z;
        // uav_Marker_pose1 = uav2cam_matrix_*pos1;
        // world_Marker_pose1 = mavros_imu_data_*uav_Marker_pose1;

        // Eigen::Vector3f average_pose;
        // average_pose = (world_Marker_pose + world_Marker_pose1)/2;

        // target_pose.header.stamp = ros::Time::now();
        // target_pose.pose.position.x = average_pose(0) + mavPos_(0);
        // target_pose.pose.position.y = average_pose(1) + mavPos_(1);
        // target_pose.pose.position.z = average_pose(2) + mavPos_(2);
        // pub_target.publish(target_pose);

        target_pose.header.stamp = ros::Time::now();
        target_pose.pose.position.x = (pos(0) + pos1(0))/2;
        target_pose.pose.position.y = (pos(1) + pos1(1))/2;
        target_pose.pose.position.z = (pos(2) + pos1(2))/2;
        pub_pose_marker_camera_frame.publish(target_pose);
    }
    break;
  case 3:
    {
        // point 1
        Eigen::Vector3f pos, pos1, pos2;
        pos << ps[0].position.x, ps[0].position.y, ps[0].position.z;
        /* UAV ----> Marker */
        // uav_Marker_pose = uav2cam_matrix_*pos;
        // /* World ----> Marker */
        // world_Marker_pose = mavros_imu_data_*uav_Marker_pose;

        //point 2
        pos1 << ps[1].position.x, ps[1].position.y, ps[1].position.z;
        // uav_Marker_pose1 = uav2cam_matrix_*pos1;
        // world_Marker_pose1 = mavros_imu_data_*uav_Marker_pose1;

        //point 3
        pos2 << ps[2].position.x, ps[2].position.y, ps[2].position.z;
        // uav_Marker_pose2 = uav2cam_matrix_*pos2;
        // world_Marker_pose2 = mavros_imu_data_*uav_Marker_pose2;

        // compare point
        Eigen::Vector3f com1, com2, com3;
        Eigen::Vector3f average_pose;
        // com1 = world_Marker_pose - world_Marker_pose1;

        // com2 = world_Marker_pose - world_Marker_pose2;

        // com3 = world_Marker_pose1 - world_Marker_pose2;

        // target_pose.header.stamp = ros::Time::now();
        // if (com1.norm() > com2.norm() && com1.norm() > com3.norm())
        // {
        //     average_pose = (world_Marker_pose + world_Marker_pose1)/2;
        //     target_pose.pose.position.x = average_pose(0) + mavPos_(0);
        //     target_pose.pose.position.y = average_pose(1) + mavPos_(1);
        //     target_pose.pose.position.z = average_pose(2) + mavPos_(2);
        //     pub_target.publish(target_pose);
        // } 
        // else if(com2.norm()> com1.norm() && com2.norm()> com3.norm())
        // {
        //     average_pose = (world_Marker_pose + world_Marker_pose2)/2;
        //     target_pose.pose.position.x = average_pose(0) + mavPos_(0);
        //     target_pose.pose.position.y = average_pose(1) + mavPos_(1);
        //     target_pose.pose.position.z = average_pose(2) + mavPos_(2);
        //     pub_target.publish(target_pose);
        // }
        // else
        // {
        //     average_pose = (world_Marker_pose1 + world_Marker_pose2)/2;
        //     target_pose.pose.position.x = average_pose(0) + mavPos_(0);
        //     target_pose.pose.position.y = average_pose(1) + mavPos_(1);
        //     target_pose.pose.position.z = average_pose(2) + mavPos_(2);
        //     pub_target.publish(target_pose);
        // }

        com1 = pos - pos1;
        com2 = pos - pos2;
        com3 = pos1 - pos2;
        if (com1.norm() > com2.norm() && com1.norm() > com3.norm())
        {
            average_pose = (pos + pos1)/2;
            target_pose.pose.position.x = average_pose(0);
            target_pose.pose.position.y = average_pose(1);
            target_pose.pose.position.z = average_pose(2);
            pub_pose_marker_camera_frame.publish(target_pose);
        } 
        else if(com2.norm()> com1.norm() && com2.norm()> com3.norm())
        {
            average_pose = (pos + pos2)/2;
            target_pose.pose.position.x = average_pose(0);
            target_pose.pose.position.y = average_pose(1);
            target_pose.pose.position.z = average_pose(2);
            pub_pose_marker_camera_frame.publish(target_pose);
        }
        else
        {
            average_pose = (pos1 + pos2)/2;
            target_pose.pose.position.x = average_pose(0);
            target_pose.pose.position.y = average_pose(1);
            target_pose.pose.position.z = average_pose(2);
            pub_pose_marker_camera_frame.publish(target_pose);
        }
    }
    break;
  case 4:
    {
        // point 1
        Eigen::Vector3f pos, pos1, pos2, pos3;
        pos << ps[0].position.x, ps[0].position.y, ps[0].position.z;
        /* UAV ----> Marker */
        // uav_Marker_pose = uav2cam_matrix_*pos;
        // /* World ----> Marker */
        // world_Marker_pose = mavros_imu_data_*uav_Marker_pose;

        //point 2
        pos1 << ps[1].position.x, ps[1].position.y, ps[1].position.z;
        // uav_Marker_pose1 = uav2cam_matrix_*pos1;
        // world_Marker_pose1 = mavros_imu_data_*uav_Marker_pose1;

        //point 3
        pos2 << ps[2].position.x, ps[2].position.y, ps[2].position.z;
        // uav_Marker_pose2 = uav2cam_matrix_*pos2;
        // world_Marker_pose2 = mavros_imu_data_*uav_Marker_pose2;

        //point 4
        pos3 << ps[2].position.x, ps[2].position.y, ps[2].position.z;
        // uav_Marker_pose3 = uav2cam_matrix_*pos3;
        // world_Marker_pose3 = mavros_imu_data_*uav_Marker_pose3;
        // compare point
        Eigen::Vector3f com1, com2, com3;
        Eigen::Vector3f average_pose;
        // com1 = world_Marker_pose - world_Marker_pose1;

        // com2 = world_Marker_pose - world_Marker_pose2;

        // com3 = world_Marker_pose - world_Marker_pose3;

        // target_pose.header.stamp = ros::Time::now();
        // if (com1.norm() > com2.norm() && com1.norm() > com3.norm())
        // {
        //     average_pose = (world_Marker_pose + world_Marker_pose1)/2;
        //     target_pose.pose.position.x = average_pose(0) + mavPos_(0);
        //     target_pose.pose.position.y = average_pose(1) + mavPos_(1);
        //     target_pose.pose.position.z = average_pose(2) + mavPos_(2);
        //     pub_target.publish(target_pose);
        // } 
        // else if(com2.norm()> com1.norm() && com2.norm()> com3.norm())
        // {
        //     average_pose = (world_Marker_pose + world_Marker_pose2)/2;
        //     target_pose.pose.position.x = average_pose(0) + mavPos_(0);
        //     target_pose.pose.position.y = average_pose(1) + mavPos_(1);
        //     target_pose.pose.position.z = average_pose(2) + mavPos_(2);
        //     pub_target.publish(target_pose);
        // }
        // else 
        // {
        //     average_pose = (world_Marker_pose + world_Marker_pose3)/2;
        //     target_pose.pose.position.x = average_pose(0) + mavPos_(0);
        //     target_pose.pose.position.y = average_pose(1) + mavPos_(1);
        //     target_pose.pose.position.z = average_pose(2) + mavPos_(2);
        //     pub_target.publish(target_pose);
        // }
        com1 = pos - pos1;
        com2 = pos - pos2;
        com3 = pos - pos3;
        if (com1.norm() > com2.norm() && com1.norm() > com3.norm())
        {
            average_pose = (pos + pos1)/2;
            target_pose.pose.position.x = average_pose(0);
            target_pose.pose.position.y = average_pose(1);
            target_pose.pose.position.z = average_pose(2);
            pub_pose_marker_camera_frame.publish(target_pose);
        } 
        else if(com2.norm()> com1.norm() && com2.norm()> com3.norm())
        {
            average_pose = (pos + pos2)/2;
            target_pose.pose.position.x = average_pose(0);
            target_pose.pose.position.y = average_pose(1);
            target_pose.pose.position.z = average_pose(2);
            pub_pose_marker_camera_frame.publish(target_pose);
        }
        else 
        {
            average_pose = (pos + pos)/2;
            target_pose.pose.position.x = average_pose(0);
            target_pose.pose.position.y = average_pose(1);
            target_pose.pose.position.z = average_pose(2);
            pub_pose_marker_camera_frame.publish(target_pose);
        }

    }
    break;
  
    default:
        ROS_INFO("error");
        break;
  }
  
//   ROS_INFO_STREAM("poses: " << ps[0].position << " " << ps[1].position << " " << ps[2].position);

//   dists[0] = sqrt(pow(ps[0].position.x - ps[1].position.x, 2) + pow(ps[0].position.y - ps[1].position.y, 2)); // d(0,1)
//   dists[1] = sqrt(pow(ps[1].position.x - ps[2].position.x, 2) + pow(ps[1].position.y - ps[2].position.y, 2)); // d(1,2)
//   dists[2] = sqrt(pow(ps[2].position.x - ps[0].position.x, 2) + pow(ps[2].position.y - ps[0].position.y, 2)); // d(2,0)
//   ROS_INFO_STREAM("distances: " << dists[0] << " " << dists[1] << " " << dists[2]);
//   size_t max_idx = (std::max_element(dists, dists + 3) - dists);
//   std::swap(dists[max_idx], dists[0]); // dists[0] is now the longer distance (hypothenuse)


//   /* the two smaller distances correspond to the two cathetus, which should be equal in length (up to a certain tolerance) */
//   if (fabsf(dists[1] - dists[2]) > axis_length_tolerance) { ROS_WARN_STREAM("Axis size differ: " << dists[1] << " , " << dists[2]); return ; }
//   if (fabsf(dists[0] - sqrt(pow(dists[1],2) + pow(dists[2],2))) > axis_length_tolerance) { ROS_WARN_STREAM("Pythagoras check not passed"); return ; }

//   /* compute robot position as center of coordinate frame */
//   const geometry_msgs::Pose& center = ps[(max_idx + 2) % 3];
//   T.setOrigin(tf::Vector3(center.position.x, center.position.y, 0.0));

//   /* compute robot orientation as orientation of coordinate frame */
//   const geometry_msgs::Pose& p1 = ps[(max_idx + 0) % 3];
//   const geometry_msgs::Pose& p2 = ps[(max_idx + 1) % 3];

//   float v1[2], v2[2];
//   v1[0] = p1.position.x - center.position.x; v1[1] = p1.position.y - center.position.y;
//   v2[0] = p2.position.x - center.position.x; v2[1] = p2.position.y - center.position.y;

//   float n1 = sqrt(v1[0] * v1[0] + v1[1] * v1[1]);
//   float n2 = sqrt(v2[0] * v2[0] + v2[1] * v2[1]);

//   v1[0] /= n1; v1[1] /= n1;
//   v2[0] /= n2; v2[1] /= n2;

//   if (angles::normalize_angle(atan2(v1[1], v1[0]) - atan2(v2[1], v2[0])) > 0)
//     T.setRotation(tf::createQuaternionFromYaw(atan2(v2[1], v2[0])));
//   else
//     T.setRotation(tf::createQuaternionFromYaw(atan2(v1[1], v1[0])));

//   broadcaster->sendTransform(tf::StampedTransform(T, pose_array->header.stamp, world_frame, target_frame));
}

