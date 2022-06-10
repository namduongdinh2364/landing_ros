#ifndef GENERATE_CURVE_H
#define GENERATE_CURVE_H

#include <ros/ros.h>
#include <sstream>
#include <unistd.h>
#include <math.h>
#include "std_msgs/Float32.h"
#include "std_msgs/String.h"
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <eigen3/Eigen/Dense>
#include <sensor_msgs/Imu.h>
#include <tf2_msgs/TFMessage.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <std_srvs/SetBool.h>

using namespace std;
using namespace Eigen;

namespace generate_curve {
	void getIntersectionPoints(vector<Eigen::Vector3d> *points, const Eigen::Vector3f& marker_pos, const Eigen::Vector3f& uav_pos);
};

#endif