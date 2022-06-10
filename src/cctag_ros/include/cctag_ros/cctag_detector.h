#ifndef CCTAG_DETECTOR_H
#define CCTAG_DETECTOR_H

#include <ros/ros.h>
#include <iostream>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/types_c.h>
#include <opencv2/opencv.hpp>
#include <opencv2/videoio.hpp>
#include "image_geometry/pinhole_camera_model.h"
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include "cctag/Detection.hpp"

#ifdef CCTAG_WITH_CUDA
#include "cctag/cuda/debug_macros.hpp"
#include "cctag/cuda/device_prop.hpp"
#endif // CCTAG_WITH_CUDA

using namespace std;
using namespace cctag;
using namespace Eigen;

class CCTagDetector
{
private:
	ros::NodeHandle nh_;
	ros::NodeHandle nh_private_;
	ros::Publisher pub_tf_cctag_;

	std::shared_ptr<image_transport::ImageTransport> it_;
	image_transport::CameraSubscriber camera_image_subscriber_;
	cv_bridge::CvImagePtr cv_image_;
	const std::size_t nCrowns_;
	std::size_t frameId_;
	double marker_size_;

public:

	CCTagDetector(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private);
	void imageCallback(const sensor_msgs::ImageConstPtr& image_rect,
						const sensor_msgs::CameraInfoConstPtr& camera_info);

	void detectTags(const cv_bridge::CvImagePtr& image,
					const sensor_msgs::CameraInfoConstPtr& camera_info);
	Eigen::Matrix4d getRelativeTransform(std::vector<cv::Point3d > objectPoints,
										std::vector<cv::Point2d > imagePoints,
										double fx, double fy, double cx, double cy);
	void addObjectPoints(double s, cv::Matx44d T_oi, std::vector<cv::Point3d >& objectPoints);
	geometry_msgs::PoseWithCovarianceStamped makeTagPose(const Eigen::Matrix4d& transform,
								const Eigen::Quaternion<double> rot_quaternion);
};

#endif /* CCTAG_DETECTOR_H */