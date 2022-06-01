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

#include "cctag/Detection.hpp"

#ifdef CCTAG_WITH_CUDA
#include "cctag/cuda/debug_macros.hpp"
#include "cctag/cuda/device_prop.hpp"
#endif // CCTAG_WITH_CUDA

using namespace std;
using namespace cctag;

class CCTagDetector
{
private:
	std::shared_ptr<image_transport::ImageTransport> it_;
	image_transport::CameraSubscriber camera_image_subscriber_;
	cv_bridge::CvImagePtr cv_image_;
	const std::size_t nCrowns_;
	std::size_t frameId_;


public:

	CCTagDetector(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private);
	void imageCallback(const sensor_msgs::ImageConstPtr& image_rect,
						const sensor_msgs::CameraInfoConstPtr& camera_info);
	void detectTags(const cv_bridge::CvImagePtr& image,
					const sensor_msgs::CameraInfoConstPtr& camera_info);

};

#endif /* CCTAG_DETECTOR_H */