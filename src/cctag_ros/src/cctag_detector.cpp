#include "cctag_ros/cctag_detector.h"

CCTagDetector::CCTagDetector(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private)
	: nCrowns_(3)
{
	it_ = std::shared_ptr<image_transport::ImageTransport>(
			new image_transport::ImageTransport(nh));
	camera_image_subscriber_ = it_->subscribeCamera("image_rect", 1,
								&CCTagDetector::imageCallback, this,
								image_transport::TransportHints());

	frameId_ = 0;
}

void CCTagDetector::imageCallback (
    const sensor_msgs::ImageConstPtr& image_rect,
    const sensor_msgs::CameraInfoConstPtr& camera_info)
{

	/* Convert ROS's sensor_msgs::Image to cv_bridge::CvImagePtr
	 * in order to run on the iamge.
	 */
	try {
		cv_image_ = cv_bridge::toCvCopy(image_rect, image_rect->encoding);
	}
	catch (cv_bridge::Exception& e) {
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return;
	}

	detectTags(cv_image_,camera_info);

//   // Publish the camera image overlaid by outlines of the detected tags and
//   // their payload values
//   if (draw_tag_detections_image_)
//   {
//     tag_detector_->drawDetections(cv_image_);
//     tag_detections_image_publisher_.publish(cv_image_->toImageMsg());
//   }
}

void CCTagDetector::detectTags(const cv_bridge::CvImagePtr& image,
				const sensor_msgs::CameraInfoConstPtr& camera_info)
{
	cctag::Parameters params(nCrowns_);
	CCTagMarkersBank bank(params._nCrowns);
	cv::Mat gray_image;
	if (image->image.channels() == 1) {
		gray_image = image->image;
	}
	else {
		cv::cvtColor(image->image, gray_image, CV_BGR2GRAY);
	}
	boost::ptr_list<CCTag> markers;
	const int pipeId = 0;

	static cctag::logtime::Mgmt* durations = nullptr;
	cctagDetection(markers, pipeId, frameId_, gray_image, params, bank, true, durations);
	++frameId_;
	std::size_t nMarkers = 0;
	for(const cctag::CCTag& marker : markers) {
		std::cout << marker.x() << " " << marker.y() << " " << marker.id() << " " << marker.getStatus() << endl;
		if(marker.getStatus() == status::id_reliable)
			++nMarkers;
	}
	std::cout << std::endl << nMarkers << " markers detected and identified" << std::endl;

}
