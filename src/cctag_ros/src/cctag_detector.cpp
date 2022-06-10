#include "cctag_ros/cctag_detector.h"

CCTagDetector::CCTagDetector(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private)
	: nh_(nh), nh_private_(nh_private), nCrowns_(3)
{
	nh_private_.param<double>("marker_size", marker_size_, 0.4);
	it_ = std::shared_ptr<image_transport::ImageTransport>(
			new image_transport::ImageTransport(nh));
	camera_image_subscriber_ = it_->subscribeCamera("image_rect", 1,
								&CCTagDetector::imageCallback, this,
								image_transport::TransportHints());
	pub_tf_cctag_ = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>
		("/tf_cctag", 1);
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

}

void CCTagDetector::detectTags(const cv_bridge::CvImagePtr& image,
				const sensor_msgs::CameraInfoConstPtr& camera_info)
{
	cctag::Parameters params(nCrowns_);
	CCTagMarkersBank bank(params._nCrowns);
	cv::Mat gray_image;
	image_geometry::PinholeCameraModel camera_model;
	camera_model.fromCameraInfo(camera_info);

	// Get camera intrinsic properties for rectified image.
	double fx = camera_model.fx(); // focal length in camera x-direction [px]
	double fy = camera_model.fy(); // focal length in camera y-direction [px]
	double cx = camera_model.cx(); // optical center x-coordinate [px]
	double cy = camera_model.cy(); // optical center y-coordinate [px]

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
	std::vector<cv::Point3d > objectPoints;
	std::vector<cv::Point2d > imagePoints;

	for(const cctag::CCTag& marker : markers) {
		// std::cout << marker.x() << " " << marker.y() << " " << marker.id() << " " << marker.getStatus() << endl;
		if(marker.getStatus() == status::id_reliable)
			imagePoints.push_back(cv::Point2d(marker.x(), marker.y()));
			++nMarkers;
	}
	if (imagePoints.size() == 4) {
		// std::cout << std::endl << nMarkers << " markers detected and identified" << std::endl;
		addObjectPoints(marker_size_/2, cv::Matx44d::eye(), objectPoints);
		Eigen::Matrix4d transform = getRelativeTransform(objectPoints, imagePoints, fx, fy, cx, cy);
		Eigen::Matrix3d rot = transform.block(0, 0, 3, 3);
		Eigen::Quaternion<double> rot_quaternion(rot);
		geometry_msgs::PoseWithCovarianceStamped tag_pose;
		tag_pose = makeTagPose(transform, rot_quaternion);
		pub_tf_cctag_.publish(tag_pose);
	}
}

Eigen::Matrix4d CCTagDetector::getRelativeTransform(
    std::vector<cv::Point3d > objectPoints,
    std::vector<cv::Point2d > imagePoints,
    double fx, double fy, double cx, double cy)
{
	// perform Perspective-n-Point camera pose estimation using the
	// above 3D-2D point correspondences
	cv::Mat rvec, tvec;
	cv::Matx33d cameraMatrix(fx,  0, cx,
							 0,  fy, cy,
							 0,   0,  1);
	cv::Vec4f distCoeffs(0,0,0,0); // distortion coefficients
	// TODO Perhaps something like SOLVEPNP_EPNP would be faster? Would
	// need to first check WHAT is a bottleneck in this code, and only
	// do this if PnP solution is the bottleneck.
	cv::solvePnP(objectPoints, imagePoints, cameraMatrix, distCoeffs, rvec, tvec);
	cv::Matx33d R;
	cv::Rodrigues(rvec, R);
	Eigen::Matrix3d wRo;
	wRo << R(0,0), R(0,1), R(0,2), R(1,0), R(1,1), R(1,2), R(2,0), R(2,1), R(2,2);
	Eigen::Matrix4d T; // homogeneous transformation matrix
	T.topLeftCorner(3, 3) = wRo;
	T.col(3).head(3) <<
		tvec.at<double>(0), tvec.at<double>(1), tvec.at<double>(2);
	T.row(3) << 0,0,0,1;
	return T;
}

void CCTagDetector::addObjectPoints (
    double s, cv::Matx44d T_oi, std::vector<cv::Point3d >& objectPoints)
{
  // Add to object point vector the tag corner coordinates in the bundle frame
  // Going counterclockwise starting from the bottom left corner
  objectPoints.push_back(T_oi.get_minor<3, 4>(0, 0)*cv::Vec4d(-s,-s, 0, 1));
  objectPoints.push_back(T_oi.get_minor<3, 4>(0, 0)*cv::Vec4d( s,-s, 0, 1));
  objectPoints.push_back(T_oi.get_minor<3, 4>(0, 0)*cv::Vec4d( s, s, 0, 1));
  objectPoints.push_back(T_oi.get_minor<3, 4>(0, 0)*cv::Vec4d(-s, s, 0, 1));
}

geometry_msgs::PoseWithCovarianceStamped CCTagDetector::makeTagPose(
	const Eigen::Matrix4d& transform,
	const Eigen::Quaternion<double> rot_quaternion)
{
  geometry_msgs::PoseWithCovarianceStamped pose;
  //===== Position and orientation
  pose.pose.pose.position.x    = transform(0, 3);
  pose.pose.pose.position.y    = transform(1, 3);
  pose.pose.pose.position.z    = transform(2, 3);
  pose.pose.pose.orientation.x = rot_quaternion.x();
  pose.pose.pose.orientation.y = rot_quaternion.y();
  pose.pose.pose.orientation.z = rot_quaternion.z();
  pose.pose.pose.orientation.w = rot_quaternion.w();
  return pose;
}
