#include <camera_info_manager/camera_info_manager.h>
#include <fstream>
#include <sensor_msgs/CameraInfo.h>
#include <tf/tf.h>
#include <sstream>
#include <geometry_msgs/PoseArray.h>
#include <yaml-cpp/yaml.h>
#include <whycon/Projection.h>
#include "whycon_ros.h"

whycon::WhyConROS::WhyConROS(ros::NodeHandle& n) : is_tracking(false), should_reset(true), it(n)
{
	similarity.setIdentity();

	if (!n.getParam("targets", targets)) throw std::runtime_error("Private parameter \"targets\" is missing");

	n.param("name", frame_id, std::string("whycon"));
	n.param("world_frame", world_frame_id, std::string("world"));
	n.param("max_attempts", max_attempts, 1);
	n.param("max_refine", max_refine, 1);

	n.getParam("outer_diameter", parameters.outer_diameter);
	n.getParam("inner_diameter", parameters.inner_diameter);
	n.getParam("center_distance_tolerance_abs", parameters.center_distance_tolerance_abs);
	n.getParam("center_distance_tolerance_ratio", parameters.center_distance_tolerance_ratio);
	n.getParam("roundness_tolerance", parameters.roundness_tolerance);
	n.getParam("circularity_tolerance", parameters.circularity_tolerance);
	n.getParam("max_size", parameters.max_size);
	n.getParam("min_size", parameters.min_size);
	n.getParam("ratio_tolerance", parameters.ratio_tolerance);
	n.getParam("max_eccentricity", parameters.max_eccentricity);

	/* initialize ros */
	int input_queue_size = 1;
	n.param("input_queue_size", input_queue_size, input_queue_size);
	cam_sub = it.subscribeCamera("/camera/color/image_raw", input_queue_size, boost::bind(&WhyConROS::on_image, this, _1, _2));
	// ArucoImage_sub = it.subscribe("/aruco_bundle/result", 1, &WhyConROS::ArucoImage_callback, this);

	image_pub = n.advertise<sensor_msgs::Image>("image_out", 1);
	poses_pub = n.advertise<geometry_msgs::PoseArray>("pose", 1);
	context_pub = n.advertise<sensor_msgs::Image>("context", 1);
	// projection_pub = n.advertise<whycon::Projection>("projection", 1);

	reset_service = n.advertiseService("reset", &WhyConROS::reset, this);
}

// void whycon::WhyConROS::ArucoImage_callback(const sensor_msgs::ImageConstPtr& msg)
// {
// 	cv_bridge::CvImagePtr CVpArucoImage;
// 	CVpArucoImage = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::RGB8);
// 	MDrawImage = CVpArucoImage->image;
// }

void whycon::WhyConROS::on_image(const sensor_msgs::ImageConstPtr& image_msg, const sensor_msgs::CameraInfoConstPtr& info_msg)
{
	camera_model.fromCameraInfo(info_msg);
	if (camera_model.fullResolution().width == 0) { ROS_ERROR_STREAM("camera is not calibrated!"); return; }

	cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(image_msg, "rgb8");
	const cv::Mat& image = cv_ptr->image;

	if(!system)
	{
		cv::Mat dis_coeff_temp = cv::Mat(camera_model.distortionCoeffs());
		if (dis_coeff_temp.empty())
		{
			dis_coeff_temp = cv::Mat::zeros(1, 5, CV_32F);
		}
		system = boost::make_shared<whycon::LocalizationSystem>(targets, image.size().width, image.size().height, cv::Mat(camera_model.fullIntrinsicMatrix()), dis_coeff_temp, parameters);
	}
	is_tracking = system->localize(image, should_reset, max_attempts, max_refine);

	if(is_tracking) {
		publish_results(image_msg->header, cv_ptr);
		should_reset = false;
	}
	else if(image_pub.getNumSubscribers() != 0)
		image_pub.publish(cv_ptr);

	if(context_pub.getNumSubscribers() != 0) {
		cv_bridge::CvImage cv_img_context;
		cv_img_context.encoding = cv_ptr->encoding;
		cv_img_context.header.stamp = cv_ptr->header.stamp;
		system->detector.context.debug_buffer(cv_ptr->image, cv_img_context.image);
		context_pub.publish(cv_img_context.toImageMsg());
	}
}

bool whycon::WhyConROS::reset(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
	should_reset = true;
	return true;
}

void whycon::WhyConROS::publish_results(const std_msgs::Header& header, const cv_bridge::CvImageConstPtr& cv_ptr)
{
	bool publish_images = (image_pub.getNumSubscribers() != 0);
	bool publish_poses = (poses_pub.getNumSubscribers() != 0);
	
	if (!publish_images && !publish_poses) return;
	
	// prepare image outpu
	cv::Mat output_image;
	if (publish_images)
		output_image = cv_ptr->image.clone();

	geometry_msgs::PoseArray pose_array;
	
	// go through detected targets
	for(int i = 0; i < system->targets; i++) {
		const whycon::CircleDetector::Circle& circle = system->get_circle(i);
		whycon::LocalizationSystem::Pose pose = system->get_pose(circle);
		cv::Vec3f coord = pose.pos;

		// draw each target
		if (publish_images) {
			std::ostringstream ostr;
			ostr << std::fixed << std::setprecision(2);
			ostr << coord << " " << i;
			circle.draw(output_image, ostr.str(), cv::Vec3b(0,255,255));
			/*whycon::CircleDetector::Circle new_circle = circle.improveEllipse(cv_ptr->image);
			new_circle.draw(output_image, ostr.str(), cv::Vec3b(0,255,0));*/

            cv::putText(output_image, //target image
                        "Whycon is detected", //text
                        cv::Point(30, 20), //top-left position
                        cv::FONT_HERSHEY_DUPLEX,
                        1.0,
                        CV_RGB(100, 255, 0), //font color
                        2);

			cv::circle(output_image, camera_model.project3dToPixel(cv::Point3d(coord)), 1, cv::Scalar(255,0,255), 1, cv::LINE_AA);
		}

		if(publish_poses) {
			geometry_msgs::Pose p;
			p.position.x = pose.pos(0);
			p.position.y = pose.pos(1);
			p.position.z = pose.pos(2);
			p.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, pose.rot(0), pose.rot(1));
			pose_array.poses.push_back(p);
		}
	}

	if(publish_images) {
		cv_bridge::CvImage output_image_bridge = *cv_ptr;
		output_image_bridge.image = output_image;
		image_pub.publish(output_image_bridge);
	}

	if(publish_poses) {
		pose_array.header = header;
		pose_array.header.frame_id = frame_id;
		poses_pub.publish(pose_array);
	}
}
