#include <iostream>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf2_msgs/TFMessage.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/highgui.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;
using namespace sensor_msgs;

class DebugImage
{
private:
	ros::NodeHandle nh_;
	ros::Subscriber cam_info_sub;
	ros::Subscriber PoseAruco_sub;
	ros::Subscriber PoseAprilTag_sub;
	ros::Subscriber PoseWhycon_sub;
	geometry_msgs::PoseStamped apriltagPose_, whyconPose_, arucoPose_;

	image_transport::Publisher image_pub;
	image_transport::Subscriber image_sub;
	image_transport::ImageTransport it;
	cv::Mat inImage;
	cv::Mat cameraMatrix;
	cv::Mat distCoeffs;

	bool detected_apriltag, detected_whycon, detected_aruco;

public:
	
	DebugImage(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private)
	: nh_(nh), it(nh_private)
	{
		image_sub = it.subscribe("/camera/color/image_raw", 1, &DebugImage::image_callback, this);
		cam_info_sub = nh_.subscribe("/camera/color/camera_info", 1, &DebugImage::cam_info_callback, this);
		PoseAruco_sub = nh_.subscribe("/aruco_bundle/pose", 1, &DebugImage::getPoseArucoCallback,
											this, ros::TransportHints().tcpNoDelay());
		PoseAprilTag_sub = nh_.subscribe("/tf", 1, &DebugImage::getPoseApriltagCallback,
											this, ros::TransportHints().tcpNoDelay());
		PoseWhycon_sub = nh_.subscribe("/whycon/camera/pose", 1, &DebugImage::getPoseWhyconCallback,
											this, ros::TransportHints().tcpNoDelay());

		image_pub = it.advertise("/result", 1);

		cameraMatrix = Mat(3, 3, CV_64FC1);
		distCoeffs = Mat(1, 5, CV_64FC1);
		int i;
		for (i = 0; i < 5; i++)
		{
			distCoeffs.at<float>(0,i) = 0.0;
		}
	}

	void image_callback(const sensor_msgs::ImageConstPtr& msg)
	{
		if (image_pub.getNumSubscribers() == 0)
		{
			ROS_DEBUG("No subscribers");
			return;
		}

		ros::Time curr_stamp = msg->header.stamp;
		cv_bridge::CvImagePtr cv_ptr;
		std::vector<cv::Vec3d> rvecs, tvecs;

		if (detected_aruco) {
			// detected_aruco = false;
			cv::Vec3d tvec, rvec;
			double roll, pitch, yaw;
			tf2::Quaternion q;

			tvec[0] = arucoPose_.pose.position.x;
			tvec[1] = arucoPose_.pose.position.y;
			tvec[2] = arucoPose_.pose.position.z;
			tvecs.push_back(tvec);

			// q.setValue(arucoPose_.pose.orientation.x,
			// 			arucoPose_.pose.orientation.y,
			// 			arucoPose_.pose.orientation.z,
			// 			arucoPose_.pose.orientation.w);

			// tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
			// rvec[0] = roll;
			// rvec[1] = pitch;
			// rvec[2] = yaw;

			rvec[0] = arucoPose_.pose.orientation.x;
			rvec[1] = arucoPose_.pose.orientation.y;
			rvec[2] = arucoPose_.pose.orientation.z;

			rvecs.push_back(rvec);
		}

		// if (detected_apriltag) {
		// 	detected_apriltag = false;
		// 	cv::Vec3d tvec, rvec;
		// 	double roll, pitch, yaw;
		// 	tf2::Quaternion q;

		// 	tvec[0] = apriltagPose_.pose.position.x;
		// 	tvec[1] = apriltagPose_.pose.position.y;
		// 	tvec[2] = apriltagPose_.pose.position.z;
		// 	tvecs.push_back(tvec);

		// 	q.setValue(apriltagPose_.pose.orientation.x,
		// 				apriltagPose_.pose.orientation.y,
		// 				apriltagPose_.pose.orientation.z,
		// 				apriltagPose_.pose.orientation.w);

		// 	tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
		// 	rvec[0] = roll;
		// 	rvec[1] = pitch;
		// 	rvec[2] = yaw;
		// 	rvecs.push_back(rvec);
		// }

		try
		{
			cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::RGB8);
			inImage = cv_ptr->image;
			// std::cout << rvecs.size() << std::endl;
			for (int i = 0; i < rvecs.size(); ++i) {
				// cv::Mat tvec = Mat(3, 1, CV_64FC1);
				// cv::Mat rvec = Mat(3, 1, CV_64FC1);

				// tvec.at<double>(0,0) = tvecs[i][0];
				// tvec.at<double>(1,0) = tvecs[i][1];
				// tvec.at<double>(2,0) = tvecs[i][2];

				// rvec.at<double>(0,0) = rvecs[i][0];
				// rvec.at<double>(1,0) = rvecs[i][1];
				// rvec.at<double>(2,0) = rvecs[i][2];
				auto rvec = rvecs[i];
				auto tvec = tvecs[i];
				std::ostringstream ostr;
				ostr << std::fixed << std::setprecision(2);
				ostr << "Position:"<<" " << tvec;

				// cv::putText(inImage, ostr.str(), cv::Point(30 , 30), cv::FONT_HERSHEY_DUPLEX, 1.0, CV_RGB(26,249,249), 2);

				// std::cout << tvec << std::endl;
				// std::cout << rvec << std::endl;
				cv::drawFrameAxes(inImage, cameraMatrix, distCoeffs, rvec, tvec, 1.0, 3);
			}

			if (image_pub.getNumSubscribers() > 0)
			{
				// show input with augmented information
				cv_bridge::CvImage out_msg;
				out_msg.header.stamp = curr_stamp;
				out_msg.encoding = sensor_msgs::image_encodings::RGB8;
				out_msg.image = inImage;
				image_pub.publish(out_msg.toImageMsg());
			}
		}
		catch (cv_bridge::Exception& e)
		{
			ROS_ERROR("cv_bridge exception: %s", e.what());
			return;
		}
	}

	void cam_info_callback(const sensor_msgs::CameraInfo &msg)
	{
		cameraMatrix.at<double>(0, 0) = msg.K[0];
		cameraMatrix.at<double>(0, 1) = msg.K[1];
		cameraMatrix.at<double>(0, 2) = msg.K[2];
		cameraMatrix.at<double>(1, 0) = msg.K[3];
		cameraMatrix.at<double>(1, 1) = msg.K[4];
		cameraMatrix.at<double>(1, 2) = msg.K[5];
		cameraMatrix.at<double>(2, 0) = msg.K[6];
		cameraMatrix.at<double>(2, 1) = msg.K[7];
		cameraMatrix.at<double>(2, 2) = msg.K[8];
		cam_info_sub.shutdown();
	}

	void getPoseArucoCallback(const geometry_msgs::PoseStamped& msg)
	{
		arucoPose_ = msg;
		detected_aruco = true;
	}

	/* Get Position of marker AprilTag in camera frame */
	void getPoseApriltagCallback(const tf2_msgs::TFMessage& msg)
	{
		if(msg.transforms[0].child_frame_id == "apriltag_bundle") {
			apriltagPose_.pose.position.x = msg.transforms[0].transform.translation.x;
			apriltagPose_.pose.position.y = msg.transforms[0].transform.translation.y;
			apriltagPose_.pose.position.z = msg.transforms[0].transform.translation.z;
			apriltagPose_.pose.orientation.x = msg.transforms[0].transform.rotation.x;
			apriltagPose_.pose.orientation.y = msg.transforms[0].transform.rotation.y;
			apriltagPose_.pose.orientation.z = msg.transforms[0].transform.rotation.z;
			apriltagPose_.pose.orientation.w = msg.transforms[0].transform.rotation.w;
			detected_apriltag = true;

			
		}
	}

	void getPoseWhyconCallback(const geometry_msgs::PoseStamped& msg)
	{
		whyconPose_.pose.position.x = msg.pose.position.x;
		whyconPose_.pose.position.y = msg.pose.position.y;
		whyconPose_.pose.position.z = msg.pose.position.z;
		whyconPose_.pose.orientation.x = msg.pose.orientation.x;
		whyconPose_.pose.orientation.y = msg.pose.orientation.y;
		whyconPose_.pose.orientation.z = msg.pose.orientation.z;
		whyconPose_.pose.orientation.w = msg.pose.orientation.w;
		detected_whycon = true;
	}


};

int main(int argc, char **argv)
{
	ros::init(argc, argv, "debug_image");
	ros::NodeHandle nh("");
	ros::NodeHandle nh_private("~");

	DebugImage DebugImage(nh, nh_private);
	ros::spin();
	return 0;
}
