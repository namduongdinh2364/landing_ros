#include <iostream>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <geometry_msgs/PoseStamped.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/highgui.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>

using namespace std;
using namespace sensor_msgs;
using namespace cv;

class ArucoDetector
{
private:
	ros::NodeHandle nh;
	ros::Subscriber cam_info_sub;
	ros::Publisher pose_pub;
	std::string reference_frame;
	image_transport::Publisher image_pub;
	image_transport::Subscriber image_sub;
	image_transport::ImageTransport it;

	cv::Mat inImage;
	cv::Mat cameraMatrix;
	cv::Mat distCoeffs;
	std::vector<std::vector<cv::Point3f>> objPoints;
	std::vector<int> board_ids;
	cv::Ptr<cv::aruco::Board> board;
	cv::Ptr<cv::aruco::Dictionary> dictionary;
	bool cam_info_received;

public:
	ArucoDetector() : nh("~"), it(nh)
	{
		image_sub = it.subscribe("/camera/color/image_raw", 1, &ArucoDetector::image_callback, this);
		cam_info_sub = nh.subscribe("/camera/color/camera_info", 1, &ArucoDetector::cam_info_callback, this);
		image_pub = it.advertise("result", 1);
		pose_pub = nh.advertise<geometry_msgs::PoseStamped>("pose", 10);
		nh.param<std::string>("reference_frame", reference_frame, "aruco");

		cameraMatrix = Mat(1, 9, CV_64F);
		distCoeffs = Mat(1, 5, CV_64F);
		int i;
		for (i = 0; i < 5; i++)
		{
			distCoeffs.at<double>(0,i) = 0.0;
		}
		vector<Point3f> points_id3, points_id4, points_id5, points_id6;
		// points ID 3
		points_id3.push_back(Point3f(-0.125, 0.4, 0.0));
		points_id3.push_back(Point3f(0.125, 0.4, 0.0));
		points_id3.push_back(Point3f(0.125, 0.15, 0.0));
		points_id3.push_back(Point3f(-0.125, 0.15, 0.0));
		// points ID 4
		points_id4.push_back(Point3f(0.15, 0.125, 0.0));
		points_id4.push_back(Point3f(0.4, 0.125, 0.0));
		points_id4.push_back(Point3f(0.4, -0.125, 0.0));
		points_id4.push_back(Point3f(0.15, -0.125, 0.0));
		// points ID 5
		points_id5.push_back(Point3f(-0.125, -0.15, 0.0));
		points_id5.push_back(Point3f(0.125, -0.15, 0.0));
		points_id5.push_back(Point3f(0.125, -0.4, 0.0));
		points_id5.push_back(Point3f(-0.125, -0.4, 0.0));
		// points ID 6
		points_id6.push_back(Point3f(-0.4, 0.125, 0.0));
		points_id6.push_back(Point3f(-0.15, 0.125, 0.0));
		points_id6.push_back(Point3f(-0.15, -0.125, 0.0));
		points_id6.push_back(Point3f(-0.4, -0.125, 0.0));

		objPoints.push_back(points_id3);
		objPoints.push_back(points_id4);
		objPoints.push_back(points_id5);
		objPoints.push_back(points_id6);

		// std::cout << objPoints.at(3) << std::endl;
		// store ids are used
		board_ids.push_back(3);
		board_ids.push_back(4);
		board_ids.push_back(5);
		board_ids.push_back(6);
		// std::cout << "number objec: " << objPoints.size()<< std::endl;
		// std::cout << "number ids: " << board_ids.size()<< std::endl;
		dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_250);
		board = cv::aruco::Board::create(objPoints, dictionary, board_ids);
	}

	void image_callback(const sensor_msgs::ImageConstPtr& msg)
	{
		if ((image_pub.getNumSubscribers() == 0) && (pose_pub.getNumSubscribers() == 0))
		{
			ROS_DEBUG("No subscribers, not looking for ArUco markers");
			return;
		}

		if (cam_info_received)
		{
			ros::Time curr_stamp = msg->header.stamp;
			cv_bridge::CvImagePtr cv_ptr;
			try
			{
				cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::RGB8);
				inImage = cv_ptr->image;

				std::vector<std::vector<cv::Point2f>> markerCorners;
				std::vector<int> markerIds;
				cv::aruco::detectMarkers(inImage, dictionary, markerCorners, markerIds);
				// if at least one marker detected
				if(markerIds.size() > 0) {
					cv::Mat rvec, tvec;
					int valid = cv::aruco::estimatePoseBoard(markerCorners,
															 markerIds,
															 board, 
															 cameraMatrix, 
															 distCoeffs, 
															 rvec, 
															 tvec);
					if (valid) {
						geometry_msgs::PoseStamped poseMsg;
						poseMsg.header.frame_id = reference_frame;
						poseMsg.header.stamp = curr_stamp;
						poseMsg.pose.position.x = tvec.at<double>(0);
						poseMsg.pose.position.y = tvec.at<double>(1);
						poseMsg.pose.position.z = tvec.at<double>(2);
						pose_pub.publish(poseMsg);
					}
					
					// std::cout << "tvec: " << tvec << std::endl;

					// cv::Matx33d R;
					// cv::Rodrigues(rvec, R);
					// Eigen::Matrix3d wRo;
					// wRo << R(0,0), R(0,1), R(0,2), R(1,0), R(1,1), R(1,2), R(2,0), R(2,1), R(2,2);
					// Eigen::Matrix3d inverM = wRo.inverse();
					// std::cout << "rvec: " << wRo << std::endl;
					// std::cout << "rvec inverse: " << inverM << std::endl;
					// std::cout << "rvecRPY: " << wRo.eulerAngles(0, 1, 2) * 180 /3.14 << std::endl;
					// Eigen::Vector3d v3dPoint (tvec.at<double>(2), tvec.at<double>(0), tvec.at<double>(1));
					// v3dPoint << tvec.at<double>(0), tvec.at<double>(1), tvec.at<double>(2);
					// std::cout << "Marker Frame: " << inverM* v3dPoint << std::endl;

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
			}
			catch (cv_bridge::Exception& e)
			{
				ROS_ERROR("cv_bridge exception: %s", e.what());
				return;
			}
		}
	}

	// wait for one camerainfo, then shut down that subscriber
	void cam_info_callback(const sensor_msgs::CameraInfo &msg)
	{
		int i;

		for (i = 0; i < 9; i++) {
			cameraMatrix.at<double>(0, i) = msg.K[i];
			// std::cout << cameraMatrix.at<float>(0, i) << "---"<< std::endl;
			// cameraMatrix.push_back(msg.K[i]);
		}
		cameraMatrix = cameraMatrix.reshape(3, 3);
		cam_info_received = true;
		std::cout << "Get cameraMaxtrix done!!!" << std::endl;
		cam_info_sub.shutdown();
	}
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "aruco_detector");

  ArucoDetector node;

  ros::spin();
}
