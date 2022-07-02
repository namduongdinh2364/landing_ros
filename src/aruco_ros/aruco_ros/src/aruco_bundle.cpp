/*****************************
 Copyright 2011 Rafael Muñoz Salinas. All rights reserved.

 Redistribution and use in source and binary forms, with or without modification, are
 permitted provided that the following conditions are met:

 1. Redistributions of source code must retain the above copyright notice, this list of
 conditions and the following disclaimer.

 2. Redistributions in binary form must reproduce the above copyright notice, this list
 of conditions and the following disclaimer in the documentation and/or other materials
 provided with the distribution.

 THIS SOFTWARE IS PROVIDED BY Rafael Muñoz Salinas ''AS IS'' AND ANY EXPRESS OR IMPLIED
 WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND
 FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL Rafael Muñoz Salinas OR
 CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
 ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

 The views and conclusions contained in the software and documentation are those of the
 authors and should not be interpreted as representing official policies, either expressed
 or implied, of Rafael Muñoz Salinas.
 ********************************/

#include <iostream>
#include <aruco/aruco.h>
#include <aruco/cvdrawingutils.h>

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <aruco_ros/aruco_ros_utils.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/Marker.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <dynamic_reconfigure/server.h>
#include <aruco_ros/ArucoThresholdConfig.h>

using namespace cv;
using namespace aruco;
using namespace std;

class ArucoSimple
{
private:
	cv::Mat inImage;
	aruco::CameraParameters camParam;
	tf::StampedTransform rightToLeft;
	bool useRectifiedImages;
	aruco::MarkerDetector mDetector;
	std::vector<aruco::Marker> markers;
	ros::Subscriber cam_info_sub;
	bool cam_info_received;
	image_transport::Publisher image_pub;
	ros::Publisher pose_pub;
	ros::Publisher transform_pub;
	ros::Publisher position_pub;

	std::string marker_frame;
	std::string camera_frame;
	std::string reference_frame;
	MarkerMap TheMarkerMapConfig;

	ros::NodeHandle nh;
	image_transport::ImageTransport it;
	image_transport::Subscriber image_sub;

	tf::TransformListener _tfListener;

	dynamic_reconfigure::Server<aruco_ros::ArucoThresholdConfig> dyn_rec_server;

public:
	ArucoSimple() :
	cam_info_received(false), nh("~"), it(nh)
	{
        /* Currently aruco map file use absolute path. */
		TheMarkerMapConfig.readFromFile("/home/nam97/master/landing_ros/src/aruco_ros/aruco_ros/cfg/config.yml");

		aruco::MarkerDetector::Params params = mDetector.getParameters();
		std::string thresh_method;
		switch (params._thresMethod)
		{
			case aruco::MarkerDetector::ThresMethod::THRES_ADAPTIVE:
				thresh_method = "THRESH_ADAPTIVE";
				break;
			case aruco::MarkerDetector::ThresMethod::THRES_AUTO_FIXED:
				thresh_method = "THRESH_AUTO_FIXED";
				break;
			default:
				thresh_method = "UNKNOWN";
				break;
		}

		// Print parameters of ArUco marker detector:
		ROS_INFO_STREAM("Threshold method: " << thresh_method);

		float min_marker_size; // percentage of image area
		nh.param<float>("min_marker_size", min_marker_size, 0.02);

		std::string detection_mode;
		nh.param<std::string>("detection_mode", detection_mode, "DM_FAST");
		if (detection_mode == "DM_FAST")
			mDetector.setDetectionMode(aruco::DM_FAST, min_marker_size);
		else if (detection_mode == "DM_VIDEO_FAST")
			mDetector.setDetectionMode(aruco::DM_VIDEO_FAST, min_marker_size);
		else
			// Aruco version 2 mode
			mDetector.setDetectionMode(aruco::DM_NORMAL, min_marker_size);

		ROS_INFO_STREAM("Marker size min: " << min_marker_size << "% of image area");
		ROS_INFO_STREAM("Detection mode: " << detection_mode);

		image_sub = it.subscribe("/image", 1, &ArucoSimple::image_callback, this);
		cam_info_sub = nh.subscribe("/camera_info", 1, &ArucoSimple::cam_info_callback, this);

		image_pub = it.advertise("result", 1);
		pose_pub = nh.advertise<geometry_msgs::PoseStamped>("pose", 100);
		transform_pub = nh.advertise<geometry_msgs::TransformStamped>("transform", 100);
		position_pub = nh.advertise<geometry_msgs::Vector3Stamped>("position", 100);

		nh.param<std::string>("reference_frame", reference_frame, "");
		nh.param<std::string>("camera_frame", camera_frame, "");
		nh.param<std::string>("marker_frame", marker_frame, "");
		nh.param<bool>("image_is_rectified", useRectifiedImages, true);

		ROS_ASSERT(camera_frame != "" && marker_frame != "");

		if (reference_frame.empty())
			reference_frame = camera_frame;

		dyn_rec_server.setCallback(boost::bind(&ArucoSimple::reconf_callback, this, _1, _2));
	}

	bool getTransform(const std::string& refFrame, const std::string& childFrame, tf::StampedTransform& transform)
	{
		std::string errMsg;

		if (!_tfListener.waitForTransform(refFrame, childFrame, ros::Time(0), 
                                            ros::Duration(0.5), ros::Duration(0.01), &errMsg))
		{
			ROS_ERROR_STREAM("Unable to get pose from TF: " << errMsg);
			return false;
		}
		else
		{
			try
			{
				_tfListener.lookupTransform(refFrame, childFrame, ros::Time(0), transform);
			}
			catch (const tf::TransformException& e)
			{
				ROS_ERROR_STREAM("Error in lookupTransform of " << childFrame << " in " << refFrame);
				return false;
			}

		}
		return true;
	}

	void image_callback(const sensor_msgs::ImageConstPtr& msg)
	{
		if (cam_info_received)
		{
			ros::Time curr_stamp = msg->header.stamp;
			cv_bridge::CvImagePtr cv_ptr;
			try
			{
				cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::RGB8);
				inImage = cv_ptr->image;

				// detection results will go into "markers"
				markers.clear();
				vector<Marker> Markers = mDetector.detect(inImage);
				// detect the 3d camera location wrt the markerset (if possible)
				if (TheMarkerMapConfig.isExpressedInMeters() && camParam.isValid())
				{
					MarkerMapPoseTracker MSPoseTracker;  // tracks the pose of the marker map
					MSPoseTracker.setParams(camParam, TheMarkerMapConfig);
					if (MSPoseTracker.estimatePose(Markers)) {
						tf::Transform transform = aruco_ros::arucoMarkerMap2Tf(MSPoseTracker);
						tf::StampedTransform cameraToReference;
						cameraToReference.setIdentity();
						if (reference_frame != camera_frame) {
							getTransform(reference_frame, camera_frame, cameraToReference);
						}
						transform = static_cast<tf::Transform>(cameraToReference) * 
									static_cast<tf::Transform>(rightToLeft) * transform;
						tf::StampedTransform stampedTransform(transform, curr_stamp, reference_frame, marker_frame);
						geometry_msgs::PoseStamped poseMsg;
						tf::poseTFToMsg(transform, poseMsg.pose);
						poseMsg.header.frame_id = reference_frame;
						poseMsg.header.stamp = curr_stamp;
						pose_pub.publish(poseMsg);
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
		camParam = aruco_ros::rosCameraInfo2ArucoCamParams(msg, useRectifiedImages);

		// handle cartesian offset between stereo pairs
		// see the sensor_msgs/CameraInfo documentation for details
		rightToLeft.setIdentity();
		rightToLeft.setOrigin(tf::Vector3(-msg.P[3] / msg.P[0], -msg.P[7] / msg.P[5], 0.0));

		cam_info_received = true;
		cam_info_sub.shutdown();
	}

	void reconf_callback(aruco_ros::ArucoThresholdConfig &config, uint32_t level)
	{
		mDetector.setDetectionMode(aruco::DetectionMode(config.detection_mode), config.min_image_size);
		if (config.normalizeImage)
		{
			ROS_WARN("normalizeImageIllumination is unimplemented!");
		}
	}
};

int main(int argc, char **argv)
{
	ros::init(argc, argv, "aruco_bundle");
	ArucoSimple node;

	ros::spin();
	return 0;
}
