#include <iostream>
#include <ros/ros.h>
#include <image_transport/image_transport.h>

using namespace std;
using namespace sensor_msgs;

class CamInformation
{
private:
	ros::NodeHandle nh_;
	ros::NodeHandle nh_private_;
	ros::Publisher cam_info_pub;
	ros::Timer pub_camInfo_loop_;
	sensor_msgs::CameraInfo camera_model;
	ros::Subscriber image_sub;
	ros::Time timestamp;

public:
	CamInformation(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private)
	: nh_(nh), nh_private_(nh_private)
	{
		cam_info_pub = nh_.advertise<sensor_msgs::CameraInfo>
			("/camera/color/camera_info", 1);
		image_sub = nh_.subscribe("/camera/color/image_raw", 1, &CamInformation::image_callback, this);

		pub_camInfo_loop_ = nh_.createTimer(ros::Duration(0.1), &CamInformation::pubCamInfoCallback, this);

		camera_model.header.frame_id = "cameraIn4";
		camera_model.header.stamp = timestamp;

		camera_model.height = 720;
		camera_model.width = 1280;
		camera_model.K[0] = 917.4971923828125;
		camera_model.K[1] = 0.0;
		camera_model.K[2] = 635.0016479492188;
		camera_model.K[3] = 0.0;
		camera_model.K[4] = 915.86474609375;
		camera_model.K[5] = 368.91497802734375;
		camera_model.K[6] = 0.0;
		camera_model.K[7] = 0.0;
		camera_model.K[8] = 1.0;
	}

	void pubCamInfoCallback(const ros::TimerEvent& event)
	{
		cam_info_pub.publish(camera_model);
	}

	void image_callback(const sensor_msgs::ImageConstPtr& msg)
	{
		timestamp = msg->header.stamp;
		// rgb_pub.publish(rgb_msg);

		// camera_model.header.stamp = timestamp;
	}


};


int main(int argc, char **argv)
{
	ros::init(argc, argv, "cam_info");
	ros::NodeHandle nh("");
	ros::NodeHandle nh_private("~");

	CamInformation CamInformation(nh, nh_private);
	ros::spin();
	return 0;
}
