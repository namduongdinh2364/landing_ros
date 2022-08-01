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

public:
	CamInformation(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private)
	: nh_(nh), nh_private_(nh_private)
	{
		cam_info_pub = nh_.advertise<sensor_msgs::CameraInfo>
			("/camera/color/camera_info", 1);

		pub_camInfo_loop_ = nh_.createTimer(ros::Duration(0.1), &CamInformation::pubCamInfoCallback, this);

		camera_model.header.frame_id = "cameraIn4";
		camera_model.height = 720;
		camera_model.width = 1280;
		camera_model.K[0] = 1.0;
		camera_model.K[1] = 1.0;
		camera_model.K[2] = 1.0;
		camera_model.K[3] = 1.0;
		camera_model.K[4] = 1.0;
		camera_model.K[5] = 1.0;
		camera_model.K[6] = 1.0;
		camera_model.K[7] = 1.0;
		camera_model.K[8] = 1.0;
	}

	void pubCamInfoCallback(const ros::TimerEvent& event)
	{
		cam_info_pub.publish(camera_model);
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
