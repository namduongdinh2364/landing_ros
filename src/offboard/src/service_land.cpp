#include "ros/ros.h"
#include <cstdlib>
#include <std_srvs/SetBool.h>

int main(int argc, char **argv)
{
	ros::init(argc, argv, "land_services");
	ros::NodeHandle n;
	// ros::ServiceClient client_returnHome = n.serviceClient<std_srvs::SetBool>("return_home");
	ros::ServiceClient client2 = n.serviceClient<std_srvs::SetBool>("start_land_1");
	ros::ServiceClient client3 = n.serviceClient<std_srvs::SetBool>("start_land");
	// ros::ServiceClient client4 = n.serviceClient<std_srvs::SetBool>("land4");
	std_srvs::SetBool land_cmd;
	land_cmd.request.data = true;
	// client_returnHome.call(land_cmd);
	client2.call(land_cmd);
	client3.call(land_cmd);
	// client4.call(land_cmd);
	ROS_WARN_STREAM("Done");

	return 0;
}