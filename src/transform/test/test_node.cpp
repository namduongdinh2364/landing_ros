#include "transform_adaption.h"

int main(int argc, char **argv) {
	ros::init(argc, argv, "test_transform_adaption");
	ros::NodeHandle nh("");
	ros::NodeHandle nh_private("~");
	transformAdaption transformApriltag(nh, nh_private);
	ros::spin();
	return 0;
}
