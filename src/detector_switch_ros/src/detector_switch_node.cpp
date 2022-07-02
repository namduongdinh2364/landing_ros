#include "detector_switch.h"

int main(int argc, char **argv) {
	ros::init(argc, argv, "detector_switch");
	ros::NodeHandle nh("");
	ros::NodeHandle nh_private("~");
	DetectorSwitch DetectorSwitch(nh, nh_private);
	ros::spin();
	return 0;
}
