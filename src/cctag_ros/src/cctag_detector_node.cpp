#include "cctag_ros/cctag_detector.h"

int main(int argc, char **argv) {
	ros::init(argc, argv, "cctag_detector");
	ros::NodeHandle nh("");
	ros::NodeHandle nh_private("~");
	CCTagDetector CCTagDetector(nh, nh_private);
	ros::spin();
	return 0;
}
