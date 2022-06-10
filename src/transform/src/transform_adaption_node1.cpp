/**
 * Input pose of marker to drone
 * Output pose of marker to NEU
*/
#include <iostream>
#include <ros/ros.h>
#include <sstream>
#include <fstream>
#include <unistd.h>
#include <ctime>
#include <math.h>
#include "std_msgs/Float32.h"
#include "std_msgs/String.h"
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <eigen3/Eigen/Dense>
#include <sensor_msgs/Imu.h>
#include <tf2_msgs/TFMessage.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <std_srvs/SetBool.h>
#include "landing.h"

using namespace std;
using namespace Eigen;

static geometry_msgs::PoseStamped mavros_local_position_pose;
static geometry_msgs::PoseStamped desPose, mov2CurPose;
static Matrix3f imu_rotation_matrix, cam2drone_matrix, marker_rotation_matrix;
static Vector3f drone_postition, cam_postition, marker2neu_postition, point_change, marker2neu_postition_change;
static mavros_msgs::State current_state;
mavros_msgs::SetMode set_mode;

static int stable_loop = 0;
static bool locked_trans = false;
static bool locked_landing = false;
static bool locked_inc_altitude = false;
static bool accept_landing = false;
static bool marker_detected = false;
int detect_failed_repeat = 0;
ros::Time last_time_trans, stable_time_trans, time_increase_repeat_trans;
ros::Publisher desPose_pub;
ros::Publisher pub_desyaw;
std::map< int, Eigen::Vector3d> IntersectionPoints;

double cur_yaw;

#define DEGREE2RADIAN(x)	x*3.14159265/180

void mavrosPose_Callback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
	mavros_local_position_pose = *msg;
}

double get_yaw_from_quat(double xq, double yq, double zq, double wq)
{
	tf2::Quaternion q;
	double roll, pitch, yaw;

	q.setValue(xq, yq, zq, wq);
	tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

	return yaw;
}

void imuPose_Callback(const sensor_msgs::Imu::ConstPtr& msg)
{
	double x, y, z, w;
	Quaternionf quat;

	x = msg->orientation.x;
	y = msg->orientation.y;
	z = msg->orientation.z;
	w = msg->orientation.w;

	cur_yaw = get_yaw_from_quat(x, y, z, w);
	/* making a quaternion of position */
	quat = Eigen::Quaternionf(w, x, y, z);
	/* making rotation matrix from quaternion */
	imu_rotation_matrix = quat.toRotationMatrix();
}

void getIntersectionPoint(Eigen::Vector3d pp1, Eigen::Vector3d pp2, Eigen::Vector3d pp3, Eigen::Vector3d pp4)
{
	// Eigen::Vector3d intersectionPoint(1, 2, 3);
	// IntersectionPoints[0] = intersectionPoint;
	// std::cout << IntersectionPoints.at(0) << std::endl;
	// IntersectionPoints.clear();
}

// vector<Eigen::Vector3d> get_points()
// {
// 	/**
// 	 *      A *---------------* B
// 	 *        |   \           |1
// 	 *        |       \       |
// 	 *        |           \   |
// 	 *      D *---------------* C (UAV)
// 	 */

// 	double AC, ang_CAD, A30, A20, A15, A10;
// 	double axisx30, axisx20, axisx15, axisx10;
// 	double axisy30, axisy20, axisy15, axisy10;
// 	AC = (double)sqrt(pow(marker2neu_postition[0],2) +
// 								pow(marker2neu_postition[1],2));
// 	ang_CAD = acos(abs(marker2neu_postition[0])/ AC) * 180 / PI;

// 	Eigen::Vector3d point30, point20, point15, point10;

// 	A30 = tan(DEGREE2RADIAN(30)) * mavros_local_position_pose.pose.position.z;
// 	A20 = tan(DEGREE2RADIAN(20)) * mavros_local_position_pose.pose.position.z;
// 	A15 = tan(DEGREE2RADIAN(15)) * mavros_local_position_pose.pose.position.z;
// 	A10 = tan(DEGREE2RADIAN(10)) * mavros_local_position_pose.pose.position.z;

// 	axisx30 = cos(ang_CAD * PI/180) * A30;
// 	axisx20 = cos(ang_CAD * PI/180) * A20;
// 	axisx15 = cos(ang_CAD * PI/180) * A15;
// 	axisx10 = cos(ang_CAD * PI/180) * A10;

// 	axisy30 = cos((90 - ang_CAD) * PI/180) * A30;
// 	axisy20 = cos((90 - ang_CAD) * PI/180) * A20;
// 	axisy15 = cos((90 - ang_CAD) * PI/180) * A15;
// 	axisy10 = cos((90 - ang_CAD) * PI/180) * A10;

// 	Eigen::Vector3d vtmarker, vtdrone, IntersectionPoint1;
// 	vtmarker << mov2CurPose.pose.position.x, mov2CurPose.pose.position.y, mov2CurPose.pose.position.z;
// 	vtdrone << mavros_local_position_pose.pose.position.x, mavros_local_position_pose.pose.position.y, mavros_local_position_pose.pose.position.z;
// 	point10 << axisx10, axisy10, mavros_local_position_pose.pose.position.z;

// 	getIntersectionPoint(vtmarker, vtdrone, vtmarker, point10);
// 	// vtcp = traj_origin_ - vtmarker;
// 	// position = traj_origin_ + (vtcp * 0.1);

// 	// std::cout << "AC: " << AC << std::endl;
// 	// std::cout << "A30: " << A10 << std::endl;
// 	// std::cout << "angle CAD: " << ang_CAD << std::endl;
// 	// std::cout << "Point 30: " << axisx10 << " - "<< axisy10 << std::endl;

// }

static void get_info_form_marker_Callback(const tf2_msgs::TFMessage::ConstPtr& msg)
{
	/** Reset last transform time */
	last_time_trans = TIME_NOW;
	marker_detected = true;
	/**
	 * The detection stabilizes after 3 sec.
	 * detect_failed_repeat and locked_inc_altitude wil be reset.
	 */
	if(TIME_NOW - stable_time_trans > TIME_DURATION(3.0)) {
		detect_failed_repeat = 0;
		locked_inc_altitude = false;
	}

	if(!locked_landing) {
		/* Marker ----> Drone */
		cam_postition[0] = msg->transforms[0].transform.translation.x;
		cam_postition[1] = msg->transforms[0].transform.translation.y;
		cam_postition[2] = msg->transforms[0].transform.translation.z;
		cam2drone_matrix << 0.0 , -1.0 , 0.0 , -1.0 , 0.0 , 0.0 , 0.0 , 0.0 , -1.0;

		drone_postition = cam2drone_matrix * cam_postition;

		/* Marker ----> NEU */
		marker2neu_postition = imu_rotation_matrix * drone_postition;
		// get_points();
		if(!locked_trans) {
			mov2CurPose.pose.position.x = marker2neu_postition[0] + mavros_local_position_pose.pose.position.x;
			mov2CurPose.pose.position.y = marker2neu_postition[1] + mavros_local_position_pose.pose.position.y;
			mov2CurPose.pose.position.z = marker2neu_postition[2] + mavros_local_position_pose.pose.position.z;

			mov2CurPose.pose.position.x = PRECISION(mov2CurPose.pose.position.x);
			mov2CurPose.pose.position.y = PRECISION(mov2CurPose.pose.position.y);
			mov2CurPose.pose.position.z = 0.5;
			locked_trans = true;
			// Eigen::Quaternionf q(msg->transforms[0].transform.rotation.w, msg->transforms[0].transform.rotation.x,
			// 		msg->transforms[0].transform.rotation.y, msg->transforms[0].transform.rotation.z);
			// // Eigen::Quaternionf q1(mavros_local_position_pose.pose.orientation.x,
			// // 						mavros_local_position_pose.pose.orientation.y,
			// // 						mavros_local_position_pose.pose.orientation.z,
			// // 						mavros_local_position_pose.pose.orientation.w)

			// marker_rotation_matrix = q.toRotationMatrix();
			// Eigen::Matrix3f rpy2drone;
			// rpy2drone = marker_rotation_matrix;
			// Eigen::Vector3f rpy = rpy2drone.eulerAngles(0, 1, 2);
			// Eigen::Vector3f rpyD = imu_rotation_matrix.eulerAngles(0, 1, 2);


			// std_msgs::Float32 msg_yaw;
			//         /**
			//  * If yaw into range of +- 15 degrees.
			//  * It should be continually updated orientation
			//  */
			// if (abs(rpy(2) + rpyD(2)) <= 3.1) {
			// 		// q_update = AngleAxisf(0, Vector3f::UnitX()) *
			// 		// 			AngleAxisf(0, Vector3f::UnitY()) *
			// 		// 			AngleAxisf(cur_yaw - RATODE(10), Vector3f::UnitZ());
			// 	msg_yaw.data = rpy(2) + rpyD(2);
			// 	// pub_desyaw.publish(msg_yaw);
			// 	std::cout << rpy(2) << "   " << rpyD(2)  << "==== " << msg_yaw.data << std::endl;
			// }
			// else
			// {
			// 	msg_yaw.data = 0;
			// 	// pub_desyaw.publish(msg_yaw);
			// 	std::cout << rpy(2) << "   " << rpyD(2)  << "==== " << msg_yaw.data << std::endl;
			// }
			
		}

		/* Check error angle */
		float var_AC, var_alpha;
		static float curErr_alpha;
		var_AC = (float)sqrt(pow(drone_postition[0],2) + pow(drone_postition[1],2));
		var_alpha = atan(var_AC / abs(drone_postition[2])) * 180 / PI;

		if(mavros_local_position_pose.pose.position.z >= ALTITUDE_CHANGE_ANGLE) {
			curErr_alpha = ERROR_ACCEPTANCE_DEGREES_20;
		}
		else {
			curErr_alpha = ERROR_ACCEPTANCE_DEGREES_10;
		}
		
		if(var_alpha <= curErr_alpha && mavros_local_position_pose.pose.position.z > ALTITUDE_CHANGE_METHOD) {
			desPose.pose.position.x = mov2CurPose.pose.position.x;
			desPose.pose.position.y = mov2CurPose.pose.position.y;

			if (mavros_local_position_pose.pose.position.z >= 0.8) {
				if (desPose.pose.position.z <= 0.5) {
					desPose.pose.position.z = 0.5;
				} else {
					desPose.pose.position.z = mavros_local_position_pose.pose.position.z - 2.0;
				}
			}
			else {
				accept_landing = true;
			}
		}
		else if(mavros_local_position_pose.pose.position.z <= (ALTITUDE_CHANGE_METHOD + 0.2)) {
			if(var_AC <= DISTANCE) {
				desPose.pose.position.x = mov2CurPose.pose.position.x;
				desPose.pose.position.y = mov2CurPose.pose.position.y;

				if (mavros_local_position_pose.pose.position.z >= 0.8) {
					if (desPose.pose.position.z <= 0.5) {
						desPose.pose.position.z = 0.5;
					} else {
						desPose.pose.position.z = mavros_local_position_pose.pose.position.z - 2.0;
					}
				}
				else {
					accept_landing = true;
				}
			} else {
				desPose.pose.position.x = mov2CurPose.pose.position.x;
				desPose.pose.position.y = mov2CurPose.pose.position.y;
				desPose.pose.position.z = mavros_local_position_pose.pose.position.z;
				// ROS_INFO("Aligning under 5m........!");
			}
		}
		else {
			desPose.pose.position.x = mov2CurPose.pose.position.x;
			desPose.pose.position.y = mov2CurPose.pose.position.y;
			desPose.pose.position.z = mavros_local_position_pose.pose.position.z;
			// ROS_INFO("Aligning........!");
		}

		// desPose.pose.position.x = mov2CurPose.pose.position.x;
		// desPose.pose.position.y = mov2CurPose.pose.position.y;
		// desPose.pose.position.z = mov2CurPose.pose.position.z;

		/**
		 * stable param
		*/
		stable_loop ++;
		if(stable_loop == NUM_LOOP) {
			locked_trans = false;
			stable_loop = 0;
		}
		desPose_pub.publish(desPose);

			// Eigen::Quaterniond q(msg->transforms[0].transform.rotation.x,
			// 		msg->transforms[0].transform.rotation.y, msg->transforms[0].transform.rotation.z, msg->transforms[0].transform.rotation.w);
			// Eigen::Quaterniond q1(mavros_local_position_pose.pose.orientation.x,
			// 						mavros_local_position_pose.pose.orientation.y,
			// 						mavros_local_position_pose.pose.orientation.z,
			// 						mavros_local_position_pose.pose.orientation.w);

			// marker_rotation_matrix = q.toRotationMatrix();
			// Eigen::Matrix3f rpy2drone;
			// rpy2drone = marker_rotation_matrix;
			// Eigen::Vector3f rpy = rpy2drone.eulerAngles(0, 1, 2);
			// Eigen::Vector3d rpy = Eigen::Matrix3d(q).eulerAngles(0, 1, 2);  // RPY
			double err_yaw;
			err_yaw = get_yaw_from_quat(msg->transforms[0].transform.rotation.x, msg->transforms[0].transform.rotation.y,
										msg->transforms[0].transform.rotation.z, msg->transforms[0].transform.rotation.w);


			std_msgs::Float32 msg_yaw;
			        /**
			 * If yaw into range of +- 15 degrees.
			 * It should be continually updated orientation
			 */
			// if (rpy(2) + rpyD(2) >= 3.0) {
			// 	msg_yaw.data = (rpy(2) + rpyD(2)) - 3.14;
			// 	std::cout << rpy(2) << "   " << rpyD(2) << "==== " << msg_yaw.data << std::endl;
			// }
			// else if (rpy(2) + rpyD(2) <= -3.0) {
			// 	msg_yaw.data = rpy(2) + rpyD(2) + 3.14;
			// 	std::cout << rpy(2) << "   " << rpyD(2) << "==== " << msg_yaw.data << std::endl;
			// }
			// // else {
			// // 	msg_yaw.data = rpy(2) + rpyD(2);
			// // }
			// else if (rpy(2) < 0 & rpyD(2) > 0) {
			// 	msg_yaw.data = rpy(2) + rpyD(2);
			// 	std::cout << rpy(2) << "   " << rpyD(2) << "==== " << msg_yaw.data << std::endl;
			// }
			std::cout << err_yaw << " === " << cur_yaw <<std::endl;
			// else if () {
				
			// }
			// msg_yaw.data =  -rpy(2) + rpyD(2);
			// pub_desyaw.publish(msg_yaw);


#ifdef LOG_INFO
	cout<< "Marker2Drone : "<< PRECISION(drone_postition[0]) <<'\t'
				<< PRECISION(drone_postition[1]) << '\t'
				<< PRECISION(drone_postition[2]) << endl;
	cout<< "Marker2NEU : " << mov2CurPose.pose.position.x <<'\t'<< mov2CurPose.pose.position.y << '\t' << mov2CurPose.pose.position.z << endl;
	cout << "===================================================" << endl;
#endif /* LOG_INFO */
	}
}

void get_current_setpoint_position_local_Callback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
}

void state_Callback(const mavros_msgs::State::ConstPtr& msg) {
	current_state = *msg;
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "transform_node");
	ros::NodeHandle transform;

	ros::Subscriber mavros_imu_data_sub = transform.subscribe<sensor_msgs::Imu>
		("/mavros/imu/data", 10, imuPose_Callback);
	ros::Subscriber mavros_local_position_sub = transform.subscribe<geometry_msgs::PoseStamped>
		("/mavros/local_position/pose", 10, mavrosPose_Callback);
	ros::Subscriber marker_pose_sub = transform.subscribe<tf2_msgs::TFMessage>
		("/tf", 10, get_info_form_marker_Callback);
	desPose_pub = transform.advertise<geometry_msgs::PoseStamped>
		("cmd/set_desposition/local", 10);
	pub_desyaw = transform.advertise<std_msgs::Float32>
		("reference/yaw", 10);
	ros::ServiceClient set_mode_client = transform.serviceClient<mavros_msgs::SetMode>
		("mavros/set_mode");
	ros::Subscriber mavros_state_sub = transform.subscribe<mavros_msgs::State>
		("mavros/state", 10, state_Callback);
	ros::ServiceClient arming_client = transform.serviceClient<mavros_msgs::CommandBool>
		("mavros/cmd/arming");
	ros::ServiceClient land_client = transform.serviceClient<std_srvs::SetBool>
		("/land");
	ros::Rate rate(20.0);
	mavros_msgs::CommandBool arm_cmd;
	mavros_msgs::SetMode offboard_set_mode;
	offboard_set_mode.request.custom_mode = "OFFBOARD";
	arm_cmd.request.value = true;
	ros::Time last_time = ros::Time::now();

	last_time_trans = TIME_NOW;
	time_increase_repeat_trans = TIME_NOW;
	int timeout_trans = 2 * MAX_REPEAT_DETECT;
	bool off_run = false;

	// Eigen::Vector3d abc, cd;
	// abc << 1.0, 1.2, 3.0;
	// IntersectionPoints[0] = abc;

    // map<int, Eigen::Vector3d>::iterator it;
	// cd = IntersectionPoints.at(0);
	// cout << cd(0) << " done " <<cd(1) << endl;

	while(ros::ok()) {
		// while (!off_run) {
		// 	if(current_state.mode != "OFFBOARD" && (ros::Time::now() - last_time > ros::Duration(5.0))) {
		// 		if(set_mode_client.call(offboard_set_mode) && offboard_set_mode.response.mode_sent) {
		// 			ROS_INFO("Offboard enabled");
		// 		}
		// 		last_time = ros::Time::now();
		// 	} else {
		// 		if(!current_state.armed && (ros::Time::now() - last_time > ros::Duration(5.0))) {
		// 			if(arming_client.call(arm_cmd) && arm_cmd.response.success) {
		// 				ROS_INFO("Vehicle armed");
		// 				off_run = true;
		// 			}
		// 			last_time = ros::Time::now();
		// 		}
		// 	}
		// }
		/**
		 * If can't detect marker during 2 second.
		 * Get detect again failed more than maxium,
		 * Landing will be called.
		 */
		if((TIME_NOW - last_time_trans) > TIME_DURATION(2.0) && marker_detected) {
			static double new_altitude = 0;
			if(TIME_NOW - time_increase_repeat_trans > TIME_DURATION(2.0)) {
				detect_failed_repeat ++;
				time_increase_repeat_trans = TIME_NOW;
				cout << "Can't detect Marker "<< detect_failed_repeat << endl;
			}
			if(!locked_inc_altitude) {
				new_altitude = mavros_local_position_pose.pose.position.z + INCREASE_ALTITUDE_NOT_DETECT;
			}
			locked_inc_altitude = true;
			stable_time_trans = TIME_NOW;
			desPose.pose.position.x = mavros_local_position_pose.pose.position.x;
			desPose.pose.position.y = mavros_local_position_pose.pose.position.y;
			desPose.pose.position.z = new_altitude;
			if((TIME_NOW - last_time_trans > TIME_DURATION(timeout_trans)) || detect_failed_repeat == MAX_REPEAT_DETECT) {
				cout << "Detection failed: Timeout" <<endl;
				accept_landing = true;
			}
			desPose_pub.publish(desPose);
		}
		/**
		 * If the current pose greater than the maximum altitude,
		 * Landing mode should be enabled.
		 */
		if(MAX_ALTITUDE < mavros_local_position_pose.pose.position.z) {
			accept_landing = true;
			desPose.pose.position.z = MAX_ALTITUDE;
			desPose_pub.publish(desPose);
			cout << "Enable Landing: UAV is approaded the maximum altitude" <<endl;
		}

		if (accept_landing) {
			locked_landing = true;
			/* Public message to the status topic for the next status will be required */
			if(current_state.mode != "AUTO.LAND") {
				std_srvs::SetBool land_cmd;
				land_cmd.request.data = true;
				// land_client.call(land_cmd);
				set_mode.request.custom_mode = "AUTO.LAND";
				if(set_mode_client.call(set_mode) && set_mode.response.mode_sent) {
					ROS_INFO("AUTO LANDING MODE is required");
				}
			}
			cout << "\n========================================"<< endl;
			cout << "-------------SENT LANDING------------" << endl;
			cout<< "Drone : x= " << mavros_local_position_pose.pose.position.x << \
				      " y= " << mavros_local_position_pose.pose.position.y << \
				      " z= " << mavros_local_position_pose.pose.position.z << endl;
			cout << "========================================" << endl;

			return 0;
		}

		ros::spinOnce();
		rate.sleep();
	}

	return 0;
}
