/**
 * 
*/
#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <sstream>
#include <string>
#include <ros/ros.h>
#include <eigen3/Eigen/Geometry>
#include <geometry_msgs/PoseStamped.h>
#include "controller_msgs/FlatTarget.h"

using namespace std;
using namespace Eigen;

static geometry_msgs::PoseStamped mavros_local_position_pose, desPose;
// controller_msgs::FlatTarget desPose;

/**
 * @brief Get local position form mavros
 */
void mavrosPose_Callback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
        mavros_local_position_pose = *msg;
}

int main(int argc, char **argv) {
        /* Display console */
        cout << "-------Mode CONTROL-------"<< endl;
        cout << "======================================="<< endl;
        ros::init(argc, argv, "interactive_node");
        ros::NodeHandle nh;

        ros::Subscriber mavros_local_position_sub = nh.subscribe<geometry_msgs::PoseStamped>
                ("/mavros/local_position/pose", 10, mavrosPose_Callback);
        ros::Publisher mavros_position_local_pub = nh.advertise<controller_msgs::FlatTarget>
                ("reference/flatsetpoint", 1);
        ros::Publisher set_desposition_local_pub = nh.advertise<geometry_msgs::PoseStamped>
                ("cmd/set_desposition/local", 10);
        ros::Rate rate(20.0);
        char char_c;
        while(ros::ok()) {
                /* Init position */
                cout << "POSITION [x y z] = ";
                cin  >> desPose.pose.position.x >> desPose.pose.position.y >> desPose.pose.position.z;
                cout << "x: " << desPose.pose.position.x << "m" << endl;
                cout << "y: " << desPose.pose.position.y << "m" << endl;
                cout << "z: " << desPose.pose.position.z << "m" << endl;

                desPose.header.stamp = ros::Time::now();
                desPose.header.frame_id = "map";
                set_desposition_local_pub.publish(desPose);
                cout << "to quit, enter q: " << endl;
                cin >> char_c;
                if (char_c == 'q') {
                        return 0;
                }
                ros::spinOnce();
                rate.sleep();
        }
        return 0;
}
