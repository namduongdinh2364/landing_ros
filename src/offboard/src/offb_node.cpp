#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <eigen3/Eigen/Core>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <tf/tf.h>
#include <eigen3/Eigen/Geometry>
#include <sensor_msgs/Imu.h>
#include <iostream>
#include <sstream>
#include <fstream>
#include <unistd.h>
#include <ctime>
#include <string>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_msgs/TFMessage.h>
#include <geometry_msgs/TwistStamped.h>
#include <signal.h>
#include <cstdlib>
#include "std_msgs/String.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Int16.h"
#include "MiniPID.h"
#include <stdio.h>
#include <stdlib.h>
#include <mavros_msgs/PositionTarget.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#define LOCAL    1
#define PID      2

#define PRECISION(x)    round(x * 100) / 100
#define CHECK_M         (CHECK_MARK="\033[0;32m\xE2\x9C\x94\033[0m")
#define PRECISION(x)    round(x * 100) / 100
#define PI              3.14159265
#define YAW_ANGLE(x)    (x*(180/PI))
#define RATODE(x)       (x/(180/PI))

using namespace std;
using namespace Eigen;

time_t baygio = time(0);
tm *ltime = localtime(&baygio);
static int STATE_CHECK = 1;
ofstream outfile0;
char path[250];
double cur_roll, cur_pitch, cur_yaw;
double m_roll, m_pitch, m_yaw;
double output_x, output_y, output_z, output_yaw;
static char var_active_status[20];
int16_t mode_select = LOCAL;
Matrix3f R, FLU_rotation_;
Vector3f var_offset_pose;
Vector3f positionbe;
Vector3f positionaf;
mavros_msgs::State current_state;
geometry_msgs::PoseStamped pose;
mavros_msgs::PositionTarget raw_pose;
geometry_msgs::PoseStamped vlocal_pose;
geometry_msgs::TwistStamped var_velocity;
/** Init PID*/ 
MiniPID pid_x = MiniPID(0.4, 0.01, 0.0, 0.1);
MiniPID pid_y = MiniPID(0.4, 0.01, 0.0, 0.1);
MiniPID pid_z = MiniPID(0.4, 0.01, 0.0, 0.1);
MiniPID pid_yaw = MiniPID(0.02, 0.01, 0.0, 0.1);

/* getting the state */
void state_cb(const mavros_msgs::State::ConstPtr& msg) {
    current_state = *msg;
}

void mavrosPose_Callback(const geometry_msgs::PoseStamped::ConstPtr &msg) {
    vlocal_pose=*msg;
    double xq,yq,zq,wq;
    xq = msg->pose.orientation.x;
    yq = msg->pose.orientation.y;
    zq = msg->pose.orientation.z;
    wq = msg->pose.orientation.w;

    tf2::Quaternion q;
    q.setValue(xq, yq, zq, wq);
    tf2::Matrix3x3(q).getRPY(cur_roll, cur_pitch, cur_yaw);

    // cout << "roll: " << YAW_ANGLE(cur_roll) << endl;
    // cout << "pitch: " << YAW_ANGLE(cur_pitch) << endl;
    // cout << "yaw: " << YAW_ANGLE(cur_yaw) << endl;

}

void imuCallback(const sensor_msgs::Imu::ConstPtr &msg) {

    double x,y,z,w;

    x = msg->orientation.x;
    y = msg->orientation.y;
    z = msg->orientation.z;
    w = msg->orientation.w;

    /* making a quaternion of position */
    Quaternionf quat;
    quat = Eigen::Quaternionf(w,x,y,z);

    /*making rotation matrix from quaternion*/
    R = quat.toRotationMatrix();
    // cout << "R=" << endl << R << endl;
}

void posecallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
}

/**
 * @brief
 * 
 * @param 
 *
 * @return 
 */
void set_target_position_callback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
/**  
 * For world frame, we will use ENU (EAST, NORTH and UP)
 * #     +Z     +Y
 * #      ^    ^
 * #      |  /
 * #      |/
 * #    world------> +X
 */
    pose.header.stamp = ros::Time::now();
    pose.pose.position.x= msg->pose.position.x;
    pose.pose.position.y= msg->pose.position.y;
    pose.pose.position.z= msg->pose.position.z;

    double xq,yq,zq,wq;
    xq = msg->pose.orientation.x;
    yq = msg->pose.orientation.y;
    zq = msg->pose.orientation.z;
    wq = msg->pose.orientation.w;

    tf2::Quaternion q;
    q.setValue(xq, yq, zq, wq);
    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(m_roll, m_pitch, m_yaw);

    // cout << "roll: " << m_roll << endl;
    // cout << "pitch: " << m_pitch << endl;
    // cout << "yaw: " << YAW_ANGLE(m_yaw) << endl;

    Quaternionf q_update;
    if (YAW_ANGLE(m_yaw) >= 10 || YAW_ANGLE(m_yaw) <= -10)
    {
        q_update = AngleAxisf(0, Vector3f::UnitX())
                    * AngleAxisf(0, Vector3f::UnitY())
                    * AngleAxisf(cur_yaw - RATODE(10), Vector3f::UnitZ());
    }

    pose.pose.orientation.x = q_update.x();
    pose.pose.orientation.y = q_update.y();
    pose.pose.orientation.z = q_update.z();
    pose.pose.orientation.w = q_update.w();
    // raw_pose.header.stamp = ros::Time::now();
    // raw_pose.position.x = msg->pose.position.x;
    // raw_pose.position.y = msg->pose.position.y;
    // raw_pose.position.z = msg->pose.position.z;
    output_yaw = pid_yaw.getOutput(PRECISION(cur_yaw), PRECISION(cur_yaw - m_yaw));
    output_yaw = PRECISION(output_yaw);
    var_velocity.twist.angular.z = output_yaw;
    cout << "X: " << pose.pose.position.x << "\t" << "Y: " << pose.pose.position.y << "\t" << "Z: " << pose.pose.position.z << endl;
}

// void set_target_yaw_callback(const std_msgs::Float32::ConstPtr& msg)
// {
//     float angle_yaw = msg.data;
// }

void both_mode_callback(const std_msgs::Int16::ConstPtr& msg) {
    mode_select = msg->data;
}

void custom_activity_callback(const std_msgs::String::ConstPtr& msg) {
    strcpy(var_active_status, msg->data.c_str());
}

/**
 * @brief
 * 
 * @param 
 *
 * @return 
 */
void signal_callback_handler(int signum) {
    cout << "\n======================================="<< endl;
    cout << "EXIT programme " << endl;
    exit(signum);
}

void init_pose() {
    // cin  >> raw_pose.position.x >> raw_pose.position.y >> raw_pose.position.z ;
    cin  >> pose.pose.position.x >> pose.pose.position.y >> pose.pose.position.z;

    cout << "\x1B[93mAxis x\033[0m : " << pose.pose.position.x << "m" << endl;
    cout << "\x1B[93mAxis y\033[0m : " << pose.pose.position.y << "m" << endl;
    cout << "\x1B[93mAxis z\033[0m : " << pose.pose.position.z << "m" << endl;
}

int main(int argc, char **argv) {
    int mode_controll;
    // double output_x, output_y, output_z, output_yaw;
    FLU_rotation_ << 1 , 0 , 0 , 0 , 1 , 0 , 0 , 0 , 1;

    /** Display console*/
    cout<< "______  __   __    ___     _____    _____ " << endl;
    cout<< "| ___ \\ \\ \\ / /   /   |   / ___ \\  | ___ \\" <<endl;
    cout<< "| |_/ /  \\ V /   / /| |   | | | |  | |_/ |" <<endl;
    cout<< "|  __/   /   \\  / /_| |   | | | |  |  __ /" <<endl;
    cout<< "| |     / /^\\ \\ \\___  |   | |_| |  | |_/ \\" <<endl;
    cout<< "\\_|     \\/   \\/     |_/   \\_____/  \\_____/\n\n" <<endl;
    cout << "-----Staring mode OFFBOARD CONTROL-----"<< endl;
    cout << "======================================="<< endl;
    cout << "\U0001F449 1: Control follow local position    |"<< endl;
    cout << "\U0001F449 2: Control follow PID               |"<< endl;
    cout << "======================================="<< endl;
    cout << " \x1B[93m\u262D ENTER Option:\033[0m ";

    cin >> mode_controll;

    /* Init position */
    switch(mode_controll) {
        case LOCAL:
            cout << "PLEASE ENTER POSITION INIT Follow Local ENU frame [x y z]: ";
            init_pose();
            break;
        case PID:
            cout << "PID controller is chosen \nENTER Position Local ENU frame [x y z]: ";
            init_pose();
            break;
        default:
            cout << "not yet support" << endl;
            exit(0);
    }

    ros::init(argc, argv, "offboard_node");
    ros::NodeHandle nh;
    /** setup subscribe and public*/ 
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
        ("mavros/state", 10, state_cb);
    ros::Subscriber imu_sub = nh.subscribe<sensor_msgs::Imu>
        ("/mavros/imu/data",10,imuCallback);
    ros::Subscriber local_position_sub = nh.subscribe<geometry_msgs::PoseStamped>
        ("/mavros/local_position/pose",10,mavrosPose_Callback);
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
        ("mavros/setpoint_position/local", 10);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
        ("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
        ("mavros/set_mode");
    ros::Publisher velocity_pub   = nh.advertise <geometry_msgs::TwistStamped>
        ("/mavros/setpoint_velocity/cmd_vel", 30 );

    ros::Publisher local_pos_raw_pub = nh.advertise<mavros_msgs::PositionTarget>
        ("mavros/setpoint_raw/local", 10);

    // ros::Subscriber position_target_sub = nh.subscribe<geometry_msgs::PoseWithCovarianceStamped>
    //     ("kf/estimate",30,set_target_position_callback);

    ros::Subscriber position_target_sub = nh.subscribe<geometry_msgs::PoseStamped>
        ("cmd/set_pose/position1",10,set_target_position_callback);
    // ros::Subscriber yaw_target_sub = nh.subscribe<std_msgs::Float32>
        // ("cmd/set_pose/orientation",10,set_target_yaw_callback);
    ros::Subscriber both_mode_sub = nh.subscribe<std_msgs::Int16>
        ("cmd/set_mode/mode",10,both_mode_callback);
    ros::Subscriber custom_activity_sub = nh.subscribe<std_msgs::String>
        ("cmd/set_activity/type",10,custom_activity_callback);
    // ros::Subscriber pose_sub = nh.subscribe
        // ("/tf_list", 10, get_params_cb);

    ros::Rate rate(20.0);

    if(mode_controll == PID) {
        pid_x.setOutputLimits(-4.0, 4.0);
        pid_y.setOutputLimits(-4.0, 4.0);
        pid_z.setOutputLimits(-2.0, 2.0);
        pid_yaw.setOutputLimits(-0.1745329, 0.1745329);
        pid_x.setOutputRampRate(0.02);
        pid_y.setOutputRampRate(0.02);
        pid_z.setOutputRampRate(0.5);
        pid_yaw.setOutputRampRate(0.01);
    }

    /* wait for FCU connection */
    while(ros::ok() && !current_state.connected) {
        ros::spinOnce();
        rate.sleep();
    }

    mavros_msgs::SetMode offb_set_mode, offset_mode;
    mavros_msgs::CommandBool arm_cmd;
    offb_set_mode.request.custom_mode = "OFFBOARD";
    offset_mode.request.custom_mode = "AUTO.LAND";
    arm_cmd.request.value = true;
    signal(SIGINT,signal_callback_handler);
    ros::Time last_request = ros::Time::now();

    // raw_pose.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
    // raw_pose.type_mask = mavros_msgs::PositionTarget::IGNORE_VX + mavros_msgs::PositionTarget::IGNORE_VY + \
    //                         mavros_msgs::PositionTarget::IGNORE_PX + mavros_msgs::PositionTarget::IGNORE_PY + mavros_msgs::PositionTarget::IGNORE_PZ + \
    //                         mavros_msgs::PositionTarget::IGNORE_VZ + mavros_msgs::PositionTarget::IGNORE_AFX + \
    //                         mavros_msgs::PositionTarget::IGNORE_AFY + mavros_msgs::PositionTarget::IGNORE_AFZ + \
    //                         mavros_msgs::PositionTarget::IGNORE_YAW_RATE;

    // raw_pose.position.x = 2;
    // raw_pose.position.y = 0;
    // raw_pose.position.z = 4;
    // raw_pose.velocity.x = 0.5;
    // raw_pose.velocity.y = 0;
    // raw_pose.yaw = 45 * PI / 180;
 
    pose.pose.orientation.x = 0;
    pose.pose.orientation.y = 0;
    pose.pose.orientation.z = 0.3826834;
    pose.pose.orientation.w = 0.9238795;

    while(ros::ok()) {
#ifdef HITL
        system("echo -n \"\e[4mWaiting for activation mode\e[0m\n\"");
        system("echo -n \"OFFBOARD && ARMED mode...\"");
        while(current_state.mode != "OFFBOARD" || !current_state.armed);
        system("echo -e \"\\r\033[0;32m\xE2\x9C\x94\033[0m OFFBOARD && ARMED ready!!!\"");
#else
        if (STATE_CHECK == 1) {
            if( current_state.mode != "OFFBOARD" && (ros::Time::now() - last_request > ros::Duration(5.0))) {
                if( set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent) {
                    ROS_INFO("Offboard enabled");
                }
                last_request = ros::Time::now();
            } else {
                if( !current_state.armed && (ros::Time::now() - last_request > ros::Duration(5.0))) {
                    if( arming_client.call(arm_cmd) && arm_cmd.response.success) {
                        ROS_INFO("Vehicle armed");
                    }
                    last_request = ros::Time::now();
                }
            }
        }
#endif /* HITL */
        if (strcmp(var_active_status,"LAND") == 0) {
            last_request = ros::Time::now();

            if( current_state.mode != "AUTO.LAND" ) {
                offset_mode.request.custom_mode = "AUTO.LAND";

                if( set_mode_client.call(offset_mode) && offset_mode.response.mode_sent) {
                    ROS_INFO("AUTO LAND enabled");
                    STATE_CHECK = 0;
                }
                last_request = ros::Time::now();
            }
        }

        baygio = time(0);
        ltime = localtime(&baygio);

        if(mode_controll == PID) {
            if (abs(vlocal_pose.pose.position.x - pose.pose.position.x) <= 2) {
                pid_x.setOutputLimits(-2.0, 2.0);
                pid_y.setOutputLimits(-2.0, 2.0);
            } else {
                pid_x.setOutputLimits(-4.0, 4.0);
                pid_y.setOutputLimits(-4.0, 4.0);
            }
            
            output_x = pid_x.getOutput(PRECISION(vlocal_pose.pose.position.x), pose.pose.position.x);
            output_y = pid_y.getOutput(PRECISION(vlocal_pose.pose.position.y), pose.pose.position.y);
            output_z = pid_z.getOutput(PRECISION(vlocal_pose.pose.position.z), pose.pose.position.z);

            output_x = PRECISION(output_x);
            output_y = PRECISION(output_y);
            output_z = PRECISION(output_z);

            var_velocity.twist.linear.x = output_x;
            var_velocity.twist.linear.y = output_y;
            var_velocity.twist.linear.z = output_z;
        }
        

        switch(mode_controll) {
            case LOCAL:
                local_pos_pub.publish(pose);
                // local_pos_raw_pub.publish(raw_pose);
                break;
            case PID:
                velocity_pub.publish(var_velocity);
                break;
            default:
                cout << "not yet support" << endl;
                exit(1);
        }

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
