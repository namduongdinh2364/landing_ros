#include <iostream>
#include <sstream>
#include <fstream>
#include <unistd.h>
#include <ctime>
#include <string>
#include <signal.h>

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>

#define PRECISION(x)    round(x * 100) / 100

using namespace std;

ofstream outfile0;
ofstream outfile1;



class ParserData
{
private:
	ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;

    ros::Subscriber sub_mavros_local_position_;
    ros::Subscriber sub_reference_pose_;

    time_t baygio          = time(0);
    tm *ltime              = localtime(&baygio);


public:
	ParserData(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private)
    : nh_(nh), nh_private_(nh_private) {

        sub_mavros_local_position_ = nh_private_.subscribe
		("/mavros/local_position/pose", 1, &ParserData::mavrosPose_Callback, this, ros::TransportHints().tcpNoDelay());

        sub_reference_pose_ = nh_private_.subscribe
		("/reference/pose", 1, &ParserData::referencePose_Callback, this, ros::TransportHints().tcpNoDelay());

    }

    void mavrosPose_Callback(const geometry_msgs::PoseStamped& msg)
    {
        baygio = time(0);
        ltime = localtime(&baygio);

        outfile0 << PRECISION(msg.pose.position.x) << " " \
        << PRECISION(msg.pose.position.y) << " " \
        << PRECISION(msg.pose.position.z) << " " \
        << ltime->tm_min << " " << ltime->tm_sec << " " \
        << msg.header.seq << endl;
    }

    void referencePose_Callback(const geometry_msgs::PoseStamped& msg)
    {
        baygio = time(0);
        ltime = localtime(&baygio);

        outfile1 << PRECISION(msg.pose.position.x) << " " \
        << PRECISION(msg.pose.position.y) << " " \
        << PRECISION(msg.pose.position.z) << " " \
        << ltime->tm_min << " " << ltime->tm_sec << " " \
        << msg.header.seq << endl;
    }

};

void signal_callback_handler(int signum)
{
    cout << "\n======================================="<< endl;
    outfile0.close();
    outfile1.close();
    cout << "\nSaved File" << endl;
    cout << "/home/nam97/workspace/dataplot/" << endl;
    cout << "EXIT programme " << endl;
    exit(signum);
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "parser_data");

	ros::NodeHandle nh("");
	ros::NodeHandle nh_private("~");

    outfile0.open("/home/nam97/workspace/dataplot/datamavros.txt");
    outfile1.open("/home/nam97/workspace/dataplot/datareference.txt");

    outfile0 << "x " << "y " << "z " << "m " << "s " << "sqe" << endl;
    outfile1 << "x " << "y " << "z " << "m " << "s " << "sqe" << endl;
    signal(SIGINT, signal_callback_handler);
	ParserData ParserData(nh, nh_private);

	ros::spin();
	return 0;
}
