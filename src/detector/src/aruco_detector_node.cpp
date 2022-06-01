#include <iostream>
#include <map>
#include <vector>
#include <numeric>
#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/CameraInfo.h"
#include "std_msgs/Empty.h"
#include "std_msgs/Int16.h"
/* ROS transform */
#include <tf2/LinearMath/Vector3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_msgs/TFMessage.h>
/* ROS CvBridge */
#include "cv_bridge/cv_bridge.h"
/* Image Transport to publish output img */
#include <image_transport/image_transport.h>
/* OpenCV */
#include <opencv2/highgui.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/imgproc.hpp>
#include <stdint.h>
#include "opencv2/opencv.hpp"

#define ROUND2(x)    std::round(x * 100) / 100
#define ROUND3(x)    std::round(x * 1000) / 1000

/**
 * ID of marker
 * - marker size more than is IDLARGE 
 */
#define IDLARGE             23
#define IDLOW               10
#define SWITCH_HEIGHT       9

using namespace std;
using namespace sensor_msgs;
using namespace cv;

/* Publisher */
image_transport::Publisher read_frame_pub;
ros::Publisher tf_aruco_pose_pub;

/* Offset bwt the center of markers in coordinate marker*/
float marker_size = 0.5;
string marker_tf_prefix;
double distortion[] = {7.2697873963251586e-02, -1.4749282442847444e-01, -2.3233094539353212e-03, 8.9165121414591982e-03,-2.6332902664556002e-01};
Mat distortion_coefficients = Mat(1, 5, CV_32F, distortion);
Matx33d intrinsic_matrix = Matx33d(	917.497, 0.0, 635.002,  \
                                    0.0, 915.865, 368.915,  \
                                    0.0, 0.0, 1.0);
// Ptr<aruco::DetectorParameters> detector_params;
Ptr<cv::aruco::Dictionary> dictionary;

/**
 * ID marker to switch if approach value height be set
 */
uint8_t switch_ID = 23;

tf2::Vector3 cv_vector3d_to_tf_vector3(const Vec3d &vec) {
    return {vec[0], vec[1], vec[2]};
}

tf2::Quaternion cv_vector3d_to_tf_quaternion(const Vec3d &rotation_vector) {
    auto ax    = rotation_vector[0], ay = rotation_vector[1], az = rotation_vector[2];
    auto angle = sqrt(ax * ax + ay * ay + az * az);
    auto cosa  = cos(angle * 0.5);
    auto sina  = sin(angle * 0.5);
    auto qx    = ax * sina / angle;
    auto qy    = ay * sina / angle;
    auto qz    = az * sina / angle;
    auto qw    = cosa;
    tf2::Quaternion q;
    q.setValue(qx, qy, qz, qw);

    return q;
}

tf2::Transform create_transform(const Vec3d &tvec, const Vec3d &rotation_vector) {
    tf2::Transform transform;
    transform.setOrigin(cv_vector3d_to_tf_vector3(tvec));
    transform.setRotation(cv_vector3d_to_tf_quaternion(rotation_vector));
    return transform;
}

void callback(const ImageConstPtr& image_msg) {
    string frame_id = image_msg->header.frame_id;
    auto image = cv_bridge::toCvShare(image_msg)->image;    /* To process */
    vector<int> ids_m;
    vector<int> ids;
    vector<vector<Point2f>> corners, rejected;
    vector<vector<Point2f>> corners_cvt;

    /* Smooth the image to improve detection results */
    if (enable_blur) {
        GaussianBlur(image, image, Size(blur_window_size, blur_window_size), 0, 0);
    }

    /* Detect the markers */
    aruco::detectMarkers(image, dictionary, corners, ids_m, detector_params, rejected);

    /* Show image if no markers are detected */
    if (ids_m.empty()) {
    }

    if(ids_m.size()>0) {
        for(int i = 0;i<ids_m.size();i++) {
            if (ids_m[i] == switch_ID) {
                ROS_INFO("Marker ID: [%d]", ids_m[i]);
                ROS_INFO("Marker size: [%f]", marker_size);
                ids.push_back(ids_m[i]);
                corners_cvt.push_back(corners[i]);
            }
        }
        /* Compute poses of markers */
        vector<Vec3d> rotation_vectors, translation_vectors;
        aruco::estimatePoseSingleMarkers(corners_cvt, marker_size, intrinsic_matrix, distortion_coefficients,
                                         rotation_vectors, translation_vectors);

        for (auto i = 0; i < rotation_vectors.size(); ++i) {
            if (SWITCH_HEIGHT > translation_vectors[0](2)) {
                switch_ID = IDLOW;
                marker_size = 0.2;
            }
            else {
                switch_ID = IDLARGE;
                marker_size = 0.5;
            }
            ROS_INFO("x: [%f]", translation_vectors[i](0));
            ROS_INFO("y: [%f]", translation_vectors[i](1));
            ROS_INFO("z: [%f]", translation_vectors[i](2));
        }
        /* Publish TFs for each of the markers */
        static tf2_ros::TransformBroadcaster br;
        auto stamp = ros::Time::now();

        /* Create and publish tf message for each marker */
        tf2_msgs::TFMessage tf_msg_list;

        for (auto i = 0; i < rotation_vectors.size(); ++i) {
                geometry_msgs::TransformStamped tf_msg;
                stringstream ss;

                auto translation_vector = translation_vectors[i];
                auto rotation_vector    = rotation_vectors[i];
                auto transform          = create_transform(translation_vector, rotation_vector);
                ss << marker_tf_prefix << ids[i];
                tf_msg.header.stamp            = stamp;
                tf_msg.header.frame_id         = frame_id;
                tf_msg.child_frame_id          = ss.str();
                tf_msg.transform.translation.x = transform.getOrigin().getX();
                tf_msg.transform.translation.y = transform.getOrigin().getY();
                tf_msg.transform.translation.z = transform.getOrigin().getZ();
                tf_msg.transform.rotation.x    = transform.getRotation().getX();
                tf_msg.transform.rotation.y    = transform.getRotation().getY();
                tf_msg.transform.rotation.z    = transform.getRotation().getZ();
                tf_msg.transform.rotation.w    = transform.getRotation().getW();
                tf_msg_list.transforms.push_back(tf_msg);
                br.sendTransform(tf_msg);
        }
        
        if( tf_msg_list.transforms.size()) {
            tf_aruco_pose_pub.publish(tf_msg_list);
        }
    }
}

/*TODO: slider extension mach ne hashmap von int,array*/
int main(int argc, char **argv) {
        map<string, aruco::PREDEFINED_DICTIONARY_NAME> dictionary_names;
        dictionary_names.insert(pair<string, aruco::PREDEFINED_DICTIONARY_NAME>("DICT_4X4_50", aruco::DICT_4X4_50));
        dictionary_names.insert(pair<string, aruco::PREDEFINED_DICTIONARY_NAME>("DICT_4X4_100", aruco::DICT_4X4_100));
        dictionary_names.insert(pair<string, aruco::PREDEFINED_DICTIONARY_NAME>("DICT_4X4_250", aruco::DICT_4X4_250));
        dictionary_names.insert(pair<string, aruco::PREDEFINED_DICTIONARY_NAME>("DICT_4X4_1000", aruco::DICT_4X4_1000));
        dictionary_names.insert(pair<string, aruco::PREDEFINED_DICTIONARY_NAME>("DICT_5X5_50", aruco::DICT_5X5_50));
        dictionary_names.insert(pair<string, aruco::PREDEFINED_DICTIONARY_NAME>("DICT_5X5_100", aruco::DICT_5X5_100));
        dictionary_names.insert(pair<string, aruco::PREDEFINED_DICTIONARY_NAME>("DICT_5X5_250", aruco::DICT_5X5_250));
        dictionary_names.insert(pair<string, aruco::PREDEFINED_DICTIONARY_NAME>("DICT_5X5_1000", aruco::DICT_5X5_1000));
        dictionary_names.insert(pair<string, aruco::PREDEFINED_DICTIONARY_NAME>("DICT_6X6_50", aruco::DICT_6X6_50));
        dictionary_names.insert(pair<string, aruco::PREDEFINED_DICTIONARY_NAME>("DICT_6X6_100", aruco::DICT_6X6_100));
        dictionary_names.insert(pair<string, aruco::PREDEFINED_DICTIONARY_NAME>("DICT_6X6_250", aruco::DICT_6X6_250));
        dictionary_names.insert(pair<string, aruco::PREDEFINED_DICTIONARY_NAME>("DICT_6X6_1000", aruco::DICT_6X6_1000));
        dictionary_names.insert(pair<string, aruco::PREDEFINED_DICTIONARY_NAME>("DICT_7X7_50", aruco::DICT_7X7_50));
        dictionary_names.insert(pair<string, aruco::PREDEFINED_DICTIONARY_NAME>("DICT_7X7_100", aruco::DICT_7X7_100));
        dictionary_names.insert(pair<string, aruco::PREDEFINED_DICTIONARY_NAME>("DICT_7X7_250", aruco::DICT_7X7_250));
        dictionary_names.insert(pair<string, aruco::PREDEFINED_DICTIONARY_NAME>("DICT_7X7_1000", aruco::DICT_7X7_1000));
        dictionary_names.insert(pair<string, aruco::PREDEFINED_DICTIONARY_NAME>("DICT_ARUCO_ORIGINAL", aruco::DICT_ARUCO_ORIGINAL));

        bool enable_blur;
        int blur_window_size;
        int image_fps = 15;
        int image_width = 1280;
        int image_height = 720;
//Consolas, 'Courier New', monospace 
        /*Initalize ROS node*/
        int queue_size = 10;
        ros::init(argc, argv, "aruco_detect_node");
        ros::NodeHandle nh("~");
        string rgb_topic, rgb_info_topic, dictionary_name;

        nh.getParam("camera", rgb_topic);
        nh.getParam("camera_info", rgb_info_topic);
        nh.getParam("marker_size", marker_size);
        nh.getParam("image_fps", image_fps);
        nh.getParam("image_width", image_width);
        nh.getParam("image_height", image_height);
        nh.getParam("tf_prefix", marker_tf_prefix);
        nh.getParam("enable_blur", enable_blur);
        nh.getParam("blur_window_size", blur_window_size);

        // detector_params = aruco::DetectorParameters::create();
        // detector_params->cornerRefinementMethod = aruco::CORNER_REFINE_SUBPIX;
        nh.getParam("dictionary_name", dictionary_name);
        // nh.param("aruco_adaptiveThreshWinSizeStep", detector_params->adaptiveThreshWinSizeStep, 4);
        /* Configure ARUCO marker detector */
        dictionary = aruco::getPredefinedDictionary(dictionary_names[dictionary_name]);
        ROS_DEBUG("%f", marker_size);
        /* camera */
        ros::Subscriber rgb_sub = nh.subscribe(rgb_topic.c_str(), queue_size, callback);
        /* Publisher */
        image_transport::ImageTransport it(nh);
        read_frame_pub = it.advertise("/camera/color/image_raw", 10);
        tf_aruco_pose_pub = nh.advertise<tf2_msgs::TFMessage>("/tf_list", 10);
        ros::spin();

        return 0;
}
