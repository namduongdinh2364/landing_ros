#! /usr/bin/env python3
"""
    Script ...
"""

import numpy as np
import cv2
import rospy
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Image
import math
from image_dehaze import ImageDehze

# The list and directory contain marker's ID.
# A marker couled be detected if it's ID into list and directory.
dir_marker_ids = [7, 6, 3, 2, 1]
aruco_marker= {
    "8": {
        "id": 8,
        "size": 0.8,
        "dictionary": "DICT_6X6_50"
    },
    "7": {
        "id": 7,
        "size": 0.7,
        "dictionary": "DICT_6X6_50"
    },
    "6": {
        "id": 6,
        "size": 0.6,
        "dictionary": "DICT_5X5_50"
    },
    "5": {
        "id": 5,
        "size": 0.5,
        "dictionary": "DICT_6X6_50"
    },
    "4": {
        "id": 4,
        "size": 0.4,
        "dictionary": "DICT_6X6_50"
    },
    "3": {
        "id": 3,
        "size": 0.3,
        "dictionary": "DICT_5X5_50"
    },
    "2": {
        "id": 2,
        "size": 0.2,
        "dictionary": "DICT_6X6_50"
    },
    "1": {
        "id": 1,
        "size": 0.1,
        "dictionary": "DICT_6X6_50"
    }
}

class ImageConverter():
    def __init__(self):
        self.__image_raw = rospy.get_param("~image_raw", "/camera/color/image_raw")
        self.__bridge = CvBridge()
        self.__image_sub = rospy.Subscriber(self.__image_raw, Image, self.get_image_callback)
        self.__image_pub = rospy.Publisher("debug/image/marker",Image, queue_size=10)
        self.cv_image = None
        self.rate = rospy.Rate(20)
        rospy.loginfo("Init convert image")

    def get_image_callback(self, image_data):
        self.cv_image = self.__bridge.imgmsg_to_cv2(image_data, "bgr8")
    
    def image_message_pub(self, cv_image):
        self.__image_pub.publish(self.__bridge.cv2_to_imgmsg(cv_image, "bgr8"))

class MarkerDetector(ImageConverter):
    def __init__(self):
        super().__init__()
        self.__image_width = rospy.get_param("~image_width", 0)
        self.__image_height = rospy.get_param("~image_height", 0)
        self.marker_pose_pb = rospy.Publisher("marker/imu_position/pose", PoseStamped, queue_size=10)
        # Real camera
        self.cam_mtx = np.array([917.497, 0.0, 635.002, 0.0, 915.865, 368.915, 0.0, 0.0, 1.0]).reshape(3,3)
        self.cam_dist = np.array([7.2697873963251586e-02, -1.4749282442847444e-01, -2.3233094539353212e-03, 8.9165121414591982e-03,-2.6332902664556002e-01])
        # self.cam_mtx = np.array([1179.598752, 0.0, 928.099247, 0.0, 1177.000389, 558.635461, 0.0, 0.0, 1.0]).reshape(3,3)
        # self.cam_dist = np.array([0.061957, -0.124832, 0.002573, -0.004753, 0.0])
        self.dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_6X6_50)
        self.param = cv2.aruco.DetectorParameters_create()
        self.font = cv2.FONT_HERSHEY_SIMPLEX
        self.param.adaptiveThreshConstant = 7
        self.cam2imu_mtx = np.array([[0, -1.0, 0, 0.06],
                                     [-1.0, 0, 0, 0.04],
                                     [0, 0, -1.0, -0.08],
                                     [0, 0, 0, 1.0]])
        rospy.loginfo("Init detect marker")

    def vector3d_to_tf_quaternion(self, v3drvec):
        ax = v3drvec[0]
        ay = v3drvec[1]
        az = v3drvec[2]

        angle = math.sqrt(ax * ax + ay * ay + az * az)
        cosa = math.cos(angle * 0.5)
        sina = math.sin(angle * 0.5)
        qx = ax * sina / angle
        qy = ay * sina / angle
        qz = az * sina / angle
        qw = cosa
        return qx, qy, qz, qw

    def get_area_marker(self, corners):
        point1 = corners[0][0]
        point2 = corners[0][1]
        point3 = corners[0][2]

        height = math.sqrt(math.pow((point2[0] - point1[0]), 2) +
                           math.pow((point2[1] - point1[1]), 2))
        width = math.sqrt(math.pow((point3[0] - point2[0]), 2) +
                          math.pow((point3[1] - point2[1]), 2))

        area = int(height * width)

        return area

    def get_area_image(self):
        return self.__image_width * self.__image_height

    def start_get_pose(self):
        while not rospy.is_shutdown():
            if self.cv_image is not None:
                gray_image = cv2.cvtColor(self.cv_image, cv2.COLOR_RGB2GRAY)
                corners, ids, rejectedImgPoints = cv2.aruco.detectMarkers(gray_image, self.dict, parameters=self.param)
                if np.all(ids is not None):
                    list_size_markers = []
                    """ Get the biggest size form all of detected markers. """
                    for marker_id in dir_marker_ids:
                        for index, id in enumerate(ids):
                            if marker_id == id:
                                pixel_area_marker = self.get_area_marker(corners[index])
                                info_marker = {'id': marker_id,
                                                'size': aruco_marker[str(marker_id)]['size'],
                                                'corners': corners[index],
                                                'area': pixel_area_marker}
                                list_size_markers.append(info_marker)

                    """ Get Pose. """
                    if list_size_markers:
                        list_size_markers.sort(key=lambda x: x.get('size'), reverse = True)
                        corners = None
                        markerLength = None
                        cur_id = 0
                        for elem in list_size_markers:
                            if len(list_size_markers) > 1:
                                # Select a marker have the pixcel area under allowed bound
                                # if elem['area'] < 50000:
                                corners = elem['corners']
                                markerLength = elem['size']
                                cur_id = elem['id']
                                # print(elem['area'])
                                break
                            # If detect only one marker
                            else:
                                corners = elem['corners']
                                markerLength = elem['size']
                                cur_id = elem['id']

                        # print(cur_id)
                        if markerLength is not None:
                            ret = cv2.aruco.estimatePoseSingleMarkers(corners = corners,
                                                                    markerLength = markerLength,
                                                                    cameraMatrix = self.cam_mtx,
                                                                    distCoeffs = self.cam_dist)
                            (rvec, tvec) = ret[0][0, 0, :], ret[1][0, 0, :]
                            (rvec - tvec).any()  # get rid of that nasty numpy value array error

                            translation_vector = np.array ([[tvec[0]],
                                                            [tvec[1]],
                                                            [tvec[2]],
                                                            [1.0]])

                            transform = PoseStamped()
                            data = np.matmul(self.cam2imu_mtx, translation_vector)
                            transform.pose.position.x = data[0]
                            transform.pose.position.y = data[1]
                            transform.pose.position.z = data[2]
                            qx, qy, qz, qw = self.vector3d_to_tf_quaternion(rvec)
                            transform.pose.orientation.x = qx
                            transform.pose.orientation.y = qy
                            transform.pose.orientation.z = qz
                            transform.pose.orientation.w = qw

                            self.public_pose(transform)
                            print(translation_vector[2])
                            # For debug
                            str_position0 = f"Marker Position in Camera frame: x={translation_vector[0]} y={translation_vector[1]} z={translation_vector[2]} ID: {cur_id}"
                            cv2.putText(self.cv_image, str_position0, (0, 50),
                                        self.font, 0.7, (0, 50, 255), 1, cv2.LINE_AA)
                            self.image_message_pub(self.cv_image)
                cv2.namedWindow("frame", cv2.WINDOW_NORMAL) 
                cv2.imshow("frame", self.cv_image)
                cv2.waitKey(1)

    def public_pose(self, pose_data):
        self.marker_pose_pb.publish(pose_data)
        self.rate.sleep()

def main():
    rospy.init_node('detect_marker', anonymous=True)

    marker_detect = MarkerDetector()
    marker_detect.start_get_pose()

if __name__ == '__main__':
    try:
        main()
    finally:
        print("Shutting down")
