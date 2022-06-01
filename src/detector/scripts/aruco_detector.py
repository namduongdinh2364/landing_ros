#! /usr/bin/env python3
"""
    Script ...
"""

from numpy.core.fromnumeric import size
import pyrealsense2 as rs
import numpy as np
import cv2
import rospy
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Image
import math

class ImageConverter():
    def __init__(self):
        self.__bridge = CvBridge()
        self.__image_sub = rospy.Subscriber("/camera/color/image_raw", Image, self.get_image_callback)
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
        self.marker_pose_pb = rospy.Publisher("marker/imu_position/pose", PoseStamped, queue_size=10)
        # Real camera
        # self.cam_mtx = np.array([917.497, 0.0, 635.002, 0.0, 915.865, 368.915, 0.0, 0.0, 1.0]).reshape(3,3)
        # self.cam_dist = np.array([7.2697873963251586e-02, -1.4749282442847444e-01, -2.3233094539353212e-03, 8.9165121414591982e-03,-2.6332902664556002e-01])
        self.cam_mtx = np.array([1179.598752, 0.0, 928.099247, 0.0, 1177.000389, 558.635461, 0.0, 0.0, 1.0]).reshape(3,3)
        self.cam_dist = np.array([0.061957, -0.124832, 0.002573, -0.004753, 0.0])
        self.dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_6X6_50)
        self.param = cv2.aruco.DetectorParameters_create()
        self.font = cv2.FONT_HERSHEY_SIMPLEX
        self.param.adaptiveThreshConstant = 7
        self.cam2imu_mtx = np.array([[0, -1.0, 0, 0.06],
                                     [-1.0, 0, 0, 0.04],
                                     [0, 0, -1.0, -0.08],
                                     [0, 0, 0, 1.0]])

        self.ids_target = 23
        self.markerLength = 0.5
        rospy.loginfo("Init detect marker")

    @staticmethod
    def vector3d_to_tf_quaternion(v3drvec):
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

    def start_get_pose(self):
        while not rospy.is_shutdown():
            if self.cv_image is not None:
                gray_image = cv2.cvtColor(self.cv_image, cv2.COLOR_RGB2GRAY)
                corners, ids, rejectedImgPoints = cv2.aruco.detectMarkers(gray_image, self.dict, parameters=self.param)

                if np.all(ids is not None):
                    for i in range(0, ids.size):
                        if ids[i] == self.ids_target:
                            self.corners = corners[i]
                            markerLength = self.markerLength

                            ret = cv2.aruco.estimatePoseSingleMarkers(corners = self.corners,
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
                            qx, qy, qz, qw = MarkerDetector.vector3d_to_tf_quaternion(rvec)
                            transform.pose.orientation.x = qx
                            transform.pose.orientation.y = qy
                            transform.pose.orientation.z = qz
                            transform.pose.orientation.w = qw

                            self.public_pose(transform)

                            # For debug
                            cv2.aruco.drawAxis(self.cv_image, self.cam_mtx, self.cam_dist, rvec, tvec, 0.1)
                            str_position0 = f"Marker Position in Camera frame: x={translation_vector[0]} y={translation_vector[1]} z={translation_vector[2]}"
                            cv2.putText(self.cv_image, str_position0, (0, 50),
                                        self.font, 0.7, (0, 255, 0), 1, cv2.LINE_AA)
                            self.image_message_pub(self.cv_image)
                # cv2.imshow("frame", self.cv_image)
                # cv2.waitKey(1)

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
