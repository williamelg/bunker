#!/usr/bin/env python

import rclpy
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge
import numpy as np
import cv2
from cv2 import aruco
from cv2.aruco import DetectorParameters

class VideoSubscriberNode:
    def __init__(self):
        self.node = rclpy.create_node('video_subscriber_node')
        self.bridge = CvBridge()
        self.create_subscriber()
        self.create_publisher()
        #self.subscription  
        #self.bridge = CvBridge()

        # Load calibration data
        self.calib_data_path = "/home/jetson/WP/bunker-main/s7_code/python/calib_data/CameraCalibrationData.npz"
        calib_data = np.load(self.calib_data_path)
        self.cam_mat = calib_data["camMatrix"]
        self.dist_coef = calib_data["distCoef"]

        # Aruco marker initialization
        self.MARKER_SIZE = 14.8  # centimeters
        self.marker_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
        self.param_markers = DetectorParameters()

    # Load the image 
    def create_subscriber(self):
        self.subscriber = self.node.create_subscription(Image, '/my_camera/image_raw', self.image_callback, 10)

    def create_publisher(self):  
        self.publisher = self.node.create_publisher(PoseStamped, 'aruco_pose', 10)

   # Preocess the aruco code
    def image_callback(self, msg):
        bridge = CvBridge()
        self.get_logger().info('Receiving video frame')
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg)

            # Détecter les marqueurs
            marker_corners, marker_IDs, _ = aruco.detectMarkers(
            cv_image, self.marker_dict, parameters=self.param_markers
            )

            #corners = None  
            # Estimate pose of each marker
            if marker_corners:
            
                rvecs, tvecs, objPoints = aruco.estimatePoseSingleMarkers(
                marker_corners, self.MARKER_SIZE, self.cam_mat, self.dist_coef)
                
                total_markers = range(0, marker_IDs.size)
                for ids, corners, i in zip(marker_IDs, marker_corners, total_markers):
                    #corners = corners.reshape(4, 2)
                    distance = np.linalg.norm(tvecs[i][0])
                    #print(rvecs)
                    

                # Prepare PoseStamped message
                pose_msg = PoseStamped()
                pose_msg.header.stamp = self.node.get_clock().now().to_msg()
                pose_msg.header.frame_id = str(marker_IDs)
                pose_msg.pose.position.x = float(round(tvecs[i][0][0]))
                pose_msg.pose.position.y = float(round(tvecs[i][0][1]))
                pose_msg.pose.position.z = float(round(tvecs[i][0][2]))
                pose_msg.pose.orientation.x = float(round(rvecs[i][1][0]))
                
                pose_msg.pose.orientation.y = float(round(rvecs[i][1][1]))
                pose_msg.pose.orientation.z = float(round(rvecs[i][1][2]))
                #pose_msg.pose.orientation.w = float(round(rvecs[i][0][3]))
               
                # Publish PoseStamped message
                #self.create_publisher.publish(pose_msg)
                self.publisher.publish(pose_msg)
                #print(pose_msg)

            cv2.imshow("Video Frames", cv_image)
            cv2.waitKey(1)
        except Exception as e:
            print("Error converting image:", e)

    # loop
    def run(self):
        rclpy.spin(self.node)
        self.node.destroy_node()
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)    
    camera_node = VideoSubscriberNode()
    camera_node.run()

if __name__ == '__main__':
    main()
