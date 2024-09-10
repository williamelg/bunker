#!/usr/bin/env python3
import rclpy
import numpy as np
from rclpy.node import Node
from sensor_msgs.msg import Image # Image is the message type
from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images
import cv2 # OpenCV library
from geometry_msgs.msg import PoseStamped # Import PoseStamped message
import sys
from cv2.aruco import DetectorParameters
from cv2 import aruco

class ImageSubscriber(Node):
  def __init__(self):
    super().__init__('image_subscriber')
    self.get_logger().info('init subscriber')
    self.node = rclpy.create_node('video_subscriber_node')
    #self.bridge = CvBridge()
    #self.create_subscriber()
    #self.create_publisher()
        #self.subscription  
        #self.bridge = CvBridge()

        # Load calibration data
    self.calib_data_path = "/home/jetson/WP/bunker-main/s7_code/python/calib_data/CameraCalibrationData.npz"
    calib_data = np.load(self.calib_data_path)
    self.cam_mat = calib_data["camMatrix"]
    self.dist_coef = calib_data["distCoef"]

        # Aruco marker initialization
    self.MARKER_SIZE = 14.8  # centimeters
    self.marker_dict = aruco.getPredefinedDictionary(aruco.DICT_6X6_50)
    self.param_markers = DetectorParameters()

    self.subscription = self.create_subscription(
      Image, 'video_frames', self.image_callback, 1) 
    self.publisher = self.create_publisher(PoseStamped, 'aruco_pose', 10)   
      # Used to convert between ROS and OpenCV images
    self.br = CvBridge()

  #def create_publisher(self):  
   # self.publisher = self.node.create_publisher(PoseStamped, 'aruco_pose', 10)

  def image_callback(self,Image):
     #bridge = CvBridge()
    self.get_logger().info('Receiving video frame')
    try:
      current_frame = self.br.imgmsg_to_cv2(Image)
    # Process the aruco code

      # Detect markers
      marker_corners, marker_IDs, _ = aruco.detectMarkers(
        current_frame, self.marker_dict, parameters=self.param_markers
        )

     # Estimate pose of each marker
      if marker_corners:
        rvecs, tvecs, objPoints = aruco.estimatePoseSingleMarkers(
          marker_corners, self.MARKER_SIZE, self.cam_mat, self.dist_coef
          )
        total_markers = range(0, marker_IDs.size)
        for ids, corners, i in zip(marker_IDs, marker_corners, total_markers):
          distance = np.linalg.norm(tvecs[i][0])

                    # Prepare PoseStamped message
          pose_msg = PoseStamped()
          pose_msg.header.stamp = self.node.get_clock().now().to_msg()
          pose_msg.header.frame_id = str(marker_IDs)
          pose_msg.pose.position.x = float(round(tvecs[i][0][0]))
          pose_msg.pose.position.y = float(round(tvecs[i][0][1]))
          pose_msg.pose.position.z = float(round(tvecs[i][0][2]))
          pose_msg.pose.orientation.x = float(round(rvecs[i][0][0]))
          pose_msg.pose.orientation.y = float(round(rvecs[i][0][1]))
          pose_msg.pose.orientation.z = float(round(rvecs[i][0][2]))
          #pose_msg.pose.orientation.w = float(round(rvecs[i][0][3]))

          # Publish PoseStamped message
          #self.create_publisher.publish(pose_msg)
          self.publisher.publish(pose_msg)
          #print(pose_msg)

      cv2.imshow("current_frame", current_frame)
      cv2.waitKey(1)
    except Exception as e:
      print("Error converting image:", e)

def main(args=None):
  rclpy.init(args=args)    
  node = ImageSubscriber()
  rclpy.spin(node)
  rclpy.shutdown()

if __name__== "__main__":
  main()
