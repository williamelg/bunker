
    #!/usr/bin/env python

import rclpy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2


def video_publisher_node():
    #rospy.init_node('video_publisher_node', anonymous=True)
    node = rclpy.create_node('video_publisher_node')
    rate = node.create_rate(20)  # 20Hz

    video_capture = cv2.VideoCapture(0)  

    image_pub = node.create_publisher(Image, 'video_frames', 10)
    bridge = CvBridge()

    while rclpy.ok():
        ret, frame = video_capture.read()
        if ret:
            image_message = bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            image_pub.publish(image_message)
           
        
        rclpy.spin_once(node)
        

def main(args=None):
    rclpy.init(args=args)    
    aruco_node = video_publisher_node()
    rclpy.spin(aruco_node)
    aruco_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
