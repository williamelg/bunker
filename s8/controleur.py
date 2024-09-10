import rclpy
import numpy as np
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped, Twist

class controleur(Node):
    def __init__(self):
        super().__init__('controleur')
        self.get_logger().info('Initialisation du subscriber')
        self.subscription = self.create_subscription(PoseStamped, 'aruco_pose', self.moteur_callback, 10)
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.target_distance = 100.0 
        self.target_orientation = 0.0  
        self.phase = 'alignement'  # Initialisation de la phase à l'alignement

    def moteur_callback(self, data):
        # Extraction des coordonnées de l'ArUco
        cmd_vel = Twist()
        tvect = np.array([data.pose.position.x, data.pose.position.y, data.pose.position.z])
        distance = np.linalg.norm(tvect) 
        self.get_logger().info(str(distance))
        # Calcul de l'orientation désirée
        target_orientation = np.arctan2(tvect[0], tvect[2])
        self.get_logger().info(str(target_orientation))
        
        if abs(target_orientation) > 0.03:
          if target_orientation > 0.03:
            cmd_vel.angular.z = -(5*target_orientation)/(1+target_orientation)
          else:
            cmd_vel.angular.z = -(5*target_orientation)/(1+target_orientation)
        else:
          cmd_vel.angular.z = 0.0
          if (distance > self.target_distance):
            #cmd_vel.linear.x = (0.5*distance)/(1+distance)
            cmd_vel.linear.x = -0.1
          else:
            cmd_vel.linear.x = 0.0


        self.publisher.publish(cmd_vel)

def main(args=None):
    rclpy.init(args=args)
    aruco_node = controleur()
    rclpy.spin(aruco_node)
    aruco_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
