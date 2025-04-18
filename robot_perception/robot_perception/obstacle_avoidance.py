import rclpy
from rclpy.node import Node # pour faire des noeuds 
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2 # bibliothèque de opencv
import numpy as np
from geometry_msgs.msg import Vector3
from sensor_msgs.msg import LaserScan



class LidarSubscriber(Node):
    def __init__(self):
        super().__init__('lidar_subscriber')
        self.subscription = self.create_subscription( # recoit des images du turtlebot
            LaserScan,
            '/scan', # peut etre mettre /image_raw/compressed
            self.listener_callback,
            10)
        self.window_name='LidarScan'
        cv2.namedWindow(self.window_name,cv2.WINDOW_NORMAL)

        
    def listener_callback(self, msg):
        # Créer une image noire
        img_size = 500
        image = np.zeros((img_size, img_size, 3), dtype=np.uint8)

        center = img_size // 2
        scale = 100  # pixels par mètre

        for i, r in enumerate(msg.ranges):
            if np.isinf(r) or np.isnan(r):
                continue
            angle = msg.angle_min + i * msg.angle_increment
            x = r * np.cos(angle)
            y = r * np.sin(angle)

            px = int(center + x * scale)
            py = int(center - y * scale)

            if 0 <= px < img_size and 0 <= py < img_size:
                cv2.circle(image, (px, py), 2, (0, 255, 0), -1)

        cv2.imshow(self.window_name, image)
        cv2.waitKey(1)




def main(args=None):

    rclpy.init(args=args)
    lidar_subscriber = LidarSubscriber()
    rclpy.spin(lidar_subscriber)
    lidar_subscriber.destroy_node()
    rclpy.shutdown()



if __name__ == '__main__':
    main()









