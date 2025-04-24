import rclpy
from rclpy.node import Node # pour faire des noeuds 
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2 # bibliothèque de opencv
import numpy as np
from geometry_msgs.msg import Vector3
from sensor_msgs.msg import LaserScan


from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3

from std_msgs.msg import String


class ObstacleAvoidance(Node):
    def __init__(self):
        super().__init__('obstacle_avoidance')
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan', 
            self.lidar_callback,
            10)
        self.window_name='LidarScan'
        cv2.namedWindow(self.window_name,cv2.WINDOW_NORMAL)

        self.mess=Bool(data=False)
        self.publisher2= self.create_publisher(String, "/nodeStatus",10)

        self.message = Twist(linear=Vector3(x=0.0,y=0.0,z=0.0),angular=Vector3(x=0.0,y=0.0,z=0.0))
        self.publisher=self.create_publisher(Twist,'/cmd_vel',10)
        
    # def lidar_callback(self, msg):
    #     import time
    #     img_size = 600
    #     scale= img_size/3
    #     image = np.zeros((img_size, img_size, 3), dtype=np.uint8)

    #     center = img_size // 2
    #     scale = 100  # pixels par mètre
    #     cv2.circle(image, (center, center), 2, (255, 0, 0), -1)

    #     distSeuil=1 # à ajuster 30cm

    #     for i,r  in  enumerate(msg.ranges):

    #         if msg.ranges[i] < distSeuil:
            
    #             # afficher
    #             angle = msg.angle_min + i * msg.angle_increment
    #             x = r * np.cos(angle)
    #             y = r * np.sin(angle)
    #             px = int(center + x * scale)
    #             py = int(center - y * scale)
    #             if 0 <= px < img_size and 0 <= py < img_size:
    #                 cv2.circle(image, (px, py), 2, (0, 255, 0), -1)



  
    #     self.message.angular.z=0.5
    #     self.publisher.publish(self.message)

    #     time.sleep(1)

    #     self.message.angular.z=-0.5
    #     self.publisher.publish(self.message)

     
    #     cv2.imshow(self.window_name, image)
    #     cv2.waitKey(1)





    def lidar_callback(self, msg):

        img_size = 600
        scale= img_size/3
        image = np.zeros((img_size, img_size, 3), dtype=np.uint8)

        center = img_size // 2
        # scale = 100  # pixels par mètre
        cv2.circle(image, (center, center), 2, (255, 0, 0), -1)

   

        for i,r  in  enumerate(msg.ranges):

            if not np.isinf(r):
            
                # afficher
                angle = msg.angle_min + i * msg.angle_increment
                x = r * np.cos(angle)
                y = r * np.sin(angle)
                px = int(center + x * scale)
                py = int(center - y * scale)
                if 0 <= px < img_size and 0 <= py < img_size:
                    cv2.circle(image, (px, py), 2, (0, 255, 0), -1)


     
        cv2.imshow(self.window_name, image)
        cv2.waitKey(1)

        minimumrange= min(msg.ranges)

        if minimumrange < 0.3:
            self.mess = "NodeObstacle"
            self.message.linear.x=-0.2
            self.message.angular.z=0.1
        self.publisher2.publish(self.mess)
        self.publisher.publish(self.message)

    # def lidar_callback(self, msg):

    #     img_size = 600
    #     scale= img_size/3
    #     image = np.zeros((img_size, img_size, 3), dtype=np.uint8)

    #     center = img_size // 2
    #     # scale = 100  # pixels par mètre
    #     cv2.circle(image, (center, center), 2, (255, 0, 0), -1)

   

    #     for i,r  in  enumerate(msg.ranges):

    #         if not np.isinf(r):
            
    #             # afficher
    #             angle = msg.angle_min + i * msg.angle_increment
    #             x = r * np.cos(angle)
    #             y = r * np.sin(angle)
    #             px = int(center + x * scale)
    #             py = int(center - y * scale)
    #             if 0 <= px < img_size and 0 <= py < img_size:
    #                 cv2.circle(image, (px, py), 2, (0, 255, 0), -1)


     
    #     cv2.imshow(self.window_name, image)
    #     cv2.waitKey(1)















    #     dist_backward=0.2
    #     distLimitMax=0.35
    #     distLmitMin=0.25
    #     goBack=False
    #     BienPlace = False
    #     TropHaut=False
    #     TropBas=False
    #     listdistfront=[]
    #     listdistRight=[]
    #     offset=1

    #     angle_current=msg.angle_min
    #     for i,r in enumerate(msg.ranges):
    #         angle_current+= i*msg.angle_increment
    #         if angle_current<offset or angle_current < msg.angle_max-offset:
    #             listdistfront.append(r)
    #             print(r)

    #         # elif angle_current > 5 and angle_current < 6.5:
    #             # listdistRight.append(r)
    #             # print("o a droite")
               
    
    #     print(msg.angle_max)
    #     minimumrange= min(listdistfront)

    #     # minDroite= min(listdistRight)
    #     # for i,r in enumerate(msg.ranges):
    #     #     if msg.ranges[i] < dist_backward:
    #     #         goBack=True
    #     #     elif msg.ranges[i] > distLmitMin and msg.ranges[i] < distLimitMax:
    #     #         BienPlace= True
    #     #     elif msg.ranges[i] < distLmitMin and msg.ranges[i]>dist_backward:
    #     #         TropBas= True
    #     #     elif msg.ranges[i] > distLimitMax:
    #     #         TropHaut=True
        
    #     if minimumrange < 0.4:
    #     elif minimumrange< dist_backward:
    #         goBack=True
    #     elif minimumrange> distLmitMin and minimumrange < distLimitMax:
    #         BienPlace= True
    #     elif minimumrange< distLmitMin and minimumrange>dist_backward:
    #         TropBas= True
    #     elif minimumrange> distLimitMax:
    #         TropHaut=True
            
    #     if goBack:
    #         self.message.linear.x=-0.1
    #         self.message.angular.z=0.0
    #         self.get_logger().info("je recule")

    #     elif BienPlace:
    #         self.message.linear.x=0.1
    #         self.message.angular.z=0.0
    #         self.get_logger().info("bien place")

    #     elif TropBas :
    #         self.message.linear.x=0.1
    #         self.message.angular.z=-0.3
    #         self.get_logger().info("trop pres")

    #     elif TropHaut:
    #         self.message.linear.x=0.1
    #         self.message.angular.z=0.3
    #         self.get_logger().info("trop loin")
        
    #     self.publisher.publish(self.message)


        
   


    # for i, r in enumerate(msg.ranges):
    #         if np.isinf(r) or np.isnan(r):
    #             continue
    #         angle = msg.angle_min + i * msg.angle_increment
    #         x = r * np.cos(angle)
    #         y = r * np.sin(angle)

    #         px = int(center + x * scale)
    #         py = int(center - y * scale)

    #         if 0 <= px < img_size and 0 <= py < img_size:
    #             cv2.circle(image, (px, py), 2, (0, 255, 0), -1)

def main(args=None):

    rclpy.init(args=args)
    obstacle_avoidance = ObstacleAvoidance()
    rclpy.spin(obstacle_avoidance)
    obstacle_avoidance.destroy_node()
    rclpy.shutdown()



if __name__ == '__main__':
    main()









