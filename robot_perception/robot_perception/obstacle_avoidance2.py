# import rclpy
# from rclpy.node import Node  # pour faire des noeuds
# from sensor_msgs.msg import LaserScan
# from std_msgs.msg import String
# import cv2  # bibliothèque de opencv
# import numpy as np
# from geometry_msgs.msg import Twist
# from geometry_msgs.msg import Vector3

# class ObstacleAvoidance(Node):
#     def __init__(self):
#         super().__init__('obstacle_avoidance')

#         self.subscription = self.create_subscription(
#             LaserScan,
#             '/scan', 
#             self.lidar_callback,
#             10
#         )

#         # Initialisation des variables
#         self.img_size = 600
#         self.scale = 500
#         self.image = np.zeros((self.img_size, self.img_size, 3), dtype=np.uint8)  # Réinitialiser l'image
#         self.center = self.img_size // 2

#         self.nodeStatusObstacle= String(data="Off")
#         self.publisher2 = self.create_publisher(String, "/node_statusObstacle", 10)
#         # self.subscription2= self.create_subscription(String,"/node_statusFollow",self.Obstacle,10)


#         self.publisher3 = self.create_publisher(
#             Twist,
#             '/cmd_vel',
#             10)

#         self.cmd = Twist()
#         self.compteur=0

#         self.state="IDLE"
        

    

#     def lidar_callback(self, msg):
        
#         dist=0.2

#         front = min(min(msg.ranges[0:10]), min(msg.ranges[-10:]))  # Zone avant du robot
    
#         if front < dist :
#             self.state="Detected"
#             # self.nodeStatusObstacle.data="On"

#         # else:
#         #     self.nodeStatusObstacle.data="Off"

#         if self.state="Detected":
#             self.nodeStatusObstacle.data="On"
#         elif self.state="Avoidance":
#             self.nodeStatusObstacle.data="On"


#         self.publisher2.publish(self.nodeStatusObstacle)


    
#         if self.nodeStatusObstacle.data == "On":
#             self.window_name = 'LidarScan'
#             cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
#             cv2.startWindowThread()

#             listdistfront = []
            
#             # # range proche
#             # nearlyRange=[]

#             # # reste 

#             # otherRange=[]
            
#             for i, r in enumerate(msg.ranges):
#                 angle_current =msg.angle_min + i * msg.angle_increment
#                 # self.get_logger().info(str(angle_current))
#                 if angle_current < 3.18 and not np.isinf(r):
#                     listdistfront.append(r)
#                     print("Il y a un obstacle droit devant")



#             # Calcul des positions des obstacles
#             xfront, yfront = self.calculPosLidar(listdistfront, msg.angle_min, msg.angle_increment)
#             x, y = self.calculPosLidar(msg.ranges, msg.angle_min, msg.angle_increment)  
#             # xn,yn=self.calculPosLidar(nearlyRange,nearlyAnglemi,msg)
#             # Réinitialisation de l'image à chaque nouvelle itération
#             self.image = np.zeros((self.img_size, self.img_size, 3), dtype=np.uint8)

#             # Traçage des points Lidar
#             # self.TraceLidar(xfront, yfront, (255, 255, 0))  # Données en face
#             self.TraceLidar(x, y, (255, 0, 0))  # Toutes les données
#             self.TraceLidar([self.center], [self.center], (0, 0, 255), 5)  # Position du robot

#             # Affichage de l'image
#             cv2.imshow(self.window_name, self.image)
#             cv2.waitKey(1)


            

#             if front <= 0.05:
#                 # self.compteur=1
#                 self.go_backward()
#             elif front > 0.1:
#                 self.go_forward()
#                 # self.turn_left()

#             self.publisher3.publish(self.cmd)

                

#     def go_forward(self):
#         self.cmd.linear.x = 0.5
#         self.cmd.angular.z = 0.0
    
#     def go_backward(self):
#         self.cmd.linear.x = -0.2
#         self.cmd.angular.z = 0.0

#     def turn_left(self):
#         self.cmd.linear.x = 0.0
#         self.cmd.angular.z = 0.5      

#     def calculPosLidar(self, listRange, angleMin, angleIncrement):
#         """
#         Calcule la position (px, py) d'un point Lidar à partir de la distance et de l'angle.
#         """
#         xl = []
#         yl = []
#         for i, r in enumerate(listRange):
#             if not np.isinf(r):
#                 angle = angleMin + i * angleIncrement
#                 x = r * np.cos(angle)
#                 y = r * np.sin(angle)
#                 px = int(self.center + x * self.scale)
#                 py = int(self.center - y * self.scale)
#                 # Vérification pour s'assurer que px et py sont dans les limites de l'image
#                 if 0 <= px < self.img_size and 0 <= py < self.img_size:
#                     xl.append(px)
#                     yl.append(py)
#         return xl, yl

#     def TraceLidar(self, x, y, color, size=2):
#         """
#         Trace les points Lidar sur l'image.
#         """
#         for i in range(len(x)):
#             cv2.circle(self.image, (x[i], y[i]), size, color, -1)
        

# def main(args=None):
#     rclpy.init(args=args)
#     obstacle_avoidance = ObstacleAvoidance()
#     try:
#         rclpy.spin(obstacle_avoidance)
#     except KeyboardInterrupt:
#         pass
  
#     obstacle_avoidance.destroy_node()
#     cv2.destroyAllWindows()  # Fermer proprement la fenêtre OpenCV
#     rclpy.shutdown()


# if __name__ == '__main__':
#     main()



import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
import cv2
import numpy as np
from geometry_msgs.msg import Twist

class ObstacleAvoidance(Node):
    def __init__(self):
        super().__init__('obstacle_avoidance')

        self.subscription = self.create_subscription(LaserScan, '/scan', self.lidar_callback, 10)
        self.publisher_status = self.create_publisher(String, "/node_statusObstacle", 10)
        self.publisher_cmd = self.create_publisher(Twist, '/cmd_vel', 10)

        self.state = "IDLE"
        self.img_size = 600
        self.scale = 500
        self.center = self.img_size // 2
        self.image = np.zeros((self.img_size, self.img_size, 3), dtype=np.uint8)
        self.cmd = Twist()

        self.start_time = self.get_clock().now()

        self.timer = self.create_timer(0.1, self.control_loop)

    def lidar_callback(self, msg):
        self.front = min(min(msg.ranges[0:10]), min(msg.ranges[-10:]))
        self.ranges = msg.ranges
        self.angle_min = msg.angle_min
        self.angle_increment = msg.angle_increment

    def control_loop(self):
        current_time = self.get_clock().now()
        status_msg = String()

        if self.state == "IDLE":
            status_msg.data = "Off"
            if hasattr(self, 'front') and self.front < 0.3:
                self.state = "AVOIDING"
                self.start_time = current_time
                self.get_logger().info("Obstacle détecté")

        # elif self.state == "DETECTED":
        #     status_msg.data = "On"
        #     # self.go_backward()
        #     if (current_time - self.start_time).nanoseconds > 1e9: # a changer selon la realilé le temps de manip
        #         self.state = "AVOIDING"
        #         self.start_time = current_time
        #         self.get_logger().info("Évitement gauche")

        elif self.state == "AVOIDING":
            status_msg.data = "On"
            self.turn_left()
            if (current_time - self.start_time).nanoseconds > 4e9:
                self.state = "RETURNING"
                self.start_time = current_time
                self.get_logger().info("Retour à la ligne")

        elif self.state == "RETURNING":
            status_msg.data = "On"
            self.turn_right()
            if (current_time - self.start_time).nanoseconds > 9e9:
                self.state = "IDLE"
                self.get_logger().info("Retour terminé")

        # Publish movement command only if not IDLE
        if self.state != "IDLE":
            self.publisher_cmd.publish(self.cmd)

        self.publisher_status.publish(status_msg)

        # Visualisation
        if hasattr(self, 'ranges'):
            self.image = np.zeros((self.img_size, self.img_size, 3), dtype=np.uint8)
            x, y = self.calculPosLidar(self.ranges, self.angle_min, self.angle_increment)
            self.TraceLidar(x, y, (255, 0, 0))
            self.TraceLidar([self.center], [self.center], (0, 0, 255), 5)
            cv2.imshow("LidarScan", self.image)
            cv2.waitKey(1)

    def go_forward(self):
        self.cmd.linear.x = 0.1
        self.cmd.angular.z = 0.0

    def go_backward(self):
        self.cmd.linear.x = -0.2
        self.cmd.angular.z = 0.0

    def turn_left(self):
        self.cmd.linear.x = 0.05
        self.cmd.angular.z = 0.2

    def turn_right(self):
        self.cmd.linear.x = 0.05
        self.cmd.angular.z = -0.2

    def calculPosLidar(self, listRange, angleMin, angleIncrement):
        xl, yl = [], []
        for i, r in enumerate(listRange):
            if not np.isinf(r):
                angle = angleMin + i * angleIncrement
                x = r * np.cos(angle)
                y = r * np.sin(angle)
                px = int(self.center + x * self.scale)
                py = int(self.center - y * self.scale)
                if 0 <= px < self.img_size and 0 <= py < self.img_size:
                    xl.append(px)
                    yl.append(py)
        return xl, yl

    def TraceLidar(self, x, y, color, size=2):
        for i in range(len(x)):
            cv2.circle(self.image, (x[i], y[i]), size, color, -1)

def main(args=None):
    rclpy.init(args=args)
    obstacle_avoidance = ObstacleAvoidance()
    try:
        rclpy.spin(obstacle_avoidance)
    except KeyboardInterrupt:
        pass
    obstacle_avoidance.destroy_node()
    cv2.destroyAllWindows()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
