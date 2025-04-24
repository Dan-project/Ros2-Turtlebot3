import rclpy
from rclpy.node import Node  # pour faire des noeuds
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
import cv2  # bibliothèque de opencv
import numpy as np
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3

class ObstacleAvoidance(Node):
    def __init__(self):
        super().__init__('obstacle_avoidance')

        self.subscription = self.create_subscription(
            LaserScan,
            '/scan', 
            self.lidar_callback,
            10
        )

        # Initialisation des variables
        self.img_size = 600
        self.scale = 500
        self.image = np.zeros((self.img_size, self.img_size, 3), dtype=np.uint8)  # Réinitialiser l'image
        self.center = self.img_size // 2

        self.nodeStatusObstacle= String(data="Off")
        self.publisher2 = self.create_publisher(String, "/node_statusObstacle", 10)
        # self.subscription2= self.create_subscription(String,"/node_statusFollow",self.Obstacle,10)


        self.publisher3 = self.create_publisher(
            Twist,
            '/cmd_vel',
            10)

        self.cmd = Twist()
        self.compteur=0

    
        

    

    def lidar_callback(self, msg):
        
        dist=0.1
    
        if np.min(msg.ranges)<dist or self.compteur==1:
            self.nodeStatusObstacle.data="On"
        else:
            self.nodeStatusObstacle.data="Off"
        self.publisher2.publish(self.nodeStatusObstacle)


    
        if self.nodeStatusObstacle.data == "On":
            self.window_name = 'LidarScan'
            cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
            cv2.startWindowThread()

            listdistfront = []
            
            # # range proche
            # nearlyRange=[]

            # # reste 

            # otherRange=[]
            
            for i, r in enumerate(msg.ranges):
                angle_current =msg.angle_min + i * msg.angle_increment
                # self.get_logger().info(str(angle_current))
                if angle_current < 3.18 and not np.isinf(r):
                    listdistfront.append(r)
                    print("Il y a un obstacle droit devant")



            # Calcul des positions des obstacles
            xfront, yfront = self.calculPosLidar(listdistfront, msg.angle_min, msg.angle_increment)
            x, y = self.calculPosLidar(msg.ranges, msg.angle_min, msg.angle_increment)  
            # xn,yn=self.calculPosLidar(nearlyRange,nearlyAnglemi,msg)
            # Réinitialisation de l'image à chaque nouvelle itération
            self.image = np.zeros((self.img_size, self.img_size, 3), dtype=np.uint8)

            # Traçage des points Lidar
            # self.TraceLidar(xfront, yfront, (255, 255, 0))  # Données en face
            self.TraceLidar(x, y, (255, 0, 0))  # Toutes les données
            self.TraceLidar([self.center], [self.center], (0, 0, 255), 5)  # Position du robot

            # Affichage de l'image
            cv2.imshow(self.window_name, self.image)
            cv2.waitKey(1)


            front = min(min(msg.ranges[0:10]), min(msg.ranges[-10:]))  # Zone avant du robot

            if front <=0.1:
                # self.compteur=1
                self.go_backward()
            elif front >0.1 and front <=0.11:
                self.go_forward()
                self.turn_left()

            self.publisher3.publish(self.cmd)

                

    def go_forward(self):
        self.cmd.linear.x = 0.2
        self.cmd.angular.z = 0.0
    
    def go_backward(self):
        self.cmd.linear.x = -0.2
        self.cmd.angular.z = 0.0

    def turn_left(self):
        self.cmd.linear.x = 0.0
        self.cmd.angular.z = 0.3       

    def calculPosLidar(self, listRange, angleMin, angleIncrement):
        """
        Calcule la position (px, py) d'un point Lidar à partir de la distance et de l'angle.
        """
        xl = []
        yl = []
        for i, r in enumerate(listRange):
            if not np.isinf(r):
                angle = angleMin + i * angleIncrement
                x = r * np.cos(angle)
                y = r * np.sin(angle)
                px = int(self.center + x * self.scale)
                py = int(self.center - y * self.scale)
                # Vérification pour s'assurer que px et py sont dans les limites de l'image
                if 0 <= px < self.img_size and 0 <= py < self.img_size:
                    xl.append(px)
                    yl.append(py)
        return xl, yl

    def TraceLidar(self, x, y, color, size=2):
        """
        Trace les points Lidar sur l'image.
        """
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
    cv2.destroyAllWindows()  # Fermer proprement la fenêtre OpenCV
    rclpy.shutdown()


if __name__ == '__main__':
    main()
