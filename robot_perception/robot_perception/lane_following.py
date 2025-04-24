import rclpy
from rclpy.node import Node # pour faire des noeuds 
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2 # bibliothèque de opencv
import numpy as np
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Point
from std_msgs.msg import String

# from std.msgs.msg import Bool
class LaneFollowing(Node):
    def __init__(self):
        super().__init__('lane_following')

        self.subscription2= self.create_subscription(String,"/node_statusObstacle",self.RecoitNodeStatus,10)
        self.nodeStatusObstacle="Off"


        self.subscription = self.create_subscription( # recoit des images du turtlebot
            Image,
            '/image_raw', # /image_raw/compressed meilleur qualité
            self.listener_callback,
            10)
        self.br = CvBridge() # pont entre cv2 (utilisant numpy) et ros2 

        
        self.message=Point(x=0.0,y=0.0,z=0.0)
        self.publisher = self.create_publisher(Point,'/target',10)

        # self.nodeStatusFollow=String(data1="On")
        # self.publisher2= self.create_publisher(String,"node_statusFollow",10)
        
   

    def RecoitNodeStatus(self,msg):
        self.nodeStatusObstacle= msg.data
        self.get_logger().info(self.nodeStatusObstacle)
     
    def listener_callback(self, data):

        cx_target = 0.   # Par défaut au centre de l'image
        cy_target = 0. # Par défaut au centre de l'image
        if self.nodeStatusObstacle== "Off":

            # change node status et envoie

            

            # recevoir la video
            # self.get_logger().info('Receiving video frame')
            # get image 
            current_frame = self.br.imgmsg_to_cv2(data, desired_encoding='bgr8')
            #  passage en modele HSV, teint, saturation et luminosité
            current_frame_hsv=cv2.cvtColor(current_frame,cv2.COLOR_BGR2HSV)
            # découper l'image pour garder que le bas
            self.height, self.width = current_frame.shape[:2] # 480, 600 px
            current_frame_crop=current_frame[int(1/2*self.height):self.height,:]
            current_frame_hsv_crop=current_frame_hsv[int(1/2*self.height):self.height,:]

            #___ ROUGE ___
            # range pour detecter le rouge
            lower_red=np.array([170,100,100])
            upper_red=np.array([185,255,255])
            # lower_red=np.array([175,30,150])
            # upper_red=np.array([180,80,160])
            # masque binaire pour  isoler une couleur
            mask_red = cv2.inRange(current_frame_hsv_crop,lower_red,upper_red) # parcours chaque pixel de l'image et regarde si cela convient
            # garde juste le résultat
            result_red= cv2.bitwise_and(current_frame_crop,current_frame_crop, mask=mask_red) 
            
            #___ VERT___
            # range pour detecter le vert
            lower_green=np.array([30,40,40])
            upper_green=np.array([90,255,255])
            # lower_green=np.array([82,20,140])
            # upper_green=np.array([95,80,200])
            # masque binaire pour  isoler une couleur
            mask_green = cv2.inRange(current_frame_hsv_crop,lower_green,upper_green) # parcours chaque pixel de l'image et regarde si cela convient
            # garde juste le résultat
            result_green= cv2.bitwise_and(current_frame_crop,current_frame_crop, mask=mask_green) 

            # calcul le moment du mask red
            M_red= cv2.moments(mask_red)
            M_green= cv2.moments(mask_green)

            test=1
            if M_red['m00'] <0 and M_green['m00']<0:

                print(" je ne vois rien")

            

            # elif M_red['m00'] >0 and M_green['m00']>0: # m00 represente l'aire de l'objet vérifie donc qu'il y'a bien une couleur 
            #     # baryecentres
            #     print("je vois les deux")
            #     cx_red=int(M_red['m10']/M_red['m00']) # centre en x
            #     cy_red=int(M_red['m01']/M_red['m00']) # en y

            #     cx_green=int(M_green['m10']/M_green['m00']) # centre en x
            #     cy_green=int(M_green['m01']/M_green['m00']) # en y
            #     # Dessiner
            #     cv2.circle(current_frame_crop,(cx_red,cy_red), 5, (0,255,0),2)
            #     cv2.circle(current_frame_crop,(cx_green,cy_green), 5, (0,255,0),2)
            #     # calcul du point à faire suivre
            #     cx_target= (cx_red+cx_green)//2
            #     cy_target= (cy_red + cy_green)//2
            #     # dessiner
            #     cv2.rectangle(current_frame_crop,(cx_target-5,cy_target-5),(cx_target+5,cy_target+5), (0,255,0),2)
                

            
            elif M_red['m00']!=0:
                print(" Je ne vois plus la ligne verte! Je suis la rouge")
    

                cx_red = int(M_red['m10'] / M_red['m00'])  # Centre en x de la ligne rouge
                cy_red = int(M_red['m01'] / M_red['m00'])  # Centre en y de la ligne rouge

                # Estimation d'un nouveau point cible en fonction uniquement de la ligne rouge
                contours, _ = cv2.findContours(mask_red, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                
                if contours:
                    # Prendre le plus grand contour (la ligne rouge)
                    contour = max(contours, key=cv2.contourArea)
                    
                    # Approximation de la courbe rouge par une droite
                    [vx, vy, x0, y0] = cv2.fitLine(contour, cv2.DIST_L2, 0, 0.01, 0.01)

            
                    # Calcul des deux points à partir des coordonnées du vecteur directeur
                    # On choisit un facteur d'échelle pour obtenir une ligne visible
                    line_length = 300  # Longueur de la ligne que nous voulons dessiner
                    pt1_x = int(x0 - line_length * vx)
                    pt1_y = int(y0 - line_length * vy)
                    pt2_x = int(x0 + line_length * vx)
                    pt2_y = int(y0 + line_length * vy)

                    # Dessiner la ligne sur l'image
                    cv2.line(current_frame_crop, (pt1_x, pt1_y), (pt2_x, pt2_y), (255, 0, 0), 2)  # Bleu, épaisseur 2

                    # Calcule le vecteur normal à la droite
                    norm = np.sqrt(vx**2 + vy**2)
                    nx = -vy / norm  # x du vecteur normal
                    ny = vx / norm   # y du vecteur normal

                    if nx>0:
                        nx=-nx
                        ny=-ny
                    # pt1_x = int(x0 - line_length * nx)
                    # pt1_y = int(y0 - line_length * ny)
                    pt1_x=int(x0)
                    pt1_y=int(y0)
                    pt2_x = int(x0 + line_length * nx)
                    pt2_y = int(y0 + line_length * ny)

                    cv2.line(current_frame_crop, (pt1_x, pt1_y), (pt2_x, pt2_y), (255, 0, 0), 2)  # Bleu, épaisseur 2


                    # d =300 # longeur de la ligne normale
                    # Calcul du point cible, décalé de 15 pixels vers la gauche (selon la direction de la droite)
                    # cx_target = cx_red - 200  # Décalage du point cible sur l'axe x
                    # cy_target = int(slope * cx_target + intercept)  # Calcul de l'ordonnée correspondante
                    # cx_target = int(cx_red +d* nx)
                    # cy_target = int(cy_red +d * ny)
                    cx_target= pt2_x
                    cy_target=pt2_y
                    # Dessiner le point cible
                    cv2.circle(current_frame_crop, (cx_target, cy_target), 5, (0, 255, 0), 2)  # Vert, épaisseur 2



            # elif M_red['m00'] ==0 and M_green['m00']!=0:
            #     print(" Je ne vois plus la ligne rouge! Je suis la verte")
    

            #     cx_green = int(M_green['m10'] / M_green['m00'])  # Centre en x de la ligne rouge
            #     cy_green = int(M_green['m01'] / M_green['m00'])  # Centre en y de la ligne rouge

            #     # Estimation d'un nouveau point cible en fonction uniquement de la ligne rouge
            #     contours, _ = cv2.findContours(mask_green, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                
            #     if contours:
            #         # Prendre le plus grand contour (la ligne rouge)
            #         contour = max(contours, key=cv2.contourArea)
                    
            #         # Approximation de la courbe rouge par une droite
            #         [vx, vy, x0, y0] = cv2.fitLine(contour, cv2.DIST_L2, 0, 0.01, 0.01)

            #         # Calcul de la direction de la ligne (pente de la droite)
            #         slope = vy / vx  # pente de la droite
            #         intercept = y0 - slope * x0  # ordonnée à l'origine

            #         # Calcul des deux points à partir des coordonnées du vecteur directeur
            #         # On choisit un facteur d'échelle pour obtenir une ligne visible
            #         line_length = 200  # Longueur de la ligne que nous voulons dessiner
            #         pt1_x = int(x0 - line_length * vx)
            #         pt1_y = int(y0 - line_length * vy)
            #         pt2_x = int(x0 + line_length * vx)
            #         pt2_y = int(y0 + line_length * vy)

            #         # Dessiner la ligne sur l'image
            #         cv2.line(current_frame_crop, (pt1_x, pt1_y), (pt2_x, pt2_y), (255, 0, 0), 2)  # Bleu, épaisseur 2

            #         # Calcule le vecteur normal à la droite
            #         norm = np.sqrt(vx**2 + vy**2)
            #         nx = -vy / norm  # x du vecteur normal
            #         ny = vx / norm   # y du vecteur normal
            #         pt1_x = int(x0 - line_length * nx)
            #         pt1_y = int(y0 - line_length * ny)
            #         pt2_x = int(x0 + line_length * nx)
            #         pt2_y = int(y0 + line_length * ny)
            #         cv2.line(current_frame_crop, (pt1_x, pt1_y), (pt2_x, pt2_y), (255, 0, 0), 2)  # Bleu, épaisseur 2


            #         d =300
            #         # Calcul du point cible, décalé de 15 pixels vers la gauche (selon la direction de la droite)
            #         # cx_target = cx_red - 200  # Décalage du point cible sur l'axe x
            #         # cy_target = int(slope * cx_target + intercept)  # Calcul de l'ordonnée correspondante
            #         cx_target = int(cx_green + d* nx)
            #         cy_target = int(cy_green + d * ny)
            #         # Dessiner le point cible
            #         cv2.circle(current_frame_crop, (cx_target, cy_target), 5, (0, 255, 0), 2)  # Vert, épaisseur 2

            # else:
            #     print("je ne vois plus rien. je stop")
            #     cx_target = (self.width // 2)
            #     cy_target = (self.height // 2)



            # # Calcul distance entre la cible et le centre de l'image
            # error_x = cx_target - (self.width // 2)
            # error_y = cy_target - (self.height // 2)

            
            # #  pour tourner 
            # self.message.angular.z= - float(error_x)/100.0 # - car si error.x >0 la cible est à droite donc angular.z négatif pour tourenr vers la gauche. 100 pour normalisé on peut baisser pour tourner plus vite
            
            # # pour se déplacer tout droit
            # if abs(error_y)>0.01:
            #     self.message.linear.x=0.05
            # else:
            #     self.message.linear.x=0.0
            
            # self.publisher.publish(self.message)
            # affiche
            cv2.imshow("camera", current_frame)
            cv2.waitKey(1)
        else:
            cx_target=0.
            cy_target=0.

        self.message.x=float(cx_target)
        self.message.y= float(cy_target)
        self.publisher.publish(self.message)

            





def main(args=None):

    rclpy.init(args=args)
    lane_following = LaneFollowing()
    rclpy.spin(lane_following)
    lane_following.destroy_node()
    rclpy.shutdown()



if __name__ == '__main__':
    main()








