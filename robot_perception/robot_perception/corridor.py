import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Point
from std_msgs.msg import String
import numpy as np

class CorridorCentering(Node):
    def __init__(self):
        super().__init__('corridor_centering')

        self.subscription = self.create_subscription(LaserScan, '/scan', self.lidar_callback, 10)
        self.publisher = self.create_publisher(Point, '/target', 10)
        self.status_publisher = self.create_publisher(String, '/node_statusObstacle', 10)

        self.timer = self.create_timer(0.5, self.publish_status)
        self.node_status = String()
        self.node_status.data = "On"

    def publish_status(self):
        self.status_publisher.publish(self.node_status)

    def lidar_callback(self, msg):
        ranges = np.array(msg.ranges)
        ranges[ranges == 0.0] = np.nan  # Ignore 0.0 readings (pas d'info)

        # Plage d'angles à gauche et à droite (en degrés convertis en indices)
        angle_min = msg.angle_min
        angle_increment = msg.angle_increment
        num_ranges = len(msg.ranges)
        angles = np.arange(num_ranges) * angle_increment + angle_min
        angles_deg = np.degrees(angles)

        # Zone à gauche : entre +60° et +120°
        left_indices = np.where((angles_deg > 60) & (angles_deg < 120))
        # Zone à droite : entre -60° et -120°
        right_indices = np.where((angles_deg < -60) & (angles_deg > -120))

        left_distances = ranges[left_indices]
        right_distances = ranges[right_indices]

        # Moyennes des distances (si dispo)
        avg_left = np.nanmean(left_distances) if left_distances.size > 0 else np.nan
        avg_right = np.nanmean(right_distances) if right_distances.size > 0 else np.nan

        self.get_logger().info(f"Left: {avg_left:.2f} m | Right: {avg_right:.2f} m")

        # Centrage
        if not np.isnan(avg_left) and not np.isnan(avg_right):
            error = avg_left - avg_right  # positif = trop près du mur droit
            correction = -error * 100  # facteur de correction (peut être ajusté)

            point_msg = Point()
            point_msg.x = float(self.get_clock().now().nanoseconds % 100000)  # Juste pour changer à chaque boucle
            point_msg.y = correction  # Y utilisé comme écart latéral
            point_msg.z = 0.0

            self.publisher.publish(point_msg)
        else:
            self.get_logger().info("Pas assez de données pour se centrer")

def main(args=None):
    rclpy.init(args=args)
    node = CorridorCentering()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
