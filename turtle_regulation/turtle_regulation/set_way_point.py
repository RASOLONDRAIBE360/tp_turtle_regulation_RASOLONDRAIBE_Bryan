#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist  # Message pour commander le mouvement
from math import atan2, atan, tan

class TurtlePoseSubscriber(Node):
    def __init__(self, x, y, Kp):
        super().__init__('set_way_point')
        
        # Souscription au topic "turtle1/pose"
        self.subscription = self.create_subscription(
            Pose,
            'turtle1/pose',  # Correction du nom du topic
            self.pose_callback,
            10  # Taille de la file d'attente
        )

        # Création d'un éditeur pour envoyer des commandes de déplacement
        self.publisher = self.create_publisher(Twist, 'turtle1/cmd_vel', 10)
        
        # Initialisation des attributs
        self.initialize_attributes(x, y, Kp)

    def initialize_attributes(self, x, y, Kp):
        """ Initialisation de la position et du waypoint """
        self.turtle_pose = Pose()  # Position actuelle de la tortue
        self.waypoint = Pose()  # Position cible
        self.waypoint.x = x
        self.waypoint.y = y
        self.Kp = Kp

    def pose_callback(self, msg):
        """ Mise à jour de la position et déclenchement du déplacement """
        self.turtle_pose = msg
        self.get_logger().info(f'Nouvelle position : x={msg.x}, y={msg.y}, theta={msg.theta}')
        
        # Déplacement automatique vers le waypoint
        self.move_towards_waypoint()

    def move_towards_waypoint(self):
        """ Déplace la tortue vers le waypoint """
        cmd = Twist()

        # Orientation vers le waypoint
        cmd.angular.z = atan2(self.waypoint.y - self.turtle_pose.y, self.waypoint.x - self.turtle_pose.x)

        e = atan(tan(cmd.angular.z - self.turtle_pose.theta)/2)

        u = self.Kp * e

        cmd.angular.z = u
        # Déplacement en ligne droite vers le waypoint
        cmd.linear.x = 0.0  # Correction : la tortue avance maintenant

        # Publier la commande de mouvement
        self.publisher.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    
    # Définition du waypoint
    x_waypoint = 7.0
    y_waypoint = 7.0
    Kp = 0.2

    node = TurtlePoseSubscriber(x_waypoint, y_waypoint, Kp)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()