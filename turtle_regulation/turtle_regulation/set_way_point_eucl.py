#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist  # Message pour commander le mouvement
from math import sqrt

class TurtlePoseSubscriber(Node):
    def __init__(self, x, y, Kpl, distance_tolerance):
        super().__init__('set_way_point_eucl')
        
        # Souscription au topic "turtle1/pose"
        self.subscription = self.create_subscription(
            Pose,
            'turtle1/pose',  # Correction du nom du topic
            self.pose_callback,
            10  # Taille de la file d'attente
        )

        # Création d'un éditeur pour envoyer des commandes de déplacement
        self.publisher = self.create_publisher(Twist, 'turtle1/cmd_vel', 10)
        self.mypublisher = self.create_publisher(Bool, 'turtle1/is_moving', 10)
        
        # Initialisation des attributs
        self.initialize_attributes(x, y, Kpl, distance_tolerance)

    def initialize_attributes(self, x, y, Kpl, distance_tolerance):
        """ Initialisation de la position et du waypoint """
        self.turtle_pose = Pose()  # Position actuelle de la tortue
        self.waypoint = Pose()  # Position cible
        self.waypoint.x = x
        self.waypoint.y = y
        self.Kpl = Kpl
        self.distance_tolerance = distance_tolerance

    def pose_callback(self, msg):
        """ Mise à jour de la position et déclenchement du déplacement """
        self.turtle_pose = msg
        self.get_logger().info(f'Nouvelle position : x={msg.x}, y={msg.y}, theta={msg.theta}')
        
        # Déplacement automatique vers le waypoint
        self.move_towards_waypoint()
        self.verif_move()

    def move_towards_waypoint(self):
        """ Déplace la tortue vers le waypoint """
        cmd = Twist()

        el = sqrt((self.waypoint.y - self.turtle_pose.y)**2 + (self.waypoint.x - self.turtle_pose.x)**2)

        if el > self.distance_tolerance:

            v = self.Kpl * el
            cmd.linear.x = v
            self.publisher.publish(cmd)  # La tortue bouge seulement si elle est loin du waypoint

        else:

            self.get_logger().info("Tortue arrivée au waypoint, arrêt du mouvement !")

    def verif_move(self):
        """ Publie un message Bool indiquant si la tortue bouge ou non """
        msg = Bool()

        # Calcul de l'erreur linéaire
        el = sqrt((self.waypoint.y - self.turtle_pose.y)**2 + (self.waypoint.x - self.turtle_pose.x)**2)

        # Vérification du seuil avant de publier
        msg.data = el > self.distance_tolerance  # True si el > distance_tolerance, False sinon

        self.mypublisher.publish(msg)
        self.get_logger().info(f"Tortue en mouvement : {msg.data}")

def main(args=None):
    rclpy.init(args=args)
    
    # Définition du waypoint
    x_waypoint = 7.0
    y_waypoint = 7.0
    Kpl = 1.0
    distance_tolerance = 2.0

    node = TurtlePoseSubscriber(x_waypoint, y_waypoint, Kpl, distance_tolerance)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()