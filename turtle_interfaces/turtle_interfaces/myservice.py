import rclpy
from rclpy.node import Node
from turtle_interfaces.srv import SetWayPoint

class WayPointServer(Node):
    def __init__(self):
        super().__init__('myservice')
        self.srv = self.create_service(SetWayPoint, 'set_waypoint_service', self.handle_waypoint_request)
        self.get_logger().info("Service WayPoint prêt !")

    def handle_waypoint_request(self, request, response):
        # Logique pour gérer la position du waypoint
        self.get_logger().info(f"Waypoint reçu : x={request.x}, y={request.y}")

        # Simuler une validation de déplacement
        response.res = True  # Envoie True si l'action est réussie

        return response

def main():
    rclpy.init()
    node = WayPointServer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()