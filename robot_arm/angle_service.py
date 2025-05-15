import rclpy
from rclpy.node import Node
from custom_interfaces_cpp.srv import Adjust

class AdjAngle(Node):
    def __init__(self):
        super().__init__("adj_angle")
        self.service = self.create_service(Adjust, "adj_angle", self.Adjust_callback)
        self.i = 0

    def Adjust_callback(self, request, response):

        response.c = request.a
        response.d = request.b
        self.get_logger().info(f'Sending Adjust response: {response.c, response.d}')
        return response
    
def main(args = None):
    rclpy.init(args=args)

    adj_angle = AdjAngle()

    rclpy.spin(adj_angle)

    adj_angle.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()