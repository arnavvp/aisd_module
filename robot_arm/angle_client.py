import sys

import rclpy
from rclpy.node import Node
from custom_interfaces_cpp.srv import Adjust

from std_msgs.msg import Float32MultiArray

class AngleCaller(Node):
    def __init__(self):
        super().__init__("adjust_angle")
        self.client = self.create_client(Adjust, "adj_angle")

        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn("Waiting for service to be available...")

        self.publisher_ = self.create_publisher(Float32MultiArray, 'angle_adjustment', 10)
    def send_request(self):
        request = Adjust.Request()
        request.a = float(sys.argv[1])
        request.b = float(sys.argv[2])
        self.future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, self.future)

   
        if self.future.done():
            try:
                response = self.future.result()
                msg_c = Float32MultiArray()
                msg_c.data = [response.c, response.d]  
                self.publisher_.publish(msg_c)
                

                self.get_logger().info(f'Published k and b: {msg_c.data}')
            except Exception as e:
                self.get_logger().error(f'Service call failed: {e}')

def main(args=None):
    rclpy.init(args=args)

    adjust_angle =AngleCaller()
    adjust_angle.send_request()

    while rclpy.ok():
        rclpy.spin_once(adjust_angle)
        if adjust_angle.future.done():
            try:
                response = adjust_angle.future.result()
            except Exception as e:
                adjust_angle.get_logger().info(
                    f"Service call failed {e}"
                )
            else:
                adjust_angle.get_logger().info(
                 f"New spring constant and damping coefficient is: {response.c, response.d}"
                )

    adjust_angle.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()