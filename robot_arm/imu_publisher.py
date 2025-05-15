import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3


class AccelerationListener(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            Imu,
            '/imu/data',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.publisher_x = self.create_publisher(Vector3, '/imu/position', 10)
        self.publisher_v = self.create_publisher(Vector3, '/imu/velocity', 10)
        self.a_x = 0.0
        self.a_y = 0.0
        self.a_z = 0.0

        self.dt =0.0
        self.t_prev = 0.0
        self.t = 0.0
        self.v_x = 0.0
        self.v_y = 0.0
        self.v_z = 0.0
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0

    def listener_callback(self, msg):
        #self.get_logger().info('I heard: "%s"' % msg.data)
        self.a_x = msg.linear_acceleration.x
        self.a_y = msg.linear_acceleration.y
        self.a_z = msg.linear_acceleration.z
        self.t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.dt = self.t - self.t_prev
        self.v = self.v_calc()
        self.position = self.x_calc()
        self.t_prev = self.t

    def v_calc(self):
        self.v_x = self.v_x + self.a_x * self.dt
        self.v_y = self.v_y + self.a_y * self.dt
        self.v_z = self.v_z + self.a_z * self.dt
        self.v = Vector3()
        self.v.x = self.v_x
        self.v.y = self.v_y
        self.v.z = self.v_z
        self.publisher_v.publish(self.v)

        return self.v
    
    def x_calc(self):
        self.x = self.x + self.v_x * self.dt
        self.y = self.y + self.v_y * self.dt   
        self.z = self.z + self.v_z * self.dt
        self.position = Vector3()
        self.position.x = self.x
        self.position.y = self.y
        self.position.z = self.z
        self.publisher_x.publish(self.position)

        return self.x


def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = AccelerationListener()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()