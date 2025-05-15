import rclpy
from rclpy.node import Node
import math
from std_msgs.msg import Float32MultiArray

from sensor_msgs.msg import JointState


class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        '''self.subscription = self.create_subscription(
            JointState,
            'joint_states',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning'''
        self.publisher_ = self.create_publisher(JointState, 'joint_states', 10)
        timer_period = 0.05  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    


        self.subscription = self.create_subscription(
            Float32MultiArray,
            'angle_adjustment',
            self.listener_callback_c,
            10)
        self.subscription  # prevent unused variable warning

        
        self.theta1 = 0
        self.theta2 = 0

    def listener_callback_c(self, msg_c):
        self.theta1 = msg_c.data[0]
        self.theta2 = msg_c.data[1]
        self.get_logger().info('I heard: "%s"' % msg_c.data)


    def timer_callback(self):
        msg = JointState()
        msg.name = ['joint1', 'joint2']
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.position = [0.0, 0.0]
        msg.position[0] = self.theta1
        msg.position[1] = self.theta2
        
        self.get_logger().info('I heard: "%s"' % msg.position)
        self.x = 0.6 * math.cos(msg.position[0]) + 0.6 * math.cos(msg.position[0] + msg.position[1])
        self.y = 0.6 * math.sin(msg.position[0]) + 0.6 * math.sin(msg.position[0] + msg.position[1]) + 0.5
        
        #msg.data = 'Hello World: %d' % self.i
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.position)
        self.i += 1

    '''def listener_callback(self, msg):
        
        msg.position[0] = self.theta1
        msg.position[1] = self.theta2
        
        self.get_logger().info('I heard: "%s"' % msg.position)
        self.x = 0.6 * math.cos(msg.position[0]) + 0.6 * math.cos(msg.position[0] + msg.position[1])
        self.y = 0.6 * math.sin(msg.position[0]) + 0.6 * math.sin(msg.position[0] + msg.position[1]) + 0.5'''
         


def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()