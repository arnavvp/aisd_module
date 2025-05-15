import rclpy
from rclpy.node import Node
import math
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Quaternion
from std_msgs.msg import Float32MultiArray
from visualization_msgs.msg import Marker

class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.listener_callback_twist,
            10)
        self.subscription  # prevent unused variable warning
        self.publisher_ = self.create_publisher(Marker, 'topic', 10)
        self.prev_stamp = None
        self.x = 0.0 
        self.y = 0.0
        self.v_lat = 0.0
        self.v_long = 0.0
        self.w = 0.0
        self.a_lat = 0.0
        self.a_alpha = 0.0
        self.x_p = 0.0
        self.y_p = 0.0
        self.vx_p = 0.0
        self.vy_p = 0.0
        self.yaw = 0.0
        self.w_p = 0.0
        self.dt = 0.07
        self.prev_vlat = 0.0
        self.prev_yaw = 0.0
        self.prev_w = 0.0

        self.marker = Marker()
        self.marker.header.frame_id = "world"
        self.marker.header.stamp = self.get_clock().now().to_msg()
        self.marker.ns = "arrow_namespace"
        self.marker.id = 0
        self.marker.type = Marker.ARROW
        self.marker.action = Marker.ADD
        self.marker.scale.x = 0.5  
        self.marker.scale.y = 0.1  
        self.marker.scale.z = 0.1  
        self.marker.color.r = 0.0  
        self.marker.color.g = 1.0   
        self.marker.color.b = 0.0   
        self.marker.color.a = 1.0
        self.marker.pose.position.x =0.0
        self.marker.pose.position.y =0.0
        self.marker.pose.position.z =0.0
        self.marker.pose.orientation.x = 0.0
        self.marker.pose.orientation.y = 0.0
        self.marker.pose.orientation.z = 0.0
        self.marker.pose.orientation.w = 1.0

        self.publisher_.publish(self.marker)


    def listener_callback_twist(self, msg):
        self.w = msg.angular.z

        self.v_long = msg.linear.x
        vlon = self.v_long

        if self.v_long !=0:
            steer = math.atan(self.w*0.35/self.v_long)
        else:
            steer = 0.0

        self.prev_vlat = msg.linear.y
        new_state = self.calc(vlon, steer)

    # Update marker pose from state
        x = float(new_state[0])
        y = float(new_state[1])
        yaw = float(new_state[5])

        self.get_logger().info(f'Calculated state: x = {x}, y = {y}, yaw = {yaw}')


        self.marker.header.stamp = self.get_clock().now().to_msg()
        self.marker.pose.position.x = x
        self.marker.pose.position.y = y
        self.marker.pose.position.z = 0.0
        self.marker.pose.orientation.x = 0.0
        self.marker.pose.orientation.y = 0.0
        self.marker.pose.orientation.z = math.sin(yaw / 2.0)
        self.marker.pose.orientation.w = math.cos(yaw / 2.0)

        self.publisher_.publish(self.marker)



        

    def calc(self, vlon, steer):
        if self.v_long != 0:
            vlat_cur = self.prev_vlat + (-0.84/(4*self.v_long) * self.prev_vlat + 0.42/4 * steer)*self.dt
            yaw_cur = self.prev_yaw + self.prev_w * self.dt
            w_cur = self.prev_w + (-0.84/(0.0586*self.v_long) * self.prev_w + 0.42/0.0586 * 0.35 * steer)*self.dt
            vx_cur = (self.v_long * math.cos(yaw_cur) - vlat_cur * math.sin(yaw_cur))
            vy_cur = (self.v_long * math.sin(yaw_cur) + vlat_cur * math.cos(yaw_cur))
            x_cur = self.x + vx_cur * self.dt
            y_cur = self.y + vy_cur * self.dt
            self.x = x_cur
            self.y = y_cur
            self.yaw = yaw_cur

            state_cur = [x_cur, y_cur, vlat_cur, vx_cur, vy_cur, yaw_cur, self.w]
            self.prev_vlat = vlat_cur
            self.prev_yaw = yaw_cur
            self.prev_w = w_cur
            self.w = w_cur
            return state_cur
        else:
            state_cur = [self.x, self.y, self.prev_vlat, self.v_long * math.cos(self.prev_yaw), self.v_long * math.sin(self.prev_yaw), self.prev_yaw, self.prev_w]
            return state_cur





        

        #self.get_logger().info('I heard: "%s"' % msg.)



        
        
        


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