import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3
from nav_msgs.msg import Odometry

class Minitask1(Node):

    def __init__(self):
        super().__init__('minitask1')
        #create the publisher
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        #create the subscriber
        self.subscription = self.create_subscription(
            Odometry,
            'odom',
            self.odom_callback,
            10)
        self.subscription  # prevent unused variable warning

    #publish a message every 0.5 seconds
    timer_count = 0
    def timer_callback(self):
        #create new message of type Twist
        msg = Twist()
        #create linear component
        l = Vector3()
        l.x = 2.0
        l.y = 0.0
        l.z = 0.0
        #create angular component
        a = Vector3()
        a.x = 0.0
        a.y = 0.0
        a.z = -0.5

        #set message linear and angular
        msg.linear = l
        msg.angular = a
        #publish message
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing Twist...')

    def odom_callback(self, msg):
        location = msg.pose.pose.position
        self.get_logger().info('Robot is at: "%s"' % location)


    

def main(args=None):
    rclpy.init(args=args)

    mt = Minitask1()

    rclpy.spin(mt)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    mt.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
