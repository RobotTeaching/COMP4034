import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3
from sensor_msgs.msg import LaserScan

class Minitask1(Node):
    
    def __init__(self):
        super().__init__('minitsak2')
        self.direction_1 = 0.0
        self.direction_2 = 0.0
        self.direction_3 = 0.0
        self.direction_4 = 0.0
        
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        #create the subscriber
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10)
        self.subscription  # prevent unused variable warning
    

    #every 0.5 seconds
    def timer_callback(self):
        self.get_logger().info('Distance at direction_1: %s' % self.direction_1)
        self.get_logger().info('Distance at direction_2: %s' % self.direction_2)
        self.get_logger().info('Distance at direction_3: %s' % self.direction_3)
        self.get_logger().info('Distance at direction_4: %s' % self.direction_4)

    def scan_callback(self, msg):
        self.direction_1 = msg.ranges[0]
        self.direction_2 = msg.ranges[89]
        self.direction_3 = msg.ranges[179]
        self.direction_4 = msg.ranges[274]

        


    

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
