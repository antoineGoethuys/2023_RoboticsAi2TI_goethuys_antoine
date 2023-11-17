import rclpy
# import the ROS2 python libraries
from rclpy.node import Node
# import the Twist module from geometry_msgs interface
from geometry_msgs.msg import Twist
# import the LaserScan module from sensor_msgs interface
from sensor_msgs.msg import LaserScan
from rclpy.qos import ReliabilityPolicy, QoSProfile
# import the degreemath module from math library
from math import degrees, radians

class Avoid_obstacle(Node):

    def __init__(self):
        # Here you have the class constructor
        # call the class constructor
        super().__init__('avoid_obstacle')
        # create the publisher object
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        # create the subscriber object
        self.subscriber = self.create_subscription(LaserScan, '/scan', self.laser_callback, QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
        # define the timer period for 0.5 seconds
        self.timer_period = 0.5
        # define the variable to save the received info
        self.laser_forward = 0
        self.laser_right = 0
        self.laser_left = 0
        # create a Twist message
        self.cmd = Twist()
        self.timer = self.create_timer(self.timer_period, self.motion)

    def laser_callback(self,msg):
        # Save the frontal laser scan info at 0°
        self.laser_forward = msg.ranges[-1]
        self.laser_right = msg.ranges[15]
        self.laser_left = msg.ranges[-15]
        
        
    def motion(self):
        # print the data
        # self.get_logger().info('I receive: "%s"' % str(self.laser_forward))
        # Logic of move
        if self.laser_forward > 2:
            self.cmd.linear.x = 0.2
            self.cmd.angular.z = 0.0
            self.get_logger().info('go straight')
        elif 2 > self.laser_forward >= 0.2:
            self.cmd.linear.x = 0.1
            self.cmd.angular.z = 0.0
            self.get_logger().info('go straight low speed')
        else:
            if self.laser_right > self.laser_left:
                self.cmd.linear.x = 0.0
                self.cmd.angular.z = radians(-90)
                self.get_logger().info('turn right')
            elif self.laser_right < self.laser_left:
                self.cmd.linear.x = 0.0
                self.cmd.angular.z = radians(90)
                self.get_logger().info('turn left')
            
            
        # Publishing the cmd_vel values to a Topic
        self.publisher_.publish(self.cmd)


            
def main(args=None):
    # initialize the ROS communication
    rclpy.init(args=args)
    # declare the node constructor
    avoid_obstacle = Avoid_obstacle()       
    # pause the program execution, waits for a request to kill the node (ctrl+c)
    rclpy.spin(avoid_obstacle)
    # Explicity destroy the node
    avoid_obstacle.destroy_node()
    # shutdown the ROS communication
    rclpy.shutdown()

if __name__ == '__main__':
    main()