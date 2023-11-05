import rclpy, math
# import the ROS2 python libraries
from rclpy.node import Node
# import the Twist module from geometry_msgs interface
from geometry_msgs.msg import Twist
# import the LaserScan module from sensor_msgs interface
from sensor_msgs.msg import LaserScan
from rclpy.qos import ReliabilityPolicy, QoSProfile

class  Patrol(Node):

    def __init__(self):
        # Here you have the class constructor
        # call the class constructor
        super().__init__('lidar')
        # create the publisher object
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        # create the subscriber object
        self.subscriber = self.create_subscription(LaserScan, '/scan', self.laser_callback, QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE))
        # define the timer period for 0.5 seconds
        self.timer_period = 0.5  # seconds
        # define the variable to save the received info
        self.laser_forward = 0
        self.laser_frontLeft = 0
        self.laser_frontRight = 0

        # create a Twist message
        self.cmd = Twist()
        self.timer = self.create_timer(self.timer_period, self.motion)

    def laser_callback(self,msg):
        # Save the frontal laser scan info at 0°
        self.laser_forward = msg.ranges[359]
        self.laser_frontLeft = min(msg.ranges[0:15])
        self.laser_frontRight = min(msg.ranges[345:359])


        
    def motion(self):
        # print the data
        self.get_logger().info(f'right: {self.laser_frontRight} left: {self.laser_frontLeft}')
        
        if (self.laser_frontLeft <0.75) :
            self.get_logger().info(f'Object front left: {self.laser_frontLeft}')
        if (self.laser_frontRight <0.75) :
            self.get_logger().info(f'Object front right: {self.laser_frontRight}')

        # Logic of move
        # forward
        if (self.laser_frontRight >0.75 and self.laser_frontLeft >0.75):
            self.cmd.linear.x = 0.5
            self.cmd.angular.z = 0.0
            self.publisher_.publish(self.cmd)
            self.get_logger().info(f'Moving forward: {self.laser_forward}')
        # right
        if (self.laser_frontRight <0.75 and self.laser_frontLeft >0.75):
            self.cmd.linear.x = 0.0
            self.cmd.angular.z = -math.radians(10.0)
            self.publisher_.publish(self.cmd)
            self.get_logger().info(f'Turning right: {self.laser_forward}')
        # left
        if (self.laser_frontRight >0.75 and self.laser_frontLeft <0.75):
            self.cmd.linear.x = 0.0
            self.cmd.angular.z = math.radians(10.0)
            self.publisher_.publish(self.cmd)
            self.get_logger().info(f'Turning left: {self.laser_forward}')
        # backward
        if (self.laser_frontRight <0.75 and self.laser_frontLeft <0.75):
            self.cmd.linear.x = 0.0
            self.cmd.angular.z = math.radians(80.0)
            self.publisher_.publish(self.cmd)
            self.get_logger().info(f'Moving backward: {self.laser_forward}')
        

        # Publishing the cmd_vel values to a Topic
        # self.publisher_.publish(self.cmd)


            
def main(args=None):
    # initialize the ROS communication
    rclpy.init(args=args)
    # declare the node constructor
    patrol = Patrol()       
    # pause the program execution, waits for a request to kill the node (ctrl+c)
    rclpy.spin(patrol)
    # Explicity destroy the node
    patrol.destroy_node()
    # shutdown the ROS communication
    rclpy.shutdown()

if __name__ == '__main__':
    main()