import rclpy, math
# import the ROS2 python libraries
from rclpy.node import Node
# import the Twist module from geometry_msgs interface
from geometry_msgs.msg import Twist
# import the LaserScan module from sensor_msgs interface
from sensor_msgs.msg import LaserScan
from rclpy.qos import ReliabilityPolicy, QoSProfile

class  Practical_test(Node):

    def __init__(self):
        # Here you have the class constructor
        # call the class constructor
        super().__init__('practical_test')
        # create the publisher object
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        # create the subscriber object
        self.subscriber = self.create_subscription(LaserScan, '/scan', self.laser_callback, QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
        # define the timer period for 0.5 seconds
        self.timer_period = 1.0  # seconds
        # define the variable to save the received info
        self.laser_frontForward = 0
        self.laser_frontLeft = 0
        self.laser_frontRight = 0
        self.laser_frontBack = 0

        # create a Twist message
        self.cmd = Twist()
        self.timer = self.create_timer(self.timer_period, self.motion)

    def laser_callback(self,msg):
        # Save the frontal laser scan info at 0°
        #self.get_logger().info(f'{msg.range_min} : { msg.range_max}')
        if msg:
            degree = int(msg.range_max/12) # 30°
            self.get_logger().info(f'{degree}')
            self.laser_frontForward = msg.ranges[-1] # 0°
            self.laser_frontRight = min(msg.ranges[0:30])          # 0° to 30°
            self.laser_frontLeft = min(msg.ranges[-30:-1])          # -30° to 0°
            self.laser_frontBack = min(msg.ranges[-30*5:30*5]) # -180° to 180°


        
    def motion(self):
        # print the data
        #self.get_logger().info(f'forward: {self.laser_frontForward} right: {self.laser_frontRight} left: {self.laser_frontLeft} back: {self.laser_frontBack}')
        started = False
        stop = False
        # Logic of move
        if self.laser_frontBack < 0.1:
            started = True
        
        if self.laser_frontBack < 0.1 and self.laser_frontForward < 0.1:
            stop = True

        
        if started:
            # stop
            if self.laser_frontBack < 0.1 and self.laser_frontForward < 0.1:
                stop = True
            # move forward
            if self.laser_frontRight > 0.3 and self.laser_frontForward > 0.3 and self.laser_frontRight > 0.3:
                self.cmd.linear.x = 0.2
                self.cmd.angular.z = 0.0
            # turn left
            elif self.laser_frontRight > self.laser_frontLeft:
                self.cmd.linear.x = 0.0
                self.cmd.angular.z = math.radians(90)
            # turn right
            else:
                self.cmd.linear.x = 0.0
                self.cmd.angular.z = math.radians(-90)
            # stop
            if stop:
                self.cmd.linear.x = 0.0
                self.cmd.angular.z = 0.0
                started = False
        # Publishing the cmd_vel values to a Topic
        self.publisher_.publish(self.cmd)


            
def main(args=None):
    # initialize the ROS communication
    rclpy.init(args=args)
    # declare the node constructor
    practical_test = Practical_test()       
    # pause the program execution, waits for a request to kill the node (ctrl+c)
    rclpy.spin(practical_test)
    # Explicity destroy the node
    practical_test.destroy_node()
    # shutdown the ROS communication
    rclpy.shutdown()

if __name__ == '__main__':
    main()