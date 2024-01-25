import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from rclpy.qos import ReliabilityPolicy, QoSProfile
import matplotlib.pyplot as plt
import numpy as np
import math, time

class AutoPark(Node):

    def __init__(self):
        super().__init__('auto_park')

        # Publisher
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        
        # Subscriber
        self.subscriber = self.create_subscription(LaserScan, '/scan', self.laser_callback, QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE))

        # define the timer period for 1.0 seconds
        self.timer_period = 1.0

        # define variables
        # laser
        self.laser_forward = 0
        self.laser_right = 0
        self.laser_left = 0
        self.laser_back = 0
        # laser array
        self.laser_array = np.array([])
        # parking space
        self.parkingFound:bool = False
        self.timeStarted:bool = False
        self.parkingSpaceEnd:bool = False
        self.startTime = 0
        self.endTime = 0
        self.distance = 0.0
    
        # mathplotlib
        # Create initial polar scatter plot
        self.fig = plt.figure()
        self.ax = self.fig.add_subplot(121, polar=True)
        self.scatter = self.ax.scatter([], [])
        self.ax.set_title('Lidar Array Right')
        self.ax.set_theta_zero_location('N')
        # create a other initial normal scatter plot
        # self.fig2 = plt.figure()
        # self.ax2 = self.fig2.add_subplot(111)
        self.ax2 = self.fig.add_subplot(122)
        self.scatter2 = self.ax2.scatter([], [])
        self.ax2.set_title('Lidar Array Right')
        self.ax2.set_xlabel('Angle (deg)')
        self.ax2.set_ylabel('Distance (m)')

        # Initialize scatter plot references
        self.scatter = None
        self.scatter_front = None
        self.scatter_right = None
        self.scatter_back = None
        self.scatter_left = None

        self.cmd = Twist()

        # timer
        self.timer = self.create_timer(self.timer_period, self.motion)

    def laser_callback(self, msg):
        #legth of the lidar array
        current_length = len(msg.ranges)
        # set the lidar array
        self.laser_array = np.array(msg.ranges)

        # Get the laser values for the front, right, left and back
        self.laser_forward = self.laser_array[0]
        self.laser_right = self.laser_array[ 3 * current_length // 4]
        self.laser_left = self.laser_array[ current_length // 4]
        self.laser_back = self.laser_array[ current_length // 2]
    
    def motion(self):
        self.graph()
        self.move()
            
        # Publishing the cmd_vel values to a Topic
        self.publisher_.publish(self.cmd)
    
    def cleangraph(self):
        for collection in self.ax.collections:
            collection.remove()
        for collection in self.ax2.collections:
            collection.remove()

    def sidegraph(self):
        # Highlight the front, right, left, and back sides
        # graph 1
        self.scatter_front = self.ax.scatter(0, self.laser_forward, c='m', s=50)
        self.scatter_right = self.ax.scatter(3*np.pi/2, self.laser_right, c='m', s=50, marker='x')
        self.scatter_back = self.ax.scatter(np.pi, self.laser_back, c='m', s=50)
        self.scatter_left = self.ax.scatter(np.pi/2, self.laser_left, c='m', s=50)
        # graph 2
        self.scatter2_front = self.ax2.scatter(0, self.laser_forward, c='m', s=50)
        self.scatter2_right = self.ax2.scatter(270, self.laser_right, c='m', s=50, marker='x')
        self.scatter2_back = self.ax2.scatter(180, self.laser_back, c='m', s=50)
        self.scatter2_left = self.ax2.scatter(90, self.laser_left, c='m', s=50)

    def graph(self):
        # Update scatter plot data
        theta = np.linspace(0, 2*np.pi, len(self.laser_array), endpoint=False)
        self.cleangraph()

        # Plot the lidar array
        self.scatter = self.ax.scatter(theta, self.laser_array, c='b', s=1)
        self.scatter2 = self.ax2.scatter(range(len(self.laser_array)), self.laser_array, c='b', s=1)

        # Filter out NaN and Inf values
        laser_array_filtered = self.laser_array[np.isfinite(self.laser_array)]

        # Extend axis range by 2 meters
        self.ax.set_rmax(max(laser_array_filtered) + 2 if laser_array_filtered.size > 0 else 2)
        self.ax2.set_ylim(0, max(laser_array_filtered) + 2 if laser_array_filtered.size > 0 else 2)

        self.sidegraph()

        plt.draw()
        plt.pause(0.01)


    def move(self):
        print(self.laser_right, self.parkingFound)
        if not self.parkingFound:
            self.log("Searching for parking space...")
            self.cmd.linear.x = 0.2
        if 0.6 < self.laser_right < 1.5 and not self.parkingSpaceEnd:
            self.stop()
            self.log("Found parking space!")
            self.parkingFound = True
            if not self.timeStarted:
                self.startTime = time.time()
                self.timeStarted = True
            self.cmd.linear.x = 0.2
        elif self.timeStarted:
            self.stop()
            self.log("end of parking space")
            self.parkingFound = True
            if not self.parkingSpaceEnd:
                self.endTime = time.time()
                self.parkingSpaceEnd = True
                self.parkingManeuver((self.endTime - self.startTime))
    
    def parkingManeuver(self, timer: float):
        self.distance = float(self.laser_right)
        timed = self.distance / 0.15

        self.backward(0.2, timer*0.75)
        self.turn(90)
        self.backward(0.2, timed)


    def backward(self, speed: float, timer: float):
        self.cmd.linear.x = -speed
        self.cmd.angular.z = 0.0
        self.log(f"backwards for {timer} seconds {speed}")
        self.publisher_.publish(self.cmd)
        time.sleep(timer)
        self.stop()

    def log(self, message: str):
        self.get_logger().info(message)

    def forward(self, speed: float, distance: float):
        self.cmd.linear.x = speed
        self.cmd.angular.z = 0.0
        time_to_move = abs(distance / speed)
        self.log(f"1 forward for {time_to_move} seconds {speed}")
        self.publisher_.publish(self.cmd)
        time.sleep(time_to_move)
        self.stop()

    def turn(self, degrees: float):
        speed = 0.5  # Define your turning speed here
        radians = math.radians(degrees)  # Convert degrees to radians
        self.cmd.angular.z = speed if radians > 0 else -speed
        time_to_turn = abs(radians / speed)
        self.log(f"Turning for {time_to_turn} seconds at speed {speed}")
        self.publisher_.publish(self.cmd)
        time.sleep(time_to_turn)
        self.stop()

    def stop(self):
        self.cmd.linear.x = 0.0
        self.cmd.angular.z = 0.0
            
def main(args=None):
    # init
    rclpy.init(args=args)

    # class constructor
    auto_park = AutoPark()

    # pause the program execution, waits for a request to kill the node (ctrl+c)
    rclpy.spin(auto_park)

    # Explicity destroy the node
    auto_park.destroy_node()

    # shutdown the ROS communication
    rclpy.shutdown()

if __name__ == '__main__':
    main()