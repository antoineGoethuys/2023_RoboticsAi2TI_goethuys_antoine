import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from rclpy.qos import ReliabilityPolicy, QoSProfile
import matplotlib.pyplot as plt
import numpy as np
import math, time
from scipy import signal

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
        self.width = 0
        self.depth = 0
        self.shortest_corner = 0
        self.isNotParked:bool = True
        self.isControlled:bool = False    
        # graph
        self.lines = []   
    
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
        self.scatter_corners = None

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
        self.calculate_parking_dimensions()
        self.move()
            
        # Publishing the cmd_vel values to a Topic
        self.publisher_.publish(self.cmd)
    
    def find_edges(self):
        alpha = 360//len(self.laser_array)
        tolerance = 0.05
        upper_tolerance = 0.5
        corner_array = [[],[]]
        corners = {}
        corners_type = {}
        # print("="*30)

        for i, l in enumerate(self.laser_array):
            l1 = self.laser_array[i-4]
            a1 = alpha*(i-4)

            l2 = self.laser_array[i-3]
            a2 = alpha*(i-3)

            l3 = self.laser_array[i-2]
            a3 = alpha*(i-2)

            l4 = self.laser_array[(i-1)]
            a4 = alpha*(i-1)

            l5 = self.laser_array[(i)]
            a5 = alpha*(i)

            # Check if the values are finite before performing subtraction
            if np.isfinite(l1) and np.isfinite(l2) and np.isfinite(l3) and np.isfinite(l4) and np.isfinite(l5):
                if ((l1 - l2) < tolerance/l3) and ((l2 - l3) < tolerance/l3) and ((l3 - l4) > tolerance/l3) and ((l4 - l5) > tolerance/l3):
                    corners[a3] = l3
                    corners_type[a3] = "90"
                
                if ((l1 - l2) > tolerance/l3) and ((l2 - l3) > tolerance/l3) and ((l3 - l4) < tolerance/l3) and ((l4 - l5) < tolerance/l3):
                    corners[a3] = l3
                    corners_type[a3] = "270"

                if ((l1 - l2) < tolerance/l3) and (l2 - l3) > (upper_tolerance) and ((l3 - l4) < tolerance/l3) and ((l4 - l5) < tolerance/l3):
                    corners[a3] = l3
                    corners_type[a3] = "90s"
        return corners, corners_type
    
    def cleangraph(self):
        for collection in self.ax.collections:
            collection.remove()
        for collection in self.ax2.collections:
            collection.remove()
        for line in self.lines:
            line.remove()
        self.lines = []
    
    def calculate_parking_dimensions(self):
        corners, _ = self.find_edges()
        if len(corners) < 3:
            # self.log("Not enough corners found to calculate parking dimensions.")
            return

        # Sort corners by angle
        sorted_corners = sorted(zip(corners.keys(), corners.values()))

        # Calculate differences between consecutive angles
        angle_diffs = [(sorted_corners[i+1][0] - sorted_corners[i][0], i) for i in range(len(sorted_corners)-1)]
        angle_diffs.append((360 + sorted_corners[0][0] - sorted_corners[-1][0], len(sorted_corners)-1))  # Wrap around

        # Sort by angle difference
        angle_diffs.sort(key=lambda x: x[0])

        # The smallest difference corresponds to width, the largest to depth
        width_index = angle_diffs[0][1]
        depth_index = angle_diffs[-1][1]

        # Calculate width and depth
        self.width = abs(sorted_corners[width_index][1] - sorted_corners[(width_index+1)%len(sorted_corners)][1])
        self.depth = abs(sorted_corners[depth_index][1] - sorted_corners[(depth_index+1)%len(sorted_corners)][1])

        # Calculate shortest corner
        self.shortest_corner = min(corners.values())

        # self.log(f"Calculated parking dimensions: Width = {self.width}, Depth = {self.depth}, Shortest Corner = {self.shortest_corner}")
        self.log(f"parking dimensions are good")

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

    def connect_corners(self, corners):
        if corners:  # Check if corners is not empty
            # Sort corners by angle and plot lines
            sorted_corners = sorted(zip(corners.keys(), corners.values()))
            sorted_angles, sorted_distances = zip(*sorted_corners)
            
            # Append the first corner to the end to connect all corners
            sorted_angles = list(sorted_angles) + [sorted_angles[0]]
            sorted_distances = list(sorted_distances) + [sorted_distances[0]]
            
            line1, = self.ax.plot(np.radians(sorted_angles), sorted_distances, 'r-')
            line2, = self.ax2.plot(sorted_angles, sorted_distances, 'r-')
            self.lines.append(line1)
            self.lines.append(line2)

    def graph(self):
        # Update scatter plot data
        theta = np.linspace(0, 2*np.pi, len(self.laser_array), endpoint=False)
        self.cleangraph()
        corners, corners_type = self.find_edges()
        # print(corners)

        # Add corners to the graphs
        corner_angles = list(corners.keys())
        corner_distances = list(corners.values())
        corner_types = list(corners_type.values())

        # Define colors for each type of corner
        corner_colors = ['r' if corner_type == '90' else 'yellow' if corner_type == '90s' else'g' for corner_type in corner_types]

        self.scatter_corners = self.ax.scatter(np.radians(corner_angles), corner_distances, c=corner_colors, s=50)
        self.scatter2_corners = self.ax2.scatter(corner_angles, corner_distances, c=corner_colors, s=50)

        # Plot the lidar array
        self.scatter = self.ax.scatter(theta, self.laser_array, c='b', s=1)
        self.scatter2 = self.ax2.scatter(range(0,360), self.laser_array, c='b', s=1)

        # Filter out NaN and Inf values
        laser_array_filtered = self.laser_array[np.isfinite(self.laser_array)]

        # Extend axis range by 2 meters
        self.ax.set_rmax(max(laser_array_filtered) + 2 if laser_array_filtered.size > 0 else 2)
        self.ax2.set_ylim(0, max(laser_array_filtered) + 2 if laser_array_filtered.size > 0 else 2)

        # Connect corners with lines
        self.connect_corners(corners)
        self.sidegraph()

        plt.draw()
        plt.pause(0.01)


    def move(self):
        # turn right in block
        if self.isControlled == False:
            if self.laser_right > 0.5:
                self.isControlled = True
                self.start_turn_time = time.time()  # Start the timer when the robot starts turning
                self.turn(-90)
                self.log(f'turn right in block')
                return
            else:
                self.isControlled = False
                self.forward(0.5)
                self.log(f'forward in block')
                return
        else:
            # Stop turning after 5 seconds
            if time.time() - self.start_turn_time > 5:
                self.isControlled = False
                self.stop()
                self.log(f'stop turning')

            

    def log(self, message: str):
        self.get_logger().info(message)

    def forward(self, speed: float):
        self.cmd.linear.x = speed
        self.cmd.angular.z = 0.0

    def turn(self, degrees: float):
        self.cmd.linear.x = 0.0
        self.cmd.angular.z = math.radians(degrees)
    
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