import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

class ObstacleAvoidanceSystem(Node):
    def __init__(self):
        super().__init__("ObstacleAvoidanceSystem")
        self.system_started = False
        self.setup_subscriber_publisher()

        self.motion_command = Twist()
        self.turn_direction = 1
        self.blocked_turn_direction = False
        self.turn_speed = 0.5

    def setup_subscriber_publisher(self):
        qos_profile_scan = self.create_qos_profile(QoSReliabilityPolicy.BEST_EFFORT, QoSHistoryPolicy.KEEP_LAST)
        self.subscription = self.create_subscription(LaserScan, "/scan", self.process_scan_data, qos_profile_scan)

        qos_profile_cmd_vel = self.create_qos_profile(QoSReliabilityPolicy.RELIABLE, QoSHistoryPolicy.KEEP_LAST)
        self.publisher = self.create_publisher(Twist, "/cmd_vel", qos_profile_cmd_vel)

    def create_qos_profile(self, reliability, history):
        return QoSProfile(reliability=reliability, history=history, depth=10)

    def process_scan_data(self, scan_msg):
        if not self.system_started and scan_msg.ranges[0] < 0.2:
            self.system_started = True
            self.get_logger().info("System Started")
        else:
            self.process_motion(scan_msg)

    def process_motion(self, scan_msg):
        front_distances = self.extract_front_distances(scan_msg)

        if min(front_distances) < 0.18:
            self.handle_obstacle()
        else:
            self.adjust_linear_motion(front_distances)

        if min(front_distances) <= 0.18:
            if not self.blocked_turn_direction:
                self.check_turn_direction()

            self.adjust_angular_motion()
        else:
            self.motion_command.angular.z = 0.0

        self.publisher.publish(self.motion_command)

    def extract_front_distances(self, scan_msg):
        return scan_msg.ranges[-15:] + scan_msg.ranges[:15]

    def handle_obstacle(self):
        self.blocked_turn_direction = True
        self.motion_command.linear.x = 0.0
        self.get_logger().info("Stop")

    def adjust_linear_motion(self, front_distances):
        self.blocked_turn_direction = False
        self.motion_command.linear.x = min(0.15, 0.15 * min(front_distances))

    def adjust_angular_motion(self):
        if not self.blocked_turn_direction:
            self.check_turn_direction()

        self.motion_command.angular.z = self.turn_direction * self.turn_speed
        self.get_logger().info("Turning")

    def check_turn_direction(self):
        length_array = len(scan_msg.ranges) * 60 // 360
        right_distances = scan_msg.ranges[0:length_array]
        left_distances = scan_msg.ranges[-length_array:]
        sum_right_distances = sum(right_distances)
        sum_left_distances = sum(left_distances)

        if sum_right_distances < sum_left_distances:
            self.turn_direction = -1
        elif sum_right_distances > sum_left_distances:
            self.turn_direction = 1
        else:
            self.turn_direction = 0
            self.get_logger().info("Turn direction: " + ("right" if self.turn_direction == -1 else "left"))

def main(args=None):
    rclpy.init(args=args)
    obstacle_avoidance_system = ObstacleAvoidanceSystem()
    rclpy.spin(obstacle_avoidance_system)
    obstacle_avoidance_system.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
