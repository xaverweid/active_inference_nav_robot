import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray, Twist
from nav_msgs.msg import OccupancyGrid
from .core import get_best_exploratory_action
from .utils import ros_pose_to_np, get_map_metadata # Need to implement utils

class AICNode(Node):
    def __init__(self):
        super().__init__('aic_node')
        # publishers: command and debug
        self.actions = ['forward_short', 'rotate_left', 'rotate_right', 'spin_360', 'wait']
        self.map_data = None # Will be populated once map is received
        self.is_localized = False # State check for when to act
        self.belief_pub = self.create_publisher(PoseArray, '/aic/belief_state', 10)
        
        # ROS Publishers/Subscribers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.map_sub = self.create_subscription(
            OccupancyGrid, '/map', self.map_callback, 10
        )
        self.particle_sub = self.create_subscription(
            PoseArray, '/particlecloud', self.particle_callback, 10
        )
        self.get_logger().info("AIC Node initialized. Waiting for map and particles...")
        
        # Main control loop timer (e.g., decide every 1 second)
        self.timer = self.create_timer(1.0, self.control_loop)

    def map_callback(self, msg):
        # Load map once and stop subscribing
        self.map_data = msg
        self.get_logger().info("Map received.")
        self.destroy_subscription(self.map_sub) 

    def particle_callback(self, msg):
        # Convert ROS PoseArray to a simple Numpy array list for core.py
        self.current_particles = [ros_pose_to_np(p) for p in msg.poses]
        
    def control_loop(self):
        if self.map_data is None or not hasattr(self, 'current_particles') or not self.current_particles:
            return # Wait for data

        # Check if localized (optional: e.g., particle variance is low)
        # You can add logic here to check variance/entropy of the cloud.
        
        # 1. Run the Brain
        best_action = get_best_exploratory_action(self.current_particles, self.map_data, self.actions)
        
        # 2. Convert action to velocity command
        twist_msg = self.action_to_twist(best_action)
        self.cmd_vel_pub.publish(twist_msg)
        self.get_logger().info(f"Selected action: {best_action}")

    def action_to_twist(self, action):
        # Converts discrete action to a continuous Twist message
        # Needs to be implemented (e.g., 'forward_short' = 0.2 m/s linear, 0 angular)
        twist = Twist()
        # ... logic to fill twist.linear.x and twist.angular.z
        return twist

def main(args=None):
    rclpy.init(args=args)
    aic_node = AICNode()
    rclpy.spin(aic_node)
    aic_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()