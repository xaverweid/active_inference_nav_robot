import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray, Twist
from nav_msgs.msg import OccupancyGrid

from .core import ActiveInferenceController
from .utils import ACTION_EFFECTS  # Move your twist values here

class AICNode(Node):
    def __init__(self):
        super().__init__('aic_node')
        
        # 1. Initialize the "Brain"
        # We pass the logger so the core can log without being a Node
        self.controller = ActiveInferenceController(logger=self.get_logger())
        
        # 2. ROS Infrastructure
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        self.map_sub = self.create_subscription(
            OccupancyGrid, '/map', self.map_callback, 10
        )
        self.particle_sub = self.create_subscription(
            PoseArray, '/particlecloud', self.particle_callback, 10
        )
        
        # 3. Control Loop (1Hz is good for discrete Active Inference)
        self.timer = self.create_timer(1.0, self.control_loop)
        self.get_logger().info("AIC Node Skin initialized.")

    def map_callback(self, msg):
        # Pass the map directly to the controller's generative model
        self.controller.set_map(msg)
        self.get_logger().info("Map registered in Controller.")
        # Only need the map once for static environments
        self.destroy_subscription(self.map_sub)

    def particle_callback(self, msg):
        # Pass the ROS message to the controller
        # The controller will use the ParticleClusturer (in utils) internally
        self.controller.update_belief(msg)
        
    def control_loop(self):
        # The Node only asks the controller for a decision
        if not self.controller.is_ready():
            return

        best_action_name = self.controller.decide_action()
        
        if best_action_name:
            twist_msg = self.translate_action_to_twist(best_action_name)
            self.cmd_vel_pub.publish(twist_msg)

    def translate_action_to_twist(self, action_name):
        """Moves the mapping logic out of the way"""
        t = Twist()
        # Look up values from a dictionary in utils.py
        vals = ACTION_EFFECTS.get(action_name, {'linear': 0.0, 'angular': 0.0})
        t.linear.x = vals['linear']
        t.angular.z = vals['angular']
        return t