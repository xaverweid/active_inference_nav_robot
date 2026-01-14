from rclpy.qos import qos_profile_sensor_data
from rclpy.node import Node
from geometry_msgs.msg import PoseArray, Twist
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Float32MultiArray
from nav2_msgs.msg import ParticleCloud

from .core import ActiveInferenceController
from .utils import ACTION_EFFECTS  # Move your twist values here

class AICNode(Node):
    def __init__(self):
        super().__init__('aic_node')
        
        # 1. Initialize the "Brain"
        # We pass the logger so the core can log without being a Node
        self.controller = ActiveInferenceController(logger=self.get_logger())

        #check if particles have been received
        self.latest_particles = None
        self.latest_weights = None
        self.particles_received = False
        
        # 2. ROS Infrastructure
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        self.map_sub = self.create_subscription(
            OccupancyGrid, '/map', self.map_callback, 10
        )
        self.particle_sub = self.create_subscription(
            ParticleCloud, '/particle_cloud', self.particle_callback, qos_profile_sensor_data
        )
        
        self.metrics_pub = self.create_publisher(Float32MultiArray, '/aic_metrics', 10)
        
        # 3. Control Loop (1Hz is good for discrete Active Inference)
        self.timer = self.create_timer(1.0, self.control_loop)
        self.get_logger().info("AIC Node Skin initialized.")

    def map_callback(self, msg):
        # Pass the map directly to the controller's generative model
        self.controller.set_map(msg)
        self.get_logger().info("Map registered in Controller.")
        # Only need the map once for static environments
        self.destroy_subscription(self.map_sub)

    def particle_callback(self, msg: ParticleCloud):
        # Pass the ROS message to the controller
        # always triggers whenever AMCL talks
        # The controller will use the ParticleClusturer (in utils) internally
        points, weights = self.clusturer.cloud_to_numpy(msg)

        self.latest_particles = points
        self.latest_weights = weights

        if not self.particles_received:
            self.get_logger().info(
                f"Received particle cloud with {len(msg.poses)} particles"
            )
            self.particles_received = True
        self.controller.update_belief(points, weights)
        
    def control_loop(self):
        # The Node only asks the controller for a decision
        if self.latest_particles is None:
            self.get_logger().debug("Waiting for particle cloud...")
            return

        # Belief exists. Ask the controller to decide the best action.
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