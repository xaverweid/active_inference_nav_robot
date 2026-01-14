from rclpy.qos import qos_profile_default
from rclpy.node import Node
from geometry_msgs.msg import PoseArray, Twist
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Float32MultiArray
from nav2_msgs.msg import ParticleCloud
import numpy as np
import math
from threading import Timer
from .core import ActiveInferenceController
from .utils import ACTION_EFFECTS, ParticleClusturer  # Move your twist values here

class AICNode(Node):
    def __init__(self):
        super().__init__('aic_node')
        
        # 1. Initialize the "Brain"
        # We pass the logger so the core can log without being a Node
        self.controller = ActiveInferenceController(logger=self.get_logger())
        self.metrics_pub = self.create_publisher(Float32MultiArray, '/aic_metrics', 10)
        self.controller.set_metrics_publisher(self.metrics_pub)

        self.clusturer = ParticleClusturer(n_clusters=5)

        self.time_delta = 1.0  # Time step for discrete actions (seconds), adjust as needed

        # latest particle data
        self.latest_particles = None
        self.latest_weights = None
        self.particles_received = False

        self._stop_timer = None
        
        # 2. ROS Infrastructure
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        self.map_sub = self.create_subscription(
            OccupancyGrid, '/map', self.map_callback, 10
        )
        self.particle_sub = self.create_subscription(
            ParticleCloud, '/particle_cloud', self.particle_callback, qos_profile_default
        )
        # Note: AMCL publishes /particle_cloud with BEST_EFFORT QoS,
        # so subscribers must also use best_effort to receive messages.
        # qos_profile_sensor_data provides this (best_effort/volatile).
                
        # 3. Control Loop (1Hz is good for discrete Active Inference)
        self.timer = self.create_timer(self.time_delta, self.control_loop)
        self.get_logger().info(f"AIC Node Skin initialized with time_delta={self.time_delta}s.")

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

        if len(points) == 0:
            self.get_logger().warn("Received empty particle cloud!")
            return
        
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
            # apply action for exactly self.time_delta seconds
            self.apply_action(best_action_name)

    def apply_action(self, action_name):
        twist_msg = self.translate_action_to_twist(action_name)
        self.cmd_vel_pub.publish(twist_msg)

         # cancel any previous stop timer
        if self._stop_timer is not None:
            self._stop_timer.cancel()

        # schedule a stop after time_delta
        self._stop_timer = Timer(self.time_delta, self.stop_motion)
        self._stop_timer.daemon = True
        self._stop_timer.start()

    def stop_motion(self):
        stop_msg = Twist()
        self.cmd_vel_pub.publish(stop_msg)

    def translate_action_to_twist(self, action_name):
        """Returns a Twist corresponding to applying the discrete action for TIME_DELTA as velocities."""
        t = Twist()
        vals = ACTION_EFFECTS.get(action_name, {'linear': 0.0, 'angular': 0.0})
        # ACTION_EFFECTS should contain velocities (m/s, rad/s)
        t.linear.x = vals['linear']
        t.angular.z = vals['angular']
        return t