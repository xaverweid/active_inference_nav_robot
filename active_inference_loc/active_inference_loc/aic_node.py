import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry, OccupancyGrid
from std_msgs.msg import Float32MultiArray, String
from nav2_msgs.msg import ParticleCloud
from threading import Timer
from .core import ActiveInferenceController
from .utils import ACTION_EFFECTS, cloud_to_numpy
from std_srvs.srv import Empty
from rclpy.time import Time
import numpy as np


class AICNode(Node):
    """
    ROS Node that bridges algorithm (core.py) with ROS infrastructure.
    Responsibilities:
    - Subscribe to ROS topics (map, particles, ground truth)
    - Call core.py for decisions
    - Publish commands and metrics
    - Handle timing and synchronization
    """
    
    def __init__(self):
        super().__init__('aic_node')

        self.declare_parameter('algo_mode', 'active_inf')
        self.algorithm_mode = self.get_parameter('algo_mode').get_parameter_value().string_value
        self.get_logger().info(f"--- LAUNCHING AIC NODE IN MODE: {self.algorithm_mode} ---")

        self.time_delta = 1.0
        self.recording_delay = 0.0
        self.ticks_to_wait = int(self.recording_delay / self.time_delta)
        self.ticks_passed = 0

        # State Tracking
        self.map_ready = False
        self.particles_received = False
        self.system_active = False
        self.ready_to_think = True
        self.waiting_for_update = False
        
        # Timing
        self.last_particles_time = None
        self._stop_timer = None

        # Ground Truth (for logging only!)
        self.ground_truth_pose = None
        
        # Initialize Brain
        self.controller = ActiveInferenceController(logger=self.get_logger(), algo_mode=self.algorithm_mode)
        
        # Publishers
        self.metrics_pub = self.create_publisher(Float32MultiArray, '/aic_metrics', 10)
        self.filtered_particle_pub = self.create_publisher(Float32MultiArray, '/belief/particles_filtered', 10)
        self.status_pub = self.create_publisher(String, '/trial_status', 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        self.controller.set_metrics_publisher(self.metrics_pub)
        self.controller.set_particle_publisher(self.filtered_particle_pub)
        self.controller.set_status_publisher(self.status_pub)
        
        # Subscribers
        map_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=1
        )
        self.map_sub = self.create_subscription(
            OccupancyGrid, '/map', self.map_callback, map_qos
        )
        
        self.particle_sub = self.create_subscription(
            ParticleCloud, '/particle_cloud', self.particle_callback,
            QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT, durability=DurabilityPolicy.VOLATILE)
        )
        
        # **NEW: Subscribe to ground truth for logging**
        self.gt_sub = self.create_subscription(
            Odometry,
            '/ground_truth/pose',
            self.ground_truth_callback,
            10
        )
                
        # Control Loop
        self.timer = self.create_timer(self.time_delta, self.control_loop)
        
        # AMCL Service Client
        self.amcl_trigger_client = self.create_client(Empty, '/request_nomotion_update')

    def ground_truth_callback(self, msg):
        """Update ground truth position (for logging only)."""
        pos = msg.pose.pose.position
        self.ground_truth_pose = np.array([pos.x, pos.y])
        
        # **NEW: Pass to controller for logging**
        self.controller.true_position = self.ground_truth_pose

    def map_callback(self, msg):
        try:
            self.controller.set_map(msg)
            self.map_ready = True
            self.get_logger().info("Map ready.")
            self.destroy_subscription(self.map_sub)
        except Exception as e:
            self.get_logger().error(f"Map error: {e}")

    def particle_callback(self, msg: ParticleCloud):
        """Update belief with new particles."""
        self.last_particles_time = Time.from_msg(msg.header.stamp)
        points, weights = cloud_to_numpy(msg)

        if len(points) == 0 or len(points) != len(weights):
            self.get_logger().error("Invalid particle cloud")
            return
        
        self.controller.update_belief(points, weights)
        
        if not self.particles_received:
            self.get_logger().info(f"Particles received: {len(points)}")
            self.particles_received = True

        if self.waiting_for_update:
            self.waiting_for_update = False
            self.ready_to_think = True

    def control_loop(self):
        """Main decision loop."""
        if self.ticks_passed < self.ticks_to_wait:
            self.ticks_passed += 1
            return
        
        if not self.map_ready:
            self.get_logger().warn("Control loop: Waiting for map...", throttle_duration_sec=5.0)
            return

        if not self.particles_received:
            self.get_logger().warn("Control loop: Waiting for particle cloud...", throttle_duration_sec=5.0)
            return
        
        if not self.system_active:
            self.get_logger().info("🚀 System ready. Active Inference starting.")
            self.system_active = True

        if not self.ready_to_think or self.waiting_for_update:
            return
        
        try:
            best_action_name = self.controller.decide_action()
            if best_action_name:
                self.get_logger().info(f"Selected Action: {best_action_name}")
                self.apply_action(best_action_name)
        except Exception as e:
            self.get_logger().error(f"Control error: {e}")

        self.ready_to_think = False

    def apply_action(self, action_name):
        """Execute action and stop after time_delta."""
        twist_msg = Twist()
        vals = ACTION_EFFECTS.get(action_name, {'linear': 0.0, 'angular': 0.0})
        twist_msg.linear.x = vals['linear']
        twist_msg.angular.z = vals['angular']
        self.cmd_vel_pub.publish(twist_msg)

        if self._stop_timer:
            self._stop_timer.cancel()

        stop_time = max(0.1, self.time_delta - 0.1)
        self._stop_timer = Timer(stop_time, self.stop_motion)
        self._stop_timer.start()

    def stop_motion(self):
        """Stop robot and trigger AMCL update."""
        self.cmd_vel_pub.publish(Twist())
        self._logger.info("Robot stopped. Waiting for next particle cloud update.")
        # Trigger the AMCL update service
        if self.amcl_trigger_client.service_is_ready():
            self.amcl_trigger_client.call_async(Empty.Request())
            self.waiting_for_update = True
            self.get_logger().info("[STOP] Robot stopped. Forced AMCL update requested.")
        else:
            self.get_logger().warn("AMCL service down. Skipping forced update.")
            self.ready_to_think = True

def main(args=None):
    rclpy.init(args=args)
    aic_node = AICNode()
    try:
        rclpy.spin(aic_node)
    except KeyboardInterrupt:
        pass
    finally:
        aic_node.destroy_node()
        rclpy.shutdown()