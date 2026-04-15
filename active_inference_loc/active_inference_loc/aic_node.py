from geometry_msgs import msg
import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point, PoseStamped
from nav_msgs.msg import Odometry, OccupancyGrid
from std_msgs.msg import Float32MultiArray, String
from nav2_msgs.msg import ParticleCloud
from threading import Timer
from .core import ActiveInferenceController
from .utils import ACTION_EFFECTS_long, ACTION_EFFECTS_short, cloud_to_numpy
from rclpy.time import Time
import numpy as np
from tf_transformations import euler_from_quaternion
from std_srvs.srv import Empty


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

        # Get parameters from via aic_launch.py
        self.declare_parameter('algo_mode', 'active_inf_5')
        self.algorithm_mode = self.get_parameter('algo_mode').get_parameter_value().string_value
        self.get_logger().info(f"--- LAUNCHING AIC NODE IN MODE: {self.algorithm_mode}")

        self.declare_parameter('seconds_per_step', 1)
        self.seconds_per_step = self.get_parameter('seconds_per_step').get_parameter_value().integer_value
        self.get_logger().info(f"--- with seconds per step t := {self.seconds_per_step}")

        self.time_delta = self.seconds_per_step
        self.ticks_to_wait = 5.0
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

        # Initialize Brain
        self.controller = ActiveInferenceController(logger=self.get_logger(), 
            algo_mode=self.algorithm_mode,
            seconds_per_step=self.seconds_per_step)
        
        self.actions_dict=self.controller.actions_dict
        
        # Publishers
        self.metrics_pub = self.create_publisher(Float32MultiArray, '/aic_metrics', 10)
        self.filtered_particle_pub = self.create_publisher(Float32MultiArray, '/belief/particles_filtered', 10)
        self.status_pub = self.create_publisher(String, '/trial_status', 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        self.controller.set_metrics_publisher(self.metrics_pub)
        self.controller.set_particle_publisher(self.filtered_particle_pub)
        self.controller.set_status_publisher(self.status_pub)
        
        # Positional tracking
        self.odom_pose = None   #x, y, yaw from gazebo odom (for logging only)
        self.starting_pose = None       #x, y, yaw initial spawn position
        self.starting_pose_received = False 
        self.gt_pose_xy = None  #x, y from /robot/ground_truth_pose (for metrics only)
        self.gt_rotation = None  #yaw from /robot/ground_truth_pose (for metrics only)
        
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
            QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, durability=DurabilityPolicy.VOLATILE, depth=1)
        )
        
        # Get starting pose (published by starting_pose_publisher in robot_launch.py)
        self.starting_pose_sub = self.create_subscription(
            Point,'starting_pose', self.starting_pose_callback,
            QoSProfile(reliability= ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL, depth=1)
        )
        
        # Subscribe to ground truth for metrics
        self.gz_odom_sub = self.create_subscription(
            Odometry,
            '/gazebo/odometry',
            self.odom_callback,
            10
        )

        self.gt_pose_sub = self.create_subscription(
            PoseStamped,
            '/robot/ground_truth_pose',
            self.gt_pose_callback,
            10
        )
        
        # Control Loop
        self.timer = self.create_timer(self.time_delta, self.control_loop)
        
        # AMCL Service Client
        self.amcl_trigger_client = self.create_client(Empty, '/request_nomotion_update')

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

    def starting_pose_callback(self, msg):
        if not self.starting_pose_received:
            self.starting_pose = np.array([msg.x, msg.y, msg.z])
            self.starting_pose_received = True
            self.controller.starting_pose = self.starting_pose
            self.destroy_subscription(self.starting_pose_sub)
    
    def odom_callback(self, msg):
        pos = msg.pose.pose.position
        orient = msg.pose.pose.orientation

        quaternion = [orient.x, orient.y, orient.z, orient.w]
        roll, pitch, yaw = euler_from_quaternion(quaternion)
        # Extract x, y position
        self.odom_pose = np.array([pos.x, pos.y, yaw])
                
        self.controller.odom_pose = self.odom_pose

    def gt_pose_callback(self, msg: PoseStamped):
        pos = msg.pose.position
        orient = msg.pose.orientation

        quaternion = [orient.x, orient.y, orient.z, orient.w]
        roll, pitch, yaw = euler_from_quaternion(quaternion)
        # Extract x, y position
        self.gt_pose_xy = np.array([pos.x, pos.y])
        self.gt_rotation = yaw
    
        self.controller.gt_pose_xy = self.gt_pose_xy
        self.controller.gt_rotation = self.gt_rotation

    def control_loop(self):
        """Main decision loop."""
        if self.ticks_passed < self.ticks_to_wait:
            self.ticks_passed += 1
            self.get_logger().info(f"Control loop: Countdown at {self.ticks_to_wait - self.ticks_passed}")
            return
        
        if not self.map_ready:
            self.get_logger().warn("Control loop: Waiting for map...", throttle_duration_sec=5.0)
            return

        if not self.particles_received:
            self.get_logger().warn("Control loop: Waiting for particle cloud...", throttle_duration_sec=5.0)
            return
        
        if not self.starting_pose_received:
            self.get_logger().warn("Control loop: Waiting for starting pose", throttle_duration_sec=5.0)
            return
           
        if not self.system_active:
            self.get_logger().info("🚀 All system prerequisites met. Active Inference starting.")
            self.get_logger().info(f"Starting pose is {self.starting_pose}")

            self.system_active = True

        if not self.ready_to_think or self.waiting_for_update:
            return
        
        try:
            best_action_name = self.controller.decide_action()
            if best_action_name:
                self.apply_action(best_action_name)

        except Exception as e:
            self.get_logger().error(f"Control error: {e}")

        self.ready_to_think = False

    def apply_action(self, action_name):
        """Execute action and stop after time_delta."""
        twist_msg = Twist()
        vals = self.actions_dict.get(action_name, {'linear': 0.0, 'angular': 0.0})
        twist_msg.linear.x = vals['linear']
        twist_msg.angular.z = vals['angular']
        self.cmd_vel_pub.publish(twist_msg)

        if self._stop_timer:
            self._stop_timer.cancel()

        stop_time = max(0.1, self.time_delta - 0.1)
        self.get_logger().info(f"Action '{action_name}' applied. Will stop after {stop_time:.2f} seconds. Action set: {self.actions_dict[action_name]}")
        
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