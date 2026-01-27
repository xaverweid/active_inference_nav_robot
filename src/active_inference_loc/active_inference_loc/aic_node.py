import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
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
                
        self.time_delta = 1.0  # seconds

      # --- Replace the init_time lines with this ---
        self.recording_delay = 00.0  # Seconds to wait
        self.ticks_to_wait = int(self.recording_delay / self.time_delta)
        self.ticks_passed = 0

        # Also, send a "Stop" command immediately to ensure 
        # the robot isn't moving from a previous crash/run

        # 1. Initialize the "Brain"
        # We pass the logger so the core can log without being a Node
        # 1. State Tracking
        self.map_ready = False
        self.particles_received = False
        self.system_active = False
        self.latest_particles = None
        self._stop_timer = None
       
        # 2. Initialize Brain & Tools
        self.controller = ActiveInferenceController(logger=self.get_logger())
        self.metrics_pub = self.create_publisher(Float32MultiArray, '/aic_metrics', 10)
        self.controller.set_metrics_publisher(self.metrics_pub)
        self.clusturer = ParticleClusturer()

        
        # 3. ROS Infrastructure
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Map: Use Transient Local if map server is already running
       # Define a QoS that matches the Map Server (Transient Local is key here)
        map_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=1
        )

        self.map_sub = self.create_subscription(
            OccupancyGrid, 
            '/map', 
            self.map_callback, 
            map_qos  # Use the custom QoS here!
        )
                
        # Particles: AMCL uses Best Effort
        self.particle_sub = self.create_subscription(
            ParticleCloud, '/particle_cloud', self.particle_callback,
            QoSProfile(
                depth=1,
                reliability=ReliabilityPolicy.BEST_EFFORT,
                durability=DurabilityPolicy.VOLATILE,
            )
        )
        # Note: AMCL publishes /particle_cloud with BEST_EFFORT QoS,
        # so subscribers must also use best_effort to receive messages.
        # qos_profile_sensor_data provides this (best_effort/volatile).
                
        # 4. Control Loop (Only starts logic when ready)
        self.timer = self.create_timer(self.time_delta, self.control_loop)
        self.get_logger().info("AIC Node initialized. Waiting for Map and Particles...")

    def map_callback(self, msg):
        self.get_logger().info("Received Map. Processing...")
        try:
            self.controller.set_map(msg)
            self.map_ready = True
            self.get_logger().info("Map registered successfully.")
            # We keep the subscription alive for a moment to ensure stability, 
            # or destroy it if you are sure the map won't change.
            self.destroy_subscription(self.map_sub)
        except Exception as e:
            self.get_logger().error(f"Failed to set map: {e}")

    def particle_callback(self, msg: ParticleCloud):
        points = self.clusturer.cloud_to_numpy(msg)

        if len(points) == 0:
            return
        
        self.latest_particles = points
        
        # Update the controller's internal belief
        self.controller.update_belief(points)
        
        if not self.particles_received:
            self.get_logger().info(f"Particles initialized ({len(points)} particles).")
            self.particles_received = True
        
    def control_loop(self):
        # 1. Wait for the specified number of ticks
        if self.ticks_passed < self.ticks_to_wait:
            self.ticks_passed += 1
            self.get_logger().info(
                f"Recording Standby: Starting in {self.ticks_to_wait - self.ticks_passed}s...", 
                throttle_duration_sec=1.0
            )
            return
        
        if not self.map_ready:
            self.get_logger().warn("Control loop: Waiting for map...", throttle_duration_sec=5.0)
            return

        if not self.particles_received:
            self.get_logger().warn("Control loop: Waiting for particle cloud...", throttle_duration_sec=5.0)
            return
        
        if not self.system_active:
            self.get_logger().info("🚀 ALL SYSTEMS READY. Starting Active Inference Loop.")
            self.system_active = True

        
        try:
            best_action_name = self.controller.decide_action()
        
            if best_action_name:
                self.get_logger().info(f"Selected Action: {best_action_name}")
                self.apply_action(best_action_name)
        except Exception as e:
            self.get_logger().error(f"Error in Active Inference Node control loop: {e}")

    def apply_action(self, action_name):
        twist_msg = self.translate_action_to_twist(action_name)
        self.cmd_vel_pub.publish(twist_msg)

         # cancel any previous stop timer
        if self._stop_timer is not None:
            self._stop_timer.cancel()

        # Stop the robot exactly before the next "thought" cycle
        # Subtract 0.1s to allow the robot to fully stop before the next scan is taken
        stop_time = max(0.1, self.time_delta - 0.1)
        self._stop_timer = Timer(stop_time, self.stop_motion)
        self._stop_timer.start()

    def stop_motion(self):
        self.cmd_vel_pub.publish(Twist())

    def translate_action_to_twist(self, action_name):
        """Returns a Twist corresponding to applying the discrete action for TIME_DELTA as velocities."""
        t = Twist()
        vals = ACTION_EFFECTS.get(action_name, {'linear': 0.0, 'angular': 0.0})
        t.linear.x = vals['linear']
        t.angular.z = vals['angular']
        return t
    
def main(args=None):
    rclpy.init(args=args)
    aic_node = AICNode()
    try:
        rclpy.spin(aic_node)
    except KeyboardInterrupt:
        pass
    finally:
        aic_node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass