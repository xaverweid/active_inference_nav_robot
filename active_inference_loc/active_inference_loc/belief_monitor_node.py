import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Float32MultiArray
import matplotlib.pyplot as plt
from matplotlib.patches import Ellipse
import numpy as np
import threading

# Importing your existing utilities
from .utils import get_map_metadata, ParticleClusturer, get_covariance_ellipse

class BeliefMonitorNode(Node):
    def __init__(self):
        super().__init__('belief_monitor_node')

        self.clusturer = ParticleClusturer()
        self.map_metadata = None
        self.current_metrics = None  # Will be set once metrics are received
        self.latest_cloud_data = None
        self.latest_weights = None
        self.lock = threading.Lock()

        map_qos = QoSProfile(durability=DurabilityPolicy.TRANSIENT_LOCAL, depth=1)
        particle_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.map_sub = self.create_subscription(OccupancyGrid, '/map', self.map_callback, map_qos)
        self.cloud_sub = self.create_subscription(Float32MultiArray, '/belief/particles_filtered', self.cloud_callback, particle_qos)
        self.metrics_sub = self.create_subscription(Float32MultiArray, '/aic_metrics', self.metrics_callback, 10)

        plt.ion()
        self.fig, self.ax = plt.subplots(figsize=(10, 8))
        self.fig.canvas.manager.set_window_title('AIC Belief Monitor')
        
        self.dashboard = self.ax.text(0.02, 0.98, 'Waiting for data...', transform=self.ax.transAxes, 
                                      verticalalignment='top', family='monospace', fontsize=9,
                                      bbox=dict(boxstyle='round', facecolor='white', alpha=0.9, edgecolor='gray'),
                                      zorder=100)
        
        self.get_logger().info("Belief Monitor Node Started")

    def map_callback(self, msg):
        if self.map_metadata is None:
            self.map_metadata = get_map_metadata(msg)
            
            # --- ROTATE MAP DATA 270 CW (equivalent to -90 CW) ---
            rotated_map = np.rot90(self.map_metadata['data'], k=3)
            
            # --- CALCULATE NEW EXTENT (Correct for 270 CW) ---
            # Transformation: x' = -y, y' = x
            # New X range is negative old Y range: [-y_max, -y_min]
            # New Y range is old X range: [x_min, x_max]
            orig_x_min = self.map_metadata['origin_x']
            orig_x_max = orig_x_min + self.map_metadata['width'] * self.map_metadata['resolution']
            orig_y_min = self.map_metadata['origin_y']
            orig_y_max = orig_y_min + self.map_metadata['height'] * self.map_metadata['resolution']
            
            self.rotated_extent = [-orig_y_max, -orig_y_min, orig_x_min, orig_x_max]

            self.ax.imshow(rotated_map, cmap='gray', origin='lower',
                           extent=self.rotated_extent, alpha=0.6)
            self.ax.set_title("Robot Belief Monitor")
            
    def metrics_callback(self, msg):
        with self.lock:
            if len(msg.data) >= 3:
                self.current_metrics = msg.data

    def cloud_callback(self, msg):
        # 1. DO THE HEAVY LIFTING OUTSIDE THE LOCK
        # Convert the flat list from the message into a structured NumPy array
        try:
            # We reshape to -1 rows and 5 columns (x, y, cos, sin, weight)
            raw_data = np.array(msg.data).reshape(-1, 5)
            
            # Extract columns
            points = raw_data[:, :2]           # x, y
            cos_yaw = raw_data[:, 2]
            sin_yaw = raw_data[:, 3]
            weights = raw_data[:, 4]
            
            # Stack them into a format your plotter expects (x, y, cos, sin)
            processed_cloud = np.column_stack((points, cos_yaw, sin_yaw))

        except Exception as e:
            self.get_logger().error(f"Failed to reshape particle data: {e}")
            return

        # 2. UPDATE SHARED DATA INSIDE THE LOCK
        with self.lock:
            # Store both the points and weights so the main loop can use them
            self.latest_cloud_data = processed_cloud
            self.latest_weights = weights

    def listener_callback(self, msg):
        # Convert back to (N, 4) in one line
        data = np.array(msg.data).reshape(-1, 4)
        particles = data[:, :3] # x, y, yaw
        weights = data[:, 3]    # weights
    
        self.plotter.update(particles, weights)
        
    def update_plot(self):
        with self.lock:
            if self.current_metrics is None or self.latest_cloud_data is None or self.latest_weights is None:
                return
            
            raw_points = self.latest_cloud_data.copy()
            weights = self.latest_weights.copy()
            metrics = list(self.current_metrics)

            self.latest_cloud_data = None
            self.latest_weights = None

        try:
            # 1. ROTATE PARTICLE DATA 270 CW (equivalent to -90 CW) for synchronization with the map
            rotated_points = np.zeros_like(raw_points)
            # New X = -Old Y
            # New Y = Old X
            rotated_points[:, 0] = -raw_points[:, 1]  
            rotated_points[:, 1] = raw_points[:, 0]   
            # New Cos = -Old Sin
            # New Sin = Old Cos
            rotated_points[:, 2] = -raw_points[:, 3]
            rotated_points[:, 3] = raw_points[:, 2]

            # Now that points are rotated, we cluster them
            cluster_poses, cluster_weights = self.clusturer.get_representative_clusters_from_gmm(rotated_points, weights)
            
            # 2. RENDERING LOGIC
            for artist in list(self.ax.collections) + list(self.ax.patches):
                artist.remove()
            for txt in list(self.ax.texts):
                if txt != self.dashboard: txt.remove()

            # Plot Rotated Particles
            self.ax.scatter(rotated_points[:, 0], rotated_points[:, 1], s=1, c='blue', alpha=0.1)
            # Assign labels for the rotated clusters
            # Use the GMM to predict labels for the particles directly

            labels = self.clusturer.gmm.predict(rotated_points)
            # Normalization for colors (0 to max weight)
            # This ensures the "best" cluster is always greenest
            norm = plt.Normalize(vmin=0, vmax=np.max(cluster_weights))
            cmap = plt.cm.RdYlGn
            
            for i, (mx, my, ctheta, stheta) in enumerate(cluster_poses):
                weight = cluster_weights[i]
                cluster_mask = (labels == i)
                if np.sum(cluster_mask) < 3: continue
                
                # Determine Color based on Weight
                # High weight = Green, Low weight = Red
                color = cmap(norm(weight))
                # Get ellipse for the rotated cluster
                res = get_covariance_ellipse(rotated_points[cluster_mask, :2])
                if res:
                    width, height, angle = res
                    # Draw Belief Ellipse
                    ell = Ellipse(xy=(mx, my), width=width, height=height, angle=angle, 
                                edgecolor=color, fc=color, lw=2, alpha=0.4, zorder=3)
                    self.ax.add_patch(ell)
                    
                    # Draw Heading Arrow (Red stays red for visibility)
                    self.ax.arrow(mx, my, 0.4*ctheta, 0.4*stheta, color='red', 
                                head_width=0.1, zorder=5, alpha=min(1.0, weight*5))
                
            # Dashboard Update

            shannon_h = f"{metrics[9]:.2f}" if len(metrics) > 9 else 'N/A'
            spatial_entropy = f"{metrics[10]:.2f}" if len(metrics) > 10 else 'N/A'
            pos_error = f"{metrics[7]:.3f}" if len(metrics) > 7 else 'N/A'
            rot_error = f"{metrics[8]:.3f}" if len(metrics) > 8 else 'N/A'
            action = metrics[11] if len(metrics) > 11 else 'N/A'
            table_text = (
                f"▼ PARAMETERS (fixed)\n"
                f"alpha (epistemic):  {metrics[3]:.2f}\n"
                f"beta  (pragmatic):  {metrics[4]:.2f}\n"
                f"-------------------------------\n"
                f"▼ BELIEF METRICS on all P\n"
                f"Shannon H:    {shannon_h}\n"
                f"Spatial H:    {spatial_entropy}\n"
                f"Convergence:  {metrics[6]:.2f}\n"
                f"-------------------------------\n"
                f"▼ AIC POLICY (G)\n"
                f"Expected Epistemic: {metrics[0]:.2f}\n"
                f"Expected Pragmatic: {metrics[1]:.2f}\n"
                f"Total Expected G:   {metrics[2]:.2f}\n"
                f"-------------------------------\n"
                f"Runtime:            {int(metrics[5]):.2f}\n"
                f"Position Error:     {pos_error}\n"
                f"Rotational Error:   {rot_error}\n"
                f"Selected Action:    {action}\n"
            )
            self.dashboard.set_text(table_text)

            self.fig.canvas.draw()
            self.fig.canvas.flush_events()

        except Exception as e:
            self.get_logger().error(f"Visualization Error: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = BeliefMonitorNode()
    spin_thread = threading.Thread(target=lambda: rclpy.spin(node), daemon=True)
    spin_thread.start()

    try:
        while rclpy.ok():
            node.update_plot()
            plt.pause(0.1)
    except KeyboardInterrupt:
        pass
    finally:
        plt.close('all')
        node.destroy_node()
        rclpy.shutdown()